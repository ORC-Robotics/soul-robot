"""Detecta bolas pretas e pratas sem redes neurais."""

from dataclasses import dataclass
import math

import cv2  # type: ignore
import numpy as np


@dataclass(frozen=True)
class BallDetectorConfig:
    """Centraliza limites ajustáveis do detector de bolas pretas."""

    maximum_value: int = 100
    blur_kernel_size: int = 5
    open_kernel_size: int = 5
    close_kernel_size: int = 11
    minimum_area_pixels: float = 200.0
    maximum_area_ratio: float = 0.70
    minimum_radius_pixels: float = 5.0
    minimum_circularity: float = 0.72
    minimum_circle_fill_ratio: float = 0.68
    border_tolerance_pixels: int = 3
    minimum_top_clipped_circularity: float = 0.40
    minimum_top_clipped_fill_ratio: float = 0.35
    minimum_top_clipped_aspect_ratio: float = 0.75
    maximum_top_clipped_aspect_ratio: float = 3.50

    def validate(self):
        """Rejeita configurações que produziriam máscaras ambíguas."""

        kernel_sizes = (
            self.blur_kernel_size,
            self.open_kernel_size,
            self.close_kernel_size,
        )
        if any(size <= 0 or size % 2 == 0 for size in kernel_sizes):
            raise ValueError("Os kernels devem ser inteiros ímpares e positivos.")
        if not 0 <= self.maximum_value <= 255:
            raise ValueError("maximum_value deve estar entre 0 e 255.")
        if self.minimum_area_pixels <= 0.0:
            raise ValueError("minimum_area_pixels deve ser positivo.")
        if not 0.0 < self.maximum_area_ratio <= 1.0:
            raise ValueError("maximum_area_ratio deve estar entre 0 e 1.")
        if self.minimum_radius_pixels <= 0.0:
            raise ValueError("minimum_radius_pixels deve ser positivo.")
        if not 0.0 < self.minimum_circularity <= 1.0:
            raise ValueError("minimum_circularity deve estar entre 0 e 1.")
        if not 0.0 < self.minimum_circle_fill_ratio <= 1.0:
            raise ValueError("minimum_circle_fill_ratio deve estar entre 0 e 1.")
        if self.border_tolerance_pixels < 0:
            raise ValueError("border_tolerance_pixels não pode ser negativo.")
        clipped_ratios = (
            self.minimum_top_clipped_circularity,
            self.minimum_top_clipped_fill_ratio,
        )
        if any(not 0.0 < ratio <= 1.0 for ratio in clipped_ratios):
            raise ValueError("Os limites do recorte superior devem estar entre 0 e 1.")
        if not 0.0 < self.minimum_top_clipped_aspect_ratio:
            raise ValueError("A proporção mínima do recorte deve ser positiva.")
        if self.maximum_top_clipped_aspect_ratio <= (
            self.minimum_top_clipped_aspect_ratio
        ):
            raise ValueError("A faixa de proporção do recorte deve ser válida.")


@dataclass(frozen=True)
class BallCandidate:
    """Descreve a geometria comum a uma vítima preta ou prateada."""

    ball_type: str
    center_x: float
    center_y: float
    radius_pixels: float
    diameter_pixels: float
    contour_area_pixels: float
    circularity: float
    circle_fill_ratio: float
    top_clipped: bool
    detection_method: str
    visible_area_pixels: float = 0.0


class BlackBallDetector:
    """Encontra componentes escuros com geometria aproximadamente circular."""

    def __init__(self, config=None):
        self.config = config or BallDetectorConfig()
        self.config.validate()
        self._open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.open_kernel_size, self.config.open_kernel_size),
        )
        self._close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.close_kernel_size, self.config.close_kernel_size),
        )

    def create_mask(self, frame):
        """Cria uma máscara binária usando o brilho V do espaço HSV."""

        if not isinstance(frame, np.ndarray) or frame.ndim != 3:
            raise ValueError("O frame deve ser uma imagem BGR com três canais.")
        if frame.shape[2] != 3 or frame.size == 0:
            raise ValueError("O frame deve ser uma imagem BGR não vazia.")

        blurred = cv2.GaussianBlur(
            frame,
            (self.config.blur_kernel_size, self.config.blur_kernel_size),
            0,
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (0, 0, 0),
            (179, 255, self.config.maximum_value),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._open_kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_kernel)

    def detect(self, frame):
        """Retorna candidatos ordenados pela área realmente visível no frame."""

        mask = self.create_mask(frame)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        maximum_area = float(frame.shape[0] * frame.shape[1]) * (
            self.config.maximum_area_ratio
        )
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.minimum_area_pixels or area > maximum_area:
                continue

            # O casco convexo remove pequenas reentrâncias criadas pelas dobras
            # da fita sem preencher a área preta real usada na validação abaixo.
            convex_hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(convex_hull))
            perimeter = float(cv2.arcLength(convex_hull, True))
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * math.pi * hull_area / (perimeter * perimeter)

            bounding_x, bounding_y, bounding_width, bounding_height = (
                cv2.boundingRect(contour)
            )
            del bounding_x
            top_clipped = bounding_y <= self.config.border_tolerance_pixels
            aspect_ratio = (
                float(bounding_width) / float(bounding_height)
                if bounding_height > 0 else math.inf
            )
            if top_clipped:
                # O contorno da bola pode ser fechado pela borda superior da
                # imagem. Nesse caso, a reta artificial reduz a circularidade.
                if circularity < self.config.minimum_top_clipped_circularity:
                    continue
                if not (
                    self.config.minimum_top_clipped_aspect_ratio <= aspect_ratio <=
                    self.config.maximum_top_clipped_aspect_ratio
                ):
                    continue
            elif circularity < self.config.minimum_circularity:
                continue

            (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
            radius = float(radius)
            if radius < self.config.minimum_radius_pixels:
                continue
            enclosing_area = math.pi * radius * radius
            fill_ratio = area / enclosing_area if enclosing_area > 0.0 else 0.0
            minimum_fill_ratio = (
                self.config.minimum_top_clipped_fill_ratio
                if top_clipped else self.config.minimum_circle_fill_ratio
            )
            if fill_ratio < minimum_fill_ratio:
                continue

            candidates.append(BallCandidate(
                ball_type="black_ball",
                center_x=float(center_x),
                center_y=float(center_y),
                radius_pixels=radius,
                diameter_pixels=radius * 2.0,
                contour_area_pixels=area,
                circularity=float(circularity),
                circle_fill_ratio=float(fill_ratio),
                top_clipped=top_clipped,
                detection_method="contour",
                visible_area_pixels=area,
            ))

        candidates.sort(
            key=lambda candidate: candidate.visible_area_pixels,
            reverse=True,
        )
        return candidates


@dataclass(frozen=True)
class SilverBallDetectorConfig:
    """Centraliza os limites do detector de círculos claros e texturizados."""

    processing_scale: float = 0.50
    minimum_radius_pixels: int = 18
    maximum_radius_ratio: float = 0.49
    hough_dp: float = 1.2
    hough_minimum_distance_pixels: float = 55.0
    hough_canny_threshold: float = 90.0
    hough_accumulator_threshold: float = 28.0
    minimum_mean_value: float = 95.0
    maximum_mean_value: float = 230.0
    minimum_intensity_stddev: float = 8.0
    minimum_edge_density: float = 0.012
    maximum_edge_density: float = 0.40
    minimum_boundary_coverage: float = 0.50
    border_tolerance_pixels: int = 3

    def validate(self):
        if not 0.0 < self.processing_scale <= 1.0:
            raise ValueError("processing_scale deve estar entre 0 e 1.")
        if self.minimum_radius_pixels <= 0:
            raise ValueError("O raio mínimo da bola prata deve ser positivo.")
        if not 0.0 < self.maximum_radius_ratio <= 0.5:
            raise ValueError("maximum_radius_ratio deve estar entre 0 e 0,5.")
        if self.hough_dp <= 0.0 or self.hough_minimum_distance_pixels <= 0.0:
            raise ValueError("Os parâmetros geométricos de Hough devem ser positivos.")
        if self.hough_canny_threshold <= 0.0 or self.hough_accumulator_threshold <= 0.0:
            raise ValueError("Os limiares de Hough devem ser positivos.")
        if not 0.0 <= self.minimum_mean_value < self.maximum_mean_value <= 255.0:
            raise ValueError("A faixa de brilho da bola prata deve ser válida.")
        if self.minimum_intensity_stddev < 0.0:
            raise ValueError("A variação mínima de intensidade não pode ser negativa.")
        if not 0.0 <= self.minimum_edge_density < self.maximum_edge_density <= 1.0:
            raise ValueError("A faixa de densidade de bordas deve ser válida.")
        if not 0.0 < self.minimum_boundary_coverage <= 1.0:
            raise ValueError("A cobertura mínima da circunferência deve ser válida.")


class SilverBallDetector:
    """Detecta a bola prata pelo círculo e pela textura da folha metálica."""

    def __init__(self, config=None):
        self.config = config or SilverBallDetectorConfig()
        self.config.validate()
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    @staticmethod
    def _circle_region(shape, center_x, center_y, radius):
        mask = np.zeros(shape, dtype=np.uint8)
        cv2.circle(
            mask,
            (int(round(center_x)), int(round(center_y))),
            int(round(radius * 0.88)),
            255,
            -1,
        )
        return mask

    @staticmethod
    def _visible_circle_pixels(shape, center_x, center_y, radius):
        """Conta somente a parte do círculo que está dentro do frame."""

        mask = np.zeros(shape, dtype=np.uint8)
        cv2.circle(
            mask,
            (int(round(center_x)), int(round(center_y))),
            int(round(radius)),
            255,
            -1,
        )
        return int(cv2.countNonZero(mask))

    @staticmethod
    def _boundary_coverage(edges, center_x, center_y, radius):
        """Mede quantos trechos visíveis da circunferência possuem uma borda."""

        sample_count = 72
        search_radius = max(3, int(round(radius * 0.04)))
        visible_samples = 0
        samples_with_edge = 0
        height, width = edges.shape
        for sample_index in range(sample_count):
            angle = 2.0 * math.pi * sample_index / sample_count
            sample_x = int(round(center_x + math.cos(angle) * radius))
            sample_y = int(round(center_y + math.sin(angle) * radius))
            if not (0 <= sample_x < width and 0 <= sample_y < height):
                continue
            visible_samples += 1
            left = max(0, sample_x - search_radius)
            right = min(width, sample_x + search_radius + 1)
            top = max(0, sample_y - search_radius)
            bottom = min(height, sample_y + search_radius + 1)
            if cv2.countNonZero(edges[top:bottom, left:right]) > 0:
                samples_with_edge += 1
        if visible_samples == 0:
            return 0.0
        return float(samples_with_edge) / float(visible_samples)

    def detect(self, frame):
        """Retorna círculos prateados na ordem de confiança do Hough."""

        if not isinstance(frame, np.ndarray) or frame.ndim != 3:
            raise ValueError("O frame deve ser uma imagem BGR com três canais.")
        if frame.shape[2] != 3 or frame.size == 0:
            raise ValueError("O frame deve ser uma imagem BGR não vazia.")

        scale = self.config.processing_scale
        if scale < 1.0:
            processing_frame = cv2.resize(
                frame,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            processing_frame = frame

        gray = cv2.cvtColor(processing_frame, cv2.COLOR_BGR2GRAY)
        enhanced = self._clahe.apply(gray)
        blurred = cv2.GaussianBlur(enhanced, (9, 9), 1.6)
        edges = cv2.Canny(
            blurred,
            self.config.hough_canny_threshold * 0.5,
            self.config.hough_canny_threshold,
        )
        maximum_radius = int(
            min(processing_frame.shape[:2]) * self.config.maximum_radius_ratio
        )
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self.config.hough_dp,
            minDist=self.config.hough_minimum_distance_pixels * scale,
            param1=self.config.hough_canny_threshold,
            param2=max(1.0, self.config.hough_accumulator_threshold * scale),
            minRadius=max(2, int(round(self.config.minimum_radius_pixels * scale))),
            maxRadius=maximum_radius,
        )
        if circles is None:
            return []

        hsv = cv2.cvtColor(processing_frame, cv2.COLOR_BGR2HSV)
        value_channel = hsv[:, :, 2]
        candidates = []
        for center_x, center_y, radius in circles[0]:
            center_x = float(center_x)
            center_y = float(center_y)
            radius = float(radius)
            region = self._circle_region(
                gray.shape,
                center_x,
                center_y,
                radius,
            )
            validation_pixels = int(cv2.countNonZero(region))
            if validation_pixels <= 0:
                continue

            mean_value, intensity_stddev = cv2.meanStdDev(
                value_channel,
                mask=region,
            )
            mean_value = float(mean_value[0, 0])
            intensity_stddev = float(intensity_stddev[0, 0])
            edge_pixels = int(cv2.countNonZero(cv2.bitwise_and(edges, region)))
            edge_density = float(edge_pixels) / float(validation_pixels)
            boundary_coverage = self._boundary_coverage(
                edges,
                center_x,
                center_y,
                radius,
            )
            if not (
                self.config.minimum_mean_value <= mean_value <=
                self.config.maximum_mean_value
            ):
                continue
            if intensity_stddev < self.config.minimum_intensity_stddev:
                continue
            if not (
                self.config.minimum_edge_density <= edge_density <=
                self.config.maximum_edge_density
            ):
                continue
            # Um círculo Hough isolado não basta: a borda precisa acompanhar
            # a circunferência para rejeitar arcos formados pelas rugas ou piso.
            if boundary_coverage < self.config.minimum_boundary_coverage:
                continue

            top_clipped = (
                center_y - radius <= self.config.border_tolerance_pixels * scale
            )
            output_scale = 1.0 / scale
            output_center_x = center_x * output_scale
            output_center_y = center_y * output_scale
            output_radius = radius * output_scale
            visible_circle_pixels = self._visible_circle_pixels(
                gray.shape,
                center_x,
                center_y,
                radius,
            )
            candidates.append(BallCandidate(
                ball_type="silver_ball",
                center_x=output_center_x,
                center_y=output_center_y,
                radius_pixels=output_radius,
                diameter_pixels=output_radius * 2.0,
                contour_area_pixels=math.pi * output_radius * output_radius,
                circularity=boundary_coverage,
                circle_fill_ratio=edge_density,
                top_clipped=top_clipped,
                detection_method="hough",
                visible_area_pixels=(
                    float(visible_circle_pixels) * output_scale * output_scale
                ),
            ))

        # Mantém apenas uma hipótese por bola. Depois da remoção de duplicatas,
        # a área do círculo que realmente cabe no frame define a ordenação.
        unique_candidates = []
        for candidate in candidates:
            overlaps_existing = any(
                math.hypot(
                    candidate.center_x - existing.center_x,
                    candidate.center_y - existing.center_y,
                ) / max(candidate.radius_pixels, existing.radius_pixels) <= 0.85
                for existing in unique_candidates
            )
            if not overlaps_existing:
                unique_candidates.append(candidate)
        unique_candidates.sort(
            key=lambda candidate: candidate.visible_area_pixels,
            reverse=True,
        )
        return unique_candidates


class BallDetector:
    """Combina bolas pretas e pratas sem duplicar o mesmo alvo geométrico."""

    def __init__(self, black_detector=None, silver_detector=None):
        self.black_detector = black_detector or BlackBallDetector()
        self.silver_detector = silver_detector or SilverBallDetector()

    @staticmethod
    def _overlap_ratio(first, second):
        center_distance = math.hypot(
            first.center_x - second.center_x,
            first.center_y - second.center_y,
        )
        return center_distance / max(first.radius_pixels, second.radius_pixels)

    def detect(self, frame):
        """Prioriza preto em sobreposições e preserva bolas distintas."""

        black_candidates = self.black_detector.detect(frame)
        silver_candidates = self.silver_detector.detect(frame)
        unique_silver = [
            candidate
            for candidate in silver_candidates
            if all(
                self._overlap_ratio(candidate, black_candidate) > 0.65
                for black_candidate in black_candidates
            )
        ]
        if not black_candidates:
            return unique_silver
        if not unique_silver:
            return black_candidates

        # Cada detector já ordenou seu melhor alvo. Entre os dois tipos, a área
        # visível permite comparar as candidatas pelo mesmo critério.
        candidates = black_candidates + unique_silver
        candidates.sort(
            key=lambda candidate: candidate.visible_area_pixels,
            reverse=True,
        )
        return candidates
