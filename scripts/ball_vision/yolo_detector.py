"""Executa o detector YOLO de vítimas com ONNX Runtime."""

from dataclasses import dataclass
import logging
import math
from pathlib import Path

import cv2  # type: ignore
import numpy as np

try:
    import onnxruntime as ort  # type: ignore
except ImportError:
    ort = None

try:
    from .ball_detector import BallCandidate
except ImportError:
    from ball_detector import BallCandidate


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class YoloBallDetectorConfig:
    """Centraliza os limites de inferência do modelo de vítimas."""

    model_path: Path = Path(__file__).resolve().parents[2] / "assets" / "models" / "ball_detector.onnx"
    # A Raspberry Pi executa inferência no CPU. O perfil 384 preservou o mAP50
    # do conjunto de validação e reduziu o custo em relação ao perfil 416.
    input_size: int = 384
    confidence_threshold: float = 0.55
    nms_iou_threshold: float = 0.45

    def validate(self):
        """Impede uma configuração inválida antes da primeira inferência."""

        if self.input_size <= 0:
            raise ValueError("input_size deve ser positivo.")
        if not 0.0 < self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold deve estar entre 0 e 1.")
        if not 0.0 < self.nms_iou_threshold <= 1.0:
            raise ValueError("nms_iou_threshold deve estar entre 0 e 1.")


class YoloBallDetector:
    """Converte as caixas do YOLO em candidatos compatíveis com o rastreador."""

    _CLASS_NAMES = ("black_ball", "silver_ball")

    def __init__(self, config=None):
        self.config = config or YoloBallDetectorConfig()
        self.config.validate()
        if not self.config.model_path.is_file():
            raise FileNotFoundError(
                "Modelo YOLO não encontrado em "
                f"{self.config.model_path}. Execute yolo_ball/export.py e copie o ONNX."
            )
        if ort is None:
            raise RuntimeError(
                "onnxruntime não está instalado. O deploy deve instalar "
                "a dependência antes de iniciar a câmera frontal."
            )
        # O OpenCV 4.6 da Raspberry não executa corretamente a arquitetura
        # YOLO11. O ONNX Runtime executa o mesmo arquivo no CPU sem alterar
        # as caixas, a confiança ou a lógica de controle.
        self._session = ort.InferenceSession(
            str(self.config.model_path),
            providers=["CPUExecutionProvider"],
        )
        model_input = self._session.get_inputs()[0]
        expected_shape = [1, 3, self.config.input_size, self.config.input_size]
        if model_input.shape != expected_shape:
            raise RuntimeError(
                "O tamanho configurado para o YOLO não corresponde ao modelo: "
                f"configuração={expected_shape}, modelo={model_input.shape}."
            )
        self._input_name = model_input.name
        LOGGER.info("YOLO victim detector loaded: %s", self.config.model_path)

    @property
    def processing_scale(self):
        """Mantém a telemetria compatível sem redimensionamento parcial."""

        return 1.0

    def _prepare_input(self, frame):
        """Aplica letterbox para preservar a geometria usada no cálculo de distância."""

        height, width = frame.shape[:2]
        scale = min(self.config.input_size / width, self.config.input_size / height)
        resized_width = round(width * scale)
        resized_height = round(height * scale)
        resized = cv2.resize(frame, (resized_width, resized_height))
        padding_x = (self.config.input_size - resized_width) // 2
        padding_y = (self.config.input_size - resized_height) // 2
        letterboxed = cv2.copyMakeBorder(
            resized,
            padding_y,
            self.config.input_size - resized_height - padding_y,
            padding_x,
            self.config.input_size - resized_width - padding_x,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        blob = cv2.dnn.blobFromImage(
            letterboxed,
            scalefactor=1.0 / 255.0,
            size=(self.config.input_size, self.config.input_size),
            swapRB=True,
        )
        return blob, scale, padding_x, padding_y

    def detect(self, frame):
        """Retorna caixas confiáveis, priorizando prata antes de preta na aquisição."""

        if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.size == 0:
            raise ValueError("O frame deve ser uma imagem BGR não vazia.")

        blob, scale, padding_x, padding_y = self._prepare_input(frame)
        raw_output = self._session.run(
            None,
            {self._input_name: blob},
        )[0]
        predictions = np.squeeze(raw_output, axis=0).T
        height, width = frame.shape[:2]
        # Filtra e converte todas as previsões em um único bloco NumPy. Evitar
        # milhares de iterações Python reduz o atraso sem mudar score ou NMS.
        class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(len(predictions)), class_ids]
        accepted = confidences >= self.config.confidence_threshold
        accepted &= class_ids < len(self._CLASS_NAMES)
        predictions = predictions[accepted]
        class_ids = class_ids[accepted]
        confidences = confidences[accepted]

        if len(predictions) == 0:
            return []

        box_values = predictions[:, :4].astype(np.float32, copy=True)
        left = (box_values[:, 0] - box_values[:, 2] * 0.5 - padding_x) / scale
        top = (box_values[:, 1] - box_values[:, 3] * 0.5 - padding_y) / scale
        box_widths = box_values[:, 2] / scale
        box_heights = box_values[:, 3] / scale
        left = np.maximum(0.0, left)
        top = np.maximum(0.0, top)
        box_widths = np.minimum(box_widths, width - left)
        box_heights = np.minimum(box_heights, height - top)
        valid_boxes = (box_widths > 0.0) & (box_heights > 0.0)
        boxes = np.rint(np.column_stack(
            (left, top, box_widths, box_heights)
        )[valid_boxes]).astype(np.int32).tolist()
        class_ids = class_ids[valid_boxes]
        confidences = confidences[valid_boxes]

        if not boxes:
            return []

        kept_indices = cv2.dnn.NMSBoxes(
            boxes,
            confidences.tolist(),
            self.config.confidence_threshold,
            self.config.nms_iou_threshold,
        )
        candidates = []
        for index in np.asarray(kept_indices).reshape(-1):
            left, top, box_width, box_height = boxes[int(index)]
            radius = (box_width + box_height) * 0.25
            if radius <= 0.0 or class_ids[int(index)] >= len(self._CLASS_NAMES):
                continue
            confidence = float(confidences[int(index)])
            candidates.append(BallCandidate(
                ball_type=self._CLASS_NAMES[class_ids[int(index)]],
                center_x=left + box_width * 0.5,
                center_y=top + box_height * 0.5,
                radius_pixels=radius,
                diameter_pixels=radius * 2.0,
                contour_area_pixels=float(box_width * box_height),
                circularity=confidence,
                circle_fill_ratio=confidence,
                top_clipped=top <= 0,
                detection_method="yolo",
                visible_area_pixels=float(box_width * box_height),
                bounding_box=(
                    float(left),
                    float(top),
                    float(box_width),
                    float(box_height),
                ),
            ))

        # A prioridade vale somente na escolha de um novo alvo. Um alvo travado
        # continua sujeito à confirmação temporal do BallTracker.
        candidates.sort(
            key=lambda candidate: (
                candidate.ball_type != "silver_ball",
                -candidate.circle_fill_ratio,
            )
        )
        return candidates
