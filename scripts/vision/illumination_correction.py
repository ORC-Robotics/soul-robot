"""Compensa o padrão fixo de iluminação da câmera inferior."""

import hashlib
import math
import time
from functools import lru_cache
from pathlib import Path

import cv2  # type: ignore
import numpy as np


def empty_line_illumination_status(vision_profile=None):
    """Cria telemetria neutra para a compensação de iluminação da pista."""

    profile = vision_profile if isinstance(vision_profile, dict) else {}
    return {
        "illuminationCorrectionConfigured": bool(
            profile.get("line_illumination_correction_enabled", False)
        ),
        "illuminationCorrectionActive": False,
        "illuminationCorrectionError": "",
        "illuminationReferenceSha256": str(
            profile.get("line_illumination_reference_sha256", "")
        ),
        "illuminationMaximumGain": 1.0,
        "illuminationTargetGray": 0.0,
        "illuminationCorrectionZonePercent": 0.0,
        "illuminationCorrectionMs": 0.0,
        "illuminationDarkPixelsPreserved": 0,
    }


def build_smoothed_line_illumination_reference(gray_frames, sigma_px):
    """Resume o piso branco e reduz detalhes transitórios ou muito pequenos."""

    frames = [np.asarray(frame) for frame in gray_frames]
    if not frames:
        raise ValueError("A referência de iluminação exige ao menos um frame.")
    shape = frames[0].shape
    if len(shape) != 2 or any(frame.shape != shape for frame in frames):
        raise ValueError("Todos os frames da referência devem ter a mesma forma 2D.")
    if not math.isfinite(float(sigma_px)) or float(sigma_px) <= 0.0:
        raise ValueError("O sigma da referência deve ser positivo.")

    median = np.median(np.stack(frames, axis=0), axis=0).astype(np.float32)
    smoothed = cv2.GaussianBlur(
        median,
        (0, 0),
        sigmaX=float(sigma_px),
        sigmaY=float(sigma_px),
        borderType=cv2.BORDER_REFLECT101,
    )
    return np.clip(np.rint(smoothed), 1, 255).astype(np.uint8)


def _line_illumination_reference_path(configured_path):
    """Resolve artefatos relativos a partir do diretório do módulo de visão."""

    path = Path(str(configured_path))
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    return path


@lru_cache(maxsize=8)
def _cached_line_illumination_data(
    configured_path,
    expected_sha256,
    frame_height,
    frame_width,
    target_percentile,
    maximum_gain,
    useful_start_ratio,
    zone_gain_threshold,
    overlay_margin_px,
):
    """Carrega e valida uma vez o mapa fixo usado pela câmera inferior."""

    status = empty_line_illumination_status({
        "line_illumination_correction_enabled": True,
        "line_illumination_reference_sha256": expected_sha256,
    })
    try:
        reference_path = _line_illumination_reference_path(configured_path)
        if not reference_path.is_file():
            raise FileNotFoundError(
                f"referência ausente: {reference_path}"
            )
        reference_bytes = reference_path.read_bytes()
        actual_sha256 = hashlib.sha256(reference_bytes).hexdigest()
        if not expected_sha256 or actual_sha256 != expected_sha256:
            raise ValueError(
                "hash da referência de iluminação não confere"
            )
        reference = cv2.imdecode(
            np.frombuffer(reference_bytes, dtype=np.uint8),
            cv2.IMREAD_GRAYSCALE,
        )
        expected_shape = (int(frame_height), int(frame_width))
        if reference is None or reference.shape != expected_shape:
            actual_shape = None if reference is None else reference.shape
            raise ValueError(
                f"referência com forma {actual_shape}; esperado {expected_shape}"
            )
        percentile = float(target_percentile)
        gain_limit = float(maximum_gain)
        start_ratio = float(useful_start_ratio)
        zone_threshold = float(zone_gain_threshold)
        margin = int(overlay_margin_px)
        if not 0.0 < percentile <= 100.0:
            raise ValueError("percentil da referência fora de (0, 100]")
        if not 1.0 <= gain_limit <= 4.0:
            raise ValueError("ganho máximo da referência fora de [1, 4]")
        if not 0.0 <= start_ratio < 1.0:
            raise ValueError("início da região útil fora de [0, 1)")
        if not 1.0 <= zone_threshold <= gain_limit:
            raise ValueError("limiar visual do ganho fora da faixa configurada")
        if not 0 <= margin <= 64:
            raise ValueError("margem visual da referência fora de [0, 64]")

        useful_start_y = int(round(frame_height * start_ratio))
        useful_reference = reference[useful_start_y:, :]
        if useful_reference.size == 0:
            raise ValueError("região útil vazia na referência de iluminação")
        target_gray = float(np.percentile(useful_reference, percentile))
        gain = np.clip(
            target_gray / np.maximum(reference.astype(np.float32), 1.0),
            1.0,
            gain_limit,
        ).astype(np.float32)
        zone = np.zeros(expected_shape, dtype=np.uint8)
        zone[gain >= zone_threshold] = 255
        zone[:useful_start_y, :] = 0
        if margin > 0:
            kernel_size = margin * 2 + 1
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (kernel_size, kernel_size),
            )
            zone = cv2.dilate(zone, kernel)
            zone[:useful_start_y, :] = 0

        status.update({
            "illuminationCorrectionActive": True,
            "illuminationReferenceSha256": actual_sha256,
            "illuminationMaximumGain": float(np.max(gain)),
            "illuminationTargetGray": target_gray,
            "illuminationCorrectionZonePercent": (
                100.0 * float(np.count_nonzero(zone[useful_start_y:, :]))
                / float(zone[useful_start_y:, :].size)
            ),
        })
        gain.setflags(write=False)
        zone.setflags(write=False)
        return gain, zone, status
    except (OSError, TypeError, ValueError) as error:
        status["illuminationCorrectionError"] = str(error)
        print(
            "Compensação de iluminação da linha desativada: "
            f"{error}.",
            flush=True,
        )
        return None, None, status


def line_illumination_data(vision_profile, frame_shape):
    """Obtém ganho, zona de overlay e status sem alterar pixels do frame."""

    status = empty_line_illumination_status(vision_profile)
    if not status["illuminationCorrectionConfigured"]:
        return None, None, status
    frame_height, frame_width = frame_shape[:2]
    gain, zone, cached_status = _cached_line_illumination_data(
        str(vision_profile.get("line_illumination_reference_path", "")),
        str(vision_profile.get("line_illumination_reference_sha256", "")),
        int(frame_height),
        int(frame_width),
        float(vision_profile.get("line_illumination_target_percentile", 85.0)),
        float(vision_profile.get("line_illumination_max_gain", 2.0)),
        float(vision_profile.get("line_illumination_useful_start_ratio", 0.0)),
        float(vision_profile.get("line_illumination_overlay_gain_threshold", 1.10)),
        int(vision_profile.get("line_illumination_overlay_margin_px", 8)),
    )
    return gain, zone, dict(cached_status)


def apply_line_illumination_correction(
    gray_roi,
    vision_profile,
    frame_shape,
    roi_start_y=0,
):
    """Compensa a sombra fixa somente no cinza entregue à segmentação preta."""

    started = time.perf_counter()
    gain, _zone, cached_status = line_illumination_data(
        vision_profile,
        frame_shape,
    )
    status = dict(cached_status)
    if gain is None or not status["illuminationCorrectionActive"]:
        status["illuminationCorrectionMs"] = (
            time.perf_counter() - started
        ) * 1000.0
        return gray_roi, status

    roi_end_y = int(roi_start_y) + gray_roi.shape[0]
    gain_roi = gain[int(roi_start_y):roi_end_y, :gray_roi.shape[1]]
    if gain_roi.shape != gray_roi.shape:
        status["illuminationCorrectionActive"] = False
        status["illuminationCorrectionError"] = (
            "ROI incompatível com a referência de iluminação"
        )
        status["illuminationCorrectionMs"] = (
            time.perf_counter() - started
        ) * 1000.0
        return gray_roi, status

    corrected = cv2.multiply(
        gray_roi,
        gain_roi,
        dtype=cv2.CV_8U,
    )
    status["illuminationCorrectionMs"] = (
        time.perf_counter() - started
    ) * 1000.0
    return corrected, status
