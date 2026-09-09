"""Renderiza o efeito da compensação sem executar controle ou motores."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import cv2  # type: ignore
import numpy as np

from line_calibration import segment
from vision.camera_config import CAMERA_PROFILES
from vision.illumination_correction import line_illumination_data


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent


def portable_path(path):
    """Evita gravar o diretório pessoal do operador no diagnóstico."""

    try:
        return path.resolve().relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(path.resolve())


def titled(frame, title):
    """Adiciona a legenda fora dos pixels usados na análise."""

    panel = cv2.copyMakeBorder(
        frame,
        34,
        0,
        0,
        0,
        cv2.BORDER_CONSTANT,
        value=(245, 245, 245),
    )
    cv2.putText(
        panel,
        title,
        (9, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (20, 20, 20),
        1,
        cv2.LINE_AA,
    )
    return panel


def render(dataset, sample_id, output):
    """Compara a configuração 61/40 com e sem o mesmo mapa de iluminação."""

    raw_path = dataset / "raw" / f"{sample_id}.png"
    raw = cv2.imread(str(raw_path), cv2.IMREAD_COLOR)
    if raw is None:
        raise ValueError(f"Frame raw ausente ou inválido: {raw_path}.")

    corrected_profile = copy.deepcopy(CAMERA_PROFILES["down"]["vision"])
    original_profile = copy.deepcopy(corrected_profile)
    original_profile["line_illumination_correction_enabled"] = False
    original_structural, original_mask = segment(raw, original_profile)
    corrected_structural, corrected_mask = segment(raw, corrected_profile)
    gain, zone_mask, status = line_illumination_data(
        corrected_profile,
        raw.shape,
    )
    if gain is None or zone_mask is None or not status["illuminationCorrectionActive"]:
        raise RuntimeError(status.get("illuminationCorrectionError") or "Mapa inativo.")

    raw_overlay = raw.copy()
    zone = zone_mask > 0
    cyan = np.full_like(raw_overlay, (255, 255, 0))
    cyan_overlay = cv2.addWeighted(raw_overlay, 0.78, cyan, 0.22, 0.0)
    raw_overlay[zone] = cyan_overlay[zone]
    preserved = zone & (corrected_mask > 0)
    green = np.full_like(raw_overlay, (0, 255, 0))
    green_overlay = cv2.addWeighted(raw_overlay, 0.45, green, 0.55, 0.0)
    raw_overlay[preserved] = green_overlay[preserved]
    zone_contours, _ = cv2.findContours(
        zone_mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    cv2.drawContours(raw_overlay, zone_contours, -1, (255, 255, 0), 1)

    gain_normalized = np.clip((gain - 1.0) * 255.0, 0, 255).astype(np.uint8)
    gain_panel = cv2.applyColorMap(gain_normalized, cv2.COLORMAP_TURBO)
    gain_panel[~zone] = (gain_panel[~zone].astype(np.float32) * 0.35).astype(np.uint8)

    old_active = original_mask > 0
    new_active = corrected_mask > 0
    delta = np.zeros_like(raw)
    # Cinza preserva candidatos estruturais rejeitados pelos filtros finais.
    delta[(corrected_structural > 0) & ~new_active] = (80, 80, 80)
    delta[old_active & ~new_active] = (0, 0, 255)
    delta[old_active & new_active] = (0, 255, 0)
    delta[~old_active & new_active] = (0, 220, 255)

    final_panel = np.zeros_like(raw)
    final_panel[zone & ~new_active] = (48, 48, 0)
    final_panel[new_active] = (255, 255, 255)
    cv2.drawContours(final_panel, zone_contours, -1, (255, 255, 0), 1)

    panels = (
        titled(raw_overlay, "RGB | ciano: zona compensada | verde: candidato preservado"),
        titled(gain_panel, "Mapa de ganho | azul baixo, vermelho próximo ao limite"),
        titled(delta, "Delta | vermelho: removido | verde: preservado | amarelo: novo"),
        titled(final_panel, "Máscara final | branco é entregue ao Fusion"),
    )
    mosaic = np.vstack((
        np.hstack(panels[:2]),
        np.hstack(panels[2:]),
    ))
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), mosaic):
        raise OSError("Não foi possível gravar o diagnóstico.")

    report = {
        "dataset": portable_path(dataset),
        "sample_id": sample_id,
        "raw_sha256": hashlib.sha256(raw_path.read_bytes()).hexdigest(),
        "reference_sha256": status["illuminationReferenceSha256"],
        "target_gray": status["illuminationTargetGray"],
        "maximum_gain": status["illuminationMaximumGain"],
        "zone_percent": status["illuminationCorrectionZonePercent"],
        "original_candidate_pixels": int(np.count_nonzero(original_mask)),
        "corrected_candidate_pixels": int(np.count_nonzero(corrected_mask)),
        "removed_pixels": int(np.count_nonzero(old_active & ~new_active)),
        "preserved_pixels": int(np.count_nonzero(old_active & new_active)),
        "new_pixels": int(np.count_nonzero(~old_active & new_active)),
        "original_structural_pixels": int(np.count_nonzero(original_structural)),
        "corrected_structural_pixels": int(np.count_nonzero(corrected_structural)),
    }
    output.with_suffix(".json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--sample", default="0000")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    render(args.dataset, args.sample, args.output)


if __name__ == "__main__":
    main()
