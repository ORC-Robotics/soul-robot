"""Constrói a referência fixa de iluminação da câmera inferior."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2  # type: ignore
import numpy as np


SCRIPTS_DIRECTORY = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPTS_DIRECTORY.parent
sys.path.insert(0, str(SCRIPTS_DIRECTORY))

from vision.illumination_correction import build_smoothed_line_illumination_reference


def sha256_file(path):
    """Calcula o hash do arquivo sem depender do nome ou do horário local."""

    return hashlib.sha256(path.read_bytes()).hexdigest()


def repository_relative(path):
    """Publica caminhos reproduzíveis quando o arquivo pertence ao repositório."""

    try:
        return path.resolve().relative_to(REPOSITORY_ROOT.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def load_verified_gray_frames(dataset):
    """Lê PNGs lossless e confirma os hashes registrados durante a captura."""

    raw_directory = dataset / "raw"
    metadata_directory = dataset / "metadata"
    paths = sorted(raw_directory.glob("*.png"))
    if len(paths) != 90:
        raise ValueError(
            f"A referência definitiva exige 90 frames; encontrados {len(paths)}."
        )

    frames = []
    frame_hashes = {}
    shape = None
    for path in paths:
        metadata_path = metadata_directory / f"{path.stem}.json"
        if not metadata_path.is_file():
            raise ValueError(f"Metadados ausentes para {path.name}.")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        expected_hash = metadata.get("raw_sha256")
        actual_hash = sha256_file(path)
        if not expected_hash or expected_hash != actual_hash:
            raise ValueError(f"Hash do RGB não confere: {path.name}.")
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"PNG inválido: {path.name}.")
        if shape is None:
            shape = frame.shape
        if frame.shape != shape:
            raise ValueError(f"Resolução inconsistente: {path.name}.")
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        frame_hashes[path.name] = actual_hash
    return frames, frame_hashes


def build_reference(dataset, output, sigma_px, target_percentile,
                    maximum_gain, useful_start_ratio, zone_gain_threshold):
    """Gera PNG e manifesto determinísticos a partir de uma captura segura."""

    config_path = dataset / "config.json"
    result_path = dataset / "result.json"
    if not config_path.is_file() or not result_path.is_file():
        raise ValueError("O dataset precisa conter config.json e result.json.")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    result = json.loads(result_path.read_text(encoding="utf-8"))
    request = config.get("request", {})
    if request.get("scenario") != "white_floor":
        raise ValueError("A referência só pode usar um dataset white_floor.")
    if config.get("raw_format") != "lossless PNG from BGR main; not Bayer":
        raise ValueError("O dataset não confirma PNG BGR lossless.")
    safety = config.get("safety_at_start", {})
    if (
        safety.get("emergency") is not True
        or safety.get("esp32EmergencyStop") is not True
        or any(
            safety.get(key) != 0
            for key in (
                "appliedLeft",
                "appliedRight",
                "leftEncoderRate",
                "rightEncoderRate",
            )
        )
    ):
        raise ValueError("O dataset não comprova E-Stop e motores parados.")

    gray_frames, frame_hashes = load_verified_gray_frames(dataset)
    reference = build_smoothed_line_illumination_reference(
        gray_frames,
        sigma_px,
    )
    if reference.shape != (360, 480):
        raise ValueError(
            f"A referência inferior deve ser 480×360; recebida {reference.shape[::-1]}."
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), reference, [cv2.IMWRITE_PNG_COMPRESSION, 9]):
        raise OSError("Não foi possível gravar a referência PNG.")

    useful_start_y = int(round(reference.shape[0] * useful_start_ratio))
    target_gray = float(np.percentile(
        reference[useful_start_y:, :],
        target_percentile,
    ))
    gain = np.clip(
        target_gray / np.maximum(reference.astype(np.float32), 1.0),
        1.0,
        maximum_gain,
    )
    source_sequence_sha256 = hashlib.sha256(
        "".join(frame_hashes.values()).encode("ascii")
    ).hexdigest()
    manifest = {
        "schema_version": 1,
        "source_dataset": repository_relative(dataset),
        "source_config_sha256": sha256_file(config_path),
        "source_result_sha256": sha256_file(result_path),
        "source_sequence_sha256": source_sequence_sha256,
        "source_frame_sha256": frame_hashes,
        "source_frame_count": len(gray_frames),
        "source_mask_used": False,
        "source_raw_format": config["raw_format"],
        "source_camera": config.get("camera_details", {}),
        "source_controls": config.get("requested_camera_controls", {}),
        "source_safety_at_start": safety,
        "captured_at_unix_seconds": result.get("timestamp_unix_seconds"),
        "algorithm": {
            "temporal_reducer": "per-pixel median",
            "spatial_filter": "GaussianBlur BORDER_REFLECT101",
            "sigma_px_at_480x360": float(sigma_px),
            "target_percentile": float(target_percentile),
            "maximum_gain": float(maximum_gain),
            "minimum_gain": 1.0,
            "useful_start_ratio": float(useful_start_ratio),
            "zone_gain_threshold": float(zone_gain_threshold),
        },
        "reference": {
            "path": repository_relative(output),
            "sha256": sha256_file(output),
            "width": int(reference.shape[1]),
            "height": int(reference.shape[0]),
            "target_gray": target_gray,
            "minimum_gray": int(reference.min()),
            "maximum_gray": int(reference.max()),
            "maximum_applied_gain": float(gain.max()),
            "correction_zone_percent_before_overlay_margin": (
                100.0 * float(np.mean(gain[useful_start_y:, :] >= zone_gain_threshold))
            ),
        },
    }
    manifest_path = output.with_suffix(".json")
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    return manifest_path, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            SCRIPTS_DIRECTORY
            / "vision"
            / "calibration"
            / "down_line_illumination_480x360.png"
        ),
    )
    parser.add_argument("--sigma", type=float, default=15.0)
    parser.add_argument("--target-percentile", type=float, default=85.0)
    parser.add_argument("--max-gain", type=float, default=2.0)
    parser.add_argument("--useful-start-ratio", type=float, default=0.13)
    parser.add_argument("--zone-gain-threshold", type=float, default=1.10)
    args = parser.parse_args()
    manifest_path, manifest = build_reference(
        args.dataset,
        args.output,
        args.sigma,
        args.target_percentile,
        args.max_gain,
        args.useful_start_ratio,
        args.zone_gain_threshold,
    )
    print(json.dumps({
        "reference": str(args.output),
        "manifest": str(manifest_path),
        "sha256": manifest["reference"]["sha256"],
        "frames": manifest["source_frame_count"],
        "target_gray": manifest["reference"]["target_gray"],
        "maximum_applied_gain": manifest["reference"]["maximum_applied_gain"],
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
