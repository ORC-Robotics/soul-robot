"""Exporta o melhor modelo YOLO para um formato de implantação."""

import argparse
from pathlib import Path

from ultralytics import YOLO


PROJECT_DIR = Path(__file__).resolve().parent


def main(arguments=None):
    parser = argparse.ArgumentParser(description="Exporta o detector YOLO treinado.")
    parser.add_argument(
        "--weights",
        default=str(
            PROJECT_DIR /
            "runs" /
            "ball_detector_combined_stable" /
            "weights" /
            "best.pt"
        ),
    )
    parser.add_argument("--format", default="onnx", choices=("onnx", "tflite", "openvino"))
    parser.add_argument(
        "--image-size",
        type=int,
        default=384,
        help="Resolução de inferência; 384 é o perfil de CPU da Raspberry Pi.",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=12,
        help="Versão ONNX; 12 é compatível com o OpenCV 4.6 da Raspberry Pi.",
    )
    args = parser.parse_args(arguments)
    weights = Path(args.weights)
    if not weights.is_file():
        raise FileNotFoundError(f"Modelo não encontrado: {weights}")
    export_arguments = {"format": args.format, "imgsz": args.image_size}
    if args.format == "onnx":
        export_arguments["opset"] = args.opset
        export_arguments["simplify"] = True
    output = YOLO(str(weights)).export(**export_arguments)
    print(f"Modelo exportado: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
