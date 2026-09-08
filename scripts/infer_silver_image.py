"""Executa a detecção de faixa prata em uma imagem da câmera inferior."""

from argparse import ArgumentParser
from pathlib import Path

import cv2

from vision.silver_detection import SilverLineDetector


def parse_arguments():
    parser = ArgumentParser(
        description="Classifica uma imagem como black, other ou silver."
    )
    parser.add_argument("image", type=Path, help="Caminho da imagem a classificar.")
    return parser.parse_args()


def main():
    args = parse_arguments()

    if not args.image.is_file():
        raise FileNotFoundError(f"Imagem não encontrada: {args.image}")

    frame = cv2.imread(str(args.image))
    if frame is None:
        raise RuntimeError(f"Não foi possível abrir a imagem: {args.image}")

    detector = SilverLineDetector.from_camera_model("down")
    result = detector.detect(frame)

    print(f"Imagem:              {args.image}")
    print(f"Classe vencedora:    {result.label}")
    print(f"Detectou prata:      {'SIM' if result.detected else 'NÃO'}")
    print(f"Confiança:           {result.confidence:.2%}")
    print(f"Probabilidade black: {result.classification.black:.2%}")
    print(f"Probabilidade other: {result.classification.other:.2%}")
    print(f"Probabilidade silver: {result.classification.silver:.2%}")
    print(f"Margem da prata:     {result.silver_margin:.2%}")
    print(f"Tempo total:         {result.classification.total_ms:.2f} ms")


if __name__ == "__main__":
    main()
