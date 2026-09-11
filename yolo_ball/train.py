"""Valida o dataset e treina o detector YOLO de bolas."""

import argparse
from pathlib import Path

from ultralytics import YOLO


PROJECT_DIR = Path(__file__).resolve().parent
DATASET_DIR = PROJECT_DIR / "datasets"
DATA_CONFIG = DATASET_DIR / "data.yaml"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
CLASS_NAMES = {0: "black_ball", 1: "silver_ball"}


def image_paths(dataset_dir, split):
    directory_name = "valid" if split == "val" else split
    directory = dataset_dir / directory_name / "images"
    return sorted(
        path for path in directory.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def validate_label(label_path):
    """Valida IDs e coordenadas normalizadas de um arquivo YOLO."""

    for line_number, raw_line in enumerate(
        label_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        values = raw_line.split()
        if len(values) != 5:
            raise ValueError(f"{label_path}:{line_number}: esperado 'classe x y largura altura'.")
        try:
            class_id = int(values[0])
            coordinates = tuple(float(value) for value in values[1:])
        except ValueError as error:
            raise ValueError(f"{label_path}:{line_number}: valor inválido.") from error
        if class_id not in CLASS_NAMES:
            raise ValueError(f"{label_path}:{line_number}: classe deve ser 0 ou 1.")
        if any(value < 0.0 or value > 1.0 for value in coordinates):
            raise ValueError(f"{label_path}:{line_number}: coordenadas devem ficar entre 0 e 1.")
        if coordinates[2] <= 0.0 or coordinates[3] <= 0.0:
            raise ValueError(f"{label_path}:{line_number}: largura e altura devem ser positivas.")


def validate_dataset(dataset_dir=DATASET_DIR):
    """Impede treino vazio e confere labels; ausência de label indica negativo."""

    summary = {}
    for split in ("train", "val"):
        images = image_paths(dataset_dir, split)
        if not images:
            raise ValueError(f"Nenhuma imagem encontrada no conjunto {split}.")
        annotated = 0
        negative = 0
        for image_path in images:
            directory_name = "valid" if split == "val" else split
            relative = image_path.relative_to(dataset_dir / directory_name / "images")
            label_path = (dataset_dir / directory_name / "labels" / relative).with_suffix(".txt")
            if label_path.exists():
                validate_label(label_path)
                annotated += bool(label_path.read_text(encoding="utf-8").strip())
                negative += not bool(label_path.read_text(encoding="utf-8").strip())
            else:
                negative += 1
        if annotated == 0:
            raise ValueError(f"O conjunto {split} não possui nenhuma bounding box anotada.")
        summary[split] = (len(images), annotated, negative)
    return summary


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(description="Treina YOLO para bolas pretas e prateadas.")
    parser.add_argument("--data", type=Path, default=DATA_CONFIG)
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--name", default="ball_detector")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument(
        "--batch",
        type=int,
        default=8,
        help="Imagens por lote; use -1 para ajustar automaticamente à memória da GPU.",
    )
    parser.add_argument("--device", default=None, help="Exemplos: cpu, 0, 0,1")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument(
        "--cache",
        choices=("false", "ram", "disk"),
        default="false",
        help="Cache das imagens; ram é mais rápido e consome memória principal.",
    )
    return parser.parse_args(arguments)


def main(arguments=None):
    args = parse_arguments(arguments)
    if (
        args.epochs <= 0
        or args.image_size <= 0
        or args.batch == 0
        or args.batch < -1
        or args.workers < 0
    ):
        raise ValueError(
            "epochs e image-size devem ser positivos; batch deve ser -1 ou positivo; "
            "workers não pode ser negativo."
        )
    data_config = args.data.resolve()
    if not data_config.is_file():
        raise ValueError(f"Configuração do dataset não encontrada: {data_config}")
    summary = validate_dataset(data_config.parent)
    for split, (total, annotated, negative) in summary.items():
        print(f"{split}: {total} imagens, {annotated} anotadas, {negative} negativas")
    model = YOLO(args.model)
    cache = False if args.cache == "false" else args.cache
    model.train(
        data=str(data_config),
        epochs=args.epochs,
        imgsz=args.image_size,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        cache=cache,
        project=str(PROJECT_DIR / "runs"),
        name=args.name,
        exist_ok=True,
    )
    print(f"Melhor modelo: {PROJECT_DIR / 'runs' / args.name / 'weights' / 'best.pt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
