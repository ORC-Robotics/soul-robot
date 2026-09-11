"""Valida sessões anotadas e as adiciona aos conjuntos train e valid."""

import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
DATASET_DIR = PROJECT_DIR / "datasets"
ANNOTATED_DIR = DATASET_DIR / "annotated"
MANIFEST_PATH = DATASET_DIR / "split_manifest.json"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp"}
CLASS_NAMES = {0: "black_ball", 1: "silver_ball"}
SAFE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")


def image_paths(directory):
    """Lista imagens de forma estável para tornar a preparação reproduzível."""

    return sorted(
        path
        for path in directory.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def validate_label(label_path):
    """Valida uma anotação YOLO; arquivo vazio representa uma imagem negativa."""

    box_count = 0
    for line_number, raw_line in enumerate(
        label_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        values = raw_line.split()
        if not values:
            continue
        if len(values) != 5:
            raise ValueError(
                f"{label_path}:{line_number}: esperado 'classe x y largura altura'."
            )
        try:
            class_id = int(values[0])
            coordinates = tuple(float(value) for value in values[1:])
        except ValueError as error:
            raise ValueError(f"{label_path}:{line_number}: valor inválido.") from error
        if class_id not in CLASS_NAMES:
            raise ValueError(f"{label_path}:{line_number}: classe deve ser 0 ou 1.")
        if any(value < 0.0 or value > 1.0 for value in coordinates):
            raise ValueError(
                f"{label_path}:{line_number}: coordenadas devem ficar entre 0 e 1."
            )
        if coordinates[2] <= 0.0 or coordinates[3] <= 0.0:
            raise ValueError(
                f"{label_path}:{line_number}: largura e altura devem ser positivas."
            )
        box_count += 1
    return box_count


def load_sessions(source_dir):
    """Carrega sessões completas e impede tratar label ausente como negativo."""

    if not source_dir.is_dir():
        raise ValueError(f"Diretório de sessões anotadas não encontrado: {source_dir}")

    sessions = {}
    for session_dir in sorted(path for path in source_dir.iterdir() if path.is_dir()):
        if not SAFE_NAME_PATTERN.fullmatch(session_dir.name):
            raise ValueError(f"Nome de sessão inválido: {session_dir.name}")
        images_dir = session_dir / "images"
        labels_dir = session_dir / "labels"
        images = image_paths(images_dir) if images_dir.is_dir() else []
        if not images:
            continue
        if not labels_dir.is_dir():
            raise ValueError(f"Pasta de labels ausente: {labels_dir}")

        entries = []
        expected_labels = set()
        total_boxes = 0
        negative_images = 0
        for image_path in images:
            relative_path = image_path.relative_to(images_dir)
            label_path = (labels_dir / relative_path).with_suffix(".txt")
            if not label_path.is_file():
                raise ValueError(
                    f"Label ausente para {image_path}. Crie um .txt vazio se a imagem for negativa."
                )
            expected_labels.add(label_path.resolve())
            box_count = validate_label(label_path)
            total_boxes += box_count
            negative_images += box_count == 0
            entries.append((image_path, label_path, relative_path))

        orphan_labels = sorted(
            path
            for path in labels_dir.rglob("*.txt")
            if path.resolve() not in expected_labels
        )
        if orphan_labels:
            raise ValueError(f"Label sem imagem correspondente: {orphan_labels[0]}")

        sessions[session_dir.name] = {
            "entries": entries,
            "image_count": len(entries),
            "box_count": total_boxes,
            "negative_count": negative_images,
        }

    if not sessions:
        raise ValueError(f"Nenhuma sessão anotada encontrada em {source_dir}")
    return sessions


def stable_session_order(session_names, seed):
    """Ordena sessões por hash para que o split seja repetível em qualquer computador."""

    return sorted(
        session_names,
        key=lambda name: hashlib.sha256(f"{seed}:{name}".encode("utf-8")).hexdigest(),
    )


def assign_splits(sessions, validation_ratio, seed, existing_assignments=None):
    """Mantém sessões inteiras e aproxima a proporção desejada de validação."""

    if len(sessions) < 2:
        raise ValueError(
            "São necessárias pelo menos duas sessões anotadas para separar train e valid sem vazamento."
        )

    existing_assignments = existing_assignments or {}
    assignments = {
        name: split
        for name, split in existing_assignments.items()
        if name in sessions and split in ("train", "valid")
    }
    ordered_names = stable_session_order(
        (name for name in sessions if name not in assignments), seed
    )
    target_validation_count = round(len(sessions) * validation_ratio)
    target_validation_count = max(1, min(len(sessions) - 1, target_validation_count))
    missing_validation_count = max(
        0,
        target_validation_count
        - sum(split == "valid" for split in assignments.values()),
    )
    for index, name in enumerate(ordered_names):
        assignments[name] = "valid" if index < missing_validation_count else "train"

    if set(assignments.values()) != {"train", "valid"}:
        raise ValueError(
            "O manifesto existente não permite manter ao menos uma sessão em train e valid."
        )
    return assignments


def read_existing_assignments():
    """Preserva o split já escolhido para uma sessão em execuções futuras."""

    try:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, ValueError, TypeError, json.JSONDecodeError):
        return {}
    raw_sessions = manifest.get("sessions")
    if not isinstance(raw_sessions, dict):
        return {}
    return {
        name: status.get("split")
        for name, status in raw_sessions.items()
        if isinstance(name, str) and isinstance(status, dict)
    }


def destination_stem(session_name, relative_path):
    """Gera nome único sem depender das pastas organizacionais da captura."""

    relative_without_suffix = relative_path.with_suffix("")
    safe_parts = [
        re.sub(r"[^A-Za-z0-9_-]", "_", part)
        for part in relative_without_suffix.parts
    ]
    return "__".join((session_name, *safe_parts))


def copy_file(source, destination, overwrite):
    """Evita substituir silenciosamente uma imagem ou anotação já preparada."""

    if destination.exists() and not overwrite:
        if source.read_bytes() == destination.read_bytes():
            return False
        raise FileExistsError(
            f"Destino já existe com conteúdo diferente: {destination}. "
            "Use --overwrite para atualizar."
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return True


def prepare_dataset(source_dir, validation_ratio, seed, overwrite=False):
    """Valida, separa por sessão e copia pares imagem/label para o dataset final."""

    sessions = load_sessions(source_dir)
    assignments = assign_splits(
        sessions, validation_ratio, seed, read_existing_assignments()
    )
    copied_files = 0

    for session_name, session in sessions.items():
        split = assignments[session_name]
        for image_path, label_path, relative_path in session["entries"]:
            stem = destination_stem(session_name, relative_path)
            image_destination = (
                DATASET_DIR / split / "images" / f"{stem}{image_path.suffix.lower()}"
            )
            label_destination = DATASET_DIR / split / "labels" / f"{stem}.txt"
            copied_files += copy_file(image_path, image_destination, overwrite)
            copied_files += copy_file(label_path, label_destination, overwrite)

    manifest = {
        "validationRatio": validation_ratio,
        "seed": seed,
        "sessions": {
            name: {
                "split": assignments[name],
                "images": sessions[name]["image_count"],
                "boxes": sessions[name]["box_count"],
                "negatives": sessions[name]["negative_count"],
            }
            for name in sorted(sessions)
        },
    }
    MANIFEST_PATH.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return manifest, copied_files


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description="Valida sessões anotadas e prepara os conjuntos train e valid."
    )
    parser.add_argument("--source", type=Path, default=ANNOTATED_DIR)
    parser.add_argument("--validation-ratio", type=float, default=0.20)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(arguments)


def main(arguments=None):
    args = parse_arguments(arguments)
    if not 0.0 < args.validation_ratio < 1.0:
        raise ValueError("validation-ratio deve ficar entre 0 e 1.")

    manifest, copied_files = prepare_dataset(
        args.source.resolve(), args.validation_ratio, args.seed, args.overwrite
    )
    print(f"Preparação concluída: {copied_files} arquivos novos ou atualizados.")
    for session_name, status in manifest["sessions"].items():
        print(
            f"{session_name}: {status['split']} | {status['images']} imagens | "
            f"{status['boxes']} boxes | {status['negatives']} negativas"
        )
    print(f"Manifesto: {MANIFEST_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
