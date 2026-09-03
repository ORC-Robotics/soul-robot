from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path
import json
import os
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATASET_ROOT = PROJECT_ROOT / "dataset" / "raw"

CONTROL_PATH = Path("/dev/shm/obr_silver_dataset_capture.json")
TEMP_CONTROL_PATH = Path("/dev/shm/obr_silver_dataset_capture.tmp.json")

CLASS_NAMES = ("black", "other", "silver")


def parse_arguments():
    parser = ArgumentParser(description="Controla a coleta do dataset da faixa prata.")
    parser.add_argument("--camera", choices=("forward", "down"), required=True)
    parser.add_argument("--session", help="Nome da sessão. Se omitido, um nome é gerado automaticamente.")
    parser.add_argument("--fps", type=float, default=4.0, help="Taxa de captura do dataset.")
    return parser.parse_args()


def create_session_name():
    return datetime.now().strftime("session_%Y%m%d_%H%M%S")


def write_control(camera, session, label, fps, active):
    data = {
        "active": bool(active),
        "camera": camera,
        "session": session,
        "label": label,
        "fps": float(fps),
        "requestedAt": time.time(),
    }

    TEMP_CONTROL_PATH.write_text(json.dumps(data), encoding="utf-8")
    os.replace(TEMP_CONTROL_PATH, CONTROL_PATH)


def prepare_session(camera, session):
    session_dir = DATASET_ROOT / camera / session

    for class_name in CLASS_NAMES:
        (session_dir / class_name).mkdir(parents=True, exist_ok=True)

    return session_dir


def count_images(session_dir):
    return {
        class_name: len(list((session_dir / class_name).glob("*.jpg")))
        for class_name in CLASS_NAMES
    }


def print_status(camera, session, label, active, fps, session_dir):
    counts = count_images(session_dir)

    print()
    print(f"Câmera:  {camera}")
    print(f"Sessão:  {session}")
    print(f"Estado:  {'CAPTURANDO' if active else 'PAUSADO'}")
    print(f"Classe:  {label}")
    print(f"Taxa:    {fps:.1f} FPS")
    print()
    print(f"black:   {counts['black']}")
    print(f"other:   {counts['other']}")
    print(f"silver:  {counts['silver']}")
    print()


def main():
    args = parse_arguments()

    if not 0.5 <= args.fps <= 10.0:
        raise ValueError("--fps deve ficar entre 0.5 e 10.")

    session = args.session or create_session_name()
    session_dir = prepare_session(args.camera, session)

    label = "other"
    active = False

    write_control(args.camera, session, label, args.fps, active)

    print()
    print("COLETA DO DATASET DE FAIXA PRATA")
    print()
    print("[b] capturar BLACK")
    print("[o] capturar OTHER")
    print("[s] capturar SILVER")
    print("[p] pausar")
    print("[r] atualizar contagem")
    print("[q] encerrar")

    print_status(args.camera, session, label, active, args.fps, session_dir)

    try:
        while True:
            command = input("> ").strip().lower()

            if command in ("b", "black"):
                label = "black"
                active = True
            elif command in ("o", "other"):
                label = "other"
                active = True
            elif command in ("s", "silver"):
                label = "silver"
                active = True
            elif command in ("p", "pause"):
                active = False
            elif command in ("r", "refresh"):
                print_status(args.camera, session, label, active, args.fps, session_dir)
                continue
            elif command in ("q", "quit", "exit"):
                break
            else:
                print("Comando inválido.")
                continue

            write_control(args.camera, session, label, args.fps, active)
            print_status(args.camera, session, label, active, args.fps, session_dir)

    except KeyboardInterrupt:
        print()

    finally:
        write_control(args.camera, session, label, args.fps, False)

    print("Captura encerrada.")
    print_status(args.camera, session, label, False, args.fps, session_dir)


if __name__ == "__main__":
    main()