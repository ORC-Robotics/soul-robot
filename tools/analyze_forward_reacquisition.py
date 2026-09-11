#!/usr/bin/env python3
"""Transforma uma sessão da CAM1 em resumo, timeline CSV e keyframes."""

import argparse
import bisect
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys


CSV_FIELDS = (
    "timestamp",
    "relativeTime",
    "sequence",
    "forwardPathState",
    "forwardLinePresent",
    "forwardLinePosition",
    "confidence",
    "candidateCount",
    "selectedBandCount",
    "phase",
    "action",
    "commandSource",
    "leftCommand",
    "rightCommand",
    "yaw",
    "gyroZ",
    "ultrasonicDistanceCm",
    "obstacleSelectedSide",
    "leftClearance",
    "rightClearance",
)


def finite_number(value):
    """Retorna somente números reais finitos; campos ausentes viram None."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def sample_timestamp(sample):
    """Usa os timestamps absolutos efetivamente gravados em cada JSONL."""

    for key in ("timestamp", "forwardLineTimestamp"):
        timestamp = finite_number(sample.get(key))
        if timestamp is not None:
            return timestamp
    return None


def parse_iso_timestamp(value):
    if not isinstance(value, str) or not value:
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def load_json_object(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"não foi possível ler {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"{path} deve conter um objeto JSON")
    return value


def load_jsonl(path):
    samples = []
    try:
        with path.open("r", encoding="utf-8") as input_file:
            for line_number, line in enumerate(input_file, start=1):
                if not line.strip():
                    continue
                try:
                    sample = json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(
                        f"JSON inválido em {path}:{line_number}: {error}"
                    ) from error
                if not isinstance(sample, dict):
                    raise ValueError(
                        f"A linha {line_number} de {path} não é um objeto"
                    )
                timestamp = sample_timestamp(sample)
                if timestamp is None:
                    raise ValueError(
                        f"A linha {line_number} de {path} não possui timestamp válido"
                    )
                sample["_analysisTimestamp"] = timestamp
                samples.append(sample)
    except OSError as error:
        raise ValueError(f"não foi possível ler {path}: {error}") from error
    samples.sort(key=lambda sample: sample["_analysisTimestamp"])
    return samples


def nearest_control(controls, control_timestamps, timestamp):
    if not controls:
        return None
    index = bisect.bisect_left(control_timestamps, timestamp)
    if index == 0:
        return controls[0]
    if index == len(controls):
        return controls[-1]
    before = controls[index - 1]
    after = controls[index]
    if timestamp - before["_analysisTimestamp"] <= after["_analysisTimestamp"] - timestamp:
        return before
    return after


def value_or_empty(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    return value


def vision_confidence(sample):
    value = finite_number(sample.get("forwardPathConfidence"))
    if value is None:
        value = finite_number(sample.get("forwardLineConfidence"))
    return value


def vision_image_path(session_dir, sample):
    relative_path = sample.get("overlayFile") or sample.get("frameFile")
    if not isinstance(relative_path, str) or not relative_path:
        return None
    return (session_dir / relative_path).resolve()


def write_analysis_csv(session_dir, visions, controls, relative_start):
    output_path = session_dir / "analysis.csv"
    temporary_path = session_dir / ".analysis.csv.tmp"
    control_timestamps = [sample["_analysisTimestamp"] for sample in controls]
    try:
        with temporary_path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for vision in visions:
                timestamp = vision["_analysisTimestamp"]
                control = nearest_control(controls, control_timestamps, timestamp)
                obstacle = control.get("obstacle", {}) if control else {}
                if not isinstance(obstacle, dict):
                    obstacle = {}
                candidates = vision.get("candidates", [])
                selected_bands = vision.get("selectedBands", [])
                row = {
                    "timestamp": timestamp,
                    "relativeTime": timestamp - relative_start,
                    "sequence": vision.get("forwardLineSequence", ""),
                    "forwardPathState": vision.get("forwardPathState", ""),
                    "forwardLinePresent": vision.get("forwardLinePresent", ""),
                    "forwardLinePosition": finite_number(
                        vision.get("forwardLinePosition")
                    ),
                    "confidence": vision_confidence(vision),
                    "candidateCount": len(candidates)
                    if isinstance(candidates, list)
                    else 0,
                    "selectedBandCount": len(selected_bands)
                    if isinstance(selected_bands, list)
                    else 0,
                    "phase": control.get("autonomousPhase", "")
                    if control
                    else "",
                    "action": control.get("action", "") if control else "",
                    "commandSource": control.get("movementCommandSource", "")
                    if control
                    else "",
                    "leftCommand": finite_number(control.get("finalLeftCommand"))
                    if control
                    else None,
                    "rightCommand": finite_number(control.get("finalRightCommand"))
                    if control
                    else None,
                    "yaw": finite_number(control.get("yawDegrees"))
                    if control
                    else None,
                    "gyroZ": finite_number(
                        control.get("gyroZDegreesPerSecond")
                    )
                    if control
                    else None,
                    "ultrasonicDistanceCm": finite_number(
                        control.get("ultrasonicDistanceCm")
                    )
                    if control
                    else None,
                    "obstacleSelectedSide": obstacle.get("selectedSide", ""),
                    "leftClearance": finite_number(
                        obstacle.get("leftClearanceCm")
                    ),
                    "rightClearance": finite_number(
                        obstacle.get("rightClearanceCm")
                    ),
                }
                writer.writerow(
                    {key: value_or_empty(value) for key, value in row.items()}
                )
        os.replace(temporary_path, output_path)
    except Exception:
        try:
            temporary_path.unlink()
        except OSError:
            pass
        raise
    return output_path


def phase_summary(controls, session_end):
    if not controls:
        return [], {}
    ordered_phases = []
    previous_phase = None
    durations = {}
    for index, control in enumerate(controls):
        phase = str(control.get("autonomousPhase") or "unknown")
        if phase != previous_phase:
            ordered_phases.append(phase)
            previous_phase = phase
        current_timestamp = control["_analysisTimestamp"]
        next_timestamp = (
            controls[index + 1]["_analysisTimestamp"]
            if index + 1 < len(controls)
            else session_end
        )
        durations[phase] = durations.get(phase, 0.0) + max(
            0.0, next_timestamp - current_timestamp
        )
    return ordered_phases, durations


def latest_obstacle_values(controls):
    selected_side = None
    left_clearance = None
    right_clearance = None
    for control in controls:
        obstacle = control.get("obstacle")
        if not isinstance(obstacle, dict):
            continue
        side = obstacle.get("selectedSide")
        if isinstance(side, str) and side and side != "NONE":
            selected_side = side
        left = finite_number(obstacle.get("leftClearanceCm"))
        right = finite_number(obstacle.get("rightClearanceCm"))
        if left is not None:
            left_clearance = left
        if right is not None:
            right_clearance = right
    return selected_side, left_clearance, right_clearance


def transition_summary(visions):
    transitions = {}
    previous_state = None
    for vision in visions:
        state = str(vision.get("forwardPathState") or "UNKNOWN")
        if previous_state is not None and state != previous_state:
            transition = f"{previous_state}->{state}"
            transitions[transition] = transitions.get(transition, 0) + 1
        previous_state = state
    return transitions


def identify_keyframes(session_dir, visions, controls):
    if not visions:
        return []
    vision_timestamps = [sample["_analysisTimestamp"] for sample in visions]
    keyframes = {}

    def add(label, vision):
        if vision is None:
            return
        path = vision_image_path(session_dir, vision)
        if path is None:
            return
        keyframes.setdefault(path, []).append(label)

    confidence_samples = [
        vision for vision in visions if vision_confidence(vision) is not None
    ]
    if confidence_samples:
        add(
            "maior confidence",
            max(confidence_samples, key=vision_confidence),
        )

    present_samples = [
        vision
        for vision in visions
        if vision.get("forwardPathState") == "PRESENT"
    ]
    if present_samples:
        add("primeiro PRESENT", present_samples[0])
        add("último PRESENT", present_samples[-1])

    previous_phase = None
    for control in controls:
        phase = str(control.get("autonomousPhase") or "")
        if phase != previous_phase and phase.startswith("obstacle_"):
            add(
                f"início {phase}",
                nearest_control(
                    visions,
                    vision_timestamps,
                    control["_analysisTimestamp"],
                ),
            )
        previous_phase = phase

    if controls:
        final_control = controls[-1]
        left = finite_number(final_control.get("finalLeftCommand"))
        right = finite_number(final_control.get("finalRightCommand"))
        if left == 0.0 and right == 0.0:
            add(
                "parada final",
                nearest_control(
                    visions,
                    vision_timestamps,
                    final_control["_analysisTimestamp"],
                ),
            )

    for vision in visions:
        candidates = vision.get("candidates")
        if isinstance(candidates, list) and len(candidates) > 1:
            add(f"múltiplos candidates ({len(candidates)})", vision)

    return [(labels, path) for path, labels in keyframes.items()]


def format_seconds(value):
    return f"{max(0.0, value):.3f} s"


def format_timestamp(sample, relative_start):
    if sample is None:
        return "--"
    timestamp = sample["_analysisTimestamp"]
    return f"{timestamp:.6f} (t+{timestamp - relative_start:.3f} s)"


def analyze(session_dir):
    session_dir = session_dir.resolve()
    if not session_dir.is_dir():
        raise ValueError(f"sessão não encontrada: {session_dir}")

    session = load_json_object(session_dir / "session.json")
    visions = load_jsonl(session_dir / "vision.jsonl")
    controls = load_jsonl(session_dir / "control.jsonl")
    all_timestamps = [
        sample["_analysisTimestamp"] for sample in visions + controls
    ]
    metadata_start = parse_iso_timestamp(session.get("startedAt"))
    metadata_end = parse_iso_timestamp(session.get("endedAt"))
    relative_start = metadata_start
    if relative_start is None:
        relative_start = min(all_timestamps) if all_timestamps else 0.0
    session_end = metadata_end
    if session_end is None:
        session_end = max(all_timestamps) if all_timestamps else relative_start
    duration = max(0.0, session_end - relative_start)

    output_path = write_analysis_csv(
        session_dir, visions, controls, relative_start
    )
    ordered_phases, phase_durations = phase_summary(controls, session_end)
    selected_side, left_clearance, right_clearance = latest_obstacle_values(
        controls
    )
    present_samples = [
        vision
        for vision in visions
        if vision.get("forwardPathState") == "PRESENT"
    ]
    confidences = [
        vision_confidence(vision)
        for vision in visions
        if vision_confidence(vision) is not None
    ]
    positions = [
        finite_number(vision.get("forwardLinePosition")) for vision in visions
    ]
    positions = [position for position in positions if position is not None]
    transitions = transition_summary(visions)

    print(f"Sessão: {session.get('label') or session_dir.name}")
    print(f"Duração: {format_seconds(duration)}")
    print(f"Frames visuais: {len(visions)}")
    print(f"Amostras de controle: {len(controls)}")
    print("Fases (ordem): " + (" -> ".join(ordered_phases) or "--"))
    print(
        "Tempo por fase: "
        + (
            "; ".join(
                f"{phase}={format_seconds(phase_durations[phase])}"
                for phase in phase_durations
            )
            or "--"
        )
    )
    print(f"Lado selecionado: {selected_side or '--'}")
    print(
        "Clearance esquerda/direita: "
        f"{left_clearance if left_clearance is not None else '--'} / "
        f"{right_clearance if right_clearance is not None else '--'} cm"
    )
    print(
        "Primeiro PRESENT: "
        + format_timestamp(present_samples[0] if present_samples else None, relative_start)
    )
    print(
        "Último PRESENT: "
        + format_timestamp(present_samples[-1] if present_samples else None, relative_start)
    )
    print(
        "Maior confidence: "
        + (f"{max(confidences):.6f}" if confidences else "--")
    )
    print(
        "Faixa de position: "
        + (f"{min(positions):.6f} a {max(positions):.6f}" if positions else "--")
    )
    print(
        f"Transições de estado: {sum(transitions.values())}"
        + (
            " ("
            + ", ".join(
                f"{transition}={count}"
                for transition, count in transitions.items()
            )
            + ")"
            if transitions
            else ""
        )
    )
    print(f"CSV: {output_path}")
    print("Keyframes:")
    keyframes = identify_keyframes(session_dir, visions, controls)
    if not keyframes:
        print("  --")
    for labels, path in keyframes:
        print(f"  {', '.join(labels)}: {path}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Analisa uma sessão de reacquisição frontal."
    )
    parser.add_argument(
        "session",
        type=Path,
        help="Pasta que contém session.json, vision.jsonl e control.jsonl.",
    )
    args = parser.parse_args(argv)
    try:
        analyze(args.session)
    except ValueError as error:
        print(f"Erro: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
