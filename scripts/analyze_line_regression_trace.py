"""Resume traces A/B/C da regressão sem alterar parâmetros do robô."""

import argparse
import csv
import math
import statistics


# Este é o limite já usado pelo controle; o analisador somente conta ocorrências.
CURRENT_MAX_CORRECTION = 0.15


def percentile(values, percentage):
    """Calcula um percentil interpolado para listas pequenas ou grandes."""

    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * percentage / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def numeric(rows, field, positive_only=False):
    """Extrai números finitos e pode ignorar o primeiro zero de uma frequência."""

    values = []
    for row in rows:
        try:
            value = float(row[field])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value) and (not positive_only or value > 0.0):
            values.append(value)
    return values


def distribution(values):
    """Retorna média, mínimo, máximo e p95 com nomes estáveis."""

    if not values:
        return {"mean": 0.0, "min": 0.0, "max": 0.0, "p95": 0.0}
    return {
        "mean": statistics.fmean(values),
        "min": min(values),
        "max": max(values),
        "p95": percentile(values, 95.0),
    }


def read_trace(path):
    with open(path, "r", encoding="utf-8", newline="") as trace_file:
        return list(csv.DictReader(trace_file))


def summarize(rows):
    """Calcula as métricas pedidas para uma passagem física."""

    control_errors = numeric(rows, "controlError")
    corrections = numeric(rows, "correction")
    linear = numeric(rows, "linearComponent")
    timestamps = numeric(rows, "monotonicTimestamp")
    turning_entries = 0
    near_losses = 0
    counter_rotation_seconds = 0.0
    previous_state = None
    previous_near_valid = None
    for index, row in enumerate(rows):
        state = row.get("mainMissionState", "")
        near_valid = row.get("nearValid", "false").lower() == "true"
        if state == "TurningAhead" and previous_state != "TurningAhead":
            turning_entries += 1
        if previous_near_valid is True and not near_valid:
            near_losses += 1
        if index + 1 < len(rows):
            left = float(row.get("finalLeft", 0.0))
            right = float(row.get("finalRight", 0.0))
            if left * right < 0.0:
                counter_rotation_seconds += max(
                    0.0, timestamps[index + 1] - timestamps[index]
                )
        previous_state = state
        previous_near_valid = near_valid

    linear_jumps = [
        abs(current - previous)
        for previous, current in zip(linear, linear[1:])
    ]
    return {
        "samples": len(rows),
        "newLineSequenceHz": distribution(
            numeric(rows, "processingHz", positive_only=True)
        )["mean"],
        "frameAgeP95Ms": distribution(numeric(rows, "frameAgeMs"))["p95"],
        "controlErrorRms": math.sqrt(
            statistics.fmean(value * value for value in control_errors)
        ) if control_errors else 0.0,
        "maximumAbsControlError": max(
            (abs(value) for value in control_errors), default=0.0
        ),
        "turningAheadEntries": turning_entries,
        "nearLosses": near_losses,
        "saturatedCorrections": sum(
            abs(value) >= CURRENT_MAX_CORRECTION - 1e-9
            for value in corrections
        ),
        "counterRotationSeconds": counter_rotation_seconds,
        "maximumLinearJump": max(linear_jumps, default=0.0),
    }


def print_distribution(label, values):
    result = distribution(values)
    print(
        f"{label}: mean={result['mean']:.3f} min={result['min']:.3f} "
        f"max={result['max']:.3f} p95={result['p95']:.3f}"
    )


def print_trace(label, rows):
    print(f"\n[{label}] samples={len(rows)}")
    for field in (
        "captureHz",
        "processingHz",
        "ipcPublishHz",
        "mainMissionControlLoopHz",
        "mjpegOutputHz",
        "frameAgeMs",
        "captureMs",
        "lineDetectionMs",
        "greenMaskMs",
        "greenContoursAndFiltersMs",
        "topologyMs",
        "greenProcessingMs",
        "overlayMs",
        "mjpegPublishMs",
        "ipcPublishMs",
        "totalVisionMs",
    ):
        print_distribution(
            field,
            numeric(rows, field, positive_only=field.endswith("Hz")),
        )


def main():
    parser = argparse.ArgumentParser(
        description="Compara traces A/B/C do seguidor de linha."
    )
    parser.add_argument("--a", help="CSV do modo A")
    parser.add_argument("--b", help="CSV do modo B")
    parser.add_argument("--c", help="CSV do modo C")
    arguments = parser.parse_args()
    traces = [
        (label, path)
        for label, path in (("A", arguments.a), ("B", arguments.b), ("C", arguments.c))
        if path
    ]
    if not traces:
        parser.error("informe ao menos um arquivo com --a, --b ou --c")

    summaries = {}
    for label, path in traces:
        rows = read_trace(path)
        print_trace(label, rows)
        summaries[label] = summarize(rows)

    print("\nmetric," + ",".join(label for label, _path in traces))
    for metric in next(iter(summaries.values())):
        print(
            metric + "," + ",".join(
                str(summaries[label][metric]) for label, _path in traces
            )
        )


if __name__ == "__main__":
    main()
