"""Gera diagnóstico de componentes, scanlines, centros e larguras sem tocar no robô."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from line_calibration import segment


def title_panel(frame, title):
    """Reserva espaço para a legenda sem escrever sobre pixels de diagnóstico."""
    panel = cv2.copyMakeBorder(frame, 32, 0, 0, 0, cv2.BORDER_CONSTANT,
                              value=(245, 245, 245))
    cv2.putText(panel, title, (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (20, 20, 20), 1, cv2.LINE_AA)
    return panel


def render(experiment, sample_id, output):
    config = json.loads((experiment / "config.json").read_text(encoding="utf-8"))
    records = json.loads((experiment / "frames.json").read_text(encoding="utf-8"))
    record = next(r for r in records if r["sample_id"] == sample_id)
    raw = cv2.imread(str(Path(config["dataset"]) / "raw" / (sample_id + ".png")))
    mask = cv2.imread(str(experiment / "mask" / (sample_id + ".png")), cv2.IMREAD_GRAYSCALE)
    structural, replay_mask = segment(raw, config["segmentation"])
    if not np.array_equal(mask, replay_mask):
        raise ValueError("O pipeline mudou: a máscara atual não reproduz este experimento.")
    annotation = config["annotation"]
    reference = raw.copy()
    expected = np.zeros(mask.shape, np.uint8)
    region = np.zeros(mask.shape, np.uint8)
    polygons = [np.asarray(p, np.int32) for p in annotation["line_polygons"]]
    for polygon in polygons:
        cv2.fillPoly(expected, [polygon], 255)
    for polygon in annotation["evaluation_polygons"]:
        cv2.fillPoly(region, [np.asarray(polygon, np.int32)], 255)
    cv2.polylines(reference, polygons, True, (0, 255, 255), 1)
    margin = annotation["boundary_uncertainty_px"]
    kernel = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)
    positive = (cv2.erode(expected, kernel) > 0) & (region > 0)
    negative = (cv2.dilate(expected, kernel) == 0) & (region > 0)
    components = (raw.astype(float) * 0.35).astype(np.uint8)
    components[(structural > 0) & (mask == 0)] = (255, 0, 255)
    components[(mask > 0) & (expected > 0)] = (0, 200, 0)
    components[(mask > 0) & negative] = (0, 0, 255)
    components[(mask == 0) & positive] = (255, 255, 0)
    scanlines = raw.copy()
    count, labels = cv2.connectedComponents(mask)
    overlaps = np.bincount(labels[positive], minlength=count)
    overlaps[0] = 0
    selected = np.isin(labels, np.flatnonzero(overlaps)) & (mask > 0) & (region > 0)
    # O espaçamento é apenas visual. As métricas usam todas as linhas anotadas,
    # e o alvo mostrado abaixo continua sendo o alvo original do Fusion.
    spacing = max(1, raw.shape[0] // 9)
    for index, y in enumerate(record["row_ys"]):
        if y % spacing:
            continue
        xs = np.flatnonzero(selected[y])
        if not xs.size:
            continue
        center = record["row_centers_px"][index]
        width = record["widths_px"][index]
        cv2.line(scanlines, (int(xs[0]), y), (int(xs[-1]), y), (0, 255, 255), 1)
        if center is not None:
            cv2.circle(scanlines, (int(round(center)), y), 3, (255, 0, 255), -1)
            cv2.putText(scanlines, f"w={width} c={center:.1f}", (5, y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1, cv2.LINE_AA)
    target = record["trajectory"].get("farPoint")
    near = record["trajectory"].get("nearPoint")
    if target and near:
        near_xy = (near["x"], near["y"])
        target_xy = (target["x"], target["y"])
        cv2.arrowedLine(scanlines, near_xy, target_xy, (0, 0, 255), 2, tipLength=0.04)
    binary = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    panels = [title_panel(reference, "RGB | yellow: manual tape boundary"),
              title_panel(binary, "Binary mask | no overlays"),
              title_panel(components, "green: tape; red: FP; magenta: rejected"),
              title_panel(scanlines, "scanlines: width/center | red: Fusion target")]
    mosaic = np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), mosaic):
        raise OSError("Não foi possível gravar o diagnóstico.")
    print(str(output))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--sample", default="0000")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    render(args.experiment, args.sample, args.output)


if __name__ == "__main__":
    main()
