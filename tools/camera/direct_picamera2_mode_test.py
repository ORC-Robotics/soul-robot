"""Compara modos do IMX219 sem usar dashboard nem pipeline de visão."""

import json
import os
import pprint
import time

from PIL import Image
from libcamera import Transform
from picamera2 import Picamera2


CAMERA_INDEX = 0
DISCARD_FRAME_COUNT = 30
OUTPUT_DIR = "/tmp/obr_picamera2_direct_test"
PIXEL_FORMAT = "RGB888"


def json_value(value):
    """Converte objetos do Picamera2 em valores simples para o relatório."""

    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def capture_mode(picam2, camera_transform, name, main_size, sensor_size, fps):
    """Reconfigura a mesma câmera, estabiliza o modo e salva um frame direto."""

    picam2.stop()
    frame_duration_us = int(1_000_000 / fps)
    requested_configuration = picam2.create_video_configuration(
        main={"size": main_size, "format": PIXEL_FORMAT},
        sensor={"output_size": sensor_size, "bit_depth": 10},
        controls={
            "FrameDurationLimits": (frame_duration_us, frame_duration_us),
        },
        transform=camera_transform,
        buffer_count=4,
    )
    picam2.configure(requested_configuration)
    applied_configuration = picam2.camera_configuration()
    picam2.start()

    # O descarte permite que exposição, balanço de branco e buffers estabilizem
    # depois de cada troca de modo físico do sensor.
    for _ in range(DISCARD_FRAME_COUNT):
        picam2.capture_array("main")

    frame = picam2.capture_array("main")
    metadata = picam2.capture_metadata()
    png_path = os.path.join(OUTPUT_DIR, name + ".png")
    Image.fromarray(frame, mode="RGB").save(png_path, format="PNG")

    result = {
        "name": name,
        "pngPath": png_path,
        "frameShape": list(frame.shape),
        "frameDtype": str(frame.dtype),
        "appliedConfiguration": json_value(applied_configuration),
        "ScalerCrop": json_value(metadata.get("ScalerCrop")),
        "SensorTimestamp": json_value(metadata.get("SensorTimestamp")),
        "FrameDuration": json_value(metadata.get("FrameDuration")),
    }
    print(f"\n=== {name} ===")
    print("Configuração aplicada:")
    print(pprint.pformat(applied_configuration, sort_dicts=False))
    print(f"ScalerCrop: {metadata.get('ScalerCrop')}")
    print(f"Frame salvo: {png_path}")
    return result


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    camera_transform = Transform(hflip=True, vflip=True)
    picam2 = Picamera2(CAMERA_INDEX)

    camera_id = str(picam2.camera.id)
    camera_model = str(picam2.camera_properties.get("Model", ""))
    print(f"Índice físico selecionado: {CAMERA_INDEX}")
    print(f"ID físico selecionado: {camera_id}")
    print(f"Modelo: {camera_model}")
    print(f"Resolução do sensor: {picam2.sensor_resolution}")
    print("Transformação usada em todos os modos: hvflip")

    # A instância precisa estar iniciada para que a primeira troca também siga
    # exatamente a sequência stop, configure e start usada entre os modos.
    initial_configuration = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": PIXEL_FORMAT},
        transform=camera_transform,
    )
    picam2.configure(initial_configuration)
    picam2.start()

    results = []
    try:
        results.append(
            capture_mode(
                picam2,
                camera_transform,
                "main_960x540_sensor_1920x1080",
                (960, 540),
                (1920, 1080),
                30.0,
            )
        )
        results.append(
            capture_mode(
                picam2,
                camera_transform,
                "main_640x480_sensor_1640x1232",
                (640, 480),
                (1640, 1232),
                30.0,
            )
        )
        results.append(
            capture_mode(
                picam2,
                camera_transform,
                "main_3280x2464_sensor_3280x2464",
                (3280, 2464),
                (3280, 2464),
                21.0,
            )
        )
    finally:
        picam2.stop()
        picam2.close()

    report = {
        "cameraIndex": CAMERA_INDEX,
        "cameraId": camera_id,
        "cameraModel": camera_model,
        "sensorResolution": list(picam2.sensor_resolution),
        "transform": "hvflip",
        "discardedFramesPerMode": DISCARD_FRAME_COUNT,
        "captures": results,
    }
    report_path = os.path.join(OUTPUT_DIR, "report.json")
    with open(report_path, "w", encoding="utf-8") as report_file:
        json.dump(report, report_file, indent=2, ensure_ascii=False)
    print(f"\nRelatório salvo: {report_path}")


if __name__ == "__main__":
    main()
