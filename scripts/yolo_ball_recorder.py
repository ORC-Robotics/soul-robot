"""Recorder exclusivo das imagens usadas para treinar o detector YOLO."""

import dataset_recorder as _core
from dataset_recorder import DatasetRecorder, YOLO_BALL_DATASET_ROOT


class YoloBallRecorder(DatasetRecorder):
    """Salva somente frames YOLO crus, separados por câmera e sessão."""

    def __init__(self, camera_role):
        super().__init__(camera_role, "yolo_ball", _core.YOLO_BALL_DATASET_ROOT)


__all__ = ("YOLO_BALL_DATASET_ROOT", "YoloBallRecorder")
