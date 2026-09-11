"""Recorder exclusivo das imagens do classificador de faixa prata."""

import time

import dataset_recorder as _core
from dataset_recorder import (
    ACTIVE_STATUS_TIMEOUT_SECONDS,
    CONTROL_PATH,
    DATASET_ROOT,
    STATUS_PATH,
    DatasetRecorder,
)


def _sync_paths():
    """Mantém os caminhos legados sincronizados com o módulo compartilhado."""

    _core.CONTROL_PATH = CONTROL_PATH
    _core.STATUS_PATH = STATUS_PATH
    _core.DATASET_ROOT = DATASET_ROOT


def empty_dataset_status():
    """Retorna o formato de status usado pelo dashboard existente."""

    return _core.empty_dataset_status()


def read_dataset_status():
    """Lê o status usando os caminhos configuráveis do módulo legado."""

    _sync_paths()
    _core.time = time
    return _core.read_dataset_status()


class SilverDatasetRecorder(DatasetRecorder):
    """Salva somente amostras destinadas ao classificador Silver."""

    def __init__(self, camera_role):
        _sync_paths()
        super().__init__(camera_role, "silver_classifier", DATASET_ROOT)


__all__ = (
    "ACTIVE_STATUS_TIMEOUT_SECONDS",
    "CONTROL_PATH",
    "DATASET_ROOT",
    "STATUS_PATH",
    "SilverDatasetRecorder",
    "empty_dataset_status",
    "read_dataset_status",
)
