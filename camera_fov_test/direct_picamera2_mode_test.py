"""Mantém o comando histórico do diagnóstico direto da câmera."""

from pathlib import Path
import runpy


runpy.run_path(
    Path(__file__).resolve().parents[1]
    / "tools"
    / "camera"
    / "direct_picamera2_mode_test.py",
    run_name="__main__",
)
