"""Mantém o comando histórico do diagnóstico de desempenho."""

from pathlib import Path
import runpy


runpy.run_path(
    Path(__file__).resolve().parents[1]
    / "tools"
    / "diagnostics"
    / "performance_baseline.py",
    run_name="__main__",
)
