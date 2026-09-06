"""Mantém o comando histórico do painel de deploy."""

from pathlib import Path
import runpy


runpy.run_path(
    Path(__file__).resolve().parents[1] / "deployment" / "deploy_panel.py",
    run_name="__main__",
)
