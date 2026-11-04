"""Milestone 3 Integrated Tiltrotor Analysis Package (Team 9)."""

import json
from pathlib import Path
from typing import Any, Dict


def load_config(path: str | Path) -> Dict[str, Any]:
    """Load YAML or JSON configuration file without hard-coded parameters."""
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
        return yaml.safe_load(text)
    except Exception:
        return json.loads(text)
