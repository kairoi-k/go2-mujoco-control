"""Run Isaac Lab's RSL-RL train/play after registering this package's gym ids."""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


def isaaclab_root() -> Path:
    env = os.environ.get("ISAACLAB_PATH")
    if env:
        return Path(env).expanduser().resolve()
    try:
        import isaaclab
    except ImportError as exc:
        raise SystemExit(
            "Isaac Lab is not importable. Activate the Isaac Lab Python env and "
            "set ISAACLAB_PATH to the checkout root (the directory that contains scripts/)."
        ) from exc
    # IsaacLab/source/isaaclab/isaaclab/__init__.py → checkout root is parents[3]
    return Path(isaaclab.__file__).resolve().parents[3]


def launch_isaaclab_script(name: str) -> None:
    import go2_velocity_fast.tasks  # noqa: F401

    root = isaaclab_root()
    script = root / "scripts" / "reinforcement_learning" / "rsl_rl" / name
    if not script.is_file():
        raise SystemExit(
            f"Isaac Lab script not found: {script}\n"
            "Set ISAACLAB_PATH to the Isaac Lab checkout root."
        )
    sys.path.insert(0, str(script.parent))
    runpy.run_path(str(script), run_name="__main__")
