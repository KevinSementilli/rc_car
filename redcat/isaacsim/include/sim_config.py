# SPDX-License-Identifier: BSD-3-Clause

"""Minimal YAML config loading for the Isaac Sim launch scripts, styled like ROS 2 parameter
YAML files: a single nested mapping, grouped by subsystem (lidar, diff_drive, teleop, ...).

No isaacsim/carb imports here, so this is safe to import before a SimulationApp exists.
"""

from pathlib import Path

import yaml


def load_config(path: str | Path, *, root_key: str = "isaacsim") -> dict:
    """Load a YAML config file and return the mapping under root_key.

    Raises FileNotFoundError if the file doesn't exist, and KeyError if root_key is missing --
    both fail loudly rather than silently falling back, since this file is meant to be the
    single source of truth for a launch script's configuration.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open() as f:
        data = yaml.safe_load(f) or {}

    if root_key not in data:
        raise KeyError(f"'{root_key}' key not found in {path}")

    return data[root_key]
