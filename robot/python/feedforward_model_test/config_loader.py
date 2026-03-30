from __future__ import annotations

from pathlib import Path

import yaml


def get_project_root() -> Path:
    """
    Returns the repository root assuming this file lives at:
    <repo>/robot/python/feedforward_model_test/config_loader.py
    """
    return Path(__file__).resolve().parents[3]


def load_config(config_name: str = "robot_ff_model_test.yaml") -> dict:
    """
    Load the feedforward model test YAML config from robot/config.
    """
    project_root = get_project_root()
    config_path = project_root / "robot" / "config" / config_name

    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if not isinstance(cfg, dict):
        raise ValueError(f"Config file did not load as a dict: {config_path}")

    return cfg


def require_keys(config: dict, required: dict[str, list[str]]) -> None:
    """
    Verify that required top-level sections and keys exist.

    Example:
      {
        "comms": ["port", "baud"],
        "test": ["motor_under_test", "command_mode"],
      }
    """
    for section, keys in required.items():
        if section not in config:
            raise KeyError(f"Missing config section: '{section}'")

        section_obj = config[section]
        if not isinstance(section_obj, dict):
            raise TypeError(f"Config section '{section}' must be a dict")

        for key in keys:
            if key not in section_obj:
                raise KeyError(f"Missing config key: '{section}.{key}'")
