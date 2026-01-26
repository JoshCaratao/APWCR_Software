from pathlib import Path
import yaml


def get_project_root() -> Path:
    """
    Returns the repository root assuming this file lives at:
    <repo>/robot/python/pwc_robot/config_loader.py
    """
    return Path(__file__).resolve().parents[3]

def load_config(config_name: str = "robot.yaml") -> dict:
    """
    Loads a YAML config from <repo>/robot/config/<config_name>.

    Returns:
        config (dict): parsed YAML
    """
    project_root = get_project_root()
    config_path = project_root / "robot" / "config" / config_name

    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict):
        raise ValueError(f"Config file did not load as a dict: {config_path}")

    return config

def resolve_paths(config: dict) -> dict:
    """
    Converts important relative paths in the config into absolute paths.
    Mutates a copy of the dict and returns it.
    """
    project_root = get_project_root()
    cfg = dict(config)  # shallow copy

    detector = dict(cfg.get("detector", {}))
    model_path = detector.get("model_path")

    if model_path is not None:
        detector["model_path"] = str((project_root / model_path).resolve())

    cfg["detector"] = detector
    return cfg

def require_keys(config: dict, required: dict) -> None:
    """
    required format:
      {
        "camera": ["index", "width", "height"],
        "detector": ["model_path", "imgsz", "conf"]
      }
    """
    for section, keys in required.items():
        if section not in config:
            raise KeyError(f"Missing config section: '{section}'")
        for k in keys:
            if k not in config[section]:
                raise KeyError(f"Missing config key: '{section}.{k}'")
