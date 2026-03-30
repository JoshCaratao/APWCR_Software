from pathlib import Path
import sys

# Add robot/python to sys.path so the test package can be imported
THIS_DIR = Path(__file__).resolve().parent
PY_ROOT = THIS_DIR.parent  # robot/python
sys.path.insert(0, str(PY_ROOT))

from feedforward_model_test.main import main


if __name__ == "__main__":
    main(config_name="robot_ff_model_test.yaml")
