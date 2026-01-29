from pathlib import Path
import sys

# Add robot/python to sys.path so 'pwc_robot' can be imported when running this script directly
THIS_DIR = Path(__file__).resolve().parent
PY_ROOT = THIS_DIR.parent  # robot/python
sys.path.insert(0, str(PY_ROOT))

from pwc_robot.main import main


if __name__ == "__main__":
    # Change this if you want a different config file
    main(config_name="robot_default.yaml")
