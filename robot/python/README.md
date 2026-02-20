# APWCR Python Runtime

This directory contains the high-level robot software that runs on a laptop or Raspberry Pi.
It handles perception, high-level control logic, GUI, and serial communication with the Arduino firmware.

## Runtime Responsibilities
- Camera capture and YOLO inference
- Target selection and detection stability filtering
- Manual/autonomous controller logic
- Serial command + telemetry exchange with Arduino
- Flask web dashboard and control endpoints

## Package Layout
```text
robot/python/
|-- pwc_robot/
|   |-- main.py                    # runtime assembly + main loop
|   |-- config_loader.py           # load and validate YAML config
|   |-- perception/
|   |   |-- camera.py
|   |   |-- detector.py
|   |   |-- computer_vision.py
|   |   `-- ground_plane.py
|   |-- controller/
|   |   |-- controller.py
|   |   |-- commands.py
|   |   `-- states.py
|   |-- comms/
|   |   |-- serial_link.py
|   |   |-- protocol.py
|   |   |-- ports.py
|   |   `-- types.py
|   |-- gui/
|   |   |-- gui_server.py
|   |   |-- templates/gui.html
|   |   `-- static/
|   |       |-- css/gui.css
|   |       `-- js/gui.js
|   `-- utils/rate.py
|-- scripts/
|   |-- run_robot.py               # primary entry point
|   `-- debug_serial_rx.py
|-- requirements.txt
|-- requirements-cuda.txt
`-- README.md
```

## Configuration
Primary config file:
- `robot/config/robot_default.yaml`

Secondary/testing config:
- `robot/config/robot_test.yaml`

Important config sections:
- `camera`
- `detector`
- `comp_vision`
- `ground_plane`
- `controller`
- `comms`
- `gui`

`config_loader.py` resolves model paths relative to repository root and validates required keys.

## Main Execution Flow
1. `scripts/run_robot.py` imports `pwc_robot.main.main` and calls:
   - `main(config_name="robot_default.yaml")`
2. `main.py` creates and starts:
   - `Camera`
   - `Detector`
   - `ComputerVision`
   - `Controller`
   - `SerialLink` (if enabled)
   - Flask GUI thread (if enabled)
3. Loop scheduling uses `Rate` objects for:
   - Vision update rate
   - Controller update rate
   - Comms TX rate

## GUI Endpoints
Provided by `pwc_robot/gui/gui_server.py`:
- `GET /` dashboard page
- `GET /stream/comp_vision` MJPEG annotated stream
- `GET /perception/status` perception status JSON
- `GET /controller/status` controller status JSON
- `GET /telemetry/status` serial/telemetry JSON
- `POST /controller/mode` set manual/auto mode
- `POST /controller/manual_cmd` send manual drive/mechanism commands

## Installation

### Prerequisites
- Python 3.10+
- Camera accessible by OpenCV
- Optional: Arduino connected over USB serial

### Create Virtual Environment
From `robot/python`:
```bash
python -m venv .venv
```

Activate it.

Windows PowerShell:
```powershell
.\.venv\Scripts\Activate.ps1
```

macOS/Linux:
```bash
source .venv/bin/activate
```

### Install Dependencies
CPU/default:
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

CUDA optional path:
```bash
pip install -r requirements-cuda.txt
pip install -r requirements.txt
```

## Running

From `robot/python` with venv active:
```bash
python scripts/run_robot.py
```

The launcher prints discovered LAN URLs.
Default GUI port is configured in YAML (`gui.port`, default `5000`).

## Typical First-Run Checklist
- Camera:
  - Set `camera.index` correctly
  - Optionally tune resolution and capture rate
- Model:
  - Confirm `detector.model_path` exists
- Comms:
  - Set `comms.comms_enabled`
  - Set `comms.port` (or enable `auto_detect`)
  - Ensure `comms.baud` matches Arduino firmware
- GUI:
  - Keep `gui.enabled: true` for dashboard access

## Notes
- Most tuning should happen in `robot/config/robot_default.yaml`, not in source code.
- If no serial link is available, set `comms.comms_enabled: false` to run perception/GUI without Arduino comms.
- `test/` contains stand-alone scripts for isolated CV and subsystem testing.
