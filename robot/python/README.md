# APWCR Python Runtime

This directory contains the host-side robot software. It is designed to run on a laptop or Raspberry Pi and acts as the high-level brain for perception, autonomy state transitions, the operator dashboard, and communication with the Arduino Mega.

## What This Runtime Owns

- Camera capture with OpenCV
- YOLO model loading and inference
- Detection ranking, target stability tracking, and annotated video output
- Optional ground-plane distance estimation from image coordinates
- Manual and autonomous high-level controller logic
- Host-side serial communication with the Arduino firmware
- Flask dashboard, API endpoints, and control-test tooling

## Package Layout

```text
robot/python/
|-- pwc_robot/
|   |-- main.py
|   |-- config_loader.py
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
|   |   |-- control_test_runner.py
|   |   |-- templates/gui.html
|   |   `-- static/
|   |       |-- css/gui.css
|   |       `-- js/gui.js
|   `-- utils/rate.py
|-- scripts/
|   |-- run_robot.py
|   |-- debug_serial_rx.py
|   `-- run_feedforward_model_test.py
|-- feedforward_model_test/
|   |-- main.py
|   |-- sweep_runner.py
|   |-- sweep_plan.py
|   |-- serial_link.py
|   `-- csv_logger.py
|-- requirements.txt
|-- requirements-cuda.txt
`-- README.md
```

## Key Files And What They Do

### `scripts/run_robot.py`

Simple launcher for the normal robot stack. It:

- Adds `robot/python` to `sys.path`
- Prints local/LAN GUI URLs
- Calls `pwc_robot.main.main(config_name="robot_default.yaml")`

### `pwc_robot/main.py`

Main orchestration file for normal robot operation. It:

- Loads and validates YAML configuration
- Creates camera, detector, ground-plane calibration, and computer-vision objects
- Creates the controller
- Starts the serial link if comms are enabled
- Starts the Flask GUI in a daemon thread if enabled
- Runs the host main loop and coordinates perception, controller, and comms timing

### `pwc_robot/perception/`

Perception pipeline pieces:

- `camera.py`
  Camera capture wrapper.
- `detector.py`
  YOLO model interface.
- `computer_vision.py`
  Detection execution, target scoring, stability tracking, and annotated-frame generation.
- `ground_plane.py`
  Converts image-space information into approximate forward/lateral ground-plane distances when enabled.

### `pwc_robot/controller/`

High-level behavior selection:

- `states.py`
  Controller state enum.
- `commands.py`
  Drive and mechanism command types.
- `controller.py`
  Manual mode, autonomous phase logic, deadman behavior, and ultrasonic gating.

Current autonomous phases in code:

- `MANUAL`
- `AUTO_SEARCHING`
- `AUTO_APPROACHING`
- `AUTO_PICKUP`
- `AUTO_DEPOSIT`

At the moment, search and approach contain the most substantive behavior. Pickup and deposit are still effectively placeholders that return stop/no-op outputs.

### `pwc_robot/comms/`

Host-side serial link implementation:

- Port selection / management
- Command encoding
- Telemetry decoding
- RX/TX statistics and link health
- Reconnect behavior

### `pwc_robot/gui/`

Flask web dashboard and API endpoints. This is the main operator interface for:

- Viewing the annotated perception stream
- Checking controller status
- Checking serial link and telemetry health
- Switching between manual and autonomous modes
- Sending manual drive/mechanism commands
- Running control tests and downloading the latest CSV log

### `feedforward_model_test/`

Dedicated tooling for motor characterization. This is not part of the normal runtime loop. It exists so the team can gather sweep data against the separate `feedforward_firmware` firmware image.

## Configuration

Configuration files live in `robot/config/`.

### Normal runtime config

- `robot/config/robot_default.yaml`

Main sections:

- `camera`
- `detector`
- `comp_vision`
- `ground_plane`
- `gui`
- `controller`
- `comms`

### Feedforward test config

- `robot/config/robot_ff_model_test.yaml`

Used only by the characterization workflow. It defines:

- Serial settings for the dedicated test firmware
- Which motor is under test
- Sweep direction and command levels
- Command/logging rates
- CSV output settings

## Main Execution Flow

Normal robot execution looks like this:

1. `scripts/run_robot.py` launches the runtime with `robot_default.yaml`.
2. `config_loader.py` loads the YAML and resolves the YOLO model path relative to the repository root.
3. `Camera` starts capturing frames.
4. `Detector` loads the selected YOLO weights.
5. `ComputerVision` performs target detection and target selection.
6. `Controller` consumes the most recent vision observation and telemetry to generate drive and mechanism commands.
7. `CommsWorker` handles serial RX continuously and sends the latest command outputs at the configured communications rate.
8. The Flask GUI exposes both status and operator controls.

## GUI Endpoints

The Flask server is defined in `pwc_robot/gui/gui_server.py`.

Main endpoints:

- `GET /`
  Dashboard page.
- `GET /stream/comp_vision`
  MJPEG annotated video stream.
- `GET /perception/status`
  Perception status JSON.
- `GET /controller/status`
  Controller state and last command JSON.
- `GET /telemetry/status`
  Serial link status and latest telemetry JSON.
- `POST /controller/mode`
  Switch between `manual` and `auto`.
- `POST /controller/manual_cmd`
  Send manual drive and mechanism commands.
- `POST /control_test/start`
  Start a control test.
- `POST /control_test/stop`
  Stop the active control test.
- `GET /control_test/status`
  Get control-test status.
- `GET /control_test/download_latest`
  Download the most recent control-test CSV.

Control-test CSV outputs are saved under:

- `robot/data/control_tests/`

## Installation

### Prerequisites

- Python 3.10+
- Camera available to OpenCV
- Optional but recommended for full testing: Arduino Mega connected over USB serial

### Create a virtual environment

From `robot/python`:

```bash
python -m venv .venv
```

Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

macOS/Linux:

```bash
source .venv/bin/activate
```

### Install dependencies

Default install:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

Optional CUDA install path:

```bash
pip install -r requirements-cuda.txt
pip install -r requirements.txt
```

Current dependency files:

- `requirements.txt`
  `ultralytics`, `opencv-python`, `pyyaml`, `pyserial`, `numpy`, `flask`
- `requirements-cuda.txt`
  PyTorch index and CUDA-enabled `torch`, `torchvision`, `torchaudio`

## Running The Normal Robot Stack

From `robot/python` with the virtual environment active:

```bash
python scripts/run_robot.py
```

Before running, verify:

- `camera.index` matches the connected camera.
- `detector.model_path` points to a valid model file.
- `comms.port` points to the Arduino serial port, or set `auto_detect` as needed.
- `comms.baud` matches the firmware baud.
- `gui.enabled` is `true` if you want the dashboard.

## Running Without The Arduino

To work on perception or the GUI without the robot connected, set:

```yaml
comms:
  comms_enabled: false
```

That bypasses serial setup and lets the rest of the host stack run independently.

## Feedforward Characterization Workflow

The characterization path is separate from the main robot runtime.

Pieces involved:

- Python runner:
  `robot/python/feedforward_model_test/main.py`
- Config:
  `robot/config/robot_ff_model_test.yaml`
- Matching firmware:
  `robot/firmware/feedforward_firmware/`

Purpose:

1. Open a serial link to the dedicated test firmware.
2. Sweep one chosen motor through a configured set of duty commands.
3. Log RPM telemetry for each step.
4. Save raw CSV data for later fitting and parameter updates.

CSV output path is configured in the YAML and currently points to:

- `robot/python/feedforward_model_test/data/raw/`

## Files Future Teams Will Touch Most Often

- `robot/config/robot_default.yaml`
  Day-to-day host runtime tuning.
- `robot/config/robot_ff_model_test.yaml`
  Sweep-test configuration.
- `pwc_robot/main.py`
  Runtime assembly and timing.
- `pwc_robot/controller/controller.py`
  High-level behavior logic.
- `pwc_robot/gui/gui_server.py`
  Web UI routes and control-test endpoints.
- `pwc_robot/comms/serial_link.py`
  Host serial communication and telemetry handling.

## Practical Notes

- `config_loader.py` assumes the current repository layout and resolves the model path relative to the repo root.
- The GUI can be useful even when autonomy is disabled because it exposes live perception, link health, and control-test tools.
- If communication appears broken, check baud, COM port, and whether the correct firmware image is flashed.
- If the robot runtime starts but no detections appear, verify both the model path and the camera index first.

