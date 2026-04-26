# Autonomous Pet Waste Collection Robot (APWCR) Software

This repository contains the software and supporting assets for the APWCR senior design project. The codebase is split between:

- A high-level Python runtime that runs perception, autonomy logic, the operator GUI, and host-side serial communication.
- Embedded Arduino Mega firmware that executes deterministic motor, mechanism, sensor, and telemetry tasks.
- Supporting CV training artifacts, hardware references, and standalone test utilities.

The goal of this README is to give a future team enough context to understand the architecture, set the project up on a new machine, and run the robot safely.

## System Overview

At a high level, the robot is organized as a two-layer system:

1. The host computer runs Python code for camera capture, YOLO inference, target selection, high-level controller state transitions, and the Flask-based dashboard.
2. The Arduino Mega runs the low-level control loop for drive motors, mechanism motors, servos, the ultrasonic sensor, and serial telemetry.
3. The two layers communicate over newline-delimited JSON over USB serial.

Current startup path:

1. `robot/python/scripts/run_robot.py` prints local access URLs and launches the Python runtime with `robot_default.yaml`.
2. `robot/python/pwc_robot/main.py` loads config, opens the camera, loads the YOLO model, starts the controller, optionally connects to the Arduino, and optionally starts the GUI server.
3. The runtime main loop updates perception and controller state.
4. A dedicated comms worker thread performs serial RX continuously and transmits the latest drive/mechanism commands at the configured comms rate.
5. `robot/firmware/apwcr_firmware_v2/src/main.cpp` receives commands, updates subsystems, applies timeout safety behavior, and transmits telemetry back to Python.

## Repository Layout

```text
APWCR_Software/
|-- cv_training/                    # model-training notebooks and experiments
|-- electrical_hardware/            # wiring references, pin maps, hardware notes
|-- images/                         # README/report figures
|-- robot/
|   |-- config/                     # YAML runtime and test configurations
|   |-- cv_models/                  # YOLO weights and stored training results
|   |-- data/                       # generated control-test logs
|   |-- firmware/
|   |   |-- apwcr_firmware_v2/      # active Arduino Mega firmware
|   |   |-- apwcr_firmware/         # older firmware snapshot
|   |   `-- feedforward_firmware/   # motor characterization firmware
|   `-- python/                     # host runtime, GUI, control, perception, comms
|-- test/                           # standalone experiments and test scripts
`-- README.md
```

## Electrical Layout

![Electrical Component and Wiring Layout](images/APWCR_Wire_Layout_MEGA+Pi5_V3.png)

## Architecture By Subsystem

### Python host runtime

Primary package: `robot/python/pwc_robot/`

- `main.py`
  Coordinates startup, configuration, subsystem construction, threading, and the main loop.
- `config_loader.py`
  Loads YAML from `robot/config/` and resolves model paths relative to the repository root.
- `perception/`
  Contains camera capture, YOLO detection, annotated-frame generation, target selection, and optional ground-plane range estimation.
- `controller/`
  Implements manual mode and the current autonomous phases:
  `AUTO_SEARCHING`, `AUTO_APPROACHING`, `AUTO_PICKUP`, and `AUTO_DEPOSIT`.
- `comms/`
  Encodes/decodes the serial protocol, tracks link health, and stores the latest telemetry.
- `gui/`
  Hosts the Flask dashboard, MJPEG stream, controller endpoints, telemetry endpoints, and control-test tooling.

### Firmware

Active project: `robot/firmware/apwcr_firmware_v2/`

- `src/main.cpp`
  Owns startup and the top-level scheduler.
- `src/comms/`
  Serial command and telemetry handling.
- `src/control/`
  Drive and mechanism controllers, PID helpers, and stall protection.
- `src/actuators/`
  DC motor and servo wrappers.
- `src/sensors/`
  Encoder and ultrasonic sensor interfaces.
- `include/Pins.h`
  Hardware pin mapping.
- `include/Params.h`
  Tunable constants for geometry, feedforward, PID, safety, and timing.

### Characterization and testing path

The project also includes a dedicated motor feedforward characterization workflow:

- Firmware: `robot/firmware/feedforward_firmware/`
- Python runner: `robot/python/feedforward_model_test/`
- Config: `robot/config/robot_ff_model_test.yaml`
- Raw CSV output: `robot/python/feedforward_model_test/data/raw/`

This workflow is separate from the normal robot runtime and is used to gather data for the feedforward constants currently stored in firmware parameter headers.

## Main Runtime Flow

When the normal robot stack is running, the control/data flow is:

1. `Camera` captures frames.
2. `Detector` runs YOLO inference on those frames.
3. `ComputerVision` selects a target, tracks stability, and optionally estimates ground-plane range.
4. `Controller` decides drive and mechanism commands based on either:
   manual GUI input, or
   the current autonomous phase plus perception and telemetry.
5. `SerialLink` sends commands to the Arduino and receives telemetry back.
6. The GUI exposes the live annotated stream plus controller/perception/telemetry status.

## Configuration Files

Current config files under `robot/config/`:

- `robot_default.yaml`
  Main runtime configuration for the Python robot stack.
- `robot_ff_model_test.yaml`
  Separate configuration for the feedforward motor sweep tooling.

Important sections in `robot_default.yaml`:

- `camera`
  Camera index, resolution, and capture behavior.
- `detector`
  Model path, input size, and detection confidence threshold.
- `comp_vision`
  Inference/update rate and target-selection policy.
- `ground_plane`
  Camera calibration and range estimation settings.
- `controller`
  Manual/autonomous gains, speed limits, target hold timing, and ultrasonic safety settings.
- `comms`
  Serial port, baud rate, timeouts, reconnect behavior, and TX/RX rates.
- `gui`
  Dashboard host/port, stream rate, manual jog values, and control-test defaults.

## First-Time Setup

### Prerequisites

- `git`
- Python 3.10 or newer
- A camera supported by OpenCV
- PlatformIO CLI or the PlatformIO IDE extension for firmware work
- Arduino Mega 2560 for full hardware testing

### Clone the repository

```bash
git clone <REPO_URL>
cd APWCR_Software
```

### Create a Python virtual environment

```bash
cd robot/python
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

### Install Python dependencies

Default / CPU-oriented install:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

Optional CUDA/PyTorch index setup:

```bash
pip install -r requirements-cuda.txt
pip install -r requirements.txt
```

Current Python package list includes:

- `ultralytics`
- `opencv-python`
- `pyyaml`
- `pyserial`
- `numpy`
- `flask`

## Running the Robot Software

From `robot/python` with the virtual environment active:

```bash
python scripts/run_robot.py
```

The launcher prints LAN URLs and the Flask server also prints:

- `http://localhost:5000`
- `http://<local_ip>:5000`

### Before the first run

Review `robot/config/robot_default.yaml` and confirm:

- `camera.index` matches the camera device on the current machine.
- `detector.model_path` points to a valid model file in `robot/cv_models/`.
- `comms.comms_enabled` is set appropriately.
- `comms.port` and `comms.baud` match the Arduino connection.
- `gui.enabled` is set to `true` if you want the dashboard.

### Running without hardware

If the Arduino is not connected, set:

```yaml
comms:
  comms_enabled: false
```

That allows perception and GUI work without serial communication.

## GUI and Operator Features

The dashboard is defined in `robot/python/pwc_robot/gui/` and currently provides:

- A live MJPEG annotated perception stream
- Perception status JSON
- Controller status JSON
- Telemetry and link-health JSON
- Manual mode / auto mode switching
- Manual drive commands
- Manual mechanism commands for servos and mechanism motors
- Control-test start/stop/download actions for drivetrain and mechanism experiments

Control-test CSV files are saved under:

- `robot/data/control_tests/`

## Firmware Build and Upload

Active firmware project:

- `robot/firmware/apwcr_firmware_v2`

Typical workflow:

```bash
cd robot/firmware/apwcr_firmware_v2
pio run
pio run -t upload
pio device monitor -b 250000
```

Key serial settings that must stay aligned:

- `robot/firmware/apwcr_firmware_v2/include/Params.h`
  `SERIAL_BAUD`
- `robot/firmware/apwcr_firmware_v2/platformio.ini`
  `monitor_speed`
- `robot/config/robot_default.yaml`
  `comms.baud`

The active firmware uses:

- `Servo`
- `ArduinoJson`

## Other Firmware Projects

- `robot/firmware/apwcr_firmware/`
  Older firmware snapshot kept for reference.
- `robot/firmware/feedforward_firmware/`
  Separate firmware used only for motor characterization and sweep testing.

Future teams should treat `apwcr_firmware_v2` as the deployed baseline unless there is a deliberate reason to branch from an older snapshot.

## Feedforward Characterization Workflow

The dedicated feedforward test runner is launched from the Python side and talks to the separate `feedforward_firmware` project.

Relevant files:

- `robot/python/feedforward_model_test/main.py`
- `robot/config/robot_ff_model_test.yaml`
- `robot/firmware/feedforward_firmware/`

This tooling:

1. Opens a serial link to the characterization firmware.
2. Sweeps one motor channel through configured duty commands.
3. Logs returned RPM telemetry to CSV.
4. Produces data used to tune feedforward constants in firmware parameter headers.

## Important Files For New Teams

If a future team only reads a small number of files first, start here:

- `README.md`
  Repository-level onboarding and architecture.
- `robot/python/README.md`
  Host runtime details, GUI endpoints, and run workflow.
- `robot/firmware/README.md`
  Firmware project guide and subsystem mapping.
- `robot/config/robot_default.yaml`
  Day-to-day runtime tuning.
- `robot/firmware/apwcr_firmware_v2/include/Params.h`
  Core low-level tuning constants and timing.
- `robot/firmware/apwcr_firmware_v2/include/Pins.h`
  Hardware pin assignments.

## Known Practical Notes

- The Python runtime assumes relative paths are resolved from the repository structure, not from an installed package layout.
- The active autonomy path is strongest in search and approach behavior; pickup and deposit remain placeholder states in the current controller.
- GUI control tests and feedforward sweeps are separate workflows and use different firmware/runtime combinations.
- `robot/config/robot_default.yaml` is the main tuning surface for host-side behavior.

## Additional Documentation

- Python runtime guide: `robot/python/README.md`
- Firmware guide: `robot/firmware/README.md`

