# Autonomous Pet Waste Collection Robot (APWCR) Software

This repository contains the software stack for the APWCR senior design project:
- High-level robot runtime (Python on laptop/Raspberry Pi)
- Low-level embedded firmware (Arduino Mega 2560)
- CV training notebooks and isolated subsystem tests

## Electrical Layout
![Electrical Component and Wiring Layout](images/APWCR_Wire_Layout_MEGA+Pi5_V2.png)

## Repository Layout
```text
APWCR_Software/
|-- cv_training/           # model training notebooks (not runtime)
|-- electrical_hardware/   # wiring diagrams and pin maps
|-- images/                # README/report images
|-- robot/
|   |-- config/            # YAML runtime configs
|   |-- cv_models/         # YOLO model weights
|   |-- python/            # high-level runtime (vision, control, GUI, comms)
|   |-- apwcr_firmware/    # PlatformIO Arduino firmware
|   `-- arduino/           # legacy Arduino sketch version
|-- test/                  # stand-alone testing scripts
`-- README.md
```

## How The System Works
1. `robot/python/scripts/run_robot.py` starts the runtime using `robot_default.yaml`.
2. `robot/python/pwc_robot/main.py` loads config and initializes:
   - Camera
   - YOLO detector
   - Computer vision pipeline
   - Controller (manual + autonomous phases)
   - Serial link to Arduino
   - Flask GUI server
3. Main loop timing:
   - Vision tick updates detections/target selection
   - Controller tick generates drive/mechanism commands
   - Serial RX/TX ticks exchange telemetry and commands with Arduino
4. Arduino firmware (`robot/apwcr_firmware/src/main.cpp`) receives commands, updates actuators/sensors, and transmits telemetry.
5. Flask GUI provides:
   - Live MJPEG vision stream
   - Perception status
   - Controller status
   - Serial/telemetry status
   - Manual control endpoints

## Quick Setup

### Prerequisites
- `git`
- `python` 3.10+
- (Firmware) PlatformIO CLI or PlatformIO IDE extension

### 1. Clone
```bash
git clone <REPO_URL>
cd APWCR_Software
```

### 2. Create And Activate A Python Virtual Environment
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

### 3. Install Python Dependencies
CPU/default:
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

NVIDIA CUDA systems (optional):
```bash
pip install -r requirements-cuda.txt
pip install -r requirements.txt
```

### 4. Configure Runtime
Edit `robot/config/robot_default.yaml` for your hardware:
- `camera.index` (camera device)
- `detector.model_path` (YOLO weights path)
- `comms.comms_enabled`
- `comms.port`
- `comms.baud`

### 5. Run Robot Software
From `robot/python` with venv active:
```bash
python scripts/run_robot.py
```

GUI URLs:
- `http://localhost:5000`
- `http://<robot_ip>:5000`

## Firmware Setup (Arduino Mega, PlatformIO)
From repo root:
```bash
cd robot/apwcr_firmware
pio run
pio run -t upload
pio device monitor -b 230400
```

Keep serial settings aligned:
- Firmware baud in `robot/apwcr_firmware/platformio.ini` / params headers
- Python baud in `robot/config/robot_default.yaml` (`comms.baud`)

## Additional References
- Python runtime details: `robot/python/README.md`
- Firmware entry point: `robot/apwcr_firmware/src/main.cpp`
- CV model testing scripts: `test/cv_model_testing/`
