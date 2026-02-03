# APWCR Robot Software (Python)

This directory contains the **high-level Python software** for the Autonomous Pet Waste Collection Robot (APWCR).
It is responsible for perception, coordination, user interface, and high-level robot logic.

The architecture is intentionally lightweight and modular, borrowing concepts from ROS (node separation, clear interfaces) without requiring ROS itself.

This code is designed to run on:
- A **development laptop** (primary development and demo platform)
- A **single-board computer** such as a Raspberry Pi (deployment platform)

Low-level real-time motor control and sensor actuation are handled by an external microcontroller (Arduino), which communicates with this software over a serial interface.

---

## Directory Structure

```
robot/python/
├── pwc_robot/
│   ├── __init__.py
│   ├── main.py
│   ├── config_loader.py
│   │
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── camera.py
│   │   ├── detector.py
│   │   └── computer_vision.py
│   │
│   ├── gui/
│   │   ├── gui_server.py          # Flask app + endpoints (HTML, MJPEG stream, JSON status)
│   │   ├── templates/             # Flask templates (HTML)
│   │   │   └── gui.html
│   │   └── static/                # Static frontend assets served by Flask
│   │       ├── css/
│   │       │   └── gui.css
│   │       └── js/
│   │           └── gui.js
│   │
│   ├── utils/
│   │   ├── __init__.py
│   │   └── rate.py
│   │
│   └── (future packages)
│       ├── state/
│       ├── control/
│       └── comms/
│
├── scripts/
│   └── run_robot.py
│
├── requirements.txt
├── requirements-cuda.txt
└── README.md
```

---

## Configuration and Tuning

All important robot parameters are centralized in a single YAML configuration file:

```
robot/config/robot_default.yaml
```

This design allows the robot’s behavior to be modified **without editing source code**.

Examples of configurable parameters include:
- Camera selection and resolution
- Detection confidence thresholds
- Perception timing and anti-flicker settings
- Control gains and motion parameters
- Hardware and communication settings

The Python code loads this configuration at startup using `config_loader.py` and distributes parameters to each subsystem.

**Design intent:**
- Avoid hard-coded values
- Enable rapid tuning during testing
- Allow safe adjustments without modifying logic
- Keep source code stable and readable

Unless new functionality is being added, most behavior changes should be made by editing the YAML configuration file rather than modifying Python source files.

---

## Key Files and Their Roles

### `scripts/run_robot.py`
- Main entry point to launch the robot software
- Starts `pwc_robot.main` and initializes subsystems

### `requirements.txt`
- **Always required**
- Full baseline dependency set
- Must be installed **inside the project virtual environment**
- Used on all systems (laptop, Raspberry Pi, CPU-only machines)

### `requirements-cuda.txt`
- **Optional**
- Only for systems with an NVIDIA GPU and CUDA installed
- Installs GPU-accelerated packages
- Must be installed **before** `requirements.txt`

---

## `pwc_robot` Package

### `main.py`
- Central coordination point
- Loads YAML config
- Creates subsystem instances (camera, detector, perception, GUI)
- Runs the main loop
- Uses non-blocking rate control (`utils.rate.Rate`)

### `config_loader.py`
- Loads YAML config files
- Resolves paths (model weights, output directories, etc.)
- Keeps configuration logic out of the behavior code

---

## Perception Package (`pwc_robot/perception`)

### `camera.py`
- Wrapper around OpenCV `VideoCapture`
- Handles camera initialization and frame acquisition

### `detector.py`
- Wraps an Ultralytics YOLO model
- Runs inference on frames
- Produces detection outputs for downstream logic

### `computer_vision.py`
- Owns the `Camera` and `Detector`
- Executes detection when `tick()` is called
- Implements anti-flicker logic (streak, hold time)
- Provides:
  - latest observation (`get_latest_obs()`)
  - latest annotated frame (`get_latest_annotated_frame()`)

---

## GUI Package (`pwc_robot/gui`)

The GUI is a lightweight **Flask dashboard** used to monitor the robot.

### How it works
- Flask serves the dashboard HTML from `templates/gui.html`
- Flask serves frontend assets (CSS and JS) from `static/`
- The live camera view is displayed using an MJPEG stream endpoint
- The status panel updates by polling a JSON endpoint

### `gui_server.py`
Defines the Flask app and routes, typically including:
- `/` for the dashboard page
- `/stream/comp_vision` for MJPEG stream (annotated frames)
- `/perception/status` for JSON perception status

### `templates/gui.html`
- Page layout only
- References external CSS and JS using `url_for('static', ...)`
- Contains placeholders (elements with IDs) that the JS updates

### `static/css/gui.css`
- All styling for the dashboard (layout, fonts, cards)

### `static/js/gui.js`
- Polls `/perception/status` on a timer
- Maps detection info to a human-readable **STATUS**
  - Searching
  - Detected
  - Stable detection
- Updates the right-hand status panel by writing to DOM element IDs

---

## Installation and Setup

### Why Use a Virtual Environment (venv)
A virtual environment:
- Isolates project dependencies
- Prevents conflicts with system Python
- Ensures consistent behavior across machines

**All dependencies must be installed inside the virtual environment.**

---

### Create and Activate a Virtual Environment

From the `robot/python` directory:

```bash
python -m venv venv
```

Activate it:

**macOS / Linux**
```bash
source venv/bin/activate
```

**Windows**
```bash
venv\Scripts\activate
```

You should see `(venv)` in your terminal prompt after activation.

---

### Install Dependencies

#### CPU-only systems (default)
```bash
pip install -r requirements.txt
```

#### NVIDIA GPU + CUDA systems
```bash
pip install -r requirements-cuda.txt
pip install -r requirements.txt
```

---

## Running the Robot

From the `robot/python` directory:

```bash
python scripts/run_robot.py
```

This will:
- Load YAML configuration
- Initialize the camera and perception pipeline
- Start the main robot loop
- Launch the Flask GUI server (if enabled)

---

## Accessing the GUI

Open a web browser and navigate to:

```
http://localhost:5000
```

If running on a Raspberry Pi, replace `localhost` with the Pi’s IP address:

```
http://<raspberry_pi_ip>:5000
```

---

## Notes
- The software is designed to run without ROS
- Configuration should be modified via YAML files, not source code
- Laptop-first development is intentional
