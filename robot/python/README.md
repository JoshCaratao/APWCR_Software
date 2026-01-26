# APWCR Robot Software (Python)

This directory contains the **high-level Python software** for the Autonomous Pet Waste Collection Robot (APWCR).  
It handles perception, scheduling, and high-level robot logic, and is structured to resemble a lightweight, ROS-inspired architecture.

This code runs on a laptop or single-board computer (e.g., Raspberry Pi) and interfaces with lower-level microcontroller code (Arduino) that handles motor control and real-time actuation.

## Directory Structure

```
robot/python/
├── pwc_robot/ # Main Python package (robot logic)
│ ├── init.py
│ ├── main.py # Entry point for running the robot logic
│ ├── config_loader.py # YAML config loading + path handling
│ │
│ ├── perception/ # Computer vision & sensing
│ │ ├── init.py
│ │ ├── camera.py # OpenCV camera wrapper
│ │ ├── detector.py # YOLO detector wrapper
│ │ └── computer_vision.py # Perception orchestration + anti-flicker
│ │
│ ├── utils/ # Small shared utilities
│ │ ├── init.py
│ │ └── rate.py # Non-blocking rate limiter
│ │
│ └── (future folders)
│ ├── state/ # Finite state machine (SEARCH, APPROACH, COLLECT)
│ ├── control/ # High-level control decisions
│ └── comms/ # Communication with Arduino
│
├── scripts/ # Launch / helper scripts
│ └── run_robot.py # Convenience script to start the robot
│
├── requirements.txt # Python dependencies
└── README.md # This file
```


## How to Run the Robot

From the `robot/python` directory:

```bash
python scripts/run_robot.py
```
This launches pwc_robot.main, loads the YAML configuration, initializes the camera and detector, and begins running the perception loop.

## The pwc_robot Package

pwc_robot is the main robot software package.
It is written as a proper Python package so it can be cleanly imported, tested, and extended.

``main.py``

Central coordination point

Loads YAML config

Creates subsystem instances (camera, detector, perception)

Runs the main loop

Uses non-blocking rate control (utils.rate.Rate)

config_loader.py

Loads YAML config files

Resolves relative paths (e.g., model paths)

Performs basic validation

Keeps configuration logic out of the robot code





