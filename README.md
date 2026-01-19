# Autonomous Pet Waste Collection Robot (APWCR) Software Repository

This repository contains the software for the Autonomous Pet Waste Collection Robot (APWCR) senior design project. It includes model training artifacts, robot runtime code, and isolated testing utilities.

The codebase is organized to clearly separate training, deployment, and testing, following best practices for robotics and embedded systems development.

## Repository Structure
```
APWCR_Software/
├── cv_training/
├── robot/
├── test/
└── README.md
```

###  `cv_training/`
This folder contains computer vision model training artifacts. Includes Jupyter notebooks used to train the YOLO-based pet waste detection model. Training was performed in Google Colab using GPU acceleration. Notebooks document dataset usage, training parameters, and model configuration. Raw datasets and large training outputs are intentionally excluded from the repository.

Purpose:
Documentation and reproducibility of the machine learning workflow.
This code is not run on the robot.

### `robot/`
This folder contains **all software required to run the robot**. It represents the deployed system and separates high-level autonomy from low-level control.

- **`python/`**  
  High-level autonomy code executed on the Raspberry Pi.
  - Computer vision inference (YOLO)
  - State machine logic
  - Motion command generation
  - Serial communication with the Arduino

- **`arduino/`**  
  Low-level embedded code running on the Arduino.
  - Motor control
  - Actuator sequencing
  - Sensor handling
  - Serial command parsing

- **`cv_model/`**  
  Trained YOLO model weights used for runtime inference.

- **`config/`**  
  YAML configuration files containing all tunable robot parameters.
  - Camera settings
  - Detection thresholds
  - Control gains
  - Hardware configuration (e.g. serial port, baud rate)
 
### `test/`
This folder contains **isolated testing and development scripts**.

- Used for subsystem-level testing (e.g. computer vision, camera input)
- Allows rapid experimentation without modifying deployed robot code
- Test scripts reuse configuration files from the `robot/` directory to ensure consistency

**Purpose:**  
Safe development, debugging, and experimentation without impacting runtime code.

## Design Philosophy
- Clear separation between **training**, **runtime**, and **testing**
- Centralized configuration using YAML
- Platform-agnostic file paths for portability
- Architecture designed to remain compatible with ROS if required in the future
