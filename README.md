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

This folder contains computer vision model training artifacts.

Includes Jupyter notebooks used to train the YOLO-based pet waste detection model.

Training was performed in Google Colab using GPU acceleration.

Notebooks document dataset usage, training parameters, and model configuration.

Raw datasets and large training outputs are intentionally excluded from the repository.

Purpose:
Documentation and reproducibility of the machine learning workflow.
This code is not run on the robot.
