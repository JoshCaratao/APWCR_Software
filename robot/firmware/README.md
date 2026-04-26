# APWCR Firmware Guide

This directory contains the PlatformIO firmware projects used by the APWCR robot. The most important project for normal robot operation is `apwcr_firmware_v2`, which is the current Arduino Mega firmware expected by the Python runtime.

## Projects In This Folder

### `apwcr_firmware_v2`

Current low-level firmware for the deployed robot.

Responsibilities:

- Receive drive and mechanism commands from the Python host over USB serial.
- Run periodic control loops for drive motors and mechanism motors.
- Control lid and sweeper servos.
- Sample wheel encoders, mechanism encoders, and the ultrasonic sensor.
- Enforce command-timeout safety behavior.
- Publish telemetry back to the host.

### `apwcr_firmware`

Older firmware snapshot kept for reference. It is useful for comparing implementation history, but it should not be assumed to match the current Python runtime or current hardware tuning.

### `feedforward_firmware`

Separate test firmware used only for motor characterization. It is paired with the Python tooling under `robot/python/feedforward_model_test/` and should not be flashed when the goal is to run the normal robot stack.

## Active Firmware Architecture

The active project lives in:

- `robot/firmware/apwcr_firmware_v2/`

Important folders and files:

- `src/main.cpp`
  Top-level scheduler, command application, timeout handling, and telemetry publishing.
- `src/comms/`
  Serial protocol parsing and message serialization.
- `src/control/DriveController.*`
  Drive-wheel command conversion, feedforward, PID, and stall protection.
- `src/control/MechanismController.*`
  Mechanism motor control, servo coordination, safe/stow handling, and telemetry filling.
- `src/control/PID.*`
  Reusable PID helper implementation.
- `src/control/StallGuard.*`
  Stall/fault detection helpers.
- `src/actuators/`
  Motor and servo wrappers.
- `src/sensors/`
  Encoder and ultrasonic sensor interfaces.
- `src/utils/Rate.h`
  Non-blocking periodic task scheduling helper.
- `include/Pins.h`
  Hardware pin assignments.
- `include/Params.h`
  Tunable robot constants and safety limits.

## Runtime Behavior

The firmware loop in `apwcr_firmware_v2/src/main.cpp` performs these tasks:

1. Parse incoming host commands.
2. Apply new drive and mechanism commands when a new command sequence arrives.
3. Detect host-command timeout and move to a safe state.
4. Tick the drive controller.
5. Tick the mechanism controller.
6. Tick the ultrasonic sensor.
7. Publish telemetry at the configured telemetry rate.

Safety behavior on command timeout:

- Drive outputs are stopped.
- Mechanism outputs are moved to a safe/stow state.

## Build and Upload

From the repository root:

```bash
cd robot/firmware/apwcr_firmware_v2
pio run
pio run -t upload
pio device monitor -b 250000
```

The active PlatformIO environment is:

- `env:megaatmega2560`

## Parameters That Future Teams Will Change Most Often

The primary tuning file is:

- `robot/firmware/apwcr_firmware_v2/include/Params.h`

That file contains:

- Robot geometry constants
- Encoder scaling
- Feedforward coefficients
- PID gains
- Servo positions and ramp rates
- Ultrasonic range/sanity limits
- Update rates
- Serial baud and command timeout settings

Hardware pin mapping is kept separately in:

- `robot/firmware/apwcr_firmware_v2/include/Pins.h`

This separation is intentional: future teams can retune behavior in `Params.h` without mixing those changes with board wiring definitions in `Pins.h`.

## Relationship To The Python Runtime

The Python host expects the firmware to:

- Use the same serial baud rate
- Understand the same JSON command schema
- Return telemetry with the fields consumed by `pwc_robot/comms/` and the Flask GUI

When making changes, keep these files aligned:

- Python baud and comms settings:
  `robot/config/robot_default.yaml`
- Firmware baud:
  `robot/firmware/apwcr_firmware_v2/include/Params.h`
- PlatformIO monitor baud:
  `robot/firmware/apwcr_firmware_v2/platformio.ini`

## Recommended Reading Order For New Team Members

1. `robot/firmware/README.md`
2. `robot/firmware/apwcr_firmware_v2/src/main.cpp`
3. `robot/firmware/apwcr_firmware_v2/include/Pins.h`
4. `robot/firmware/apwcr_firmware_v2/include/Params.h`
5. `robot/firmware/apwcr_firmware_v2/src/control/DriveController.*`
6. `robot/firmware/apwcr_firmware_v2/src/control/MechanismController.*`

