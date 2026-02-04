/*
  Ultrasonic.h

  Purpose:
  Declares a non-blocking interface for the HC-SR04 ultrasonic distance sensor used on the
  Autonomous Pet Waste Collection Robot (APWCR).

  Sensor overview:
  The HC-SR04 measures distance by emitting an ultrasonic pulse on the TRIG pin and timing
  the duration of the echo pulse received on the ECHO pin. Distance is computed from the
  echo pulse width.

  Responsibilities:
  - Trigger ultrasonic measurements at a controlled rate (typically 10–20 Hz)
  - Measure echo pulse duration without blocking the main control loop
  - Convert echo time to distance in centimeters
  - Detect timeouts or invalid measurements and flag them as invalid

  Design notes:
  - This class must be non-blocking to avoid disrupting motor control and PID timing.
  - The class does not make safety or stop decisions. It only reports distance and validity.
  - Obstacle stopping logic should be handled by higher-level controllers (e.g. DriveController).

  Outputs:
  - distance_cm: most recent measured distance in centimeters
  - valid: true if the measurement is valid, false if timed out or invalid
*/
