/*
  Ultrasonic.cpp

  Purpose:
  Implements the HC-SR04 ultrasonic sensor interface for APWCR using a non-blocking
  state-machine approach.

  Implementation details:
  - The TRIG pin is pulsed HIGH for approximately 10 microseconds to initiate a measurement.
  - Echo timing is measured using micros() without blocking the main loop.
  - Timeouts are used to handle cases where no echo is received.
  - Distance is calculated using the speed of sound and the measured echo pulse width.

  Notes:
  - Measurement rate should be limited to avoid cross-talk and noisy readings.
  - Invalid readings are expected and should be filtered or debounced by higher-level logic.
*/
