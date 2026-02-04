/*
  APWCR Arduino Controller (Low Level Hardware Layer)

  Purpose:
  This Arduino sketch is the low level controller for the Autonomous Pet Waste Collection Robot (APWCR).
  It acts as the real-time hardware interface for the robot while a separate Python program on the host
  computer handles computer vision, state logic, and high-level control decisions.

  Responsibilities on Arduino:
  - Receive high-level commands from Python over USB serial (drive commands and mechanism setpoints).
  - Read sensors reliably (encoders, ultrasonic, and any other low-level inputs).
  - Run fast, deterministic low-level control loops (PID for wheel speed and mechanism position as needed).
  - Drive actuators safely (DC motors and servos) with watchdog and safety limits.
  - Send telemetry back to Python (encoder counts/speeds, ultrasonic distance, status flags).

  Design note:
  Serial parsing should only update the latest command state. Motors and servos are updated in timed tasks
  so partial serial reads never directly move hardware.
*/


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
