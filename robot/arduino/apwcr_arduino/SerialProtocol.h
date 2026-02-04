/*
  SerialProtocol.h

  Purpose:
  Declares the SerialProtocol class which handles communication between Python and Arduino over USB serial.

  Responsibilities:
  - Read incoming serial data and parse complete commands
  - Update the latest command state used by controllers
  - Provide helper functions to send telemetry back to Python in a consistent format

  Design rule:
  SerialProtocol must not directly drive motors or servos. It only updates command data.
*/
