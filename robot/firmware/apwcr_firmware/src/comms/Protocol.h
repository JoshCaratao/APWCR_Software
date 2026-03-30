#pragma once
#include <Arduino.h>

#include "comms/Messages.h"

/*
===============================================================================
  Protocol.h
===============================================================================

  PURPOSE
  -------
  Encode/decode helpers for the Arduino <-> Laptop wire protocol.

  Wire format:
    - Newline-delimited JSON (one object per line)

  Matches Python:
    pwc_robot/comms/protocol.py
===============================================================================
*/

namespace protocol {

/*=============================================================================
  ENCODE (Arduino -> Laptop)
=============================================================================*/

// Writes one telemetry JSON line (includes trailing '\n')
void encodeTelemetryLine(const TelemetryFrame& t, Print& out);


/*=============================================================================
  DECODE (Laptop -> Arduino)
=============================================================================*/

/*
  Attempts to parse one command JSON line.

  Returns:
    - true if decoded into out_cmd (and out_cmd.valid will be true)
    - false if not a valid command frame or parse failed
*/
bool decodeCommandLine(const char* line, CommandFrame& out_cmd);

}  // namespace protocol
