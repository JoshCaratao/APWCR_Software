#pragma once
#include <Arduino.h>

#include "comms/Messages.h"

/*
===============================================================================
  Protocol.h
===============================================================================

  Purpose:
  Encode and decode the minimal newline-delimited JSON protocol used by the
  feedforward motor identification firmware.

  Wire format:
  - Python -> Arduino: type="cmd"
  - Arduino -> Python: type="telemetry"
  - One JSON object per line
===============================================================================
*/

namespace protocol {

// Writes one telemetry JSON line, including the trailing newline.
void encodeTelemetryLine(const TelemetryFrame& t, Print& out);

// Attempts to decode one command JSON line into out_cmd.
bool decodeCommandLine(const char* line, CommandFrame& out_cmd);

}  // namespace protocol
