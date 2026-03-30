#include "comms/Protocol.h"

#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

#include "Params.h"

/*
===============================================================================
  Protocol.cpp
===============================================================================

  Purpose:
  Implements the minimal newline-delimited JSON protocol used by the
  feedforward motor identification firmware.

  This protocol intentionally avoids extra robot-runtime fields so the test
  firmware stays focused on motor commands and RPM telemetry only.
===============================================================================
*/

namespace protocol {

namespace {

void writeFloatOrNull(JsonObject obj, const char* key, float value) {
  if (isfinite(value)) {
    obj[key] = value;
  } else {
    obj[key] = nullptr;
  }
}

}  // namespace

/*=============================================================================
  ENCODE (Arduino -> Python)
=============================================================================*/

void encodeTelemetryLine(const TelemetryFrame& t, Print& out) {
  StaticJsonDocument<SERIAL_JSON_DOC_BYTES> doc;

  doc["type"] = "telemetry";
  doc["arduino_time_ms"] = t.arduino_time_ms;
  doc["ack_seq"] = t.ack_seq;

  doc["drive_lhs_cmd"] = t.motors.drive_lhs_cmd;
  doc["drive_rhs_cmd"] = t.motors.drive_rhs_cmd;
  doc["mech_rhs_cmd"] = t.motors.mech_rhs_cmd;
  doc["mech_lhs_cmd"] = t.motors.mech_lhs_cmd;

  writeFloatOrNull(doc.as<JsonObject>(), "drive_lhs_rpm", t.motors.drive_lhs_rpm);
  writeFloatOrNull(doc.as<JsonObject>(), "drive_rhs_rpm", t.motors.drive_rhs_rpm);
  writeFloatOrNull(doc.as<JsonObject>(), "mech_rhs_rpm", t.motors.mech_rhs_rpm);
  writeFloatOrNull(doc.as<JsonObject>(), "mech_lhs_rpm", t.motors.mech_lhs_rpm);

  serializeJson(doc, out);
  out.println();
}

/*=============================================================================
  DECODE (Python -> Arduino)
=============================================================================*/

bool decodeCommandLine(const char* line, CommandFrame& out_cmd) {
  out_cmd = CommandFrame();
  if (!line) return false;

  StaticJsonDocument<SERIAL_JSON_DOC_BYTES> doc;
  const DeserializationError err = deserializeJson(doc, line);
  if (err) return false;

  JsonObject obj = doc.as<JsonObject>();
  if (obj.isNull()) return false;

  const char* type = obj["type"];
  if (!type || strcmp(type, "cmd") != 0) return false;
  if (!obj.containsKey("seq")) return false;

  out_cmd.seq = obj["seq"].as<uint32_t>();
  out_cmd.motors.drive_lhs_cmd = obj["drive_lhs_cmd"] | 0.0f;
  out_cmd.motors.drive_rhs_cmd = obj["drive_rhs_cmd"] | 0.0f;
  out_cmd.motors.mech_rhs_cmd = obj["mech_rhs_cmd"] | 0.0f;
  out_cmd.motors.mech_lhs_cmd = obj["mech_lhs_cmd"] | 0.0f;
  out_cmd.valid = true;
  return true;
}

}  // namespace protocol
