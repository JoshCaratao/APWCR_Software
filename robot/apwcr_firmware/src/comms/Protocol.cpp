#include "comms/Protocol.h"
#include <math.h>

/*
===============================================================================
  Protocol.cpp
===============================================================================

  PURPOSE
  -------
  Implements newline-delimited JSON protocol helpers.

  Wire format:
    - One JSON object per line
    - Laptop -> Arduino: type="cmd"
    - Arduino -> Laptop: type="telemetry"

  Notes:
  - Telemetry encoding uses simple Serial/Print printing.
  - Command decoding uses ArduinoJson for safe parsing.
===============================================================================
*/

#include <ArduinoJson.h>
#include <string.h>


/*=============================================================================
  SMALL HELPERS
=============================================================================*/

// Convert mode string -> enum
static MechMotorMode parseMode(const char* s) {
  if (!s) return MechMotorMode::UNKNOWN;
  if (strcmp(s, "POS_DEG") == 0) return MechMotorMode::POS_DEG;
  if (strcmp(s, "DUTY") == 0)    return MechMotorMode::DUTY;
  return MechMotorMode::UNKNOWN;
}


namespace protocol {

/*=============================================================================
  ENCODE (Arduino -> Laptop)
=============================================================================*/

void encodeTelemetryLine(const TelemetryFrame& t, Print& out) {
  StaticJsonDocument<384> doc;

  doc["type"] = "telemetry";
  doc["arduino_time_ms"] = t.arduino_time_ms;
  doc["ack_seq"] = t.ack_seq;

  // wheel
  JsonObject wheel = doc.createNestedObject("wheel");
  if (isfinite(t.wheel.left_rpm))
    wheel["left_rpm"] = t.wheel.left_rpm;
  else
    wheel["left_rpm"] = nullptr;

  if (isfinite(t.wheel.right_rpm))
    wheel["right_rpm"] = t.wheel.right_rpm;
  else
    wheel["right_rpm"] = nullptr;

  // mech
  JsonObject mech = doc.createNestedObject("mech");

  if (isfinite(t.mech.servo_LID_deg))    
    mech["servo_LID_deg"] = t.mech.servo_LID_deg;
  else                                  
    mech["servo_LID_deg"] = nullptr;

  if (isfinite(t.mech.servo_SWEEP_deg))  
    mech["servo_SWEEP_deg"] = t.mech.servo_SWEEP_deg;
  else                                  
    mech["servo_SWEEP_deg"] = nullptr;
  if (isfinite(t.mech.motor_RHS_deg))    
    mech["motor_RHS_deg"] = t.mech.motor_RHS_deg;
  else                                  
    mech["motor_RHS_deg"] = nullptr;

  if (isfinite(t.mech.motor_LHS_deg))    
    mech["motor_LHS_deg"] = t.mech.motor_LHS_deg;
  else                                  
    mech["motor_LHS_deg"] = nullptr;

  // ultrasonic
  JsonObject us = doc.createNestedObject("ultrasonic");
  us["valid"] = t.ultrasonic.valid;

  if (t.ultrasonic.valid && isfinite(t.ultrasonic.distance_in)) 
    us["distance_in"] = t.ultrasonic.distance_in;
  else                                                          
    us["distance_in"] = nullptr;

  if (t.note)
    doc["note"] = t.note;
  else
    doc["note"] = nullptr;

  serializeJson(doc, out);
  out.println();
}


/*=============================================================================
  DECODE (Laptop -> Arduino)
=============================================================================*/

bool decodeCommandLine(const char* line, CommandFrame& out_cmd) {
  out_cmd = CommandFrame();   // reset everything
  if (!line) return false;

  // 512 bytes is safe for your current command schema
  StaticJsonDocument<512> doc;

  if (deserializeJson(doc, line)) {
    return false;
  }

  JsonObject obj = doc.as<JsonObject>();
  if (obj.isNull()) return false;

  // Must be a command
  const char* type = obj["type"];
  if (!type || strcmp(type, "cmd") != 0) return false;

  // Required fields
  if (!obj.containsKey("seq")) return false;
  if (!obj.containsKey("host_time_ms")) return false;
  if (!obj.containsKey("drive")) return false;
  if (!obj.containsKey("mech")) return false;

  out_cmd.seq = obj["seq"].as<uint32_t>();
  out_cmd.host_time_ms = obj["host_time_ms"].as<uint32_t>();

  // drive
  JsonObject drive = obj["drive"].as<JsonObject>();
  if (drive.isNull()) return false;

  out_cmd.drive.linear_ftps = drive["linear"] | 0.0f;
  out_cmd.drive.angular_dps = drive["angular"] | 0.0f;

  // mech
  JsonObject mech = obj["mech"].as<JsonObject>();
  if (mech.isNull()) return false;

  // servos (nullable)
  if (!mech["servo_LID_deg"].isNull()) {
    out_cmd.mech.servo_LID_deg = mech["servo_LID_deg"] | 0.0f;
    out_cmd.mech.servo_LID_present = true;
  }

  if (!mech["servo_SWEEP_deg"].isNull()) {
    out_cmd.mech.servo_SWEEP_deg = mech["servo_SWEEP_deg"] | 0.0f;
    out_cmd.mech.servo_SWEEP_present = true;
  }

  // motors (nullable objects)
  if (!mech["motor_RHS"].isNull()) {
    JsonObject m = mech["motor_RHS"].as<JsonObject>();
    MechMotorMode mode = parseMode(m["mode"]);
    if (mode != MechMotorMode::UNKNOWN) {
      out_cmd.mech.motor_RHS.mode = mode;
      out_cmd.mech.motor_RHS.value = m["value"] | 0.0f;
      out_cmd.mech.motor_RHS.present = true;
    }
  }

  if (!mech["motor_LHS"].isNull()) {
    JsonObject m = mech["motor_LHS"].as<JsonObject>();
    MechMotorMode mode = parseMode(m["mode"]);
    if (mode != MechMotorMode::UNKNOWN) {
      out_cmd.mech.motor_LHS.mode = mode;
      out_cmd.mech.motor_LHS.value = m["value"] | 0.0f;
      out_cmd.mech.motor_LHS.present = true;
    }
  }

  out_cmd.valid = true;
  return true;
}

}  // namespace protocol
