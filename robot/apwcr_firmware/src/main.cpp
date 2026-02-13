/*
  APWCR Arduino Controller (Low Level Hardware Layer)

  Purpose:
  Minimal bring-up main loop to validate comms between Arduino and laptop.

  For now:
  - RX: call SerialLink.RxTick() so we can receive + parse commands
  - TX: send telemetry at TELEMETRY_UPDATE_HZ so the GUI can display data
  - No sensors yet (no DistanceSensor, encoders, motors, servos)
*/

#include <Arduino.h>

#include "Pins.h"
#include "Params.h"

#include "utils/Rate.h"
#include "comms/SerialLink.h"
#include "sensors/DistanceSensor.h"


/*=============================================================================
  GLOBALS
=============================================================================*/

// Serial link (USB)
SerialLink g_link(SERIAL_USB);

// Distance Sensor
DistanceSensor g_distance_sensor(PIN_ULTRASONIC_TRIG, PIN_ULTRASONIC_ECHO, ULTRASONIC_MAX_DISTANCE_CM, ULTRASONIC_TIMEOUT_US, ULTRASONIC_MIN_IN, ULTRASONIC_MAX_VALID_IN);

// Rates
Rate g_comms_rate(RxCOMM_UPDATE_HZ);                    // RX parsing tick (fast, non-blocking)
Rate g_telemetry_rate(TELEMETRY_UPDATE_HZ);
Rate g_ultrasonic_rate(ULTRASONIC_UPDATE_HZ);


/*=============================================================================
  SETUP
=============================================================================*/

void setup() {
  SERIAL_USB.begin(SERIAL_BAUD);
  g_link.begin();
  g_distance_sensor.begin();
}

/*=============================================================================
  LOOP
=============================================================================*/

void loop() {

  const uint32_t now_ms = millis();

  // RX tick: read serial and parse command frames
  if (g_comms_rate.ready(now_ms)) {
    g_link.RxTick(now_ms);
  }

  // Distance Sensor Tick: Read Ultrasonic Sensor Data
  if (g_ultrasonic_rate.ready(now_ms)) {
    g_distance_sensor.tick(now_ms);
}


  // TX tick: publish telemetry so Python/GUI can confirm link health
  if (g_telemetry_rate.ready(now_ms)) {
    TelemetryFrame t;
    t.arduino_time_ms = now_ms;
    t.ack_seq = g_link.ackSeq();     // ACK = last received + parsed command seq

    // For bring-up, these stay as null (NAN encodes to JSON null in Protocol.cpp)
    t.wheel.left_rpm  = 0.0;
    t.wheel.right_rpm = 0.0;
    //t.mech.*          = NAN;

    // Add ultrasonic data
    const auto& ultrasonic_state = g_distance_sensor.getState();
    t.ultrasonic.valid = ultrasonic_state.valid;
    if(ultrasonic_state.valid == true){
      t.ultrasonic.distance_in = ultrasonic_state.distance_in;
      
    } else {
      t.ultrasonic.distance_in = NAN;
    }
    

    // Optional note
    t.note = nullptr;

    g_link.TxTick(t);
  }

}
