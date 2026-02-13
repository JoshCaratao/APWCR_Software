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


/*=============================================================================
  GLOBALS
=============================================================================*/

// Serial link (USB)
SerialLink g_link(SERIAL_USB);

// Rates
Rate g_comms_rate(RxCOMM_UPDATE_HZ);                    // RX parsing tick (fast, non-blocking)
Rate g_telemetry_rate(TELEMETRY_UPDATE_HZ);


/*=============================================================================
  SETUP
=============================================================================*/

void setup() {
  SERIAL_USB.begin(SERIAL_BAUD);
  g_link.begin();
}

/*=============================================================================
  LOOP
=============================================================================*/

void loop() {

  const uint32_t rx_ms = millis();
  // RX tick: read serial and parse command frames
  if (g_comms_rate.ready(rx_ms)) {
    g_link.RxTick(rx_ms);
  }

  // TX tick: publish telemetry so Python/GUI can confirm link health
  const uint32_t tx_ms = millis();
  if (g_telemetry_rate.ready(tx_ms)) {
    TelemetryFrame t;
    t.arduino_time_ms = tx_ms;
    t.ack_seq = g_link.ackSeq();     // ACK = last received + parsed command seq

    // For bring-up, these stay as null (NAN encodes to JSON null in Protocol.cpp)
    t.wheel.left_rpm  = 0.0;
    t.wheel.right_rpm = 0.0;
    //t.mech.*          = NAN;

    // Ultrasonic not wired yet, so publish invalid
    t.ultrasonic.valid = false;
    t.ultrasonic.distance_in = NAN;

    // Optional note
    t.note = nullptr;

    g_link.TxTick(t);
  }
}
