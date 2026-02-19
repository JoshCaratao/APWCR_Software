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
#include "actuators/ServoActuator.h"



/*=============================================================================
  GLOBALS
=============================================================================*/

// Serial link (USB)
SerialLink g_link(SERIAL_USB);

// Distance Sensor
DistanceSensor g_distance_sensor(PIN_ULTRASONIC_TRIG, PIN_ULTRASONIC_ECHO, ULTRASONIC_MAX_DISTANCE_CM, ULTRASONIC_TIMEOUT_US, ULTRASONIC_MIN_IN, ULTRASONIC_MAX_VALID_IN);

// Servos
ServoActuator g_lid_servo(
  PIN_SERVO_LID,
  SERVO_MIN_DEG,
  SERVO_MAX_DEG,
  LID_SERVO_RAMP_DPS,
  SERVO_DEADBAND_DEG,
  LID_SERVO_SETTLE_MS,
  LID_SERVO_AUTO_DETACH_ON_CLOSED,
  (float)LID_CLOSED_DEG
);

ServoActuator g_sweep_servo(
  PIN_SERVO_SWEEP,
  SERVO_MIN_DEG,
  SERVO_MAX_DEG,
  SWEEP_SERVO_RAMP_DPS,
  SERVO_DEADBAND_DEG,
  SWEEP_SERVO_SETTLE_MS,        // settle_ms (or define SWEEP_SERVO_SETTLE_MS)
  SWEEP_SERVO_AUTO_DETACH_ON_CLOSED,      // auto_detach_on_closed (usually false for sweep)
  (float)SWEEP_STOW_DEG
);

// Rates
Rate g_comms_rate(RxCOMM_UPDATE_HZ);                    // RX parsing tick (fast, non-blocking)
Rate g_telemetry_rate(TELEMETRY_UPDATE_HZ);
Rate g_ultrasonic_rate(ULTRASONIC_UPDATE_HZ);
Rate g_servo_rate(SERVO_UPDATE_HZ);

// Track last applied command seq so we only apply new targets once
static uint32_t g_last_applied_seq = 0;
static bool g_in_timeout = false;


/*=============================================================================
  SETUP
=============================================================================*/

void setup() {
  // Serial Comms Setup
  SERIAL_USB.begin(SERIAL_BAUD);
  g_link.begin();

  // Ultrasonic Sensor Setup
  g_distance_sensor.begin();

  // Servo Setups
  g_lid_servo.begin((float)LID_CLOSED_DEG);
  g_sweep_servo.begin((float)SWEEP_STOW_DEG);

}

/*=============================================================================
  LOOP
=============================================================================*/

void loop() {

  const uint32_t now_ms = millis();

  // RX tick: read serial and parse command frames
  if (g_comms_rate.ready(now_ms)) {
    g_link.RxTick(now_ms);

    // Apply servo targets only when a new command arrives and return to closed, if commands time out
    if (g_link.hasCommand()) {
      const CommandFrame& cmd = g_link.latestCommand();
      if (cmd.seq != g_last_applied_seq) {
        g_last_applied_seq = cmd.seq;

        if (cmd.mech.servo_LID_present) {
          g_lid_servo.setTargetDeg(cmd.mech.servo_LID_deg, now_ms);
        }

        if (cmd.mech.servo_SWEEP_present) {
          g_sweep_servo.setTargetDeg(cmd.mech.servo_SWEEP_deg, now_ms);
        }

      }
    }

  }

  // Check if telemetry commands have timed out and apply safety logic if timed out
  const bool timed_out = g_link.commandTimedOut(now_ms);
  if (timed_out && !g_in_timeout) {
    g_in_timeout = true;
    g_lid_servo.setTargetDeg((float)LID_CLOSED_DEG, now_ms);
    g_sweep_servo.setTargetDeg((float)SWEEP_STOW_DEG, now_ms);
  } else if (!timed_out) {
    g_in_timeout = false;
  }



  // Distance Sensor Tick: Read Ultrasonic Sensor Data
  if (g_ultrasonic_rate.ready(now_ms)) {
    g_distance_sensor.tick(now_ms);
  }


  // Servo Tick
  if (g_servo_rate.ready(now_ms)) {
    g_lid_servo.tick(now_ms);
    g_sweep_servo.tick(now_ms);
  }


  // TX tick: publish telemetry so Python/GUI can confirm link health
  if (g_telemetry_rate.ready(now_ms)) {
    TelemetryFrame t;
    t.arduino_time_ms = now_ms;
    t.ack_seq = g_link.ackSeq();     // ACK = last received + parsed command seq

    // For bring-up, these stay as null (NAN encodes to JSON null in Protocol.cpp)
    t.wheel.left_rpm  = 0.0;
    t.wheel.right_rpm = 0.0;


    //t.mech       
    t.mech.servo_LID_deg   = g_lid_servo.getState().current_deg;
    t.mech.servo_SWEEP_deg = g_sweep_servo.getState().current_deg;


    // Add ultrasonic data
    const auto& ultrasonic_state = g_distance_sensor.getState();
    t.ultrasonic.valid = ultrasonic_state.valid;
    if(ultrasonic_state.valid == true){
      t.ultrasonic.distance_in = ultrasonic_state.distance_in;
      
    } else {
      t.ultrasonic.distance_in = NAN;
    }
    

    // Optional note
    t.note = g_link.debugNote(now_ms);




    g_link.TxTick(t);
  }

}
