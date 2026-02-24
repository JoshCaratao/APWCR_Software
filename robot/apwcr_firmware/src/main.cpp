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
#include "control/DriveController.h"



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

// DriveController
static DriveController::Config makeDriveConfig() {
  DriveController::Config c;

  // Motor pins
  c.pin_lhs_dir = PIN_LHS_DRIVE_DIR;
  c.pin_lhs_pwm = PIN_LHS_DRIVE_PWM;
  c.pin_rhs_dir = PIN_RHS_DRIVE_DIR;
  c.pin_rhs_pwm = PIN_RHS_DRIVE_PWM;

  // Encoder pins
  c.pin_enc_lhs_a = PIN_ENC_LHS_DRIVE_A;
  c.pin_enc_lhs_b = PIN_ENC_LHS_DRIVE_B;
  c.pin_enc_rhs_a = PIN_ENC_RHS_DRIVE_A;
  c.pin_enc_rhs_b = PIN_ENC_RHS_DRIVE_B;

  // Inversions
  c.invert_lhs_motor = DRIVE_INVERT_LHS_MOTOR;
  c.invert_rhs_motor = DRIVE_INVERT_RHS_MOTOR;
  c.invert_lhs_encoder = DRIVE_INVERT_LHS_ENCODER;
  c.invert_rhs_encoder = DRIVE_INVERT_RHS_ENCODER;

  // Geometry / conversion
  c.track_width_ft = TRACK_WIDTH_FT;
  c.wheel_circumference_ft = WHEEL_CIRCUMFERENCE_FT;
  c.counts_per_wheel_rev = COUNTS_PER_WHEEL_REV;

  // Limits
  c.max_linear_ftps = MAX_LINEAR_SPEED_FTPS;
  c.max_angular_dps = MAX_ANGULAR_SPEED_DPS;

  // PID
  c.kp = DRIVE_KP;
  c.ki = DRIVE_KI;
  c.kd = DRIVE_KD;
  c.integral_limit = DRIVE_INTEGRAL_LIMIT;

  return c;
}

DriveController::Config g_drive_cfg = makeDriveConfig();
DriveController g_drive(g_drive_cfg);

// Rates
Rate g_comms_rate(RxCOMM_UPDATE_HZ);                    // RX parsing tick (fast, non-blocking)
Rate g_drive_rate(DRIVE_UPDATE_HZ);                     // Drive control tick
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

  // Drive Setup
  g_drive.begin();

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

        // Apply drive command
        g_drive.setCommand(cmd.drive);

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
    g_drive.stop();
  } else if (!timed_out) {
    g_in_timeout = false;
  }

  // Drive Tick
  if (g_drive_rate.ready(now_ms)) {
    g_drive.tick(now_ms);
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

    // Drive telemetry
    const auto& drive_state = g_drive.getState();
    if (drive_state.valid_feedback) {
      t.wheel.left_rpm  = drive_state.meas_left_rpm;
      t.wheel.right_rpm = drive_state.meas_right_rpm;
    } else {
      t.wheel.left_rpm  = NAN;
      t.wheel.right_rpm = NAN;
    }


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
