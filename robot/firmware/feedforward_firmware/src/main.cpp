/*
  Feedforward Motor Identification Firmware

  Purpose:
  This firmware runs on the Arduino Mega as a lightweight motor identification
  target for the Python feedforward-model-test harness. It accepts direct motor
  commands over serial, samples output-side encoder RPM, and publishes compact
  telemetry back to the host.

  Responsibilities:
  - Decode newline-delimited JSON command frames
  - Apply the latest command to the four motor channels
  - Sample encoder position and speed at a fixed rate
  - Publish command echo + RPM telemetry at a fixed rate

  Not responsible for:
  - Autonomous robot control
  - Mechanism sequencing
  - Ultrasonic or servo handling
  - Complex link-state management
*/

#include <Arduino.h>

#include "Pins.h"
#include "Params.h"

#include "actuators/DcMotorActuator.h"
#include "comms/Messages.h"
#include "comms/Protocol.h"
#include "sensors/EncoderSensor.h"
#include "utils/Rate.h"

/*=============================================================================
  GLOBALS
=============================================================================*/

DcMotorActuator g_drive_lhs(
  PIN_LHS_DRIVE_DIR,
  PIN_LHS_DRIVE_PWM,
  DRIVE_INVERT_LHS_MOTOR,
  PWM_MIN,
  PWM_MAX
);

DcMotorActuator g_drive_rhs(
  PIN_RHS_DRIVE_DIR,
  PIN_RHS_DRIVE_PWM,
  DRIVE_INVERT_RHS_MOTOR,
  PWM_MIN,
  PWM_MAX
);

DcMotorActuator g_mech_rhs(
  PIN_RHS_MECH_DIR,
  PIN_RHS_MECH_PWM,
  MECH_INVERT_RHS_MOTOR,
  PWM_MIN,
  PWM_MAX
);

DcMotorActuator g_mech_lhs(
  PIN_LHS_MECH_DIR,
  PIN_LHS_MECH_PWM,
  MECH_INVERT_LHS_MOTOR,
  PWM_MIN,
  PWM_MAX
);

EncoderSensor g_enc_drive_lhs(
  PIN_ENC_LHS_DRIVE_A,
  PIN_ENC_LHS_DRIVE_B,
  COUNTS_PER_DRIVE_OUTPUT_REV,
  DRIVE_INVERT_LHS_ENCODER
);

EncoderSensor g_enc_drive_rhs(
  PIN_ENC_RHS_DRIVE_A,
  PIN_ENC_RHS_DRIVE_B,
  COUNTS_PER_DRIVE_OUTPUT_REV,
  DRIVE_INVERT_RHS_ENCODER
);

EncoderSensor g_enc_mech_rhs(
  PIN_ENC_RHS_MECH_A,
  PIN_ENC_RHS_MECH_B,
  COUNTS_PER_MECH_RHS_OUTPUT_REV,
  MECH_INVERT_RHS_ENCODER
);

EncoderSensor g_enc_mech_lhs(
  PIN_ENC_LHS_MECH_A,
  PIN_ENC_LHS_MECH_B,
  COUNTS_PER_MECH_LHS_OUTPUT_REV,
  MECH_INVERT_LHS_ENCODER
);

Rate g_rx_rate(RX_COMM_UPDATE_HZ);
Rate g_encoder_rate(ENCODER_SAMPLE_HZ);
Rate g_telemetry_rate(TELEMETRY_UPDATE_HZ);

CommandFrame g_latest_cmd;
uint32_t g_last_cmd_ms = 0;
uint32_t g_last_ack_seq = 0;

char g_rx_buf[SERIAL_LINE_BUFFER_BYTES];
size_t g_rx_len = 0;
bool g_rx_dropping = false;

/*=============================================================================
  HELPERS
=============================================================================*/

float clampDuty(float duty) {
  // Clamp host commands into the supported normalized duty range.
  if (duty > MAX_ABS_DUTY) return MAX_ABS_DUTY;
  if (duty < -MAX_ABS_DUTY) return -MAX_ABS_DUTY;
  return duty;
}

void applyCommand(const CommandFrame& cmd) {
  // Apply the latest decoded motor commands directly to each channel.
  g_drive_lhs.setDuty(clampDuty(cmd.motors.drive_lhs_cmd));
  g_drive_rhs.setDuty(clampDuty(cmd.motors.drive_rhs_cmd));
  g_mech_rhs.setDuty(clampDuty(cmd.motors.mech_rhs_cmd));
  g_mech_lhs.setDuty(clampDuty(cmd.motors.mech_lhs_cmd));
}

void applyZeroCommand() {
  // Force all four motor channels to a safe zero-duty state.
  g_drive_lhs.setDuty(0.0f);
  g_drive_rhs.setDuty(0.0f);
  g_mech_rhs.setDuty(0.0f);
  g_mech_lhs.setDuty(0.0f);
}

void handleLine(const char* line, uint32_t now_ms) {
  // Decode one full JSON command line and apply it if valid.
  CommandFrame cmd;
  if (!protocol::decodeCommandLine(line, cmd) || !cmd.valid) {
    return;
  }

  g_latest_cmd = cmd;
  g_last_cmd_ms = now_ms;
  g_last_ack_seq = cmd.seq;
  applyCommand(g_latest_cmd);
}

void rxTick(uint32_t now_ms) {
  // Accumulate serial bytes until newline, then decode one command frame.
  while (SERIAL_USB.available() > 0) {
    const int c = SERIAL_USB.read();
    if (c < 0) break;

    const char ch = (char)c;
    if (ch == '\r') continue;

    if (g_rx_dropping) {
      if (ch == '\n') {
        g_rx_dropping = false;
        g_rx_len = 0;
      }
      continue;
    }

    if (ch == '\n') {
      g_rx_buf[g_rx_len] = '\0';
      if (g_rx_len > 0) {
        handleLine(g_rx_buf, now_ms);
      }
      g_rx_len = 0;
      continue;
    }

    if (g_rx_len + 1 < SERIAL_LINE_BUFFER_BYTES) {
      g_rx_buf[g_rx_len++] = ch;
    } else {
      g_rx_dropping = true;
      g_rx_len = 0;
    }
  }
}

void sampleEncoders(uint32_t now_ms) {
  // Update all encoder position and speed states from the latest counts.
  g_enc_drive_lhs.sample(now_ms);
  g_enc_drive_rhs.sample(now_ms);
  g_enc_mech_rhs.sample(now_ms);
  g_enc_mech_lhs.sample(now_ms);
}

float rpmOrNan(const EncoderSensor::State& state) {
  // Report RPM only when the latest speed estimate is valid.
  return state.valid_speed ? state.rpm : NAN;
}

void txTelemetry(uint32_t now_ms) {
  // Send the latest command echo and output-side RPM telemetry to the host.
  TelemetryFrame t;
  t.arduino_time_ms = now_ms;
  t.ack_seq = g_last_ack_seq;

  t.motors.drive_lhs_cmd = g_drive_lhs.dutyCmd();
  t.motors.drive_rhs_cmd = g_drive_rhs.dutyCmd();
  t.motors.mech_rhs_cmd = g_mech_rhs.dutyCmd();
  t.motors.mech_lhs_cmd = g_mech_lhs.dutyCmd();

  t.motors.drive_lhs_rpm = rpmOrNan(g_enc_drive_lhs.getState());
  t.motors.drive_rhs_rpm = rpmOrNan(g_enc_drive_rhs.getState());
  t.motors.mech_rhs_rpm = rpmOrNan(g_enc_mech_rhs.getState());
  t.motors.mech_lhs_rpm = rpmOrNan(g_enc_mech_lhs.getState());

  protocol::encodeTelemetryLine(t, SERIAL_USB);
}

/*=============================================================================
  SETUP
=============================================================================*/

void setup() {
  // Initialize serial, motors, and encoders into a safe zero-command state.
  SERIAL_USB.begin(SERIAL_BAUD);

  g_drive_lhs.begin();
  g_drive_rhs.begin();
  g_mech_rhs.begin();
  g_mech_lhs.begin();

  g_enc_drive_lhs.begin();
  g_enc_drive_rhs.begin();
  g_enc_mech_rhs.begin();
  g_enc_mech_lhs.begin();

  applyZeroCommand();
}

/*=============================================================================
  LOOP
=============================================================================*/

void loop() {
  // Run the minimal command, sampling, and telemetry schedule.
  const uint32_t now_ms = millis();

  if (g_rx_rate.ready(now_ms)) {
    rxTick(now_ms);
  }

  if (g_encoder_rate.ready(now_ms)) {
    sampleEncoders(now_ms);
  }

  if (g_last_cmd_ms != 0 && (now_ms - g_last_cmd_ms) > COMMAND_TIMEOUT_MS) {
    applyZeroCommand();
  }

  if (g_telemetry_rate.ready(now_ms)) {
    txTelemetry(now_ms);
  }
}
