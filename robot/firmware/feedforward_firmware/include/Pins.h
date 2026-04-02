#pragma once
#include <Arduino.h>

/*
  Pins.h

  Purpose:
  Central location for the feedforward-model-test firmware pin assignments.
  This test firmware only needs motor, encoder, and USB serial mappings.

  Board:
  Arduino Mega 2560
*/

/* ============================================================================
   MOTOR DRIVER PINS
   IN1 = Direction
   IN2 = PWM
============================================================================ */

// Drive motors
constexpr uint8_t PIN_LHS_DRIVE_DIR = 4;
constexpr uint8_t PIN_LHS_DRIVE_PWM = 5;
constexpr uint8_t PIN_RHS_DRIVE_DIR = 13;
constexpr uint8_t PIN_RHS_DRIVE_PWM = 6;

// Mechanism motors
constexpr uint8_t PIN_LHS_MECH_DIR = 11;
constexpr uint8_t PIN_LHS_MECH_PWM = 9;
constexpr uint8_t PIN_RHS_MECH_DIR = 12;
constexpr uint8_t PIN_RHS_MECH_PWM = 10;

/* ============================================================================
   QUADRATURE ENCODER PINS
   Channel A is kept on interrupt-capable pins as in deployed firmware.
============================================================================ */

// Drive encoders
constexpr uint8_t PIN_ENC_LHS_DRIVE_A = 2;
constexpr uint8_t PIN_ENC_LHS_DRIVE_B = 20;
constexpr uint8_t PIN_ENC_RHS_DRIVE_A = 3;
constexpr uint8_t PIN_ENC_RHS_DRIVE_B = 21;

// Mechanism encoders
constexpr uint8_t PIN_ENC_LHS_MECH_A = 18;
constexpr uint8_t PIN_ENC_LHS_MECH_B = 22;
constexpr uint8_t PIN_ENC_RHS_MECH_A = 19;
constexpr uint8_t PIN_ENC_RHS_MECH_B = 23;

/* ============================================================================
   SERIAL INTERFACE
============================================================================ */

// USB serial used by the Python feedforward test harness.
#define SERIAL_USB Serial
