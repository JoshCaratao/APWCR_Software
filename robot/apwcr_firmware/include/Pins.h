#pragma once
#include <Arduino.h>

/*
  Pins.h

  Purpose:
  Central location for all Arduino pin assignments for the APWCR robot.
  Keeps hardware mapping explicit, readable, and easy to modify.

  Board:
  Arduino Mega 2560

  Notes:
  - External interrupts are prioritized for encoder Channel A
  - Motor drivers use PWM + direction (DRV8871)
  - Servos use analog pins as digital outputs
  - Ultrasonic uses regular digital GPIO
*/

/* ============================================================================
   DRV8871 MOTOR DRIVER PINS
   IN1 = Direction
   IN2 = PWM
============================================================================ */

// LHS Arm Motor
constexpr uint8_t PIN_LHS_ARM_DIR = 32;   // IN1
constexpr uint8_t PIN_LHS_ARM_PWM = 9;    // IN2 (PWM)

// RHS Arm Motor
constexpr uint8_t PIN_RHS_ARM_DIR = 33;   // IN1
constexpr uint8_t PIN_RHS_ARM_PWM = 10;   // IN2 (PWM)

// LHS Drive Motor
constexpr uint8_t PIN_LHS_DRIVE_DIR = 30; // IN1
constexpr uint8_t PIN_LHS_DRIVE_PWM = 5;  // IN2 (PWM)

// RHS Drive Motor
constexpr uint8_t PIN_RHS_DRIVE_DIR = 31; // IN1
constexpr uint8_t PIN_RHS_DRIVE_PWM = 6;  // IN2 (PWM)

/* ============================================================================
   QUADRATURE ENCODER PINS
   Channel A = External Interrupt
   Channel B = External or Pin-Change Interrupt
============================================================================ */

// LHS Arm Encoder
constexpr uint8_t PIN_ENC_LHS_ARM_A = 18; // INT5 (TX1)
constexpr uint8_t PIN_ENC_LHS_ARM_B = 22; // GPIO / PCINT

// RHS Arm Encoder
constexpr uint8_t PIN_ENC_RHS_ARM_A = 19; // INT4 (RX1)
constexpr uint8_t PIN_ENC_RHS_ARM_B = 23; // GPIO / PCINT

// LHS Drive Encoder
constexpr uint8_t PIN_ENC_LHS_DRIVE_A = 2;  // INT0
constexpr uint8_t PIN_ENC_LHS_DRIVE_B = 20; // INT1

// RHS Drive Encoder
constexpr uint8_t PIN_ENC_RHS_DRIVE_A = 3;  // INT1
constexpr uint8_t PIN_ENC_RHS_DRIVE_B = 21; // INT2

/* ============================================================================
   ULTRASONIC DISTANCE SENSOR (HC-SR04)
============================================================================ */
// NOTE: Screw-terminal shield mapping for D24/D25 was unreliable.
// Ultrasonic validated working on D7/D8.

constexpr uint8_t PIN_ULTRASONIC_TRIG = 8;
constexpr uint8_t PIN_ULTRASONIC_ECHO = 7;

/* ============================================================================
   SERVO SIGNAL PINS
   Analog pins used as digital outputs
============================================================================ */

constexpr uint8_t PIN_SERVO_LID   = A0;
constexpr uint8_t PIN_SERVO_SWEEP = A1;

/* ============================================================================
   SERIAL INTERFACES
============================================================================ */

// USB Serial (Laptop ↔ Arduino)
#define SERIAL_USB Serial

// Reserved hardware serials (not currently used)
// Serial1 -> D19 (RX1), D18 (TX1)
// Serial2 -> D17 (RX2), D16 (TX2)
// Serial3 -> D15 (RX3), D14 (TX3)
