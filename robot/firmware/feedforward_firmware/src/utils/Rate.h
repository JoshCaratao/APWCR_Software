#pragma once

#include <Arduino.h>

/*
===============================================================================
  Rate.h
===============================================================================

  Purpose:
  Small non-blocking helper for running periodic tasks from the main loop.

  Responsibilities:
  - Convert a requested rate or period into milliseconds
  - Track the next scheduled run time
  - Report when a task is ready to run

  Typical usage:
  - Create one Rate object per periodic task
  - Call ready(now_ms) inside loop()
  - Run the task only when ready(...) returns true
===============================================================================
*/

class Rate {
public:
  // Construct from a target frequency in Hz.
  explicit Rate(uint16_t hz = 1) { setHz(hz); }

  // Set the task frequency in Hz.
  void setHz(uint16_t hz) {
    if (hz == 0) hz = 1;
    _period_ms = (uint32_t)(1000UL / hz);
    if (_period_ms == 0) _period_ms = 1;
  }

  // Set the task period directly in milliseconds.
  void setPeriodMs(uint32_t period_ms) {
    _period_ms = (period_ms == 0) ? 1 : period_ms;
  }

  // Return true when the task should run, and schedule the next deadline.
  bool ready(uint32_t now_ms) {
    if (!_initialized) {
      _next_ms = now_ms;  // run immediately on the first call
      _initialized = true;
    }

    // Signed subtraction keeps this safe across millis() rollover.
    if ((int32_t)(now_ms - _next_ms) >= 0) {
      _next_ms = now_ms + _period_ms;
      return true;
    }
    return false;
  }

  uint32_t periodMs() const { return _period_ms; }
  uint32_t nextMs() const { return _next_ms; }

private:
  uint32_t _period_ms = 1000;
  uint32_t _next_ms = 0;
  bool _initialized = false;
};
