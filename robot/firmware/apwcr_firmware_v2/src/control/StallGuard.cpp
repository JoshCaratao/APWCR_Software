#include "control/StallGuard.h"
#include <stdlib.h>
#include <math.h>

int8_t StallGuard::sign_(float x) {
  if (x > 0.0f) return 1;
  if (x < 0.0f) return -1;
  return 0;
}

void StallGuard::reset() {
  _faulted = false;
  _fault_dir = 0;
  _watching = false;
  _watch_dir = 0;
  _watch_start_ms = 0;
  _watch_start_count = 0;
  _zero_seen = false;
  _zero_start_ms = 0;
}

float StallGuard::apply(float duty,
                        float target_rpm,
                        int32_t encoder_count,
                        uint32_t now_ms,
                        const StallGuardConfig& cfg) {
  if (!cfg.enabled) {
    reset();
    return duty;
  }

  const bool zero_command = fabsf(target_rpm) < cfg.target_rpm_threshold;
  const bool enough_duty = fabsf(duty) >= cfg.duty_threshold;

  const int8_t duty_dir = sign_(duty);

  if (_faulted) {
    if (zero_command) {
      if (!_zero_seen) {
        _zero_seen = true;
        _zero_start_ms = now_ms;
      }

      if ((now_ms - _zero_start_ms) >= cfg.reset_zero_ms) {
        reset();
      }
      return 0.0f;
    }

    _zero_seen = false;

    if (duty_dir != 0 && duty_dir != _fault_dir) {
      // Allow the motor to back out of a jam. If that direction is also stuck,
      // the fresh watch window below can trip a new fault.
      reset();
    } else {
      return 0.0f;
    }
  }

  if (zero_command || !enough_duty || duty_dir == 0) {
    _watching = false;
    return duty;
  }

  if (!_watching || _watch_dir != duty_dir) {
    _watching = true;
    _watch_dir = duty_dir;
    _watch_start_ms = now_ms;
    _watch_start_count = encoder_count;
    return duty;
  }

  const int32_t count_delta = labs(encoder_count - _watch_start_count);
  if (count_delta >= cfg.min_count_delta) {
    // Movement happened. Start a new window from the current position.
    _watch_start_ms = now_ms;
    _watch_start_count = encoder_count;
    return duty;
  }

  if ((now_ms - _watch_start_ms) >= cfg.timeout_ms) {
    _faulted = true;
    _fault_dir = duty_dir;
    _watching = false;
    return 0.0f;
  }

  return duty;
}
