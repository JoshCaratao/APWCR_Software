// src/actuators/ServoActuator.cpp
#include "actuators/ServoActuator.h"
#include <Arduino.h>
#include <math.h>  // fabsf, NAN

ServoActuator::ServoActuator(uint8_t pin, float min_deg, float max_deg, float ramp_dps, float deadband_deg, uint32_t settle_ms, bool auto_detach_on_closed, float closed_deg)
: _pin(pin),
  _min_deg(min_deg),
  _max_deg(max_deg),
  _ramp_dps(ramp_dps < 0.0f ? 0.0f : ramp_dps),
  _deadband_deg(deadband_deg < 0.0f ? 0.0f : deadband_deg),
  _settle_ms(settle_ms),
  _auto_detach_on_closed(auto_detach_on_closed),
  _closed_deg(closed_deg) 
  {
    _closed_deg = clampDeg_(_closed_deg);
}

void ServoActuator::begin(float initial_deg) {
  const uint32_t now_ms = millis();

  _servo.attach(_pin);
  _state.is_attached = true;

  const float init = clampDeg_(initial_deg);
  _state.current_deg = init;
  _state.target_deg = init;
  _state.last_update_ms = now_ms;

  _servo.write((int)roundDeg_(init));

  // Initialize at-target bookkeeping
  _state.at_target_since_ms = now_ms;
  updateAtTargetFlags_(now_ms);
}

void ServoActuator::attach(uint32_t now_ms) {
  if (_state.is_attached) return;

  _servo.attach(_pin);
  _state.is_attached = true;

  // Immediately output current position to avoid jumps
  _servo.write((int)roundDeg_(clampDeg_(_state.current_deg)));

  // Reset timing so next tick has a sane dt
  _state.last_update_ms = now_ms;
}

void ServoActuator::detach() {
  if (!_state.is_attached) return;
  _servo.detach();
  _state.is_attached = false;
}

void ServoActuator::setRampDps(float ramp_dps) {
  _ramp_dps = (ramp_dps < 0.0f) ? 0.0f : ramp_dps;
}

void ServoActuator::setAutoDetachOnClosed(bool enable, float closed_deg) {
  _auto_detach_on_closed = enable;
  _closed_deg = clampDeg_(closed_deg);
}

void ServoActuator::setSettleParams(float deadband_deg, uint32_t settle_ms) {
  _deadband_deg = (deadband_deg < 0.0f) ? 0.0f : deadband_deg;
  _settle_ms = settle_ms;
}

void ServoActuator::setTargetDeg(float deg, uint32_t now_ms) {
  const float new_target = clampDeg_(deg);

  // If target is unchanged (within a tiny epsilon), do nothing
  if (fabsf(new_target - _state.target_deg) < 0.001f) {
    return;
  }

  _state.target_deg = new_target;

  // If we need motion, ensure we're attached
  // (especially important if we previously auto-detached)
  attach(now_ms);

  // Reset settle timer bookkeeping when target changes
  _state.at_target_since_ms = 0;
  _state.at_target = false;

  // If ramp disabled, snap immediately
  if (_ramp_dps <= 0.0f) {
    _state.current_deg = _state.target_deg;
    if (_state.is_attached) {
      _servo.write((int)roundDeg_(_state.current_deg));
    }
    _state.last_update_ms = now_ms;

    // Update at-target bookkeeping right away
    _state.at_target_since_ms = now_ms;
    updateAtTargetFlags_(now_ms);
  }
}

void ServoActuator::tick(uint32_t now_ms) {
  // If detached, nothing to do (we detach only when we do not need holding torque)
  if (!_state.is_attached) return;

  // Compute dt
  uint32_t dt_ms = now_ms - _state.last_update_ms;
  _state.last_update_ms = now_ms;
  if (dt_ms == 0) return;

  // If ramp disabled, we should already have snapped in setTargetDeg()
  if (_ramp_dps <= 0.0f) {
    updateAtTargetFlags_(now_ms);
    return;
  }

  // Move current toward target by at most (ramp_dps * dt)
  const float tgt = _state.target_deg;
  float cur = _state.current_deg;

  const float err = tgt - cur;
  const float dt_s = (float)dt_ms / 1000.0f;
  const float max_step = _ramp_dps * dt_s;

  if (fabsf(err) <= 0.0001f) {
    // already there
    cur = tgt;
  } else if (err > 0.0f) {
    cur = (cur + max_step >= tgt) ? tgt : (cur + max_step);
  } else {
    cur = (cur - max_step <= tgt) ? tgt : (cur - max_step);
  }

  cur = clampDeg_(cur);
  _state.current_deg = cur;

  _servo.write((int)roundDeg_(cur));

  // Update at-target bookkeeping and potentially detach
  updateAtTargetFlags_(now_ms);

  if (_auto_detach_on_closed) {
    // Consider "closed" reached if target is basically the closed setpoint,
    // and we're at target (within deadband) for settle_ms.
    const bool target_is_closed = (fabsf(_state.target_deg - _closed_deg) <= _deadband_deg);

    if (target_is_closed && _state.at_target) {
      if (_state.at_target_since_ms != 0 && (now_ms - _state.at_target_since_ms) >= _settle_ms) {
        detach();
      }
    }
  }
}

void ServoActuator::updateAtTargetFlags_(uint32_t now_ms) {
  const float err = _state.target_deg - _state.current_deg;
  const bool now_at_target = (fabsf(err) <= _deadband_deg);

  if (now_at_target) {
    if (!_state.at_target) {
      // Just entered at-target region
      _state.at_target_since_ms = now_ms;
    }
    _state.at_target = true;
  } else {
    _state.at_target = false;
    _state.at_target_since_ms = 0;
  }
}

float ServoActuator::clampDeg_(float deg) const {
  if (deg < _min_deg) return _min_deg;
  if (deg > _max_deg) return _max_deg;
  return deg;
}

int ServoActuator::roundDeg_(float deg) const {
  deg = clampDeg_(deg);
  return (int)(deg + 0.5f);  // deg is non-negative
}

