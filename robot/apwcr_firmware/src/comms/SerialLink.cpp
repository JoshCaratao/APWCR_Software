#include "comms/SerialLink.h"

/*
===============================================================================
  SerialLink.cpp
===============================================================================

  PURPOSE
  -------
  Arduino-side serial link handler (similar to Python serial_link.py):

    - Non-blocking read from Stream
    - Accumulate bytes into a newline-delimited line buffer
    - Decode "cmd" frames and store latest valid command
    - Track command age for COMMAND_TIMEOUT_MS
    - Send telemetry frames via Protocol

  Behavior:
    - Ignores '\r'
    - Uses '\n' as end-of-frame
    - If a line exceeds the buffer, drops it to resynchronize
===============================================================================
*/

#include <string.h>

#include "comms/Protocol.h"


/*=============================================================================
  PUBLIC API
=============================================================================*/

SerialLink::SerialLink(Stream& serial)
: _serial(serial)
{
  memset(_rx_buf, 0, sizeof(_rx_buf));
}

void SerialLink::begin() {
  _rx_len = 0;
  _has_cmd = false;
  _last_cmd_ms = 0;
  _ack_seq = 0;
  memset(_rx_buf, 0, sizeof(_rx_buf));
}

void SerialLink::tick(uint32_t now_ms) {
  while (_serial.available() > 0) {
    int c = _serial.read();
    if (c < 0) break;

    char ch = (char)c;

    // Ignore carriage returns (Windows line endings)
    if (ch == '\r') continue;

    // Newline ends the current frame
    if (ch == '\n') {
      _rx_buf[_rx_len] = '\0';
      handleLine_(now_ms);
      _rx_len = 0;
      continue;
    }

    // Append to buffer if there is room (leave space for '\0')
    if (_rx_len + 1 < RX_BUF_SIZE) {
      _rx_buf[_rx_len++] = ch;
    } else {
      // Buffer overflow: drop this line and wait for next newline
      _rx_len = 0;
    }
  }
}

void SerialLink::RxTick(uint32_t now_ms) {
  tick(now_ms);
}

void SerialLink::TxTick(const TelemetryFrame& t) {
  sendTelemetry(t);
}


bool SerialLink::commandTimedOut(uint32_t now_ms) const {
  // If we've never received a command, treat as timed out
  if (_last_cmd_ms == 0) return true;
  return (now_ms - _last_cmd_ms) > COMMAND_TIMEOUT_MS;
}

uint32_t SerialLink::commandAgeMs(uint32_t now_ms) const {
  if (_last_cmd_ms == 0) return 0xFFFFFFFFUL;
  return now_ms - _last_cmd_ms;
}

void SerialLink::sendTelemetry(const TelemetryFrame& t) {
  protocol::encodeTelemetryLine(t, _serial);
}


/*=============================================================================
  PRIVATE HELPERS
=============================================================================*/

void SerialLink::handleLine_(uint32_t now_ms) {
  if (_rx_buf[0] == '\0') return;

  CommandFrame cmd;
  if (protocol::decodeCommandLine(_rx_buf, cmd) && cmd.valid) {
    _latest_cmd = cmd;
    _has_cmd = true;
    _last_cmd_ms = now_ms;

    // ACK = received + parsed (comms-level)
    _ack_seq = cmd.seq;
  }
}
