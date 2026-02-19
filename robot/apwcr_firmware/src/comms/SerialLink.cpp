#include "comms/SerialLink.h"

#include <string.h>
#include <stdarg.h>

#include "comms/Protocol.h"

/*
===============================================================================
  SerialLink.cpp
===============================================================================

  Key behavior:
  - Ignores '\r'
  - '\n' ends a frame
  - If RX buffer would overflow, enters "dropping" mode until next '\n'
===============================================================================
*/

SerialLink::SerialLink(Stream& serial)
: _serial(serial)
{
  memset(_rx_buf, 0, sizeof(_rx_buf));
  memset(_note_buf, 0, sizeof(_note_buf));
}

void SerialLink::begin() {
  _rx_len = 0;
  _dropping = false;

  _has_cmd = false;
  _last_cmd_ms = 0;
  _ack_seq = 0;

  _lines = _ok = _fail = _ovf = 0;
  _max_len_seen = 0;

  memset(_rx_buf, 0, sizeof(_rx_buf));
  memset(_note_buf, 0, sizeof(_note_buf));
  _note_until_ms = 0;
  note_(0, "BOOT RX_BUF_SIZE=%u", (unsigned)RX_BUF_SIZE);

}

void SerialLink::TxTick(const TelemetryFrame& t) {
  sendTelemetry(t);
}

bool SerialLink::commandTimedOut(uint32_t now_ms) const {
  if (_last_cmd_ms == 0) return true; // never received
  return (now_ms - _last_cmd_ms) > COMMAND_TIMEOUT_MS;
}

uint32_t SerialLink::commandAgeMs(uint32_t now_ms) const {
  if (_last_cmd_ms == 0) return 0xFFFFFFFFUL;
  return now_ms - _last_cmd_ms;
}

void SerialLink::sendTelemetry(const TelemetryFrame& t) {
  protocol::encodeTelemetryLine(t, _serial);
}

void SerialLink::note_(uint32_t now_ms, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(_note_buf, sizeof(_note_buf), fmt, args);
  va_end(args);
  _note_until_ms = now_ms + 1500;
}

void SerialLink::tick(uint32_t now_ms) {
  while (_serial.available() > 0) {
    int c = _serial.read();
    if (c < 0) break;

    char ch = (char)c;

    if (ch == '\r') continue;

    if (_dropping) {
      // We overflowed earlier; discard until newline to resync
      if (ch == '\n') {
        _dropping = false;
        _rx_len = 0;
        memset(_rx_buf, 0, sizeof(_rx_buf));
        // note_(now_ms, "RX RESYNC");
      }
      continue;
    }

    if (ch == '\n') {
      // End of frame
      _rx_buf[_rx_len] = '\0';

      _lines++;

      // Track max length seen (helps confirm sizing)
      if (_rx_len > _max_len_seen) _max_len_seen = (uint16_t)_rx_len;

      handleLine_(now_ms);

      _rx_len = 0;
      continue;
    }

    // Append to buffer if there is room (leave space for '\0')
    if (_rx_len + 1 < RX_BUF_SIZE) {
      _rx_buf[_rx_len++] = ch;
    } else {
      // Buffer overflow: discard remainder until newline
      _ovf++;
      _dropping = true;

      // Capture a short debug note (head + stats)
      // NOTE: _rx_buf currently holds the head of the frame, which is useful.
      _rx_buf[RX_BUF_SIZE - 1] = '\0';
      const size_t n = strlen(_rx_buf);
      const char* tail = (n > 24) ? (_rx_buf + (n - 24)) : _rx_buf;
      note_(now_ms,
            "RX FAIL lines=%lu ok=%lu fail=%lu ovf=%lu len=%u head=%.24s tail=%.24s",
            (unsigned long)_lines,
            (unsigned long)_ok,
            (unsigned long)_fail,
            (unsigned long)_ovf,
            (unsigned)_rx_len,
            _rx_buf,
            tail);

      // Reset buffer for next frame after resync
      _rx_len = 0;
      memset(_rx_buf, 0, sizeof(_rx_buf));
    }
  }
}

void SerialLink::handleLine_(uint32_t now_ms) {
  if (_rx_buf[0] == '\0') return;

  CommandFrame cmd;
  if (protocol::decodeCommandLine(_rx_buf, cmd) && cmd.valid) {
    _latest_cmd = cmd;
    _has_cmd = true;
    _last_cmd_ms = now_ms;
    _ack_seq = cmd.seq;
    _ok++;

    // Optional: success note (comment out later)
    note_(now_ms, "RX OK seq=%lu len=%u",
          (unsigned long)cmd.seq,
          (unsigned)_rx_len);

  } else {
    _fail++;

    // Show head + length so we can tell if schema/JSON is weird
    note_(now_ms,
          "RX FAIL (lines=%lu ok=%lu fail=%lu ovf=%lu) len=%u head=%.24s",
          (unsigned long)_lines,
          (unsigned long)_ok,
          (unsigned long)_fail,
          (unsigned long)_ovf,
          (unsigned)_rx_len,
          _rx_buf);
  }
}
