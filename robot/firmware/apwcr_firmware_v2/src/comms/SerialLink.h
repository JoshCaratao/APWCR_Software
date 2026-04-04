#pragma once
#include <Arduino.h>

#include "Params.h"
#include "comms/Messages.h"

/*
===============================================================================
  SerialLink.h
===============================================================================

  PURPOSE
  -------
  Arduino-side serial link handler:

    - Non-blocking read from Stream
    - Accumulate bytes into a newline-delimited line buffer
    - Decode "cmd" frames and store latest valid command
    - Track command age for COMMAND_TIMEOUT_MS
    - Send telemetry frames via Protocol

  IMPORTANT
  ---------
  On RX buffer overflow, this class will DISCARD bytes until the next '\n'
  to resynchronize cleanly. This prevents "tail fragments" from being decoded.

===============================================================================
*/

class SerialLink {
public:
  explicit SerialLink(Stream& serial);

  void begin();

  // Call frequently (e.g., 20–100 Hz). Reads any available bytes and decodes
  // complete lines. Never blocks waiting for input.
  void tick(uint32_t now_ms);

  // Convenience aliases
  void RxTick(uint32_t now_ms) { tick(now_ms); }
  void TxTick(const TelemetryFrame& t);

  // True if at least one valid command has been received since boot.
  bool hasCommand() const { return _has_cmd; }

  // Latest successfully decoded command (only meaningful if hasCommand()).
  const CommandFrame& latestCommand() const { return _latest_cmd; }

  // True if we have not received a command recently.
  bool commandTimedOut(uint32_t now_ms) const;

  // Time since last command was received (ms). If never received, returns large.
  uint32_t commandAgeMs(uint32_t now_ms) const;

  // ACK = last command seq that was received + parsed successfully
  uint32_t ackSeq() const { return _ack_seq; }

  // Encodes and writes one telemetry line to the serial stream.
  void sendTelemetry(const TelemetryFrame& t);

  // Optional: expose a short RX debug note (valid until _note_until_ms)
  const char* debugNote(uint32_t now_ms) const {
    return (now_ms <= _note_until_ms) ? _note_buf : nullptr;
  }

  // Optional: RX stats
  uint32_t rxLines() const { return _lines; }
  uint32_t rxOk() const { return _ok; }
  uint32_t rxFail() const { return _fail; }
  uint32_t rxOverflow() const { return _ovf; }
  uint16_t rxMaxLenSeen() const { return _max_len_seen; }

private:
  void handleLine_(uint32_t now_ms);
  void note_(uint32_t now_ms, const char* fmt, ...);

  Stream& _serial;

  static constexpr size_t RX_BUF_SIZE = SERIAL_LINE_BUFFER_BYTES;
  char _rx_buf[RX_BUF_SIZE];
  size_t _rx_len = 0;

  // When true, we are discarding bytes until newline due to overflow
  bool _dropping = false;

  // Latest decoded command
  CommandFrame _latest_cmd;
  bool _has_cmd = false;

  // Command freshness
  uint32_t _last_cmd_ms = 0;

  // ACK bookkeeping
  uint32_t _ack_seq = 0;

  // RX debug stats
  uint32_t _lines = 0;
  uint32_t _ok = 0;
  uint32_t _fail = 0;
  uint32_t _ovf = 0;
  uint16_t _max_len_seen = 0;

  // Debug note buffer (for telemetry note)
  char _note_buf[96];
  uint32_t _note_until_ms = 0;
};
