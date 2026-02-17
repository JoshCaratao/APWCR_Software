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
  Owns Arduino-side serial I/O behavior (similar to Python serial_link.py):

    - Read bytes from Serial (non-blocking)
    - Build newline-delimited lines
    - Decode "cmd" frames via protocol::decodeCommandLine()
    - Store the latest valid command frame
    - Track last command receive time (for COMMAND_TIMEOUT_MS)
    - Provide helpers to send telemetry frames

  Notes:
  - This class does NOT schedule itself.
  - main.cpp calls tick(now_ms) using your Rate class.
===============================================================================
*/

class SerialLink {
public:
  /*---------------------------------------------------------------------------
    Construction / setup
  ---------------------------------------------------------------------------*/

  explicit SerialLink(Stream& serial);

  void begin();

    /*---------------------------------------------------------------------------
    Tick
  ---------------------------------------------------------------------------*/

  // Call frequently (e.g., 20–100 Hz). Reads any available bytes and decodes
  // complete lines. Never blocks waiting for input.
  void tick(uint32_t now_ms);

  /*---------------------------------------------------------------------------
    RX / TX Ticks
  ---------------------------------------------------------------------------*/

  // RX tick: reads serial bytes and decodes any complete command lines.
  void RxTick(uint32_t now_ms);

  // TX tick: sends one telemetry frame (one JSON line).
  void TxTick(const TelemetryFrame& t);


  /*---------------------------------------------------------------------------
    Command access
  ---------------------------------------------------------------------------*/

  // True if at least one valid command has been received since boot.
  bool hasCommand() const { return _has_cmd; }

  // Latest successfully decoded command (only meaningful if hasCommand()).
  const CommandFrame& latestCommand() const { return _latest_cmd; }

  // True if we have not received a command recently.
  bool commandTimedOut(uint32_t now_ms) const;

  // Convenience: time since last command was received (ms). If never received,
  // returns a large value.
  uint32_t commandAgeMs(uint32_t now_ms) const;


  /*---------------------------------------------------------------------------
    ACK tracking
  ---------------------------------------------------------------------------*/

  // ACK = last command seq that was received + parsed successfully
  uint32_t ackSeq() const { return _ack_seq; }


  /*---------------------------------------------------------------------------
    Telemetry send
  ---------------------------------------------------------------------------*/

  // Encodes and writes one telemetry line to the serial stream.
  void sendTelemetry(const TelemetryFrame& t);

private:
  void handleLine_(uint32_t now_ms);

  Stream& _serial;

  // Fixed RX line buffer (newline-delimited JSON)
  static constexpr size_t RX_BUF_SIZE = SERIAL_LINE_BUFFER_BYTES;
  char _rx_buf[RX_BUF_SIZE];
  size_t _rx_len = 0;

  // Latest decoded command
  CommandFrame _latest_cmd;
  bool _has_cmd = false;

  // Command freshness
  uint32_t _last_cmd_ms = 0;

  // ACK bookkeeping (last received + parsed cmd seq)
  uint32_t _ack_seq = 0;

};
