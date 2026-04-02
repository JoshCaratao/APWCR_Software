/* ============================================================================
   APWCR Dashboard GUI Script (gui.js)
============================================================================ */

/* ============================================================================
   0) Config
   Read GUI config that Flask embeds into gui.html.
============================================================================ */

function getGuiConfig() {
  const el = document.getElementById("guiConfig");
  if (!el) return {};

  try {
    return JSON.parse(el.textContent || "{}");
  } catch {
    return {};
  }
}

const cfg = getGuiConfig();

// Default teleop speeds. Units: linear ft/s, angular deg/s.
const LIN = Number(cfg.manual_speed_linear ?? 0.5);
const ANG = Number(cfg.manual_speed_angular ?? 5.0);

/* ============================================================================
   1) Small DOM Helpers
   Safe helpers so the refresh loops stay compact and readable.
============================================================================ */

function setDot(mode) {
  const dot = document.getElementById("statusDot");
  if (!dot) return;

  dot.classList.remove("ok");
  dot.classList.remove("bad");
  if (mode === "ok") dot.classList.add("ok");
  if (mode === "bad") dot.classList.add("bad");
}

function setText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function setHtml(id, html) {
  const el = document.getElementById(id);
  if (el) el.innerHTML = html;
}

function setHidden(id, hidden) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.toggle("hidden", !!hidden);
}

function setClass(id, className, enabled) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.toggle(className, !!enabled);
}

function setTeleopEnabled(enabled) {
  const pad = document.getElementById("teleopPad");
  if (!pad) return;
  pad.classList.toggle("disabled", !enabled);
}

function setMechEnabled(enabled) {
  const el = document.getElementById("mech-quick");
  if (!el) return;
  el.classList.toggle("disabled", !enabled);
}

function setModeButtonActive(stateStr) {
  const btnManual = document.getElementById("btnModeManual");
  const btnAuto = document.getElementById("btnModeAuto");
  if (!btnManual || !btnAuto) return;

  const isManual = stateStr === "MANUAL";
  btnManual.classList.toggle("active", isManual);
  btnAuto.classList.toggle("active", !isManual);
  setTeleopEnabled(isManual);
  setMechEnabled(isManual);
}

/* ============================================================================
   2) Formatting + Nested Metric Tiles
   Keep summary displays consistent across control and telemetry windows.
============================================================================ */

function fmtHz(v) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return `${n.toFixed(1)} Hz`;
}

function fmtNum(v, digits = 0) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return n.toFixed(digits);
}

function fmtFt(v, digits = 2) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return `${n.toFixed(digits)} ft`;
}

function fmtAgeSec(v, digits = 2) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return `${n.toFixed(digits)} s`;
}

function fmtUltrasonic(u) {
  if (!u) return "N/A";
  if (!Boolean(u.valid)) return "INVALID";

  const d = Number(u.distance_in);
  if (!Number.isFinite(d)) return "N/A";
  return `${d.toFixed(1)} in`;
}

function renderMetricTile(label, value, { mono = false } = {}) {
  const valueClass = mono ? "metric-value mono" : "metric-value";
  return `
    <div class="metric-tile">
      <div class="metric-label">${label}</div>
      <div class="${valueClass}">${value}</div>
    </div>
  `;
}

function renderMetricGrid(items, { columns = 2 } = {}) {
  let gridClass = "metric-grid";
  if (columns === 3) gridClass = "metric-grid metric-grid-3";
  if (columns === 4) gridClass = "metric-grid metric-grid-4";
  return `<div class="${gridClass}">${items.join("")}</div>`;
}

function formatMotorCommand(cmd) {
  if (!cmd) return "N/A";
  return `${cmd.mode ?? "N/A"} @ ${fmtNum(cmd.value, 2)}`;
}

function renderDriveCmd(cmd) {
  return renderMetricGrid([
    renderMetricTile("Linear Speed", `${fmtNum(cmd?.linear, 2)} ft/s`, { mono: true }),
    renderMetricTile("Turn Speed", `${fmtNum(cmd?.angular, 2)} deg/s`, { mono: true }),
  ]);
}

function renderMechCmd(mech) {
  return renderMetricGrid([
    renderMetricTile("Bucket Lift Motor", formatMotorCommand(mech?.motor_RHS), { mono: true }),
    renderMetricTile("Bucket Rotation Motor", formatMotorCommand(mech?.motor_LHS), { mono: true }),
    renderMetricTile("LID Servo", `${fmtNum(mech?.servo_LID_deg, 1)} deg`, { mono: true }),
    renderMetricTile("Sweeper Servo", `${fmtNum(mech?.servo_SWEEP_deg, 1)} deg`, { mono: true }),
  ]);
}

function renderWheelState(wheel) {
  return `
    <div class="metric-grid mech-debug-columns">
      <div class="metric-stack">
        ${renderMetricTile("Left Wheel RPM", `${fmtNum(wheel?.left_rpm, 1)} rpm`, { mono: true })}
        ${renderMetricTile("Left Wheel Duty", fmtNum(wheel?.left_duty, 2), { mono: true })}
      </div>
      <div class="metric-stack">
        ${renderMetricTile("Right Wheel RPM", `${fmtNum(wheel?.right_rpm, 1)} rpm`, { mono: true })}
        ${renderMetricTile("Right Wheel Duty", fmtNum(wheel?.right_duty, 2), { mono: true })}
      </div>
    </div>
  `;
}

function renderMechState(mech) {
  const generalItems = [
    renderMetricTile("LID", `${fmtNum(mech?.servo_LID_deg, 1)} deg`, { mono: true }),
    renderMetricTile("Sweeper", `${fmtNum(mech?.servo_SWEEP_deg, 1)} deg`, { mono: true }),
  ];

  const bucketLiftItems = [
    renderMetricTile("Bucket Lift Pos", `${fmtNum(mech?.motor_RHS_deg, 1)} deg`, { mono: true }),
    renderMetricTile("Bucket Lift RPM", `${fmtNum(mech?.motor_RHS_rpm, 1)} rpm`, { mono: true }),
    renderMetricTile("Bucket Lift Duty", fmtNum(mech?.motor_RHS_duty, 2), { mono: true }),
  ];

  const bucketRotItems = [
    renderMetricTile("Bucket Rot Pos", `${fmtNum(mech?.motor_LHS_deg, 1)} deg`, { mono: true }),
    renderMetricTile("Bucket Rot RPM", `${fmtNum(mech?.motor_LHS_rpm, 1)} rpm`, { mono: true }),
    renderMetricTile("Bucket Rot Duty", fmtNum(mech?.motor_LHS_duty, 2), { mono: true }),
  ];

  return `
    ${renderMetricGrid(generalItems)}
    <div class="metric-grid mech-debug-columns">
      <div class="metric-stack">
        ${bucketLiftItems.join("")}
      </div>
      <div class="metric-stack">
        ${bucketRotItems.join("")}
      </div>
    </div>
  `;
}

function renderTelemetryMeta(connection) {
  const lines = [];

  if (connection?.port) lines.push(`Port: ${connection.port}`);
  if (connection?.baud) lines.push(`Baud: ${connection.baud}`);
  if (connection?.last_rx_age_s !== null && connection?.last_rx_age_s !== undefined) {
    lines.push(`RX age: ${fmtAgeSec(connection.last_rx_age_s, 2)}`);
  }
  if (connection?.rx_stale_s !== null && connection?.rx_stale_s !== undefined) {
    lines.push(`Stale > ${fmtAgeSec(connection.rx_stale_s, 2)}`);
  }
  if (connection?.last_error) lines.push(`Err: ${connection.last_error}`);

  if (lines.length === 0) return "N/A";
  return lines.map((line) => `<div>${line}</div>`).join("");
}

/* ============================================================================
   3) HTTP Helper
============================================================================ */

async function apiPost(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  return await response.json();
}

/* ============================================================================
   4) Perception Window
   Poll /perception/status and refresh the left-side perception card.
============================================================================ */

async function refreshObs() {
  try {
    const response = await fetch("/perception/status", { cache: "no-store" });
    const data = await response.json();

    if (!data.ok) {
      setDot("bad");
      setText("subTitle", data.reason || "no data");
      setText("detectionStatusValue", "CONNECTING");
      setText("targetStatusValue", "N/A");
      setText("targetGpFwValue", "N/A");
      setText("targetGpLtValue", "N/A");
      setText("targetGpValidValue", "false");
      return;
    }

    const stable = data.target_status === "Stable Detection";
    setDot(stable ? "ok" : "");
    setText("subTitle", stable ? "STABLE" : "RUNNING");

    const detections = Number(data.num_detections ?? 0);
    setText("detectionStatusValue", detections > 0 ? "DETECTED" : "SEARCHING");
    setText("detectionsValue", String(detections));

    setText("targetInferHzValue", fmtHz(data.target_infer_hz));
    setText("measuredInferHzValue", fmtHz(data.measured_infer_hz));
    setText("targetModeValue", data.target_policy ?? "N/A");

    let targetStatus = "N/A";
    if ((data.target ?? "N/A") === "Selected") {
      targetStatus = stable ? "STABLE DETECTION" : "DETECTION";
    }
    setText("targetStatusValue", targetStatus);

    const td = data.target_data;
    if (!td) {
      setText("targetConfValue", "N/A");
      setText("targetAreaValue", "N/A");
      setText("targetCenterValue", "N/A");
    } else {
      setText("targetConfValue", fmtNum(td.conf, 2));
      setText("targetAreaValue", fmtNum(td.area, 0));
      setText("targetCenterValue", `(${fmtNum(td.cx, 0)}, ${fmtNum(td.cy, 0)})`);
    }

    const gpValid = Boolean(data.target_gp_valid);
    setText("targetGpValidValue", String(gpValid));
    setText("targetGpFwValue", gpValid ? fmtFt(data.target_gp_fw_dist, 2) : "N/A");
    setText("targetGpLtValue", gpValid ? fmtFt(data.target_gp_lt_dist, 2) : "N/A");
  } catch {
    setDot("bad");
    setText("subTitle", "disconnected");
    setText("detectionStatusValue", "DISCONNECTED");
    setText("targetStatusValue", "N/A");
  }
}

/* ============================================================================
   5) Control Window
   Poll /controller/status and refresh the center control card.
============================================================================ */

async function refreshController() {
  try {
    const response = await fetch("/controller/status", { cache: "no-store" });
    const data = await response.json();

    if (!data.ok) {
      setText("controlStateValue", "CONNECTING");
      setHtml("driveCmdValue", renderDriveCmd({ linear: 0.0, angular: 0.0 }));
      setHtml("mechCmdValue", renderMechCmd(null));
      setTeleopEnabled(false);
      setClass("controlPanel", "controlBlocked", false);
      setHidden("ultraAlert", true);
      return;
    }

    const stateStr = data.status?.state ?? "N/A";
    const blocked = Boolean(data?.status?.ultrasonic?.blocked);
    const mech =
      data?.cmd?.mech ??
      data?.mech_cmd ??
      data?.mech ??
      data?.cmd?.mechanism ??
      null;

    setText("controlStateValue", stateStr);
    setHtml("driveCmdValue", renderDriveCmd(data.cmd));
    setHtml("mechCmdValue", renderMechCmd(mech));
    setClass("controlPanel", "controlBlocked", blocked);
    setHidden("ultraAlert", !blocked);
    setModeButtonActive(stateStr);
  } catch {
    setText("controlStateValue", "DISCONNECTED");
    setHtml("driveCmdValue", renderDriveCmd({ linear: null, angular: null }));
    setHtml("mechCmdValue", renderMechCmd(null));
    setClass("controlPanel", "controlBlocked", false);
    setHidden("ultraAlert", true);
    setTeleopEnabled(false);
  }
}

async function setMode(mode) {
  try {
    await apiPost("/controller/mode", { mode });
  } catch {}
  refreshController();
}

/* ============================================================================
   6) Manual Command Helpers
   Shared helpers for teleop, arm controls, and one-shot mechanism buttons.
============================================================================ */

async function sendManualCmd(linear, angular, mech = null) {
  const body = { linear, angular };
  if (mech && typeof mech === "object") {
    body.mech = mech;
  }

  try {
    await apiPost("/controller/manual_cmd", body);
  } catch {
    // Ignore transient failures; the deadman still protects drive motion.
  }
}

// Repeat a one-shot mechanism setpoint briefly so slower comms do not miss it.
async function sendManualMech(
  { lid_deg = null, sweep_deg = null } = {},
  { durationMs = 1200, hz = 15 } = {}
) {
  const mech = {};
  if (lid_deg !== null && lid_deg !== undefined) mech.servo_LID_deg = lid_deg;
  if (sweep_deg !== null && sweep_deg !== undefined) mech.servo_SWEEP_deg = sweep_deg;
  if (Object.keys(mech).length === 0) return;

  const periodMs = Math.max(40, Math.floor(1000 / hz));
  const endTimeMs = Date.now() + durationMs;

  while (Date.now() < endTimeMs) {
    try {
      await sendManualCmd(0.0, 0.0, mech);
    } catch {}
    await new Promise((resolve) => setTimeout(resolve, periodMs));
  }
}

/* ============================================================================
   7) Telemetry Window
   Poll /telemetry/status and refresh the right-side telemetry card.
============================================================================ */

async function refreshTelemetry() {
  try {
    const response = await fetch("/telemetry/status", { cache: "no-store" });
    const data = await response.json();

    if (!data.ok) {
      setText("telConnState", "N/A");
      setHtml("telConnMeta", `<div>${data.reason || "no data"}</div>`);
      setText("telTickHz", "N/A");
      setText("telRxHz", "N/A");
      setText("telTxHz", "N/A");
      setHtml("telWheelState", renderWheelState(null));
      setHtml("telMechState", renderMechState(null));
      setText("telUltrasonic", "N/A");
      return;
    }

    const c = data.connection || {};

    setText("telConnState", c.state ?? "UNKNOWN");
    setHtml("telConnMeta", renderTelemetryMeta(c));
    setText("telTickHz", fmtHz(c.tick_hz));
    setText("telRxHz", fmtHz(c.rx_hz));
    setText("telTxHz", fmtHz(c.tx_hz));
    setHtml("telWheelState", renderWheelState(data.wheel));
    setHtml("telMechState", renderMechState(data.mech));
    setText("telUltrasonic", fmtUltrasonic(data.ultrasonic));
  } catch {
    setText("telConnState", "DISCONNECTED");
    setHtml("telConnMeta", "<div>telemetry fetch failed</div>");
    setText("telTickHz", "N/A");
    setText("telRxHz", "N/A");
    setText("telTxHz", "N/A");
    setHtml("telWheelState", renderWheelState(null));
    setHtml("telMechState", renderMechState(null));
    setText("telUltrasonic", "N/A");
  }
}

/* ============================================================================
   8) Hold-to-Repeat Input Binding
   Shared pointer-safe repeat behavior for teleop and jog buttons.
============================================================================ */

function bindHoldRepeat(btnId, cmdFn, { hz = 15, stopFn = null } = {}) {
  const el = document.getElementById(btnId);
  if (!el) return;

  const periodMs = Math.max(20, Math.floor(1000 / hz));
  let timer = null;
  let activePointerId = null;
  let pressToken = 0;

  const setPressed = (pressed) => {
    el.classList.toggle("pressed", pressed);
  };

  const sendStop = () => {
    if (typeof stopFn === "function") {
      stopFn();
    } else {
      sendManualCmd(0.0, 0.0, {});
    }
  };

  const stop = (ev) => {
    if (activePointerId === null) return;
    if (ev && ev.pointerId !== undefined && ev.pointerId !== activePointerId) return;

    pressToken += 1;
    activePointerId = null;
    setPressed(false);

    if (timer) {
      clearInterval(timer);
      timer = null;
    }

    if (ev && el.hasPointerCapture && ev.pointerId !== undefined) {
      try {
        if (el.hasPointerCapture(ev.pointerId)) {
          el.releasePointerCapture(ev.pointerId);
        }
      } catch {}
    }

    sendStop();
  };

  const start = (ev) => {
    if (ev && ev.cancelable) ev.preventDefault();
    if (activePointerId !== null) return;

    pressToken += 1;
    const token = pressToken;
    activePointerId = ev?.pointerId ?? -1;
    setPressed(true);

    if (ev && el.setPointerCapture && ev.pointerId !== undefined) {
      try {
        el.setPointerCapture(ev.pointerId);
      } catch {}
    }

    if (token === pressToken) {
      cmdFn();
    }

    timer = setInterval(() => {
      if (activePointerId === null) return;
      if (token !== pressToken) return;
      cmdFn();
    }, periodMs);
  };

  el.addEventListener("pointerdown", start);
  el.addEventListener("pointerup", stop);
  el.addEventListener("pointercancel", stop);
  el.addEventListener("lostpointercapture", stop);

  window.addEventListener("blur", () => stop());
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stop();
  });
}

/* ============================================================================
   9) UI Wiring
============================================================================ */

function initControlUI() {
  const btnManual = document.getElementById("btnModeManual");
  const btnAuto = document.getElementById("btnModeAuto");
  const btnStop = document.getElementById("btnStop");

  if (btnManual) btnManual.addEventListener("click", () => setMode("manual"));
  if (btnAuto) btnAuto.addEventListener("click", () => setMode("auto"));
  if (btnStop) btnStop.addEventListener("click", () => sendManualCmd(0.0, 0.0));

  bindHoldRepeat("btnFwd", () => sendManualCmd(+LIN, 0.0), { hz: 15 });
  bindHoldRepeat("btnRev", () => sendManualCmd(-LIN, 0.0), { hz: 15 });
  bindHoldRepeat("btnLeft", () => sendManualCmd(0.0, +ANG), { hz: 15 });
  bindHoldRepeat("btnRight", () => sendManualCmd(0.0, -ANG), { hz: 15 });
}

function initArmManualUI() {
  const btnArmGround = document.getElementById("btnArmGround");
  const btnArmStow = document.getElementById("btnArmStow");
  const btnLhsArmGround = document.getElementById("btnLhsArmGround");
  const btnLhsArmStow = document.getElementById("btnLhsArmStow");

  const rhsJogDuty = Number(cfg.rhs_arm_jog_duty ?? 0.35);
  const rhsStowDeg = Number(cfg.rhs_arm_stow_deg ?? 100.0);
  const lhsJogDuty = Number(cfg.lhs_arm_jog_duty ?? 0.35);
  const lhsStowDeg = Number(cfg.lhs_arm_stow_deg ?? 0.0);

  function sendRhsArmDuty(duty) {
    sendManualCmd(0.0, 0.0, {
      motor_RHS: { mode: "DUTY", value: duty },
    });
  }

  function sendRhsArmPos(positionDeg) {
    sendManualCmd(0.0, 0.0, {
      motor_RHS: { mode: "POS_DEG", value: positionDeg },
    });
  }

  function sendLhsArmDuty(duty) {
    sendManualCmd(0.0, 0.0, {
      motor_LHS: { mode: "DUTY", value: duty },
    });
  }

  function sendLhsArmPos(positionDeg) {
    sendManualCmd(0.0, 0.0, {
      motor_LHS: { mode: "POS_DEG", value: positionDeg },
    });
  }

  bindHoldRepeat("btnArmUp", () => sendRhsArmDuty(+rhsJogDuty), {
    hz: 15,
    stopFn: () => sendRhsArmDuty(0.0),
  });

  bindHoldRepeat("btnArmDown", () => sendRhsArmDuty(-rhsJogDuty), {
    hz: 15,
    stopFn: () => sendRhsArmDuty(0.0),
  });

  bindHoldRepeat("btnLhsArmUp", () => sendLhsArmDuty(+lhsJogDuty), {
    hz: 15,
    stopFn: () => sendLhsArmDuty(0.0),
  });

  bindHoldRepeat("btnLhsArmDown", () => sendLhsArmDuty(-lhsJogDuty), {
    hz: 15,
    stopFn: () => sendLhsArmDuty(0.0),
  });

  if (btnArmGround) {
    btnArmGround.addEventListener("click", () => {
      sendManualCmd(0.0, 0.0, {
        motor_RHS: { mode: "DUTY", value: 0.0 },
        reset_RHS_zero: true,
      });
    });
  }

  if (btnArmStow) {
    btnArmStow.addEventListener("click", () => sendRhsArmPos(rhsStowDeg));
  }

  if (btnLhsArmGround) {
    btnLhsArmGround.addEventListener("click", () => {
      sendManualCmd(0.0, 0.0, {
        motor_LHS: { mode: "DUTY", value: 0.0 },
        reset_LHS_zero: true,
      });
    });
  }

  if (btnLhsArmStow) {
    btnLhsArmStow.addEventListener("click", () => sendLhsArmPos(lhsStowDeg));
  }
}

function initMechQuickUI() {
  const btnLidOpen = document.getElementById("btnLidOpen");
  const btnLidClose = document.getElementById("btnLidClose");
  const btnSweepExtend = document.getElementById("btnSweepExtend");
  const btnSweepStow = document.getElementById("btnSweepStow");

  const lidOpenDeg = Number(cfg.lid_deg_opened ?? 80);
  const lidClosedDeg = Number(cfg.lid_deg_closed ?? 0);
  const sweepExtendDeg = Number(cfg.sweeper_deg_extend ?? 0);
  const sweepStowDeg = Number(cfg.sweeper_deg_closed ?? 30);

  if (btnLidOpen) {
    btnLidOpen.addEventListener("click", async () => {
      await sendManualMech({ lid_deg: lidOpenDeg });
      btnLidOpen.classList.add("active");
      if (btnLidClose) btnLidClose.classList.remove("active");
    });
  }

  if (btnLidClose) {
    btnLidClose.addEventListener("click", async () => {
      await sendManualMech({ lid_deg: lidClosedDeg });
      btnLidClose.classList.add("active");
      if (btnLidOpen) btnLidOpen.classList.remove("active");
    });
  }

  if (btnSweepExtend) {
    btnSweepExtend.addEventListener("click", async () => {
      await sendManualMech({ sweep_deg: sweepExtendDeg });
      btnSweepExtend.classList.add("active");
      if (btnSweepStow) btnSweepStow.classList.remove("active");
    });
  }

  if (btnSweepStow) {
    btnSweepStow.addEventListener("click", async () => {
      await sendManualMech({ sweep_deg: sweepStowDeg });
      btnSweepStow.classList.add("active");
      if (btnSweepExtend) btnSweepExtend.classList.remove("active");
    });
  }
}

/* ============================================================================
   10) Boot
============================================================================ */

document.addEventListener("DOMContentLoaded", () => {
  initControlUI();
  initArmManualUI();
  initMechQuickUI();

  refreshObs();
  refreshController();
  refreshTelemetry();

  setInterval(refreshObs, 100);
  setInterval(refreshController, 100);
  setInterval(refreshTelemetry, 150);
});
