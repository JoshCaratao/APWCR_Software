/* ============================================================================
   APWCR Dashboard GUI Script (gui.js)
============================================================================ */

/* ============================================================================
   0) Config (rendered by Flask into gui.html as JSON)
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

/**
 * Default teleop speeds.
 * Units: linear ft/s, angular deg/s.
 */
const LIN = Number(cfg.manual_speed_linear ?? 0.5);
const ANG = Number(cfg.manual_speed_angular ?? 5.0);

/* ============================================================================
   1) Small DOM helpers (safe setters)
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

function setTeleopEnabled(enabled) {
  const pad = document.getElementById("teleopPad");
  if (!pad) return;
  pad.classList.toggle("disabled", !enabled);
}

function setModeButtonActive(stateStr) {
  const bM = document.getElementById("btnModeManual");
  const bA = document.getElementById("btnModeAuto");
  if (!bM || !bA) return;

  const isManual = stateStr === "MANUAL";
  bM.classList.toggle("active", isManual);
  bA.classList.toggle("active", !isManual);
  setTeleopEnabled(isManual);
  setMechEnabled(isManual);


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

/* ============================================================================
   2) Formatting helpers
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

function fmtCmd(cmd) {
  const lin = Number(cmd?.linear ?? 0);
  const ang = Number(cmd?.angular ?? 0);

  const linStr = Number.isFinite(lin) ? lin.toFixed(2) : "0.00";
  const angStr = Number.isFinite(ang) ? ang.toFixed(2) : "0.00";

  return `Linear Speed = ${linStr} ft/s, Turn Speed = ${angStr} deg/s`;
}

function fmtMechCmd(mech) {
  const na = "N/A";
  if (!mech) {
    return `Bucket Lift Motor = ${na} | Bucket Rotation Motor = ${na} | LID Servo = ${na} | SWEEPER Servo = ${na}`;
  }

  const rhsVal = mech.motor_RHS?.value;
  const lhsVal = mech.motor_LHS?.value;

  const rhsStr = rhsVal === null || rhsVal === undefined ? na : fmtNum(rhsVal, 1);
  const lhsStr = lhsVal === null || lhsVal === undefined ? na : fmtNum(lhsVal, 1);

  const lid = mech.servo_LID_deg;
  const sweep = mech.servo_SWEEP_deg;

  const lidStr = lid === null || lid === undefined ? na : fmtNum(lid, 1);
  const sweepStr = sweep === null || sweep === undefined ? na : fmtNum(sweep, 1);

  return `Bucket Lift Motor = ${rhsStr} | Bucket Rotation Motor = ${lhsStr} | LID Servo = ${lidStr} | SWEEPER Servo = ${sweepStr}`;
}

function fmtAgeSec(v, digits = 2) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return `${n.toFixed(digits)} s`;
}

function fmtWheelState(wheel) {
  if (!wheel) return "N/A";
  const l = wheel.left_rpm;
  const r = wheel.right_rpm;
  const lStr = l === null || l === undefined ? "N/A" : fmtNum(l, 1);
  const rStr = r === null || r === undefined ? "N/A" : fmtNum(r, 1);
  return `L = ${lStr} rpm | R = ${rStr} rpm`;
}

function fmtMechState(mech) {
  if (!mech) return "N/A";

  const lid = mech.servo_LID_deg;
  const sweep = mech.servo_SWEEP_deg;
  const rhs = mech.motor_RHS_deg;
  const lhs = mech.motor_LHS_deg;

  const lidStr = lid === null || lid === undefined ? "N/A" : fmtNum(lid, 1);
  const sweepStr = sweep === null || sweep === undefined ? "N/A" : fmtNum(sweep, 1);
  const rhsStr = rhs === null || rhs === undefined ? "N/A" : fmtNum(rhs, 1);
  const lhsStr = lhs === null || lhs === undefined ? "N/A" : fmtNum(lhs, 1);

  return `LID = ${lidStr}° | SWEEP = ${sweepStr}° | RHS = ${rhsStr}° | LHS = ${lhsStr}°`;
}

function fmtFt(v, digits = 2) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return `${n.toFixed(digits)} ft`;
}

function fmtUltrasonic(u) {
  if (!u) return "N/A";

  const valid = Boolean(u.valid);
  if (!valid) return "INVALID";

  const d = u.distance_in;
  if (d === null || d === undefined) return "N/A";

  const n = Number(d);
  if (!Number.isFinite(n)) return "N/A";

  return `${n.toFixed(1)} in`;
}

/* ============================================================================
   3) HTTP helpers (API calls)
============================================================================ */

async function apiPost(url, body) {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  return await r.json();
}

/* ============================================================================
   4) Perception Window (poll /perception/status)
============================================================================ */

async function refreshObs() {
  try {
    const r = await fetch("/perception/status", { cache: "no-store" });
    const data = await r.json();

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
    const detectionStatus = detections > 0 ? "DETECTED" : "SEARCHING";
    setText("detectionStatusValue", detectionStatus);
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
   5) Control Window (poll /controller/status + send commands)
============================================================================ */

async function refreshController() {
  try {
    const r = await fetch("/controller/status", { cache: "no-store" });
    const data = await r.json();

    if (!data.ok) {
      setText("controlStateValue", "CONNECTING");
      setText("driveCmdValue", "Linear Speed = 0.00 ft/s, Turn Speed = 0.00 deg/s");
      setText("mechCmdValue", "Bucket Lift Motor = N/A | Bucket Rotation Motor = N/A | LID Servo = N/A | SWEEPER Servo = N/A");
      setTeleopEnabled(false);
      setClass("controlPanel", "controlBlocked", false);
      setHidden("ultraAlert", true);
      return;
    }

    const stateStr = data.status?.state ?? "N/A";
    setText("controlStateValue", stateStr);
    setText("driveCmdValue", fmtCmd(data.cmd));

    const blocked = !!(data?.status?.ultrasonic?.blocked);
    setClass("controlPanel", "controlBlocked", blocked);
    setHidden("ultraAlert", !blocked);

    const mech =
      data?.cmd?.mech ??
      data?.mech_cmd ??
      data?.mech ??
      data?.cmd?.mechanism ??
      null;

    setText("mechCmdValue", fmtMechCmd(mech));
    setModeButtonActive(stateStr);

  } catch {
    setText("controlStateValue", "DISCONNECTED");
    setText("driveCmdValue", "Linear Speed = N/A ft/s, Turn Speed = N/A deg/s");
    setText("mechCmdValue", "Bucket Lift Motor = N/A | Bucket Rotation Motor = N/A | LID Servo = N/A | SWEEPER Servo = N/A");
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

/* ===========================
   NEW: allow optional mech payload
   =========================== */
async function sendManualCmd(linear, angular, mech = null) {
  const body = { linear, angular };
  if (mech && typeof mech === "object") body.mech = mech;

  try {
    await apiPost("/controller/manual_cmd", body);
  } catch {
    // ignore; deadman will stop anyway
  }
}

/* ===========================
   one-shot servo commands
   =========================== */
// Send a mech command repeatedly for a short duration so slow comms/deadman won’t miss it
async function sendManualMech(
  { lid_deg = null, sweep_deg = null } = {},
  { durationMs = 1200, hz = 15 } = {}
) {
  const mech = {};
  if (lid_deg !== null && lid_deg !== undefined) mech.servo_LID_deg = lid_deg;
  if (sweep_deg !== null && sweep_deg !== undefined) mech.servo_SWEEP_deg = sweep_deg;

  if (Object.keys(mech).length === 0) return;

  const periodMs = Math.max(40, Math.floor(1000 / hz));
  const tEnd = Date.now() + durationMs;

  while (Date.now() < tEnd) {
    try {
      await sendManualCmd(0.0, 0.0, mech);  // keep drive stopped, send mech setpoint
    } catch {
      // ignore; next pulse will try again
    }
    await new Promise((res) => setTimeout(res, periodMs));
  }
}


/* ============================================================================
   6) Telemetry Window (poll /telemetry/status)
============================================================================ */

async function refreshTelemetry() {
  try {
    const r = await fetch("/telemetry/status", { cache: "no-store" });
    const data = await r.json();

    if (!data.ok) {
      setText("telConnState", "N/A");
      setText("telConnMeta", data.reason || "no data");
      setText("telTickHz", "N/A");
      setText("telRxHz", "N/A");
      setText("telTxHz", "N/A");
      setText("telWheelState", "N/A");
      setText("telMechState", "N/A");
      setText("telUltrasonic", "N/A");
      return;
    }

    const c = data.connection || {};
    setText("telConnState", c.state ?? "UNKNOWN");

    const metaParts = [];
    if (c.port) metaParts.push(`Port: ${c.port}`);
    if (c.baud) metaParts.push(`Baud: ${c.baud}`);
    if (c.last_rx_age_s !== null && c.last_rx_age_s !== undefined) metaParts.push(`RX age: ${fmtAgeSec(c.last_rx_age_s, 2)}`);
    if (c.rx_stale_s !== null && c.rx_stale_s !== undefined) metaParts.push(`Stale > ${fmtAgeSec(c.rx_stale_s, 2)}`);
    if (c.last_error) metaParts.push(`Err: ${c.last_error}`);
    setText("telConnMeta", metaParts.length ? metaParts.join(" | ") : "N/A");

    setText("telTickHz", fmtHz(c.tick_hz));
    setText("telRxHz", fmtHz(c.rx_hz));
    setText("telTxHz", fmtHz(c.tx_hz));

    setText("telWheelState", fmtWheelState(data.wheel));
    setText("telMechState", fmtMechState(data.mech));
    setText("telUltrasonic", fmtUltrasonic(data.ultrasonic));
  } catch {
    setText("telConnState", "DISCONNECTED");
    setText("telConnMeta", "telemetry fetch failed");
    setText("telTickHz", "N/A");
    setText("telRxHz", "N/A");
    setText("telTxHz", "N/A");
    setText("telWheelState", "N/A");
    setText("telMechState", "N/A");
    setText("telUltrasonic", "N/A");
  }
}

/* ============================================================================
   7) Teleop input helpers (press-and-hold repeat)
============================================================================ */

function bindHoldRepeat(btnId, cmdFn, { hz = 15 } = {}) {
  const el = document.getElementById(btnId);
  if (!el) return;

  const periodMs = Math.max(20, Math.floor(1000 / hz));
  let timer = null;
  let isDown = false;

  const setPressed = (pressed) => {
    el.classList.toggle("pressed", pressed);
  };

  const start = (ev) => {
    ev.preventDefault();
    if (isDown) return;
    isDown = true;

    setPressed(true);
    cmdFn();

    timer = setInterval(() => {
      if (!isDown) return;
      cmdFn();
    }, periodMs);
  };

  const stop = (ev) => {
    if (ev) ev.preventDefault();
    isDown = false;

    setPressed(false);

    if (timer) {
      clearInterval(timer);
      timer = null;
    }

    sendManualCmd(0.0, 0.0, {});
  };

  el.addEventListener("mousedown", start);
  el.addEventListener("mouseup", stop);
  el.addEventListener("mouseleave", stop);

  el.addEventListener("touchstart", start, { passive: false });
  el.addEventListener("touchend", stop, { passive: false });
  el.addEventListener("touchcancel", stop, { passive: false });

  window.addEventListener("mouseup", stop);
  window.addEventListener("touchend", stop, { passive: false });
}

/* ============================================================================
   8) UI initialization (wire up buttons)
============================================================================ */

function initControlUI() {
  const btnManual = document.getElementById("btnModeManual");
  const btnAuto = document.getElementById("btnModeAuto");

  if (btnManual) btnManual.addEventListener("click", () => setMode("manual"));
  if (btnAuto) btnAuto.addEventListener("click", () => setMode("auto"));

  bindHoldRepeat("btnFwd", () => sendManualCmd(+LIN, 0.0), { hz: 15 });
  bindHoldRepeat("btnRev", () => sendManualCmd(-LIN, 0.0), { hz: 15 });
  bindHoldRepeat("btnLeft", () => sendManualCmd(0.0, -ANG), { hz: 15 });
  bindHoldRepeat("btnRight", () => sendManualCmd(0.0, +ANG), { hz: 15 });

  const btnStop = document.getElementById("btnStop");
  if (btnStop) btnStop.addEventListener("click", () => sendManualCmd(0.0, 0.0));
}

/* ===========================
   NEW: mechanism quick buttons
   These just send setpoints once on click
   =========================== */
function initMechQuickUI() {
  const btnLidOpen = document.getElementById("btnLidOpen");
  const btnLidClose = document.getElementById("btnLidClose");
  const btnSweepExtend = document.getElementById("btnSweepExtend");
  const btnSweepStow = document.getElementById("btnSweepStow");

  const LID_OPEN_DEG = Number(cfg.lid_deg_opened ?? 80);
  const LID_CLOSED_DEG = Number(cfg.lid_deg_closed ?? 0);
  const SWEEP_EXTEND_DEG = Number(cfg.sweeper_deg_extend ?? 0);
  const SWEEP_STOW_DEG = Number(cfg.sweeper_deg_closed ?? 30);


  if (btnLidOpen) {
    btnLidOpen.addEventListener("click", async () => {
      await sendManualMech({ lid_deg: LID_OPEN_DEG });
      btnLidOpen.classList.add("active");
      if (btnLidClose) btnLidClose.classList.remove("active");
    });
  }

  if (btnLidClose) {
    btnLidClose.addEventListener("click", async () => {
      await sendManualMech({ lid_deg: LID_CLOSED_DEG });
      btnLidClose.classList.add("active");
      if (btnLidOpen) btnLidOpen.classList.remove("active");
    });
  }

  if (btnSweepExtend) {
    btnSweepExtend.addEventListener("click", async () => {
      await sendManualMech({ sweep_deg: SWEEP_EXTEND_DEG });
      btnSweepExtend.classList.add("active");
      if (btnSweepStow) btnSweepStow.classList.remove("active");
    });
  }

  if (btnSweepStow) {
    btnSweepStow.addEventListener("click", async () => {
      await sendManualMech({ sweep_deg: SWEEP_STOW_DEG });
      btnSweepStow.classList.add("active");
      if (btnSweepExtend) btnSweepExtend.classList.remove("active");
    });
  }
}

function setMechEnabled(enabled) {
  const el = document.getElementById("mech-quick");
  if (!el) return;
  el.classList.toggle("disabled", !enabled);
}


/* ============================================================================
   9) Boot (start polling loops after DOM is ready)
============================================================================ */

document.addEventListener("DOMContentLoaded", () => {
  initControlUI();
  initMechQuickUI(); // NEW

  refreshObs();
  refreshController();
  refreshTelemetry();

  setInterval(refreshObs, 100);
  setInterval(refreshController, 100);
  setInterval(refreshTelemetry, 150);
});
