/* ============================================================================
   APWCR Dashboard GUI Script (gui.js)

   Responsibilities:
   - Poll perception status (/perception/status) and update Perception Window UI
   - Poll controller status (/controller/status) and update Control Window UI
   - Send controller actions:
       - Switch MANUAL/AUTO mode (/controller/mode)
       - Send manual teleop commands (/controller/manual_cmd)
   - Implement press-and-hold teleop so commands refresh fast enough for deadman
============================================================================ */

/* ============================================================================
   0) Config (rendered by Flask into gui.html as JSON)
============================================================================ */

/**
 * Reads a JSON blob embedded in gui.html to keep JS in a separate file
 * while still using YAML-configured values (manual speeds, etc.).
 */
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
 * Pulled from config if available (YAML -> Flask -> HTML JSON -> JS).
 * Units: linear ft/s, angular deg/s.
 */
const LIN = Number(cfg.manual_speed_linear ?? 0.5);
const ANG = Number(cfg.manual_speed_angular ?? 5.0);

/* ============================================================================
   1) Small DOM helpers (safe setters)
============================================================================ */

/**
 * Sets the status dot state: "ok", "bad", or "" (neutral).
 */
function setDot(mode) {
  const dot = document.getElementById("statusDot");
  if (!dot) return;

  dot.classList.remove("ok");
  dot.classList.remove("bad");
  if (mode === "ok") dot.classList.add("ok");
  if (mode === "bad") dot.classList.add("bad");
}

/**
 * Convenience for setting textContent on an element by id.
 */
function setText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

/**
 * Enables/disables the teleop pad by toggling the "disabled" class.
 * Disabled teleop is both visually dimmed and pointer-events blocked.
 */
function setTeleopEnabled(enabled) {
  const pad = document.getElementById("teleopPad");
  if (!pad) return;
  pad.classList.toggle("disabled", !enabled);
}

/**
 * Updates which mode button is visually active, and enables teleop only in MANUAL.
 */
function setModeButtonActive(stateStr) {
  const bM = document.getElementById("btnModeManual");
  const bA = document.getElementById("btnModeAuto");
  if (!bM || !bA) return;

  const isManual = stateStr === "MANUAL";
  bM.classList.toggle("active", isManual);
  bA.classList.toggle("active", !isManual);
  setTeleopEnabled(isManual);
}

/* ============================================================================
   2) Formatting helpers
============================================================================ */

/**
 * Formats a numeric Hz value for display.
 */
function fmtHz(v) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return `${n.toFixed(1)} Hz`;
}

/**
 * Formats a number with fixed digits, or "N/A" for invalid inputs.
 */
function fmtNum(v, digits = 0) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return n.toFixed(digits);
}

/**
 * Formats a speed (generic numeric) with fixed digits, or "N/A".
 * (Currently not used directly, but kept for readability/expansion.)
 */
function fmtSpeed(v, digits = 2) {
  if (v === null || v === undefined) return "N/A";
  const n = Number(v);
  if (!Number.isFinite(n)) return "N/A";
  return n.toFixed(digits);
}

/**
 * Formats a DriveCommand object into a friendly string with units.
 * Expects cmd like: { linear: ft/s, angular: deg/s }.
 */
function fmtCmd(cmd) {
  const lin = Number(cmd?.linear ?? 0);
  const ang = Number(cmd?.angular ?? 0);

  const linStr = Number.isFinite(lin) ? lin.toFixed(2) : "0.00";
  const angStr = Number.isFinite(ang) ? ang.toFixed(2) : "0.00";

  return `Linear Speed = ${linStr} ft/s, Turn Speed = ${angStr} deg/s`;
}

/**
 * Formats a MechanismCommand for display, showing ONLY command VALUES
 * (no POS_DEG / DUTY / RPM strings).
 *
 * Expected shape:
 * {
 *   motor_RHS: { mode: "...", value: number } | null,
 *   motor_LHS: { mode: "...", value: number } | null,
 *   servo_LID_deg: number | null,
 *   servo_SWEEP_deg: number | null
 * }
 */
function fmtMechCmd(mech) {
  const na = "N/A";
  if (!mech) {
    return `Bucket Lift Motor = ${na} | Bucket Rotation Motor = ${na} | LID Servo = ${na} | SWEEPER Servo = ${na}`;
  }

  // Motors: show only the numeric value
  const rhsVal = mech.motor_RHS?.value;
  const lhsVal = mech.motor_LHS?.value;

  const rhsStr = (rhsVal === null || rhsVal === undefined) ? na : fmtNum(rhsVal, 1);
  const lhsStr = (lhsVal === null || lhsVal === undefined) ? na : fmtNum(lhsVal, 1);

  // Servos: show numeric value (deg) without extra mode text
  const lid = mech.servo_LID_deg;
  const sweep = mech.servo_SWEEP_deg;

  const lidStr = (lid === null || lid === undefined) ? na : fmtNum(lid, 1);
  const sweepStr = (sweep === null || sweep === undefined) ? na : fmtNum(sweep, 1);

  return `Bucket Lift Motor = ${rhsStr} | Bucket Rotation Motor = ${lhsStr} | LID Servo = ${lidStr} | SWEEPER Servo = ${sweepStr}`;
}

/* ============================================================================
   3) HTTP helpers (API calls)
============================================================================ */

/**
 * Sends a JSON POST to the server and returns parsed JSON response.
 */
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

/**
 * Polls perception status and updates the Perception UI fields:
 * - Detection status, count, inference speeds
 * - Target policy/mode, target status, confidence/area/center
 */
async function refreshObs() {
  try {
    const r = await fetch("/perception/status", { cache: "no-store" });
    const data = await r.json();

    if (!data.ok) {
      setDot("bad");
      setText("subTitle", data.reason || "no data");
      setText("detectionStatusValue", "CONNECTING");
      setText("targetStatusValue", "N/A");
      return;
    }

    // Header subtitle + dot reflect "stable" detection
    const stable = data.target_status === "Stable Detection";
    setDot(stable ? "ok" : "");
    setText("subTitle", stable ? "STABLE" : "RUNNING");

    // General detection section
    const detections = Number(data.num_detections ?? 0);
    const detectionStatus = detections > 0 ? "DETECTED" : "SEARCHING";
    setText("detectionStatusValue", detectionStatus);
    setText("detectionsValue", String(detections));

    setText("targetInferHzValue", fmtHz(data.target_infer_hz));
    setText("measuredInferHzValue", fmtHz(data.measured_infer_hz));

    // Target section
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
  } catch (e) {
    // Network/server error
    setDot("bad");
    setText("subTitle", "disconnected");
    setText("detectionStatusValue", "DISCONNECTED");
    setText("targetStatusValue", "N/A");
  }
}

/* ============================================================================
   5) Control Window (poll /controller/status + send commands)
============================================================================ */

/**
 * Polls controller status and updates Control UI fields:
 * - Current state
 * - Current drive command (units)
 * - Current mechanism command (values only)
 * Also toggles teleop enabled/disabled based on MANUAL vs AUTO.
 */
async function refreshController() {
  try {
    const r = await fetch("/controller/status", { cache: "no-store" });
    const data = await r.json();

    if (!data.ok) {
      setText("controlStateValue", "CONNECTING");
      setText("driveCmdValue", "Linear Speed = 0.00 ft/s, Turn Speed = 0.00 deg/s");
      setText("mechCmdValue", "Bucket Lift Motor = N/A | Bucket Rotation Motor = N/A | LID Servo = N/A | SWEEPER Servo = N/A");
      setTeleopEnabled(false);
      return;
    }

    const stateStr = data.status?.state ?? "N/A";
    setText("controlStateValue", stateStr);

    // Drive command
    setText("driveCmdValue", fmtCmd(data.cmd));

    // Mechanism command (support multiple payload shapes)
    const mech =
      data?.cmd?.mech ??
      data?.mech_cmd ??
      data?.mech ??
      data?.cmd?.mechanism ??
      null;

    setText("mechCmdValue", fmtMechCmd(mech));

    setModeButtonActive(stateStr);
  } catch (e) {
    setText("controlStateValue", "DISCONNECTED");
    setText("driveCmdValue", "Linear Speed = N/A ft/s, Turn Speed = N/A deg/s");
    setText("mechCmdValue", "Bucket Lift Motor = N/A | Bucket Rotation Motor = N/A | LID Servo = N/A | SWEEPER Servo = N/A");
    setTeleopEnabled(false);
  }
}

/**
 * Requests controller mode change (manual/auto).
 */
async function setMode(mode) {
  try {
    await apiPost("/controller/mode", { mode });
  } catch (e) {
    // ignore and let polling reflect reality
  }
  refreshController();
}

/**
 * Sends a manual teleop command to the controller.
 * The controller should apply this only if currently in MANUAL.
 */
async function sendManualCmd(linear, angular) {
  try {
    await apiPost("/controller/manual_cmd", { linear, angular });
  } catch (e) {
    // ignore; deadman will stop anyway
  }
}

/* ============================================================================
   6) Teleop input helpers (press-and-hold repeat)
============================================================================ */

/**
 * Binds press-and-hold behavior to a button:
 * - On press: send the command immediately, then keep sending at `hz`
 * - On release: stop repeating and send STOP (0,0)
 */
function bindHoldRepeat(btnId, cmdFn, { hz = 15 } = {}) {
  const el = document.getElementById(btnId);
  if (!el) return;

  const periodMs = Math.max(20, Math.floor(1000 / hz));
  let timer = null;
  let isDown = false;

  const start = (ev) => {
    ev.preventDefault();
    if (isDown) return;
    isDown = true;

    cmdFn();

    timer = setInterval(() => {
      if (!isDown) return;
      cmdFn();
    }, periodMs);
  };

  const stop = (ev) => {
    if (ev) ev.preventDefault();
    isDown = false;

    if (timer) {
      clearInterval(timer);
      timer = null;
    }

    sendManualCmd(0.0, 0.0);
  };

  el.addEventListener("mousedown", start);
  el.addEventListener("mouseup", stop);
  el.addEventListener("mouseleave", stop);

  el.addEventListener("touchstart", start, { passive: false });
  el.addEventListener("touchend", stop, { passive: false });
  el.addEventListener("touchcancel", stop, { passive: false });
}

/* ============================================================================
   7) UI initialization (wire up buttons)
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

/* ============================================================================
   8) Boot (start polling loops after DOM is ready)
============================================================================ */

document.addEventListener("DOMContentLoaded", () => {
  initControlUI();

  refreshObs();
  refreshController();

  setInterval(refreshObs, 100);
  setInterval(refreshController, 100);
});
