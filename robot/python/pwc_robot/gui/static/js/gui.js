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

let latestControllerState = "N/A";

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

function fmtStallFault(fault, dir) {
  if (!Boolean(fault)) return "OK";
  const d = Number(dir);
  if (d > 0) return "FAULT +";
  if (d < 0) return "FAULT -";
  return "FAULT";
}

function renderDriveCmd(cmd) {
  return renderMetricGrid([
    renderMetricTile("Linear Speed", `${fmtNum(cmd?.linear, 2)} ft/s`, { mono: true }),
    renderMetricTile("Turn Speed", `${fmtNum(cmd?.angular, 2)} deg/s`, { mono: true }),
    renderMetricTile("Left Wheel Target RPM", `${fmtNum(cmd?.left_target_rpm, 1)} rpm`, { mono: true }),
    renderMetricTile("Right Wheel Target RPM", `${fmtNum(cmd?.right_target_rpm, 1)} rpm`, { mono: true }),
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
        ${renderMetricTile("Left Wheel Measured RPM", `${fmtNum(wheel?.left_rpm, 2)} rpm`, { mono: true })}
        ${renderMetricTile("Left Motor Duty", fmtNum(wheel?.left_duty, 2), { mono: true })}
        ${renderMetricTile("Left Stall", fmtStallFault(wheel?.left_stall_fault, wheel?.left_stall_dir), { mono: true })}
      </div>
      <div class="metric-stack">
        ${renderMetricTile("Right Wheel Measured RPM", `${fmtNum(wheel?.right_rpm, 2)} rpm`, { mono: true })}
        ${renderMetricTile("Right Motor Duty", fmtNum(wheel?.right_duty, 2), { mono: true })}
        ${renderMetricTile("Right Stall", fmtStallFault(wheel?.right_stall_fault, wheel?.right_stall_dir), { mono: true })}
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
    renderMetricTile("Bucket Lift Target RPM", `${fmtNum(mech?.motor_RHS_target_rpm, 2)} rpm`, { mono: true }),
    renderMetricTile("Bucket Lift RPM", `${fmtNum(mech?.motor_RHS_rpm, 2)} rpm`, { mono: true }),
    renderMetricTile("Bucket Lift Duty", fmtNum(mech?.motor_RHS_duty, 2), { mono: true }),
    renderMetricTile("Bucket Lift Stall", fmtStallFault(mech?.motor_RHS_stall_fault, mech?.motor_RHS_stall_dir), { mono: true }),
  ];

  const bucketRotItems = [
    renderMetricTile("Bucket Rot Pos", `${fmtNum(mech?.motor_LHS_deg, 1)} deg`, { mono: true }),
    renderMetricTile("Bucket Rot Target RPM", `${fmtNum(mech?.motor_LHS_target_rpm, 2)} rpm`, { mono: true }),
    renderMetricTile("Bucket Rot RPM", `${fmtNum(mech?.motor_LHS_rpm, 2)} rpm`, { mono: true }),
    renderMetricTile("Bucket Rot Duty", fmtNum(mech?.motor_LHS_duty, 2), { mono: true }),
    renderMetricTile("Bucket Rot Stall", fmtStallFault(mech?.motor_LHS_stall_fault, mech?.motor_LHS_stall_dir), { mono: true }),
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

function getControlTestSpec() {
  const type = document.getElementById("controlTestType")?.value ?? "forward";
  const rawValue = Number(document.getElementById("controlTestValue")?.value ?? 0.0);
  const durationS = Number(document.getElementById("controlTestDuration")?.value ?? 6.0);
  const magnitude = Math.abs(rawValue);

  if (type === "forward") {
    return {
      type,
      linear: magnitude,
      angular: 0.0,
      commandValue: magnitude,
      commandUnits: "ft/s",
      label: `FORWARD @ ${magnitude.toFixed(2)} ft/s`,
    };
  }

  if (type === "reverse") {
    return {
      type,
      linear: -magnitude,
      angular: 0.0,
      commandValue: magnitude,
      commandUnits: "ft/s",
      label: `REVERSE @ ${magnitude.toFixed(2)} ft/s`,
    };
  }

  if (type === "turn_left") {
    return {
      type,
      domain: "drive",
      linear: 0.0,
      angular: magnitude,
      commandValue: magnitude,
      commandUnits: "deg/s",
      label: `TURN LEFT @ ${magnitude.toFixed(2)} deg/s`,
    };
  }

  if (type === "turn_right") {
    return {
      type: "turn_right",
      domain: "drive",
      linear: 0.0,
      angular: -magnitude,
      commandValue: magnitude,
      commandUnits: "deg/s",
      label: `TURN RIGHT @ ${magnitude.toFixed(2)} deg/s`,
    };
  }

  if (type === "rhs_mech_speed") {
    return {
      type,
      domain: "mechanism",
      commandValue: rawValue,
      commandUnits: "rpm",
      label: `BUCKET LIFT SPEED @ ${rawValue.toFixed(2)} rpm`,
    };
  }

  if (type === "lhs_mech_speed") {
    return {
      type,
      domain: "mechanism",
      commandValue: rawValue,
      commandUnits: "rpm",
      label: `BUCKET ROT SPEED @ ${rawValue.toFixed(2)} rpm`,
    };
  }

  if (type === "rhs_mech_speed_sine") {
    return {
      type,
      domain: "mechanism",
      commandValue: rawValue,
      commandUnits: "rpm",
      label: `BUCKET LIFT SPEED SINE +/- ${Math.abs(rawValue).toFixed(2)} rpm`,
    };
  }

  if (type === "lhs_mech_speed_sine") {
    return {
      type,
      domain: "mechanism",
      commandValue: rawValue,
      commandUnits: "rpm",
      label: `BUCKET ROT SPEED SINE +/- ${Math.abs(rawValue).toFixed(2)} rpm`,
    };
  }

  if (type === "rhs_mech_pos") {
    return {
      type,
      domain: "mechanism",
      commandValue: rawValue,
      commandUnits: "deg",
      label: `BUCKET LIFT POS @ ${rawValue.toFixed(1)} deg`,
    };
  }

  return {
    type: "lhs_mech_pos",
    domain: "mechanism",
    commandValue: rawValue,
    commandUnits: "deg",
    label: `BUCKET ROT POS @ ${rawValue.toFixed(1)} deg`,
  };
}

function updateControlTestUnits() {
  const type = document.getElementById("controlTestType")?.value ?? "forward";
  const unitsEl = document.getElementById("controlTestUnits");
  if (!unitsEl) return;

  if (type === "forward" || type === "reverse") {
    unitsEl.textContent = "Linear drive test in ft/s";
  } else if (type === "turn_left" || type === "turn_right") {
    unitsEl.textContent = "Pure turn test in deg/s";
  } else if (type === "rhs_mech_speed" || type === "lhs_mech_speed") {
    unitsEl.textContent = "Mechanism speed step test in rpm (signed)";
  } else if (type === "rhs_mech_speed_sine" || type === "lhs_mech_speed_sine") {
    unitsEl.textContent = "Mechanism speed sine test amplitude in rpm; duration is one full period";
  } else {
    unitsEl.textContent = "Mechanism position step test in deg";
  }
}

function renderControlTestLive(sample) {
  if (!sample) return "No test data yet.";

  if (sample.command_mech_mode) {
    const side = sample.command_mech_side === "RHS" ? "Bucket Lift" : "Bucket Rot";
    const base = [
      `t=${sample.elapsed_s.toFixed(2)} s`,
      `${side} ${String(sample.command_mech_mode)}`,
      `cmd=${sample.command_mech_value.toFixed(2)} ${sample.command_units}`,
      `target=${sample.test_target_rpm.toFixed(1)} rpm`,
      `measured=${sample.test_measured_rpm.toFixed(1)} rpm`,
      `duty=${sample.test_motor_duty.toFixed(2)}`,
    ];
    if (sample.command_mech_mode === "POS_DEG") {
      base.push(`pos=${sample.test_measured_deg.toFixed(1)} deg`);
    }
    return base.join(" | ");
  }

  return [
    `t=${sample.elapsed_s.toFixed(2)} s`,
    `cmd=(${sample.command_linear_ftps.toFixed(2)} ft/s, ${sample.command_angular_dps.toFixed(2)} deg/s)`,
    `targets=(${sample.drive_left_target_rpm.toFixed(1)}, ${sample.drive_right_target_rpm.toFixed(1)}) rpm`,
    `measured=(${sample.drive_left_measured_rpm.toFixed(1)}, ${sample.drive_right_measured_rpm.toFixed(1)}) rpm`,
    `duty=(${sample.drive_left_motor_duty.toFixed(2)}, ${sample.drive_right_motor_duty.toFixed(2)})`,
  ].join(" | ");
}

function updateControlTestStatus(text) {
  setText("controlTestStatus", text);
}

async function stopControlTest({ completed = false } = {}) {
  try {
    await apiPost("/control_test/stop", {});
  } catch {}
  updateControlTestStatus(completed ? "COMPLETE" : "STOPPED");
}

async function startControlTest() {
  if (latestControllerState !== "MANUAL") {
    updateControlTestStatus("SWITCH TO MANUAL FIRST");
    return;
  }

  const spec = getControlTestSpec();
  const durationS = Math.max(1.0, Number(document.getElementById("controlTestDuration")?.value ?? 6.0));

  try {
    const result = await apiPost("/control_test/start", {
      test_type: spec.type,
      command_value: spec.commandValue,
      duration_s: durationS,
    });

    if (!result?.ok) {
      updateControlTestStatus(`START FAILED: ${result?.reason ?? "unknown"}`);
      return;
    }

    setText("controlTestSamples", "0");
    setText("controlTestLive", "Waiting for first sample...");
    updateControlTestStatus(`RUNNING ${spec.label}`);
  } catch {
    updateControlTestStatus("START FAILED");
  }
}

function downloadControlTestCsv() {
  window.location.href = "/control_test/download_latest";
}

async function refreshControlTest() {
  try {
    const response = await fetch("/control_test/status", { cache: "no-store" });
    const data = await response.json();

    if (!data?.ok) {
      updateControlTestStatus("IDLE");
      return;
    }

    setText("controlTestSamples", String(data.samples ?? 0));
    if (data.latest_sample) {
      setText("controlTestLive", renderControlTestLive(data.latest_sample));
    }

    if (data.active) {
      const spec = data.spec || {};
      let units = "deg";
      if (spec.command_units === "ftps") units = "ft/s";
      else if (spec.command_units === "degps") units = "deg/s";
      else if (spec.command_units === "rpm") units = "rpm";
      const commandValue = Number(spec.command_value ?? 0).toFixed(2);
      updateControlTestStatus(`RUNNING ${String(spec.test_type ?? "test").toUpperCase()} @ ${commandValue} ${units}`);
      return;
    }

    updateControlTestStatus(data.status ?? "IDLE");
  } catch {
    updateControlTestStatus("DISCONNECTED");
  }
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
    latestControllerState = stateStr;
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
    latestControllerState = "N/A";
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

// Repeat a one-shot mechanism command briefly so slower comms do not miss it.
async function sendManualMechBurst(mech, { durationMs = 1200, hz = 15 } = {}) {
  if (!mech || typeof mech !== "object" || Object.keys(mech).length === 0) return;
  const periodMs = Math.max(40, Math.floor(1000 / hz));
  const endTimeMs = Date.now() + durationMs;

  while (Date.now() < endTimeMs) {
    try {
      await sendManualCmd(0.0, 0.0, mech);
    } catch {}
    await new Promise((resolve) => setTimeout(resolve, periodMs));
  }
}

async function sendManualMech(
  { lid_deg = null, sweep_deg = null } = {},
  { durationMs = 1200, hz = 15 } = {}
) {
  const mech = {};
  if (lid_deg !== null && lid_deg !== undefined) mech.servo_LID_deg = lid_deg;
  if (sweep_deg !== null && sweep_deg !== undefined) mech.servo_SWEEP_deg = sweep_deg;
  await sendManualMechBurst(mech, { durationMs, hz });
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

  const controlTestType = document.getElementById("controlTestType");
  const btnControlTestStart = document.getElementById("btnControlTestStart");
  const btnControlTestStop = document.getElementById("btnControlTestStop");
  const btnControlTestDownload = document.getElementById("btnControlTestDownload");

  if (controlTestType) {
    controlTestType.addEventListener("change", updateControlTestUnits);
  }

  if (btnControlTestStart) {
    btnControlTestStart.addEventListener("click", startControlTest);
  }

  if (btnControlTestStop) {
    btnControlTestStop.addEventListener("click", () => stopControlTest({ completed: false }));
  }

  if (btnControlTestDownload) {
    btnControlTestDownload.addEventListener("click", downloadControlTestCsv);
  }

  updateControlTestUnits();
}

function initArmManualUI() {
  const btnArmGround = document.getElementById("btnArmGround");
  const btnArmStow = document.getElementById("btnArmStow");
  const btnLhsArmGround = document.getElementById("btnLhsArmGround");
  const btnLhsArmStow = document.getElementById("btnLhsArmStow");

  const rhsJogRpm = Number(cfg.rhs_arm_jog_rpm ?? 4.0);
  const rhsStowDeg = Number(cfg.rhs_arm_stow_deg ?? 100.0);
  const lhsJogRpm = Number(cfg.lhs_arm_jog_rpm ?? 4.0);
  const lhsStowDeg = Number(cfg.lhs_arm_stow_deg ?? 0.0);

  function sendRhsArmRpm(rpm) {
    sendManualCmd(0.0, 0.0, {
      motor_RHS: { mode: "RPM", value: rpm },
    });
  }

  function sendRhsArmPos(positionDeg) {
    sendManualCmd(0.0, 0.0, {
      motor_RHS: { mode: "POS_DEG", value: positionDeg },
    });
  }

  function sendLhsArmRpm(rpm) {
    sendManualCmd(0.0, 0.0, {
      motor_LHS: { mode: "RPM", value: rpm },
    });
  }

  function sendLhsArmPos(positionDeg) {
    sendManualCmd(0.0, 0.0, {
      motor_LHS: { mode: "POS_DEG", value: positionDeg },
    });
  }

  bindHoldRepeat("btnArmUp", () => sendRhsArmRpm(+rhsJogRpm), {
    hz: 15,
    stopFn: () => sendRhsArmRpm(0.0),
  });

  bindHoldRepeat("btnArmDown", () => sendRhsArmRpm(-rhsJogRpm), {
    hz: 15,
    stopFn: () => sendRhsArmRpm(0.0),
  });

  bindHoldRepeat("btnLhsArmUp", () => sendLhsArmRpm(+lhsJogRpm), {
    hz: 15,
    stopFn: () => sendLhsArmRpm(0.0),
  });

  bindHoldRepeat("btnLhsArmDown", () => sendLhsArmRpm(-lhsJogRpm), {
    hz: 15,
    stopFn: () => sendLhsArmRpm(0.0),
  });

  if (btnArmGround) {
    btnArmGround.addEventListener("click", () => {
      sendManualMechBurst({
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
      sendManualMechBurst({
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
  refreshControlTest();

  setInterval(refreshObs, 100);
  setInterval(refreshController, 100);
  setInterval(refreshTelemetry, 150);
  setInterval(refreshControlTest, 200);

  window.addEventListener("blur", () => stopControlTest({ completed: false }));
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopControlTest({ completed: false });
  });
});
