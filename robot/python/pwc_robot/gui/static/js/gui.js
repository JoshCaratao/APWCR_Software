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

function fmtSpeed(v, digits = 2) {
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

  // Units based on your config comments: ft/s and deg/s
  return `Linear Speed = ${linStr} ft/s, Turn Speed = ${angStr} deg/s`;
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
}

async function apiPost(url, body) {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  return await r.json();
}

/* -------------------------
   Perception polling
------------------------- */
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

    // Subtitle and dot
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
    setDot("bad");
    setText("subTitle", "disconnected");
    setText("detectionStatusValue", "DISCONNECTED");
    setText("targetStatusValue", "N/A");
  }
}

/* -------------------------
   Controller polling + actions
------------------------- */
async function refreshController() {
  try {
    const r = await fetch("/controller/status", { cache: "no-store" });
    const data = await r.json();
    if (!data.ok) {
      setText("controlStateValue", "CONNECTING");
      setText("driveCmdValue", "Linear Speed = 0.0 ft/s, Turn Speed = 0.0 deg/s");
      setTeleopEnabled(false);
      return;
    }

    const stateStr = data.status?.state ?? "N/A";
    setText("controlStateValue", stateStr);
    setText("driveCmdValue", fmtCmd(data.cmd));

    setModeButtonActive(stateStr);
  } catch (e) {
    setText("controlStateValue", "DISCONNECTED");
    setText("driveCmdValue", "Linear Speed = N/A ft/s, Turn Speed = N/A deg/s");
    setTeleopEnabled(false);
  }
}

async function setMode(mode) {
  try {
    await apiPost("/controller/mode", { mode });
  } catch (e) {}
  refreshController();
}

async function sendManualCmd(linear, angular) {
  try {
    await apiPost("/controller/manual_cmd", { linear, angular });
  } catch (e) {}
}

/* press-and-hold helpers */
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

    // Send immediately
    cmdFn();

    // Then keep sending
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
    // On release send STOP
    sendManualCmd(0.0, 0.0);
  };

  el.addEventListener("mousedown", start);
  el.addEventListener("mouseup", stop);
  el.addEventListener("mouseleave", stop);

  el.addEventListener("touchstart", start, { passive: false });
  el.addEventListener("touchend", stop, { passive: false });
  el.addEventListener("touchcancel", stop, { passive: false });
}


function initControlUI() {
  const btnManual = document.getElementById("btnModeManual");
  const btnAuto = document.getElementById("btnModeAuto");

  if (btnManual) btnManual.addEventListener("click", () => setMode("manual"));
  if (btnAuto) btnAuto.addEventListener("click", () => setMode("auto"));

  // Default teleop step sizes.
  // If you want these to match your YAML exactly without inline HTML,
  // you can expose a small endpoint later (e.g. /gui/config) and fetch it once.
  const LIN = 0.5;
  const ANG = 10.0;

  bindHoldRepeat("btnFwd",  () => sendManualCmd(+LIN, 0.0), { hz: 15 });
  bindHoldRepeat("btnRev",  () => sendManualCmd(-LIN, 0.0), { hz: 15 });
  bindHoldRepeat("btnLeft", () => sendManualCmd(0.0, +ANG), { hz: 15 });
  bindHoldRepeat("btnRight",() => sendManualCmd(0.0, -ANG), { hz: 15 });

  const btnStop = document.getElementById("btnStop");
  if (btnStop) btnStop.addEventListener("click", () => sendManualCmd(0.0, 0.0));

}

document.addEventListener("DOMContentLoaded", () => {
  initControlUI();
  refreshObs();
  refreshController();
  setInterval(refreshObs, 100);
  setInterval(refreshController, 100);
});
