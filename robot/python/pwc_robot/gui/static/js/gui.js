let lastTs = null;
let hzSmoothed = null;

function setDot(mode) {
  const dot = document.getElementById("statusDot");
  dot.classList.remove("ok");
  dot.classList.remove("bad");
  if (mode === "ok") dot.classList.add("ok");
  if (mode === "bad") dot.classList.add("bad");
}

function fmtNum(x, digits = 0) {
  if (x === null || x === undefined) return "-";
  return Number(x).toFixed(digits);
}

function setStatusText(statusText) {
  const el = document.getElementById("statusValue");
  el.textContent = statusText;
}

function computeStatus(data) {
  // Priority:
  // 1) stable_detected => Stable detection
  // 2) any detection evidence (best exists OR streak > 0) => Detected
  // 3) otherwise => Searching
  if (data.stable_detected) return "STABLE DETECTION";

  const hasBest = data.best !== null && data.best !== undefined;
  const hasStreak = (data.streak !== null && data.streak !== undefined && Number(data.streak) > 0);
  return (hasBest || hasStreak) ? "DETECTED" : "SEARCHING ...";
}

async function refreshObs() {
  try {
    const r = await fetch("/perception/status", { cache: "no-store" });
    const data = await r.json();

    if (!data.ok) {
      setDot("bad");
      document.getElementById("subTitle").textContent = data.reason || "no data";
      setStatusText("Connecting");
      return;
    }

    const statusText = computeStatus(data);
    setStatusText(statusText);

    // Dot behavior: green on stable detection, neutral otherwise
    setDot(data.stable_detected ? "ok" : "");
    document.getElementById("subTitle").textContent = data.stable_detected ? "STABLE" : "RUNNING";

    // Detection streak
    document.getElementById("streakValue").textContent = fmtNum(data.streak, 0);

    // Center and confidence:
    // Prefer stable_center when present, otherwise fall back to best if present.
    const c = (data.stable_center && data.stable_center.length >= 3) ? data.stable_center : data.best;

    if (c && c.length >= 3) {
      document.getElementById("centerValue").textContent = `${fmtNum(c[0], 0)}, ${fmtNum(c[1], 0)}`;
      document.getElementById("confValue").textContent = fmtNum(c[2], 2);
    } else {
      document.getElementById("centerValue").textContent = "-";
      document.getElementById("confValue").textContent = "-";
    }

    // Optional: compute perception update rate (not displayed now, but kept for debug)
    if (lastTs !== null && data.timestamp !== null && data.timestamp !== undefined) {
      const dt = data.timestamp - lastTs;
      if (dt > 0) {
        const hz = 1.0 / dt;
        hzSmoothed = (hzSmoothed === null) ? hz : (0.85 * hzSmoothed + 0.15 * hz);
      }
    }
    lastTs = data.timestamp;

  } catch (e) {
    setDot("bad");
    document.getElementById("subTitle").textContent = "disconnected";
    setStatusText("Disconnected");
  }
}

// Update the status panel at a steady interval.
// 100 ms keeps it responsive without being too spammy.
setInterval(refreshObs, 100);
refreshObs();
