(() => {
  const el = (id) => document.getElementById(id);

  const ui = {
    state:        el("state-value"),
    auto:         el("auto-value"),
    line:         el("line-value"),
    lineConf:     el("line-conf"),
    usL:          el("us-left"),
    usC:          el("us-center"),
    usR:          el("us-right"),
    objName:      el("obj-name"),
    objConf:      el("obj-conf"),
    objDist:      el("obj-dist"),
    fps:          el("fps-value"),
    conn:         el("connection-pill"),
    servoRange:   el("servo-range"),
    servoValue:   el("servo-value"),
  };

  // ---------- send control command ----------
  async function sendCommand(cmd, extra = {}) {
    try {
      const res = await fetch("/api/control", {
        method:  "POST",
        headers: { "Content-Type": "application/json" },
        body:    JSON.stringify({ cmd, ...extra }),
      });
      return await res.json();
    } catch (e) {
      console.warn("control error", e);
      return null;
    }
  }

  // ---------- wire up buttons ----------
  document.querySelectorAll(".dpad .btn, .auto-controls .btn").forEach((b) => {
    const cmd = b.dataset.cmd;
    if (!cmd) return;
    b.addEventListener("click", () => sendCommand(cmd));
  });

  // ---------- wire up servo ----------
  let servoTimer = null;
  ui.servoRange.addEventListener("input", () => {
    ui.servoValue.textContent = ui.servoRange.value;
    clearTimeout(servoTimer);
    servoTimer = setTimeout(() => {
      sendCommand("servo", { angle: parseInt(ui.servoRange.value, 10) });
    }, 80);
  });

  // ---------- format helper ----------
  function fmtCm(v) {
    if (v === null || v === undefined) return "–";
    v = parseFloat(v);
    if (!isFinite(v) || v < 0) return "–";
    return v.toFixed(0);
  }

  // ---------- poll status ----------
  async function pollStatus() {
    try {
      const r = await fetch("/api/status", { cache: "no-store" });
      const s = await r.json();

      ui.state.textContent = s.state;
      ui.auto.textContent  = s.autonomous ? "ON" : "OFF";

      ui.line.textContent     = s.line_position;
      ui.lineConf.textContent = (s.line_confidence || 0).toFixed(2);

      ui.usL.textContent = fmtCm(s.ultrasonic_left);
      ui.usC.textContent = fmtCm(s.ultrasonic_center);
      ui.usR.textContent = fmtCm(s.ultrasonic_right);

      ui.objName.textContent = s.object_name || "none";
      ui.objConf.textContent = (s.object_confidence || 0).toFixed(2);
      ui.objDist.textContent = fmtCm(s.object_distance_cm);

      ui.fps.textContent = s.fps;

      if (s.esp32_online) {
        ui.conn.textContent = "ESP32-CAM: online";
        ui.conn.classList.remove("pill-off");
        ui.conn.classList.add("pill-on");
      } else {
        ui.conn.textContent = "ESP32-CAM: offline";
        ui.conn.classList.add("pill-off");
        ui.conn.classList.remove("pill-on");
      }
    } catch (e) {
      ui.conn.textContent = "Server: disconnected";
      ui.conn.classList.add("pill-off");
    }
  }

  setInterval(pollStatus, 400);
  pollStatus();

  // ---------- keyboard controls ----------
  const keymap = {
    ArrowUp:    "forward",
    ArrowDown:  "backward",
    ArrowLeft:  "left",
    ArrowRight: "right",
    " ":        "stop",
  };
  document.addEventListener("keydown", (e) => {
    const cmd = keymap[e.key];
    if (cmd) {
      e.preventDefault();
      sendCommand(cmd);
    }
  });
})();
