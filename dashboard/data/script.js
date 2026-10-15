// EdgePulse dashboard client.
//
// DATA SOURCE OF TRUTH:
//   The ESP32 pushes measured values over the WebSocket. Values the
//   firmware could not measure arrive as JSON `null` and are rendered as
//   "—". They are NEVER rendered as 0, and never invented.
//
//   `SIM` exists only as a deliberate, labelled fallback for demonstrating
//   the UI with no board attached. It is OFF. The badge in the top-right
//   switches automatically based on what the firmware reports, so the
//   screen can never silently claim to show real data while showing
//   generated data.

(function () {
  "use strict";

  var SIM = false;                      // real data from the ESP32
  var RECONNECT_MS = 2000;
  var MAX_PTS = 600;                    // ~60 s of history at 10 Hz

  var vibHist = [], motorHist = [], ambHist = [];
  var ws = null, live = false, lastMsg = 0;

  // ── element refs ──────────────────────────────────────────────────────
  var $ = function (id) { return document.getElementById(id); };
  var el = {
    wsStatus: $("wsStatus"), badge: $("modeBadge"), banner: $("banner"),
    state: $("stateText"), rule: $("ruleText"),
    rms: $("vRms"), motor: $("vMotor"), amb: $("vAmb"),
    hum: $("vHum"), delta: $("vDelta"), hz: $("vHz"),
    run: $("runId"), cond: $("cond"), pwm: $("pwm"), drop: $("drop"),
    calib: $("calib"), sensors: $("sensors"), rec: $("rec"),
    pwmOut: $("pwmOut"), pwmSlider: $("pwmSlider"),
    btnRun: $("btnRun"), btnStop: $("btnStop"), btnBrake: $("btnBrake"),
    btnStartRun: $("btnStartRun"), btnStopRun: $("btnStopRun"),
    runIn: $("runIn"), condIn: $("condIn"),
    cVib: $("cVib"), cTemp: $("cTemp")
  };

  // ── WebSocket ─────────────────────────────────────────────────────────
  function connect() {
    if (SIM) { startSim(); return; }
    try {
      ws = new WebSocket("ws://" + location.host + "/ws");
    } catch (e) { setLive(false); return; }

    ws.onopen = function () { setLive(true); };
    ws.onclose = function () { setLive(false); setTimeout(connect, RECONNECT_MS); };
    ws.onerror = function () { setLive(false); };
    ws.onmessage = function (ev) {
      var d;
      try { d = JSON.parse(ev.data); } catch (e) { return; }
      lastMsg = Date.now();
      live = true;
      apply(d);
    };
  }

  // ── send a command to the firmware ────────────────────────────────────
  function send(obj) {
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }

  function setLive(on) {
    el.wsStatus.innerHTML = on
      ? '<span class="dot on"></span>live'
      : '<span class="dot"></span>connecting…';
  }

  // ── Simulation (explicit fallback, never silently enabled) ────────────
  var simT = 0, motorT = 28 + Math.random() * 0.5, ambT = 28;

  function startSim() {
    setLive(true);
    el.badge.textContent = "SIMULATED";
    el.badge.className = "badge sim";

    setInterval(function () {
      simT += 0.1;
      var carrier = Math.sin(simT * 2 * Math.PI * 50) * 0.010;
      var noise   = (Math.random() - 0.5) * 0.012;
      var drift   = Math.sin(simT * 0.12) * 0.006;
      var r = carrier + noise + drift;
      motorT += (46 - motorT) * 0.012 + (Math.random() - 0.5) * 0.08;
      ambT   += (28.5 - ambT) * 0.004 + (Math.random() - 0.5) * 0.02;

      apply({
        run: "SIM-001", cond: "SIMULATED", pwm: 0,
        vib_rms: Math.abs(r),
        motor_temp_C: motorT, ambient_temp_C: ambT,
        temp_delta_C: motorT - ambT, humidity: 48,
        status: "SIMULATED", rule: "generated locally — not measured",
        dropped: 0, fs_hz: 0, calibrated: false,
        sensors: { mpu6050: false, ds18b20: false, dht22: false }
      });
    }, 100);
  }

  // ── apply one real reading ────────────────────────────────────────────
  function apply(d) {
    // null means "not measured". Push null into the history so the chart
    // breaks rather than drawing a straight line through a gap.
    vibHist.push(d.vib_rms == null ? null : d.vib_rms);
    motorHist.push(d.motor_temp_C == null ? null : d.motor_temp_C);
    ambHist.push(d.ambient_temp_C == null ? null : d.ambient_temp_C);
    if (vibHist.length > MAX_PTS) {
      vibHist.shift(); motorHist.shift(); ambHist.shift();
    }

    el.rms.textContent   = f(d.vib_rms, 4);
    el.motor.textContent = f(d.motor_temp_C, 1);
    el.amb.textContent   = f(d.ambient_temp_C, 1);
    el.hum.textContent   = f(d.humidity, 1);
    el.delta.textContent = f(d.temp_delta_C, 1);
    el.hz.textContent    = f(d.fs_hz, 2);

    el.run.textContent  = d.run  || "—";
    el.cond.textContent = d.cond || "—";
    el.pwm.textContent  = d.pwm != null ? d.pwm : "—";
    el.drop.textContent = d.dropped != null ? d.dropped : "—";
    el.pwmOut.textContent = d.pwm != null ? d.pwm : "—";

    var st = (d.status || "UNCALIBRATED").toLowerCase();
    el.banner.className = "banner " + st;
    el.state.textContent = d.status || "UNCALIBRATED";
    el.rule.textContent  = d.rule || "—";

    // The badge states the truth about the data on screen.
    el.badge.textContent = d.status === "SIMULATED" ? "SIMULATED" : "LIVE DATA";
    el.badge.className   = "badge live";

    // Sensor presence — the jury asked "is this real?" so answer it on
    // screen rather than in a footnote.
    var s = d.sensors || {};
    el.sensors.innerHTML =
        pill("MPU6050", s.mpu6050) +
        pill("DS18B20", s.ds18b20) +
        pill("DHT22",   s.dht22) +
        pill("INA219",  false);

    // No measured baseline means no opinion, and the screen must say so.
    el.calib.textContent = d.calibrated
      ? "baseline loaded"
      : "no baseline — thresholds not active";
    el.calib.className = d.calibrated ? "ok" : "warn";

    el.rec.textContent = d.recording ? "● REC" : "○ idle";
    el.rec.className   = d.recording ? "rec on" : "rec";

    drawVib();
    drawTemp();
  }

  function pill(name, present) {
    return '<span class="pill ' + (present ? "yes" : "no") + '">' +
           name + (present ? " ✓" : " —") + '</span>';
  }

  function f(v, n) { return (v == null || isNaN(v)) ? "—" : Number(v).toFixed(n); }

  // ── plotting (hand-rolled: no CDN, no chart library) ──────────────────
  // A null breaks the line. Drawing through a gap would imply data that
  // was never measured.
  function prep(cv) {
    var dpr = window.devicePixelRatio || 1;
    var w = cv.clientWidth || 900, h = 180;
    if (cv.width !== w * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
    var c = cv.getContext("2d");
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);
    return { c: c, w: w, h: h };
  }

  function series(cv, sets) {
    var g = prep(cv), pad = 8;
    var lo = Infinity, hi = -Infinity;
    sets.forEach(function (s) {
      s.data.forEach(function (v) {
        if (v == null || isNaN(v)) return;      // ignore gaps for scaling
        if (v < lo) lo = v; if (v > hi) hi = v;
      });
    });
    if (!isFinite(lo)) { lo = 0; hi = 1; }
    if (hi - lo < 1e-9) { hi = lo + 1; }
    var span = hi - lo;
    lo -= span * 0.12; hi += span * 0.12; span = hi - lo;

    // grid
    g.c.strokeStyle = "#21262d"; g.c.lineWidth = 1;
    for (var i = 0; i <= 3; i++) {
      var y = pad + (g.h - 2 * pad) * (i / 3);
      g.c.beginPath(); g.c.moveTo(pad, y); g.c.lineTo(g.w - pad, y); g.c.stroke();
    }

    sets.forEach(function (s) {
      if (s.data.length < 2) return;
      g.c.strokeStyle = s.color;
      g.c.lineWidth = s.width || 1.6;
      var pen = false;
      g.c.beginPath();
      for (var i = 0; i < s.data.length; i++) {
        var v = s.data[i];
        if (v == null || isNaN(v)) { pen = false; continue; }   // break line
        var x = pad + (g.w - 2 * pad) * (i / (MAX_PTS - 1));
        var t = s.data.length === 1 ? 0 : (i / (s.data.length - 1));
        var y = g.h - pad - (g.h - 2 * pad) * ((v - lo) / span);
        if (!pen) { g.c.moveTo(x, y); pen = true; }
        else      { g.c.lineTo(x, y); }
      }
      g.c.stroke();
    });
  }

  function drawVib()  { series(el.cVib,  [{ data: vibHist, color: "#58a6ff", width: 1.4 }]); }
  function drawTemp() {
    series(el.cTemp, [
      { data: motorHist, color: "#f0b429", width: 1.6 },
      { data: ambHist,   color: "#8b949e", width: 1.2 }
    ]);
  }

  window.addEventListener("resize", function () { drawVib(); drawTemp(); });

  // ── control wiring ────────────────────────────────────────────────────
  el.btnRun.onclick   = function () { send({ cmd: "RUN" }); };
  el.btnStop.onclick  = function () { send({ cmd: "STOP" }); };
  el.btnBrake.onclick = function () { send({ cmd: "BRAKE" }); };
  el.btnStartRun.onclick = function () {
    send({ cmd: "START_RUN", run: el.runIn.value || "R001",
           cond: el.condIn.value || "NORMAL" });
  };
  el.btnStopRun.onclick = function () { send({ cmd: "STOP_RUN" }); };

  var pending = null;
  el.pwmSlider.oninput = function () { el.pwmOut.textContent = el.pwmSlider.value; };
  el.pwmSlider.onchange = function () {
    send({ cmd: "SET_PWM", pwm: parseInt(el.pwmSlider.value, 10), dir: 0 });
  };

  // A watchdog: if the socket goes quiet the badge must say so, so the
  // screen can never look live while stale.
  setInterval(function () {
    if (!SIM && live && Date.now() - lastMsg > 3000) {
      live = false;
      setLive(false);
      el.badge.textContent = "STALE";
      el.badge.className = "badge sim";
    }
  }, 1000);

  connect();
})();