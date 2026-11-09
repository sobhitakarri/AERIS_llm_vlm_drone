/* LiteWing flight console. Talks to /api/litewing/* and the 10 Hz /api/litewing/ws feed. */
(() => {
"use strict";
const $ = (id) => document.getElementById(id);
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const API = "/api/litewing";

async function api(path, body, base = API) {
    try {
        const r = await fetch(base + path, body === undefined ? {} : {
            method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
        });
        return await r.json();
    } catch (e) { return { success: false, error: String(e) }; }
}
let toastTimer;
function toast(msg) {
    const t = $("toast"); t.textContent = msg; t.classList.add("show");
    clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove("show"), 3500);
}

/* ------------------------------------------------------------------ data */
const KEYS = ["t", "vx", "vy", "px", "py", "cx", "cy", "h", "r", "tx", "ty"];
const MAXPTS = 6000;
let H = Object.fromEntries(KEYS.map((k) => [k, []]));
let epoch = -1, release = [], state = null, route = [];
function clearHistory() { H = Object.fromEntries(KEYS.map((k) => [k, []])); release = []; }
function pushHistory(h) {
    if (!h) return;
    if (h.epoch !== epoch) { clearHistory(); epoch = h.epoch; }
    for (const k of KEYS) {
        const a = H[k]; for (const v of h[k] || []) a.push(v);
        if (a.length > MAXPTS) a.splice(0, a.length - MAXPTS);
    }
    release = h.release || [];
}

/* ----------------------------------------------------------------- plots */
class Plot {
    constructor(canvas, spec) {
        this.c = canvas; this.spec = spec; this.hover = null;
        canvas.addEventListener("mousemove", (e) => {
            const r = canvas.getBoundingClientRect(); this.hover = e.clientX - r.left; this.draw();
        });
        canvas.addEventListener("mouseleave", () => { this.hover = null; this.draw(); });
    }
    setup() {
        const dpr = window.devicePixelRatio || 1, w = this.c.clientWidth, h = this.c.clientHeight;
        if (this.c.width !== Math.round(w * dpr)) { this.c.width = Math.round(w * dpr); this.c.height = Math.round(h * dpr); }
        const g = this.c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
        return { g, w, h };
    }
    frame(g, w, h, xr, yr, xl, yl, square = false) {
        const m = { l: 52, r: 14, t: 28, b: 34 };          /* top row holds the legend */
        if (square) {      /* equal aspect like the official plot: one metre is the same length on both axes */
            const aw = w - m.l - m.r, ah = h - m.t - m.b, side = Math.min(aw, ah), dx = (aw - side) / 2, dy = (ah - side) / 2;
            m.l += dx; m.r += dx; m.t += dy; m.b += dy;
        }
        const X = (v) => m.l + (v - xr[0]) / (xr[1] - xr[0] || 1) * (w - m.l - m.r);
        const Y = (v) => h - m.b - (v - yr[0]) / (yr[1] - yr[0] || 1) * (h - m.t - m.b);
        g.strokeStyle = css("--grid"); g.fillStyle = css("--muted"); g.lineWidth = 1; g.font = "11px system-ui";
        g.textAlign = "right"; g.textBaseline = "middle";
        for (let i = 0; i <= 4; i++) {
            const v = yr[0] + (yr[1] - yr[0]) * i / 4, y = Y(v);
            g.beginPath(); g.moveTo(m.l, y); g.lineTo(w - m.r, y); g.stroke(); g.fillText(v.toFixed(2), m.l - 6, y);
        }
        g.textAlign = "center"; g.textBaseline = "top";
        for (let i = 0; i <= 4; i++) {
            const v = xr[0] + (xr[1] - xr[0]) * i / 4, x = X(v);
            g.beginPath(); g.moveTo(x, m.t); g.lineTo(x, h - m.b); g.stroke(); g.fillText(v.toFixed(xl === "t" ? 0 : 2), x, h - m.b + 5);
        }
        g.fillStyle = css("--ink2"); g.textAlign = "center";
        g.fillText(xl === "t" ? "Time (s)" : "X (m, right +)", m.l + (w - m.l - m.r) / 2, h - 14);
        g.save(); g.translate(12, m.t + (h - m.t - m.b) / 2); g.rotate(-Math.PI / 2); g.fillText(yl, 0, 0); g.restore();
        return { m, X, Y };
    }
    legend(g, w, items) {
        g.font = "11px system-ui"; g.textAlign = "left"; g.textBaseline = "middle";
        let x = 60; const y = 11;
        for (const it of items) {
            g.fillStyle = it.color; g.fillRect(x, y - 4, 14, 3);
            g.fillStyle = css("--ink2"); g.fillText(it.label, x + 18, y); x += 26 + g.measureText(it.label).width;
        }
    }
    draw() {
        const { g, w, h } = this.setup();
        if (this.spec.kind === "xy") return this.drawXY(g, w, h);
        const t = H.t; const series = this.spec.series;
        if (t.length < 2) { g.fillStyle = css("--muted"); g.textAlign = "center"; g.fillText("waiting for data", w / 2, h / 2); return; }
        let lo = Infinity, hi = -Infinity;
        for (const s of series) for (const v of H[s.key]) { if (v < lo) lo = v; if (v > hi) hi = v; }
        if (this.spec.ref !== undefined) { lo = Math.min(lo, this.spec.ref); hi = Math.max(hi, this.spec.ref); }
        const pad = Math.max((hi - lo) * 0.1, this.spec.minPad || 0.05); lo -= pad; hi += pad;
        const f = this.frame(g, w, h, [t[0], t[t.length - 1]], [lo, hi], "t", this.spec.ylabel);
        g.save(); g.beginPath(); g.rect(f.m.l, f.m.t, w - f.m.l - f.m.r, h - f.m.t - f.m.b); g.clip();
        if (this.spec.ref !== undefined) {
            g.strokeStyle = css("--target"); g.setLineDash([5, 4]); g.beginPath();
            g.moveTo(f.m.l, f.Y(this.spec.ref)); g.lineTo(w, f.Y(this.spec.ref)); g.stroke(); g.setLineDash([]);
        }
        for (const s of series) {
            g.strokeStyle = css(s.color); g.lineWidth = s.width || 2; g.setLineDash(s.dash || []); g.beginPath();
            const a = H[s.key], step = Math.max(1, Math.floor(t.length / 1200));
            for (let i = 0; i < t.length; i += step) { const x = f.X(t[i]), y = f.Y(a[i]); i === 0 ? g.moveTo(x, y) : g.lineTo(x, y); }
            g.stroke(); g.setLineDash([]);
        }
        g.restore();
        this.legend(g, w, [...series.map((s) => ({ label: s.label, color: css(s.color) })),
            ...(this.spec.ref !== undefined ? [{ label: "Target", color: css("--target") }] : [])]);
        if (this.hover !== null && this.hover > f.m.l && this.hover < w - f.m.r) {
            const frac = (this.hover - f.m.l) / (w - f.m.l - f.m.r);
            const i = Math.min(t.length - 1, Math.max(0, Math.round(frac * (t.length - 1))));
            g.strokeStyle = css("--ink2"); g.lineWidth = 1; g.beginPath(); g.moveTo(f.X(t[i]), f.m.t); g.lineTo(f.X(t[i]), h - f.m.b); g.stroke();
            const lines = [`t ${t[i].toFixed(1)} s`, ...series.map((s) => `${s.label}: ${H[s.key][i].toFixed(3)}`)];
            this.tip(g, w, f.X(t[i]), lines);
        }
    }
    tip(g, w, x, lines) {
        g.font = "11px system-ui"; const bw = Math.max(...lines.map((l) => g.measureText(l).width)) + 14, bh = lines.length * 15 + 8;
        const bx = x + 10 + bw > w ? x - bw - 10 : x + 10;
        g.fillStyle = css("--panel"); g.strokeStyle = css("--line"); g.beginPath(); g.rect(bx, 30, bw, bh); g.fill(); g.stroke();
        g.fillStyle = css("--ink"); g.textAlign = "left"; g.textBaseline = "top";
        lines.forEach((l, k) => g.fillText(l, bx + 7, 34 + k * 15));
    }
    drawXY(g, w, h) {
        const xs = H.px.map((v) => -v), ys = H.py;       /* official plot: horizontal = -X (right +) */
        const pts = [...xs.map((x, i) => [x, ys[i]]), [0, 0], ...release.map((p) => [-p[0], p[1]]),
            ...route.map((p) => [-p[1], p[0]])];
        let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
        for (const [x, y] of pts) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
        const r = Math.max(x1 - x0, y1 - y0, 0.3) * 0.7, cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
        const f = this.frame(g, w, h, [cx - r, cx + r], [cy - r, cy + r], "x", "Y (m, forward +)", true);
        const m = f.m, X = f.X, Y = f.Y;                  /* the same mapping the tick labels use */
        g.save(); g.beginPath(); g.rect(m.l, m.t, w - m.l - m.r, h - m.t - m.b); g.clip();
        if (route.length > 1) {                           /* planned legs */
            g.strokeStyle = css("--muted"); g.setLineDash([4, 4]); g.beginPath();
            route.forEach((p, i) => { const x = X(-p[1]), y = Y(p[0]); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke(); g.setLineDash([]);
        }
        g.strokeStyle = css("--purple"); g.lineWidth = 2; g.globalAlpha = .8; g.beginPath();
        const step = Math.max(1, Math.floor(xs.length / 1500));
        for (let i = 0; i < xs.length; i += step) { const x = X(xs[i]), y = Y(ys[i]); i ? g.lineTo(x, y) : g.moveTo(x, y); }
        g.stroke(); g.globalAlpha = 1;
        g.fillStyle = css("--warn"); g.font = "bold 12px system-ui"; g.textAlign = "center"; g.textBaseline = "middle";
        for (const p of release) g.fillText("x", X(-p[0]), Y(p[1]));
        if (state) {                                      /* target ring */
            const v = state.values; g.strokeStyle = css("--target"); g.lineWidth = 2; g.beginPath();
            g.arc(X(-v.target_x), Y(v.target_y), 7, 0, 7); g.stroke();
        }
        g.fillStyle = "#f2d230"; g.strokeStyle = "#111"; g.beginPath(); g.arc(X(0), Y(0), 6, 0, 7); g.fill(); g.stroke();
        if (xs.length) { g.fillStyle = css("--danger"); g.beginPath(); g.arc(X(xs[xs.length - 1]), Y(ys[ys.length - 1]), 5, 0, 7); g.fill(); }
        g.restore();
        this.legend(g, w, [{ label: "Trajectory", color: css("--purple") }, { label: "Current", color: css("--danger") },
            { label: "Origin", color: "#f2d230" }, { label: "Target", color: css("--target") }]);
    }
}
const plots = [
    new Plot($("p-vel"), { series: [{ key: "vx", label: "VX", color: "--s1" }, { key: "vy", label: "VY", color: "--s2", dash: [6, 3] }], ylabel: "Velocity (m/s)" }),
    new Plot($("p-pos"), { kind: "xy" }),
    new Plot($("p-corr"), { series: [{ key: "cx", label: "Corr VX", color: "--s3" }, { key: "cy", label: "Corr VY", color: "--s2", dash: [6, 3] }], ylabel: "Correction", minPad: 0.02 }),
    new Plot($("p-h"), { series: [{ key: "h", label: "Height", color: "--s2" }, { key: "r", label: "Range (ToF)", color: "--s1", dash: [3, 3], width: 1.5 }], ylabel: "Height (m)", ref: 0.3 }),
];
const redraw = () => plots.forEach((p) => p.draw());
window.addEventListener("resize", redraw);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", redraw);

/* ------------------------------------------------------------ parameters */
const PARAMS = [
    ["target_height", "Target Height (m):"], ["takeoff_time", "Takeoff Time (s):"], ["stabilize_time", "Stabilize Time (s):"],
    ["hover_duration", "Hover Duration (s):"], ["landing_time", "Landing Time (s):"], ["descent_rate", "Descent Rate (m/s):"],
    ["idle_hold_s", "Idle Hold Before Landing (s):"], ["smoothing_alpha", "Velocity Smoothing α:"],
    ["max_correction", "Max Correction:"], ["velocity_threshold", "Velocity Threshold (m/s):"],
    ["drift_comp_rate", "Drift Compensation Rate:"], ["reset_interval", "Reset Interval (s):"],
    ["max_position_error", "Max Position Error (m):"], ["maneuver_threshold", "Arrival Threshold (m):"],
    ["waypoint_stabilization", "Waypoint Stabilize (s):"], ["waypoint_timeout", "Waypoint Timeout (s):"],
];
const PID = {
    "f-pos": [["pos_kp", "Kp:"], ["pos_ki", "Ki:"], ["pos_kd", "Kd:"]],
    "f-vel": [["vel_kp", "Kp:"], ["vel_ki", "Ki:"], ["vel_kd", "Kd:"]],
    "f-trim": [["trim_vx", "TRIM VX:"], ["trim_vy", "TRIM VY:"]],
    "f-flow": [["flow_scale", "Scale Factor:"], ["flow_gain", "Calibration Gain:"], ["use_height_scaling", "Use Height Scaling", "bool"]],
};
function field(key, label, kind, cfg, lim) {
    const l = document.createElement("label"); const s = document.createElement("span"); s.textContent = label; l.append(s);
    const i = document.createElement("input"); i.dataset.key = key;
    if (kind === "bool") { i.type = "checkbox"; i.checked = !!cfg[key]; }
    else { i.type = "number"; i.step = "any"; i.value = cfg[key]; if (lim) { i.min = lim[0]; i.max = lim[1]; l.title = `allowed ${lim[0]} to ${lim[1]}`; } }
    l.append(i); return l;
}
function populate() {
    if (!state) return;
    const put = (id, defs) => { const el = $(id); el.replaceChildren(...defs.map(([k, lb, kind]) => field(k, lb, kind, state.config, state.limits[k]))); };
    put("f-params", PARAMS.map((p) => p)); Object.entries(PID).forEach(([id, d]) => put(id, d));
}
function collect(ids) {
    const out = {};
    for (const id of ids) $(id).querySelectorAll("input").forEach((i) => { out[i.dataset.key] = i.type === "checkbox" ? i.checked : i.value; });
    return out;
}
async function applyValues(ids) {
    const r = await api("/params", { values: collect(ids) });
    const n = Object.keys(r.clamped || {}).length;
    toast(n ? `Applied; ${n} value(s) clamped to the safe range` : "Values applied");
    setTimeout(populate, 150);
}
document.querySelectorAll("[data-apply]").forEach((b) => b.addEventListener("click", () =>
    applyValues(b.dataset.apply === "pid" ? ["f-pos", "f-vel", "f-trim", "f-flow"] : ["f-params"])));
document.querySelectorAll("[data-reset]").forEach((b) => b.addEventListener("click", async () => { await api("/params/reset", {}); setTimeout(populate, 150); toast("Defaults restored"); }));
document.querySelectorAll("[data-open]").forEach((b) => b.addEventListener("click", () => { populate(); $(b.dataset.open).showModal(); }));

/* ----------------------------------------------------------------- values */
function fmt(v, d = 3) { return Number.isFinite(v) ? v.toFixed(d) : "-"; }
function renderValues(s) {
    const v = s.values, bad = (c) => c ? " bad" : "";
    const rows = [
        ["Height", `${fmt(v.height)} m`, v.stale], ["Range", `${fmt(v.range)} m`, v.stale], ["Phase", s.phase],
        ["Battery", v.battery_v > 0 ? `${fmt(v.battery_v, 2)} V${v.low_battery ? " (LOW!)" : ""}` : "N/A", v.low_battery],
        ["VX", `${fmt(v.vx)} m/s`, v.stale], ["VY", `${fmt(v.vy)} m/s`, v.stale],
        ["Position X", `${fmt(v.pos_x)} m`, v.stale], ["Position Y", `${fmt(v.pos_y)} m`, v.stale],
        ["Correction VX", fmt(v.corr_vx)], ["Correction VY", fmt(v.corr_vy)],
        ["Target X / Y", `${fmt(v.target_x, 2)} / ${fmt(v.target_y, 2)}`], ["Yaw", `${fmt(v.yaw, 0)}°`],
        ["Roll / Pitch", `${fmt(v.roll, 1)}° / ${fmt(v.pitch, 1)}°`], ["Mode", s.flight_mode],
        ["Flow Δx / Δy", `${fmt(v.flow_dx, 0)} / ${fmt(v.flow_dy, 0)} counts`],
        ["Surface quality", v.squal === null || v.squal === undefined ? "n/a" : String(v.squal)],
        ["Flow gain", fmt(s.config.flow_gain, 3)],
    ];
    $("values").innerHTML = rows.map(([k, val, b]) => `<div><span>${k}</span><b class="${bad(b)}">${val}${b && v.stale ? " (STALE!)" : ""}</b></div>`).join("");
}
function setChip(id, cls, text) { const c = $("c-" + id); c.className = "chip " + cls; $("v-" + id).textContent = text; }

/* ------------------------------------------------------------------- plan */
function renderPlan(p) {
    if (!p) return;
    $("reasoning").textContent = p.reasoning || "";
    $("plan-src").textContent = p.source || "";
    $("steps").innerHTML = p.skills.map((s, i) => {
        const a = Object.entries(s.params || {}).filter(([k]) => k !== "speed").map(([k, v]) => `${k}=${typeof v === "number" ? +v.toFixed(2) : v}`).join(" ");
        return `<li data-i="${i + 1}">${s.skill} <code>${a}</code></li>`;
    }).join("");
    const clips = (p.clips || []);
    const box = $("clips");
    if (clips.length) {
        const seen = new Set();
        box.hidden = false;
        box.textContent = "Limited to the vehicle envelope: " + clips.filter((c) => { const k = c.param + c.requested; return seen.has(k) ? 0 : seen.add(k); })
            .map((c) => `${c.param} ${c.requested} → ${c.clipped} m`).join(", ");
    } else box.hidden = true;
}
function markSteps(m) {
    if (!m || !m.monitor) return;
    const cur = m.monitor.current_step, done = m.monitor.status === "done";
    document.querySelectorAll("#steps li").forEach((li) => {
        const i = +li.dataset.i; li.className = done || i < cur ? "done" : i === cur && m.busy ? "cur" : "";
    });
    $("prog").style.width = (m.monitor.total_steps ? (done ? 100 : Math.max(0, cur - 1) / m.monitor.total_steps * 100) : 0) + "%";
    $("mission-line").textContent = `Mission: ${m.monitor.status}` + (m.busy ? ` · step ${cur}/${m.monitor.total_steps} ${m.monitor.current_skill}` : "") +
        (m.error ? ` · ${m.error}` : "");
}

/* ------------------------------------------------------------------ state */
function applyState(s) {
    const first = !state; state = s; route = (s.mission && s.mission.route) || [];
    if (first) populate();
    const live = s.connected && !s.debug;
    setChip("link", s.connected ? "ok" : "bad", s.connected ? (s.simulated ? "bench sim" : "connected") : "disconnected");
    setChip("mode", s.simulated ? "warn" : (s.debug ? "warn" : "bad"), s.simulated ? "BENCH SIM" : (s.debug ? "DEBUG (motors off)" : "LIVE (motors on)"));
    setChip("ceil", "ok", `${Math.round(s.ceiling * 100)} cm`);
    setChip("sens", s.values.stale ? "bad" : (s.connected ? "ok" : ""), s.values.stale ? "stale" : (s.connected ? "streaming" : "-"));
    $("status").textContent = "Status: " + (s.tripped ? `SAFETY: ${s.tripped}` : s.phase + (s.sensor_test ? " (sensor test)" : ""));
    const b = $("banner");
    if (s.tripped) { b.hidden = false; b.className = "banner trip"; b.textContent = `Safety stop: ${s.tripped}`; }
    else if (s.connected && s.debug) { b.hidden = false; b.className = "banner"; b.textContent = "DEBUG MODE: no motor command is sent. Position and height are simulated from the commanded setpoints."; }
    else if (s.connected && s.simulated) { b.hidden = false; b.className = "banner"; b.textContent = "BENCH SIMULATION: the flight stack runs against a simulated LiteWing. No hardware involved."; }
    else if (live && s.armed) { b.hidden = false; b.className = "banner live"; b.textContent = "LIVE: motors on. Keep clear. STOP lands, E-STOP cuts the motors."; }
    else b.hidden = true;
    $("ck-debug").checked = s.debug; $("ck-csv").checked = s.csv.active;
    $("btn-sensor").textContent = s.sensor_test ? "Stop Sensor Test" : "Sensor Test";
    $("btn-sensor").classList.toggle("on", s.sensor_test);
    $("btn-connect").textContent = s.connected ? "Reconnect" : "Connect";
    const busy = s.mission && s.mission.busy;
    document.querySelectorAll("[data-fly], #btn-start").forEach((x) => { x.disabled = !s.connected || busy || s.manual.active || s.sensor_test; });
    $("cmd-send").disabled = !s.connected || busy;
    $("joy-start").textContent = s.manual.active ? "Stop Joystick" : "Start Joystick";
    $("joy-start").className = "btn " + (s.manual.active ? "stop" : "go");
    renderValues(s); markSteps(s.mission);
    $("model-tag").textContent = s.model || "";
}
function appendLog(lines) {
    if (!lines || !lines.length) return;
    const el = $("outlog"), stick = el.scrollTop + el.clientHeight >= el.scrollHeight - 20;
    el.append(lines.map((l) => `[${l.t}] ${l.msg}`).join("\n") + "\n");
    if (stick) el.scrollTop = el.scrollHeight;
}
function appendEvents(evs) {
    const el = $("events");
    for (const e of evs || []) {
        const d = document.createElement("div"); if (e.level === "error") d.className = "error";
        d.textContent = `${new Date(e.t * 1000).toLocaleTimeString()}  ${e.msg}`; el.prepend(d);
    }
    while (el.children.length > 60) el.lastChild.remove();
}

let ws, retry = 0;
function connectWS() {
    ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}${API}/ws`);
    setChip("link", "warn", "connecting to backend...");
    ws.onopen = () => { retry = 0; lastMsg = Date.now(); setChip("link", "warn", "socket open, waiting for data"); };
    ws.onmessage = (ev) => {
        lastMsg = Date.now();
        try {
            const m = JSON.parse(ev.data);
            if (m.error) { setChip("link", "bad", "backend error"); $("status").textContent = "Status: backend error: " + m.error; return; }
            if (!m.active) { setChip("link", "bad", "no LiteWing"); $("status").textContent = "Status: press Connect"; return; }
            pushHistory(m.history); applyState(m); appendLog(m.log); appendEvents(m.events);
            if (m.plan) renderPlan(m.plan);
            redraw(); if (!$("table-wrap").hidden) renderTable();
        } catch (e) {
            setChip("link", "bad", "page error");
            $("status").textContent = "Status: page error: " + e.message + " (press F12 for details)";
            console.error("litewing console:", e);
        }
    };
    ws.onerror = () => setChip("link", "bad", "socket error");
    ws.onclose = () => { setChip("link", "bad", "backend offline"); setTimeout(connectWS, Math.min(5000, 500 * ++retry)); };
}
let lastMsg = Date.now();
setInterval(() => {                      /* a socket that is open but silent must not look healthy */
    if (ws && ws.readyState === 1 && Date.now() - lastMsg > 3000) {
        setChip("link", "bad", "no data from backend");
        $("status").textContent = "Status: the backend has stopped sending. Check its terminal for a traceback.";
    }
}, 1000);

/* --------------------------------------------------------------- controls */
$("uri").addEventListener("input", () => { if ($("uri").value.startsWith("sim://")) $("ck-debug").checked = false; else $("ck-debug").checked = true; });
$("btn-connect").addEventListener("click", async () => {
    const r = await api("/connect", { uri: $("uri").value.trim(), debug: $("ck-debug").checked });
    toast(r.connected ? "Connected" : (r.error || r.detail || "Could not connect"));
});
$("ck-debug").addEventListener("change", async (e) => {
    const want = e.target.checked;
    if (!want && !confirm("Turn debug mode OFF?\n\nSetpoints will go to the motors. Props on, area clear, battery charged.")) { e.target.checked = true; return; }
    const r = await api("/debug", { enabled: want });
    if (!r.success) { e.target.checked = !want; toast(r.error || r.detail || "Could not change debug mode"); }
});
$("ck-csv").addEventListener("change", async (e) => { const r = await api("/csv", { enabled: e.target.checked }); if (r.path) toast("Logging to " + r.path); });
$("btn-sensor").addEventListener("click", async () => {
    const r = await api("/sensor_test", { action: state && state.sensor_test ? "stop" : "start", log_csv: $("ck-log").checked });
    if (!r.success) toast(r.error || r.detail);
});
$("cal-btn").addEventListener("click", async () => {
    const r = await api("/calibrate", { distance: parseFloat($("cal-dist").value) });
    if (r.success) { toast(`Measured ${r.measured.toFixed(3)} m for ${r.true.toFixed(3)} m: gain ${r.old_gain.toFixed(3)} → ${r.gain.toFixed(3)}. Repeat the slide to verify.`); populate(); }
    else toast(r.error || r.detail || "Calibration failed");
});
$("btn-battery").addEventListener("click", () => api("/battery/reset", {}));
$("btn-start").addEventListener("click", async () => {
    if (state && !state.debug && !confirm("The drone will take off, hold, and land. Props on, area clear?")) return;
    const r = await api("/flight", { kind: "start_flight" }); if (!r.success) toast(r.error || (r.errors || []).join("; "));
});
document.querySelectorAll("[data-fly]").forEach((b) => b.addEventListener("click", async () => {
    if (state && !state.debug && !state.armed && !confirm("The drone will take off and fly this. Props on, area clear?")) return;
    const r = await api("/flight", { kind: b.dataset.fly, distance: parseFloat($("dist").value) });
    if (!r.success) toast(r.error || (r.errors || []).join("; "));
}));
const stop = async () => { const r = await api("/stop", {}); toast(r.success ? "Landing under control" : (r.detail || r.error)); };
$("btn-stop").addEventListener("click", stop); $("btn-stop2").addEventListener("click", stop); $("joy-stop").addEventListener("click", stop);
$("btn-estop").addEventListener("click", async () => { await api("/api/abort", {}, ""); toast("Motors cut"); });
$("btn-clear").addEventListener("click", () => { clearHistory(); redraw(); });
$("log-clear").addEventListener("click", () => { $("outlog").textContent = ""; });
$("log-save").addEventListener("click", () => {
    const a = document.createElement("a"); a.href = URL.createObjectURL(new Blob([$("outlog").textContent], { type: "text/plain" }));
    a.download = `litewing_log_${Date.now()}.txt`; a.click();
});

/* Motor test */
function motorPrecheck() {
    if (!$("mt-ok").checked) { toast("Tick the box: drone on a flat surface, area clear."); return false; }
    if (state && state.debug) { toast("Untick Debug Mode first: debug never sends a motor command."); return false; }
    return true;
}
$("mt-prop").addEventListener("click", async () => {
    if (!motorPrecheck()) return;
    if (!confirm("Run the propeller self-test?\n\nThe firmware pulses each motor for about 50 ms and takes about 7 s in total. Keep the drone still.")) return;
    $("mt-prop").disabled = true; $("mt-result").textContent = "Running the firmware self-test (about 7 s)...";
    const r = await api("/prop_test", { confirmed: true, props_fitted: $("mt-props").checked });
    $("mt-prop").disabled = false;
    if (!r.motors) { $("mt-result").textContent = "Not run: " + (r.error || r.detail); return; }
    $("mt-result").textContent = (r.success ? "All four motors showed vibration. " : "Check: " + r.error + ". ") +
        r.motors.map((m) => `M${m.motor}: vibration ${m.variance === null ? "n/a" : m.variance.toFixed(3)}${m.vibration_seen ? "" : " (low)"}`).join("  |  ") +
        `  (firmware pass flags ${r.motorPass}; low vibration also counts as a pass there)`;
});
$("mt-run").addEventListener("click", async () => {
    if (!motorPrecheck()) return;
    if (!confirm("Run the motor test now?\n\nAll four motors will spin at " + $("mt-pct").value + "% for " + $("mt-sec").value + " s.")) return;
    $("mt-run").disabled = true; $("mt-result").textContent = "Running...";
    const r = await api("/motor_test", { percent: parseFloat($("mt-pct").value), seconds: parseFloat($("mt-sec").value), confirmed: true, props_fitted: $("mt-props").checked });
    $("mt-run").disabled = false;
    if (r.error && r.motor_power_peak === undefined) { $("mt-result").textContent = "Not run: " + r.error; return; }
    $("mt-result").textContent = (r.success ? "Done. " : "Ended early: " + r.error + ". ") +
        `Firmware motor power seen m1..m4 = ${r.motor_power_peak.join(", ")} (commanded ${r.motor_power_expected}).` +
        (r.battery_min ? ` Battery ${r.battery_start.toFixed(2)} V → min ${r.battery_min.toFixed(2)} V under load.` : "") +
        (r.motors_log_available ? "" : " The firmware's motor log gave no data, so output could not be confirmed.");
});

/* LEDs */
const rgb = () => ["led-r", "led-g", "led-b"].map((i) => Math.max(0, Math.min(255, parseInt($(i).value || 0))));
$("led-pick").addEventListener("input", (e) => { const h = e.target.value; ["led-r", "led-g", "led-b"].forEach((id, i) => { $(id).value = parseInt(h.substr(1 + i * 2, 2), 16); }); });
document.querySelectorAll("[data-led]").forEach((b) => b.addEventListener("click", async () => {
    const [r, g, bl] = rgb(); const x = await api("/led", { action: b.dataset.led, r, g, b: bl }); if (!x.success) toast(x.error || x.detail);
}));

/* Joystick */
const held = new Set();
async function joyKey(k, down) { if (!state || !state.manual.active) return; if (down ? held.has(k) : !held.has(k)) return; down ? held.add(k) : held.delete(k); api("/joystick", { action: "key", key: k, down }); }
$("joy-start").addEventListener("click", async () => {
    if (state && state.manual.active) { await api("/joystick", { action: "stop" }); return; }
    if (state && !state.debug && !confirm("The drone will take off for a joystick session. Props on, area clear?")) return;
    await api("/params", { values: { joystick_sensitivity: $("joy-sens").value } });
    const r = await api("/joystick", { action: "start" }); if (!r.success) toast(r.error || r.detail);
});
$("joy-origin").addEventListener("change", (e) => api("/joystick", { action: "hold", hold_origin: e.target.checked }));
document.querySelectorAll("[data-key]").forEach((b) => {
    const k = b.dataset.key;
    b.addEventListener("pointerdown", () => joyKey(k, true));
    ["pointerup", "pointerleave", "pointercancel"].forEach((ev) => b.addEventListener(ev, () => joyKey(k, false)));
});
document.addEventListener("keydown", (e) => { const k = e.key.toLowerCase(); if ("wasd".includes(k) && k.length === 1 && !e.repeat && !/INPUT|TEXTAREA/.test(document.activeElement.tagName)) { joyKey(k, true); } });
document.addEventListener("keyup", (e) => { const k = e.key.toLowerCase(); if ("wasd".includes(k) && k.length === 1) joyKey(k, false); });

/* Instruction box */
const PROMPTS = ["take off to 0.3 m", "take off to 0.5 m", "fly a square", "fly a 0.4 m square", "move forward 0.3 m", "rotate 90", "hover for 5 seconds", "go home and land", "land"];
$("prompts").innerHTML = PROMPTS.map((p) => `<button type="button">${p}</button>`).join("");
$("prompts").addEventListener("click", (e) => { if (e.target.tagName === "BUTTON") { $("cmd").value = e.target.textContent; $("cmd").focus(); } });
$("cmd-form").addEventListener("submit", async (e) => {
    e.preventDefault();
    const command = $("cmd").value.trim(); if (!command) return;
    if (state && !state.debug && !state.armed && !confirm("This may take off. Props on, area clear?")) return;
    $("cmd-send").disabled = true;
    const r = await api("/command", { command }, "/api");
    if (!r.success) { toast(r.error || (r.errors || []).join("; ") || "Not sent"); $("reasoning").textContent = r.error || (r.errors || []).join("; "); }
    else $("cmd").value = "";
});
$("cmd").addEventListener("keydown", (e) => { if (e.key === "Enter" && !e.shiftKey) { e.preventDefault(); $("cmd-form").requestSubmit(); } });

/* Table view (the chart's data, for screen readers and checking numbers) */
function renderTable() {
    const n = H.t.length, rows = [];
    for (let i = Math.max(0, n - 25); i < n; i++)
        rows.push(`<tr>${["t", "px", "py", "vx", "vy", "cx", "cy", "h", "r"].map((k) => `<td>${H[k][i].toFixed(3)}</td>`).join("")}</tr>`);
    $("table-wrap").innerHTML = `<table><thead><tr>${["t (s)", "X left", "Y fwd", "VX", "VY", "Corr VX", "Corr VY", "Height", "Range"].map((h) => `<th>${h}</th>`).join("")}</tr></thead><tbody>${rows.join("")}</tbody></table>`;
}
$("btn-table").addEventListener("click", () => { const w = $("table-wrap"); w.hidden = !w.hidden; if (!w.hidden) renderTable(); });

/* Opened from the main dashboard: remember the last link */
try { const u = localStorage.getItem("lw_uri"); if (u) $("uri").value = u; } catch (e) { /* storage unavailable */ }
$("uri").addEventListener("change", () => { try { localStorage.setItem("lw_uri", $("uri").value); } catch (e) { /* ignore */ } });
connectWS(); redraw();
})();
