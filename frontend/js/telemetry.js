/**
 * AERIS Avionics Telemetry — Altitude Gauge, Compass, 2D Radar & Smooth Transitions.
 */
const trailPoints = [];
const TRAIL_MAX = 400;

// Smooth number transition state
const _smoothValues = {};

function _smoothTransition(elementId, newValue, decimals = 2) {
    const el = document.getElementById(elementId);
    if (!el) return;

    const key = elementId;
    if (_smoothValues[key] === undefined) _smoothValues[key] = 0;

    const current = _smoothValues[key];
    const diff = newValue - current;

    // If change is tiny, skip animation
    if (Math.abs(diff) < 0.001) {
        el.innerText = newValue.toFixed(decimals);
        return;
    }

    // Lerp towards new value
    const lerped = current + diff * 0.3;
    _smoothValues[key] = Math.abs(newValue - lerped) < 0.005 ? newValue : lerped;
    el.innerText = _smoothValues[key].toFixed(decimals);
}

function worldToArena(x, y) {
    const leftPct = ((x + 1.5) / 3.0) * 100;
    const topPct = ((1.5 - y) / 3.0) * 100;
    return {
        left: Math.min(Math.max(leftPct, 2), 98),
        top: Math.min(Math.max(topPct, 2), 98),
    };
}

function appendTrail(x, y) {
    const last = trailPoints[trailPoints.length - 1];
    if (last && Math.hypot(last.x - x, last.y - y) < 0.02) return;
    trailPoints.push({ x, y });
    if (trailPoints.length > TRAIL_MAX) trailPoints.shift();
    const line = document.getElementById('trail-line');
    if (!line) return;
    const pts = trailPoints.map((p) => {
        const a = worldToArena(p.x, p.y);
        return `${a.left.toFixed(2)},${a.top.toFixed(2)}`;
    });
    line.setAttribute('points', pts.join(' '));
}

function clearOverheadTrail() {
    trailPoints.length = 0;
    const line = document.getElementById('trail-line');
    if (line) line.setAttribute('points', '');
}

/**
 * Update the vertical altitude bar gauge.
 */
function updateAltitudeGauge(z) {
    const maxAlt = 1.5;
    const pct = Math.min(Math.max((z / maxAlt) * 100, 0), 100);
    const fill = document.getElementById('alt-fill');
    if (fill) {
        fill.style.height = `${pct}%`;

        // Color shifts at thresholds
        if (z > 1.3) {
            fill.style.background = 'linear-gradient(180deg, #ef4444, rgba(239,68,68,0.3))';
        } else if (z > 1.0) {
            fill.style.background = 'linear-gradient(180deg, #f59e0b, rgba(245,158,11,0.3))';
        } else {
            fill.style.background = 'linear-gradient(180deg, #38bdf8, rgba(56,189,248,0.3))';
        }
    }
}

/**
 * Rotate the compass needle to match yaw heading.
 */
function updateCompass(yawDegrees) {
    const needle = document.getElementById('compass-needle');
    if (needle) {
        needle.style.transform = `rotate(${yawDegrees}deg)`;
    }
}

/**
 * Add an entry to the event log panel.
 */
function addLogEntry(message, level = 'info') {
    const log = document.getElementById('event-log');
    if (!log) return;

    const now = new Date();
    const time = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;

    const entry = document.createElement('div');
    entry.className = `log-entry ${level}`;
    entry.innerHTML = `<span class="log-time">${time}</span> ${message}`;
    log.appendChild(entry);

    // Keep max 50 entries
    while (log.children.length > 50) log.removeChild(log.firstChild);
    log.scrollTop = log.scrollHeight;
}

/**
 * Main telemetry render — called by WebSocket on every packet.
 */
function renderTelemetry(packet) {
    if (!packet || !packet.state) return;

    const state = packet.state;

    // Smooth telemetry values
    _smoothTransition('val-x', state.x);
    _smoothTransition('val-y', state.y);
    _smoothTransition('val-z', state.z);
    _smoothTransition('val-yaw', state.yaw, 1);

    // Speed
    const speed = Math.hypot(state.vx || 0, state.vy || 0, state.vz || 0);
    _smoothTransition('val-vel', speed);

    // Altitude gauge
    updateAltitudeGauge(state.z);

    // Compass
    updateCompass(state.yaw);

    // Flight mode badge
    const stateEl = document.getElementById('val-state');
    if (stateEl) {
        stateEl.innerText = state.flight_mode;

        // Dynamic badge color
        const mode = state.flight_mode;
        if (mode === 'FLYING' || mode === 'MOVING') {
            stateEl.className = 'badge badge-blue';
        } else if (mode === 'EMERGENCY' || mode === 'ERROR') {
            stateEl.className = 'badge badge-red';
        } else if (mode === 'HOVERING') {
            stateEl.className = 'badge badge-amber';
        } else {
            stateEl.className = 'badge badge-green';
        }
    }

    // Planner status chip
    const plannerChip = document.getElementById('planner-status-chip');
    if (plannerChip) {
        const ps = packet.planner_status || 'IDLE';
        plannerChip.innerText = ps;
        if (ps === 'EXECUTING') {
            plannerChip.className = 'badge badge-blue';
        } else if (ps === 'ERROR') {
            plannerChip.className = 'badge badge-red';
        } else {
            plannerChip.className = 'badge badge-muted';
        }
    }

    // Battery
    const batEl = document.getElementById('battery-volts');
    if (batEl) {
        const v = (state.battery_v || 4.1).toFixed(1);
        batEl.innerText = `${v}V`;
    }

    // 2D Vector Arena — drone marker
    const pos = worldToArena(state.x, state.y);
    const marker = document.getElementById('drone-marker');
    if (marker) {
        marker.style.left = `${pos.left}%`;
        marker.style.top = `${pos.top}%`;
    }

    // Trail
    if (state.flight_mode !== 'IDLE' || trailPoints.length > 0) {
        appendTrail(state.x, state.y);
    }

    // Perceived Objects
    if (packet.objects && Array.isArray(packet.objects) && packet.objects.length > 0) {
        const guardObj = document.getElementById('guard-objects');
        if (guardObj) guardObj.innerText = `${packet.objects.length}`;

        // Render first target on arena
        const targetObj = packet.objects[0];
        const targetMarker = document.getElementById('target-marker');
        const targetTag = document.getElementById('target-tag-label');
        if (targetMarker && targetObj && targetObj.world_x != null) {
            const tPos = worldToArena(targetObj.world_x, targetObj.world_y);
            targetMarker.style.left = `${tPos.left}%`;
            targetMarker.style.top = `${tPos.top}%`;
            targetMarker.style.display = 'flex';
            if (targetTag) {
                targetTag.innerText = `${targetObj.label} (${targetObj.confidence.toFixed(2)})`;
            }
        }
    } else {
        const guardObj = document.getElementById('guard-objects');
        if (guardObj) guardObj.innerText = '0';
    }

    const replanChip = document.getElementById('replan-count-chip');
    if (replanChip) {
        const n = (packet.replan && packet.replan.count) || packet.replan_count || 0;
        replanChip.innerText = `replan ${n}`;
        replanChip.className = n > 0 ? 'badge badge-amber' : 'badge badge-muted';
    }
}
