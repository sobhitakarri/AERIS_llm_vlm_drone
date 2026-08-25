/**
 * Telemetry Gauge Renderer + overhead XY trail.
 */
const trailPoints = [];
const TRAIL_MAX = 400;

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
    if (last && Math.hypot(last.x - x, last.y - y) < 0.02) {
        return;
    }
    trailPoints.push({ x, y });
    if (trailPoints.length > TRAIL_MAX) {
        trailPoints.shift();
    }
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

function renderTelemetry(packet) {
    if (!packet || !packet.state) return;

    const state = packet.state;

    document.getElementById('val-x').innerHTML = `${state.x.toFixed(2)} <small>m</small>`;
    document.getElementById('val-y').innerHTML = `${state.y.toFixed(2)} <small>m</small>`;
    document.getElementById('val-z').innerHTML = `${state.z.toFixed(2)} <small>m</small>`;
    document.getElementById('val-yaw').innerHTML = `${state.yaw.toFixed(1)} <small>°</small>`;

    document.getElementById('val-state').innerText = state.flight_mode;
    document.getElementById('val-planner').innerText = packet.planner_status;
    document.getElementById('battery-volts').innerText = `${state.battery_v.toFixed(1)}V (${state.battery_percentage}%)`;

    const pos = worldToArena(state.x, state.y);
    const marker = document.getElementById('drone-marker');
    if (marker) {
        marker.style.left = `${pos.left}%`;
        marker.style.top = `${pos.top}%`;
    }

    if (state.flight_mode !== 'IDLE' || trailPoints.length > 0) {
        appendTrail(state.x, state.y);
    }
}
