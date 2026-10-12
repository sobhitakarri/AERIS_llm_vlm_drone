/**
 * AERIS ground station: one page, one WebSocket.
 * Everything shown comes from /ws/telemetry and a few REST calls; the page
 * holds no scene knowledge of its own.
 */
(() => {
    const $ = (id) => document.getElementById(id);
    const PROMPTS = [
        'Take off to 1 m and hover',
        'Fly in a square',
        'Find the red bottle and hover above it',
        'Inspect the blue cube from 0.5 m away',
        'Circle around the traffic cone',
        'Go to the yellow crate and land',
        'Return home and land',
    ];

    const state = {
        config: null, cam: null, steps: [], trail: [], truthTrail: [],
        lastSim: null, lastWall: null, rtf: null, telem: null,
    };

    // ------------------------------------------------------------ helpers
    function toast(message, isError = false) {
        const el = $('toast');
        el.textContent = message;
        el.className = 'toast' + (isError ? ' error' : '');
        el.style.display = 'block';
        clearTimeout(el._t);
        el._t = setTimeout(() => { el.style.display = 'none'; }, 4000);
    }

    function chip(id, cls, text) {
        $(id).className = 'chip ' + cls;
        if (text !== undefined) $(id.replace('chip-', 'v-')).textContent = text;
    }

    function log(message, level = 'info', t = Date.now() / 1000) {
        const box = $('log');
        const row = document.createElement('div');
        row.className = level;
        const time = document.createElement('time');
        time.textContent = new Date(t * 1000).toLocaleTimeString([], { hour12: false });
        row.append(time, message);
        box.prepend(row);
        while (box.children.length > 120) box.lastChild.remove();
    }

    async function post(url, body) {
        const res = await fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: body ? JSON.stringify(body) : undefined,
        });
        return res.json();
    }

    const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '-' : Number(v).toFixed(d);

    // ------------------------------------------------------------- config
    async function loadConfig() {
        try {
            const cfg = await (await fetch('/api/config')).json();
            state.config = cfg;
            $('sel-drone').value = cfg.drone_backend;
            $('sel-model').value = cfg.model_mode === 'gemini' ? 'gemini' : 'local_split';
            chip('chip-model', cfg.llm_live ? 'ok' : 'warn', cfg.llm_live ? cfg.model_name : `${cfg.model_name} (offline rules)`);
            buildCameraTabs(cfg.cameras || ['auto']);
            const sel = $('d-name');
            sel.replaceChildren(...(cfg.movable_objects || []).map((n) => new Option(n.replace(/_/g, ' '), n)));
            const twin = cfg.drone_backend === 'twin';
            $('disturb').style.display = twin && sel.options.length ? '' : 'none';
            $('btn-restart').style.display = twin ? '' : 'none';
            $('eval-panel').style.display = twin ? '' : 'none';
        } catch (e) {
            chip('chip-link', 'bad', 'unreachable');
        }
    }

    function buildCameraTabs(cameras) {
        const names = cameras.includes('overhead') ? [...cameras, 'depth'] : cameras;
        const labels = { overhead: 'Overhead', fpv: 'Drone', chase: 'Room', depth: 'Depth', auto: 'Camera' };
        const tabs = $('cam-tabs');
        tabs.replaceChildren(...names.map((name) => {
            const b = document.createElement('button');
            b.type = 'button';
            b.textContent = labels[name] || name;
            b.dataset.cam = name;
            b.onclick = () => setCamera(name);
            return b;
        }));
        setCamera(names.includes(state.cam) ? state.cam : names[0]);
    }

    function setCamera(name) {
        state.cam = name;
        document.querySelectorAll('#cam-tabs button').forEach((b) => b.classList.toggle('on', b.dataset.cam === name));
        const overlay = $('tg-overlay').checked ? 1 : 0;
        const proposals = $('tg-proposals').checked ? 1 : 0;
        $('video').src = `/api/video_feed?cam=${name}&overlay=${overlay}&proposals=${proposals}&t=${Date.now()}`;
        const notes = {
            overhead: 'Overhead RGB-D camera: what perception sees',
            fpv: 'Onboard camera',
            chase: 'Fixed room camera',
            depth: 'Overhead depth (near = warm)',
        };
        $('video-note').textContent = notes[name] || '';
    }

    // -------------------------------------------------------------- plan
    function renderSteps(skills) {
        state.steps = skills || [];
        $('steps').replaceChildren(...state.steps.map((s, i) => {
            const li = document.createElement('li');
            const n = document.createElement('span'); n.className = 'n'; n.textContent = i + 1;
            const k = document.createElement('span'); k.textContent = s.skill;
            const p = document.createElement('span'); p.className = 'p';
            p.textContent = Object.entries(s.params || {}).map(([a, b]) => `${a}=${b}`).join('  ');
            li.append(n, k, p);
            return li;
        }));
    }

    function markSteps(monitor) {
        if (!monitor) return;
        const items = $('steps').children;
        for (let i = 0; i < items.length; i++) {
            const idx = i + 1;
            const running = monitor.status === 'running';
            items[i].className =
                (idx < monitor.current_step || monitor.status === 'done') ? 'done'
                : (idx === monitor.current_step && running) ? 'active'
                : (idx === monitor.current_step && (monitor.status === 'error' || monitor.status === 'aborted')) ? 'failed' : '';
        }
    }

    async function sendCommand(text) {
        const reasoning = $('reasoning');
        reasoning.className = 'reasoning';
        reasoning.textContent = 'Planning...';
        $('plan-source').textContent = '';
        $('cmd-send').disabled = true;
        try {
            const data = await post('/api/command', { command: text });
            if (!data.success) {
                const why = data.errors ? data.errors.join('\n') : (data.error || 'Rejected.');
                reasoning.className = 'reasoning error';
                reasoning.textContent = why;
                renderSteps([]);
                toast('No mission started', true);
                return;
            }
            state.trail = []; state.truthTrail = [];
        } catch (e) {
            reasoning.className = 'reasoning error';
            reasoning.textContent = 'Could not reach the backend.';
        } finally {
            $('cmd-send').disabled = false;
        }
    }

    // --------------------------------------------------------- telemetry
    function onTelemetry(m) {
        state.telem = m;
        const s = m.state;
        $('s-x').textContent = fmt(s.x); $('s-y').textContent = fmt(s.y); $('s-z').textContent = fmt(s.z);
        $('s-roll').textContent = fmt(s.roll, 1); $('s-pitch').textContent = fmt(s.pitch, 1); $('s-yaw').textContent = fmt(s.yaw, 0);
        $('s-speed').textContent = fmt(Math.hypot(s.vx, s.vy, s.vz)) + ' m/s';
        $('s-batt').textContent = `${fmt(s.battery_v)} V  ${s.battery_percentage}%`;
        $('batt-fill').style.width = s.battery_percentage + '%';
        $('batt-fill').style.background = s.battery_percentage < 25 ? 'var(--bad)' : s.battery_percentage < 50 ? 'var(--warn)' : 'var(--good)';
        $('flight-mode').textContent = (s.is_armed ? '' : 'DISARMED · ') + s.flight_mode;

        const connected = m.connected !== false;
        const name = state.config ? (state.config.robot_name || state.config.drone_backend) : '-';
        chip('chip-drone', connected ? 'ok' : 'bad', connected ? name : `${name} (waiting)`);

        const twin = m.twin;
        if (twin) {
            $('s-range').textContent = twin.range != null ? fmt(twin.range) + ' m' : '-';
            $('s-current').textContent = twin.current_a != null ? fmt(twin.current_a) + ' A' : '-';
            const hover = state.config && state.config.airframe ? state.config.airframe.hover_rad_s : 1000;
            const bars = $('motors').querySelectorAll('i');
            (twin.motors || []).forEach((w, i) => { if (bars[i]) bars[i].style.height = Math.min(100, 60 * w / hover) + '%'; });
            if (twin.sim_time != null) {
                const now = performance.now() / 1000;
                if (state.lastSim != null && now - state.lastWall > 1.5) {
                    state.rtf = (twin.sim_time - state.lastSim) / (now - state.lastWall);
                    state.lastSim = twin.sim_time; state.lastWall = now;
                } else if (state.lastSim == null) { state.lastSim = twin.sim_time; state.lastWall = now; }
                const rtf = state.rtf;
                chip('chip-sim', !connected ? 'bad' : rtf == null || rtf > 0.8 ? 'ok' : 'warn',
                    `${fmt(twin.sim_time, 0)} s` + (rtf != null ? `  x${fmt(rtf, 2)}` : ''));
            }
            if (twin.truth) {
                state.truthTrail.push([twin.truth.x, twin.truth.y]);
                if (state.truthTrail.length > 600) state.truthTrail.shift();
            }
        } else {
            chip('chip-sim', '', 'n/a');
        }

        const cal = m.scene;
        if (cal) {
            chip('chip-cal', cal.calibrated ? 'ok' : 'warn',
                cal.calibrated ? `${cal.markers.length} markers  ${fmt(cal.reproj_rms_px, 2)} px` : 'searching for markers');
        } else {
            chip('chip-cal', '', 'not used');
        }

        if (s.is_armed) {
            state.trail.push([s.x, s.y]);
            if (state.trail.length > 600) state.trail.shift();
        }
        if (m.plan) {
            $('reasoning').className = 'reasoning';
            $('reasoning').textContent = m.plan.reasoning || 'Plan ready.';
            $('plan-source').textContent = m.plan.source || '';
            renderSteps(m.plan.skills);
        }
        if (m.last_error && m.monitor && (m.monitor.status === 'error')) {
            $('reasoning').className = 'reasoning error';
            $('reasoning').textContent = m.last_error;
        }
        markSteps(m.monitor);
        renderTargets(m);
        (m.events || []).forEach((e) => log(e.msg, e.level, e.t));
        drawMap(m);
    }

    function renderTargets(m) {
        const byLabel = Object.fromEntries((m.objects || []).map((o) => [o.label, o]));
        const labels = [...new Set([...(m.targets || []), ...Object.keys(byLabel)])];
        const box = $('targets');
        if (!labels.length) {
            box.innerHTML = '<div class="muted">No active targets.</div>';
            return;
        }
        box.replaceChildren(...labels.map((label) => {
            const o = byLabel[label];
            const item = document.createElement('div'); item.className = 'item';
            const t = document.createElement('div'); t.className = 't';
            const name = document.createElement('span'); name.textContent = label;
            const b = document.createElement('span');
            const locked = o && o.actionable && o.world_x != null;
            b.className = 'badge ' + (locked ? 'ok' : o ? 'bad' : 'wait');
            b.textContent = locked ? 'TRACKING' : o ? 'LOST' : 'SEEKING';
            t.append(name, b);
            item.append(t);
            if (locked) {
                const d = document.createElement('div'); d.className = 'd';
                d.textContent = `(${fmt(o.world_x)}, ${fmt(o.world_y)}) m` +
                    (o.top_z != null ? `  top ${fmt(o.top_z)} m` : '') +
                    (o.support_z ? `  on ${fmt(o.support_z)} m` : '') +
                    (o.grounding ? `  [${o.grounding}]` : '');
                item.append(d);
            }
            return item;
        }));
    }

    // ---------------------------------------------------------------- map
    function mapTransform(canvas) {
        const cfg = state.config || {};
        const room = cfg.room && cfg.room.size_m ? cfg.room.size_m : null;
        const g = cfg.geofence || { x_min: -1.5, x_max: 1.5, y_min: -1.5, y_max: 1.5 };
        const xr = room ? [-room[0] / 2, room[0] / 2] : [g.x_min - 0.3, g.x_max + 0.3];
        const yr = room ? [-room[1] / 2, room[1] / 2] : [g.y_min - 0.3, g.y_max + 0.3];
        const pad = 12;
        const scale = Math.min((canvas.width - 2 * pad) / (xr[1] - xr[0]), (canvas.height - 2 * pad) / (yr[1] - yr[0]));
        const ox = (canvas.width - scale * (xr[1] - xr[0])) / 2;
        const oy = (canvas.height - scale * (yr[1] - yr[0])) / 2;
        return {
            xr, yr, g, scale,
            px: (x, y) => [ox + (x - xr[0]) * scale, canvas.height - oy - (y - yr[0]) * scale],
            world: (u, v) => [xr[0] + (u - ox) / scale, yr[0] + (canvas.height - oy - v) / scale],
        };
    }

    function drawMap(m) {
        const c = $('map');
        const ctx = c.getContext('2d');
        const css = getComputedStyle(document.documentElement);
        const col = (n) => css.getPropertyValue(n).trim();
        const T = mapTransform(c);
        ctx.clearRect(0, 0, c.width, c.height);

        // room and metre grid
        const [x0, y0] = T.px(T.xr[0], T.yr[1]);
        ctx.strokeStyle = '#3a4656'; ctx.lineWidth = 1.5;
        ctx.strokeRect(x0, y0, T.scale * (T.xr[1] - T.xr[0]), T.scale * (T.yr[1] - T.yr[0]));
        ctx.strokeStyle = '#18202b'; ctx.lineWidth = 1;
        for (let x = Math.ceil(T.xr[0]); x <= T.xr[1]; x++) { const [a, b] = T.px(x, T.yr[0]); const [, d] = T.px(x, T.yr[1]); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(a, d); ctx.stroke(); }
        for (let y = Math.ceil(T.yr[0]); y <= T.yr[1]; y++) { const [a, b] = T.px(T.xr[0], y); const [d] = T.px(T.xr[1], y); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(d, b); ctx.stroke(); }

        // geofence
        const [gx, gy] = T.px(T.g.x_min, T.g.y_max);
        ctx.strokeStyle = '#2f6b45'; ctx.setLineDash([5, 4]);
        ctx.strokeRect(gx, gy, T.scale * (T.g.x_max - T.g.x_min), T.scale * (T.g.y_max - T.g.y_min));
        ctx.setLineDash([]);

        // home pad
        if (state.config && state.config.home) {
            const [hx, hy] = T.px(state.config.home[0], state.config.home[1]);
            ctx.strokeStyle = '#6b7788'; ctx.beginPath(); ctx.arc(hx, hy, 9, 0, 2 * Math.PI); ctx.stroke();
            ctx.fillStyle = '#6b7788'; ctx.font = '10px monospace'; ctx.fillText('H', hx - 3, hy + 3.5);
        }

        const line = (pts, colour, width = 1.5) => {
            if (pts.length < 2) return;
            ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.beginPath();
            pts.forEach((p, i) => { const [u, v] = T.px(p[0], p[1]); i ? ctx.lineTo(u, v) : ctx.moveTo(u, v); });
            ctx.stroke();
        };
        line(state.truthTrail, col('--truth'), 1);
        line(state.trail, col('--accent'), 1.5);
        if (m.route && m.route.length > 1) {
            line(m.route, col('--route'), 2);
            m.route.slice(1).forEach((p) => { const [u, v] = T.px(p[0], p[1]); ctx.fillStyle = col('--route'); ctx.beginPath(); ctx.arc(u, v, 3, 0, 2 * Math.PI); ctx.fill(); });
        }

        // perceived objects
        (m.objects || []).forEach((o) => {
            if (o.world_x == null) return;
            const [u, v] = T.px(o.world_x, o.world_y);
            ctx.fillStyle = o.actionable ? col('--good') : col('--bad');
            ctx.fillRect(u - 5, v - 5, 10, 10);
            ctx.font = '11px sans-serif'; ctx.fillText(o.label, u + 9, v + 4);
        });

        // drone: truth (hollow) and estimate (filled, with heading)
        const s = m.state;
        if (m.twin && m.twin.truth) {
            const [u, v] = T.px(m.twin.truth.x, m.twin.truth.y);
            ctx.strokeStyle = col('--truth'); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(u, v, 8, 0, 2 * Math.PI); ctx.stroke();
        }
        const [du, dv] = T.px(s.x, s.y);
        const yaw = s.yaw * Math.PI / 180;
        ctx.fillStyle = col('--accent'); ctx.beginPath(); ctx.arc(du, dv, 5, 0, 2 * Math.PI); ctx.fill();
        ctx.strokeStyle = col('--accent'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(du, dv);
        ctx.lineTo(du + 16 * Math.cos(yaw), dv - 16 * Math.sin(yaw)); ctx.stroke();
    }

    // -------------------------------------------------------- evaluation
    async function pollEval() {
        if (!state.config || state.config.drone_backend !== 'twin') return;
        try {
            const e = await (await fetch('/api/eval')).json();
            if (!e.available) return;
            $('e-est').textContent = e.estimator_error_cm != null ? fmt(e.estimator_error_cm, 1) + ' cm' : '-';
            $('e-yaw').textContent = e.yaw_error_deg != null ? fmt(e.yaw_error_deg, 1) + ' deg' : '-';
            $('eval-objects').replaceChildren(...(e.objects || []).map((o) => {
                const item = document.createElement('div'); item.className = 'item';
                const t = document.createElement('div'); t.className = 't';
                const a = document.createElement('span'); a.textContent = o.label;
                const b = document.createElement('span'); b.className = 'badge ' + (o.error_cm < 10 ? 'ok' : 'bad');
                b.textContent = fmt(o.error_cm, 1) + ' cm';
                t.append(a, b);
                const d = document.createElement('div'); d.className = 'd';
                d.textContent = `nearest true object: ${String(o.nearest_truth).replace(/_/g, ' ')}`;
                item.append(t, d);
                return item;
            }));
        } catch (e) { /* backend restarting */ }
    }

    // ------------------------------------------------------------- wiring
    function connect() {
        const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws = new WebSocket(`${proto}//${location.host}/ws/telemetry`);
        ws.onopen = () => { chip('chip-link', 'ok', 'online'); loadConfig(); };
        ws.onmessage = (ev) => onTelemetry(JSON.parse(ev.data));
        ws.onclose = () => { chip('chip-link', 'bad', 'offline'); setTimeout(connect, 2000); };
    }

    document.addEventListener('DOMContentLoaded', () => {
        $('prompts').replaceChildren(...PROMPTS.map((p) => {
            const b = document.createElement('button');
            b.type = 'button'; b.textContent = p;
            b.onclick = () => { $('cmd-input').value = p; $('cmd-input').focus(); };
            return b;
        }));

        $('cmd-form').addEventListener('submit', (e) => {
            e.preventDefault();
            const text = $('cmd-input').value.trim();
            if (text) sendCommand(text);
        });
        $('cmd-input').addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); $('cmd-form').requestSubmit(); }
        });

        $('tg-overlay').onchange = () => setCamera(state.cam);
        $('tg-proposals').onchange = () => setCamera(state.cam);

        const action = (id, url, okMsg) => {
            $(id).onclick = async () => {
                const r = await post(url);
                toast(r.success ? okMsg : (r.error || 'Failed'), !r.success);
            };
        };
        action('btn-land', '/api/land', 'Landing');
        action('btn-home', '/api/home', 'Returning home');
        action('btn-stop', '/api/abort', 'Motors cut');
        $('btn-restart').onclick = async () => {
            const r = await post('/api/twin/restart');
            toast(r.success ? 'Simulation restarting' : (r.error || 'Failed'), !r.success);
            state.trail = []; state.truthTrail = []; state.lastSim = null; state.rtf = null;
        };

        $('sel-drone').onchange = async (e) => {
            const r = await post('/api/config/drone', { drone_backend: e.target.value });
            toast(r.success ? `Vehicle: ${r.drone_backend}` : (r.error || 'Could not connect'), !r.success);
            state.trail = []; state.truthTrail = []; state.lastSim = null; state.rtf = null;
            loadConfig();
        };
        $('sel-model').onchange = async (e) => {
            const r = await post('/api/config/model', { model_mode: e.target.value });
            toast(`Model: ${r.llm_model}` + (r.llm_live ? '' : ' (not reachable)'), !r.llm_live);
            loadConfig();
        };

        const moveObject = async (name, x, y) => {
            const r = await post('/api/twin/object', { name, x, y });
            toast(r.success ? `Moved ${name.replace(/_/g, ' ')} to (${fmt(x)}, ${fmt(y)})` : (r.error || 'Failed'), !r.success);
        };
        $('d-move').onclick = () => {
            const x = parseFloat($('d-x').value), y = parseFloat($('d-y').value);
            if ($('d-name').value && !Number.isNaN(x) && !Number.isNaN(y)) moveObject($('d-name').value, x, y);
        };
        $('map').addEventListener('click', (ev) => {
            const c = $('map');
            const r = c.getBoundingClientRect();
            const [x, y] = mapTransform(c).world((ev.clientX - r.left) * c.width / r.width, (ev.clientY - r.top) * c.height / r.height);
            $('d-x').value = x.toFixed(2); $('d-y').value = y.toFixed(2);
        });

        connect();
        setInterval(pollEval, 1500);
    });
})();
