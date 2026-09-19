/**
 * AERIS Application Controller — Settings, Hardware Management & Event Wiring.
 */
document.addEventListener('DOMContentLoaded', () => {
    // 1. Initial State Fetch
    fetchSystemConfig();

    // 2. Initialize WebSocket Telemetry Stream
    new TelemetrySocket(null, renderTelemetry, (isOnline) => {
        const textElem = document.getElementById('backend-text');
        const dotElem = document.getElementById('backend-dot');

        if (isOnline) {
            if (textElem) textElem.innerText = 'Online';
            if (dotElem) dotElem.className = 'pill-dot online';
            fetchSystemConfig();
            if (typeof addLogEntry === 'function') addLogEntry('WebSocket connected.', 'info');
        } else {
            if (textElem) textElem.innerText = 'Offline';
            if (dotElem) dotElem.className = 'pill-dot offline';
        }
    });

    // 3. Command Form Submission
    const form = document.getElementById('command-form');
    if (form) {
        form.addEventListener('submit', (e) => {
            e.preventDefault();
            const input = document.getElementById('command-input');
            if (input && input.value.trim()) {
                submitNaturalLanguageCommand(input.value.trim());
            }
        });
    }

    // 4. Action Buttons
    const clearBtn = document.getElementById('clear-path-btn');
    if (clearBtn) clearBtn.addEventListener('click', clearFlightPath);

    const homeBtn = document.getElementById('home-btn');
    if (homeBtn) homeBtn.addEventListener('click', returnHome);

    const abortBtn = document.getElementById('abort-btn');
    if (abortBtn) abortBtn.addEventListener('click', abortMission);

    // 5. Preset Flight Macros
    document.querySelectorAll('.macro-chip').forEach((chip) => {
        chip.addEventListener('click', () => {
            const cmd = chip.getAttribute('data-cmd');
            const input = document.getElementById('command-input');
            if (input && cmd) {
                input.value = cmd;
                submitNaturalLanguageCommand(cmd);
            }
        });
    });

    // 6. View Tabs (Radar vs Camera)
    const tabArenaBtn = document.getElementById('tab-arena-btn');
    const tabCameraBtn = document.getElementById('tab-camera-btn');
    const viewArena = document.getElementById('view-arena');
    const viewCamera = document.getElementById('view-camera');

    if (tabArenaBtn && tabCameraBtn && viewArena && viewCamera) {
        tabArenaBtn.addEventListener('click', () => {
            tabArenaBtn.classList.add('active');
            tabCameraBtn.classList.remove('active');
            viewArena.classList.add('active');
            viewCamera.classList.remove('active');
        });

        tabCameraBtn.addEventListener('click', () => {
            tabCameraBtn.classList.add('active');
            tabArenaBtn.classList.remove('active');
            viewCamera.classList.add('active');
            viewArena.classList.remove('active');
            const img = document.getElementById('live-stream-img');
            if (img) img.src = `/api/video_feed?t=${Date.now()}`;
        });
    }

    // 7. Settings Modal
    const settingsModal = document.getElementById('settings-modal');
    const openSettingsBtn = document.getElementById('open-settings-btn');
    const closeSettingsBtn = document.getElementById('close-settings-btn');
    const pillModel = document.getElementById('pill-model-status');
    const pillStream = document.getElementById('pill-stream-status');
    const pillDrone = document.getElementById('pill-drone-status');

    function openModal() {
        if (settingsModal) {
            settingsModal.classList.add('open');
            fetchSystemConfig();
        }
    }

    function closeModal() {
        if (settingsModal) settingsModal.classList.remove('open');
    }

    if (openSettingsBtn) openSettingsBtn.addEventListener('click', openModal);
    if (closeSettingsBtn) closeSettingsBtn.addEventListener('click', closeModal);
    if (pillModel) pillModel.addEventListener('click', openModal);
    if (pillStream) pillStream.addEventListener('click', openModal);
    if (pillDrone) pillDrone.addEventListener('click', openModal);

    if (settingsModal) {
        settingsModal.addEventListener('click', (e) => {
            if (e.target === settingsModal) closeModal();
        });
    }

    // Keyboard shortcut: Escape to close modal
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') closeModal();
    });

    // Radio card styling sync
    document.querySelectorAll('.opt-card input[type="radio"]').forEach((radio) => {
        radio.addEventListener('change', () => {
            const name = radio.name;
            document.querySelectorAll(`input[name="${name}"]`).forEach((r) => {
                const card = r.closest('.opt-card');
                if (card) {
                    card.classList.toggle('selected', r.checked);
                }
            });

            // Show/hide LiteWing config
            if (name === 'cfg-drone') {
                const hwBox = document.getElementById('litewing-config-box');
                if (hwBox) hwBox.style.display = radio.value === 'litewing' ? 'flex' : 'none';
            }
        });
    });

    // 8. Connect LiteWing ESP32
    const connectDroneBtn = document.getElementById('btn-connect-drone');
    if (connectDroneBtn) {
        connectDroneBtn.addEventListener('click', async () => {
            const uriInput = document.getElementById('drone-uri-input');
            const uri = uriInput ? uriInput.value.trim() : 'udp://192.168.43.42:1988';
            const btnText = document.getElementById('btn-connect-text');

            if (btnText) btnText.innerText = 'Connecting...';

            try {
                const res = await fetch('/api/config/drone', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ drone_backend: 'litewing', drone_uri: uri })
                });
                const data = await res.json();

                if (data.success) {
                    if (btnText) btnText.innerText = 'Connected ✓';
                    connectDroneBtn.style.background = 'linear-gradient(135deg, #10b981, #059669)';
                    showToast(`Connected to LiteWing at ${uri}!`);
                    if (typeof addLogEntry === 'function') addLogEntry(`ESP32 connected: ${uri}`, 'info');
                    fetchSystemConfig();
                } else {
                    if (btnText) btnText.innerText = 'Retry';
                    connectDroneBtn.style.background = 'linear-gradient(135deg, #f43f5e, #be123c)';
                    showToast(data.error || 'Connection failed.', true);
                    if (typeof addLogEntry === 'function') addLogEntry('ESP32 connection failed.', 'error');
                }
            } catch (err) {
                if (btnText) btnText.innerText = 'Error';
                showToast('Network error.', true);
            }
        });
    }

    // 9. Apply All Settings
    const applyBtn = document.getElementById('apply-settings-btn');
    if (applyBtn) {
        applyBtn.addEventListener('click', async () => {
            const selectedModel = document.querySelector('input[name="cfg-model"]:checked')?.value || 'gemini';
            const selectedStream = document.querySelector('input[name="cfg-stream"]:checked')?.value || 'webcam';
            const selectedDrone = document.querySelector('input[name="cfg-drone"]:checked')?.value || 'sim';
            const droneUri = document.getElementById('drone-uri-input')?.value.trim() || 'udp://192.168.43.42:1988';

            applyBtn.innerText = 'Applying...';

            try {
                await fetch('/api/config/model', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ model_mode: selectedModel })
                });

                await fetch('/api/config/stream', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ stream_source: selectedStream, camera_index: 0 })
                });

                await fetch('/api/config/drone', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ drone_backend: selectedDrone, drone_uri: droneUri })
                });

                showToast('Settings applied!');
                if (typeof addLogEntry === 'function') {
                    addLogEntry(`Config: model=${selectedModel}, cam=${selectedStream}, drone=${selectedDrone}`, 'info');
                }
                closeModal();
                fetchSystemConfig();

                const img = document.getElementById('live-stream-img');
                if (img) img.src = `/api/video_feed?t=${Date.now()}`;

            } catch (e) {
                showToast('Error updating settings.', true);
            } finally {
                applyBtn.innerText = 'Apply Changes';
            }
        });
    }
});

/**
 * Syncs UI badges and modal radios with the active backend state.
 */
async function fetchSystemConfig() {
    try {
        const res = await fetch('/api/config');
        const data = await res.json();

        // Model
        const badgeModel = document.getElementById('badge-model');
        if (badgeModel) {
            badgeModel.innerText = data.model_mode === 'gemini' ? 'Gemini' : 'Local';
        }
        const rModel = document.querySelector(`input[name="cfg-model"][value="${data.model_mode}"]`);
        if (rModel) { rModel.checked = true; rModel.dispatchEvent(new Event('change')); }

        // Stream
        const badgeStream = document.getElementById('badge-stream');
        const streamLabel = document.getElementById('stream-source-label');
        if (badgeStream) badgeStream.innerText = data.stream_source === 'webcam' ? 'Webcam' : 'PySimverse';
        if (streamLabel) streamLabel.innerText = data.stream_source === 'webcam' ? 'Webcam Feed' : 'PySimverse 3D';

        const rStream = document.querySelector(`input[name="cfg-stream"][value="${data.stream_source}"]`);
        if (rStream) { rStream.checked = true; rStream.dispatchEvent(new Event('change')); }

        // Drone
        const badgeDrone = document.getElementById('badge-drone');
        const dotDrone = document.getElementById('dot-drone');
        const names = { sim: 'Simulator', litewing: 'LiteWing', pysimverse: 'PySimverse', matlab: 'MATLAB' };

        if (badgeDrone) {
            const dName = names[data.drone_backend] || data.drone_backend;
            badgeDrone.innerText = data.drone_connected ? `${dName} ✓` : dName;
        }
        if (dotDrone) {
            dotDrone.className = data.drone_connected ? 'pill-dot online' : 'pill-dot offline';
        }

        const rDrone = document.querySelector(`input[name="cfg-drone"][value="${data.drone_backend}"]`);
        if (rDrone) { rDrone.checked = true; rDrone.dispatchEvent(new Event('change')); }

        const uriInput = document.getElementById('drone-uri-input');
        if (uriInput && data.drone_uri) uriInput.value = data.drone_uri;

        // Geofence
        const gf = data.geofence;
        const gfEl = document.getElementById('guard-geofence');
        if (gfEl && gf && gf.x_min !== undefined) {
            gfEl.innerText = `[${gf.x_min}, ${gf.x_max}]m`;
        }

    } catch (err) {
        // Server not reachable yet
    }
}
