/**
 * Application Entry Point & Event Wiring.
 */
document.addEventListener('DOMContentLoaded', () => {
    refreshHealthStatus();

    // 1. Initialize WebSocket for Live Telemetry
    new TelemetrySocket(null, renderTelemetry, (isOnline) => {
        const textElem = document.getElementById('backend-text');
        const dotElem = document.querySelector('#backend-status .indicator-dot');

        if (isOnline) {
            textElem.innerText = "Online";
            dotElem.className = "indicator-dot online";
            refreshHealthStatus();
        } else {
            textElem.innerText = "Disconnected";
            dotElem.className = "indicator-dot offline";
        }
    });

    // 2. Command Form Submission Handler
    const form = document.getElementById('command-form');
    form.addEventListener('submit', (e) => {
        e.preventDefault();
        const input = document.getElementById('command-input');
        if (input.value.trim()) {
            submitNaturalLanguageCommand(input.value.trim());
        }
    });

    document.getElementById('clear-path-btn').addEventListener('click', clearFlightPath);
    document.getElementById('home-btn').addEventListener('click', returnHome);
    document.getElementById('abort-btn').addEventListener('click', abortMission);
});

async function refreshHealthStatus() {
    try {
        const res = await fetch('/api/health');
        const data = await res.json();
        const droneMap = {
            SimInterface: "SIMULATOR",
            MatlabInterface: "MATLAB",
            LiteWingInterface: "REAL",
        };
        const modelMap = {
            GeminiProvider: "Gemini 2.5 Flash",
            QwenProvider: "Qwen3 (local)",
        };
        const droneEl = document.getElementById('drone-mode-text');
        if (droneEl) {
            droneEl.innerText = droneMap[data.drone_backend] || data.drone_backend || "UNKNOWN";
        }
        const modelEl = document.querySelector('#model-status strong');
        if (modelEl && data.model_backend) {
            const base = modelMap[data.model_backend] || data.model_backend;
            const modelName = data.llm_model ? ` ${data.llm_model}` : "";
            modelEl.innerText = data.llm_live ? `${base}${modelName} (LIVE)` : `${base} (offline)`;
        }
    } catch (e) {
        // health endpoint not reachable yet
    }
}
