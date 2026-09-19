/**
 * AERIS Mission Control — Command Submission & Skill Timeline.
 */

function showToast(message, isError = false) {
    const toast = document.getElementById('toast-banner');
    const toastMsg = document.getElementById('toast-msg');
    if (!toast || !toastMsg) return;

    toastMsg.innerText = message;
    const dot = toast.querySelector('.toast-dot');
    if (dot) {
        dot.style.background = isError ? 'var(--rust)' : 'var(--sage)';
        dot.style.boxShadow = 'none';
    }
    toast.style.borderColor = isError ? 'var(--rust)' : 'var(--line)';
    toast.style.display = 'flex';

    clearTimeout(toast._hideTimer);
    toast._hideTimer = setTimeout(() => { toast.style.display = 'none'; }, 3500);
}

async function submitNaturalLanguageCommand(rawCommand) {
    const reasoningElem = document.getElementById('reasoning-output');
    const timelineElem = document.getElementById('skill-timeline');
    const stepCountElem = document.getElementById('step-count');
    const pipelineBadge = document.getElementById('pipeline-badge');

    if (reasoningElem) reasoningElem.innerText = 'Reading the instruction…';
    if (timelineElem) timelineElem.innerHTML = '<div class="step">Decomposing into skill primitives...</div>';

    if (typeof addLogEntry === 'function') addLogEntry(`CMD: "${rawCommand}"`, 'info');

    try {
        const response = await fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: rawCommand })
        });

        const data = await response.json();

        if (!data.success) {
            const errDetail = data.errors ? data.errors.join(', ') : (data.error || 'Validation error');
            if (reasoningElem) {
                reasoningElem.innerHTML = `<span style="color:var(--rust);font-weight:600;">Held by the fence:</span>\n${errDetail}`;
            }
            if (timelineElem) timelineElem.innerHTML = '<div class="step empty">Blocked by safety validator.</div>';
            showToast('Plan rejected by safety guardrails!', true);
            if (typeof addLogEntry === 'function') addLogEntry('Plan REJECTED: ' + errDetail, 'error');
            return;
        }

        if (reasoningElem) reasoningElem.innerText = data.reasoning || 'Plan formulated.';
        if (data.pipeline && data.pipeline.length && pipelineBadge) {
            pipelineBadge.innerText = data.pipeline.join(' → ');
        }

        if (timelineElem) timelineElem.innerHTML = '';
        if (data.skills && data.skills.length > 0) {
            if (stepCountElem) stepCountElem.innerText = `${data.skills.length} steps`;

            data.skills.forEach((item, idx) => {
                const div = document.createElement('div');
                div.className = 'step';

                const numSpan = `<span class="step-num">${idx + 1}</span>`;
                const skillSpan = `<span class="step-skill">${item.skill}</span>`;
                const paramsStr = Object.keys(item.params || {}).length > 0
                    ? `<span class="step-params">${JSON.stringify(item.params)}</span>`
                    : '<span class="step-params">()</span>';

                div.innerHTML = `${numSpan} ${skillSpan} ${paramsStr}`;
                timelineElem.appendChild(div);
            });

            showToast(`Mission dispatched: ${data.skills.length} primitives.`);
            if (typeof addLogEntry === 'function') {
                addLogEntry(`Dispatched ${data.skills.length} skills [${data.source || ''}]`, 'info');
            }
        } else {
            if (timelineElem) timelineElem.innerHTML = '<div class="step empty">No steps generated.</div>';
        }

    } catch (err) {
        if (reasoningElem) reasoningElem.innerText = 'Failed to connect to AERIS backend.';
        if (timelineElem) timelineElem.innerHTML = '<div class="step empty">Network error.</div>';
        showToast('Error communicating with AERIS.', true);
        if (typeof addLogEntry === 'function') addLogEntry('Network error: ' + err.message, 'error');
    }
}

async function clearFlightPath() {
    try {
        const response = await fetch('/api/clear-path', { method: 'POST' });
        const data = await response.json();
        if (data.success) {
            if (typeof clearOverheadTrail === 'function') clearOverheadTrail();
            showToast('Flight trail cleared.');
            if (typeof addLogEntry === 'function') addLogEntry('Trail cleared.', 'info');
        }
    } catch (err) {
        showToast('Failed to clear path.', true);
    }
}

async function abortMission() {
    const reasoningElem = document.getElementById('reasoning-output');
    try {
        const response = await fetch('/api/abort', { method: 'POST' });
        const data = await response.json();
        if (data.success) {
            if (reasoningElem) {
                reasoningElem.innerHTML = '<span style="color:var(--rust);font-weight:600;">Stopped.</span> Motors cut; the craft is still.';
            }
            showToast('Stopped.', true);
            if (typeof addLogEntry === 'function') addLogEntry('ABORT TRIGGERED', 'error');
        }
    } catch (err) {
        showToast('Abort signal failed.', true);
    }
}

async function returnHome() {
    const reasoningElem = document.getElementById('reasoning-output');
    const timelineElem = document.getElementById('skill-timeline');
    try {
        const response = await fetch('/api/home', { method: 'POST' });
        const data = await response.json();
        if (data.success) {
            if (typeof clearOverheadTrail === 'function') clearOverheadTrail();
            if (reasoningElem) reasoningElem.innerText = 'Returning to Origin (0, 0, 0).';
            if (timelineElem) timelineElem.innerHTML = '<div class="step empty">RTH. Ready for new mission.</div>';
            showToast('Returning home...');
            if (typeof addLogEntry === 'function') addLogEntry('Return to Home initiated.', 'info');
        }
    } catch (err) {
        showToast('Failed to return home.', true);
    }
}
