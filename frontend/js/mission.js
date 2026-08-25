/**
 * Mission Control & Command Submission module.
 */
async function submitNaturalLanguageCommand(rawCommand) {
    const reasoningElem = document.getElementById('reasoning-output');
    const timelineElem = document.getElementById('skill-timeline');

    reasoningElem.innerText = "Analyzing command and querying LLM planner...";
    timelineElem.innerHTML = `<li class="skill-step">Planning mission sequence...</li>`;

    try {
        const response = await fetch('http://127.0.0.1:8000/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: rawCommand })
        });

        const data = await response.json();

        if (!data.success) {
            reasoningElem.innerHTML = `<span style="color: #ff7b72;">Plan Rejected by Safety Validator:</span> ${data.errors.join(', ')}`;
            timelineElem.innerHTML = `<li class="skill-step empty">Mission execution blocked.</li>`;
            return;
        }

        reasoningElem.innerText = data.reasoning || "Plan generated successfully.";
        if (data.pipeline && data.pipeline.length) {
            reasoningElem.innerText += `\nPipeline: ${data.pipeline.join(' → ')}`;
        }

        timelineElem.innerHTML = '';
        data.skills.forEach((item, idx) => {
            const li = document.createElement('li');
            li.className = 'skill-step';
            li.innerText = `${idx + 1}. ${item.skill} ${JSON.stringify(item.params)}`;
            timelineElem.appendChild(li);
        });

    } catch (err) {
        reasoningElem.innerText = "Failed to connect to backend server API.";
        timelineElem.innerHTML = `<li class="skill-step empty">Error submitting command.</li>`;
    }
}

async function clearFlightPath() {
    const reasoningElem = document.getElementById('reasoning-output');
    try {
        const response = await fetch('/api/clear-path', { method: 'POST' });
        const data = await response.json();
        if (data.success) {
            if (typeof clearOverheadTrail === 'function') clearOverheadTrail();
            reasoningElem.innerText = "Cleared MATLAB flight trails. Drone pose unchanged.";
        } else {
            reasoningElem.innerText = data.error || "Failed to clear path.";
        }
    } catch (err) {
        reasoningElem.innerText = "Failed to reach /api/clear-path.";
    }
}

async function abortMission() {
    const reasoningElem = document.getElementById('reasoning-output');
    try {
        const response = await fetch('/api/abort', { method: 'POST' });
        const data = await response.json();
        if (data.success) {
            reasoningElem.innerText = "Operator abort — emergency stop sent.";
        } else {
            reasoningElem.innerText = data.error || "Abort failed.";
        }
    } catch (err) {
        reasoningElem.innerText = "Failed to reach /api/abort.";
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
            reasoningElem.innerText = "Reset to home (0, 0, 0). Ready for a new mission.";
            timelineElem.innerHTML = `<li class="skill-step empty">At home. No active mission plan.</li>`;
        } else {
            reasoningElem.innerText = data.error || "Failed to return home.";
        }
    } catch (err) {
        reasoningElem.innerText = "Failed to reach /api/home.";
    }
}
