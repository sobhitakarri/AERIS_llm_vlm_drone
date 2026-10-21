"""
Run modes: the baselines and ablations the evaluation compares.

Every mode runs the same backend; a mode only switches components off.
The default comes from AERIS_RUN_MODE (else "full"); a command can name its
own mode, which then holds for that mission.

    full            the complete closed-loop system
    scripted        B0: waypoints supplied with the command, no models, no perception
    open_loop       B3: LLM/VLM plan once, then fly it blind (no tracking, no replanning)
    no_vlm          B1: LLM plans from a prior object map supplied with the command; no VLM
    no_depth_check  VLM boxes grounded by floor projection, without the depth verification
    no_replan       tracking stays on, but target drift never changes the plan
    safety_shadow   the validator's verdict is recorded but does not block (twin / sim only)

The plan cache stays on in every mode. Plans built from a live detection are
never cached, and each mission's trace says whether the cache answered.
"""
from __future__ import annotations

import os
import threading
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class RunMode:
    name: str = "full"
    planner: str = "auto"         # "auto" | "scripted"
    use_vlm: bool = True          # any VLM call, at planning time or in the perception loop
    depth_check: bool = True      # a box must match a physical object in the depth map
    tracking: bool = True         # keep following the target after planning
    replan: bool = True           # target drift patches the plan
    enforce_safety: bool = True   # a rejected plan is not flown
    force_llm: bool = False       # bypass the code router and send the command to the LLM


MODES = {
    "full": RunMode(),
    "scripted": RunMode(name="scripted", planner="scripted", use_vlm=False, tracking=False, replan=False),
    "open_loop": RunMode(name="open_loop", tracking=False, replan=False),
    "no_vlm": RunMode(name="no_vlm", use_vlm=False, tracking=False, replan=False, force_llm=True),
    "no_depth_check": RunMode(name="no_depth_check", depth_check=False),
    "no_replan": RunMode(name="no_replan", replan=False),
    "safety_shadow": RunMode(name="safety_shadow", enforce_safety=False),
}

# Backends on which a plan the validator rejected may still be flown.
SHADOW_BACKENDS = ("twin", "sim")

_lock = threading.Lock()
_active: RunMode = MODES["full"]


def resolve(name: str | None) -> RunMode:
    key = (name or "").strip().lower() or default_name()
    if key not in MODES:
        raise ValueError(f"Unknown run mode '{name}'. Choose from: {', '.join(MODES)}.")
    return MODES[key]


def default_name() -> str:
    return (os.getenv("AERIS_RUN_MODE") or "full").strip().lower()


def activate(name: str | None = None) -> RunMode:
    """Set the mode for the mission about to start."""
    global _active
    mode = resolve(name)
    with _lock:
        _active = mode
    return mode


def current() -> RunMode:
    with _lock:
        return _active


def override(**flags) -> RunMode:
    """For tests: the active mode with some flags changed."""
    global _active
    with _lock:
        _active = replace(_active, **flags)
        return _active


activate(default_name())
