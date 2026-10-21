"""
Per-mission trace for the evaluation: which mode ran, which planner made the
plan, every model call (model ID, latency, raw output), the time each stage
took, and what happened during the flight (steps, replans, lost targets).

One trace is open at a time, from the command arriving to the mission
ending. Finished traces are appended to cache/traces/traces.jsonl (or
AERIS_TRACE_DIR) and the latest is served at /api/trace.
"""
from __future__ import annotations

import json
import os
import threading
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Dict, Optional

_RAW_LIMIT = 4000
_lock = threading.Lock()
_current: Optional["MissionTrace"] = None


def _trace_dir() -> Path:
    override = os.getenv("AERIS_TRACE_DIR")
    return Path(override) if override else Path(__file__).resolve().parents[2] / "cache" / "traces"


class MissionTrace:
    def __init__(self, command: str, mode: str, clock: Optional[Callable[[], float]] = None, **meta):
        self.id = uuid.uuid4().hex[:12]
        self.command = command
        self.mode = mode
        self.meta = dict(meta)
        self.clock = clock
        self.t0 = time.time()
        self.stages: Dict[str, float] = {}
        self.model_calls: list = []
        self.events: list = []
        self.planner: Dict[str, Any] = {}
        self.validation: Dict[str, Any] = {}
        self.outcome: Dict[str, Any] = {}
        self.open = True

    def _stamp(self) -> dict:
        out = {"t": round(time.time() - self.t0, 3)}
        if self.clock is not None:
            try:
                out["sim_t"] = round(float(self.clock()), 3)
            except Exception:
                pass
        return out

    def as_dict(self) -> dict:
        return {
            "id": self.id, "command": self.command, "mode": self.mode,
            "started": self.t0, "meta": self.meta, "planner": self.planner, "stages": self.stages,
            "validation": self.validation, "model_calls": self.model_calls, "events": self.events,
            "outcome": self.outcome, "open": self.open,
        }


def begin(command: str, mode: str, clock: Optional[Callable[[], float]] = None, **meta) -> MissionTrace:
    global _current
    tr = MissionTrace(command, mode, clock, **meta)
    with _lock:
        _current = tr
    return tr


def current() -> Optional[MissionTrace]:
    with _lock:
        return _current


def _open() -> Optional[MissionTrace]:
    tr = current()
    return tr if tr is not None and tr.open else None


def add_stage(name: str, seconds: float) -> None:
    tr = _open()
    if tr is not None:
        with _lock:
            tr.stages[name] = round(tr.stages.get(name, 0.0) + seconds, 4)


@contextmanager
def stage(name: str):
    t = time.perf_counter()
    try:
        yield
    finally:
        add_stage(name, time.perf_counter() - t)


def model_call(kind: str, provider: Any, latency_s: float, ok: bool, **extra) -> None:
    """One call to an LLM or VLM. Raw outputs come from the provider's
    `last_attempts` when it records them (one entry per request it sent)."""
    tr = _open()
    if tr is None:
        return
    attempts = []
    for a in getattr(provider, "last_attempts", None) or []:
        a = dict(a)
        if isinstance(a.get("raw"), str):
            a["raw"] = a["raw"][:_RAW_LIMIT]
        attempts.append(a)
    rec = {"kind": kind, "provider": type(provider).__name__, "model": getattr(provider, "model_name", None),
           "latency_s": round(latency_s, 3), "ok": bool(ok), "attempts": attempts, **tr._stamp(), **extra}
    with _lock:
        tr.model_calls.append(rec)


def event(kind: str, **data) -> None:
    tr = _open()
    if tr is not None:
        with _lock:
            tr.events.append({"kind": kind, **tr._stamp(), **data})


def set_planner(**info) -> None:
    tr = _open()
    if tr is not None:
        with _lock:
            tr.planner.update(info)


def set_validation(**info) -> None:
    tr = _open()
    if tr is not None:
        with _lock:
            tr.validation.update(info)


def finish(of: Optional[MissionTrace] = None, **outcome) -> Optional[dict]:
    """Close a trace (the open one by default) and append it to the trace log.
    A mission closes its own trace, so a late finish never closes the next one."""
    tr = of if of is not None else _open()
    if tr is None or not tr.open:
        return None
    with _lock:
        tr.outcome.update(outcome, duration_s=round(time.time() - tr.t0, 3), **tr._stamp())
        tr.open = False
        data = tr.as_dict()
    try:
        d = _trace_dir()
        d.mkdir(parents=True, exist_ok=True)
        with open(d / "traces.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(data, default=str) + "\n")
    except OSError:
        pass
    return data


def snapshot() -> Optional[dict]:
    tr = current()
    if tr is None:
        return None
    with _lock:
        return json.loads(json.dumps(tr.as_dict(), default=str))
