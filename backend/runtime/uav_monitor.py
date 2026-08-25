"""Operator intervention during execution (NeLV UAV Monitor)."""
from __future__ import annotations

import threading
from typing import Optional


class UavMonitor:
    def __init__(self):
        self._abort = threading.Event()
        self.current_step: int = 0
        self.current_skill: str = ""
        self.total_steps: int = 0
        self.status: str = "idle"

    def arm_mission(self, n_steps: int) -> None:
        self._abort.clear()
        self.current_step = 0
        self.current_skill = ""
        self.total_steps = n_steps
        self.status = "running"

    def note_step(self, index: int, skill: str) -> None:
        self.current_step = index
        self.current_skill = skill

    def request_abort(self) -> None:
        self.status = "aborting"
        self._abort.set()

    def aborted(self) -> bool:
        return self._abort.is_set()

    def finish(self, ok: bool) -> None:
        if self.aborted():
            self.status = "aborted"
        else:
            self.status = "done" if ok else "error"

    def snapshot(self) -> dict:
        return {
            "status": self.status,
            "current_step": self.current_step,
            "current_skill": self.current_skill,
            "total_steps": self.total_steps,
            "aborted": self.aborted(),
        }
