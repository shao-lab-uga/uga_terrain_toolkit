"""Progress reporting and cancellation shared by the pipeline and the app."""
from __future__ import annotations

import threading
import time
from typing import Optional


class JobCancelled(Exception):
    pass


class Progress:
    def __init__(self, echo: bool = False):
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self.phase = ""
        self.done = 0
        self.total = 1
        self.detail = ""
        self.started = time.time()
        self.echo = echo
        self._last_echo = 0.0
        self.log: list = []

    def set(self, phase: str, done: int = 0, total: int = 1, detail: str = "") -> None:
        with self._lock:
            new_phase = phase != self.phase
            self.phase, self.done, self.total, self.detail = phase, int(done), max(1, int(total)), detail
        if self.echo and (new_phase or time.time() - self._last_echo > 2.0 or done >= total):
            self._last_echo = time.time()
            print("[%s] %d/%d %s" % (phase, done, total, detail), flush=True)

    def note(self, msg: str) -> None:
        with self._lock:
            self.log.append(msg)
        if self.echo:
            print(msg, flush=True)

    def cancel(self) -> None:
        self._cancel.set()

    def cancelled(self) -> bool:
        return self._cancel.is_set()

    def check(self) -> None:
        if self._cancel.is_set():
            raise JobCancelled("stopped")

    def snapshot(self) -> dict:
        with self._lock:
            return {"phase": self.phase, "done": self.done, "total": self.total,
                    "detail": self.detail, "elapsed": time.time() - self.started,
                    "log": list(self.log[-50:])}


def null_progress() -> Progress:
    return Progress(echo=False)


def maybe(progress: Optional[Progress]) -> Progress:
    return progress if progress is not None else null_progress()
