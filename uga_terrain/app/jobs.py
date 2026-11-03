"""Run one pipeline job at a time in a child process.

A crash inside a compiled library (lidar odometry, ground segmentation) then
fails the job with a message instead of closing the app. Progress and Stop go
through a queue and an event.
"""
from __future__ import annotations

import multiprocessing as mp
import queue
import threading
import time
import traceback
import uuid
from typing import Any, Dict, Optional

from uga_terrain.progress import JobCancelled, Progress

_STOP_GRACE_S = 5.0


class _ChildProgress(Progress):
    def __init__(self, q, ev):
        super().__init__()
        self._q, self._ev = q, ev

    def cancel(self) -> None:
        self._ev.set()

    def cancelled(self) -> bool:
        return self._ev.is_set()

    def check(self) -> None:
        if self._ev.is_set():
            raise JobCancelled("stopped")

    def set(self, phase, done=0, total=1, detail=""):
        super().set(phase, done, total, detail)
        self._q.put(("progress", phase, int(done), int(total), detail))

    def note(self, msg):
        super().note(msg)
        self._q.put(("note", msg))


def _child(kind: str, cfg: dict, args: dict, q, ev, crash_log: Optional[str]) -> None:
    import faulthandler
    if crash_log:
        try:
            f = open(crash_log, "a")
            f.write("\n=== job %s %s ===\n" % (kind, time.strftime("%Y-%m-%d %H:%M:%S")))
            f.flush()
            faulthandler.enable(file=f, all_threads=True)
        except OSError:
            pass
    from uga_terrain import pipeline
    prog = _ChildProgress(q, ev)
    try:
        if kind == "build":
            project = pipeline.Project.open(cfg, args["name"])
            result = pipeline.build(project, cfg, prog, start=args.get("start") or "scan")
        else:
            raise ValueError("unknown job %s" % kind)
        q.put(("done", result))
    except JobCancelled:
        q.put(("cancelled",))
    except Exception as e:  # reported to the user as the job's error
        q.put(("error", str(e), traceback.format_exc()))


class JobManager:
    def __init__(self, crash_log: Optional[str] = None):
        self.crash_log = crash_log
        self.job: Optional[Dict[str, Any]] = None
        self._lock = threading.Lock()
        self._cancel = threading.Event()

    def busy(self) -> bool:
        return self.job is not None and self.job["state"] == "running"

    def start(self, kind: str, cfg: dict, args: dict) -> Dict[str, Any]:
        with self._lock:
            if self.busy():
                raise RuntimeError("a job is already running")
            self._cancel = threading.Event()
            self.job = {"id": uuid.uuid4().hex[:8], "kind": kind, "args": args,
                        "state": "running", "phase": "starting", "done": 0, "total": 1,
                        "detail": "", "log": [], "error": None, "result": None,
                        "started": time.time(), "ended": None}
            job = self.job
        threading.Thread(target=self._run, args=(job, kind, cfg, args), daemon=True).start()
        return self.snapshot()

    def stop(self) -> None:
        self._cancel.set()

    def snapshot(self) -> Optional[Dict[str, Any]]:
        if self.job is None:
            return None
        j = dict(self.job)
        j["log"] = list(j["log"][-200:])
        j["elapsed"] = (j["ended"] or time.time()) - j["started"]
        return j

    def _run(self, job, kind, cfg, args) -> None:
        ctx = mp.get_context("spawn")
        q = ctx.Queue()
        ev = ctx.Event()
        proc = ctx.Process(target=_child, args=(kind, cfg, args, q, ev, self.crash_log),
                           name="uga-terrain-%s" % kind, daemon=True)
        proc.start()
        outcome = None
        stop_at = None

        def handle(msg):
            nonlocal outcome
            if msg[0] == "progress":
                _, job["phase"], job["done"], job["total"], job["detail"] = msg
            elif msg[0] == "note":
                job["log"].append(msg[1])
            else:
                outcome = msg

        while True:
            try:
                handle(q.get(timeout=0.2))
                continue
            except queue.Empty:
                pass
            if self._cancel.is_set() and not ev.is_set():
                ev.set()
                stop_at = time.monotonic() + _STOP_GRACE_S
            if stop_at is not None and proc.is_alive() and time.monotonic() > stop_at:
                proc.terminate()
            if not proc.is_alive():
                break
        proc.join()
        while outcome is None:
            try:
                handle(q.get(timeout=0.5))
            except queue.Empty:
                break
        job["ended"] = time.time()
        if outcome is None:
            if self._cancel.is_set():
                job["state"] = "stopped"
            else:
                job["state"] = "error"
                job["error"] = ("The job crashed inside a compiled library (exit code %s). "
                                "The app is still running; details are in %s"
                                % (proc.exitcode, self.crash_log))
        elif outcome[0] == "done":
            job["state"], job["result"] = "done", outcome[1]
        elif outcome[0] == "cancelled":
            job["state"] = "stopped"
        else:
            job["state"], job["error"] = "error", outcome[1]
            job["log"].append(outcome[2])
