"""Make sure starting the app always gives you this version.

If an earlier copy of the app is still running (an old terminal, a closed VS
Code window), it keeps the port, the new copy cannot start, and the browser
shows the old version. Before starting, this module finds such a copy and
stops it; if the port is taken by something else, it picks the next free port.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional

from uga_terrain import __version__


def _in_use(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def _wait_free(host: str, port: int, seconds: float) -> bool:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if not _in_use(host, port):
            return True
        time.sleep(0.2)
    return not _in_use(host, port)


def running_version(host: str, port: int) -> Optional[str]:
    """Version of the terrain app answering on host:port, or None if it is not ours."""
    try:
        with urllib.request.urlopen("http://%s:%d/api/state" % (host, port), timeout=2) as r:
            d = json.loads(r.read().decode())
    except Exception:
        return None
    if isinstance(d, dict) and "version" in d and "work_dir" in d:
        return str(d["version"])
    return None


def _ask_to_quit(host: str, port: int) -> Optional[str]:
    """Newer copies have /api/quit. Returns None on success, else the reason."""
    req = urllib.request.Request("http://%s:%d/api/quit" % (host, port), data=b"{}",
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=3):
            return None
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read().decode()).get("detail") or str(e)
        except Exception:
            return str(e)
    except Exception as e:
        return str(e)


def _listening_pids(port: int):
    """PIDs of this user's processes listening on ``port`` (Linux /proc only)."""
    inodes = set()
    for f in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            lines = Path(f).read_text().splitlines()[1:]
        except OSError:
            continue
        for ln in lines:
            p = ln.split()
            if len(p) > 9 and p[3] == "0A" and int(p[1].rsplit(":", 1)[1], 16) == port:
                inodes.add(p[9])
    if not inodes:
        return []
    out = []
    me = os.getuid() if hasattr(os, "getuid") else None
    for d in Path("/proc").iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            if me is not None and d.stat().st_uid != me:
                continue
            for fd in (d / "fd").iterdir():
                t = os.readlink(fd)
                if t.startswith("socket:[") and t[8:-1] in inodes:
                    out.append(int(d.name))
                    break
        except OSError:
            continue
    return out


def _stop_by_pid(port: int) -> bool:
    stopped = False
    for pid in _listening_pids(port):
        try:
            cmd = Path("/proc/%d/cmdline" % pid).read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "uga_terrain" not in cmd and "uga-terrain" not in cmd:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            stopped = True
        except OSError:
            pass
    return stopped


def claim_port(host: str, port: int, echo=print) -> int:
    """Return a port to serve on, stopping an older copy of this app if it holds ``port``."""
    if not _in_use(host, port):
        return port
    ver = running_version(host, port)
    if ver is not None:
        echo("Another copy of the app (v%s) is already running on port %d; stopping it so "
             "you get v%s." % (ver, port, __version__))
        why = _ask_to_quit(host, port)
        if why is None and _wait_free(host, port, 8):
            return port
        if why and "job" in why.lower():
            echo("It is still running a job (%s), so it was left alone." % why)
        elif _stop_by_pid(port) and _wait_free(host, port, 8):
            return port
    for p in range(port + 1, port + 50):
        if not _in_use(host, p):
            echo("Port %d is busy, so the app is using port %d instead." % (port, p))
            return p
    raise RuntimeError("no free port between %d and %d" % (port, port + 49))
