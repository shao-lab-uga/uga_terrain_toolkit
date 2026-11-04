"""Choose which CPU cores the app runs on (Settings: app.cpus).

"all" uses every core. "e-cores" uses only the efficiency cores of an Intel
hybrid CPU (12th-14th gen): on 13th/14th-gen chips damaged by the Raptor Lake
voltage fault, the fast P-cores give random errors while the E-cores still
work. A list such as "16-31" or "0-3,8" picks cores by number. Linux only;
elsewhere the setting is ignored.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional, Set


def _parse(text: str) -> Set[int]:
    out: Set[int] = set()
    for part in str(text).replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.update(range(int(a), int(b) + 1))
        else:
            out.add(int(part))
    return out


def e_cores() -> Set[int]:
    """Efficiency cores as Linux reports them (empty if the CPU has none)."""
    f = Path("/sys/devices/cpu_atom/cpus")
    try:
        return _parse(f.read_text().strip())
    except (OSError, ValueError):
        return set()


def resolve(spec) -> Optional[Set[int]]:
    """The cores to use, or None for all of them."""
    spec = str(spec or "all").strip().lower()
    if spec in ("all", "", "none"):
        return None
    if spec in ("e-cores", "ecores", "e"):
        return e_cores() or None
    return _parse(spec) or None


def apply(spec) -> Optional[str]:
    """Restrict this process (and the ones it starts afterwards) to the chosen
    cores. Returns a line for the log, or None when nothing was changed."""
    if not hasattr(os, "sched_setaffinity"):
        return None
    try:
        want = resolve(spec)
    except ValueError:
        return "Settings: app.cpus %r is not a core list; using all cores." % (spec,)
    if want is None:
        if str(spec or "all").strip().lower() in ("e-cores", "ecores", "e"):
            return "Settings asks for E-cores only, but this CPU has none; using all cores."
        return None
    have = set(range(os.cpu_count() or 1))
    want &= have
    if not want:
        return "Settings: app.cpus %r names no cores of this computer; using all cores." % (spec,)
    try:
        os.sched_setaffinity(0, want)
    except OSError as e:
        return "Could not limit the app to cores %s: %s" % (spec, e)
    return "Running on %d of %d CPU cores (%s), as set in Settings." % (len(want), len(have), spec)
