"""Stop lines and crosswalks from lane paint near junctions.

For each road end at a junction, a road-aligned image of relative lidar
intensity is made from 25 m before the junction edge to 10 m inside it
(``lanes.intensity_image``). Paint across the road shows as bands of bright
pixels. Crosswalks are not always square to the road, so the image is sheared
over a range of angles and the angle that gives the sharpest bands is used.

* crosswalk: a band 1.8-6 m deep covering most of the road width (ladder or
  continental bars), or two thin full-width lines 1.8-4.5 m apart;
* stop line: a thin band (0.2-0.9 m) across the lanes heading into the
  junction, before the crosswalk if there is one.

Each marking is stored as a world polygon (``road.meta["markings"]``); the
OpenDRIVE writer turns them into ``<object>`` records on the road.
"""
from __future__ import annotations

import math
from typing import List

import numpy as np

from uga_terrain import lanes
from uga_terrain.roads import Road

BEFORE = 25.0      # m of road before the junction edge
INSIDE = 10.0      # m into the junction
BIN = 0.2


def _bands(frac: np.ndarray, lo: float, gap: int = 2) -> List[tuple]:
    on = frac >= lo
    idx = np.flatnonzero(on)
    if not len(idx):
        return []
    out, a, b = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - b <= gap + 1:
            b = i
        else:
            out.append((a, b))
            a = b = i
    out.append((a, b))
    return out


def detect_end(road: Road, contact: str, scans_fn, cfg: dict) -> List[dict]:
    """Markings at one junction end of a road. Returns dicts with kind,
    polygon [[x, y]...] and depth/width in metres."""
    if contact == "end":
        e = np.array([road.x[-1], road.y[-1]])
        h = float(road.heading[-1])
    else:
        e = np.array([road.x[0], road.y[0]])
        h = float(road.heading[0]) + math.pi
    u = np.array([math.cos(h), math.sin(h)])
    n = np.array([-u[1], u[0]])
    d = np.arange(-BEFORE, INSIDE + 1e-6, 1.0)
    probe = Road(-1, "probe", d - d[0], e[0] + d * u[0], e[1] + d * u[1], np.full(len(d), h))
    # road width at the end, in the probe's frame (left of travel into the junction)
    def w(key, fill):
        v = np.asarray(road.data.get(key, [fill]), float)
        v = v[np.isfinite(v)]
        return float(np.median(v[-8:] if contact == "end" else v[:8])) if len(v) else fill
    if contact == "end":
        t_left, t_right = w("road_l", 3.5), w("road_r", 3.5)
    else:
        t_left, t_right = w("road_r", 3.5), w("road_l", 3.5)
    hw = max(t_left, t_right) + 2.0
    img = lanes.intensity_image(probe, scans_fn(np.stack([probe.x, probe.y], 1)), hw,
                                float((cfg.get("lanes") or {}).get("paint_range", 20.0)))
    ns, nt = img.shape
    tt = np.arange(nt) * lanes.DT - hw
    ss = np.arange(ns) * lanes.DS                     # 0 = 25 m before the edge
    cols = (tt >= -t_right + 0.3) & (tt <= t_left - 0.3)
    fin = np.isfinite(img)
    vals = img[fin & cols[None, :]]
    if len(vals) < 500:
        return []
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med))) * 1.4826
    thr = med + max(0.8, 5.0 * mad)
    bright = fin & (img > thr)
    S, Tm = np.meshgrid(ss, tt, indexing="ij")
    nb = int(math.ceil((ss[-1] + 2 * hw) / BIN)) + 1
    sel = cols[None, :] & fin
    right_half = cols & (tt < 0)

    def profile(k):
        bi = np.clip(((S - Tm * k + hw) / BIN).astype(int), 0, nb - 1)
        tot = np.bincount(bi[sel], minlength=nb).astype(float)
        hit = np.bincount(bi[sel & bright], minlength=nb).astype(float)
        return np.where(tot > 20, hit / np.maximum(tot, 1), 0.0)

    def bands(k, frac, lo):
        found = []
        for a, b in _bands(frac, lo):
            s0, s1 = a * BIN - hw, (b + 1) * BIN - hw
            sp = S - Tm * k
            inb = (sp >= s0) & (sp < s1)
            colhit = (bright & inb).sum(0) / np.maximum((fin & inb).sum(0), 1)
            covered = colhit > 0.15
            found.append({"s0": s0, "s1": s1, "depth": (b - a + 1) * BIN, "k": k,
                          "cov": float(covered[cols].mean()) if cols.any() else 0.0,
                          "cov_in": float(covered[right_half].mean()) if right_half.any() else 0.0,
                          "f": float(frac[a:b + 1].mean())})
        return found

    angles = np.arange(-30.0, 30.1, 2.5)
    profs = {deg: profile(math.tan(math.radians(deg))) for deg in angles}
    marks = []
    # crosswalks: the angle with the strongest paint band
    deg = max(angles, key=lambda a_: float(np.convolve(profs[a_], np.ones(5) / 5, mode="same").max()))
    k = math.tan(math.radians(deg))
    found = bands(k, profs[deg], 0.12)
    used = set()
    for i, bd in enumerate(found):
        if 1.8 <= bd["depth"] <= 6.0 and bd["cov"] >= 0.45 and bd["f"] >= 0.15:
            marks.append(("crosswalk", bd["s0"], bd["s1"], -t_right, t_left, k, deg))
            used.add(i)
    thin = [i for i, bd in enumerate(found) if i not in used and bd["depth"] <= 0.9 and bd["cov"] >= 0.7]
    for a_i in thin:
        for b_i in thin:
            gap = found[b_i]["s0"] - found[a_i]["s1"]
            if b_i != a_i and a_i not in used and b_i not in used and 1.8 <= gap + found[b_i]["depth"] <= 4.5:
                marks.append(("crosswalk", found[a_i]["s0"], found[b_i]["s1"], -t_right, t_left, k, deg))
                used.update((a_i, b_i))
    first_cw = min((m[1] for m in marks), default=None)
    # stop line: a thin band across the lanes heading in, nearly square to the road
    stop = None
    for dg in angles[np.abs(angles) <= 12.5]:
        kk = math.tan(math.radians(dg))
        for bd in bands(kk, profs[dg], 0.2):
            mid = 0.5 * (bd["s0"] + bd["s1"])
            if not (0.2 <= bd["depth"] <= 0.9) or bd["cov_in"] < 0.55 or bd["f"] < 0.2:
                continue
            if first_cw is not None and not (first_cw - 10.0 <= mid <= first_cw):
                continue
            if first_cw is None and mid > 2.0:
                continue
            if any(m[1] - 0.3 <= mid <= m[2] + 0.3 for m in marks):
                continue
            if stop is None or bd["cov_in"] > stop[0]:
                stop = (bd["cov_in"], bd, kk, dg)
    if stop is not None:
        _, bd, kk, dg = stop
        marks.append(("stop_line", bd["s0"], bd["s1"], -t_right, 0.0, kk, dg))
    out = []
    origin = np.array([probe.x[0], probe.y[0]])
    for kind, s0, s1, t0, t1, k, deg in marks:
        corners = []
        for sv, tv in ((s0, t0), (s1, t0), (s1, t1), (s0, t1)):
            sa = sv + tv * k                          # undo the shear
            corners.append((origin + sa * u + tv * n).round(3).tolist())
        out.append({"kind": kind, "polygon": corners, "depth": round(s1 - s0, 2),
                    "width": round(t1 - t0, 2), "angle_deg": round(float(deg), 1),
                    "from_edge": round(0.5 * (s0 + s1) - BEFORE, 2)})
    return out


def detect(road: Road, junction_ids, scans_fn, cfg: dict) -> List[dict]:
    out = []
    for contact in ("start", "end"):
        if road.meta.get("junction_" + contact) in junction_ids:
            for m in detect_end(road, contact, scans_fn, cfg):
                m["contact"] = contact
                out.append(m)
    return out
