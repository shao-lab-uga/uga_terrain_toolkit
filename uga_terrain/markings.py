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


# --- crosswalks straight from the paint around a junction ------------------------------
LINE_TOL = 0.12       # m either side of a paint line
LINE_GAP = 1.2        # m of missing paint that still counts as one line
LINE_MIN = 4.0        # m; shorter paint is not a crosswalk side line


def ground_paint(A: np.ndarray, centre, radius: float) -> np.ndarray:
    """x, y of the paint cells (SNAP_CELL) among ground points A within radius."""
    lo = np.asarray(centre, float) - radius
    shape = (int(2 * radius / SNAP_CELL),) * 2
    _, B = _bright(A, lo, shape)
    if B is None:
        return np.zeros((0, 2))
    return lo + (np.argwhere(B > 0) + 0.5) * SNAP_CELL


def paint_lines(X: np.ndarray, min_len: float = LINE_MIN, n_max: int = 24) -> List[dict]:
    """Straight painted lines in paint cells X, strongest first (Hough + fit).
    Each {p (point), d (unit direction), a0, a1 (extent along d from p)}."""
    lines: List[dict] = []
    if len(X) < 30:
        return lines
    th = np.radians(np.arange(0.0, 180.0, 0.5))
    C, S = np.cos(th), np.sin(th)
    c0 = X.mean(0)
    for _ in range(60):
        if len(X) < 30 or len(lines) >= n_max:
            break
        rb = np.round(((X[:, 0:1] - c0[0]) * C + (X[:, 1:2] - c0[1]) * S) / 0.1).astype(int)
        off = rb.min()
        rb -= off
        acc = np.zeros((rb.max() + 1, len(th)))
        np.add.at(acc, (rb, np.broadcast_to(np.arange(len(th)), rb.shape)), 1)
        acc = acc + np.roll(acc, 1, 0) + np.roll(acc, -1, 0)
        r_i, t_i = np.unravel_index(np.argmax(acc), acc.shape)
        if acc[r_i, t_i] < 30:
            break
        n = np.array([C[t_i], S[t_i]])
        on = np.abs((X - c0) @ n - (r_i + off) * 0.1) < LINE_TOL
        m = X[on].mean(0)
        d = np.linalg.svd(X[on] - m, full_matrices=False)[2][0]
        n = np.array([-d[1], d[0]])
        on = np.flatnonzero(np.abs((X - m) @ n) < LINE_TOL)
        a = (X[on] - m) @ d
        order = np.argsort(a)
        a, on = a[order], on[order]
        took = np.zeros(len(X), bool)
        for sg in np.split(np.arange(len(a)), np.flatnonzero(np.diff(a) > LINE_GAP) + 1):
            ln = a[sg[-1]] - a[sg[0]]
            if ln >= min_len and len(sg) >= 4 * ln:      # >= 0.4 of the cells painted
                lines.append({"p": m, "d": d, "a0": float(a[sg[0]]), "a1": float(a[sg[-1]])})
            took[on[sg]] = True
        if not took.any():
            break
        X = X[~took]
    return lines


def _mid(L):
    return L["p"] + L["d"] * 0.5 * (L["a0"] + L["a1"])


def _poly(p0, d, n, s0, s1, t0, t1):
    return np.round([p0 + d * s0 + n * t0, p0 + d * s1 + n * t0,
                     p0 + d * s1 + n * t1, p0 + d * s0 + n * t1], 3).tolist()


def find_painted(A: np.ndarray, centre, radius: float = 40.0) -> List[dict]:
    """Crosswalks (two parallel side lines 1.8-4.5 m apart, any angle, with or
    without bars between) and stop lines (a lone line parallel to a crosswalk,
    just outside it) in ground points A around a junction. World polygons."""
    from uga_terrain.junction_shape import point_in_poly
    X = ground_paint(A, centre, radius)
    lines = paint_lines(X)
    cand = []
    for i, a in enumerate(lines):
        n = np.array([-a["d"][1], a["d"][0]])
        for j in range(i + 1, len(lines)):
            b = lines[j]
            if abs(a["d"] @ b["d"]) < math.cos(math.radians(5.0)):
                continue
            sep = float((b["p"] - a["p"]) @ n)
            if not 1.8 <= abs(sep) <= 4.5:
                continue
            ba = sorted(float((b["p"] + b["d"] * e - a["p"]) @ a["d"]) for e in (b["a0"], b["a1"]))
            lo, hi = max(a["a0"], ba[0]), min(a["a1"], ba[1])
            if hi - lo >= 3.0:
                cand.append((hi - lo, i, j, sep, lo, hi))
    out: List[dict] = []
    used = set()
    for _, i, j, sep, lo, hi in sorted(cand, reverse=True):
        if i in used or j in used:
            continue
        if any(point_in_poly(*_mid(lines[k]), o["polygon"]) for k in (i, j) for o in out):
            continue                                  # a bar inside a crosswalk already found
        a = lines[i]
        n = np.array([-a["d"][1], a["d"][0]])
        poly = _poly(a["p"], a["d"], n, lo, hi, 0.0, sep)
        c = np.mean(poly, 0)
        if any(point_in_poly(c[0], c[1], o["polygon"]) or point_in_poly(*np.mean(o["polygon"], 0), poly)
               for o in out):
            continue
        used.update((i, j))
        out.append({"kind": "crosswalk", "polygon": poly, "depth": round(abs(sep), 2),
                    "width": round(hi - lo, 2), "source": "paint"})
    # stop lines: lone lines along a crosswalk, 0.5-10 m outside it, not inside any
    stops = []
    for k, L in enumerate(lines):
        if k in used or L["a1"] - L["a0"] > 12.0:
            continue
        mid = _mid(L)
        if any(point_in_poly(mid[0], mid[1], o["polygon"]) for o in out):
            continue
        for cw in out:
            P = np.asarray(cw["polygon"], float)
            d = (P[1] - P[0]) / np.linalg.norm(P[1] - P[0])
            if abs(d @ L["d"]) < math.cos(math.radians(10.0)):
                continue
            n = np.array([-d[1], d[0]])
            t = (mid - P[0]) @ n
            t0, t1 = sorted((0.0, float((P[3] - P[0]) @ n)))
            gap = t0 - t if t < t0 else t - t1
            along = (mid - P[0]) @ d
            if 0.5 <= gap <= 10.0 and -2.0 <= along <= np.linalg.norm(P[1] - P[0]) + 2.0:
                stops.append({"kind": "stop_line", "source": "paint", "depth": 0.3,
                              "width": round(L["a1"] - L["a0"], 2),
                              "polygon": _poly(L["p"], L["d"], np.array([-L["d"][1], L["d"][0]]),
                                               L["a0"], L["a1"], -0.15, 0.15)})
                break
    return out + stops


def merge_found(roads, juncs, found: List[dict]) -> int:
    """Put markings found straight from the paint on the roads: they replace
    the road-by-road guesses of the same kind they overlap. Returns how many."""
    from uga_terrain.junction_shape import point_in_poly
    for r in roads:
        keep = []
        for m in r.meta.get("markings") or []:
            c = np.asarray(m["polygon"], float).mean(0)
            if any(f["kind"] == m["kind"] and (
                    point_in_poly(c[0], c[1], f["polygon"])
                    or point_in_poly(*np.asarray(f["polygon"], float).mean(0), m["polygon"])
                    or float(np.hypot(*(np.asarray(f["polygon"], float).mean(0) - c))) < 3.0)
                   for f in found):
                continue
            keep.append(m)
        if r.meta.get("markings") is not None:
            r.meta["markings"] = keep
    return attach(roads, juncs, found)


# --- copying markings from another map of the same place -------------------------------

def _ground_points(project, centre, radius: float, cfg: dict) -> np.ndarray:
    from uga_terrain import register
    rng = float((cfg.get("register") or {}).get("max_range") or 20.0)
    out = []
    for i in range(len(project.recordings)):
        T = register.poses_of(project, i)
        if T is None:
            continue
        near = np.hypot(T[:, 0, 3] - centre[0], T[:, 1, 3] - centre[1]) < radius + rng
        if not near.any():
            continue
        P = _points_of(project.cache(i) / "frames", T, np.flatnonzero(near), rng)
        if len(P):
            out.append(P[np.hypot(P[:, 0] - centre[0], P[:, 1] - centre[1]) < radius])
    return np.vstack(out) if out else np.zeros((0, 4))


def _points_of(frames_dir, T, idx, rng: float) -> np.ndarray:
    """register._points for the frames ``idx`` only (every second one)."""
    from uga_terrain import register
    import tempfile, os
    sel = idx[::2]
    if not len(sel):
        return np.zeros((0, 4))
    # register._points walks frames 0..N-1; give it just the chosen ones
    with tempfile.TemporaryDirectory() as tmp:
        for k, f in enumerate(sel):
            src = frames_dir / ("%06d.npy" % f)
            if src.is_file():
                os.symlink(src, os.path.join(tmp, "%06d.npy" % k))
        from pathlib import Path
        return register._points(Path(tmp), T[sel], rng)


def transfer(src, dst, cfg: dict, refine: bool = True) -> dict:
    """Markings found on map ``src`` placed on map ``dst`` (both
    pipeline.Project of the same place). GPS puts them in place; the lidar paint
    of both maps is then matched to remove the GPS offset between the drives.
    Returns {"markings": [{kind, polygon, ...}] in dst coordinates, "match": ...}."""
    from uga_terrain import geo, pipeline
    o_s = tuple(src.meta.get("origin") or ())
    o_d = tuple(dst.meta.get("origin") or ())
    if len(o_s) < 3 or len(o_d) < 3:
        raise RuntimeError("both maps need a GPS position")
    marks = []
    for r in pipeline.load_roads(src):
        for m in r.meta.get("markings") or []:
            if m.get("copied_from"):
                continue
            marks.append(dict(m))
    if not marks:
        raise RuntimeError("%s has no stop lines or crosswalks to copy" % src.name)

    def to_dst(P):
        P = np.asarray(P, float)
        lla = geo.enu_to_lla(P[:, 0], P[:, 1], np.zeros(len(P)), o_s)
        return geo.lla_to_enu(lla[:, 0], lla[:, 1], lla[:, 2], o_d)[:, :2]
    for m in marks:
        m["polygon"] = to_dst(m["polygon"])
    allp = np.vstack([m["polygon"] for m in marks])
    centre = allp.mean(0)
    radius = float(np.max(np.hypot(*(allp - centre).T))) + 25.0
    match = None
    C = np.eye(3)
    if refine:
        A = _ground_points(dst, centre, radius, cfg)
        B = _ground_points(src, _back(centre, o_d, o_s), radius, cfg)
        if len(B):
            B[:, :2] = to_dst(B[:, :2])
        match = snap([m["polygon"] for m in marks], A, B)
        if match and match["used"]:
            C = match.pop("C")
        elif match:
            match.pop("C", None)
    out = []
    for m in marks:
        P = np.asarray(m["polygon"], float) @ C[:2, :2].T + C[:2, 2]
        m["polygon"] = np.round(P, 3).tolist()
        m["copied_from"] = src.name
        out.append(m)
    return {"markings": out, "match": match}


def _back(p, o_d, o_s):
    """A dst point in src coordinates."""
    from uga_terrain import geo
    lla = geo.enu_to_lla(np.array([p[0]]), np.array([p[1]]), np.zeros(1), o_d)
    return geo.lla_to_enu(lla[:, 0], lla[:, 1], lla[:, 2], o_s)[0, :2]


SNAP_CELL = 0.1
SNAP_SHIFT = 4.0      # m; GPS of two drives rarely disagrees by more
SNAP_YAW = 3.0        # deg


def _bright(P: np.ndarray, lo, shape):
    """Seen cells and paint cells (relative intensity well above the rest)."""
    keep = np.all((P[:, :2] >= lo) & (P[:, :2] < lo + np.array(shape) * SNAP_CELL), 1) & np.isfinite(P[:, 3])
    k = np.floor((P[keep, :2] - lo) / SNAP_CELL).astype(int)
    cnt = np.zeros(shape)
    tot = np.zeros(shape)
    np.add.at(cnt, (k[:, 0], k[:, 1]), 1.0)
    np.add.at(tot, (k[:, 0], k[:, 1]), P[keep, 3])
    seen = cnt > 0
    if seen.sum() < 2000:
        return None, None
    mean = np.where(seen, tot / np.maximum(cnt, 1), 0.0)
    v = mean[seen]
    med = np.median(v)
    mad = 1.4826 * np.median(np.abs(v - med))
    return seen.astype(float), (seen & (mean > med + max(0.8, 5 * mad))).astype(float)


def snap(polys, A: np.ndarray, B: np.ndarray) -> dict:
    """One rotation + shift that puts the paint of map B (ground points x, y, z,
    relative intensity, already in A's coordinates by GPS) on the paint of map A,
    looking only around the markings ``polys``. Score is the overlap of the two
    paint masks where both maps saw the ground (1 = identical).
    Returns {used, score, score_before, dx, dy, yaw_deg, C (3x3)}."""
    no = {"used": False, "why_not": "too little lidar here", "C": np.eye(3)}
    if len(A) < 1000 or len(B) < 1000:
        return no
    allp = np.vstack(polys)
    c0 = allp.mean(0)
    lo = allp.min(0) - SNAP_AROUND
    hi = allp.max(0) + SNAP_AROUND
    shape = tuple(int(v) for v in np.ceil((hi - lo) / SNAP_CELL))
    OA, PA = _bright(A, lo, shape)
    if OA is None:
        return no
    reach = int(round(SNAP_SHIFT / SNAP_CELL))
    FOA, FPA = np.fft.rfft2(OA), np.fft.rfft2(PA)

    def corr(F, M):   # sum of F's raster under M moved by each shift (circular)
        return np.fft.irfft2(F * np.conj(np.fft.rfft2(M)), s=shape)
    ii = np.r_[0:reach + 1, shape[0] - reach:shape[0]]
    jj = np.r_[0:reach + 1, shape[1] - reach:shape[1]]
    best = None
    base = 0.0
    for yaw in np.arange(-SNAP_YAW, SNAP_YAW + 1e-9, 0.25):
        R = _rot(yaw)
        Bp = B.copy()
        Bp[:, :2] = (B[:, :2] - c0) @ R.T + c0
        OB, PB = _bright(Bp, lo, shape)
        if OB is None:
            continue
        nab = corr(FPA, PB)[np.ix_(ii, jj)]
        na = corr(FPA, OB)[np.ix_(ii, jj)]       # A's paint where B saw ground
        nb = corr(FOA, PB)[np.ix_(ii, jj)]       # B's paint where A saw ground
        sc = np.where((na > 200) & (nb > 200), nab / np.sqrt(np.maximum(na * nb, 1e-9)), 0.0)
        a, b = np.unravel_index(np.argmax(sc), sc.shape)
        di = ii[a] if ii[a] <= reach else ii[a] - shape[0]
        dj = jj[b] if jj[b] <= reach else jj[b] - shape[1]
        if abs(yaw) < 1e-9:
            base = float(sc[0, 0])
        if best is None or sc[a, b] > best[0]:
            best = (float(sc[a, b]), float(yaw), di * SNAP_CELL, dj * SNAP_CELL)
    if best is None:
        return no
    score, yaw, dx, dy = best
    R = _rot(yaw)
    C = np.eye(3)
    C[:2, :2] = R
    C[:2, 2] = c0 - R @ c0 + np.array([dx, dy])
    used = score >= SNAP_MIN
    out = {"used": bool(used), "score": round(score, 3), "score_before": round(base, 3),
           "dx": round(float(dx), 2), "dy": round(float(dy), 2), "yaw_deg": round(yaw, 2), "C": C}
    if not used:
        out["why_not"] = "the paint of the two maps does not line up clearly"
    return out


SNAP_AROUND = 10.0    # m of road around the markings that is compared
SNAP_MIN = 0.3        # paint overlap needed to trust the move


def _rot(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def _duplicate(m, c, roads) -> bool:
    """A marking of the same kind already on any road overlaps or is within 3 m."""
    from uga_terrain.junction_shape import point_in_poly
    for r in roads:
        for o in r.meta.get("markings") or []:
            if o["kind"] != m["kind"]:
                continue
            oc = np.asarray(o["polygon"], float).mean(0)
            if (np.hypot(*(oc - c)) < 3.0 or point_in_poly(c[0], c[1], o["polygon"])
                    or point_in_poly(oc[0], oc[1], m["polygon"])):
                return True
    return False


def attach(roads, juncs, marks, max_dist: float = 25.0) -> int:
    """Put copied markings on the roads of this map: each on the road whose
    junction end is nearest. Markings the map already found itself (same kind,
    within 2 m) are not doubled. Returns how many were added."""
    jids = {j.id for j in juncs}
    ends = []
    for r in roads:
        for contact, i in (("start", 0), ("end", -1)):
            if r.meta.get("junction_" + contact) in jids:
                ends.append((r, contact, np.array([r.x[i], r.y[i]])))
    n = 0
    for m in marks:
        c = np.asarray(m["polygon"], float).mean(0)
        if not ends:
            break
        r, contact, e = min(ends, key=lambda t: float(np.hypot(*(t[2] - c))))
        if np.hypot(*(e - c)) > max_dist:
            continue
        if _duplicate(m, c, roads):
            continue
        own = r.meta.setdefault("markings", [])
        own.append(dict(m, contact=contact))
        n += 1
    return n
