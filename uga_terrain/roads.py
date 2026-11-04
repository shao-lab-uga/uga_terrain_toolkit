"""Roads and sidewalks from cross-sections of the elevation map.

Every ``station_step`` metres along each drive, the elevation map is sampled
on a line across the road. Walking outward from the drive, the road ends at
the first curb (a height step), rough surface (grass, gravel), steep slope or
missing data. Past a curb or road edge, the first smooth, nearly level strip
at least ``sidewalk_min_width`` wide is the sidewalk; anything between is the
verge. The result per station is what OpenDRIVE needs: road widths each side
of the reference line, curb heights, verge and sidewalk widths, elevation,
grade and banking.

Repeated passes over the same street (another recording, or a loop) feed the
elevation map but do not create a second road.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from uga_terrain.grid import TerrainGrid

EDGE_NAMES = ("limit", "curb", "drop", "rough", "slope", "gap")


@dataclass
class Road:
    id: int
    source: str
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    data: Dict[str, np.ndarray] = field(default_factory=dict)

    @property
    def length(self) -> float:
        return float(self.s[-1] - self.s[0]) if len(self.s) > 1 else 0.0

    def rows(self) -> List[dict]:
        keys = ["s", "x", "y", "heading"] + list(self.data)
        cols = [self.s, self.x, self.y, self.heading] + [self.data[k] for k in self.data]
        return [{k: (None if not np.isfinite(v[i]) else float(v[i])) for k, v in zip(keys, cols)}
                for i in range(len(self.s))]


# --- drive paths -------------------------------------------------------------------

def _gauss1d(v: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0 or len(v) < 3:
        return v.copy()
    r = int(math.ceil(3 * sigma))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    ok = np.isfinite(v).astype(float)
    vv = np.where(ok > 0, v, 0.0)
    if ok.all() and len(v) > r:
        # odd reflection keeps straight-line trends straight up to the ends
        pv = np.concatenate([2 * v[0] - v[r:0:-1], v, 2 * v[-1] - v[-2:-r - 2:-1]])
        return np.convolve(pv, k, "valid") / k.sum()
    num = np.convolve(np.pad(vv, r, mode="edge"), k, "valid")
    den = np.convolve(np.pad(ok, r, mode="edge"), k, "valid")
    return np.where(den > 1e-9, num / np.maximum(den, 1e-9), np.nan)


def resample_path(xy: np.ndarray, step: float, smooth_m: float = 3.0):
    """Even stations along a driven path: s, x, y, heading (stationary parts drop out)."""
    xy = np.asarray(xy, np.float64)
    if len(xy) < 2:
        return None
    d = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] < 2 * step:
        return None
    keep = np.concatenate([[True], d > 1e-3])
    s, xy = s[keep], xy[keep]
    ss = np.arange(0.0, s[-1], step)
    x = np.interp(ss, s, xy[:, 0])
    y = np.interp(ss, s, xy[:, 1])
    sig = smooth_m / step
    x, y = _gauss1d(x, sig), _gauss1d(y, sig)
    h = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    # re-measure arc length after smoothing
    s2 = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return s2, x, y, h


class _Hash:
    def __init__(self, cell: float):
        self.cell = cell
        self.buckets: Dict[tuple, List[np.ndarray]] = {}

    def add(self, xy: np.ndarray) -> None:
        keys = np.floor(xy / self.cell).astype(np.int64)
        for k in np.unique(keys, axis=0):
            sel = np.all(keys == k, axis=1)
            self.buckets.setdefault((int(k[0]), int(k[1])), []).append(xy[sel])

    def near(self, xy: np.ndarray, dist: float) -> np.ndarray:
        out = np.zeros(len(xy), bool)
        keys = np.floor(xy / self.cell).astype(np.int64)
        for i, (k, p) in enumerate(zip(keys, xy)):
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for arr in self.buckets.get((int(k[0]) + dx, int(k[1]) + dy), ()):
                        if np.any(np.hypot(arr[:, 0] - p[0], arr[:, 1] - p[1]) < dist):
                            out[i] = True
                            break
                    if out[i]:
                        break
                if out[i]:
                    break
        return out


def split_roads(paths: Sequence[tuple], cfg: dict) -> List[Road]:
    """paths: [(name, xy (N,2))] in drive order -> non-overlapping road pieces."""
    rc = cfg.get("roads") or {}
    step = float(rc.get("station_step") or 1.0)
    dedup = float(rc.get("dedup_dist") or 3.0)
    min_len = float(rc.get("min_road_length") or 15.0)
    hsh = _Hash(max(dedup, 5.0))
    roads: List[Road] = []
    for name, xy in paths:
        rs = resample_path(xy, step)
        if rs is None:
            continue
        s, x, y, h = rs
        pts = np.stack([x, y], 1)
        dup = hsh.near(pts, dedup)
        # the same drive coming back over itself (a loop) counts as a repeat too
        own = np.zeros(len(pts), bool)
        gap = int(30.0 / step)
        for i in range(gap, len(pts)):
            prev = pts[: i - gap]
            if len(prev) and np.min(np.hypot(prev[:, 0] - pts[i, 0], prev[:, 1] - pts[i, 1])) < dedup:
                own[i] = True
        dup |= own
        runs = np.flatnonzero(np.diff(np.concatenate([[1], dup.astype(int), [1]])))
        for a, b in zip(runs[::2], runs[1::2]):
            if b - a < 2 or (s[b - 1] - s[a]) < min_len:
                continue
            sl = slice(a, b)
            roads.append(Road(len(roads) + 1, name, s[sl] - s[a], x[sl], y[sl], h[sl]))
        hsh.add(pts)
    return roads


# --- cross-sections ------------------------------------------------------------------

def _first_run(mask: np.ndarray, start: int, min_len: int) -> Optional[int]:
    """Index where the first run of >= min_len True values starts, from ``start``."""
    n = 0
    for j in range(start, len(mask)):
        n = n + 1 if mask[j] else 0
        if n >= min_len:
            return j - min_len + 1
    return None


def _side(z, rough, slope, t, rc) -> dict:
    """Walk one side outward (t >= 0 increasing). Returns road edge and sidewalk."""
    dt = t[1] - t[0]
    n = len(t)
    curb_min, curb_max = float(rc["curb_min"]), float(rc["curb_max"])
    rough_max = float(rc["rough_road_max"])
    min_j = int(round(float(rc["min_half_width"]) / dt))
    gap_j = max(2, int(round(1.0 / dt)))
    look = max(1, int(round(0.6 / dt)))
    # detrend with the road's own cross slope near the drive
    near = np.isfinite(z[: max(min_j, 3)])
    slope_c = 0.0
    if near.sum() >= 3:
        slope_c = float(np.polyfit(t[: max(min_j, 3)][near], z[: max(min_j, 3)][near], 1)[0])
    zd = z - slope_c * t
    edge, kind, curb_h, j_after = n - 1, "limit", 0.0, None
    edge_z = None
    miss = 0
    # a rough or steep patch right at the road's edge (gutter, drain grate,
    # worn asphalt) often sits just in front of the curb: remember it and keep
    # looking ~1 m further for a curb step before settling for it
    pending = None
    ahead = max(1, int(round(1.0 / dt)))
    for j in range(1, n):
        if pending is not None and j > pending[0] + ahead:
            break
        if not np.isfinite(z[j]):
            miss += 1
            if miss >= gap_j and j - miss + 1 >= min_j:
                if pending is None:
                    edge, kind = j - miss + 1, "gap"
                break
            continue
        miss = 0
        if j < min_j:
            continue
        k = min(n - 1, j + look)
        seg = zd[j:k + 1]
        if np.isfinite(seg).sum() >= 2:
            fin = seg[np.isfinite(seg)]
            dz = float(fin[-1] - fin[0])
            if curb_min <= abs(dz) <= curb_max:
                # the step sits where the height changes fastest
                lo = max(j - 1, 0)
                dd = np.abs(np.diff(zd[lo:k + 1]))
                e = lo + int(np.nanargmax(np.where(np.isfinite(dd), dd, -1))) + 1
                q = max(1, int(round(0.4 / dt)))
                r_ = max(q + 2, int(round(1.0 / dt)))
                te = t[e] - dt / 2
                h = dz
                ia = np.arange(e + q, min(n, e + r_ + 1))
                ib = np.arange(max(0, e - r_), max(0, e - q + 1))
                ia = ia[np.isfinite(z[ia])]
                ib = ib[np.isfinite(z[ib])]
                if len(ia) >= 2 and len(ib) >= 2:
                    # extend the surface on each side to the step and compare there
                    # outer side: its typical height (verges and sidewalks are near level)
                    za = float(np.median(z[ia]))
                    zb = np.polyval(np.polyfit(t[ib], z[ib], 1), te)
                    h = float(za - zb)
                    edge_z = float(zb)
                edge, kind, curb_h = e, ("curb" if dz > 0 else "drop"), h
                j_after = min(n - 1, e + q)
                pending = None
                break
        if pending is not None:
            continue
        r_win = rough[j:min(n, j + 3)]
        if len(r_win) == 3 and np.all(r_win > rough_max):
            pending = (j, "rough")
        elif np.isfinite(slope[j]) and slope[j] > 15.0:
            pending = (j, "slope")
    if pending is not None:
        edge, kind = pending
        j_after = edge
    out = {"road_w": float(max(t[edge] - dt / 2, 0.0)), "edge": kind, "curb_h": curb_h,
           "verge_w": 0.0, "walk_w": 0.0, "walk_h": np.nan}
    if j_after is None or kind == "drop":
        return out
    # sidewalk: first smooth, level strip past the edge
    walk_rough = float(rc["rough_walk_max"])
    xs = float(rc["walk_max_cross_slope"])
    min_w = int(round(float(rc["sidewalk_min_width"]) / dt))
    stop = min(n, j_after + int(round(float(rc["sidewalk_search"]) / dt)))
    grad = np.abs(np.gradient(z)) / dt if n > 2 else np.zeros(n)
    grad = np.where(np.isfinite(grad), grad, np.inf)
    good = np.isfinite(z) & (rough <= walk_rough) & (grad <= xs + 0.02)
    good[:j_after + 1] = False
    good[stop:] = False
    j0 = _first_run(good, j_after + 1, min_w)
    if j0 is None:
        return out
    j1 = j0
    max_w = int(round(4.0 / dt))
    while j1 + 1 < n and good[j1 + 1] and j1 - j0 < max_w:
        j1 += 1
    if edge_z is None:
        edge_z = z[edge - 1] if np.isfinite(z[edge - 1]) else np.nanmedian(z[:edge])
    verge = float(t[j0] - dt / 2 - out["road_w"])
    if verge < 0.8:   # a sidewalk right behind the curb; the gap is the curb's own slope
        verge = 0.0
    walk = float(t[j1] + dt / 2 - out["road_w"] - verge)
    out.update({"verge_w": verge, "walk_w": walk,
                "walk_h": float(np.nanmedian(z[j0:j1 + 1]) - edge_z)})
    return out


def _running_median(v: np.ndarray, w: int) -> np.ndarray:
    if w <= 1:
        return v
    r = w // 2
    p = np.pad(v, r, mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(p, 2 * r + 1)
    with np.errstate(all="ignore"):
        return np.nanmedian(win, axis=1)


def analyse_road(road: Road, grid: TerrainGrid, cfg: dict,
                 vehicle_rpy=None) -> Road:
    """Fill ``road.data`` with widths, curbs, sidewalks, elevation, grade, banking."""
    rc = cfg.get("roads") or {}
    step = float(rc.get("station_step") or 1.0)
    dt = float(rc.get("sample_step") or 0.2)
    W = float(rc.get("half_width") or 25.0)
    t = np.arange(0.0, W + dt / 2, dt)
    tt = np.concatenate([-t[:0:-1], t])          # -W .. W, left is +
    nx, ny = -np.sin(road.heading), np.cos(road.heading)   # left normal
    X = road.x[:, None] + tt[None, :] * nx[:, None]
    Y = road.y[:, None] + tt[None, :] * ny[:, None]
    Z = grid.sample(X, Y, "z")
    R = grid.sample(X, Y, "roughness")
    S = grid.sample(X, Y, "slope_deg")
    c = len(t) - 1
    M = len(road.s)
    res = {k: np.full(M, np.nan) for k in (
        "road_l", "road_r", "curb_h_l", "curb_h_r", "verge_l", "verge_r",
        "walk_l", "walk_r", "walk_h_l", "walk_h_r", "edge_l", "edge_r")}
    for i in range(M):
        for side, sl in (("l", slice(c, None)), ("r", slice(c, None, -1))):
            o = _side(Z[i, sl], R[i, sl], S[i, sl], t, rc)
            res["road_" + side][i] = o["road_w"]
            res["curb_h_" + side][i] = o["curb_h"] if o["edge"] in ("curb", "drop") else 0.0
            res["verge_" + side][i] = o["verge_w"]
            res["walk_" + side][i] = o["walk_w"]
            res["walk_h_" + side][i] = o["walk_h"]
            res["edge_" + side][i] = EDGE_NAMES.index(o["edge"])
    w = max(1, int(round(float(rc.get("smooth_m") or 5.0) / step)))
    for k in ("road_l", "road_r", "curb_h_l", "curb_h_r", "verge_l", "verge_r",
              "walk_l", "walk_r", "walk_h_l", "walk_h_r"):
        res[k] = _running_median(res[k], 2 * (w // 2) + 1)

    # elevation and cross slope from a line fit over the paved part
    elev = np.full(M, np.nan)
    bank = np.full(M, np.nan)
    xl = np.full(M, np.nan)
    xr = np.full(M, np.nan)
    rough = np.full(M, np.nan)
    for i in range(M):
        wl = max(res["road_l"][i] - 0.4, 0.6)
        wr = max(res["road_r"][i] - 0.4, 0.6)
        m = (tt <= wl) & (tt >= -wr) & np.isfinite(Z[i])
        if m.sum() >= 5:
            b, a = np.polyfit(tt[m], Z[i, m], 1)
            elev[i], bank[i] = a, math.degrees(math.atan(b))
            rough[i] = float(np.nanmean(R[i, m]))
        ml = m & (tt >= 0)
        mr = m & (tt <= 0)
        if ml.sum() >= 4:
            xl[i] = 100.0 * np.polyfit(tt[ml], Z[i, ml], 1)[0]
        if mr.sum() >= 4:
            xr[i] = -100.0 * np.polyfit(tt[mr], Z[i, mr], 1)[0]
    elev_s = _gauss1d(elev, 2.0 / step)
    grade = 100.0 * np.gradient(elev_s, road.s) if M > 2 else np.zeros(M)
    curv = np.gradient(_gauss1d(road.heading, 3.0 / step), road.s) if M > 2 else np.zeros(M)
    res.update({"z": elev_s, "z_raw": elev, "grade_pct": grade,
                "bank_deg": _gauss1d(bank, 2.0 / step),
                "cross_slope_left_pct": xl, "cross_slope_right_pct": xr,
                "curvature": curv, "roughness": rough})
    if vehicle_rpy is not None:
        roll, pitch = vehicle_rpy
        res["vehicle_bank_deg"] = np.degrees(roll)
        res["vehicle_grade_pct"] = -100.0 * np.tan(pitch)   # FLU pitch > 0 is nose down
    road.data = res
    return road


def paint_labels(road: Road, grid: TerrainGrid, sub: int = 5) -> None:
    """Mark road / curb / sidewalk cells on the grid (road wins over sidewalk)."""
    d = road.data
    if len(road.s) < 2:
        return
    ss = np.linspace(road.s[0], road.s[-1], (len(road.s) - 1) * sub + 1)

    def I(v):
        return np.interp(ss, road.s, np.nan_to_num(v))
    x, y = I(road.x), I(road.y)
    h = np.interp(ss, road.s, road.heading)
    nx, ny = -np.sin(h), np.cos(h)
    step = grid.cell * 0.7
    prio = np.array([0, 4, 3, 2, 1], np.float32)   # by label: none, road, sidewalk, curb, other

    def put(lo, hi, label, sign):
        width = np.maximum(hi - lo, 0.0)
        kmax = int(np.ceil(np.nanmax(width) / step)) + 1 if np.any(width > 0) else 0
        for k in range(kmax):
            off = lo + np.minimum(k * step, width)
            ok = width > 0
            px = x[ok] + sign * off[ok] * nx[ok]
            py = y[ok] + sign * off[ok] * ny[ok]
            grid.set_cells(px, py, "label", np.float32(label),
                           only_if=lambda old, new: np.isfinite(old) & (old > 0)
                           & (prio[np.nan_to_num(new).astype(int).clip(0, 4)]
                              > prio[np.nan_to_num(old).astype(int).clip(0, 4)]))

    for side, sign in (("l", 1.0), ("r", -1.0)):
        rw = I(d["road_" + side])
        put(np.zeros_like(rw), rw, 1, sign)
        curb = np.abs(I(d["curb_h_" + side])) > 0
        put(rw, rw + np.where(curb, 0.4, 0.0), 3, sign)
        ww = I(d["walk_" + side])
        start = rw + I(d["verge_" + side])
        put(start, start + ww, 2, sign)
