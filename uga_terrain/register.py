"""Line up several recordings of the same place with each other.

Every recording is placed on the map by its own GNSS/INS track, and two
tracks of the same street can disagree by tens of centimetres sideways, a
degree in heading and 10-30 cm in height. Merged as they are, that gives
doubled lane paint, curbs smeared into a ramp and a rough, stepped road
surface, which is useless for lane-level maps.

Each recording after the first is matched to the ones before it on the
ground they both saw:

* sideways and heading: the lane paint, crosswalks and other markings. Both
  are drawn as images of *relative* lidar intensity (as in lanes.py), and the
  shift and rotation with the best normalised cross-correlation wins;
* height and tilt: a plane fitted to the height differences of the shared
  ground (robust to cars and other things seen by only one recording).

The correction is one rigid transform per recording, applied to all of its
frame poses. The result goes in ``align.json`` in the map folder and is used
wherever frame poses are read (see ``poses_of``).
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from uga_terrain import geo

CELL = 0.1             # m, intensity image pixel
ZCELL = 0.5            # m, height comparison cell
MIN_SCORE = 0.3        # weaker matches than this are not trusted
MIN_OVERLAP_M2 = 60.0


def _points(frames_dir: Path, T: np.ndarray, max_range: float, every: int = 1) -> np.ndarray:
    """World x, y, z and relative intensity of every ground point within
    ``max_range`` of the lidar."""
    out = []
    for k in range(0, len(T), every):
        f = frames_dir / ("%06d.npy" % k)
        if not f.is_file():
            continue
        g = np.load(f).astype(np.float64)
        if g.shape[1] < 4 or not len(g):
            continue
        r = np.hypot(g[:, 0], g[:, 1])
        m = (r < max_range) & np.isfinite(g[:, 3])
        g, r = g[m], r[m]
        if len(g) < 50:
            continue
        it = g[:, 3].copy()
        b = np.floor(r).astype(int)
        order = np.argsort(b, kind="stable")
        bs = b[order]
        starts = np.flatnonzero(np.r_[True, np.diff(bs) > 0])
        for a, e in zip(starts, np.r_[starts[1:], len(bs)]):
            sel = order[a:e]
            med = np.median(it[sel])
            it[sel] = it[sel] / med if med > 0 else np.nan
        w = geo.transform(T[k], g[:, :3])
        out.append(np.c_[w, it])
    return np.vstack(out) if out else np.zeros((0, 4))


def _raster(P: np.ndarray, lo: np.ndarray, shape) -> tuple:
    ij = np.floor((P[:, :2] - lo) / CELL).astype(int)
    ok = ((ij[:, 0] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[1])
          & np.isfinite(P[:, 3]))
    acc = np.zeros(shape)
    cnt = np.zeros(shape)
    np.add.at(acc, (ij[ok, 0], ij[ok, 1]), np.clip(P[ok, 3], 0, 4))
    np.add.at(cnt, (ij[ok, 0], ij[ok, 1]), 1)
    return np.where(cnt > 0, acc / np.maximum(cnt, 1), 0.0), cnt > 0


def _ncc(ia, ma, ib, mb, reach: int, min_px: int):
    """Best shift (pixels) of image b onto image a by masked normalised
    cross-correlation, within +-reach pixels. Returns (score, di, dj)."""
    fa = np.where(ma, ia - ia[ma].mean(), 0.0)
    fb = np.where(mb, ib - ib[mb].mean(), 0.0)
    s = [2 * n for n in ia.shape]
    F, I = np.fft.rfft2, np.fft.irfft2
    Ma, Mb = F(ma.astype(float), s), F(mb.astype(float), s)
    Fa, Fb = F(fa, s), F(fb, s)
    num = I(Fa * np.conj(Fb), s)
    ov = I(Ma * np.conj(Mb), s)
    ea = I(F(fa ** 2, s) * np.conj(Mb), s)
    eb = I(Ma * np.conj(F(fb ** 2, s)), s)
    d = np.arange(-reach, reach + 1)
    ii, jj = np.meshgrid(d, d, indexing="ij")
    n = ov[ii, jj]
    with np.errstate(all="ignore"):
        v = num[ii, jj] / np.sqrt(np.maximum(ea[ii, jj] * eb[ii, jj], 1e-12))
    v = np.where(n >= min_px, v, -np.inf)
    k = np.unravel_index(int(np.argmax(v)), v.shape)
    best = float(v[k])
    di, dj = float(ii[k]), float(jj[k])
    # sub-pixel: parabola through the neighbours
    for axis in (0, 1):
        a = list(k)
        b = list(k)
        a[axis] -= 1
        b[axis] += 1
        if min(a) < 0 or max(b) >= v.shape[0]:
            continue
        y0, y1, y2 = v[tuple(a)], best, v[tuple(b)]
        den = y0 - 2 * y1 + y2
        if np.isfinite(den) and den < 0:
            off = 0.5 * (y0 - y2) / den
            if axis == 0:
                di += float(np.clip(off, -0.5, 0.5))
            else:
                dj += float(np.clip(off, -0.5, 0.5))
    return best, di, dj


def _rot2(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def match_xy(A: np.ndarray, B: np.ndarray, max_shift: float = 2.5, max_yaw: float = 2.0) -> Optional[dict]:
    """Rotation (about the overlap centre) and shift that put B's markings on A's."""
    if len(A) < 1000 or len(B) < 1000:
        return None
    lo = np.minimum(A[:, :2].min(0), B[:, :2].min(0))
    ca = {tuple(k) for k in np.floor((A[:, :2] - lo) / 2.0).astype(int)}
    cb = {tuple(k) for k in np.floor((B[:, :2] - lo) / 2.0).astype(int)}
    com = np.array(sorted(ca & cb))
    if len(com) * 4.0 < MIN_OVERLAP_M2:
        return None
    clo = lo + com.min(0) * 2.0 - max_shift - 1
    chi = lo + (com.max(0) + 1) * 2.0 + max_shift + 1
    shape = tuple(int(v) for v in np.ceil((chi - clo) / CELL))
    c0 = 0.5 * (clo + chi)
    ia, ma = _raster(A, clo, shape)
    reach = int(math.ceil(max_shift / CELL))
    min_px = int(0.5 * MIN_OVERLAP_M2 / CELL ** 2)

    def score(yaw):
        Bp = B.copy()
        Bp[:, :2] = (B[:, :2] - c0) @ _rot2(yaw).T + c0
        ib, mb = _raster(Bp, clo, shape)
        return _ncc(ia, ma, ib, mb, reach, min_px)

    cache = {}

    def at(yaw):
        key = round(yaw, 4)
        if key not in cache:
            cache[key] = score(yaw)
        return cache[key]
    coarse = np.arange(-max_yaw, max_yaw + 1e-9, 0.25)
    y_best = max(coarse, key=lambda y: at(y)[0])
    for step in (0.1, 0.04):
        cand = [y_best + k * step for k in (-2, -1, 0, 1, 2)]
        y_best = max(cand, key=lambda y: at(y)[0])
    v, di, dj = at(y_best)
    v0 = at(0.0)[0]
    return {"score": v, "score_before": v0, "yaw_deg": float(y_best),
            "dx": di * CELL, "dy": dj * CELL, "pivot": [float(c0[0]), float(c0[1])]}


def match_z(A: np.ndarray, B: np.ndarray) -> Optional[dict]:
    """Plane a + b (x - x0) + c (y - y0) fitted to height(A) - height(B)."""
    lo = np.minimum(A[:, :2].min(0), B[:, :2].min(0))

    def cells(P):
        k = np.floor((P[:, :2] - lo) / ZCELL).astype(np.int64)
        key = k[:, 0] * 1_000_003 + k[:, 1]
        order = np.argsort(key, kind="stable")
        ks = key[order]
        st = np.flatnonzero(np.r_[True, np.diff(ks) > 0])
        out = {}
        for a, e in zip(st, np.r_[st[1:], len(ks)]):
            if e - a >= 5:
                sel = order[a:e]
                out[int(ks[a])] = (float(np.median(P[sel, 2])), P[sel[0], 0], P[sel[0], 1])
        return out
    ca, cb = cells(A), cells(B)
    com = [k for k in ca if k in cb]
    if len(com) * ZCELL ** 2 < MIN_OVERLAP_M2:
        return None
    d = np.array([ca[k][0] - cb[k][0] for k in com])
    xy = np.array([[ca[k][1], ca[k][2]] for k in com])
    x0 = xy.mean(0)
    M = np.c_[np.ones(len(d)), xy - x0]
    w = np.ones(len(d))
    sol = np.zeros(3)
    for _ in range(8):
        sw = np.sqrt(w)
        sol, *_ = np.linalg.lstsq(M * sw[:, None], d * sw, rcond=None)
        r = d - M @ sol
        mad = 1.4826 * np.median(np.abs(r)) + 1e-3
        w = 1.0 / np.maximum(1.0, np.abs(r) / (2.5 * mad))
    r = d - M @ sol
    return {"dz": float(sol[0]), "slope_x": float(sol[1]), "slope_y": float(sol[2]),
            "pivot": [float(x0[0]), float(x0[1])], "cells": len(com),
            "before_m": float(np.median(np.abs(d))), "after_m": float(np.median(np.abs(r)))}


def correction(xy: Optional[dict], z: Optional[dict]) -> np.ndarray:
    """4x4 world-frame transform from a match_xy / match_z result."""
    C = np.eye(4)
    if xy:
        R = _rot2(xy["yaw_deg"])
        p = np.array(xy["pivot"])
        C[:2, :2] = R
        C[:2, 3] = p - R @ p + np.array([xy["dx"], xy["dy"]])
    if z:
        Z = np.eye(4)
        bx, by = z["slope_x"], z["slope_y"]
        # small tilt: z += bx (x - x0) + by (y - y0) + dz, as a rotation
        Rt = geo.rotvec_to_mat(np.array([[by, -bx, 0.0]]))[0]
        p = np.array([z["pivot"][0], z["pivot"][1], 0.0])
        Z[:3, :3] = Rt
        Z[:3, 3] = p - Rt @ p + np.array([0.0, 0.0, z["dz"]])
        C = Z @ C
    return C


# --- per map ------------------------------------------------------------------------

def _stamp(fz: Path) -> str:
    st = fz.stat()
    return "%d-%d" % (st.st_size, int(st.st_mtime))


def align_project(project, cfg: dict, progress=None) -> Dict[str, dict]:
    """Match every recording of the map to the ones before it; saves align.json."""
    rc = cfg.get("register") or {}
    out: Dict[str, dict] = {}
    n = len(project.recordings)
    pts_done: List[np.ndarray] = []
    rng = float(rc.get("max_range") or 20.0)
    for i in range(n):
        fz = project.cache(i) / "frames.npz"
        if not fz.is_file():
            continue
        T = np.load(fz)["T"]
        if progress:
            progress.set("line up drives", i, n, "recording %d of %d" % (i + 1, n))
            progress.check()
        P = _points(project.cache(i) / "frames", T, rng, every=int(rc.get("frame_step") or 1))
        rec = {"stamp": _stamp(fz), "C": np.eye(4).tolist(), "applied": False}
        if pts_done and rc.get("enabled", True):
            A = np.vstack(pts_done)
            xy = match_xy(A, P, float(rc.get("max_shift") or 2.5), float(rc.get("max_yaw_deg") or 2.0))
            if xy and xy["score"] >= MIN_SCORE:
                C = correction(xy, None)
                P2 = P.copy()
                P2[:, :3] = geo.transform(C, P[:, :3])
                z = match_z(A, P2)
                C = correction(xy, z)
                P = P2
                if z:
                    P[:, :3] = geo.transform(correction(None, z), P[:, :3])
                rec.update({"C": C.tolist(), "applied": True, "xy": xy, "z": z})
            else:
                rec["why_not"] = ("too little shared ground with the earlier recordings" if xy is None
                                  else "markings did not match well enough (score %.2f)" % xy["score"])
                rec["xy"] = xy
        out[str(i)] = rec
        pts_done.append(P[:: max(1, len(P) // 1_500_000)] if len(P) > 1_500_000 else P)
    (project.root / "align.json").write_text(json.dumps(out, indent=1))
    return out


def load(project) -> Dict[str, dict]:
    f = project.root / "align.json"
    if not f.is_file():
        return {}
    try:
        return json.loads(f.read_text())
    except ValueError:
        return {}


def poses_of(project, i: int, align: Optional[Dict[str, dict]] = None) -> Optional[np.ndarray]:
    """Frame poses (N,4,4) of recording i, lined up with the other recordings
    when align.json is current for that recording's scan."""
    fz = project.cache(i) / "frames.npz"
    if not fz.is_file():
        return None
    T = np.load(fz)["T"]
    a = (align if align is not None else load(project)).get(str(i))
    if a and a.get("applied") and a.get("stamp") == _stamp(fz):
        T = np.asarray(a["C"], float)[None] @ T
    return T


def summary_text(name: str, rec: dict) -> str:
    if not rec.get("applied"):
        return "%s: not moved%s" % (name, (" (%s)" % rec["why_not"]) if rec.get("why_not") else "")
    xy, z = rec.get("xy") or {}, rec.get("z") or {}
    s = "%s: moved %.2f m sideways, turned %.2f deg" % (
        name, math.hypot(xy.get("dx", 0), xy.get("dy", 0)), xy.get("yaw_deg", 0))
    if z:
        s += ", raised %+.2f m (height difference %.2f -> %.2f m)" % (z["dz"], z["before_m"], z["after_m"])
    s += "; marking match %.2f -> %.2f" % (xy.get("score_before", 0), xy.get("score", 0))
    return s
