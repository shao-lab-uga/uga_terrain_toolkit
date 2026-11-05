"""Lanes from lane paint.

Road paint reflects the lidar far more strongly than asphalt. For each road
the ground points of every scan are put into a road-aligned image (along the
road x across it) of *relative* intensity: each point's intensity divided by
the typical intensity at the same distance in the same scan, which removes
the strong fall-off with range. In that image a lane line is a thin bright
stripe running along the road.

Stripes are found in 2 m blocks, linked along the road into lines, and each
line is fitted with a smooth curve. The lanes are then the strips between
neighbouring boundaries (road edge, lines, road edge). Where no paint is
found, the paved width is split into equal lanes of about ``lane_width`` with
smoothed edges, so lane widths never wobble from one metre to the next.

The result is stored per road (``Road.meta["lanes"]``) and used by the
OpenDRIVE writer: the reference line goes on the centre line between the two
directions, and each lane gets a smooth width.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from uga_terrain import geo
from uga_terrain.roads import Road, _gauss1d

DS = 0.2            # m, image rows along the road
DT = 0.05           # m, image columns across the road
BLOCK = 2.0         # m of road per stripe search
EDGE_MERGE = 0.5    # a line this close to the road edge is the edge line


def _running_median(v: np.ndarray, w: int) -> np.ndarray:
    if w < 2:
        return v.copy()
    pad = np.pad(v, (w // 2, w - 1 - w // 2), mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(pad, w)
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            return np.nanmedian(win, axis=1)


def intensity_image(road: Road, scans, half_width: float, max_range: float = 20.0):
    """Road-aligned relative-intensity image (rows along s, columns across t).

    ``scans`` yields (T_world_from_lidar 4x4, points Nx4 [x, y, z, intensity])."""
    ns = max(1, int(math.ceil(road.length / DS)) + 1)
    nt = int(round(2 * half_width / DT)) + 1
    acc = np.zeros((ns, nt))
    cnt = np.zeros((ns, nt))
    P = np.stack([road.x, road.y], 1)
    for T, g in scans:
        r = np.hypot(g[:, 0], g[:, 1])
        m = (r < max_range) & np.isfinite(g[:, 3])
        if m.sum() < 50:
            continue
        g, r = g[m], r[m]
        it = g[:, 3].astype(np.float64)
        b = np.floor(r).astype(int)
        order = np.argsort(b, kind="stable")
        bs = b[order]
        starts = np.flatnonzero(np.r_[True, np.diff(bs) > 0])
        ends = np.r_[starts[1:], len(bs)]
        for a, e in zip(starts, ends):
            sel = order[a:e]
            med = np.median(it[sel])
            it[sel] = it[sel] / med if med > 0 else np.nan
        w = geo.transform(T, g[:, :3].astype(np.float64))[:, :2]
        # nearest station: only stations near this scan can be close
        c = T[:2, 3]
        near = np.flatnonzero(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < max_range + half_width + 2)
        if not len(near):
            continue
        Q = P[near]
        j = np.empty(len(w), np.int64)
        for a in range(0, len(w), 4096):
            d2 = ((w[a:a + 4096, None, :] - Q[None, :, :]) ** 2).sum(-1)
            j[a:a + 4096] = near[np.argmin(d2, 1)]
        h = road.heading[j]
        rel = w - P[j]
        s = road.s[j] + rel[:, 0] * np.cos(h) + rel[:, 1] * np.sin(h)
        t = -rel[:, 0] * np.sin(h) + rel[:, 1] * np.cos(h)
        si = np.round((s - road.s[0]) / DS).astype(int)
        ti = np.round((t + half_width) / DT).astype(int)
        ok = (si >= 0) & (si < ns) & (ti >= 0) & (ti < nt) & np.isfinite(it)
        np.add.at(acc, (si[ok], ti[ok]), it[ok])
        np.add.at(cnt, (si[ok], ti[ok]), 1)
    with np.errstate(all="ignore"):
        img = np.where(cnt > 0, acc / np.maximum(cnt, 1), np.nan)
    return img


def find_stripes(img: np.ndarray, half_width: float, lo_t: np.ndarray, hi_t: np.ndarray,
                 min_contrast: float = 0.25):
    """Bright thin stripes per block: list of (s, t, contrast, width)."""
    rows = max(1, int(round(BLOCK / DS)))
    tt = np.arange(img.shape[1]) * DT - half_width
    bg_w = int(round(1.2 / DT)) | 1
    out = []
    import warnings
    for a in range(0, img.shape[0], rows):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            p = np.nanmean(img[a:a + rows], 0)
        s = (a + rows / 2) * DS
        k = min(int(s / 1.0), len(lo_t) - 1)          # road stations are 1 m apart
        inside = (tt > lo_t[k] + 0.05) & (tt < hi_t[k] - 0.05)
        if inside.sum() < 20 or np.isfinite(p[inside]).sum() < 0.5 * inside.sum():
            continue
        bg = _running_median(p, bg_w)
        r = p - bg
        rr = r[inside & np.isfinite(r)]
        noise = 1.4826 * np.median(np.abs(rr - np.median(rr))) if len(rr) else 1.0
        thr = max(min_contrast, 4.0 * noise)
        cand = np.flatnonzero(inside & np.isfinite(r) & (r > thr))
        if not len(cand):
            continue
        # group neighbouring columns into stripes
        groups = np.split(cand, np.flatnonzero(np.diff(cand) > 1) + 1)
        for gq in groups:
            width = (len(gq)) * DT
            if width < 0.05 or width > 0.45:
                continue
            wts = np.clip(r[gq], 0, None)
            tc = float((tt[gq] * wts).sum() / max(wts.sum(), 1e-9))
            out.append((s, tc, float(r[gq].max()), width))
    return out


def link_lines(stripes, road_len: float, max_dt: float = 0.35, max_gap: float = 15.0,
               min_span: float = 12.0):
    """Join stripes along the road into lines."""
    tracks: List[List[tuple]] = []
    for st in sorted(stripes):
        best, bd = None, max_dt
        for tr in tracks:
            ls, lt = tr[-1][0], tr[-1][1]
            if 0 < st[0] - ls <= max_gap and abs(st[1] - lt) < bd:
                best, bd = tr, abs(st[1] - lt)
        if best is None:
            tracks.append([st])
        else:
            best.append(st)
    lines = []
    for tr in tracks:
        a = np.array(tr)
        span = a[-1, 0] - a[0, 0]
        if len(a) < 3 or span < min(min_span, 0.4 * road_len):
            continue
        n_blocks = span / BLOCK + 1
        lines.append({"s": a[:, 0], "t": a[:, 1], "contrast": float(np.median(a[:, 2])),
                      "span": [float(a[0, 0]), float(a[-1, 0])],
                      "kind": "solid" if len(a) / n_blocks > 0.6 else "broken"})
    return lines


def _fit(s_obs: np.ndarray, t_obs: np.ndarray, s: np.ndarray, bw: float = 8.0) -> np.ndarray:
    """Smooth curve through a line's detections (kernel regression with a local
    slope), held flat past its ends."""
    out = np.empty(len(s))
    for i, si in enumerate(np.clip(s, s_obs.min(), s_obs.max())):
        w = np.exp(-0.5 * ((s_obs - si) / bw) ** 2)
        if w.sum() < 1e-6:
            w = np.ones_like(s_obs)
        A = np.stack([np.ones_like(s_obs), s_obs - si], 1) * np.sqrt(w)[:, None]
        sol, *_ = np.linalg.lstsq(A, t_obs * np.sqrt(w), rcond=None)
        out[i] = sol[0]
    return out


def _smooth_edge(v: np.ndarray, fallback: float, step: float) -> np.ndarray:
    v = np.where(np.isfinite(v), v, np.nan)
    if not np.isfinite(v).any():
        return np.full(len(v), fallback)
    v = _running_median(v, max(3, int(15.0 / step)) | 1)
    idx = np.arange(len(v))
    ok = np.isfinite(v)
    v = np.interp(idx, idx[ok], v[ok])
    return _gauss1d(v, 5.0 / step)


def layout(road: Road, lines: List[dict], cfg: dict) -> dict:
    """Lane boundaries (t at each station, left positive) and lane types."""
    rc = cfg.get("roads") or {}
    lane_w = float(rc.get("lane_width") or 3.5)
    step = float(rc.get("station_step") or 1.0)
    s = road.s - road.s[0]
    e_l = _smooth_edge(road.data.get("road_l", np.full(len(s), np.nan)), 1.75, step)
    e_r = -_smooth_edge(road.data.get("road_r", np.full(len(s), np.nan)), 1.75, step)
    fitted = []
    for ln in lines:
        t = _fit(ln["s"], ln["t"], s)
        if np.median(np.abs(t - e_r)) < EDGE_MERGE or np.median(np.abs(t - e_l)) < EDGE_MERGE:
            continue                       # an edge line: the edge already bounds the lane
        if np.any(t <= e_r + 0.3) or np.any(t >= e_l - 0.3):
            t = np.clip(t, e_r + 0.3, e_l - 0.3)
        fitted.append((float(np.median(t)), t, ln))
    fitted.sort(key=lambda x: x[0])
    # drop lines closer than 2 m to a neighbour (double lines count once)
    keep = []
    for f in fitted:
        if keep and f[0] - keep[-1][0] < 2.0:
            if f[2]["contrast"] > keep[-1][2]["contrast"]:
                keep[-1] = f
            continue
        keep.append(f)
    total = float(np.median(e_l - e_r))
    if keep:
        # the centre line divides the directions: the painted line nearest the
        # middle of the paved width
        mid = 0.5 * (e_l + e_r)
        c = int(np.argmin([abs(np.median(t - mid)) for _, t, _ in keep]))
        bounds = [e_r] + [t for _, t, _ in keep] + [e_l]
        centre = c + 1
        marks = ["edge"] + [ln["kind"] for _, _, ln in keep] + ["edge"]
        source = "paint"
    else:
        # no paint: two directions, equal lanes of about lane_width, steady widths
        n = max(1, int(round(total / 2 / lane_w)))
        mid = _gauss1d(0.5 * (e_l + e_r), 10.0 / step)
        half_l = e_l - mid
        half_r = mid - e_r
        bounds = [e_r + half_r * k / n for k in range(n)] + [mid] + [mid + half_l * k / n for k in range(1, n + 1)]
        centre = n
        marks = ["edge"] + ["broken"] * (n - 1) + ["solid"] + ["broken"] * (n - 1) + ["edge"]
        source = "width"
    # lane types: wide strips are driving lanes, narrow ones shoulders
    types = []
    for k in range(len(bounds) - 1):
        w = float(np.median(bounds[k + 1] - bounds[k]))
        types.append("driving" if w >= 2.4 else "shoulder")
    # outer driving lanes: the curb is not always parallel to the lanes (turn
    # pockets, side streets, parking). Keep the lane at a steady width - its
    # width at the narrow end - and give the extra to a shoulder strip, so a
    # lane never widens just because the curb moves away
    def steady(w):
        return np.minimum(w, float(np.percentile(w, 20)))
    if types[0] == "driving":
        w = bounds[1] - bounds[0]
        ws = steady(w)
        if (w - ws).max() > 0.15:
            bounds.insert(1, bounds[1] - ws)
            types.insert(0, "shoulder")
            marks.insert(1, "none")
            centre += 1
    if types[-1] == "driving":
        w = bounds[-1] - bounds[-2]
        ws = steady(w)
        if (w - ws).max() > 0.15:
            bounds.insert(len(bounds) - 1, bounds[-2] + ws)
            types.append("shoulder")
            marks.insert(len(marks) - 1, "none")
    # steady widths: an inner driving lane (one not touching the road edge)
    # whose width barely changes gets one constant width, built outward from
    # the centre line so lanes stay parallel to it
    nb = len(bounds)
    for k in range(centre - 1, 0, -1):                 # right side, outward
        w = bounds[k + 1] - bounds[k]
        if types[k] == "driving" and np.std(w) < 0.15:
            bounds[k] = bounds[k + 1] - float(np.median(w))
    for k in range(centre, nb - 2):                    # left side, outward
        w = bounds[k + 1] - bounds[k]
        if types[k] == "driving" and np.std(w) < 0.15:
            bounds[k + 1] = bounds[k] + float(np.median(w))
    return {"source": source, "centre": centre, "types": types, "marks": marks,
            "bounds": [np.asarray(b, float) for b in bounds],
            "lines": [{"t_median": round(m, 3), "kind": ln["kind"], "span": ln["span"],
                       "contrast": round(ln["contrast"], 3)} for m, _, ln in keep]}


def detect(road: Road, scans_fn, cfg: dict, blank=()) -> dict:
    """Find painted lines on one road and lay out its lanes. ``blank``: (s0, s1)
    stretches to ignore (the inside of a junction: crosswalks, stop bars)."""
    rc = cfg.get("roads") or {}
    lc = cfg.get("lanes") or {}
    hw = float(np.nanmax(np.r_[road.data.get("road_l", [3.0]), road.data.get("road_r", [3.0]), 3.0])) + 1.0
    hw = min(hw, float(rc.get("half_width") or 25.0))
    img = intensity_image(road, scans_fn(), hw, float(lc.get("paint_range", 20.0)))
    for s0, s1 in blank:
        img[max(0, int((s0 - road.s[0]) / DS)):max(0, int(math.ceil((s1 - road.s[0]) / DS)) + 1)] = np.nan
    step = float(rc.get("station_step") or 1.0)
    e_l = _smooth_edge(road.data.get("road_l", np.full(len(road.s), np.nan)), 1.75, step)
    e_r = -_smooth_edge(road.data.get("road_r", np.full(len(road.s), np.nan)), 1.75, step)
    stripes = find_stripes(img, hw, e_r, e_l, float(lc.get("min_contrast", 0.25)))
    lines = link_lines(stripes, road.length)
    lay = layout(road, lines, cfg)
    lay["stripes"] = len(stripes)
    return lay
