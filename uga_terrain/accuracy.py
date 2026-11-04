"""Accuracy checks for a built map, using only the recordings themselves.

There is no surveyed ground truth, so every check compares the data with
itself, from independent angles:

* poses      - how closely lidar odometry and GNSS/INS agree, per recording
* by range   - each scan's ground height against the height seen from close by
               (< 12 m, the most precise lidar returns), grouped by distance;
               this shows how far out the lidar can be trusted
* halves     - two maps built from the first and second half of the scans: where both
               see the ground, how far apart are they?
* roads      - the road measurements repeated on each half; the difference is
               how repeatable each value is
"""
from __future__ import annotations

import time
import warnings
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from uga_terrain import geo, roads as roads_mod
from uga_terrain.accumulate import VoxelAccumulator
from uga_terrain.grid import TerrainGrid
from uga_terrain.progress import Progress, maybe

NEAR_M = 12.0
RANGE_BINS = (0, 5, 10, 15, 20, 25, 30, 35, 40, 50, 60, 80)


def load_scans(project, max_points: Optional[int] = None):
    """World ground points of every cached scan: (xyz, range_m, scan_no, recording_no)."""
    xyz, rng, scan, rec = [], [], [], []
    k0 = 0
    for i in range(len(project.recordings)):
        cd = project.cache(i)
        fz = cd / "frames.npz"
        if not fz.is_file():
            continue
        T = np.load(fz)["T"]
        for k in range(len(T)):
            f = cd / "frames" / ("%06d.npy" % k)
            if not f.is_file():
                continue
            g = np.load(f).astype(np.float64)
            xyz.append(geo.transform(T[k], g[:, :3]))
            rng.append(np.hypot(g[:, 0], g[:, 1]))
            scan.append(np.full(len(g), k0 + k, np.int32))
            rec.append(np.full(len(g), i, np.int16))
        k0 += len(T)
    if not xyz:
        raise RuntimeError("this map has no scan cache; build it from the start first")
    out = (np.concatenate(xyz), np.concatenate(rng), np.concatenate(scan), np.concatenate(rec))
    return out


def _cell_keys(xy: np.ndarray, cell: float) -> np.ndarray:
    ij = np.floor(xy / cell).astype(np.int64)
    return ij[:, 0] * 2_000_003 + ij[:, 1]


def _cell_median(keys: np.ndarray, z: np.ndarray):
    o = np.argsort(keys, kind="stable")
    k, zz = keys[o], z[o]
    u, s0, cnt = np.unique(k, return_index=True, return_counts=True)
    med = np.array([np.median(zz[a:a + c]) for a, c in zip(s0, cnt)])
    return u, med


def by_range(xyz, rng, cell: float) -> List[dict]:
    keys = _cell_keys(xyz[:, :2], cell)
    near = rng < NEAR_M
    u, ref = _cell_median(keys[near], xyz[near, 2])
    pos = np.searchsorted(u, keys)
    pos = np.clip(pos, 0, len(u) - 1)
    has = u[pos] == keys
    err = np.where(has, xyz[:, 2] - ref[pos], np.nan)
    rows = []
    for a, b in zip(RANGE_BINS[:-1], RANGE_BINS[1:]):
        m = has & (rng >= a) & (rng < b)
        if m.sum() < 50:
            continue
        e = err[m]
        rows.append({"from_m": a, "to_m": b, "points": int(m.sum()),
                     "bias_m": float(np.median(e)), "typical_m": float(np.median(np.abs(e))),
                     "p90_m": float(np.percentile(np.abs(e), 90))})
    return rows


def _grid(xyz, cfg) -> TerrainGrid:
    acc = VoxelAccumulator(float(cfg["grid"]["cell"]), float(cfg["grid"]["z_bin"]))
    acc.add(np.c_[xyz, np.zeros(len(xyz))])
    return TerrainGrid.from_cells(acc.cells(), cfg)


ROAD_KEYS = (("road_l", "Road width, left of path", "m"), ("road_r", "Road width, right of path", "m"),
             ("curb_h_l", "Curb height, left", "m"), ("curb_h_r", "Curb height, right", "m"),
             ("walk_l", "Sidewalk width, left", "m"), ("walk_r", "Sidewalk width, right", "m"),
             ("grade_pct", "Grade", "%"), ("bank_deg", "Banking", "deg"))


def check(project, cfg: dict, progress: Optional[Progress] = None) -> dict:
    progress = maybe(progress)
    progress.set("accuracy", 0, 4, "loading scans")
    xyz, rng, scan, rec = load_scans(project)
    cell = float(cfg["grid"]["cell"])
    out: Dict[str, object] = {"checked": time.strftime("%Y-%m-%d %H:%M:%S"),
                              "scans": int(scan.max()) + 1, "points": int(len(xyz))}
    # poses
    poses = []
    for st in project.meta.get("scan") or []:
        poses.append({"recording": Path(st.get("recording", "")).name, "frames": st.get("frames"),
                      "odometry": st.get("odometry"), "odometry_resets": st.get("odometry_resets"),
                      "gnss_vs_final_m": st.get("gnss_vs_final_m"),
                      "deskewed": bool(st.get("deskewed_frames")), "range": st.get("range")})
    out["poses"] = poses
    progress.set("accuracy", 1, 4, "height error by distance")
    out["by_range"] = by_range(xyz, rng, cell)
    # two maps from the first and second half of each recording's scans: they see
    # the same ground from different car positions, so any error in the poses or
    # in how the lidar is mounted shows up as a height difference between them
    progress.set("accuracy", 2, 4, "two half maps")
    a = np.zeros(len(scan), bool)
    for i in np.unique(rec):
        m = rec == i
        a[m] = scan[m] < np.median(np.unique(scan[m]))
    gA, gB = _grid(xyz[a], cfg), _grid(xyz[~a], cfg)
    roads = []
    # sample both half maps on a regular lattice inside the full map
    full = TerrainGrid.load(project.root / "grid")
    x0, y0, x1, y1 = full.bounds()
    X, Y = np.meshgrid(np.arange(x0, x1, 2 * cell), np.arange(y0, y1, 2 * cell))
    X, Y = X.ravel(), Y.ravel()
    za, zb = gA.sample(X, Y, "z_raw"), gB.sample(X, Y, "z_raw")
    lab = full.sample(X, Y, "label")
    ok = np.isfinite(za) & np.isfinite(zb)
    d = (za - zb)[ok]
    out["halves"] = {"cells": int(ok.sum()), "typical_m": float(np.median(np.abs(d))) if ok.any() else None,
                     "p90_m": float(np.percentile(np.abs(d), 90)) if ok.any() else None,
                     "offset_m": float(np.median(d)) if ok.any() else None}
    on_road = ok & np.isin(np.nan_to_num(lab, nan=0), (1, 2, 3))
    if on_road.any():
        dr = (za - zb)[on_road]
        out["halves"]["road_typical_m"] = float(np.median(np.abs(dr)))
        out["halves"]["road_p90_m"] = float(np.percentile(np.abs(dr), 90))
    progress.set("accuracy", 3, 4, "repeating the road measurements")
    from uga_terrain.pipeline import load_roads
    for road in load_roads(project):
        warnings.filterwarnings("ignore", "All-NaN slice", RuntimeWarning)
        blank = roads_mod.Road(road.id, road.source, road.s, road.x, road.y, road.heading, {})
        ra = roads_mod.analyse_road(blank, gA, cfg)
        blank = roads_mod.Road(road.id, road.source, road.s, road.x, road.y, road.heading, {})
        rb = roads_mod.analyse_road(blank, gB, cfg)
        rows = []
        for key, label, unit in ROAD_KEYS:
            va, vb, vf = (np.asarray(x.data[key], float) for x in (ra, rb, road))
            pos_only = key.startswith(("curb", "walk"))
            m = np.isfinite(va) & np.isfinite(vb)
            if pos_only:
                m &= (va > 0) & (vb > 0)
            f = vf[np.isfinite(vf) & ((vf > 0) if pos_only else True)]
            rows.append({"key": key, "label": label, "unit": unit,
                         "value": float(np.median(f)) if len(f) else None,
                         "found_pct": float(100 * len(f) / max(len(vf), 1)),
                         "repeat": float(np.median(np.abs(va[m] - vb[m]))) if m.sum() >= 3 else None,
                         "stations": int(m.sum())})
        roads.append({"id": road.id, "length_m": float(road.length), "rows": rows})
    out["roads"] = roads
    out["verdict"] = _verdict(out)
    progress.set("accuracy", 4, 4, "done")
    return out


def _bounds(xyz):
    return (float(xyz[:, 0].min()), float(xyz[:, 1].min()), float(xyz[:, 0].max()), float(xyz[:, 1].max()))


def _verdict(o: dict) -> List[str]:
    """Plain-language findings and what to change."""
    v = []
    for p in o.get("poses") or []:
        g = (p.get("gnss_vs_final_m") or {})
        if p.get("odometry") == "kiss_icp" and p.get("frames"):
            bad = (p.get("odometry_resets") or 0) / max(p["frames"], 1)
            if bad > 0.3:
                v.append("%s: lidar odometry disagreed with GNSS/INS on %d%% of scans, so the poses "
                         "rely on GNSS/INS alone there. Check the INS heading and the lidar mounting "
                         "in Settings." % (p["recording"], round(100 * bad)))
        if g.get("median") is not None and g["median"] > 0.3:
            v.append("%s: the final path is %.2f m from GNSS on average. Expect blurred curbs; "
                     "check GNSS quality." % (p["recording"], g["median"]))
        if not p.get("deskewed"):
            v.append("%s: scans were not corrected for the car's motion (no per-point time in the "
                     "lidar data, or Settings: lidar.deskew is off)." % p["recording"])
    br = o.get("by_range") or []
    bad = [r for r in br if r["from_m"] >= 10 and (abs(r["bias_m"]) > 0.02 or r["typical_m"] > 0.03)]
    if bad:
        v.append("Lidar heights get less reliable beyond about %d m (%.1f cm off there). Lowering "
                 "Settings > Max ground range to %d m keeps only the closer, more precise points "
                 "where you have overlapping drives." % (bad[0]["from_m"], 100 * bad[0]["typical_m"], bad[0]["from_m"]))
    h = o.get("halves") or {}
    if h.get("road_typical_m") is not None:
        v.append("Maps built from the first and second half of the scans agree to %.1f cm on roads and sidewalks "
                 "(90%% within %.1f cm)." % (100 * h["road_typical_m"], 100 * h["road_p90_m"]))
    for r in o.get("roads") or []:
        for row in r["rows"]:
            if row["key"].startswith("curb") and row["found_pct"] < 60 and row["found_pct"] > 0:
                v.append("Road %d: a curb on the %s was found at only %d%% of stations; that side may "
                         "have no curb, or Settings > Roads > curb_min may be too high."
                         % (r["id"], "left" if row["key"].endswith("_l") else "right", row["found_pct"]))
    if not v:
        v.append("No problems found.")
    return v


def section(project, road, s: float, xyz=None, scan=None, half_width: float = 15.0,
            slab: float = 0.25, max_points: int = 40000) -> dict:
    """Raw ground points in a thin slice across a road at distance ``s``."""
    i = int(np.argmin(np.abs(np.asarray(road.s) - s)))
    hx, hy = np.cos(road.heading[i]), np.sin(road.heading[i])
    rel = xyz[:, :2] - [road.x[i], road.y[i]]
    along = rel @ [hx, hy]
    lat = rel @ [-hy, hx]
    m = (np.abs(along) < slab) & (np.abs(lat) < half_width)
    idx = np.nonzero(m)[0]
    if len(idx) > max_points:
        idx = np.random.default_rng(0).choice(idx, max_points, replace=False)
    zc = road.data["z"][i]
    zc = float(zc) if np.isfinite(zc) else float(np.median(xyz[idx, 2])) if len(idx) else 0.0
    D = road.data
    f = lambda k: (None if k not in D or not np.isfinite(D[k][i]) else float(D[k][i]))
    edges = {}
    for side in ("l", "r"):
        edges[side] = {"road": f("road_" + side), "curb_h": f("curb_h_" + side),
                       "verge": f("verge_" + side), "walk": f("walk_" + side),
                       "walk_h": f("walk_h_" + side),
                       "edge": roads_mod.EDGE_NAMES[int(D["edge_" + side][i])]
                       if "edge_" + side in D and np.isfinite(D["edge_" + side][i]) else None}
    return {"road": road.id, "s": float(road.s[i]), "z_ref": zc,
            "lat": np.round(lat[idx], 3).tolist(), "dz": np.round(xyz[idx, 2] - zc, 3).tolist(),
            "scan": scan[idx].tolist(), "scans": int(scan.max()) + 1 if len(scan) else 0,
            "edges": edges, "grade_pct": f("grade_pct"), "bank_deg": f("bank_deg"),
            "cross_slope_left_pct": f("cross_slope_left_pct"),
            "cross_slope_right_pct": f("cross_slope_right_pct")}
