"""The shape of an intersection from the elevation map.

Around each junction the paved surface is found in the elevation map: smooth,
nearly level ground, bounded by curbs (height steps make the surface rough
there). Narrow links such as curb ramps and sidewalks are cut off by a
morphological opening, and only the part connected to the junction centre is
kept.

Every street leaving the junction shows as a *corridor* in that surface: a
strip of steady width with an edge on both sides. For each leg (a road the
drives used, or a corridor nobody drove into) the junction ends where the
corner curves (curb returns) end and the width settles to the street's own
width. The outline joins those cut lines with the surface edge between legs.

Corridors with no road get a short *stub* road, so every street at the
junction has lanes for turns to lead into.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

CELL = 0.25


@dataclass
class Surface:
    mask: np.ndarray      # paved and connected to the junction centre
    x0: float             # world x of column 0
    y0: float             # world y of row 0
    cell: float

    def at(self, x, y) -> np.ndarray:
        c = np.floor((np.asarray(x) - self.x0) / self.cell).astype(int)
        r = np.floor((np.asarray(y) - self.y0) / self.cell).astype(int)
        ok = (r >= 0) & (r < self.mask.shape[0]) & (c >= 0) & (c < self.mask.shape[1])
        out = np.zeros(np.shape(c), bool)
        out[ok] = self.mask[r[ok], c[ok]]
        return out


def _disk_or(m: np.ndarray, k: int) -> np.ndarray:
    out = m.copy()
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            if dr * dr + dc * dc > k * k or (dr == 0 and dc == 0):
                continue
            sh = np.zeros_like(m)
            r0, r1 = max(dr, 0), m.shape[0] + min(dr, 0)
            c0, c1 = max(dc, 0), m.shape[1] + min(dc, 0)
            sh[r0:r1, c0:c1] = m[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
            out |= sh
    return out


def _grow(seed: np.ndarray, allowed: np.ndarray) -> np.ndarray:
    reg = seed & allowed
    while True:
        nr = reg.copy()
        nr[1:] |= reg[:-1]
        nr[:-1] |= reg[1:]
        nr[:, 1:] |= reg[:, :-1]
        nr[:, :-1] |= reg[:, 1:]
        nr &= allowed
        if (nr == reg).all():
            return reg
        reg = nr


def surface(grid, x: float, y: float, cfg: dict, radius: float = 60.0) -> Optional[Surface]:
    """Paved surface connected to (x, y), within ``radius``."""
    rc = cfg.get("roads") or {}
    jc = cfg.get("junctions") or {}
    rough_max = float(jc.get("surface_rough_max") or max(float(rc.get("rough_road_max") or 0.025), 0.03))
    ax = np.arange(-radius, radius, CELL) + CELL / 2
    X, Y = np.meshgrid(x + ax, y + ax)
    Z = grid.sample(X, Y, "z")
    R = grid.sample(X, Y, "roughness")
    S = grid.sample(X, Y, "slope_deg")
    with np.errstate(invalid="ignore"):
        paved = np.isfinite(Z) & (R < rough_max) & (S < 8.0)
    k1 = 1                                             # one cell: close cracks, never a curb
    m = ~_disk_or(~_disk_or(paved, k1), k1)
    k2 = int(round(float(jc.get("surface_open_m") or 1.5) / CELL))
    core = ~_disk_or(~m, k2)                           # cut ramps and sidewalks off
    ys, xs = np.nonzero(core)
    if not len(ys):
        return None
    n = len(ax)
    d2 = (ys - n / 2) ** 2 + (xs - n / 2) ** 2
    i = int(np.argmin(d2))
    if d2[i] * CELL * CELL > 10.0 ** 2:
        return None
    seed = np.zeros_like(core)
    seed[ys[i], xs[i]] = True
    reg = _disk_or(_grow(seed, core), k2) & m
    return Surface(reg, x - radius, y - radius, CELL)


def _reach(surf: Surface, p, u, max_d: float, step: float = CELL) -> float:
    d = np.arange(0.0, max_d + step / 2, step)
    ins = surf.at(p[0] + d * u[0], p[1] + d * u[1])
    bad = np.flatnonzero(~ins)
    return float(d[bad[0]]) if len(bad) else float(max_d)


def corridor(surf: Surface, p0, u, d_max: float = 55.0, side_max: float = 12.0, step: float = 1.0) -> dict:
    """Widths of the surface across the line p0 + d u (d = 0 .. d_max).

    Per station: left / right reach (np.nan where the line has left the
    surface, or the side is not closed within ``side_max``)."""
    u = np.asarray(u, float) / np.hypot(*u)
    n = np.array([-u[1], u[0]])
    d = np.arange(0.0, d_max + 1e-9, step)
    left = np.full(len(d), np.nan)
    right = np.full(len(d), np.nan)
    on = np.zeros(len(d), bool)
    for i, di in enumerate(d):
        p = np.asarray(p0) + di * u
        if not surf.at(p[0], p[1]):
            continue
        on[i] = True
        a = _reach(surf, p, n, side_max)
        b = _reach(surf, p, -n, side_max)
        if a < side_max:
            left[i] = a
        if b < side_max:
            right[i] = b
    return {"d": d, "on": on, "left": left, "right": right}


def _settle(c: dict, d_from: float, slack: float = 1.2, hold: float = 6.0) -> Optional[dict]:
    """Where a corridor's width settles (the curb returns end), its width and
    how far it continues. None if there is no steady corridor."""
    d = c["d"]
    w = c["left"] + c["right"]
    ok = c["on"] & np.isfinite(w)
    beyond = ok & (d >= d_from)
    if beyond.sum() < 4:
        return None
    w_far = float(np.median(w[beyond]))
    good = ok & (w <= w_far + slack) & (w >= 4.0)
    nh = max(1, int(round(hold / (d[1] - d[0]))))
    for i in range(len(d) - nh + 1):
        if good[i:i + nh].all():
            # the corridor runs on while the line stays on the surface
            j = i
            while j + 1 < len(d) and c["on"][j + 1] and (not np.isfinite(w[j + 1]) or w[j + 1] <= w_far + 3.0):
                j += 1
            mid = 0.5 * (c["left"][i:j + 1] - c["right"][i:j + 1])
            return {"d_cut": float(d[i]), "d_end": float(d[j]), "width": w_far,
                    "shift": float(np.nanmedian(mid)) if np.isfinite(mid).any() else 0.0}
    return None


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def point_in_poly(x, y, poly) -> np.ndarray:
    """Even-odd rule; poly [[x, y], ...]."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    P = np.asarray(poly, float)
    inside = np.zeros(np.shape(x), bool)
    xj, yj = P[-1]
    for xi, yi in P:
        cond = ((yi > y) != (yj > y))
        with np.errstate(divide="ignore", invalid="ignore"):
            xc = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= cond & (x < xc)
        xj, yj = xi, yi
    return inside


def _min_dist(seg, polylines) -> float:
    best = np.inf
    for P in polylines:
        P = np.asarray(P, float)
        for q in seg:
            best = min(best, float(np.min(np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]))))
    return best


def fit(junction, legs: Sequence[dict], grid, cfg: dict, road_lines: Sequence = ()) -> dict:
    """Outline and undriven legs of one junction.

    legs: [{"road": id, "x", "y", "h_out"}] the ends of the roads at the
    junction, h_out pointing away from it; road_lines: [(N, 2) xy] of all
    roads, so a new leg never lies on top of one. Returns
    {"outline": [[x, y]...], "legs": [{"road" or "stub", "theta_deg", "u",
    "p0", "d_cut", "d_end", "width"}]} where a leg runs from p0 + d_cut u (the
    junction's edge) outward."""
    jc = cfg.get("junctions") or {}
    rc = cfg.get("roads") or {}
    cx, cy = junction.x, junction.y
    surf = surface(grid, cx, cy, cfg)
    if surf is None:
        return {}
    loose_cfg = {"roads": rc, "junctions": dict(jc, surface_rough_max=float(jc.get("leg_rough_max") or 0.045))}
    loose = surface(grid, cx, cy, loose_cfg) or surf
    c = np.array([cx, cy])
    r_cap = max(float(junction.r), 12.0) + 4.0
    lane_w = float(rc.get("lane_width") or 3.5)
    out = []
    for lg in legs:
        u = np.array([math.cos(lg["h_out"]), math.sin(lg["h_out"])])
        e = np.array([lg["x"], lg["y"]])
        p0 = e - u * float(np.dot(e - c, u))
        st = _settle(corridor(surf, p0, u), float(np.dot(e - p0, u)) - 2.0)
        d_cut = float(np.clip(st["d_cut"], 6.0, r_cap)) if st else float(np.dot(e - p0, u))
        out.append({"road": lg["road"], "theta": float(lg["h_out"]), "u": u, "p0": p0, "d_cut": d_cut,
                    "d_end": float(st["d_end"]) if st else d_cut + 10.0,
                    "width": float(st["width"]) if st else 2 * lane_w})
    # streets nobody drove into: the continuation of each leg, the cross
    # directions, and any clear corridor
    if jc.get("add_undriven_legs", True):
        cands = []
        for lg in out:
            cands.append((lg["theta"] + math.pi, "on", lg))
            cands.append((lg["theta"] + math.pi / 2, "cross", None))
            cands.append((lg["theta"] - math.pi / 2, "cross", None))
        for deg in range(0, 360, 4):
            cands.append((math.radians(deg), "scan", None))
        min_len = float(jc.get("stub_min_len") or 6.0)
        for th0, kind, opp in cands:
            if any(abs(_wrap(th0 - lg["theta"])) < math.radians(40) for lg in out):
                continue
            tries = []
            for dd in range(-15, 16, 3):
                th = th0 + math.radians(dd)
                tries.append((_reach(loose, c, np.array([math.cos(th), math.sin(th)]), 50.0), th))
            top = max(t[0] for t in tries)
            good = [t[1] for t in tries if t[0] >= top - 0.5]
            th = float(np.median(good))                # the middle of the directions that reach furthest
            v = np.array([math.cos(th), math.sin(th)])
            rl = _reach(loose, c, v, 50.0)
            if any(abs(_wrap(th - lg["theta"])) < math.radians(40) for lg in out):
                continue
            rs = _reach(surf, c, v, r_cap)
            d0 = float(np.clip(rs, 6.0, r_cap))
            if opp is not None:
                d0 = min(d0, opp["d_cut"])
            cor = corridor(loose, c, v)
            st = _settle(cor, d0)
            if kind == "scan":
                if st is None or not 5.0 <= st["width"] <= 20.0 or st["d_end"] - max(st["d_cut"], d0) < 10.0:
                    continue
            elif kind == "cross":
                if rl < max(14.0, rs + 6.0):
                    continue
            elif rl < 12.0:
                continue
            length = float(np.clip(rl - d0, min_len, 30.0))
            if rl - d0 < min_len and kind != "on":
                continue
            width = st["width"] if st and 5.0 <= st["width"] <= 20.0 else (opp["width"] if opp else 2 * lane_w)
            seg = [c + d * v for d in np.arange(d0 + 3.0, d0 + length + 0.1, 1.0)]
            if road_lines and _min_dist(seg, road_lines) < 0.5 * width + 0.5:
                continue
            out.append({"stub": True, "kind": kind, "theta": th, "u": v, "p0": c.copy(), "d_cut": d0,
                        "d_end": d0 + length, "width": float(width),
                        "opposite": opp["road"] if opp is not None else None})
    # outline: star-shaped around the centre; each leg closes at its cut line
    reach_cap = max(lg["d_cut"] for lg in out) + 2.0
    order = sorted(out, key=lambda lg: _wrap(lg["theta"]))
    raw, cut_at = [], []
    for deg in np.arange(0.0, 360.0, 2.0):
        th = math.radians(deg)
        v = np.array([math.cos(th), math.sin(th)])
        # between two legs the corner reaches no further than the farther of their cuts
        nxt = min(order, key=lambda lg: (lg["theta"] - th) % (2 * math.pi))
        prv = min(order, key=lambda lg: (th - lg["theta"]) % (2 * math.pi))
        cap = min(reach_cap, max(nxt["d_cut"], prv["d_cut"]))
        rho = min(_reach(surf, c, v, cap), cap)
        cut = None
        for lg in out:
            den = float(np.dot(v, lg["u"]))
            if den <= 0.2:
                continue
            t = (lg["d_cut"] - float(np.dot(c - lg["p0"], lg["u"]))) / den
            if t <= 0:
                continue
            lat = float(np.dot(c + t * v - lg["p0"], np.array([-lg["u"][1], lg["u"][0]])))
            if abs(lat) <= 0.5 * lg["width"] + 2.0:
                cut = t if cut is None else min(cut, t)
        raw.append(rho)
        cut_at.append(cut)
    raw = np.asarray(raw)
    k = len(raw)
    smooth = np.array([np.median(raw[[(i + d) % k for d in range(-3, 4)]]) for i in range(k)])
    poly = []
    for i, deg in enumerate(np.arange(0.0, 360.0, 2.0)):
        th = math.radians(deg)
        rho = smooth[i] if cut_at[i] is None else cut_at[i]      # across a street: its cut line
        poly.append([cx + rho * math.cos(th), cy + rho * math.sin(th)])
    rows = []
    for lg in out:
        row = {"theta_deg": round(math.degrees(_wrap(lg["theta"])), 1), "d_cut": round(lg["d_cut"], 2),
               "d_end": round(lg["d_end"], 2), "width": round(lg["width"], 2),
               "u": [round(float(v), 6) for v in lg["u"]], "p0": [round(float(v), 3) for v in lg["p0"]]}
        if lg.get("stub"):
            row.update(stub=True, kind=lg["kind"], opposite=lg.get("opposite"))
        else:
            row["road"] = lg["road"]
        rows.append(row)
    return {"outline": [[round(a, 3), round(b, 3)] for a, b in poly], "legs": rows}


def leg_ends(junction, roads) -> List[dict]:
    """Ends of the roads at a junction, heading away from it."""
    out = []
    for r in roads:
        if r.meta.get("junction_start") == junction.id:
            out.append({"road": r.id, "x": float(r.x[0]), "y": float(r.y[0]), "h_out": float(r.heading[0])})
        if r.meta.get("junction_end") == junction.id:
            out.append({"road": r.id, "x": float(r.x[-1]), "y": float(r.y[-1]),
                        "h_out": float(r.heading[-1]) + math.pi})
    return out


def stub_roads(juncs, grid, cfg: dict, first_id: int, roads=()):
    """A short road for every street at a junction that no drive used. Its
    width is measured from the curbs like any road; where that fails, the
    street it continues (``roads``) or the junction's surface gives it."""
    from uga_terrain.roads import Road, analyse_road
    step = float((cfg.get("roads") or {}).get("station_step") or 1.0)
    out = []
    rid = first_id
    for jn in juncs:
        for lg in jn.legs:
            if not lg.get("stub"):
                continue
            u = np.asarray(lg["u"], float)
            p0 = np.asarray(lg["p0"], float)
            d = np.arange(lg["d_cut"], lg["d_end"] + 1e-6, step)
            if len(d) < 3:
                continue
            x, y = p0[0] + d * u[0], p0[1] + d * u[1]
            h = np.full(len(d), math.atan2(u[1], u[0]))
            r = Road(rid, "junction %d: street nobody drove" % jn.id, d - d[0], x, y, h)
            analyse_road(r, grid, cfg)
            guess = float(lg["width"])
            opp = next((o for o in roads if o.id == lg.get("opposite")), None)
            if opp is not None:
                n_end = min(15, len(opp.s))
                sl = slice(0, n_end) if opp.meta.get("junction_start") == jn.id else slice(len(opp.s) - n_end, None)
                ow = np.asarray(opp.data.get("road_l", [np.nan]))[sl] + np.asarray(opp.data.get("road_r", [np.nan]))[sl]
                if np.isfinite(ow).any():
                    guess = float(np.nanmedian(ow))
            mw = np.asarray(r.data["road_l"]) + np.asarray(r.data["road_r"])
            meas = float(np.nanmedian(mw)) if np.isfinite(mw).any() else np.nan
            width = meas if np.isfinite(meas) and 0.7 * guess <= meas <= 1.4 * guess and meas >= 5.0 else guess
            lg["width"] = round(width, 2)
            # widths of an unseen street are a guess: keep them steady
            half = 0.5 * width
            r.data["road_l"] = np.full(len(d), half)
            r.data["road_r"] = np.full(len(d), half)
            r.meta.update({"junction_start": jn.id, "stub": True})
            from uga_terrain import lanes
            lay = lanes.layout(r, [], cfg)
            for k, b in enumerate(lay.pop("bounds")):
                r.data["lb%d" % k] = b
            r.meta["lanes"] = lay
            lg["road"] = rid
            out.append(r)
            rid += 1
    return out
