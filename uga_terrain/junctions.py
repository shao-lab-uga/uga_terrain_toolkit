"""Intersections: where drives turn or cross, and how their lanes connect.

Finding them (``find``)
    * a drive that turns (heading changes by more than ``min_turn_deg``
      within ``turn_window`` metres): the junction centre is where the
      straight line before the turn meets the straight line after it, and the
      junction reaches as far as the turn itself;
    * two drives that cross at an angle;
    Candidates closer than ``merge_dist`` are one junction.

Cutting (``cut``)
    Each drive is cut where it enters and leaves a junction. The pieces
    outside junctions become roads (repeated pieces of the same street are
    merged by ``roads.split_roads``), each knowing which junction its start
    and end touch. A drive that goes through a junction records the movement
    (from which leg to which leg), so driven movements are known.

OpenDRIVE (``add_to_xodr``)
    Every road that ends at a junction gets a link to it. For each pair of
    legs, every allowed movement gets a *connecting road*: a smooth curve from
    the centre line of the incoming road to the centre line of the outgoing
    one, with one lane whose edges meet the incoming and outgoing lane edges
    exactly. Where a drive made that movement, the lane follows the driven
    path (moved onto the lane centres at both ends); otherwise it is a smooth
    cubic curve. Which lane connects to which:

    * right turn: the rightmost incoming lane to the rightmost outgoing lane;
    * left turn: the leftmost to the leftmost;
    * straight on: lanes paired from the right.

    Movements nobody drove are added too (``junctions.movements: all``), as
    the recordings rarely cover every turn; their connecting roads are named
    "(not driven)" so they can be removed where a turn is not allowed.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from uga_terrain.roads import resample_path


@dataclass
class Junction:
    id: int
    x: float
    y: float
    r: float
    why: List[str] = field(default_factory=list)
    movements: Dict[str, int] = field(default_factory=dict)  # "legA>legB" -> times driven (filled later)
    paths: Dict[str, list] = field(default_factory=dict)     # "legA>legB" -> a driven path [[x, y], ...]
    outline: list = field(default_factory=list)              # [[x, y], ...] edge fitted to the curbs
    legs: list = field(default_factory=list)                 # streets meeting here (junction_shape.fit)

    def to_json(self) -> dict:
        return {"id": self.id, "x": self.x, "y": self.y, "r": self.r, "why": self.why,
                "movements": self.movements, "paths": self.paths, "outline": self.outline, "legs": self.legs}

    @classmethod
    def from_json(cls, d: dict) -> "Junction":
        return cls(int(d["id"]), float(d["x"]), float(d["y"]), float(d["r"]), list(d.get("why") or []),
                   dict(d.get("movements") or {}), dict(d.get("paths") or {}),
                   list(d.get("outline") or []), list(d.get("legs") or []))

    def contains(self, x, y) -> np.ndarray:
        if len(self.outline) >= 3:
            from uga_terrain.junction_shape import point_in_poly
            return point_in_poly(x, y, self.outline)
        return np.hypot(np.asarray(x) - self.x, np.asarray(y) - self.y) < self.r


def _wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def _line_cross(p, h1, q, h2) -> Optional[np.ndarray]:
    """Where the line through p (heading h1) meets the line through q (h2)."""
    d1 = np.array([math.cos(h1), math.sin(h1)])
    d2 = np.array([math.cos(h2), math.sin(h2)])
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 0.2:          # nearly parallel
        return None
    w = q - p
    a = (w[0] * d2[1] - w[1] * d2[0]) / den
    return p + a * d1


def _turns(s, x, y, h, cfg) -> List[dict]:
    jc = cfg.get("junctions") or {}
    min_turn = math.radians(float(jc.get("min_turn_deg") or 50.0))
    win = float(jc.get("turn_window") or 35.0)
    out = []
    n = len(s)
    i = 0
    while i < n:
        j = int(np.searchsorted(s, s[i] + win))
        if j >= n:
            break
        if abs(h[j] - h[i]) >= min_turn:
            # the turn: widen to where the heading settles on each side
            a, b = i, j
            total = h[b] - h[a]
            dh = np.abs(np.gradient(h, s))
            pk = a + int(np.argmax(dh[a:b + 1]))
            settle = math.radians(2.0) / 1.0          # rad per m
            a2 = pk
            while a2 > 0 and dh[a2] > settle and s[pk] - s[a2] < win:
                a2 -= 1
            b2 = pk
            while b2 < n - 1 and dh[b2] > settle and s[b2] - s[pk] < win:
                b2 += 1
            total = h[b2] - h[a2]
            if abs(total) >= min_turn and abs(total) < math.radians(160):
                c = _line_cross(np.array([x[a2], y[a2]]), h[a2], np.array([x[b2], y[b2]]), h[b2])
                if c is not None:
                    rad = max(math.hypot(x[a2] - c[0], y[a2] - c[1]), math.hypot(x[b2] - c[0], y[b2] - c[1]))
                    out.append({"x": float(c[0]), "y": float(c[1]), "r": rad,
                                "why": "%s turn of %.0f deg" % ("left" if total > 0 else "right",
                                                                abs(math.degrees(total)))})
            i = b2 + 1
            continue
        i += 1
    return out


def _crossings(paths: Sequence[tuple]) -> List[dict]:
    """Points where two different drives cross at more than 35 degrees."""
    out = []
    for a in range(len(paths)):
        for b in range(a + 1, len(paths)):
            _, sa, xa, ya, ha = paths[a]
            _, sb, xb, yb, hb = paths[b]
            for i in range(0, len(xa) - 1, 2):
                d = np.hypot(xb - xa[i], yb - ya[i])
                j = int(np.argmin(d))
                if d[j] > 2.0:
                    continue
                ang = abs(_wrap(ha[i] - hb[j]))
                ang = min(ang, math.pi - ang)
                if ang > math.radians(35):
                    out.append({"x": float(0.5 * (xa[i] + xb[j])), "y": float(0.5 * (ya[i] + yb[j])),
                                "r": 0.0, "why": "drives cross at %.0f deg" % math.degrees(ang)})
    return out


def find(drives: Sequence[Tuple[str, np.ndarray]], cfg: dict) -> List[Junction]:
    """drives: [(name, xy (N,2))] -> junctions."""
    jc = cfg.get("junctions") or {}
    if not jc.get("detect", True):
        return []
    step = float((cfg.get("roads") or {}).get("station_step") or 1.0)
    paths = []
    cands = []
    for name, xy in drives:
        rs = resample_path(xy, step, smooth_m=4.0)
        if rs is None:
            continue
        s, x, y, h = rs
        paths.append((name, s, x, y, h))
        cands += _turns(s, x, y, h, cfg)
    cands += _crossings(paths)
    merge = float(jc.get("merge_dist") or 20.0)
    r_min, r_max = float(jc.get("min_radius") or 8.0), float(jc.get("max_radius") or 25.0)
    groups: List[List[dict]] = []
    for c in sorted(cands, key=lambda c: -c["r"]):
        for g in groups:
            if math.hypot(g[0]["x"] - c["x"], g[0]["y"] - c["y"]) < merge:
                g.append(c)
                break
        else:
            groups.append([c])
    out = []
    for k, g in enumerate(groups):
        x = float(np.mean([c["x"] for c in g]))
        y = float(np.mean([c["y"] for c in g]))
        r = max([c["r"] for c in g] + [r_min])
        out.append(Junction(1000 + k, x, y, float(min(r, r_max)), sorted({c["why"] for c in g})))
    return out


def openings(roads, known: Sequence[Junction], cfg: dict) -> List[Junction]:
    """Junctions with side streets nobody turned into: stretches where the
    road edge moves out by more than ``opening_m`` on either side (the mouth
    of a cross street, with its curb returns), at least ``opening_min_len``
    long. Openings on both sides close together are one junction."""
    jc = cfg.get("junctions") or {}
    if not jc.get("detect", True) or not jc.get("detect_side_streets", True):
        return []
    jump = float(jc.get("opening_m") or 2.5)
    min_len = float(jc.get("opening_min_len") or 6.0)
    r_min, r_max = float(jc.get("min_radius") or 8.0), float(jc.get("max_radius") or 25.0)
    out: List[Junction] = []
    next_id = 1000 + len(known)
    for r in roads:
        if r.length < 2 * min_len or "road_l" not in r.data:
            continue
        stretches = []
        for key in ("road_l", "road_r"):
            w = np.asarray(r.data[key], float)
            if not np.isfinite(w).any():
                continue
            base = float(np.nanpercentile(w, 25))
            op = np.nan_to_num(w, nan=0.0) > base + jump
            edges = np.flatnonzero(np.diff(np.r_[0, op.astype(int), 0]))
            for a, b in zip(edges[::2], edges[1::2]):
                if r.s[b - 1] - r.s[a] >= min_len and r.s[a] > 3.0 and r.s[b - 1] < r.length - 3.0:
                    stretches.append([float(r.s[a]), float(r.s[b - 1]), key])
        stretches.sort()
        merged = []
        for st in stretches:
            if merged and st[0] - merged[-1][1] < 8.0:
                merged[-1][1] = max(merged[-1][1], st[1])
                merged[-1][2] = merged[-1][2] if merged[-1][2] == st[2] else "both"
            else:
                merged.append(list(st))
        for s0, s1, side in merged:
            sm = 0.5 * (s0 + s1)
            i = int(np.argmin(np.abs(r.s - sm)))
            x, y = float(r.x[i]), float(r.y[i])
            if any(math.hypot(x - j.x, y - j.y) < j.r + 5.0 for j in list(known) + out):
                continue
            rad = float(np.clip(0.5 * (s1 - s0) + 2.0, r_min, r_max))
            what = {"road_l": "left", "road_r": "right", "both": "both sides"}[side]
            out.append(Junction(next_id, x, y, rad, ["side street on the %s, not driven into" % what
                                                     if side != "both" else "cross street, not driven into"]))
            next_id += 1
    return out


def cut(drives: Sequence[Tuple[str, np.ndarray]], junctions: Sequence[Junction]):
    """Drives cut at junction edges. Returns (pieces, passes):
    pieces: [(name, xy, start_junction or None, end_junction or None)];
    passes: [(junction id, heading in, heading out, name, xy)] per drive through a junction."""
    pieces, passes = [], []
    for name, xy in drives:
        xy = np.asarray(xy, float)
        inside = np.full(len(xy), -1)
        for j in junctions:
            m = j.contains(xy[:, 0], xy[:, 1])
            inside[m & (inside < 0)] = j.id
        k = 0
        n = len(xy)
        prev_j = None
        while k < n:
            if inside[k] >= 0:
                jid = int(inside[k])
                e = k
                while e < n and inside[e] == jid:
                    e += 1
                # heading in/out of the junction from 5 m outside it
                if k > 0 and e < n:
                    a = xy[max(0, k - 1)] - xy[max(0, k - 6)]
                    b = xy[min(n - 1, e + 5)] - xy[e]
                    if np.hypot(*a) > 0.5 and np.hypot(*b) > 0.5:
                        passes.append((jid, math.atan2(a[1], a[0]), math.atan2(b[1], b[0]), name,
                                       xy[max(0, k - 3):min(n, e + 3)]))
                prev_j = jid
                k = e
                continue
            e = k
            while e < n and inside[e] < 0:
                e += 1
            nxt = int(inside[e]) if e < n else None
            pieces.append((name, xy[k:e], prev_j, nxt))
            prev_j = None
            k = e
    return pieces, passes


# --- OpenDRIVE -------------------------------------------------------------------------

def _f(v: float) -> str:
    return "%.10g" % float(v)


def _hermite_xy(p0, h0, p1, h1, k: Optional[float] = None):
    """paramPoly3 (normalized) in the local frame of p0/h0, and its length."""
    c, s = math.cos(-h0), math.sin(-h0)
    R = np.array([[c, -s], [s, c]])
    P1 = R @ (np.asarray(p1) - np.asarray(p0))
    chord = float(np.hypot(*P1))
    k = k if k is not None else chord
    T0 = np.array([k, 0.0])
    a1 = h1 - h0
    T1 = k * np.array([math.cos(a1), math.sin(a1)])
    a = np.zeros(2)
    b = T0
    cc = 3 * P1 - 2 * T0 - T1
    d = -2 * P1 + T0 + T1
    p = np.linspace(0, 1, 20001)
    u = b[0] * p + cc[0] * p ** 2 + d[0] * p ** 3
    v = b[1] * p + cc[1] * p ** 2 + d[1] * p ** 3
    L = float(np.sum(np.hypot(np.diff(u), np.diff(v))))
    return (a[0], b[0], cc[0], d[0]), (a[1], b[1], cc[1], d[1]), L


def _smoothstep(w):
    w = np.clip(w, 0.0, 1.0)
    return w * w * (3 - 2 * w)


def _driven_reference(path, A, B, c0: float, c1: float):
    """Reference line (x, y, heading) of a connecting road that follows a
    driven path: the path is moved so it starts and ends exactly on the lane
    centres of the two roads, then shifted sideways onto the reference line."""
    from uga_terrain.roads import resample_path
    P = np.asarray(path, float)
    h_out = B["h_in"] + math.pi
    nA = np.array([-math.sin(A["h_in"]), math.cos(A["h_in"])])
    nB = np.array([-math.sin(h_out), math.cos(h_out)])
    S0 = np.array([A["x"], A["y"]]) + c0 * nA
    E0 = np.array([B["x"], B["y"]]) + c1 * nB
    # keep only the part between the two lane-centre end points
    i0 = int(np.argmin(np.hypot(*(P - S0).T)))
    i1 = int(np.argmin(np.hypot(*(P - E0).T)))
    if i1 - i0 < 3:
        return None
    P = P[i0:i1 + 1]
    rs = resample_path(P, 0.5, smooth_m=1.5)
    if rs is None:
        return None
    s, x, y, _h = rs
    w = s / max(s[-1], 1e-9)
    d0, d1 = S0 - np.array([x[0], y[0]]), E0 - np.array([x[-1], y[-1]])
    x = x + (1 - _smoothstep(w)) * d0[0] + _smoothstep(w) * d1[0]
    y = y + (1 - _smoothstep(w)) * d0[1] + _smoothstep(w) * d1[1]
    h = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    # bend the first and last metres onto the roads' headings
    h_a = A["h_in"] + 2 * math.pi * round((h[0] - A["h_in"]) / (2 * math.pi))
    h_b = h_out + 2 * math.pi * round((h[-1] - h_out) / (2 * math.pi))
    fade = np.clip(s / 2.0, 0, 1)
    h = h_a + (h - h_a) * fade
    fade = np.clip((s[-1] - s) / 2.0, 0, 1)
    h = h_b + (h - h_b) * fade
    c = c0 + (c1 - c0) * _smoothstep(w)
    rx, ry = x + c * np.sin(h), y - c * np.cos(h)
    rx[0], ry[0], rx[-1], ry[-1] = A["x"], A["y"], B["x"], B["y"]
    return rx, ry, h


def _cubic(v0, d0, v1, d1, L):
    """a + b s + c s^2 + d s^3 with the given values and slopes at 0 and L."""
    L = max(L, 1e-6)
    a, b = v0, d0
    c = (3 * (v1 - v0) / L - 2 * d0 - d1) / L
    d = (2 * (v0 - v1) / L + d0 + d1) / (L * L)
    return a, b, c, d


def _leg_end(road, contact: str):
    """State of a leg road (xodr_check.Road) at its junction end."""
    from uga_terrain import xodr_check as X
    ref = X.reference(road)
    s = 0.0 if contact == "start" else road.length
    row = ref[0] if contact == "start" else ref[-1]
    h_ref = float(row[3])
    sgn = 1.0 if contact == "end" else -1.0          # +1: s grows into the junction
    h_in = h_ref if contact == "end" else h_ref + math.pi
    z = float(X._eval(road.elevation, np.array([s]))[0]) if road.elevation else 0.0
    eps = 0.5
    s2 = min(max(s - sgn * eps, 0.0), road.length)
    z2 = float(X._eval(road.elevation, np.array([s2]))[0]) if road.elevation else 0.0
    slope_in = (z - z2) / eps                        # dz per metre, moving into the junction
    sec = road.section_at(s)
    b = X.lane_bounds(road, sec, np.array([s]))
    lanes = {}
    for lid, (inner, outer) in b.items():
        ln = sec.lanes[lid]
        if ln.type != "driving":
            continue
        # vehicles in right lanes (negative ids) drive along +s
        incoming = (lid < 0) == (contact == "end")
        lanes[lid] = {"inner": float(inner[0]), "outer": float(outer[0]), "incoming": incoming}
    return {"x": float(row[1]), "y": float(row[2]), "h_in": h_in, "z": z, "slope_in": slope_in,
            "lanes": lanes, "contact": contact}


def _pick(inc: List[int], out: List[int], turn: str) -> List[Tuple[int, int]]:
    """Lane pairs. inc/out are ordered from the right (outermost) to the left."""
    if not inc or not out:
        return []
    if turn == "right":
        return [(inc[0], out[0])]
    if turn == "left":
        return [(inc[-1], out[-1])]
    n = min(len(inc), len(out))
    return list(zip(inc[:n], out[:n]))


def _elevation(geoms, c0, c1, A, B, ground, z_off: float):
    """Elevation records of a connecting road: the ground height under its lane
    centre (from the map), joined smoothly to the two roads' heights."""
    from uga_terrain.xodr import eval_param_poly3, hermite
    from uga_terrain.roads import _gauss1d
    L = float(sum(g[4] for g in geoms))
    if ground is None or L < 2.0:
        z = _cubic(A["z"], A["slope_in"], B["z"], -B["slope_in"], L)
        return [(0.0,) + tuple(z)]
    pts = eval_param_poly3(geoms, n_per=max(20, int(L / len(geoms) * 4)))
    d = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)]) * (L / max(float(d.sum()), 1e-9))
    h = np.unwrap(np.arctan2(np.gradient(pts[:, 1]), np.gradient(pts[:, 0])))
    c = c0 + (c1 - c0) * _smoothstep(s / L)
    z = np.asarray(ground(pts[:, 0] - c * np.sin(h), pts[:, 1] + c * np.cos(h)), float) + z_off
    ok = np.isfinite(z)
    if ok.sum() < 5:
        zc = _cubic(A["z"], A["slope_in"], B["z"], -B["slope_in"], L)
        return [(0.0,) + tuple(zc)]
    z = np.interp(s, s[ok], z[ok])
    z = _gauss1d(z, 1.5 / max(L / len(s), 1e-3))
    w = _smoothstep(s / L)
    z = z + (1 - w) * (A["z"] - z[0]) + w * (B["z"] - z[-1])
    knots = np.unique(np.concatenate([np.arange(0.0, L, 3.0), [L]]))
    zk = np.interp(knots, s, z)
    dk = np.gradient(zk, knots) if len(knots) > 2 else np.full(len(knots), (zk[-1] - zk[0]) / max(L, 1e-9))
    dk[0], dk[-1] = A["slope_in"], -B["slope_in"]
    return hermite(knots, zk, dk)[:-1] if len(knots) > 1 else []


def add_to_xodr(root: ET.Element, juncs: Sequence[Junction], road_ends: Dict[int, dict], cfg: dict,
                next_id: int, ground=None, z_off: float = 0.0, removed_turns=frozenset(),
                details=None) -> dict:
    """Junction elements, connecting roads and leg links, added to ``root``.
    road_ends: road id -> {"start": junction id or None, "end": ...};
    ground: optional f(x, y) -> map height, for the connecting roads' elevation."""
    from uga_terrain import xodr_check as X
    jc = cfg.get("junctions") or {}
    every = str(jc.get("movements") or "all") == "all"
    _, xroads, _ = X.read_root(root)
    by_id = {r.id: r for r in xroads}
    elems = {e.get("id"): e for e in root.findall("road")}
    info = {"junctions": [], "connecting_roads": 0}
    rid = next_id
    for j in juncs:
        legs = []
        for road_id, ends in road_ends.items():
            for contact in ("start", "end"):
                if ends.get(contact) == j.id and str(road_id) in by_id:
                    st = _leg_end(by_id[str(road_id)], contact)
                    st["road"] = str(road_id)
                    legs.append(st)
        if len(legs) < 2:
            continue
        # leg links
        for st in legs:
            e = elems[st["road"]]
            link = e.find("link")
            if link is None:
                link = ET.Element("link")
                e.insert(0, link)
            tag = "predecessor" if st["contact"] == "start" else "successor"
            for old in link.findall(tag):
                link.remove(old)
            ET.SubElement(link, tag, elementType="junction", elementId=str(j.id))
        je = ET.SubElement(root, "junction", id=str(j.id), name="junction_%d" % j.id)
        n_con = 0
        rows = []
        for A in legs:
            # incoming lanes ordered from the right: in travel frame offsets are
            # t * dir; rightmost = most negative
            dir_a = 1.0 if A["contact"] == "end" else -1.0
            inc = sorted([l for l, v in A["lanes"].items() if v["incoming"]],
                         key=lambda l: dir_a * A["lanes"][l]["outer"])
            for B in legs:
                if B is A:
                    continue
                dir_b = 1.0 if B["contact"] == "start" else -1.0     # travel along +s leaving at start
                outl = sorted([l for l, v in B["lanes"].items() if not v["incoming"]],
                              key=lambda l: dir_b * B["lanes"][l]["outer"])
                h_out = B["h_in"] + math.pi
                ang = float(_wrap(h_out - A["h_in"]))
                if abs(ang) > math.radians(150):
                    continue                                  # U-turn
                turn = "straight" if abs(ang) < math.radians(35) else ("left" if ang > 0 else "right")
                key = "%s>%s" % (A["road"], B["road"])
                driven = int(j.movements.get(key, 0))
                if not driven and not every:
                    continue
                pairs = _pick(inc, outl, turn)
                path = j.paths.get(key)
                for la, lb in pairs:
                    if "%s:%d>%s:%d" % (A["road"], la, B["road"], lb) in removed_turns:
                        continue
                    LA, LB = A["lanes"][la], B["lanes"][lb]
                    # lane edges in the travel frame (right is negative)
                    in0, out0 = dir_a * LA["inner"], dir_a * LA["outer"]
                    in1, out1 = dir_b * LB["inner"], dir_b * LB["outer"]
                    geoms = None
                    if path is not None and len(pairs) == 1:
                        ref = _driven_reference(path, A, B, 0.5 * (in0 + out0), 0.5 * (in1 + out1))
                        if ref is not None:
                            from uga_terrain.xodr import fit_param_poly3
                            geoms = fit_param_poly3(ref[0], ref[1], ref[2], 8.0)
                    if geoms is None:
                        U, V, L = _hermite_xy((A["x"], A["y"]), A["h_in"], (B["x"], B["y"]), h_out)
                        geoms = [(0.0, A["x"], A["y"], A["h_in"], L, U, V)]
                    L = float(sum(g[4] for g in geoms))
                    rid += 1
                    name = "%s turn %s lane %d to %s lane %d%s" % (
                        turn.capitalize() if turn != "straight" else "Straight", A["road"], la, B["road"], lb,
                        "" if driven else " (not driven)")
                    if turn == "straight":
                        name = name.replace("Straight turn", "Straight from")
                    r = ET.SubElement(root, "road", name=name, length=_f(L), id=str(rid),
                                      junction=str(j.id))
                    lk = ET.SubElement(r, "link")
                    ET.SubElement(lk, "predecessor", elementType="road", elementId=A["road"],
                                  contactPoint=A["contact"])
                    ET.SubElement(lk, "successor", elementType="road", elementId=B["road"],
                                  contactPoint=B["contact"])
                    if details is not None:          # the incoming road's type and speed limit
                        from uga_terrain.xodr import _type_element
                        _type_element(r, details(A["road"]))
                    else:
                        ET.SubElement(r, "type", s="0", type="town")
                    pv = ET.SubElement(r, "planView")
                    for (s0, gx, gy, gh, gl, U, V) in geoms:
                        g = ET.SubElement(pv, "geometry", s=_f(s0), x=_f(gx), y=_f(gy), hdg=_f(gh),
                                          length=_f(gl))
                        ET.SubElement(g, "paramPoly3", aU=_f(U[0]), bU=_f(U[1]), cU=_f(U[2]), dU=_f(U[3]),
                                      aV=_f(V[0]), bV=_f(V[1]), cV=_f(V[2]), dV=_f(V[3]), pRange="normalized")
                    ep = ET.SubElement(r, "elevationProfile")
                    for (s0, a_, b_, c_, d_) in _elevation(geoms, 0.5 * (in0 + out0), 0.5 * (in1 + out1),
                                                            A, B, ground, z_off):
                        ET.SubElement(ep, "elevation", s=_f(s0), a=_f(a_), b=_f(b_), c=_f(c_), d=_f(d_))
                    ET.SubElement(r, "lateralProfile")
                    lanes = ET.SubElement(r, "lanes")
                    off = _cubic(in0, 0.0, in1, 0.0, L)
                    ET.SubElement(lanes, "laneOffset", s="0", a=_f(off[0]), b=_f(off[1]), c=_f(off[2]),
                                  d=_f(off[3]))
                    sec = ET.SubElement(lanes, "laneSection", s="0")
                    ctr = ET.SubElement(sec, "center")
                    cl = ET.SubElement(ctr, "lane", id="0", type="none", level="false")
                    ET.SubElement(cl, "roadMark", sOffset="0", type="none", weight="standard",
                                  color="standard", width="0.12")
                    right = ET.SubElement(sec, "right")
                    ln = ET.SubElement(right, "lane", id="-1", type="driving", level="false")
                    ll = ET.SubElement(ln, "link")
                    ET.SubElement(ll, "predecessor", id=str(la))
                    ET.SubElement(ll, "successor", id=str(lb))
                    w = _cubic(in0 - out0, 0.0, in1 - out1, 0.0, L)
                    ET.SubElement(ln, "width", sOffset="0", a=_f(w[0]), b=_f(w[1]), c=_f(w[2]), d=_f(w[3]))
                    ET.SubElement(ln, "roadMark", sOffset="0", type="none", weight="standard",
                                  color="standard", width="0.12")
                    n_con += 1
                    con = ET.SubElement(je, "connection", id=str(n_con - 1), incomingRoad=A["road"],
                                        connectingRoad=str(rid), contactPoint="start")
                    ET.SubElement(con, "laneLink", **{"from": str(la), "to": "-1"})
                    rows.append({"from_road": A["road"], "from_lane": la, "to_road": B["road"], "to_lane": lb,
                                 "turn": turn, "driven": driven, "connecting_road": rid,
                                 "length_m": round(L, 2), "shape": "driven path" if len(geoms) > 1 or (
                                     path is not None and len(pairs) == 1) else "smooth curve"})
        info["junctions"].append({"id": j.id, "legs": [st["road"] for st in legs], "connections": rows,
                                  "why": j.why})
        info["connecting_roads"] += n_con
    return info
