"""Check an OpenDRIVE (.xodr) file: is it valid, drivable and accurate?

The checker reads only the file, never the app's own road data, so it tests
exactly what CARLA or any other OpenDRIVE user will get. Three groups:

* structure   - the rules of the OpenDRIVE format that simulators rely on:
                ids, geometry lengths and joins, profile records, lane ids,
                lane widths, links between roads and junction connections;
* lanes       - driving lane widths: plausible, and steady along the road;
* replay      - a recorded vehicle path (the car's own trajectory, or any
                trajectory in the map's x/y frame) placed onto the map: which
                road and lane each pose falls in, how far it is from the lane
                centre, whether it drives the lane's direction, and how well
                the road surface height matches the vehicle's height.

Each finding has a level: "error" (simulators may reject or misread the
file), "warning" (valid, but not a drivable or accurate map) or "info".
"""
from __future__ import annotations

import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

STEP = 0.05                 # m, geometry sampling
GAP_ERR, GAP_WARN = 0.05, 0.01
HDG_WARN = math.radians(0.5)
LANE_W_MIN, LANE_W_MAX = 2.5, 4.6        # m, plausible driving lane widths
LANE_W_RATE = 0.05                       # m per m: steeper width change than this is not a taper
LANE_W_SPREAD = 0.30                     # m, max-min of one lane along a road
DRIVABLE = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "bidirectional"}


# --- reading ------------------------------------------------------------------------

def _num(e, k, d=0.0) -> float:
    v = e.get(k)
    return float(v) if v not in (None, "") else d


@dataclass
class Poly:
    s: float
    a: float
    b: float
    c: float
    d: float

    def __call__(self, ds):
        return self.a + self.b * ds + self.c * ds * ds + self.d * ds ** 3


def _polys(parent, tag, skey="s") -> List[Poly]:
    if parent is None:
        return []
    return [Poly(_num(e, skey), _num(e, "a"), _num(e, "b"), _num(e, "c"), _num(e, "d"))
            for e in parent.findall(tag)]


def _eval(polys: Sequence[Poly], s, base: float = 0.0, default: float = 0.0):
    """Piecewise cubic (records start at base + poly.s) evaluated at s."""
    s = np.asarray(s, float)
    out = np.full(s.shape, default)
    if not polys:
        return out
    starts = np.array([base + p.s for p in polys])
    k = np.clip(np.searchsorted(starts, s, side="right") - 1, 0, len(polys) - 1)
    for i, p in enumerate(polys):
        m = k == i
        if m.any():
            out[m] = p(s[m] - starts[i])
    return out


@dataclass
class Lane:
    id: int
    type: str
    widths: List[Poly]
    succ: Optional[int] = None
    pred: Optional[int] = None


@dataclass
class Section:
    s: float
    lanes: Dict[int, Lane]
    s_end: float = 0.0

    def ids(self, side: int) -> List[int]:
        return sorted((i for i in self.lanes if i * side > 0), key=abs)


@dataclass
class Road:
    id: str
    name: str
    length: float
    junction: str
    geoms: List[dict]
    elevation: List[Poly]
    superelevation: List[Poly]
    lane_offset: List[Poly]
    sections: List[Section]
    pred: Optional[dict] = None
    succ: Optional[dict] = None
    ref: Optional[np.ndarray] = field(default=None, repr=False)   # s, x, y, hdg samples

    def section_at(self, s: float) -> Section:
        k = 0
        for i, sec in enumerate(self.sections):
            if sec.s <= s + 1e-9:
                k = i
        return self.sections[k]


@dataclass
class Junction:
    id: str
    connections: List[dict]


def _geom_samples(g: dict) -> np.ndarray:
    """(ds, x, y, hdg) along one geometry record, about every STEP metres."""
    L = g["length"]
    n = max(2, int(math.ceil(L / STEP)) + 1)
    x0, y0, h0 = g["x"], g["y"], g["hdg"]
    kind = g["kind"]
    if kind == "line":
        ds = np.linspace(0, L, n)
        return np.stack([ds, x0 + ds * math.cos(h0), y0 + ds * math.sin(h0), np.full(n, h0)], 1)
    if kind == "arc":
        k = g["curvature"]
        ds = np.linspace(0, L, n)
        if abs(k) < 1e-12:
            return np.stack([ds, x0 + ds * math.cos(h0), y0 + ds * math.sin(h0), np.full(n, h0)], 1)
        h = h0 + k * ds
        return np.stack([ds, x0 + (np.sin(h) - math.sin(h0)) / k, y0 - (np.cos(h) - math.cos(h0)) / k, h], 1)
    if kind == "spiral":
        k0, k1 = g["curvStart"], g["curvEnd"]
        m = n * 4
        ds = np.linspace(0, L, m)
        h = h0 + k0 * ds + 0.5 * (k1 - k0) / max(L, 1e-12) * ds * ds
        dx, dy = np.cos(h), np.sin(h)
        x = x0 + np.concatenate([[0], np.cumsum(0.5 * (dx[1:] + dx[:-1]) * np.diff(ds))])
        y = y0 + np.concatenate([[0], np.cumsum(0.5 * (dy[1:] + dy[:-1]) * np.diff(ds))])
        return np.stack([ds, x, y, h], 1)[::4] if m > 8 else np.stack([ds, x, y, h], 1)
    # poly3 / paramPoly3: sample the parameter densely, then measure arc length
    m = n * 4
    if kind == "poly3":
        a, b, c, d = g["abcd"]
        # u runs along the local x axis; find the u that gives the stated length
        u = np.linspace(0, L * 1.5, m * 2)
        v = a + b * u + c * u * u + d * u ** 3
        du_, dv_ = np.ones_like(u), b + 2 * c * u + 3 * d * u * u
        U, V = u, v
    else:
        aU, bU, cU, dU, aV, bV, cV, dV = g["abcd"]
        pmax = 1.0 if g.get("pRange", "arcLength") == "normalized" else L
        p = np.linspace(0, pmax, m)
        U = aU + bU * p + cU * p * p + dU * p ** 3
        V = aV + bV * p + cV * p * p + dV * p ** 3
        du_, dv_ = bU + 2 * cU * p + 3 * dU * p * p, bV + 2 * cV * p + 3 * dV * p * p
    seg = np.hypot(np.diff(U), np.diff(V))
    arc = np.concatenate([[0], np.cumsum(seg)])
    keep = arc <= L + 1e-6
    U, V, arc, du_, dv_ = U[keep], V[keep], arc[keep], du_[keep], dv_[keep]
    c, s = math.cos(h0), math.sin(h0)
    x = x0 + U * c - V * s
    y = y0 + U * s + V * c
    h = h0 + np.arctan2(dv_, du_)
    out = np.stack([arc, x, y, h], 1)
    out = np.vstack([out, out[-1:]]) if len(out) < 2 else out
    return out


def _arc_end_mismatch(g: dict, samples: np.ndarray) -> float:
    """For poly3/paramPoly3: how much the curve's real arc length differs from @length."""
    return abs(float(samples[-1, 0]) - g["length"]) if g["kind"] in ("poly3", "paramPoly3") else 0.0


def read(path) -> Tuple[ET.Element, List[Road], List[Junction]]:
    root = ET.parse(path).getroot()
    roads = []
    for r in root.findall("road"):
        geoms = []
        pv = r.find("planView")
        for g in (pv.findall("geometry") if pv is not None else []):
            d = {"s": _num(g, "s"), "x": _num(g, "x"), "y": _num(g, "y"), "hdg": _num(g, "hdg"),
                 "length": _num(g, "length"), "kind": None}
            for kind in ("line", "arc", "spiral", "poly3", "paramPoly3"):
                e = g.find(kind)
                if e is None:
                    continue
                d["kind"] = kind
                if kind == "arc":
                    d["curvature"] = _num(e, "curvature")
                elif kind == "spiral":
                    d["curvStart"], d["curvEnd"] = _num(e, "curvStart"), _num(e, "curvEnd")
                elif kind == "poly3":
                    d["abcd"] = tuple(_num(e, k) for k in "abcd")
                elif kind == "paramPoly3":
                    d["abcd"] = tuple(_num(e, k) for k in ("aU", "bU", "cU", "dU", "aV", "bV", "cV", "dV"))
                    d["pRange"] = e.get("pRange", "arcLength")
                break
            geoms.append(d)
        lanes_e = r.find("lanes")
        sections = []
        for se in (lanes_e.findall("laneSection") if lanes_e is not None else []):
            lanes = {}
            for side in ("left", "center", "right"):
                pe = se.find(side)
                for le in (pe.findall("lane") if pe is not None else []):
                    lid = int(le.get("id"))
                    link = le.find("link")
                    sc = link.find("successor") if link is not None else None
                    pr = link.find("predecessor") if link is not None else None
                    lanes[lid] = Lane(lid, le.get("type", ""), _polys(le, "width", "sOffset"),
                                      int(sc.get("id")) if sc is not None else None,
                                      int(pr.get("id")) if pr is not None else None)
            sections.append(Section(_num(se, "s"), lanes))
        link = r.find("link")

        def _lk(tag):
            e = link.find(tag) if link is not None else None
            return None if e is None else {"type": e.get("elementType", "road"), "id": e.get("elementId"),
                                           "contact": e.get("contactPoint")}
        rd = Road(r.get("id"), r.get("name", ""), _num(r, "length"), r.get("junction", "-1"), geoms,
                  _polys(r.find("elevationProfile"), "elevation"),
                  _polys(r.find("lateralProfile"), "superelevation"),
                  _polys(lanes_e, "laneOffset") if lanes_e is not None else [], sections,
                  _lk("predecessor"), _lk("successor"))
        for i, sec in enumerate(rd.sections):
            sec.s_end = rd.sections[i + 1].s if i + 1 < len(rd.sections) else rd.length
        roads.append(rd)
    juncs = []
    for j in root.findall("junction"):
        cons = []
        for c in j.findall("connection"):
            cons.append({"id": c.get("id"), "incoming": c.get("incomingRoad"),
                         "connecting": c.get("connectingRoad"), "contact": c.get("contactPoint"),
                         "lanes": [(int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink")]})
        juncs.append(Junction(j.get("id"), cons))
    return root, roads, juncs


def reference(road: Road) -> np.ndarray:
    """(s, x, y, hdg) samples of the reference line."""
    if road.ref is None:
        parts = []
        for g in road.geoms:
            if g["kind"] is None or g["length"] <= 0:
                continue
            smp = _geom_samples(g)
            smp[:, 0] += g["s"]
            parts.append(smp)
        road.ref = np.vstack(parts) if parts else np.zeros((0, 4))
    return road.ref


def lane_bounds(road: Road, sec: Section, s: np.ndarray) -> Dict[int, Tuple[np.ndarray, np.ndarray]]:
    """For each lane id, its (inner t, outer t) at s (t positive to the left)."""
    t0 = _eval(road.lane_offset, s)
    out = {}
    for side in (1, -1):
        t = t0.copy()
        for lid in sec.ids(side):
            w = np.maximum(_eval(sec.lanes[lid].widths, s, base=sec.s), 0.0)
            out[lid] = (t.copy(), t + side * w)
            t = t + side * w
    return out


# --- checking -----------------------------------------------------------------------

class Report:
    def __init__(self):
        self.items: List[dict] = []

    def add(self, level: str, code: str, msg: str, road: Optional[str] = None):
        self.items.append({"level": level, "code": code, "road": road, "msg": msg})

    def counts(self) -> Dict[str, int]:
        c = {"error": 0, "warning": 0, "info": 0}
        for it in self.items:
            c[it["level"]] += 1
        return c


def _ascending(polys: Sequence[Poly], length: float, what: str, rid: str, rep: Report, base=0.0):
    if not polys:
        return
    ss = [base + p.s for p in polys]
    if any(b < a - 1e-9 for a, b in zip(ss, ss[1:])):
        rep.add("error", "profile_order", "%s records are not in increasing s" % what, rid)
    if abs(ss[0] - base) > 1e-6:
        rep.add("warning", "profile_start", "%s starts at s=%.3f, not at the start" % (what, ss[0]), rid)
    if ss[-1] > length + 1e-3:
        rep.add("error", "profile_range", "%s has a record beyond the road end (s=%.2f > %.2f)"
                % (what, ss[-1], length), rid)


def check_structure(root, roads: List[Road], juncs: List[Junction], rep: Report) -> None:
    if root.tag != "OpenDRIVE":
        rep.add("error", "root", "the root element is <%s>, not <OpenDRIVE>" % root.tag)
    hdr = root.find("header")
    if hdr is None:
        rep.add("error", "header", "no <header>")
    else:
        if hdr.get("revMajor") != "1":
            rep.add("error", "header", "revMajor is %s, expected 1" % hdr.get("revMajor"))
        geo = hdr.find("geoReference")
        if geo is None or not (geo.text or "").strip():
            rep.add("warning", "georef", "no geoReference: trajectories in GPS coordinates cannot be placed")
    if not roads:
        rep.add("error", "roads", "the file has no roads")
    ids = [r.id for r in roads]
    for dup in sorted({i for i in ids if ids.count(i) > 1}):
        rep.add("error", "road_id", "road id %s is used more than once" % dup)
    jids = [j.id for j in juncs]
    for dup in sorted({i for i in jids if jids.count(i) > 1}):
        rep.add("error", "junction_id", "junction id %s is used more than once" % dup)
    by_id = {r.id: r for r in roads}
    jby = {j.id: j for j in juncs}

    for r in roads:
        if r.length <= 0:
            rep.add("error", "road_length", "road length is %.3f" % r.length, r.id)
            continue
        if not r.geoms:
            rep.add("error", "geometry", "no planView geometry", r.id)
            continue
        tot = sum(g["length"] for g in r.geoms)
        if abs(tot - r.length) > max(1e-3, 1e-5 * r.length):
            rep.add("error", "road_length", "road length %.4f m differs from its geometry total %.4f m"
                    % (r.length, tot), r.id)
        s_exp = 0.0
        prev_end = None
        worst_gap, worst_hdg, worst_arc = 0.0, 0.0, 0.0
        for g in r.geoms:
            if g["kind"] is None:
                rep.add("error", "geometry", "a geometry record has no known shape", r.id)
                continue
            if abs(g["s"] - s_exp) > 1e-3:
                rep.add("error", "geometry_s", "geometry at s=%.4f should start at s=%.4f" % (g["s"], s_exp), r.id)
            s_exp = g["s"] + g["length"]
            smp = _geom_samples(g)
            worst_arc = max(worst_arc, _arc_end_mismatch(g, smp))
            if prev_end is not None:
                gap = math.hypot(smp[0, 1] - prev_end[1], smp[0, 2] - prev_end[2])
                dh = abs((smp[0, 3] - prev_end[3] + math.pi) % (2 * math.pi) - math.pi)
                worst_gap, worst_hdg = max(worst_gap, gap), max(worst_hdg, dh)
            prev_end = smp[-1]
        if worst_gap > GAP_ERR:
            rep.add("error", "geometry_gap", "pieces of the reference line are up to %.1f cm apart"
                    % (100 * worst_gap), r.id)
        elif worst_gap > GAP_WARN:
            rep.add("warning", "geometry_gap", "pieces of the reference line are up to %.1f cm apart"
                    % (100 * worst_gap), r.id)
        if worst_hdg > HDG_WARN:
            rep.add("warning", "geometry_kink", "the reference line has a %.2f deg kink" % math.degrees(worst_hdg), r.id)
        if worst_arc > 0.01:
            rep.add("warning", "geometry_arclength", "a curve's real length differs from its stated length "
                    "by %.1f cm, so positions along the road drift" % (100 * worst_arc), r.id)
        _ascending(r.elevation, r.length, "elevation", r.id, rep)
        _ascending(r.superelevation, r.length, "superelevation", r.id, rep)
        _ascending(r.lane_offset, r.length, "laneOffset", r.id, rep)
        if not r.elevation:
            rep.add("warning", "elevation", "no elevation profile: the road is flat at height 0", r.id)
        # lanes
        if not r.sections:
            rep.add("error", "lanes", "no laneSection", r.id)
            continue
        if abs(r.sections[0].s) > 1e-6:
            rep.add("error", "lanes", "the first laneSection starts at s=%.3f, not 0" % r.sections[0].s, r.id)
        for k, sec in enumerate(r.sections):
            if 0 not in sec.lanes:
                rep.add("error", "lane_ids", "laneSection %d has no center lane (id 0)" % k, r.id)
            for side, name in ((1, "left"), (-1, "right")):
                got = sorted(abs(i) for i in sec.lanes if i * side > 0)
                if got != list(range(1, len(got) + 1)):
                    rep.add("error", "lane_ids", "laneSection %d %s lane ids are %s, not 1..%d in order"
                            % (k, name, got, len(got)), r.id)
            ss = np.arange(sec.s, max(sec.s_end, sec.s + 1e-3), 0.5)
            for lid, ln in sec.lanes.items():
                if lid == 0:
                    continue
                if not ln.widths:
                    rep.add("error", "lane_width", "lane %d in laneSection %d has no width" % (lid, k), r.id)
                    continue
                w = _eval(ln.widths, ss, base=sec.s)
                if w.min() < -1e-3:
                    rep.add("error", "lane_width", "lane %d has a negative width (%.2f m) at s=%.1f"
                            % (lid, w.min(), ss[int(np.argmin(w))]), r.id)
            if not any(l.type in DRIVABLE for l in sec.lanes.values()):
                rep.add("warning", "no_driving_lane", "laneSection %d has no driving lane" % k, r.id)
        # links between roads
        for end, lk in (("predecessor", r.pred), ("successor", r.succ)):
            if lk is None:
                continue
            if lk["type"] == "road":
                if lk["id"] not in by_id:
                    rep.add("error", "link", "%s road %s does not exist" % (end, lk["id"]), r.id)
                elif lk["contact"] not in ("start", "end"):
                    rep.add("error", "link", "%s link to road %s has no contactPoint" % (end, lk["id"]), r.id)
                else:
                    _check_join(r, end, by_id[lk["id"]], lk["contact"], rep)
            elif lk["type"] == "junction" and lk["id"] not in jby:
                rep.add("error", "link", "%s junction %s does not exist" % (end, lk["id"]), r.id)
        if r.junction not in ("-1", None) and r.junction not in jby:
            rep.add("error", "junction", "road claims to be in junction %s, which does not exist" % r.junction, r.id)

    for j in juncs:
        if not j.connections:
            rep.add("warning", "junction", "junction %s has no connections" % j.id)
        for c in j.connections:
            inc, con = by_id.get(c["incoming"]), by_id.get(c["connecting"])
            if inc is None or con is None:
                rep.add("error", "junction", "junction %s connection %s names a missing road (%s -> %s)"
                        % (j.id, c["id"], c["incoming"], c["connecting"]))
                continue
            if con.junction != j.id:
                rep.add("error", "junction", "connecting road %s is not marked as part of junction %s"
                        % (con.id, j.id))
            if not c["lanes"]:
                rep.add("warning", "junction", "junction %s connection %s has no laneLink: no lane is "
                        "connected" % (j.id, c["id"]))
            for f, t in c["lanes"]:
                if not any(f in s_.lanes for s_ in inc.sections):
                    rep.add("error", "junction", "junction %s: lane %d does not exist on incoming road %s"
                            % (j.id, f, inc.id))
                if not any(t in s_.lanes for s_ in con.sections):
                    rep.add("error", "junction", "junction %s: lane %d does not exist on connecting road %s"
                            % (j.id, t, con.id))


def _end_state(road: Road, at: str):
    ref = reference(road)
    if not len(ref):
        return None
    row = ref[0] if at == "start" else ref[-1]
    return row


def _check_join(r: Road, end: str, other: Road, contact: str, rep: Report) -> None:
    a = _end_state(r, "start" if end == "predecessor" else "end")
    b = _end_state(other, contact)
    if a is None or b is None:
        return
    gap = math.hypot(a[1] - b[1], a[2] - b[2])
    if gap > GAP_ERR:
        rep.add("error", "link_gap", "the %s end is %.2f m away from road %s's %s"
                % ("start" if end == "predecessor" else "end", gap, other.id, contact), r.id)


def lane_width_stats(roads: List[Road], rep: Report) -> List[dict]:
    rows = []
    for r in roads:
        if r.junction not in ("-1", None):
            continue
        for k, sec in enumerate(r.sections):
            ss = np.arange(sec.s, max(sec.s_end, sec.s + 1e-3), 0.5)
            for lid, ln in sorted(sec.lanes.items()):
                if lid == 0 or ln.type not in DRIVABLE or not ln.widths:
                    continue
                w = _eval(ln.widths, ss, base=sec.s)
                rate = np.abs(np.diff(w)) / 0.5 if len(w) > 1 else np.zeros(1)
                row = {"road": r.id, "section": k, "lane": lid, "type": ln.type,
                       "median_m": float(np.median(w)), "min_m": float(w.min()), "max_m": float(w.max()),
                       "spread_m": float(w.max() - w.min()), "max_rate": float(rate.max())}
                rows.append(row)
                if row["min_m"] < LANE_W_MIN or row["max_m"] > LANE_W_MAX:
                    rep.add("warning", "lane_width_range", "driving lane %d is %.2f-%.2f m wide; normal lanes are "
                            "%.1f-%.1f m" % (lid, row["min_m"], row["max_m"], LANE_W_MIN, LANE_W_MAX), r.id)
                if row["spread_m"] > LANE_W_SPREAD:
                    rep.add("warning", "lane_width_steady", "driving lane %d changes width by %.2f m along the "
                            "road; real lanes keep a steady width" % (lid, row["spread_m"]), r.id)
                if row["max_rate"] > LANE_W_RATE:
                    rep.add("warning", "lane_width_wobble", "driving lane %d width changes by up to %.1f cm per "
                            "metre (wobbly lane edges)" % (lid, 100 * row["max_rate"]), r.id)
    return rows


def check_topology(roads: List[Road], juncs: List[Junction], rep: Report) -> dict:
    """Is the network connected? Counts roads with no links and driving lanes
    that end without a successor (dead ends)."""
    linked = set()
    for r in roads:
        for lk in (r.pred, r.succ):
            if lk is not None:
                linked.add(r.id)
                if lk["type"] == "road":
                    linked.add(lk["id"])
    for j in juncs:
        for c in j.connections:
            linked.update((c["incoming"], c["connecting"]))
    lone = [r.id for r in roads if r.id not in linked]
    # connected groups of roads
    parent = {r.id: r.id for r in roads}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        if a in parent and b in parent:
            parent[find(a)] = find(b)
    for r in roads:
        for lk in (r.pred, r.succ):
            if lk is not None and lk["type"] == "road":
                union(r.id, lk["id"])
    for j in juncs:
        for c in j.connections:
            union(c["incoming"], c["connecting"])
    groups = len({find(r.id) for r in roads})
    if len(roads) > 1 and lone:
        rep.add("warning", "unconnected", "%d of %d roads are not connected to any other road (%s): a vehicle "
                "cannot drive from them onto the rest of the map" % (len(lone), len(roads), ", ".join(lone[:10])))
    elif len(roads) == 1:
        rep.add("info", "unconnected", "the map is a single road with no junctions")
    if groups > 1:
        rep.add("warning", "network", "the map falls into %d separate pieces of road network" % groups)
    return {"roads": len(roads), "junctions": len(juncs), "unconnected_roads": lone, "network_pieces": groups}


# --- trajectory replay --------------------------------------------------------------

def read_trajectory(path) -> Dict[str, np.ndarray]:
    """CSV with columns x, y and optionally z, yaw_deg (or heading_deg) and t."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {"x": np.zeros(0), "y": np.zeros(0)}
    out = {}
    for k in ("x", "y", "z", "yaw_deg", "heading_deg", "t"):
        if k in rows[0] and rows[0][k] not in ("", None):
            out[k] = np.array([float(r[k]) if r[k] not in ("", None) else np.nan for r in rows])
    if "yaw_deg" not in out and "heading_deg" in out:
        out["yaw_deg"] = out["heading_deg"]
    return out


def locate(roads: List[Road], x: np.ndarray, y: np.ndarray, max_dist: float = 30.0):
    """Nearest reference-line position of each point: (road index, s, t, road heading)."""
    n = len(x)
    best_d = np.full(n, np.inf)
    ri = np.full(n, -1)
    S, T, H = np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan)
    P = np.stack([x, y], 1)
    for k, r in enumerate(roads):
        ref = reference(r)
        if len(ref) < 2:
            continue
        # nearest sample in chunks (keeps memory small), then refine on its segment
        for a in range(0, n, 2000):
            p = P[a:a + 2000]
            d2 = (p[:, None, 0] - ref[None, :, 1]) ** 2 + (p[:, None, 1] - ref[None, :, 2]) ** 2
            j = np.argmin(d2, 1)
            j = np.clip(j, 0, len(ref) - 2)
            seg = ref[j + 1, 1:3] - ref[j, 1:3]
            L2 = np.maximum((seg ** 2).sum(1), 1e-12)
            q = np.clip(((p - ref[j, 1:3]) * seg).sum(1) / L2, 0, 1)
            foot = ref[j, 1:3] + q[:, None] * seg
            d = np.hypot(*(p - foot).T)
            better = d < best_d[a:a + 2000]
            idx = np.nonzero(better)[0] + a
            best_d[idx] = d[better]
            ri[idx] = k
            S[idx] = (ref[j, 0] + q * (ref[j + 1, 0] - ref[j, 0]))[better]
            h = ref[j, 3][better]
            H[idx] = h
            rel = (p - foot)[better]
            # sign: left of the line is positive
            side = np.sign(np.cos(h) * rel[:, 1] - np.sin(h) * rel[:, 0])
            T[idx] = side * d[better]
    far = best_d > max_dist
    ri[far] = -1
    return ri, S, T, H


def replay(roads: List[Road], traj: Dict[str, np.ndarray], rep: Report, left_hand: bool = False) -> dict:
    x, y = traj["x"], traj["y"]
    n = len(x)
    if not n:
        return {"poses": 0}
    ri, S, T, H = locate(roads, x, y)
    lane = np.zeros(n, int)
    ltype = np.array([""] * n, dtype=object)
    centre_off = np.full(n, np.nan)
    z_road = np.full(n, np.nan)
    for i in range(n):
        if ri[i] < 0:
            continue
        r = roads[ri[i]]
        if S[i] < -0.5 or S[i] > r.length + 0.5:
            ri[i] = -1
            continue
        sec = r.section_at(S[i])
        b = lane_bounds(r, sec, np.array([S[i]]))
        for lid, (inner, outer) in b.items():
            lo, hi = sorted((inner[0], outer[0]))
            if lo <= T[i] <= hi and hi - lo > 1e-3:
                lane[i] = lid
                ltype[i] = sec.lanes[lid].type
                centre_off[i] = T[i] - 0.5 * (inner[0] + outer[0])
                break
        zr = _eval(r.elevation, np.array([S[i]]))[0]
        sup = _eval(r.superelevation, np.array([S[i]]))[0]
        z_road[i] = zr + math.tan(sup) * T[i] if r.elevation else np.nan
    on_road = ri >= 0
    in_drive = np.array([t in DRIVABLE for t in ltype])
    out = {"poses": int(n), "on_road_pct": 100.0 * on_road.mean(),
           "in_driving_lane_pct": 100.0 * in_drive.mean()}
    if in_drive.any():
        co = np.abs(centre_off[in_drive])
        out["lane_centre_offset_median_m"] = float(np.median(co))
        out["lane_centre_offset_p95_m"] = float(np.percentile(co, 95))
    # lane changes along the path (in driving lanes only)
    seq = [(int(ri[i]), int(lane[i])) for i in range(n) if in_drive[i]]
    changes = sum(1 for a, b in zip(seq, seq[1:]) if a != b and a[0] == b[0])
    out["lane_changes"] = int(changes)
    out["lanes_used"] = sorted({"%s:%d" % (roads[a].id, b) for a, b in seq})
    # direction: in right-hand traffic, right lanes (negative ids) run along +s
    if "yaw_deg" in traj and in_drive.any():
        yaw = np.radians(traj["yaw_deg"])
        along = np.where(lane < 0, H, H + math.pi) if not left_hand else np.where(lane > 0, H, H + math.pi)
        err = np.abs((yaw - along + math.pi) % (2 * math.pi) - math.pi)[in_drive]
        out["heading_error_median_deg"] = float(np.degrees(np.median(err)))
        out["wrong_way_pct"] = 100.0 * float((err > math.radians(90)).mean())
    if "z" in traj and np.isfinite(z_road).any():
        dz = (traj["z"] - z_road)[np.isfinite(z_road)]
        out["height_vs_road_median_m"] = float(np.median(dz))
        out["height_vs_road_spread_m"] = float(np.percentile(dz, 90) - np.percentile(dz, 10))
    # findings
    if out["on_road_pct"] < 99:
        rep.add("warning", "replay_off_road", "%.0f%% of the trajectory is outside every road"
                % (100 - out["on_road_pct"]))
    if out["in_driving_lane_pct"] < 99:
        rep.add("warning", "replay_lane", "%.0f%% of the trajectory is not inside a driving lane"
                % (100 - out["in_driving_lane_pct"]))
    if out.get("wrong_way_pct", 0) > 1:
        rep.add("warning", "replay_direction", "%.0f%% of the trajectory drives against its lane's direction"
                % out["wrong_way_pct"])
    if out.get("lane_centre_offset_p95_m", 0) > 0.8:
        rep.add("warning", "replay_centre", "the vehicle is up to %.2f m (95%%) from its lane centre"
                % out["lane_centre_offset_p95_m"])
    if out.get("height_vs_road_spread_m", 0) > 0.15:
        rep.add("warning", "replay_height", "the road height disagrees with the vehicle height by %.2f m "
                "(10-90%% spread)" % out["height_vs_road_spread_m"])
    return out


# --- entry point --------------------------------------------------------------------

def check(xodr_path, trajectories: Sequence = (), left_hand: bool = False) -> dict:
    """Run every check. ``trajectories``: CSV paths (x, y[, z, yaw_deg])."""
    rep = Report()
    try:
        root, roads, juncs = read(xodr_path)
    except (ET.ParseError, ValueError, TypeError) as e:
        rep.add("error", "parse", "the file could not be read: %s" % e)
        return {"file": str(xodr_path), "counts": rep.counts(), "findings": rep.items}
    check_structure(root, roads, juncs, rep)
    widths = lane_width_stats(roads, rep)
    topo = check_topology(roads, juncs, rep)
    replays = []
    for tp in trajectories:
        tr = read_trajectory(tp)
        rr = replay(roads, tr, rep, left_hand)
        rr["trajectory"] = Path(tp).name
        replays.append(rr)
    return {"file": str(xodr_path), "counts": rep.counts(), "findings": rep.items,
            "lane_widths": widths, "topology": topo, "replay": replays}


def text_report(res: dict) -> str:
    c = res["counts"]
    lines = ["%s: %d errors, %d warnings" % (res["file"], c["error"], c["warning"])]
    for f in res["findings"]:
        lines.append("  %-7s %s%s" % (f["level"].upper(), "road %s: " % f["road"] if f["road"] else "", f["msg"]))
    for r in res.get("replay") or []:
        lines.append("  replay %s: %d poses, %.0f%% on a road, %.0f%% in a driving lane, %d lane changes"
                     % (r.get("trajectory", ""), r["poses"], r.get("on_road_pct", 0),
                        r.get("in_driving_lane_pct", 0), r.get("lane_changes", 0)))
        if "lane_centre_offset_median_m" in r:
            lines.append("         lane-centre offset %.2f m (95%%: %.2f m)" % (
                r["lane_centre_offset_median_m"], r["lane_centre_offset_p95_m"]))
    return "\n".join(lines)
