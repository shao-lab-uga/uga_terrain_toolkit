"""Write roads as OpenDRIVE 1.6 (.xodr) with elevation, banking, lanes, sidewalks.

Each road's reference line is the centre line between the two directions
(from the lane model in ``lanes.py``; the driven path when there is none),
fitted with ``paramPoly3`` pieces. Along it:

* ``elevationProfile``: road height (cubic pieces through the measured profile);
* ``lateralProfile/superelevation``: banking, positive when the road falls to
  the right (left side higher), as OpenDRIVE defines it;
* driving lanes and shoulders from the lane model: painted lines found in the
  lidar intensity, steady widths; without a lane model the paved width is
  split into lanes of about ``lane_width`` (with a ``laneOffset``);
* per side, outward: ``border`` (curb), ``border`` (verge, when there is one)
  and ``sidewalk`` lanes, each with ``height`` records for the curb height.

Roads that meet at an intersection are joined by a junction with connecting
roads and lane-to-lane links (see ``junctions.add_to_xodr``).
"""
from __future__ import annotations

import datetime as _dt
import math
import xml.etree.ElementTree as ET
from typing import List, Optional, Sequence, Tuple

import numpy as np

from uga_terrain.roads import Road


def _f(v: float) -> str:
    return "%.10g" % float(v)


# --- reference line -----------------------------------------------------------------

def fit_param_poly3(x: np.ndarray, y: np.ndarray, heading: np.ndarray, seg_len: float):
    """Split a smooth path into paramPoly3 pieces (pRange normalized).

    Each piece is the cubic Hermite curve between two path points with the
    path's heading at both ends, so pieces join with no gap or kink and the
    line cannot drift away from the path.
    Returns [(s, x, y, hdg, length, (aU,bU,cU,dU), (aV,bV,cV,dV))].
    """
    d = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0.0], np.cumsum(d)])
    L = s[-1]
    n_seg = max(1, int(round(L / seg_len)))
    edges = np.linspace(0.0, L, n_seg + 1)
    hu = np.unwrap(np.asarray(heading, float))
    bx, by = np.interp(edges, s, x), np.interp(edges, s, y)
    bh = np.interp(edges, s, hu)
    out = []
    s_road = 0.0
    for i in range(n_seg):
        cx, cy, ch = float(bx[i]), float(by[i]), float(bh[i])
        c, sn = math.cos(ch), math.sin(ch)
        dx, dy = bx[i + 1] - cx, by[i + 1] - cy
        ue, ve = dx * c + dy * sn, -dx * sn + dy * c
        dh = float(bh[i + 1] - ch)
        k = math.hypot(ue, ve)
        bu, bv = k, 0.0
        cu = 3 * ue - 2 * k - k * math.cos(dh)
        du = -2 * ue + k + k * math.cos(dh)
        cv = 3 * ve - k * math.sin(dh)
        dv = -2 * ve + k * math.sin(dh)
        q = np.linspace(0, 1, 4001)
        length = float(np.sum(np.hypot(np.diff(bu * q + cu * q * q + du * q ** 3),
                                       np.diff(bv * q + cv * q * q + dv * q ** 3))))
        out.append((s_road, cx, cy, ch, length, (0.0, bu, cu, du), (0.0, bv, cv, dv)))
        s_road += length
    return out


def eval_param_poly3(geoms, n_per: int = 50) -> np.ndarray:
    pts = []
    for (_s, x0, y0, h, _L, U, V) in geoms:
        q = np.linspace(0, 1, n_per)
        u = U[0] + U[1] * q + U[2] * q * q + U[3] * q ** 3
        v = V[0] + V[1] * q + V[2] * q * q + V[3] * q ** 3
        c, s = math.cos(h), math.sin(h)
        pts.append(np.stack([x0 + u * c - v * s, y0 + u * s + v * c], 1))
    return np.concatenate(pts) if pts else np.zeros((0, 2))


# --- profiles ----------------------------------------------------------------------

def _clean(v: np.ndarray, fill: float = 0.0) -> np.ndarray:
    v = np.asarray(v, float).copy()
    ok = np.isfinite(v)
    if not ok.any():
        return np.full_like(v, fill)
    idx = np.arange(len(v))
    v[~ok] = np.interp(idx[~ok], idx[ok], v[ok])
    return v


def hermite(s: np.ndarray, z: np.ndarray, dz: np.ndarray) -> List[Tuple[float, ...]]:
    """Cubic pieces (s, a, b, c, d) through (s, z) with slopes dz."""
    out = []
    for i in range(len(s) - 1):
        L = s[i + 1] - s[i]
        if L <= 1e-9:
            continue
        z0, z1, m0, m1 = z[i], z[i + 1], dz[i], dz[i + 1]
        c = (3 * (z1 - z0) / L - 2 * m0 - m1) / L
        d = (m0 + m1 - 2 * (z1 - z0) / L) / (L * L)
        out.append((s[i], z0, m0, c, d))
    if len(s):
        out.append((s[-1], z[-1], dz[-1], 0.0, 0.0))
    return out


def linear(s: np.ndarray, v: np.ndarray) -> List[Tuple[float, ...]]:
    out = []
    for i in range(len(s) - 1):
        L = s[i + 1] - s[i]
        if L > 1e-9:
            out.append((s[i], v[i], (v[i + 1] - v[i]) / L, 0.0, 0.0))
    if len(s):
        out.append((s[-1], v[-1], 0.0, 0.0, 0.0))
    return out


# --- lanes ----------------------------------------------------------------------

def _lane(parent, lid: int, ltype: str, s_knots, widths, heights=None, mark=None):
    lane = ET.SubElement(parent, "lane", id=str(lid), type=ltype, level="false")
    ET.SubElement(lane, "link")
    for (s0, a, b, c, d) in linear(s_knots, widths):
        ET.SubElement(lane, "width", sOffset=_f(s0), a=_f(a), b=_f(b), c=_f(c), d=_f(d))
    if mark:
        ET.SubElement(lane, "roadMark", sOffset="0", type=mark[0], weight="standard",
                      color=mark[1], width="0.12", laneChange="both")
    if heights is not None:
        last = None
        for s0, (hi, ho) in zip(s_knots, heights):
            cur = (round(float(hi), 3), round(float(ho), 3))
            if cur != last:
                ET.SubElement(lane, "height", sOffset=_f(s0), inner=_f(cur[0]), outer=_f(cur[1]))
                last = cur
    return lane


def _present(v: np.ndarray, frac: float = 0.3) -> bool:
    v = np.asarray(v, float)
    return bool(np.mean(np.nan_to_num(v) > 0) >= frac)


def on_centre_line(road: Road) -> Road:
    """The same road with its reference line moved onto the lane model's centre
    line (the boundary between the two directions); lateral values follow."""
    lm = road.meta.get("lanes") if road.meta else None
    if not lm or ("lb%d" % lm["centre"]) not in road.data:
        return road
    d = road.data
    tc = _clean(d["lb%d" % lm["centre"]])
    if np.max(np.abs(tc)) < 1e-3:
        return road
    nx, ny = -np.sin(road.heading), np.cos(road.heading)
    x, y = road.x + tc * nx, road.y + tc * ny
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    h = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    nd = dict(d)
    for k in d:
        if k.startswith("lb"):
            nd[k] = d[k] - tc
    if "road_l" in d:
        nd["road_l"] = d["road_l"] - tc
    if "road_r" in d:
        nd["road_r"] = d["road_r"] + tc
    if "z" in d:
        nd["z"] = d["z"] + np.tan(np.radians(np.nan_to_num(d.get("bank_deg", np.zeros_like(tc))))) * tc
    return Road(road.id, road.source, s + road.s[0], x, y, h, nd, dict(road.meta))


ROAD_TYPES = ("town", "rural", "motorway", "lowSpeed", "pedestrian", "bicycle")
LANE_TYPES = ("driving", "shoulder", "parking", "biking", "sidewalk", "border", "restricted", "median", "none")


def road_details(cfg: dict, edits: Optional[dict], road_id) -> dict:
    """Type, speed limit, one-way and lane changes for one road: the config's
    defaults with the map's own edits on top."""
    d = dict((cfg.get("xodr") or {}).get("road_defaults") or {})
    d.setdefault("type", "town")
    d.setdefault("speed", None)
    d.setdefault("unit", "mph")
    d.setdefault("one_way", "no")
    d.update(((edits or {}).get("roads") or {}).get(str(road_id)) or {})
    return d


def _type_element(r, det: dict):
    t = ET.SubElement(r, "type", s="0", type=det.get("type") if det.get("type") in ROAD_TYPES else "town")
    if det.get("speed"):
        unit = det.get("unit") if det.get("unit") in ("mph", "km/h", "m/s") else "mph"
        ET.SubElement(t, "speed", max=_f(float(det["speed"])), unit=unit)
    return t


def _one_way(per_side: dict, offset: np.ndarray, how: str):
    """All driving lanes in one direction: along the road ("forward", right
    lanes) or against it ("backward", left lanes). The centre line moves to
    the edge of the lanes, so no lane changes place on the ground."""
    if how == "forward" and per_side["l"]:
        moved = per_side["l"]
        per_side["r"] = list(reversed(moved)) + per_side["r"]
        offset = offset + sum(w for _, w, _ in moved)
        per_side["l"] = []
    elif how == "backward" and per_side["r"]:
        moved = per_side["r"]
        per_side["l"] = list(reversed(moved)) + per_side["l"]
        offset = offset - sum(w for _, w, _ in moved)
        per_side["r"] = []
    return per_side, offset


def road_element(root, road: Road, cfg: dict, z_offset: float = 0.0, road_name: str = "",
                 details: Optional[dict] = None):
    xc = cfg.get("xodr") or {}
    rc = cfg.get("roads") or {}
    seg = float(xc.get("segment_len") or 20.0)
    step = float(xc.get("profile_step") or 5.0)
    lane_w = float(rc.get("lane_width") or 3.5)
    road = on_centre_line(road)
    d = road.data
    geoms = fit_param_poly3(road.x, road.y, road.heading, seg)
    Lg = sum(g[4] for g in geoms)
    scale = Lg / max(road.length, 1e-9)
    s_st = (road.s - road.s[0]) * scale
    knots = np.unique(np.concatenate([np.arange(0.0, Lg, step), [Lg]]))

    def at(name, fill=0.0):
        return np.interp(knots, s_st, _clean(d[name], fill))

    r = ET.SubElement(root, "road", name=road_name or "road_%d" % road.id, length=_f(Lg),
                      id=str(road.id), junction="-1")
    ET.SubElement(r, "link")
    det = details or road_details(cfg, None, road.id)
    _type_element(r, det)
    pv = ET.SubElement(r, "planView")
    for (s0, x0, y0, h, L, U, V) in geoms:
        g = ET.SubElement(pv, "geometry", s=_f(s0), x=_f(x0), y=_f(y0), hdg=_f(h), length=_f(L))
        ET.SubElement(g, "paramPoly3", aU=_f(U[0]), bU=_f(U[1]), cU=_f(U[2]), dU=_f(U[3]),
                      aV=_f(V[0]), bV=_f(V[1]), cV=_f(V[2]), dV=_f(V[3]), pRange="normalized")

    z = at("z") + z_offset
    dz = at("grade_pct") / 100.0
    ep = ET.SubElement(r, "elevationProfile")
    for (s0, a, b, c, dd) in hermite(knots, z, dz):
        ET.SubElement(ep, "elevation", s=_f(s0), a=_f(a), b=_f(b), c=_f(c), d=_f(dd))
    lp = ET.SubElement(r, "lateralProfile")
    sup = np.radians(at("bank_deg"))
    for (s0, a, b, c, dd) in linear(knots, sup):
        ET.SubElement(lp, "superelevation", s=_f(s0), a=_f(a), b=_f(b), c=_f(c), d=_f(dd))

    lm = road.meta.get("lanes") if road.meta else None
    nb = len([k for k in d if k.startswith("lb")])
    lanes = ET.SubElement(r, "lanes")
    if lm and nb >= 2 and nb == len(lm["types"]) + 1:
        # lanes from the lane model; the reference line is its centre line
        c = int(lm["centre"])
        bounds = [at("lb%d" % k) for k in range(nb)]
        offset = bounds[c]
        mark_of = {"solid": ("solid", "white"), "broken": ("broken", "white")}
        per_side = {"r": [(lm["types"][k], bounds[k + 1] - bounds[k], mark_of.get(lm["marks"][k]))
                          for k in range(c - 1, -1, -1)],
                    "l": [(lm["types"][k], bounds[k + 1] - bounds[k], mark_of.get(lm["marks"][k + 1]))
                          for k in range(c, nb - 1)]}
        centre_mark = lm["marks"][c] if lm["marks"][c] in ("solid", "broken") else None
        n_l = len(per_side["l"])
    else:
        road_l = np.maximum(at("road_l", 1.75), 0.5)
        road_r = np.maximum(at("road_r", 1.75), 0.5)
        total = road_l + road_r
        n = max(1, int(round(float(np.median(total)) / lane_w)))
        n_r = max(1, int(math.ceil(n / 2.0)))
        n_l = n - n_r
        w_lane = total / n
        offset = road_l - n_l * w_lane          # centre lane position, left of the reference
        per_side = {side: [("driving", w_lane, ("solid", "white") if k == n_side - 1 else ("broken", "white"))
                           for k in range(n_side)] for side, n_side in (("l", n_l), ("r", n_r))}
        centre_mark = "solid" if n_l else None
    if det.get("one_way") in ("forward", "backward"):
        per_side, offset = _one_way(per_side, offset, det["one_way"])
        centre_mark = None
        n_l = len(per_side["l"])
    lane_edits = det.get("lanes") or {}
    if np.max(np.abs(offset)) > 1e-3:
        for (s0, a, b, c_, dd) in linear(knots, offset):
            ET.SubElement(lanes, "laneOffset", s=_f(s0), a=_f(a), b=_f(b), c=_f(c_), d=_f(dd))
    sec = ET.SubElement(lanes, "laneSection", s="0")
    sides = {}
    for side, sign in (("l", 1), ("r", -1)):
        parent = ET.SubElement(sec, "left" if side == "l" else "right")
        sides[side] = parent
        lid = 0
        for ltype, widths, mark in per_side[side]:
            lid += sign
            ed = lane_edits.get(str(lid)) or {}
            if ed.get("type") in LANE_TYPES:
                ltype = ed["type"]
            if ed.get("width") not in (None, ""):
                widths = np.full(len(knots), float(ed["width"]))
            _lane(parent, lid, ltype, knots, np.maximum(widths, 0.0), mark=mark)
        curb = np.abs(at("curb_h_" + side))
        verge = at("verge_" + side)
        walk = at("walk_" + side)
        walk_h = np.nan_to_num(at("walk_h_" + side, 0.0))
        if _present(curb):
            lid += sign
            _lane(parent, lid, "border", knots, np.where(curb > 0, 0.15, 0.0),
                  heights=list(zip(np.zeros_like(curb), curb)))
        if _present(verge):
            lid += sign
            _lane(parent, lid, "border", knots, verge, heights=list(zip(curb, curb)))
        if _present(walk):
            lid += sign
            _lane(parent, lid, "sidewalk", knots, walk, heights=list(zip(walk_h, walk_h)))
    # OpenDRIVE lists left lanes from the outside in
    left = sides["l"]
    items = list(left)
    for it in items:
        left.remove(it)
    for it in sorted(items, key=lambda e: -int(e.get("id"))):
        left.append(it)
    centre = ET.SubElement(sec, "center")
    c_lane = ET.SubElement(centre, "lane", id="0", type="none", level="false")
    ET.SubElement(c_lane, "roadMark", sOffset="0", type=centre_mark or "none",
                  weight="standard", color="yellow" if centre_mark else "standard", width="0.12")
    # put <center> between <left> and <right>
    sec.remove(centre)
    sec.insert(1, centre)
    add_markings(r, road, geoms)
    add_signal_structures(r, road, geoms)
    return r


def add_markings(r, road: Road, geoms) -> int:
    """Stop lines and crosswalks (``road.meta["markings"]``, world polygons) as
    <object> records placed on the road's reference line."""
    marks = (road.meta or {}).get("markings") or []
    if not marks:
        return 0
    n_per = 200
    pts = eval_param_poly3(geoms, n_per=n_per)
    s_pts = np.concatenate([np.linspace(g[0], g[0] + g[4], n_per) for g in geoms])
    hd = np.unwrap(np.arctan2(np.gradient(pts[:, 1]), np.gradient(pts[:, 0])))
    L = float(sum(g[4] for g in geoms))
    objs = ET.SubElement(r, "objects")
    for k, m in enumerate(marks):
        P = np.asarray(m["polygon"], float)
        c = P.mean(0)
        i = int(np.argmin(np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])))
        s0, h = float(np.clip(s_pts[i], 0.0, L)), float(hd[i])
        nx, ny = -math.sin(h), math.cos(h)
        t0 = float((c[0] - pts[i, 0]) * nx + (c[1] - pts[i, 1]) * ny)
        # object frame: at (s, t), u along the road
        ox, oy = pts[i, 0] + t0 * nx, pts[i, 1] + t0 * ny
        uv = [((px - ox) * math.cos(h) + (py - oy) * math.sin(h),
               -(px - ox) * math.sin(h) + (py - oy) * math.cos(h)) for px, py in P]
        us, vs = [a for a, _ in uv], [b for _, b in uv]
        crosswalk = m["kind"] == "crosswalk"
        o = ET.SubElement(objs, "object", id=str(road.id * 100 + k),
                          type="crosswalk" if crosswalk else "roadMark",
                          subtype="" if crosswalk else "stopLine",
                          name="crosswalk" if crosswalk else "stop line",
                          s=_f(s0), t=_f(t0), zOffset="0", hdg="0", pitch="0", roll="0",
                          orientation="none", length=_f(max(us) - min(us)), width=_f(max(vs) - min(vs)),
                          height="0", validLength="0")
        if not o.get("subtype"):
            del o.attrib["subtype"]
        ol = ET.SubElement(ET.SubElement(o, "outlines"), "outline", id="0", closed="true",
                           fillType="paint", outer="true", laneType="driving")
        for a, b in uv:
            ET.SubElement(ol, "cornerLocal", u=_f(a), v=_f(b), z="0", height="0")
    return len(marks)


def _ref(geoms, n_per=200):
    pts = eval_param_poly3(geoms, n_per=n_per)
    s_pts = np.concatenate([np.linspace(g[0], g[0] + g[4], n_per) for g in geoms])
    hd = np.unwrap(np.arctan2(np.gradient(pts[:, 1]), np.gradient(pts[:, 0])))
    return pts, s_pts, hd, float(sum(g[4] for g in geoms))


def add_signal_structures(r, road: Road, geoms) -> int:
    """Signal poles (``road.meta["poles"]``: x, y, height, radius) as <object
    type="pole">, and traffic signal heads (``road.meta["signal_heads"]``: x, y,
    bottom, top, width, contact) as <signal type="1000001"> (a traffic light)
    for the traffic coming in at that end of the road. The signal stands at
    the span's corner pole (zOffset 0), as simulators draw a whole pole there;
    the measured heads are kept in <userData>."""
    poles = (road.meta or {}).get("poles") or []
    heads = (road.meta or {}).get("signal_heads") or []
    if not poles and not heads:
        return 0
    pts, s_pts, hd, L = _ref(geoms)
    if poles:
        objs = r.find("objects")
        if objs is None:
            objs = ET.SubElement(r, "objects")
        for k, q in enumerate(poles):
            i = int(np.argmin(np.hypot(pts[:, 0] - q["x"], pts[:, 1] - q["y"])))
            h = float(hd[i])
            t0 = (q["x"] - pts[i, 0]) * -math.sin(h) + (q["y"] - pts[i, 1]) * math.cos(h)
            ET.SubElement(objs, "object", id=str(road.id * 100 + 50 + k), type="pole", name="signal pole", s=_f(float(np.clip(s_pts[i], 0, L))), t=_f(t0),
                          zOffset="0", hdg="0", pitch="0", roll="0", orientation="none",
                          height=_f(q["height"]), radius=_f(q["radius"]), validLength="0")
    if heads:
        sigs = ET.SubElement(r, "signals")
        # Simulators (CARLA) do not draw the measured head: they stand their own
        # traffic-light model (pole, arm, light) on the ground at the signal's
        # s/t and lift it by zOffset. So each light is written where its pole
        # stands - the corner pole of the span on the right of the incoming
        # traffic - with zOffset 0, one per pole and approach. The measured
        # heads (position, height, size) go along in <userData>.
        groups = {}
        for q in heads:
            at_end = q.get("contact") == "end"
            i = len(pts) - 1 if at_end else 0
            h = float(hd[i])
            u = np.array([math.cos(h), math.sin(h)])
            n = np.array([-u[1], u[0]])
            travel = u if at_end else -u                    # direction of the traffic coming in
            right = np.array([travel[1], -travel[0]])
            cand = []
            for p_ in q.get("span_poles") or []:
                rel = np.array([p_["x"], p_["y"]]) - pts[i]
                if float(rel @ right) > 0.5:
                    cand.append((float(np.hypot(*rel)), p_))
            if cand:
                p_ = min(cand, key=lambda c: c[0])[1]
                key = (at_end, p_.get("id"), round(p_["x"], 1), round(p_["y"], 1))
                base = (p_["x"], p_["y"], p_.get("ground_z"))
            else:                                           # no pole found: the head's own spot
                key = (at_end, None, round(q["x"], 1), round(q["y"], 1))
                base = (q["x"], q["y"], q.get("ground_z", 0.0))
            groups.setdefault(key, {"at_end": at_end, "i": i, "h": h, "n": n, "base": base, "heads": []})["heads"].append(q)
        for k, g in enumerate(groups.values()):
            at_end, i, h, n = g["at_end"], g["i"], g["h"], g["n"]
            bx, by, bz = g["base"]
            if bz is None:
                bz = g["heads"][0].get("ground_z", 0.0)
            t0 = float((np.array([bx, by]) - pts[i]) @ n)
            # CARLA turns its model by the road heading plus hOffset (it ignores
            # "orientation"); at hOffset 0 the arm points along +s. A quarter
            # turn towards the road's centre puts the arm over the lanes, and the
            # lights then face the traffic coming in on the pole's side.
            h_off = -math.copysign(math.pi / 2, t0) if abs(t0) > 1e-6 else (math.pi / 2 if at_end else -math.pi / 2)
            face = h + h_off
            top = max(q["top"] for q in g["heads"])
            wid = max(max(q["width"], 0.3) for q in g["heads"])
            sg = ET.SubElement(sigs, "signal", s=_f(L if at_end else 0.0), t=_f(t0),
                               id=str(road.id * 100 + 70 + k), name="traffic light",
                               dynamic="yes", orientation="+" if at_end else "-",
                               zOffset="0", country="OpenDRIVE", type="1000001", subtype="-1",
                               value="-1", height=_f(top), width=_f(wid),
                               hOffset=_f(h_off), pitch="0", roll="0")
            ET.SubElement(sg, "positionInertial", x=_f(bx), y=_f(by), z=_f(bz),
                          hdg=_f(math.atan2(math.sin(face), math.cos(face))), pitch="0", roll="0")
            for j, q in enumerate(g["heads"]):
                ET.SubElement(sg, "userData", code="measured_head_%d" % j,
                              value="x=%s y=%s z_bottom=%s z_top=%s width=%s" % (
                                  _f(q["x"]), _f(q["y"]), _f(q.get("ground_z", 0.0) + q["bottom"]),
                                  _f(q.get("ground_z", 0.0) + q["top"]), _f(q["width"])))
    return len(poles) + len(heads)


def write_xodr(roads: Sequence[Road], origin_lla: Tuple[float, float, float], cfg: dict,
               path, name: str = "uga_terrain", road_names: Optional[dict] = None,
               junctions: Sequence = (), ground=None, edits: Optional[dict] = None) -> dict:
    """``edits``: the map's own changes (see ``road_details``), plus
    "removed_roads" (ids) and "removed_turns" ("road:lane>road:lane")."""
    removed = {int(v) for v in (edits or {}).get("removed_roads") or []}
    roads = [r for r in roads if r.id not in removed]
    xc = cfg.get("xodr") or {}
    z_off = 0.0 if xc.get("relative_elevation", True) else float(origin_lla[2])
    root = ET.Element("OpenDRIVE")
    xs = np.concatenate([r.x for r in roads]) if roads else np.zeros(1)
    ys = np.concatenate([r.y for r in roads]) if roads else np.zeros(1)
    hdr = ET.SubElement(root, "header", revMajor="1", revMinor="6", name=name, version="1",
                        date=_dt.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
                        north=_f(ys.max() + 30), south=_f(ys.min() - 30),
                        east=_f(xs.max() + 30), west=_f(xs.min() - 30),
                        vendor="UGA terrain toolkit")
    geo = ET.SubElement(hdr, "geoReference")
    geo.text = ("+proj=tmerc +lat_0=%.10f +lon_0=%.10f +k=1 +x_0=0 +y_0=0 +ellps=WGS84 "
                "+units=m +no_defs" % (origin_lla[0], origin_lla[1]))
    ET.SubElement(hdr, "userData", code="origin_altitude_m", value=_f(origin_lla[2]))
    for road in roads:
        road_element(root, road, cfg, z_off, (road_names or {}).get(road.id, ""),
                     details=road_details(cfg, edits, road.id))
    jinfo = {"junctions": [], "connecting_roads": 0}
    if junctions:
        from uga_terrain import junctions as junctions_mod
        ends = {r.id: {"start": r.meta.get("junction_start"), "end": r.meta.get("junction_end")}
                for r in roads}
        jinfo = junctions_mod.add_to_xodr(root, junctions, ends, cfg,
                                          next_id=max([r.id for r in roads] + [0]), ground=ground, z_off=z_off,
                                          removed_turns=set((edits or {}).get("removed_turns") or []),
                                          details=lambda rid: road_details(cfg, edits, rid))
    # OpenDRIVE wants junctions after all roads
    for je in root.findall("junction"):
        root.remove(je)
        root.append(je)
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode")
    xml = xml.replace("<geoReference>+proj", "<geoReference><![CDATA[+proj").replace(
        "+no_defs</geoReference>", "+no_defs]]></geoReference>")
    with open(path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write(xml + "\n")
    return {"roads": len(roads), "length_m": float(sum(r.length for r in roads)),
            "junctions": jinfo["junctions"], "connecting_roads": jinfo["connecting_roads"]}
