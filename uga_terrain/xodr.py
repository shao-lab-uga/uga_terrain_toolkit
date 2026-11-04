"""Write roads as OpenDRIVE 1.6 (.xodr) with elevation, banking, lanes, sidewalks.

Each road's reference line is the driven path, fitted with ``paramPoly3``
pieces. Along it:

* ``elevationProfile``: road height (cubic pieces through the measured profile);
* ``lateralProfile/superelevation``: banking, positive when the road falls to
  the right (left side higher), as OpenDRIVE defines it;
* ``laneOffset`` + driving lanes: the measured paved width split into lanes of
  about ``lane_width``; the drive is taken to be in a right-hand lane;
* per side, outward: ``border`` (curb), ``border`` (verge, when there is one)
  and ``sidewalk`` lanes, each with ``height`` records for the curb height.

Roads are not joined into junctions.
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
        q = np.linspace(0, 1, 401)
        sp = np.hypot(bu + 2 * cu * q + 3 * du * q * q, bv + 2 * cv * q + 3 * dv * q * q)
        length = float(np.sum(0.5 * (sp[1:] + sp[:-1]) * np.diff(q)))
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


def road_element(root, road: Road, cfg: dict, z_offset: float = 0.0, road_name: str = ""):
    xc = cfg.get("xodr") or {}
    rc = cfg.get("roads") or {}
    seg = float(xc.get("segment_len") or 20.0)
    step = float(xc.get("profile_step") or 5.0)
    lane_w = float(rc.get("lane_width") or 3.5)
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
    ET.SubElement(r, "type", s="0", type="town")
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

    road_l = np.maximum(at("road_l", 1.75), 0.5)
    road_r = np.maximum(at("road_r", 1.75), 0.5)
    total = road_l + road_r
    n = max(1, int(round(float(np.median(total)) / lane_w)))
    n_r = max(1, int(math.ceil(n / 2.0)))
    n_l = n - n_r
    w_lane = total / n
    offset = road_l - n_l * w_lane          # centre lane position, left of the reference
    lanes = ET.SubElement(r, "lanes")
    for (s0, a, b, c, dd) in linear(knots, offset):
        ET.SubElement(lanes, "laneOffset", s=_f(s0), a=_f(a), b=_f(b), c=_f(c), d=_f(dd))
    sec = ET.SubElement(lanes, "laneSection", s="0")
    sides = {}
    for side, sign, n_side in (("l", 1, n_l), ("r", -1, n_r)):
        parent = ET.SubElement(sec, "left" if side == "l" else "right")
        sides[side] = parent
        lid = 0
        for k in range(n_side):
            lid += sign
            mark = ("solid", "white") if k == n_side - 1 else ("broken", "white")
            _lane(parent, lid, "driving", knots, w_lane, mark=mark)
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
    ET.SubElement(c_lane, "roadMark", sOffset="0", type="solid" if n_l else "none",
                  weight="standard", color="yellow" if n_l else "standard", width="0.12")
    # put <center> between <left> and <right>
    sec.remove(centre)
    sec.insert(1, centre)
    return r


def write_xodr(roads: Sequence[Road], origin_lla: Tuple[float, float, float], cfg: dict,
               path, name: str = "uga_terrain", road_names: Optional[dict] = None) -> dict:
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
        road_element(root, road, cfg, z_off, (road_names or {}).get(road.id, ""))
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode")
    xml = xml.replace("<geoReference>+proj", "<geoReference><![CDATA[+proj").replace(
        "+no_defs</geoReference>", "+no_defs]]></geoReference>")
    with open(path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write(xml + "\n")
    return {"roads": len(roads), "length_m": float(sum(r.length for r in roads))}
