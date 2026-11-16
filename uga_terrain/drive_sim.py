"""Drive test: a simulated car drives every route an OpenDRIVE map allows.

Each route starts on a driving lane at the edge of the map and follows lane
links and junction connections until it leaves the map. A kinematic bicycle
model (1.9 m wide, 2.8 m wheelbase, 35 deg steering lock; 25 mph on roads,
15 mph in junctions) steers along the lane centre with pure pursuit. Found:
curves tighter than a car can turn, places the car cannot follow, lanes the
car does not fit in or that are wide enough for two, gaps, kinks and steps in
the road surface where lanes join, steep grades and cross slopes.
"""
import json
import math
import sys
from collections import defaultdict

import numpy as np

from uga_terrain import xodr_check as X  # noqa: E402

CAR_W, CAR_L, WB = 1.9, 4.6, 2.8
MAX_STEER = math.radians(35)
R_MIN = WB / math.tan(MAX_STEER)          # ~4.0 m
V_ROAD, V_TURN = 11.2, 6.7               # 25 mph, 15 mph
A_LAT_MAX = 3.0                          # m/s2, comfortable
DT = 0.05
STEP = 0.25


def lane_dir(lid, contact_dir=+1):
    return -1 if lid > 0 else +1         # right-hand traffic: right lanes (negative) drive with s


def centre_samples(r, sec, lid, s0, s1):
    ss = np.arange(s0, s1, STEP)
    ss = np.append(ss, s1)
    ref = X.reference(r)
    x = np.interp(ss, ref[:, 0], ref[:, 1])
    y = np.interp(ss, ref[:, 0], ref[:, 2])
    h = np.interp(ss, ref[:, 0], np.unwrap(ref[:, 3]))
    b = X.lane_bounds(r, sec, ss)
    ti, to = b[lid]
    t = 0.5 * (ti + to)
    w = np.abs(to - ti)
    z = X._eval(r.elevation, ss) + t * np.tan(X._eval(r.superelevation, ss))
    cross = X._eval(r.superelevation, ss)
    return dict(s=ss, x=x - t * np.sin(h), y=y + t * np.cos(h), z=z, w=w, cross=cross)


class Net:
    def __init__(self, roads, juncs):
        self.roads = {r.id: r for r in roads}
        self.juncs = juncs
        self.issues = []

    def issue(self, kind, where, msg, sev="repair", **kw):
        self.issues.append(dict(kind=kind, where=where, msg=msg, sev=sev, **kw))

    # one lane of one road, driven in its direction: list of piece dicts, then where it goes
    def drive_road(self, rid, lid, d):
        r = self.roads[rid]
        secs = r.sections if d > 0 else r.sections[::-1]
        pieces, cur = [], lid
        for k, sec in enumerate(secs):
            if cur not in sec.lanes:
                self.issue("lane_missing", "road %s" % rid, "lane %d does not exist in the section at s=%.1f m" % (cur, sec.s))
                return pieces, None
            ln = sec.lanes[cur]
            if ln.type not in X.DRIVABLE:
                self.issue("not_driving", "road %s lane %d" % (rid, cur), "route runs into a %s lane" % ln.type)
                return pieces, None
            p = centre_samples(r, sec, cur, sec.s, sec.s_end)
            if d < 0:
                p = {k_: v[::-1] for k_, v in p.items()}
            p["road"], p["lane"] = rid, cur
            pieces.append(p)
            if k + 1 < len(secs):
                nxt = ln.succ if d > 0 else ln.pred
                cur = nxt if nxt is not None else cur
        last = secs[-1].lanes[cur]
        return pieces, (r.succ if d > 0 else r.pred, last.succ if d > 0 else last.pred, cur)

    def next_steps(self, rid, link, lane_link, lid):
        if link is None:
            return "edge", []
        if link["type"] == "road":
            if lane_link is None:
                return "lane_ends", []
            o = self.roads[link["id"]]
            return "ok", [(o.id, lane_link, +1 if link["contact"] == "start" else -1)]
        out = []
        for j in self.juncs:
            if j.id != link["id"]:
                continue
            for c in j.connections:
                if c["incoming"] != rid:
                    continue
                for f, t in c["lanes"]:
                    if f == lid:
                        out.append((c["connecting"], t, +1 if c["contact"] == "start" else -1))
        return ("ok" if out else "no_connection"), out

    def routes(self):
        """All routes from every entry lane (a driving lane whose start is at the map edge)."""
        starts = []
        for r in self.roads.values():
            if r.junction not in ("-1", None):
                continue
            for d in (+1, -1):
                link = r.pred if d > 0 else r.succ
                if link is not None:
                    continue
                sec = r.sections[0] if d > 0 else r.sections[-1]
                for lid, ln in sec.lanes.items():
                    if lid != 0 and lane_dir(lid) == d and ln.type in X.DRIVABLE:
                        starts.append((r.id, lid, d))
        routes = []

        def walk(rid, lid, d, path, seen):
            if (rid, lid, d) in seen or len(path) > 12:
                return
            pieces, nxt = self.drive_road(rid, lid, d)
            path = path + pieces
            if nxt is None:
                routes.append((path, "broken"))
                return
            link, lane_link, cur = nxt
            how, steps = self.next_steps(rid, link, lane_link, cur)
            if how == "edge":
                routes.append((path, "map edge"))
                return
            if not steps:
                routes.append((path, how))
                return
            for (o, ol, od) in steps:
                # direction sanity: right-hand traffic
                if lane_dir(ol) != od:
                    self.issue("wrong_way", "road %s lane %d" % (o, ol),
                               "the route from road %s lane %d enters it against its driving direction" % (rid, cur))
                walk(o, ol, od, path, seen | {(rid, lid, d)})
        for st in starts:
            walk(*st, [], frozenset())
        return starts, routes


def simulate(path_pieces, net, name):
    """Join the pieces, check joins, then drive the car along with pure pursuit."""
    found = []
    X_, Y_, Z_, W_, C_, R_, L_ = [], [], [], [], [], [], []
    for i, p in enumerate(path_pieces):
        if i:
            q = path_pieces[i - 1]
            gap = math.hypot(p["x"][0] - q["x"][-1], p["y"][0] - q["y"][-1])
            dz = p["z"][0] - q["z"][-1]
            h1 = math.atan2(q["y"][-1] - q["y"][-2], q["x"][-1] - q["x"][-2]) if len(q["x"]) > 1 else 0
            h2 = math.atan2(p["y"][1] - p["y"][0], p["x"][1] - p["x"][0]) if len(p["x"]) > 1 else 0
            kink = abs(math.degrees(math.remainder(h2 - h1, 2 * math.pi)))
            dw = p["w"][0] - q["w"][-1]
            at = "road %s lane %d -> road %s lane %d" % (q["road"], q["lane"], p["road"], p["lane"])
            if gap > 0.10:
                found.append(("gap", at, "lane centres %.2f m apart at the join" % gap, gap))
            if abs(dz) > 0.03:
                found.append(("step", at, "road surface steps %+.2f m at the join" % dz, abs(dz)))
            if kink > 5:
                found.append(("kink", at, "heading turns %.0f deg at the join" % kink, kink))
            if abs(dw) > 0.3:
                found.append(("width_jump", at, "lane width jumps %.2f -> %.2f m at the join" % (q["w"][-1], p["w"][0]), abs(dw)))
        for k in ("x", "y", "z", "w", "cross"):
            pass
        X_.append(p["x"]); Y_.append(p["y"]); Z_.append(p["z"]); W_.append(p["w"]); C_.append(p["cross"])
        R_ += [p["road"]] * len(p["x"]); L_ += [p["lane"]] * len(p["x"])
    x, y, z, w, cr = map(np.concatenate, (X_, Y_, Z_, W_, C_))
    keep = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > 1e-3])
    x, y, z, w, cr = x[keep], y[keep], z[keep], w[keep], cr[keep]
    R_ = np.array(R_)[keep]; L_ = np.array(L_)[keep]
    seg = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < 3:
        return found, dict(length=float(s[-1]))
    # geometry along the route
    hd = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    k = np.gradient(hd) / np.maximum(np.gradient(s), 1e-6)
    ks = np.convolve(k, np.ones(9) / 9, mode="same")          # ~2 m
    grade = np.gradient(z) / np.maximum(np.gradient(s), 1e-6)
    gs = np.convolve(grade, np.ones(9) / 9, mode="same")
    junc = np.array([net.roads[r_].junction not in ("-1", None) for r_ in R_])
    vlim = np.where(junc, V_TURN, V_ROAD)
    # car
    px, py, ph, v = x[0], y[0], hd[0], vlim[0]
    i_near, max_err, err_at, t = 0, 0.0, 0, 0.0
    off_lane, worst_off = 0.0, (0.0, 0)
    steer_sat = 0
    max_alat, alat_at = 0.0, 0
    trace = []
    while i_near < len(s) - 3 and t < 600:
        lo, hi = max(i_near - 5, 0), min(i_near + 80, len(s))
        dd = (x[lo:hi] - px) ** 2 + (y[lo:hi] - py) ** 2
        i_near = lo + int(np.argmin(dd))
        # lateral error (signed) and room left in the lane
        hx, hy = math.cos(hd[i_near]), math.sin(hd[i_near])
        e = -(px - x[i_near]) * hy + (py - y[i_near]) * hx
        if abs(e) > max_err:
            max_err, err_at = abs(e), i_near
        out = abs(e) + CAR_W / 2 - w[i_near] / 2
        if out > worst_off[0]:
            worst_off = (out, i_near)
        # speed: limit, and slow for curves (a_lat <= A_LAT_MAX)
        ahead = slice(i_near, min(i_near + 60, len(s)))
        kmax = np.max(np.abs(ks[ahead])) if len(ks[ahead]) else 0
        vt = min(vlim[i_near], math.sqrt(A_LAT_MAX / max(kmax, 1e-4)))
        v += np.clip(vt - v, -4 * DT, 2 * DT)
        Ld = max(2.5, 0.6 * v + 1.0)
        j = i_near + int(np.searchsorted(s[i_near:], s[i_near] + Ld))
        j = min(j, len(s) - 1)
        alpha = math.atan2(y[j] - py, x[j] - px) - ph
        delta = math.atan2(2 * WB * math.sin(alpha), Ld)
        if abs(delta) > MAX_STEER:
            steer_sat += 1
        delta = max(-MAX_STEER, min(MAX_STEER, delta))
        a_lat = v * v * math.tan(delta) / WB
        if abs(a_lat) > max_alat:
            max_alat, alat_at = abs(a_lat), i_near
        px += v * math.cos(ph) * DT
        py += v * math.sin(ph) * DT
        ph += v / WB * math.tan(delta) * DT
        t += DT
        trace.append((px, py))
    where = lambda i: "road %s lane %d (%.0f m into the route)" % (R_[i], L_[i], s[i])
    # findings along the route
    tight = np.where(1 / np.maximum(np.abs(ks), 1e-6) < R_MIN + 0.5)[0]
    if len(tight):
        i = tight[np.argmin(1 / np.maximum(np.abs(ks[tight]), 1e-6))]
        found.append(("too_tight", where(i), "curve radius %.1f m; a car needs at least %.1f m" % (1 / abs(ks[i]), R_MIN), 1 / abs(ks[i])))
    steep = np.where(np.abs(gs) > 0.12)[0]
    if len(steep):
        i = steep[np.argmax(np.abs(gs[steep]))]
        found.append(("steep", where(i), "grade %.0f %%" % (100 * gs[i]), abs(gs[i])))
    bank = np.where(np.abs(cr) > 0.08)[0]
    if len(bank):
        i = bank[np.argmax(np.abs(cr[bank]))]
        found.append(("cross_slope", where(i), "cross slope %.1f %%" % (100 * math.tan(cr[i])), abs(cr[i])))
    narrow = np.where(w < CAR_W + 0.6)[0]
    if len(narrow):
        i = narrow[np.argmin(w[narrow])]
        found.append(("narrow", where(i), "lane only %.2f m wide (car %.1f m)" % (w[i], CAR_W), w[i]))
    wide = np.where(w > 4.6)[0]
    if len(wide):
        i = wide[np.argmax(w[wide])]
        found.append(("wide", where(i), "lane %.2f m wide (normal 2.7-3.7 m; a car could drive two abreast)" % w[i], w[i]))
    if worst_off[0] > 0.05:
        found.append(("leaves_lane", where(worst_off[1]), "car body crosses the lane edge by %.2f m" % worst_off[0], worst_off[0]))
    if max_err > 0.8:
        found.append(("off_track", where(err_at), "car could not follow the lane: %.2f m off its centre" % max_err, max_err))
    if steer_sat > 2:
        found.append(("steering_limit", where(alat_at), "steering at full lock for %.1f s" % (steer_sat * DT), steer_sat * DT))
    dpos = math.hypot(trace[-1][0] - x[-1], trace[-1][1] - y[-1]) if trace else 0
    return found, dict(length=float(s[-1]), time=t, max_err=max_err, max_alat=max_alat,
                       min_w=float(w.min()), max_w=float(w.max()), max_grade=float(np.max(np.abs(gs))),
                       reached_end=dpos < 2.0, route_xy=np.stack([x, y], 1)[::8].round(2).tolist(),
                       car_xy=np.array(trace[::8]).round(2).tolist())


def describe(pieces):
    out, last = [], None
    for p in pieces:
        k = (p["road"], p["lane"])
        if k != last:
            out.append("%s:%d" % k)
        last = k
    return " > ".join(out)


def drive(roads, juncs, rep=None) -> dict:
    """Drive every route; findings go into ``rep`` (an xodr_check.Report) as
    warnings. Returns a summary per route."""
    net = Net(roads, juncs)
    starts, routes = net.routes()
    visited, res = set(), []
    for pieces, end in routes:
        if not pieces:
            continue
        for p in pieces:
            visited.add((p["road"], p["lane"]))
        found, st = simulate(pieces, net, describe(pieces))
        st.pop("route_xy", None)
        st.pop("car_xy", None)
        res.append(dict(route=describe(pieces), end=end,
                        findings=[dict(kind=a, where=b, msg=c, value=d) for a, b, c, d in found], **st))
    unused = sorted({"road %s lane %d" % (r.id, lid) for r in net.roads.values() for sec in r.sections
                     for lid, ln in sec.lanes.items()
                     if lid and ln.type in X.DRIVABLE and (r.id, lid) not in visited})
    if rep is not None:
        seen = set()
        for rt in res:
            for f in rt["findings"]:
                if f["kind"] in ("wide",):
                    continue                     # lane_width_range already says so
                key = (f["kind"], f["where"].split(" (")[0])
                if key in seen:
                    continue
                seen.add(key)
                rep.add("warning", "drive_" + f["kind"], "drive test, route %s: %s at %s"
                        % (rt["route"], f["msg"], f["where"]))
            if rt["end"] not in ("map edge",):
                rep.add("warning", "drive_dead_end", "drive test, route %s ends: %s" % (rt["route"], rt["end"]))
        for u in unused:
            rep.add("warning", "drive_unreachable", "drive test: no route uses %s" % u)
        for it in net.issues:
            rep.add("warning", "drive_" + it["kind"], "drive test: %s: %s" % (it["where"], it["msg"]))
    return {"routes": res, "entries": len(starts), "unused_lanes": unused,
            "problem_routes": sum(1 for r in res if r["findings"])}
