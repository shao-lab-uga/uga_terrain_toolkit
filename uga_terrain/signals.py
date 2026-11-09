"""Traffic signal structures from the lidar: the span wires over a junction, the
poles at the corners that carry them, and the signal heads hanging from them.

Works on the merged lidar cloud (lidar_map.collect) of a map:
1. height of every point above the ground (1 m ground grid);
2. "thin" points 4-9 m up: cells whose 2.4 m neighbourhood is a narrow line
   (structure tensor), which keeps wires and arms and drops tree canopy;
3. straight wires (Hough + fit) near the junction; where two wires meet near
   their ends is a corner;
4. at each corner, the pole: a narrow column of points from the ground up to
   the wire, within a few metres of the corner;
5. along each wire, signal heads: compact blobs 0.6-2 m tall hanging just
   below it."""
from __future__ import annotations

import math
from typing import List

import numpy as np

CELL = 0.2
POLE_MERGE = 4.5      # m; one pole seen from two wires
SPAN_MIN_R, SPAN_MAX_R = 6.0, 30.0   # m from the junction centre to a span pole
SPAN_MIN_H = 6.0      # m; span poles reach above the wires (~6.5 m)
CORNER_GAP = 9.0      # m from a wire's last seen point to the corner it runs to
WIRE_LO, WIRE_HI = 4.0, 9.0


def ground_height(G: np.ndarray, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
    """Ground height at X, Y from ground points G (x, y, z): 1 m median grid,
    holes filled from the nearest cells."""
    k = np.floor(G[:, :2]).astype(np.int64)
    lo = k.min(0)
    k -= lo
    shape = tuple(k.max(0) + 1)
    key = k[:, 0] * shape[1] + k[:, 1]
    order = np.lexsort((G[:, 2], key))
    ks = key[order]
    st = np.flatnonzero(np.r_[True, np.diff(ks) > 0])
    en = np.r_[st[1:], len(ks)]
    Z = np.full(shape[0] * shape[1], np.nan)
    Z[ks[st]] = G[order[(st + en) // 2], 2]
    Z = Z.reshape(shape)
    for _ in range(40):                       # grow into holes
        bad = ~np.isfinite(Z)
        if not bad.any():
            break
        acc = np.zeros(shape)
        cnt = np.zeros(shape)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            sh = np.roll(Z, (di, dj), (0, 1))
            ok = np.isfinite(sh)
            acc[ok] += sh[ok]
            cnt[ok] += 1
        fill = bad & (cnt > 0)
        Z[fill] = acc[fill] / cnt[fill]
    i = np.clip(np.floor(X).astype(np.int64) - lo[0], 0, shape[0] - 1)
    j = np.clip(np.floor(Y).astype(np.int64) - lo[1], 0, shape[1] - 1)
    return Z[i, j]


def _box(A, r):
    S = np.pad(A, ((r + 1, r), (r + 1, r))).cumsum(0).cumsum(1)
    return S[2 * r + 1:, 2 * r + 1:] - S[:-2 * r - 1, 2 * r + 1:] - S[2 * r + 1:, :-2 * r - 1] + S[:-2 * r - 1, :-2 * r - 1]


def thin_points(P: np.ndarray, h: np.ndarray) -> np.ndarray:
    """x, y, height-above-ground of the cells 4-9 m up that lie on a narrow line."""
    m = (h > WIRE_LO) & (h < WIRE_HI)
    Q, hq = P[m], h[m]
    if len(Q) < 50:
        return np.zeros((0, 3))
    lo = Q[:, :2].min(0) - 2.0
    k = np.floor((Q[:, :2] - lo) / CELL).astype(int)
    shape = tuple(k.max(0) + 6)
    O = np.zeros(shape)
    Hs = np.zeros(shape)
    np.add.at(O, (k[:, 0], k[:, 1]), 1.0)
    np.add.at(Hs, (k[:, 0], k[:, 1]), hq)
    Hm = np.where(O > 0, Hs / np.maximum(O, 1), np.nan)
    O = (O > 0).astype(float)
    I, J = np.meshgrid(np.arange(shape[0], dtype=float), np.arange(shape[1], dtype=float), indexing="ij")
    r = 6
    n = _box(O, r)
    with np.errstate(invalid="ignore", divide="ignore"):
        mx, my = _box(O * I, r) / n, _box(O * J, r) / n
        cxx = _box(O * I * I, r) / n - mx ** 2
        cyy = _box(O * J * J, r) / n - my ** 2
        cxy = _box(O * I * J, r) / n - mx * my
        tr = cxx + cyy
        disc = np.sqrt(np.maximum(tr ** 2 / 4 - (cxx * cyy - cxy ** 2), 0))
        l1, l2 = tr / 2 + disc, tr / 2 - disc
        thin = (O > 0) & (n >= 12) & (n <= 110) & (l2 / l1 < 0.3) & (l1 > 8)
    ti, tj = np.nonzero(thin)
    return np.c_[lo + (np.c_[ti, tj] + 0.5) * CELL, Hm[ti, tj]]


def lines(X: np.ndarray, min_len: float = 6.0, tol: float = 0.45, max_gap: float = 4.0,
          n_max: int = 30) -> List[dict]:
    """Straight runs in the points X (x, y, ...): {p, d, a0, a1, h (median 3rd col)}."""
    out: List[dict] = []
    if len(X) < 20:
        return out
    th = np.radians(np.arange(0.0, 180.0, 0.5))
    C, S = np.cos(th), np.sin(th)
    c0 = X[:, :2].mean(0)
    for _ in range(80):
        if len(X) < 20 or len(out) >= n_max:
            break
        rb = np.round(((X[:, 0:1] - c0[0]) * C + (X[:, 1:2] - c0[1]) * S) / 0.2).astype(int)
        off = rb.min()
        rb -= off
        acc = np.zeros((rb.max() + 1, len(th)))
        np.add.at(acc, (rb, np.broadcast_to(np.arange(len(th)), rb.shape)), 1)
        acc = acc + np.roll(acc, 1, 0) + np.roll(acc, -1, 0)
        r_i, t_i = np.unravel_index(np.argmax(acc), acc.shape)
        if acc[r_i, t_i] < 15:
            break
        n = np.array([C[t_i], S[t_i]])
        on = np.abs((X[:, :2] - c0) @ n - (r_i + off) * 0.2) < tol
        m = X[on, :2].mean(0)
        d = np.linalg.svd(X[on, :2] - m, full_matrices=False)[2][0]
        n = np.array([-d[1], d[0]])
        on = np.flatnonzero(np.abs((X[:, :2] - m) @ n) < tol)
        a = (X[on, :2] - m) @ d
        order = np.argsort(a)
        a, on = a[order], on[order]
        took = np.zeros(len(X), bool)
        for sg in np.split(np.arange(len(a)), np.flatnonzero(np.diff(a) > max_gap) + 1):
            ln = a[sg[-1]] - a[sg[0]]
            if ln >= min_len and len(sg) >= 1.0 * ln:      # >= 1 cell per m
                out.append({"p": m, "d": d, "a0": float(a[sg[0]]), "a1": float(a[sg[-1]]),
                            "h": float(np.median(X[on[sg], 2])) if X.shape[1] > 2 else 0.0})
            took[on[sg]] = True
        if not took.any():
            break
        X = X[~took]
    return out


def _ends(L):
    return L["p"] + L["d"] * L["a0"], L["p"] + L["d"] * L["a1"]


def _cross(A, B):
    """Intersection of the two infinite lines, and how far it is past each segment's end."""
    M = np.array([A["d"], -B["d"]]).T
    if abs(np.linalg.det(M)) < 1e-6:
        return None
    s, t = np.linalg.solve(M, B["p"] - A["p"])
    x = A["p"] + A["d"] * s
    past_a = max(A["a0"] - s, s - A["a1"], 0.0)
    past_b = max(B["a0"] - t, t - B["a1"], 0.0)
    return x, past_a, past_b


def detect(lidar: np.ndarray, centre, radius: float = 40.0) -> dict:
    """Span wires, corner poles and signal heads around a junction at ``centre``.
    ``lidar`` is a lidar_map.DTYPE array. Coordinates are the map's."""
    centre = np.asarray(centre, float)
    near = np.hypot(lidar["x"] - centre[0], lidar["y"] - centre[1]) < radius + 15
    L = lidar[near]
    G = np.c_[L["x"], L["y"], L["z"]][L["ground"] == 1]
    O = L[(L["ground"] == 0) & (L["hits"] >= 2)]
    P = np.c_[O["x"], O["y"], O["z"]]
    if len(G) < 100 or len(P) < 100:
        return {"wires": [], "poles": [], "heads": []}
    h = P[:, 2] - ground_height(G, P[:, 0], P[:, 1])
    X = thin_points(P, h)
    X = X[np.hypot(X[:, 0] - centre[0], X[:, 1] - centre[1]) < radius]
    W = [w for w in lines(X) if w["a1"] - w["a0"] >= 7.0]
    poles = []
    keep = []
    for k, w in enumerate(W):
        hit = False
        for sgn, e in ((-1, w["p"] + w["d"] * w["a0"]), (1, w["p"] + w["d"] * w["a1"])):
            pole = _pole_beyond(P, h, e, sgn * w["d"], w["h"])
            if pole is None:
                continue
            hit = True
            for q in poles:
                if np.hypot(q["x"] - pole["x"], q["y"] - pole["y"]) < POLE_MERGE:
                    q["wires"].append(k)
                    if pole["height"] > q["height"]:     # the column that reaches the wires wins
                        q.update({kk: pole[kk] for kk in ("x", "y", "radius", "score", "height")})
                    break
            else:
                pole["wires"] = [k]
                poles.append(pole)
        if hit:
            keep.append(k)
    poles, keep = _span(poles, keep, centre)
    keep = [k for k in keep if _joins(W[k], poles)]
    for q in poles:
        q["wires"] = [k for k in q["wires"] if k in keep]
    for q in poles:
        q["ground_z"] = round(float(ground_height(G, np.array([q["x"]]), np.array([q["y"]]))[0]), 2)
        q.pop("score", None)
    for i, q in enumerate(poles):
        q["id"] = i
    # the span: poles in order round the junction, a wire from each to the next
    c = np.asarray(centre, float)
    poles.sort(key=lambda q: math.atan2(q["y"] - c[1], q["x"] - c[0]))
    for i, q in enumerate(poles):
        q["id"] = i
        q.pop("wires", None)
    wires = []
    if len(poles) >= 2:
        for i in range(len(poles) if len(poles) > 2 else 1):
            A, B = poles[i], poles[(i + 1) % len(poles)]
            a_ = np.array([A["x"], A["y"]])
            b_ = np.array([B["x"], B["y"]])
            seen = [W[k] for k in keep if _on_edge(W[k], a_, b_)]
            hgt = float(np.median([w["h"] for w in seen])) if seen else None
            wires.append({"id": len(wires), "poles": [A["id"], B["id"]], "a": a_.tolist(), "b": b_.tolist(),
                          "height": round(hgt, 2) if hgt is not None else None,
                          "seen_m": round(float(sum(w["a1"] - w["a0"] for w in seen)), 1),
                          "length": round(float(np.hypot(*(b_ - a_))), 1)})
    heads = []
    for w in wires:
        heads += _heads(P, h, np.array(w["a"]), np.array(w["b"]), w["height"] or 6.5)
    for i, hd in enumerate(heads):
        hd["id"] = i
        hd["ground_z"] = round(float(ground_height(G, np.array([hd["x"]]), np.array([hd["y"]]))[0]), 2)
    return {"wires": wires, "poles": poles, "heads": heads}


def _on_edge(w, a, b) -> bool:
    """Wire segment w lies along the line from pole a to pole b."""
    u = (b - a) / max(np.hypot(*(b - a)), 1e-6)
    if abs(float(w["d"] @ u)) < math.cos(math.radians(12)):
        return False
    n = np.array([-u[1], u[0]])
    return abs(float((w["p"] - a) @ n)) < 2.0


def _heads(P, h, a, b, wire_h) -> List[dict]:
    """Signal heads hanging from the wire a -> b: compact blobs 0.6-2.2 m tall,
    0.25-2 m wide, just under the wire, with nothing (no tree) above them."""
    L = float(np.hypot(*(b - a)))
    u = (b - a) / max(L, 1e-6)
    n = np.array([-u[1], u[0]])
    rel = P[:, :2] - a
    s, t = rel @ u, rel @ n
    band = (np.abs(t) < 1.2) & (s > 1.5) & (s < L - 1.5)
    under = band & (h > wire_h - 3.0) & (h < wire_h - 0.2)
    above = band & (h > wire_h + 0.8) & (h < wire_h + 6.0)
    out = []
    if under.sum() < 10:
        return out
    ss, hh, tt = s[under], h[under], t[under]
    order = np.argsort(ss)
    ss, hh, tt = ss[order], hh[order], tt[order]
    sa = np.sort(s[above])
    for sg in np.split(np.arange(len(ss)), np.flatnonzero(np.diff(ss) > 0.5) + 1):
        if len(sg) < 15:
            continue
        s0, s1 = ss[sg[0]], ss[sg[-1]]
        top, bot = np.percentile(hh[sg], 98), np.percentile(hh[sg], 2)
        n_above = np.searchsorted(sa, s1 + 0.5) - np.searchsorted(sa, s0 - 0.5)
        if not (0.25 <= s1 - s0 <= 2.0 and 0.6 <= top - bot <= 2.2) or n_above > 0.5 * len(sg):
            continue
        xy = a + u * 0.5 * (s0 + s1) + n * float(np.median(tt[sg]))
        out.append({"x": round(float(xy[0]), 2), "y": round(float(xy[1]), 2),
                    "bottom": round(float(bot), 2), "top": round(float(top), 2),
                    "width": round(float(s1 - s0), 2), "wire_dir": np.round(u, 4).tolist()})
    return out


def _pole_beyond(P, h, end, u, wire_h, reach: float = CORNER_GAP):
    """The pole a wire runs to: a narrow column of points (0.3-4 m up covered,
    reaching the wire's height) within 1.5 m of the wire's line, 0-``reach`` m past
    its last seen point ``end`` in direction ``u``."""
    n = np.array([-u[1], u[0]])
    rel = P[:, :2] - end
    s, t = rel @ u, rel @ n
    sel = (s > -1.0) & (s < reach + 1.0) & (np.abs(t) < 2.0)
    Ps, hs = P[sel], h[sel]
    if len(Ps) < 20:
        return None
    best = None
    low = (hs > 0.5) & (hs < 3.0)
    for kk in {tuple(v) for v in np.floor(Ps[low, :2] / 0.3).astype(int)}:
        c = (np.array(kk) + 0.5) * 0.3
        sc, tc = (c - end) @ u, (c - end) @ n
        if not (-0.5 <= sc <= reach and abs(tc) <= 1.5):
            continue
        d = np.hypot(Ps[:, 0] - c[0], Ps[:, 1] - c[1])
        zs = hs[d < 0.35]
        cover = len(np.unique(np.floor(zs[(zs > 0.3) & (zs < 4.0)] / 0.25))) / 15.0
        if cover < 0.6 or not len(zs) or np.percentile(zs, 99) < wire_h - 1.5:
            continue
        score = cover - 0.15 * abs(tc) - 0.02 * sc
        if best is None or score > best[0]:
            best = (score, c)
    if best is None:
        return None
    c = best[1]
    d = np.hypot(Ps[:, 0] - c[0], Ps[:, 1] - c[1])
    core = (d < 0.5) & (hs > 0.5) & (hs < 3.0)
    cc = Ps[core, :2].mean(0)
    d = np.hypot(Ps[:, 0] - cc[0], Ps[:, 1] - cc[1])
    col = (d < 0.4) & (hs > 0.2)
    rad = float(np.percentile(d[(d < 0.4) & (hs > 0.5) & (hs < 3.0)], 80))
    return {"x": round(float(cc[0]), 2), "y": round(float(cc[1]), 2),
            "height": round(float(np.percentile(hs[col], 99)), 2),
            "radius": round(min(max(rad, 0.1), 0.3), 2), "score": float(best[0])}


def _inside(pt, poly) -> bool:
    from uga_terrain.junction_shape import point_in_poly
    return bool(point_in_poly(pt[0], pt[1], poly))


def _span(poles, keep, centre):
    """The poles of the signal span: up to 4 tall poles round the junction, one
    per corner (30 deg and 10 m apart at least), best first: carrying
    more wires, then taller. Wires kept are those that run from one of these
    poles towards another. Returns (poles, wire indices)."""
    c = np.asarray(centre, float)
    cand = []
    for q in poles:
        d = float(np.hypot(q["x"] - c[0], q["y"] - c[1]))
        if SPAN_MIN_R <= d <= SPAN_MAX_R and q["height"] >= SPAN_MIN_H:
            cand.append((len(q["wires"]), q["height"], q))
    cand.sort(key=lambda t: (-t[0], -t[1]))
    picked = []
    for _, _, q in cand:
        ang = math.atan2(q["y"] - c[1], q["x"] - c[0])
        if all(abs((ang - math.atan2(o["y"] - c[1], o["x"] - c[0]) + math.pi) % (2 * math.pi) - math.pi)
               >= math.radians(30) and math.hypot(q["x"] - o["x"], q["y"] - o["y"]) >= 10.0 for o in picked):
            picked.append(q)
        if len(picked) == 4:
            break
    return [dict(q) for q in picked], keep




def _joins(w, poles) -> bool:
    """The wire ends at one of the poles and points at another (or ends at two)."""
    P = np.array([[q["x"], q["y"]] for q in poles]) if poles else np.zeros((0, 2))
    if len(P) < 2:
        return False
    n = np.array([-w["d"][1], w["d"][0]])
    s = (P - w["p"]) @ w["d"]
    t = np.abs((P - w["p"]) @ n)
    on_line = (t < 2.0) & (s > w["a0"] - CORNER_GAP - 1) & (s < w["a1"] + CORNER_GAP + 1)
    return int(on_line.sum()) >= 2
