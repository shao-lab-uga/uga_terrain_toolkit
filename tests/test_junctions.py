"""Junctions: found from a turning drive, written with lane-to-lane links that
the OpenDRIVE checker accepts, and a turning vehicle replays through them."""
import csv
import math

import numpy as np

from uga_terrain import junctions as J
from uga_terrain import register, xodr
from uga_terrain import xodr_check as X
from uga_terrain.config import load_config
from uga_terrain.roads import Road, split_roads


def _l_drive():
    """North 60 m, a right turn of radius 8 m, then east 60 m."""
    pts = [(0.0, y) for y in np.arange(-60.0, 0.0, 0.5)]
    for a in np.linspace(math.pi, math.pi / 2, 40):
        pts.append((8.0 + 8.0 * math.cos(a), 8.0 * math.sin(a)))
    pts += [(x, 8.0) for x in np.arange(8.5, 68.0, 0.5)]
    return np.array(pts)


def _road(rid, x, y, h, meta):
    n = len(x)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    data = {k: np.zeros(n) for k in ("z", "grade_pct", "bank_deg", "curb_h_l", "curb_h_r", "verge_l",
                                     "verge_r", "walk_l", "walk_r", "walk_h_l", "walk_h_r")}
    data["road_l"] = np.full(n, 3.5)
    data["road_r"] = np.full(n, 3.5)
    data["lb0"], data["lb1"], data["lb2"] = np.full(n, -3.5), np.zeros(n), np.full(n, 3.5)
    meta = dict(meta, lanes={"source": "paint", "centre": 1, "types": ["driving", "driving"],
                             "marks": ["edge", "solid", "edge"], "lines": []})
    return Road(rid, "0:drive", s, x, y, h, data, meta)


def test_turn_makes_a_junction_and_two_roads():
    cfg = load_config()
    drive = _l_drive()
    js = J.find([("0:drive", drive)], cfg)
    assert len(js) == 1
    j = js[0]
    assert math.hypot(j.x - 0.0, j.y - 8.0) < 2.0          # where the two straight lines meet
    pieces, passes = J.cut([("0:drive", drive)], js)
    roads = split_roads(pieces, cfg)
    assert len(roads) == 2 and len(passes) == 1
    ends = {(r.meta.get("junction_start"), r.meta.get("junction_end")) for r in roads}
    assert ends == {(None, j.id), (j.id, None)}


def test_junction_xodr_is_valid_and_replays(tmp_path):
    cfg = load_config()
    # road 1 ends at the junction heading north, road 2 starts there heading east
    y1 = np.arange(-60.0, -10.0, 1.0)
    r1 = _road(1, np.zeros(len(y1)), y1, np.full(len(y1), math.pi / 2), {"junction_end": 1000})
    x2 = np.arange(10.0, 60.0, 1.0)
    r2 = _road(2, x2, np.zeros(len(x2)), np.zeros(len(x2)), {"junction_start": 1000})
    jn = J.Junction(1000, 0.0, 0.0, 10.0, ["test"], {"1>2": 1})
    out = tmp_path / "j.xodr"
    info = xodr.write_xodr([r1, r2], (33.9, -83.4, 200.0), cfg, out, junctions=[jn])
    # right turn (driven) and the left turn back (not driven), one lane each
    assert info["connecting_roads"] == 2
    turns = sorted(c["turn"] for c in info["junctions"][0]["connections"])
    assert turns == ["left", "right"]
    # a right turn from the right lane of road 1 into the right lane of road 2
    tr = tmp_path / "t.csv"
    with open(tr, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "yaw_deg"])
        for y in np.arange(-55.0, -10.0, 1.0):
            w.writerow([1.75, y, 0.0, 90.0])
        for a in np.linspace(math.pi, math.pi / 2, 30)[1:-1]:
            w.writerow([10.0 + 8.25 * math.cos(a), -10.0 + 8.25 * math.sin(a), 0.0,
                        math.degrees(a - math.pi / 2)])
        for x in np.arange(10.0, 55.0, 1.0):
            w.writerow([x, -1.75, 0.0, 0.0])
    o = X.check(out, [tr])
    assert o["counts"]["error"] == 0, o["findings"]
    assert o["topology"]["network_pieces"] == 1 and o["topology"]["junctions"] == 1
    r = o["replay"][0]
    assert r["in_driving_lane_pct"] == 100 and not r["unlinked_moves"]
    assert r["lanes_used"][0] == "1:-1" and r["lanes_used"][-1] == "2:-1" and len(r["lanes_used"]) == 3


def test_register_recovers_a_known_offset():
    rng = np.random.default_rng(1)
    # ground with painted stripes: relative intensity 1, stripes 3
    xy = rng.uniform(0, 40, size=(400000, 2))
    it = np.where((np.mod(xy[:, 0], 6.0) < 0.15) | (np.mod(xy[:, 1] + 0.3 * xy[:, 0], 9.0) < 0.2), 3.0, 1.0)
    it = it + rng.normal(0, 0.1, len(it))
    z = 0.01 * xy[:, 0]
    A = np.c_[xy, z, it]
    # B: the same ground seen through a pose that is off by 0.4 m, -0.3 m, 0.8 deg, 12 cm
    C = register.correction({"yaw_deg": 0.8, "dx": 0.4, "dy": -0.3, "pivot": [20.0, 20.0]},
                            {"dz": 0.12, "slope_x": 0.0, "slope_y": 0.0, "pivot": [20.0, 20.0]})
    Cinv = np.linalg.inv(C)
    B = A.copy()
    B[:, :3] = (Cinv[:3, :3] @ A[:, :3].T).T + Cinv[:3, 3]
    m = register.match_xy(A, B)
    assert abs(m["yaw_deg"] - 0.8) < 0.1
    Bc = B.copy()
    Bc[:, :3] = (register.correction(m, None)[:3, :3] @ B[:, :3].T).T + register.correction(m, None)[:3, 3]
    k = rng.choice(len(A), 2000, replace=False)
    err = np.hypot(*(Bc[k, :2] - A[k, :2]).T)
    assert np.median(err) < 0.05
    zfit = register.match_z(A, Bc)
    assert abs(zfit["dz"] - 0.12) < 0.02


def _crossroads_grid():
    """Flat 10 m wide streets along x and y through (0, 0), raised sidewalks
    behind 15 cm curbs elsewhere (rough where the curb steps up)."""
    from uga_terrain.grid import TerrainGrid
    g = TerrainGrid(0.2, 512)
    for kx in (-1, 0):
        for ky in (-1, 0):
            c = (np.arange(512) + 0.5) * 0.2
            X, Y = np.meshgrid(kx * 102.4 + c, ky * 102.4 + c)
            def street(X, Y):
                # 10 m streets; the corners are rounded a bit, like curb returns
                corner = (np.abs(X) < 8.0) & (np.abs(Y) < 8.0) & (np.hypot(np.abs(X) - 8.0, np.abs(Y) - 8.0) > 3.0)
                return (np.abs(X) < 5.0) | (np.abs(Y) < 5.0) | corner
            road = street(X, Y)
            z = np.where(road, 0.0, 0.15).astype(np.float32)
            edge = np.zeros_like(road)
            for dx, dy in ((0.4, 0), (-0.4, 0), (0, 0.4), (0, -0.4)):   # within 0.4 m of the curb
                edge |= street(X + dx, Y + dy) != road
            rough = np.where(edge, 0.06, 0.01).astype(np.float32)
            g.tiles[(kx, ky)] = {"z": z, "roughness": rough, "slope_deg": np.zeros_like(z),
                                 "label": np.full_like(z, 4)}
    return g


def test_junction_fits_the_curbs_and_adds_undriven_streets():
    from uga_terrain import junction_shape as JS
    cfg = load_config()
    g = _crossroads_grid()
    # one drive comes in from the west and stops at the junction
    x = np.arange(-60.0, -12.0, 1.0)
    r1 = Road(1, "0:drive", x - x[0], x, np.full(len(x), -2.0), np.zeros(len(x)), {}, {"junction_end": 1000})
    jn = J.Junction(1000, 0.0, 0.0, 16.0, ["test"])
    res = JS.fit(jn, JS.leg_ends(jn, [r1]), g, cfg, [np.c_[r1.x, r1.y]])
    stubs = [l for l in res["legs"] if l.get("stub")]
    dirs = sorted(round(l["theta_deg"] / 90.0) % 4 for l in stubs)
    assert dirs == [0, 1, 3], res["legs"]                    # east, north, south
    assert all(8.0 <= l["width"] <= 12.0 for l in stubs)
    # the edge sits where the curb returns end, well inside the old circle
    west = next(l for l in res["legs"] if l.get("road") == 1)
    assert 6.0 <= west["d_cut"] <= 14.0
    jn.outline, jn.legs = res["outline"], res["legs"]
    assert jn.contains(0.0, 0.0) and not jn.contains(-30.0, 0.0)
    roads = JS.stub_roads([jn], g, cfg, 2)
    assert len(roads) == 3 and all(r.meta["junction_start"] == 1000 and r.meta["stub"] for r in roads)


def test_edits_one_way_speed_and_removed_turn(tmp_path):
    cfg = load_config()
    y1 = np.arange(-60.0, -10.0, 1.0)
    r1 = _road(1, np.zeros(len(y1)), y1, np.full(len(y1), math.pi / 2), {"junction_end": 1000})
    x2 = np.arange(10.0, 60.0, 1.0)
    r2 = _road(2, x2, np.zeros(len(x2)), np.zeros(len(x2)), {"junction_start": 1000})
    jn = J.Junction(1000, 0.0, 0.0, 10.0, ["test"], {"1>2": 1})
    out = tmp_path / "j.xodr"
    edits = {"roads": {"2": {"one_way": "forward", "speed": 30, "unit": "mph"}}}
    info = xodr.write_xodr([r1, r2], (33.9, -83.4, 200.0), cfg, out, junctions=[jn], edits=edits)
    # road 2 is one-way away from the junction: only the right turn into it is left
    assert info["connecting_roads"] == 1
    root, roads, _ = X.read(out)
    r2x = next(r for r in roads if r.id == "2")
    ids = sorted(l for l, ln in r2x.sections[0].lanes.items() if ln.type == "driving")
    assert ids == [-2, -1]
    sp = root.find("road[@id='2']/type/speed")
    assert sp.get("max") == "30" and sp.get("unit") == "mph"
    # a removed turn is not written
    info = xodr.write_xodr([r1, r2], (33.9, -83.4, 200.0), cfg, out, junctions=[jn],
                           edits={"removed_turns": ["1:-1>2:-1"]})
    assert [c["turn"] for c in info["junctions"][0]["connections"]] == ["left"]


def test_markings_become_road_objects(tmp_path):
    cfg = load_config()
    x2 = np.arange(10.0, 60.0, 1.0)
    r2 = _road(2, x2, np.zeros(len(x2)), np.zeros(len(x2)), {})
    r2.meta["markings"] = [{"kind": "crosswalk", "polygon": [[12, -3.5], [15, -3.5], [15, 3.5], [12, 3.5]]},
                           {"kind": "stop_line", "polygon": [[17, -3.5], [17.5, -3.5], [17.5, 0], [17, 0]]}]
    out = tmp_path / "m.xodr"
    xodr.write_xodr([r2], (33.9, -83.4, 200.0), cfg, out)
    o = X.check(out)
    assert o["counts"]["error"] == 0, o["findings"]
    m = X.lane_geometry(out)["markings"]
    assert [v["type"] for v in m] == ["crosswalk", "roadMark"]
    P = np.array(m[0]["polygon"])
    assert abs(P[:, 0].min() - 12) < 0.05 and abs(P[:, 1].max() - 3.5) < 0.05


def _ladder_points(shift=(0.0, 0.0), seed=0):
    """Ground points of a road with one diagonal ladder crosswalk (two side
    lines 3 m apart, 12 m long, bars between) and a stop line beside it."""
    import math
    rng = np.random.default_rng(seed)
    P = rng.uniform(-20, 20, (60000, 2))
    it = rng.normal(1.0, 0.1, len(P))
    a = math.radians(35.0)
    d = np.array([math.cos(a), math.sin(a)])
    n = np.array([-d[1], d[0]])
    s, t = P @ d, P @ n
    along = np.abs(s) < 6.0
    side = along & ((np.abs(t) < 0.1) | (np.abs(t - 3.0) < 0.1))
    bars = along & (t > 0) & (t < 3.0) & (np.mod(s, 1.2) < 0.5)
    stop = (np.abs(s) < 3.0) & (np.abs(t - 6.0) < 0.15)
    it[side | bars | stop] = 4.0
    P = P + np.asarray(shift)
    return np.c_[P, np.zeros(len(P)), it]


def test_crosswalk_found_from_the_paint_at_any_angle():
    from uga_terrain import markings
    found = markings.find_painted(_ladder_points(), (0.0, 0.0), 20.0)
    cw = [m for m in found if m["kind"] == "crosswalk"]
    st = [m for m in found if m["kind"] == "stop_line"]
    assert len(cw) == 1 and len(st) == 1
    assert abs(cw[0]["depth"] - 3.0) < 0.3 and abs(cw[0]["width"] - 12.0) < 1.0
    c = np.mean(cw[0]["polygon"], 0)
    assert np.hypot(*(c - 1.5 * np.array([-np.sin(np.radians(35)), np.cos(np.radians(35))]))) < 0.5


def test_markings_from_another_map_are_lined_up_on_the_paint():
    from uga_terrain import markings
    A = _ladder_points()
    B = _ladder_points(shift=(0.6, -0.4), seed=1)       # the other drive, GPS off by 0.7 m
    poly = [[-5.0, -5.0], [5.0, -5.0], [5.0, 5.0], [-5.0, 5.0]]
    res = markings.snap([poly], A, B)
    assert res["used"]
    assert abs(res["dx"] + 0.6) < 0.15 and abs(res["dy"] - 0.4) < 0.15


def _span_cloud():
    """Synthetic lidar of a junction: flat ground, 4 corner poles 8 m tall,
    span wires between them at 6.5 m and two signal heads on one wire."""
    from uga_terrain import lidar_map
    rng = np.random.default_rng(3)
    g = np.mgrid[-40:40:0.25, -40:40:0.25].reshape(2, -1).T
    pts = [np.c_[g, np.zeros(len(g))]]
    gnd = [np.ones(len(g), bool)]
    poles = np.array([[-14.0, 12.0], [13.0, 14.0], [15.0, -12.0], [-12.0, -15.0]])
    for p in poles:
        z = np.arange(0.1, 8.0, 0.05)
        a = rng.uniform(0, 2 * np.pi, len(z))
        pts.append(np.c_[p[0] + 0.15 * np.cos(a), p[1] + 0.15 * np.sin(a), z])
        gnd.append(np.zeros(len(z), bool))
    for i in range(4):
        a, b = poles[i], poles[(i + 1) % 4]
        f = np.arange(0, 1, 0.1 / np.hypot(*(b - a)))
        xy = a + (b - a) * f[:, None] + rng.normal(0, 0.03, (len(f), 2))
        pts.append(np.c_[xy, 6.5 + rng.normal(0, 0.03, len(f))])
        gnd.append(np.zeros(len(f), bool))
    a, b = poles[0], poles[1]
    for f in (0.4, 0.6):
        c = a + (b - a) * f
        box = np.mgrid[-0.2:0.2:0.05, -0.15:0.15:0.05, 5.1:6.2:0.05].reshape(3, -1).T
        pts.append(np.c_[c + box[:, :2], box[:, 2]])
        gnd.append(np.zeros(len(box), bool))
    P = np.vstack(pts)
    out = np.zeros(len(P), lidar_map.DTYPE)
    out["x"], out["y"], out["z"] = P.T
    out["intensity"] = 10
    out["ground"] = np.concatenate(gnd)
    out["hits"] = 10
    return out, poles


def test_signal_span_poles_wires_and_heads_found():
    from uga_terrain import signals
    cloud, poles = _span_cloud()
    res = signals.detect(cloud, (0.0, 0.0))
    assert len(res["poles"]) == 4
    for p in poles:
        d = min(np.hypot(q["x"] - p[0], q["y"] - p[1]) for q in res["poles"])
        assert d < 0.5
    assert all(7.0 < q["height"] < 8.5 for q in res["poles"])
    assert len(res["wires"]) == 4 and sum(w["seen_m"] > 0 for w in res["wires"]) >= 3
    assert len(res["heads"]) == 2
    assert all(abs(h["bottom"] - 5.1) < 0.3 and abs(h["top"] - 6.2) < 0.3 for h in res["heads"])


def test_tidy_fills_gaps_and_paints_junction_as_road():
    from types import SimpleNamespace
    from uga_terrain.grid import TerrainGrid
    from uga_terrain import tidy
    g = TerrainGrid(0.2, 128)
    z = np.full((128, 128), 1.0, np.float32)
    z[60:70, 60:70] = np.nan            # 2 m gap: filled
    z[:, :20] = np.nan                  # open edge: left alone
    lab = np.where(np.isfinite(z), 4, 0).astype(np.float32)
    lab[:, 40:90] = 1                   # a road ...
    lab[50, 50] = 4                     # ... with a speck of grass in it
    lab[np.isnan(z)] = 0
    nan = np.full_like(z, np.nan)
    g.tiles[(0, 0)] = {"z": z, "label": lab, "filled": np.zeros_like(z), "slope_deg": nan.copy(),
                       "aspect_deg": nan.copy()}
    jn = SimpleNamespace(outline=[[20.0, 20.0], [24.0, 20.0], [24.0, 24.0], [20.0, 24.0]])
    res = tidy.tidy(g, [jn], {})
    t = g.tiles[(0, 0)]
    assert np.isfinite(t["z"][60:70, 60:70]).all() and np.allclose(t["z"][60:70, 60:70], 1.0)
    assert (t["filled"][60:70, 60:70] == tidy.TIDY_FILLED).all()
    assert np.isnan(t["z"][:, :20]).all()
    assert t["label"][50, 50] == 1
    assert (t["label"][105:115, 105:115] == 1).all()      # inside the junction outline
    assert res["filled_m2"] > 3.9
    tidy.untidy(g)
    assert np.isnan(g.tiles[(0, 0)]["z"][60:70, 60:70]).all()
