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
