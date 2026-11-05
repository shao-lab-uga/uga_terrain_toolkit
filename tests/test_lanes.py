"""Lane lines from a synthetic road-aligned intensity image."""
import numpy as np

from uga_terrain import lanes
from uga_terrain.config import load_config
from uga_terrain.roads import Road


def test_painted_lines_become_steady_lanes():
    L, hw = 60.0, 9.0
    s = np.arange(0, L + 0.5, 1.0)
    road = Road(1, "0:x", s, s.copy(), np.zeros_like(s), np.zeros_like(s),
                {"road_l": np.full(len(s), 3.6), "road_r": np.full(len(s), 7.0)})
    rng = np.random.default_rng(1)
    ns, nt = int(L / lanes.DS) + 1, int(2 * hw / lanes.DT) + 1
    img = 1.0 + 0.08 * rng.standard_normal((ns, nt))
    tt = np.arange(nt) * lanes.DT - hw
    rows = np.arange(ns) * lanes.DS
    img[:, np.abs(tt - 0.0) < 0.07] += 1.0                                   # solid centre line
    dashed = (rows % 12.0) < 3.0                                             # 3 m dash, 9 m gap
    img[np.ix_(dashed, np.abs(tt + 3.5) < 0.07)] += 1.0                     # lane divider
    stripes = lanes.find_stripes(img, hw, -road.data["road_r"], road.data["road_l"])
    lines = lanes.link_lines(stripes, L)
    assert sorted(round(float(np.median(l["t"])), 1) for l in lines) == [-3.5, 0.0]
    kinds = {("centre" if abs(np.median(l["t"])) < 1 else "divider"): l["kind"] for l in lines}
    assert kinds == {"centre": "solid", "divider": "broken"}
    lay = lanes.layout(road, lines, load_config())
    assert lay["source"] == "paint"
    assert lay["types"] == ["driving", "driving", "driving"]
    c = lay["centre"]
    assert abs(np.median(lay["bounds"][c])) < 0.05                          # centre line at t=0
    widths = [np.median(b2 - b1) for b1, b2 in zip(lay["bounds"], lay["bounds"][1:])]
    assert np.allclose(widths, [3.5, 3.5, 3.6], atol=0.1)


def test_no_paint_splits_evenly_with_steady_widths():
    s = np.arange(0, 40.5, 1.0)
    wob = 0.3 * np.sin(s)                                                    # noisy curb positions
    road = Road(1, "0:x", s, s.copy(), np.zeros_like(s), np.zeros_like(s),
                {"road_l": 3.6 + wob, "road_r": 3.6 - wob})
    lay = lanes.layout(road, [], load_config())
    assert lay["source"] == "width" and lay["types"].count("driving") == 2
    for b1, b2, t in zip(lay["bounds"], lay["bounds"][1:], lay["types"]):
        if t == "driving":
            assert np.ptp(b2 - b1) < 0.15                                    # no wobble
