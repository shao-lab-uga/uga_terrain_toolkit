"""Grid, road cross-sections and OpenDRIVE on a synthetic street (4 % grade, 3 deg bank,
3.5 m lanes, 0.15 m curbs, grass verge, sidewalk)."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from synth import ground_points
from uga_terrain import roads, xodr
from uga_terrain.accumulate import VoxelAccumulator
from uga_terrain.config import load_config
from uga_terrain.grid import TerrainGrid


@pytest.fixture(scope="module")
def street():
    cfg = load_config()
    pts, _ = ground_points()
    acc = VoxelAccumulator(cfg["grid"]["cell"], cfg["grid"]["z_bin"])
    acc.add(pts)
    grid = TerrainGrid.from_cells(acc.cells(), cfg)
    path = np.stack([np.linspace(0, 120, 600), np.zeros(600)], 1)
    rs = roads.split_roads([("a", path)], cfg)
    road = roads.analyse_road(rs[0], grid, cfg)
    return cfg, grid, road


def med(road, key):
    return float(np.nanmedian(road.data[key][10:-10]))


def test_grade_and_bank(street):
    _, _, r = street
    assert med(r, "grade_pct") == pytest.approx(4.0, abs=0.3)
    assert abs(med(r, "bank_deg")) == pytest.approx(3.0, abs=0.6)


def test_curbs_and_widths(street):
    _, _, r = street
    for side in "lr":
        assert med(r, "curb_h_" + side) == pytest.approx(0.15, abs=0.04)
        assert med(r, "walk_" + side) > 1.0


def test_xodr_is_valid_xml(street, tmp_path):
    cfg, _, r = street
    out = tmp_path / "t.xodr"
    xodr.write_xodr([r], (33.9, -83.3, 186.0), cfg, out, "test")
    root = ET.parse(out).getroot()
    assert root.tag == "OpenDRIVE"
    road = root.find("road")
    assert float(road.get("length")) == pytest.approx(r.length, rel=0.02)
    assert root.find("header/geoReference") is not None
    assert road.find("elevationProfile/elevation") is not None
    assert road.find("lateralProfile/superelevation") is not None
    types = {ln.get("type") for ln in road.iter("lane")}
    assert {"driving", "sidewalk"} <= types


def test_param_poly3_follows_path():
    t = np.linspace(0, np.pi, 200)
    x, y = 30 * np.cos(t), 30 * np.sin(t)
    hd = t + np.pi / 2
    pts = xodr.eval_param_poly3(xodr.fit_param_poly3(x, y, hd, 20.0))
    d = np.abs(np.hypot(pts[:, 0], pts[:, 1]) - 30)
    assert d.max() < 0.05
