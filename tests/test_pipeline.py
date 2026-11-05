"""End to end: a simulated 60 m drive written as a ROS 2 MCAP, built into a map."""
import xml.etree.ElementTree as ET

import pytest

from sim import write_drive
from uga_terrain import accuracy, pipeline
from uga_terrain.config import load_config


def test_build_from_mcap(tmp_path, monkeypatch):
    monkeypatch.setenv("UGA_TERRAIN_WORK_DIR", str(tmp_path / "work"))
    rec = tmp_path / "drive.mcap"
    write_drive(rec, x0=20.0, x1=80.0, speed=8.0, n_az=360)
    cfg = load_config()
    cfg["odometry"]["method"] = "none"
    p = pipeline.Project.create(cfg, "sim", [str(rec)])
    summary = pipeline.build(p, cfg)
    for f in ("sim.xodr", "road_profile.csv", "trajectory.csv", "ground.ply", "summary.json"):
        assert (p.out / f).is_file(), f
    root = ET.parse(p.out / "sim.xodr").getroot()
    assert root.findall("road")
    assert summary["roads"]
    assert pipeline.steps_available(p) == ["scan", "map", "roads", "export"]
    # the exported OpenDRIVE passes the checks and the drive replays inside one lane
    import json
    chk = json.loads((p.out / "xodr_check.json").read_text())
    assert chk["counts"]["error"] == 0, chk["findings"]
    rp = chk["replay"][0]
    assert rp["in_driving_lane_pct"] > 95 and rp["wrong_way_pct"] == 0

    # accuracy check: a simulated drive with exact poses should agree with itself
    acc = accuracy.check(p, cfg)
    assert acc["by_range"] and acc["by_range"][0]["typical_m"] < 0.03
    assert acc["halves"]["typical_m"] < 0.03
    assert acc["roads"] and acc["verdict"]
    road = pipeline.load_roads(p)[0]
    xyz, _rng, scan, _rec = accuracy.load_scans(p)
    sec = accuracy.section(p, road, float(road.s[len(road.s) // 2]), xyz, scan)
    assert len(sec["lat"]) == len(sec["dz"]) == len(sec["scan"]) > 50

    # naming a road updates the exported .xodr without a rebuild
    p.set_road_names({road.id: "Main  St "})
    names = [r.get("name") for r in ET.parse(p.out / "sim.xodr").getroot().findall("road")]
    assert "Main St" in names
    assert pipeline.Project(p.root).road_names() == {road.id: "Main St"}


def test_build_part_of_recording(tmp_path, monkeypatch):
    monkeypatch.setenv("UGA_TERRAIN_WORK_DIR", str(tmp_path / "work"))
    rec = tmp_path / "drive.mcap"
    write_drive(rec, x0=20.0, x1=80.0, speed=8.0, n_az=360)
    cfg = load_config()
    cfg["odometry"]["method"] = "none"
    p = pipeline.Project.create(cfg, "part", [str(rec)])
    p.meta["ranges"] = {str(rec): pipeline.clean_range({"unit": "s", "start": 2, "end": 5})}
    p.save()
    summary = pipeline.build(p, cfg)
    sc = summary["scan"][0]
    # the car drives 8 m/s, so 3 s of the drive is about 24 m of a 60 m road
    assert 2.0 < sc["duration_s"] <= 3.05
    assert 18 < sc["distance_m"] < 26
    assert sc["range"]["start"] == 2
