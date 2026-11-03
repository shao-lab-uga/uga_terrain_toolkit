"""End to end: a simulated 60 m drive written as a ROS 2 MCAP, built into a map."""
import xml.etree.ElementTree as ET

import pytest

from sim import write_drive
from uga_terrain import pipeline
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
