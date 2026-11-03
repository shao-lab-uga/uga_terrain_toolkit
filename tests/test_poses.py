import numpy as np

from uga_terrain import geo, poses


def test_yaw_offset_measured_from_gnss_course():
    origin = (33.9, -83.3, 180.0)
    t = np.arange(0, 10, 0.01)
    heading = np.radians(120.0)
    e, n = 5 * t * np.cos(heading), 5 * t * np.sin(heading)
    lla = geo.enu_to_lla(e, n, np.zeros_like(t), origin)
    fixes = np.c_[t, lla]
    # INS whose x axis points 93 deg clockwise of the car's nose
    q = geo.mat_to_quat(geo.rpy_to_mat(0.0, 0.0, heading + np.radians(93.0)))
    imus = np.c_[t, np.repeat(np.atleast_2d(q), len(t), 0)]
    off, n_s, spread = poses.estimate_yaw_offset(fixes, imus, origin)
    assert n_s > 100 and spread < 1.0
    assert abs(np.degrees(off) + 93.0) < 0.5


def test_parked_recording_uses_fallback():
    origin = (33.9, -83.3, 180.0)
    t = np.arange(0, 5, 0.01)
    fixes = np.c_[t, np.tile(origin, (len(t), 1))]
    imus = np.c_[t, np.tile([0, 0, 0, 1.0], (len(t), 1))]
    cfg = {"pose": {"imu_yaw_offset_deg": "auto", "imu_yaw_offset_fallback_deg": -90.0}}
    tr = poses.build_track(fixes, imus, origin, cfg)
    assert tr.yaw_info["source"] == "fallback"
    assert abs(np.degrees(geo.mat_to_rpy(geo.quat_to_mat(tr.quat[0]))[2]) + 90) < 1e-6
