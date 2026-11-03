"""Simulated drive down the synthetic street (tests/synth.py) as a ROS 2 MCAP.

A 64-beam lidar is ray-cast against the street plus a parked car, a pole and
a pedestrian; GNSS + INS (OXTS-style, FRD orientation) and /tf_static are
written alongside, so the whole pipeline can run on it.
"""
from __future__ import annotations

import math

import numpy as np

import ros2_fixtures as fx
import synth
from uga_terrain import geo

ORIGIN = (33.9459, -83.3742, 186.0)
T_BASE_LIDAR = geo.make_T(geo.quat_to_mat([-0.0003, 0.0351, 0.0, 0.9994]), [1.74, 0.0, 1.762])
# boxes: xmin xmax ymin ymax zmin zmax (z relative to the road surface below)
OBJECTS = [(40.0, 44.5, -2.6, -0.8, 0.0, 1.5),     # parked car on the right lane edge
           (60.0, 60.3, 5.0, 5.3, 0.0, 6.0),       # pole on the left verge
           (75.0, 75.5, -4.6, -4.1, 0.0, 1.8)]     # pedestrian on the right sidewalk


def _ground_hit(o, d, rmax=60.0):
    """Distance along unit rays d from o to the heightfield (inf if none)."""
    n = len(d)
    t = np.full(n, np.inf)
    prev = np.zeros(n)
    above = np.ones(n, bool)
    for step in np.arange(0.5, rmax, 0.25):
        p = o + d * step
        z, _ = synth.surface(p[:, 0], p[:, 1])
        below = (p[:, 2] <= z) & above & ~np.isfinite(t)
        if below.any():
            lo, hi = prev[below], np.full(below.sum(), step)
            for _ in range(8):
                mid = 0.5 * (lo + hi)
                pm = o + d[below] * mid[:, None]
                zm, _ = synth.surface(pm[:, 0], pm[:, 1])
                b = pm[:, 2] <= zm
                hi = np.where(b, mid, hi)
                lo = np.where(b, lo, mid)
            t[np.where(below)[0]] = hi
        prev[:] = step
    return t


def _box_hit(o, d, box):
    x0, x1, y0, y1, z0, z1 = box
    zb, _ = synth.surface(np.array([(x0 + x1) / 2]), np.array([(y0 + y1) / 2]))
    lo = np.array([x0, y0, zb[0] + z0])
    hi = np.array([x1, y1, zb[0] + z1])
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo - o) / d
        t2 = (hi - o) / d
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (tmax >= tmin) & (tmax > 0)
    return np.where(hit, np.maximum(tmin, 0), np.inf)


def scan(G: np.ndarray, rng, n_az: int = 720, objects=True):
    """Ray-cast one lidar sweep; points in the lidar frame (x, y, z, intensity)."""
    W = G @ T_BASE_LIDAR
    el = np.radians(np.linspace(-22.5, 22.5, 64))
    az = np.linspace(-np.pi, np.pi, n_az, endpoint=False)
    E, A = np.meshgrid(el, az, indexing="ij")
    dl = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    dw = dl @ W[:3, :3].T
    o = W[:3, 3]
    t = _ground_hit(o, dw)
    if objects:
        for b in OBJECTS:
            t = np.minimum(t, _box_hit(o, dw, b))
    ok = np.isfinite(t) & (t < 60)
    r = t[ok] + rng.normal(0, 0.01, ok.sum())
    pts = dl[ok] * r[:, None]
    inten = np.full(len(pts), 30.0)
    return np.concatenate([pts, inten[:, None]], 1).astype(np.float32)


def vehicle_pose(x: float) -> np.ndarray:
    z, _ = synth.surface(np.array([x]), np.array([0.0]))
    pitch = -math.atan(synth.GRADE)              # FLU: nose up is negative pitch
    roll = math.radians(synth.BANK_DEG)          # left side up
    return geo.make_T(geo.rpy_to_mat(roll, pitch, 0.0), [x, 0.0, z[0]])


def write_drive(path, x0=5.0, x1=105.0, speed=8.0, rate=10.0, seed=0, objects=True,
                gnss_noise=0.0, n_az=720):
    rng = np.random.default_rng(seed)
    t0 = 1_700_000_000.0
    msgs = []
    msgs.append(("/tf_static", "tf2_msgs/msg/TFMessage", t0, fx.tf(t0, [
        ("base_footprint", "lidar", tuple(T_BASE_LIDAR[:3, 3]),
         tuple(geo.mat_to_quat(T_BASE_LIDAR[:3, :3])))])))
    dur = (x1 - x0) / speed
    for k in range(int(dur * 50) + 1):          # GNSS / INS at 50 Hz
        t = t0 + k / 50.0
        x = x0 + speed * (t - t0)
        G = vehicle_pose(x)
        e, n, u = G[:3, 3] + rng.normal(0, gnss_noise, 3) * (gnss_noise > 0)
        lla = geo.enu_to_lla(e, n, u, ORIGIN)
        msgs.append(("/oxts/fix", "sensor_msgs/msg/NavSatFix", t,
                     fx.navsatfix(t, float(lla[0]), float(lla[1]), float(lla[2]))))
        q = geo.mat_to_quat(G[:3, :3] @ np.diag([1.0, -1.0, -1.0]))   # FRD body
        msgs.append(("/oxts/imu", "sensor_msgs/msg/Imu", t, fx.imu(t, tuple(q))))
    for k in range(int(dur * rate)):
        t = t0 + k / rate
        G = vehicle_pose(x0 + speed * (t - t0))
        pts = scan(G, rng, n_az, objects)
        m = fx.pointcloud(t, pts)
        msgs.append(("/lidar/points", "sensor_msgs/msg/PointCloud2", t + 0.05, m))
    msgs.sort(key=lambda m: m[2])
    fx.write_mcap(path, msgs, chunk_size=1 << 20)
