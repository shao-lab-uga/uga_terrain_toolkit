"""Geodesy (WGS84 <-> local ENU) and small rotation helpers.

Quaternions are (x, y, z, w) like ROS. Rotation matrices map body -> world.
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def lla_to_ecef(lat, lon, alt):
    lat = np.radians(np.asarray(lat, dtype=np.float64))
    lon = np.radians(np.asarray(lon, dtype=np.float64))
    alt = np.asarray(alt, dtype=np.float64)
    s, c = np.sin(lat), np.cos(lat)
    n = _A / np.sqrt(1.0 - _E2 * s * s)
    return np.stack([(n + alt) * c * np.cos(lon),
                     (n + alt) * c * np.sin(lon),
                     (n * (1.0 - _E2) + alt) * s], axis=-1)


def _enu_rot(lat0: float, lon0: float) -> np.ndarray:
    la, lo = math.radians(lat0), math.radians(lon0)
    sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    return np.array([[-so, co, 0.0],
                     [-sl * co, -sl * so, cl],
                     [cl * co, cl * so, sl]])


def lla_to_enu(lat, lon, alt, origin: Tuple[float, float, float]) -> np.ndarray:
    """(N,) arrays -> (N, 3) east, north, up metres from ``origin`` (lat, lon, alt)."""
    o = lla_to_ecef(*origin)
    d = lla_to_ecef(lat, lon, alt) - o
    return d @ _enu_rot(origin[0], origin[1]).T


def enu_to_lla(e, n, u, origin: Tuple[float, float, float]) -> np.ndarray:
    """Inverse of :func:`lla_to_enu` (iterative ECEF -> geodetic)."""
    R = _enu_rot(origin[0], origin[1])
    enu = np.stack([np.asarray(e, float), np.asarray(n, float), np.asarray(u, float)], -1)
    x, y, z = (enu @ R + lla_to_ecef(*origin)).T
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - _E2))
    for _ in range(6):
        s = np.sin(lat)
        nn = _A / np.sqrt(1.0 - _E2 * s * s)
        alt = p / np.cos(lat) - nn
        lat = np.arctan2(z, p * (1.0 - _E2 * nn / (nn + alt)))
    s = np.sin(lat)
    nn = _A / np.sqrt(1.0 - _E2 * s * s)
    alt = p / np.cos(lat) - nn
    return np.stack([np.degrees(lat), np.degrees(lon), alt], -1)


# --- rotations ---------------------------------------------------------------

def quat_to_mat(q) -> np.ndarray:
    """(..., 4) -> (..., 3, 3)."""
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    m = np.empty(q.shape[:-1] + (3, 3))
    m[..., 0, 0] = 1 - 2 * (y * y + z * z)
    m[..., 0, 1] = 2 * (x * y - z * w)
    m[..., 0, 2] = 2 * (x * z + y * w)
    m[..., 1, 0] = 2 * (x * y + z * w)
    m[..., 1, 1] = 1 - 2 * (x * x + z * z)
    m[..., 1, 2] = 2 * (y * z - x * w)
    m[..., 2, 0] = 2 * (x * z - y * w)
    m[..., 2, 1] = 2 * (y * z + x * w)
    m[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return m


def mat_to_quat(m) -> np.ndarray:
    m = np.asarray(m, dtype=np.float64)
    flat = m.reshape(-1, 3, 3)
    out = np.empty((len(flat), 4))
    for i, r in enumerate(flat):
        t = r[0, 0] + r[1, 1] + r[2, 2]
        if t > 0:
            s = math.sqrt(t + 1.0) * 2
            out[i] = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                      (r[1, 0] - r[0, 1]) / s, 0.25 * s]
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            out[i] = [0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s,
                      (r[2, 1] - r[1, 2]) / s]
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            out[i] = [(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s,
                      (r[0, 2] - r[2, 0]) / s]
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            out[i] = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s,
                      (r[1, 0] - r[0, 1]) / s]
    return out.reshape(m.shape[:-2] + (4,))


def rpy_to_mat(roll, pitch, yaw) -> np.ndarray:
    """ZYX: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    m = np.empty(np.shape(roll) + (3, 3))
    m[..., 0, 0] = cy * cp
    m[..., 0, 1] = cy * sp * sr - sy * cr
    m[..., 0, 2] = cy * sp * cr + sy * sr
    m[..., 1, 0] = sy * cp
    m[..., 1, 1] = sy * sp * sr + cy * cr
    m[..., 1, 2] = sy * sp * cr - cy * sr
    m[..., 2, 0] = -sp
    m[..., 2, 1] = cp * sr
    m[..., 2, 2] = cp * cr
    return m


def mat_to_rpy(m) -> np.ndarray:
    m = np.asarray(m, dtype=np.float64)
    roll = np.arctan2(m[..., 2, 1], m[..., 2, 2])
    pitch = np.arcsin(np.clip(-m[..., 2, 0], -1.0, 1.0))
    yaw = np.arctan2(m[..., 1, 0], m[..., 0, 0])
    return np.stack([roll, pitch, yaw], -1)


def rotvec_to_mat(v) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    th = np.linalg.norm(v, axis=-1)
    k = v / np.where(th > 1e-12, th, 1.0)[..., None]
    K = np.zeros(v.shape[:-1] + (3, 3))
    K[..., 0, 1], K[..., 0, 2] = -k[..., 2], k[..., 1]
    K[..., 1, 0], K[..., 1, 2] = k[..., 2], -k[..., 0]
    K[..., 2, 0], K[..., 2, 1] = -k[..., 1], k[..., 0]
    s, c = np.sin(th)[..., None, None], np.cos(th)[..., None, None]
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def mat_to_rotvec(m) -> np.ndarray:
    m = np.asarray(m, dtype=np.float64)
    c = np.clip((np.trace(m, axis1=-2, axis2=-1) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    w = np.stack([m[..., 2, 1] - m[..., 1, 2], m[..., 0, 2] - m[..., 2, 0],
                  m[..., 1, 0] - m[..., 0, 1]], -1)
    s = np.sin(th)
    scale = np.where(s > 1e-9, th / (2 * np.where(s > 1e-9, s, 1.0)), 0.5)
    return w * scale[..., None]


def slerp(q0, q1, f) -> np.ndarray:
    """Row-wise slerp of (N, 4) quaternions with weights f (N,)."""
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float).copy()
    f = np.asarray(f, float)[..., None]
    d = np.sum(q0 * q1, axis=-1, keepdims=True)
    q1 = np.where(d < 0, -q1, q1)
    d = np.abs(d)
    th = np.arccos(np.clip(d, -1.0, 1.0))
    s = np.sin(th)
    small = s < 1e-6
    w0 = np.where(small, 1 - f, np.sin((1 - f) * th) / np.where(small, 1, s))
    w1 = np.where(small, f, np.sin(f * th) / np.where(small, 1, s))
    q = w0 * q0 + w1 * q1
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi
