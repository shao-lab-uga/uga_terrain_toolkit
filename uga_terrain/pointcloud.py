"""sensor_msgs/PointCloud2 -> numpy, plus simple filters."""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

_DT = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


_TIME_FIELDS = ("t", "time", "timestamp", "offset_time")


def decode_pointcloud2(msg, with_time: bool = False) -> np.ndarray:
    """Return (N, 4) float32 x, y, z, intensity (intensity 0 when absent).

    With ``with_time``, a 5th column holds each point's time within the sweep in
    seconds (Ouster ``t`` in ns, or a float field in seconds), or NaN when the
    cloud has no per-point time."""
    end = ">" if getattr(msg, "is_bigendian", False) else "<"
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype in _DT and f.name not in names:
            names.append(f.name)
            formats.append(end + _DT[f.datatype])
            offsets.append(f.offset)
    step = int(msg.point_step)
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                   "itemsize": step})
    n = int(msg.width) * int(msg.height)
    raw = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    out = np.zeros((n, 5 if with_time else 4), dtype=np.float32)
    for k, name in enumerate(("x", "y", "z")):
        out[:, k] = raw[name]
    for name in ("intensity", "reflectivity", "i"):
        if name in names:
            out[:, 3] = raw[name]
            break
    if with_time:
        out[:, 4] = np.nan
        for name in _TIME_FIELDS:
            if name in names:
                v = raw[name]
                if np.issubdtype(v.dtype, np.integer):
                    out[:, 4] = v.astype(np.float64) * 1e-9      # nanoseconds
                else:
                    out[:, 4] = v
                    if np.nanmax(np.abs(out[:, 4])) > 10:       # absolute stamps
                        out[:, 4] -= np.nanmin(out[:, 4])
                break
    return out


def range_filter(pts: np.ndarray, rmin: float, rmax: float) -> np.ndarray:
    xyz = pts[:, :3]
    ok = np.isfinite(xyz).all(axis=1)
    r = np.linalg.norm(np.where(ok[:, None], xyz, 0), axis=1)
    return pts[ok & (r >= rmin) & (r <= rmax)]


def box_mask(xyz: np.ndarray, box: Optional[Sequence[float]]) -> np.ndarray:
    """True for points INSIDE an axis-aligned box [xmin xmax ymin ymax zmin zmax]."""
    if not box:
        return np.zeros(len(xyz), dtype=bool)
    x0, x1, y0, y1, z0, z1 = [float(v) for v in box]
    return ((xyz[:, 0] >= x0) & (xyz[:, 0] <= x1) & (xyz[:, 1] >= y0)
            & (xyz[:, 1] <= y1) & (xyz[:, 2] >= z0) & (xyz[:, 2] <= z1))


def voxel_down(pts: np.ndarray, size: float) -> np.ndarray:
    """Mean of the points in each voxel (all columns averaged)."""
    if size <= 0 or len(pts) == 0:
        return pts
    key = np.floor(pts[:, :3] / size).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    out = np.zeros((len(cnt), pts.shape[1]), dtype=np.float64)
    for c in range(pts.shape[1]):
        out[:, c] = np.bincount(inv, weights=pts[:, c], minlength=len(cnt))
    return (out / cnt[:, None]).astype(pts.dtype)
