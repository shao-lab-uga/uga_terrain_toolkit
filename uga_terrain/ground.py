"""Split one lidar frame into ground and everything else.

Points arrive in the vehicle base frame (base_footprint: on the ground, z up,
parallel to the road under the car). Patchwork++ is used when installed;
otherwise a built-in version of the same idea: polar bins, a plane fitted to
the lowest points of each bin, accepted when it is near-level and continues
from the bins nearer the car.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np


def patchwork_available() -> bool:
    try:
        import pypatchworkpp  # noqa: F401
        return True
    except Exception:
        return False


class GroundSegmenter:
    def __init__(self, cfg: dict, sensor_xyz: np.ndarray):
        g = cfg.get("ground") or {}
        lid = cfg.get("lidar") or {}
        method = str(g.get("method") or "auto").lower()
        if method == "auto":
            method = "patchworkpp" if patchwork_available() else "builtin"
        self.method = method
        self.th_dist = float(g.get("th_dist") or 0.125)
        self.max_slope = math.radians(float(g.get("max_slope_deg") or 25.0))
        self.max_range = float(lid.get("ground_max_range") or 40.0)
        self.sensor = np.asarray(sensor_xyz, dtype=np.float64)
        self._pw = None
        if method == "patchworkpp":
            import pypatchworkpp
            p = pypatchworkpp.Parameters()
            p.sensor_height = float(max(self.sensor[2], 0.3))
            p.max_range = self.max_range
            p.min_range = 1.0
            p.th_dist = self.th_dist
            p.verbose = False
            self._pw = pypatchworkpp.patchworkpp(p)

    def split(self, pts_base: np.ndarray) -> np.ndarray:
        """Boolean mask of ground points for (N, >=3) points in the base frame."""
        if len(pts_base) == 0:
            return np.zeros(0, bool)
        if self._pw is not None:
            local = np.asarray(pts_base[:, :4], dtype=np.float64).copy()
            local[:, :3] -= self.sensor
            self._pw.estimateGround(local)
            mask = np.zeros(len(pts_base), bool)
            idx = np.asarray(self._pw.getGroundIndices(), dtype=np.int64)
            mask[idx[(idx >= 0) & (idx < len(mask))]] = True
            return mask
        return builtin_ground(pts_base[:, :3], self.th_dist, self.max_slope, self.max_range)


_RING_EDGES = np.array([0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.5, 15.0, 18.0, 21.0, 25.0,
                        30.0, 35.0, 40.0, 50.0, 60.0, 80.0])
_SECTORS = 48


def _fit_plane(p: np.ndarray):
    c = p.mean(axis=0)
    _u, _s, vt = np.linalg.svd(p - c, full_matrices=False)
    n = vt[2]
    if n[2] < 0:
        n = -n
    return c, n


def builtin_ground(xyz: np.ndarray, th_dist: float = 0.125,
                   max_slope: float = math.radians(25), max_range: float = 40.0,
                   max_step: float = 0.35) -> np.ndarray:
    xyz = np.asarray(xyz, dtype=np.float64)
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    ring = np.searchsorted(_RING_EDGES, r, side="right") - 1
    sec = ((np.arctan2(xyz[:, 1], xyz[:, 0]) + np.pi) / (2 * np.pi) * _SECTORS).astype(int)
    sec = np.clip(sec, 0, _SECTORS - 1)
    valid = (r <= max_range) & (ring >= 0) & (ring < len(_RING_EDGES) - 1)
    mask = np.zeros(len(xyz), bool)
    nring = len(_RING_EDGES) - 1
    bin_id = np.where(valid, ring * _SECTORS + sec, -1)
    order = np.argsort(bin_id, kind="stable")
    sorted_ids = bin_id[order]
    starts = np.searchsorted(sorted_ids, np.arange(nring * _SECTORS))
    ends = np.searchsorted(sorted_ids, np.arange(nring * _SECTORS), side="right")
    cos_max = math.cos(max_slope)
    # expected ground height per sector, carried outward ring by ring
    exp_z = np.zeros(_SECTORS)
    for ri in range(nring):
        rc = 0.5 * (_RING_EDGES[ri] + _RING_EDGES[ri + 1])
        new_exp = exp_z.copy()
        for si in range(_SECTORS):
            b = ri * _SECTORS + si
            idx = order[starts[b]:ends[b]]
            if len(idx) < 5:
                continue
            p = xyz[idx]
            z = p[:, 2]
            low = np.sort(z)[: max(3, len(z) // 5)].mean()
            seeds = p[z < low + 0.3]
            if len(seeds) < 3:
                continue
            c, n = _fit_plane(seeds)
            for _ in range(2):
                d = (p - c) @ n
                inl = p[np.abs(d) < th_dist]
                if len(inl) < 3:
                    break
                c, n = _fit_plane(inl)
            if n[2] < cos_max:
                continue
            # plane height at this bin's centre must follow from nearer bins
            phi = (si + 0.5) / _SECTORS * 2 * math.pi - math.pi
            cx, cy = rc * math.cos(phi), rc * math.sin(phi)
            zc = c[2] - (n[0] * (cx - c[0]) + n[1] * (cy - c[1])) / max(n[2], 1e-6)
            allow = max_step + math.tan(max_slope) * (_RING_EDGES[ri + 1] - _RING_EDGES[ri])
            if abs(zc - exp_z[si]) > allow:
                continue
            d = (p - c) @ n
            mask[idx[np.abs(d) < th_dist]] = True
            new_exp[si] = zc
        exp_z = new_exp
    return mask


def ground_height_near(pts_base: np.ndarray, mask: np.ndarray,
                       rmin: float = 3.0, rmax: float = 10.0) -> Optional[float]:
    """Median height of ground points near the car, for sanity checks."""
    g = pts_base[mask]
    r = np.hypot(g[:, 0], g[:, 1])
    g = g[(r > rmin) & (r < rmax)]
    return float(np.median(g[:, 2])) if len(g) > 20 else None
