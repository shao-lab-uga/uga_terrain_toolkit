"""Lidar odometry (KISS-ICP) fused with the GNSS/INS track.

Lidar odometry is precise from frame to frame but drifts over distance;
GNSS/INS does not drift but is noisier locally. The fused pose keeps the lidar
motion and lets a smoothed GNSS/INS correction pull it back slowly
(``fuse_sigma_s``), so the map is both sharp and anchored to the world.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from uga_terrain import geo


def kiss_available() -> bool:
    try:
        from kiss_icp.kiss_icp import KissICP  # noqa: F401
        return True
    except Exception:
        return False


def resolve_method(cfg: dict) -> str:
    m = str((cfg.get("odometry") or {}).get("method") or "auto").lower()
    if m == "auto":
        return "kiss_icp" if kiss_available() else "none"
    if m == "kiss_icp" and not kiss_available():
        raise RuntimeError("odometry.method is kiss_icp but kiss-icp is not installed "
                           "(pip install kiss-icp)")
    return m


class LidarOdometry:
    """Feeds frames to KISS-ICP; returns base poses in the frame of the first scan."""

    def __init__(self, cfg: dict, T_base_lidar: np.ndarray):
        from kiss_icp.config import load_config
        from kiss_icp.kiss_icp import KissICP
        o = cfg.get("odometry") or {}
        lid = cfg.get("lidar") or {}
        kc = load_config(None)
        kc.data.max_range = float(lid.get("max_range") or 80.0)
        kc.data.min_range = float(lid.get("min_range") or 2.0)
        kc.data.deskew = False
        kc.mapping.voxel_size = float(o.get("voxel") or 1.0)
        self._cfg = kc
        self._make = lambda: KissICP(kc)
        self.L = T_base_lidar
        self.Linv = np.linalg.inv(T_base_lidar)
        self.reset()

    def reset(self) -> None:
        self.odom = self._make()
        self.n = 0

    def step(self, xyz_lidar: np.ndarray, guess_delta_base: Optional[np.ndarray] = None
             ) -> Optional[np.ndarray]:
        """Register one scan; return 4x4 base_0 <- base_i, or None on failure."""
        xyz = np.ascontiguousarray(xyz_lidar[:, :3], dtype=np.float64)
        if len(xyz) < 100:
            return None
        if guess_delta_base is not None and self.n > 0:
            try:
                self.odom.last_delta = self.Linv @ guess_delta_base @ self.L
            except Exception:
                pass
        try:
            self.odom.register_frame(xyz, np.zeros(len(xyz)))
            P = np.asarray(self.odom.last_pose, dtype=np.float64)
        except Exception:
            return None
        if not np.isfinite(P).all():
            return None
        self.n += 1
        return self.L @ P @ self.Linv


def _gauss_smooth(t: np.ndarray, v: np.ndarray, w: np.ndarray, sigma: float) -> np.ndarray:
    """Weighted Gaussian smoothing of (N, k) values on irregular times t."""
    out = np.empty_like(v)
    n = len(t)
    if n == 0:
        return out
    j0 = 0
    j1 = 0
    span = 3.0 * sigma
    for i in range(n):
        while t[j0] < t[i] - span:
            j0 += 1
        while j1 < n and t[j1] <= t[i] + span:
            j1 += 1
        dt = t[j0:j1] - t[i]
        k = np.exp(-0.5 * (dt / sigma) ** 2) * w[j0:j1]
        s = k.sum()
        out[i] = (k[:, None] * v[j0:j1]).sum(0) / s if s > 1e-12 else np.nan
    # fill gaps with the nearest smoothed value
    bad = ~np.isfinite(out).all(1)
    if bad.all():
        return np.zeros_like(v)
    if bad.any():
        good = np.where(~bad)[0]
        near = good[np.clip(np.searchsorted(good, np.arange(n)), 0, len(good) - 1)]
        out[bad] = out[near[bad]]
    return out


def fuse(t: np.ndarray, T_icp: np.ndarray, T_gnss: np.ndarray, gnss_w: np.ndarray,
         sigma_s: float) -> np.ndarray:
    """Pull lidar-odometry poses onto the GNSS/INS track at low frequency.

    ``T_icp`` (N,4,4) must already be expressed in the world frame (e.g. the
    first scan placed at its GNSS/INS pose). Returns corrected (N,4,4).
    """
    dp = T_gnss[:, :3, 3] - T_icp[:, :3, 3]
    dR = T_gnss[:, :3, :3] @ np.transpose(T_icp[:, :3, :3], (0, 2, 1))
    dw = geo.mat_to_rotvec(dR)
    corr = _gauss_smooth(t, np.concatenate([dp, dw], 1), gnss_w.astype(float), sigma_s)
    out = T_icp.copy()
    out[:, :3, :3] = geo.rotvec_to_mat(corr[:, 3:]) @ T_icp[:, :3, :3]
    out[:, :3, 3] = T_icp[:, :3, 3] + corr[:, :3]
    return out
