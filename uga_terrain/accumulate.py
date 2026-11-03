"""Merge ground points from many frames into elevation cells.

Points are binned into (cell, cell, z_bin) voxels; per voxel the count and
sums are kept relative to the voxel corner (so float32 stays exact even far
from the origin). At the end each map cell keeps its most populated height
bin and the bins next to it, which drops stray points above or below the
surface (a car bumper that slipped through, a reflection under the road).
"""
from __future__ import annotations

from typing import Dict, List

import numpy as np

_OFF = 1 << 20
_BITS = 21


def pack(ix, iy, iz) -> np.ndarray:
    return (((ix.astype(np.int64) + _OFF) << (2 * _BITS))
            | ((iy.astype(np.int64) + _OFF) << _BITS)
            | (iz.astype(np.int64) + _OFF))


def unpack(k: np.ndarray):
    m = (1 << _BITS) - 1
    return ((k >> (2 * _BITS)) & m) - _OFF, ((k >> _BITS) & m) - _OFF, (k & m) - _OFF


class VoxelAccumulator:
    def __init__(self, cell: float, z_bin: float, flush_points: int = 3_000_000):
        self.cell = float(cell)
        self.z_bin = float(z_bin)
        self.flush_points = int(flush_points)
        self.keys = np.zeros(0, np.int64)
        self.n = np.zeros(0, np.float64)
        self.s = np.zeros((0, 5), np.float64)   # dx, dy, dz, dz^2, intensity
        self._pending: List[tuple] = []
        self._pending_n = 0

    def add(self, xyzi: np.ndarray) -> None:
        if len(xyzi) == 0:
            return
        xyz = np.asarray(xyzi[:, :3], dtype=np.float64)
        ix = np.floor(xyz[:, 0] / self.cell).astype(np.int64)
        iy = np.floor(xyz[:, 1] / self.cell).astype(np.int64)
        iz = np.floor(xyz[:, 2] / self.z_bin).astype(np.int64)
        dz = xyz[:, 2] - iz * self.z_bin
        vals = np.stack([xyz[:, 0] - ix * self.cell, xyz[:, 1] - iy * self.cell, dz,
                         dz * dz, np.asarray(xyzi[:, 3], np.float64)], 1)
        self._pending.append((pack(ix, iy, iz), vals))
        self._pending_n += len(xyz)
        if self._pending_n >= self.flush_points:
            self.flush()

    def flush(self) -> None:
        if not self._pending:
            return
        keys = np.concatenate([self.keys] + [k for k, _ in self._pending])
        n = np.concatenate([self.n] + [np.ones(len(k)) for k, _ in self._pending])
        s = np.concatenate([self.s] + [v for _, v in self._pending])
        self._pending, self._pending_n = [], 0
        uk, inv = np.unique(keys, return_inverse=True)
        inv = inv.ravel()
        self.keys = uk
        self.n = np.bincount(inv, weights=n, minlength=len(uk))
        self.s = np.stack([np.bincount(inv, weights=s[:, c], minlength=len(uk))
                           for c in range(s.shape[1])], 1)

    def __len__(self) -> int:
        self.flush()
        return len(self.keys)

    def cells(self, min_points: int = 2) -> Dict[str, np.ndarray]:
        """Per map cell: ix, iy, z (mean), std, n, intensity, and centroid x/y."""
        self.flush()
        if len(self.keys) == 0:
            return {k: np.zeros(0) for k in ("ix", "iy", "z", "std", "n", "intensity", "x", "y")}
        ix, iy, iz = unpack(self.keys)
        c2 = pack(ix, iy, np.zeros_like(ix))
        # most populated z bin per cell, then keep bins within +-1 of it
        order = np.lexsort((-self.n, c2))
        c2s = c2[order]
        first = np.concatenate([[True], c2s[1:] != c2s[:-1]])
        grp = np.cumsum(first) - 1
        mode_iz = iz[order][first][grp]
        keep = np.abs(iz[order] - mode_iz) <= 1
        o = order[keep]
        g = grp[keep]
        ng = int(grp[-1]) + 1
        n = np.bincount(g, weights=self.n[o], minlength=ng)
        izo = iz[o].astype(np.float64)
        base = izo * self.z_bin
        sdz, sdz2 = self.s[o, 2], self.s[o, 3]
        sz = np.bincount(g, weights=self.n[o] * base + sdz, minlength=ng)
        szz = np.bincount(g, weights=self.n[o] * base * base + 2 * base * sdz + sdz2, minlength=ng)
        sx = np.bincount(g, weights=self.s[o, 0], minlength=ng)
        sy = np.bincount(g, weights=self.s[o, 1], minlength=ng)
        si = np.bincount(g, weights=self.s[o, 4], minlength=ng)
        cix, ciy, _ = unpack(c2s[first])
        z = sz / n
        std = np.sqrt(np.maximum(szz / n - z * z, 0.0))
        ok = n >= min_points
        return {"ix": cix[ok], "iy": ciy[ok], "z": z[ok], "std": std[ok], "n": n[ok],
                "intensity": (si / n)[ok],
                "x": (cix * self.cell + sx / n)[ok], "y": (ciy * self.cell + sy / n)[ok]}

    def save(self, path) -> None:
        self.flush()
        np.savez_compressed(path, keys=self.keys, n=self.n, s=self.s,
                            cell=self.cell, z_bin=self.z_bin)

    @classmethod
    def load(cls, path) -> "VoxelAccumulator":
        d = np.load(path)
        a = cls(float(d["cell"]), float(d["z_bin"]))
        a.keys, a.n, a.s = d["keys"], d["n"], d["s"]
        return a
