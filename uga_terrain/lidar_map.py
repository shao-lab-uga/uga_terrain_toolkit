"""All the lidar of a map in one cloud (ground and everything above it), for the
3D view's Lidar layer and as lidar.ply.

Each 0.1 m voxel keeps the mean position and intensity of its points, whether it
is ground, and in how many scans it was hit (things that stay put are hit in many
scans; a passing car leaves a thin trail hit only a few times)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from uga_terrain import geo

VOXEL = 0.1

DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                  ("ground", "u1"), ("hits", "<u2")])


def collect(project, every: int = 1) -> np.ndarray:
    """Structured array (DTYPE) of the voxelised cloud in map coordinates."""
    from uga_terrain import register
    keys, sums, hits = [], [], []
    for i in range(len(project.recordings)):
        T = register.poses_of(project, i)
        if T is None:
            continue
        cdir = project.cache(i)
        for k in range(0, len(T), every):
            for sub, gflag in (("frames", 1.0), ("objects", 0.0)):
                f = cdir / sub / ("%06d.npy" % k)
                if not f.is_file():
                    continue
                g = np.load(f).astype(np.float64)
                if not len(g) or g.shape[1] < 4:
                    continue
                w = geo.transform(T[k], g[:, :3])
                key = np.floor(w / VOXEL).astype(np.int64)
                # one hit per voxel per scan
                u, inv = np.unique(key, axis=0, return_inverse=True)
                inv = inv.ravel()
                cnt = np.bincount(inv).astype(np.float64)
                acc = np.zeros((len(u), 6))
                for c in range(3):
                    acc[:, c] = np.bincount(inv, weights=w[:, c])
                acc[:, 3] = np.bincount(inv, weights=np.nan_to_num(g[:, 3]))
                acc[:, 4] = cnt * gflag
                acc[:, 5] = cnt
                keys.append(u)
                sums.append(acc)
                hits.append(np.ones(len(u)))
    if not keys:
        return np.zeros(0, DTYPE)
    K = np.vstack(keys)
    A = np.vstack(sums)
    Hh = np.concatenate(hits)
    u, inv = np.unique(K, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = len(u)
    tot = np.zeros((n, 6))
    for c in range(6):
        tot[:, c] = np.bincount(inv, weights=A[:, c], minlength=n)
    h = np.bincount(inv, weights=Hh, minlength=n)
    out = np.zeros(n, DTYPE)
    m = tot[:, 5]
    out["x"], out["y"], out["z"] = tot[:, 0] / m, tot[:, 1] / m, tot[:, 2] / m
    out["intensity"] = tot[:, 3] / m
    out["ground"] = (tot[:, 4] >= 0.5 * m).astype(np.uint8)
    out["hits"] = np.minimum(h, 65535).astype(np.uint16)
    return out


def write(project, path: Path) -> int:
    arr = collect(project)
    header = ("ply\nformat binary_little_endian 1.0\n"
              "comment UGA terrain toolkit lidar map (ground and everything above it), local ENU metres\n"
              "comment ground: 1 ground, 0 above the ground; hits: scans that saw this 0.1 m voxel\n"
              "element vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
              "property float intensity\nproperty uchar ground\nproperty ushort hits\nend_header\n" % len(arr))
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(arr.tobytes())
    return len(arr)


def read(path: Path, max_points: int = 0) -> np.ndarray:
    raw = Path(path).read_bytes()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    n = int([ln for ln in raw[:end].decode().splitlines() if ln.startswith("element vertex")][0].split()[-1])
    arr = np.frombuffer(raw[end:end + n * DTYPE.itemsize], DTYPE)
    if max_points and n > max_points:
        arr = arr[:: int(np.ceil(n / max_points))]
    return arr
