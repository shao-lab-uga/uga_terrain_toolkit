"""Tiled elevation map (DEM) with slope, roughness and surface labels.

Cells are ``cell`` metres on a side in the local ENU frame; tiles hold
``tile`` x ``tile`` cells and are stored as arrays indexed [row = north, col =
east]. Only tiles that contain ground are kept, so long drives stay small.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

LABELS = {0: "none", 1: "road", 2: "sidewalk", 3: "curb", 4: "other ground"}
RAW_LAYERS = ("z_raw", "n", "std", "intensity")
LAYERS = ("z", "z_raw", "filled", "n", "std", "intensity", "slope_deg", "aspect_deg",
          "roughness", "label")

Key = Tuple[int, int]


class TerrainGrid:
    def __init__(self, cell: float, tile: int):
        self.cell = float(cell)
        self.tile = int(tile)
        self.tiles: Dict[Key, Dict[str, np.ndarray]] = {}

    # --- building -----------------------------------------------------------------

    @classmethod
    def from_cells(cls, cells: Dict[str, np.ndarray], cfg: dict, progress=None) -> "TerrainGrid":
        g = cfg.get("grid") or {}
        grid = cls(float(g.get("cell") or 0.2), int(g.get("tile") or 512))
        T = grid.tile
        ix, iy = cells["ix"].astype(np.int64), cells["iy"].astype(np.int64)
        tx, ty = np.floor_divide(ix, T), np.floor_divide(iy, T)
        raw: Dict[Key, Dict[str, np.ndarray]] = {}
        order = np.lexsort((ty, tx))
        keys = np.stack([tx[order], ty[order]], 1)
        if len(keys):
            brk = np.concatenate([[0], np.where(np.any(np.diff(keys, axis=0) != 0, 1))[0] + 1,
                                  [len(keys)]])
            for a, b in zip(brk[:-1], brk[1:]):
                sel = order[a:b]
                k = (int(tx[sel[0]]), int(ty[sel[0]]))
                t = {name: np.full((T, T), np.nan, np.float32) for name in RAW_LAYERS}
                r, c = iy[sel] - k[1] * T, ix[sel] - k[0] * T
                t["z_raw"][r, c] = cells["z"][sel]
                t["n"][r, c] = cells["n"][sel]
                t["std"][r, c] = cells["std"][sel]
                t["intensity"][r, c] = cells["intensity"][sel]
                raw[k] = t
        fill_r = int(g.get("fill_radius") or 3)
        win = int(g.get("window") or 5)
        obj_r = int(round(float(g.get("object_window_m") or 8.0) / grid.cell / 2))
        obj_h = float(g.get("object_height") or 0.4)
        halo = max(fill_r + win // 2 + 2, obj_r + 2)
        for i, k in enumerate(sorted(raw)):
            if progress is not None:
                progress.set("elevation map", i, len(raw), "tile %d/%d" % (i + 1, len(raw)))
                progress.check()
            grid.tiles[k] = _process_tile(raw, k, T, halo, grid.cell, fill_r, win, obj_r, obj_h)
        return grid

    # --- queries ------------------------------------------------------------------

    def bounds(self) -> Tuple[float, float, float, float]:
        """xmin, ymin, xmax, ymax in metres of the occupied tiles."""
        if not self.tiles:
            return (0.0, 0.0, 0.0, 0.0)
        ks = np.array(list(self.tiles))
        span = self.tile * self.cell
        return (float(ks[:, 0].min() * span), float(ks[:, 1].min() * span),
                float((ks[:, 0].max() + 1) * span), float((ks[:, 1].max() + 1) * span))

    def sample(self, x, y, layer: str = "z") -> np.ndarray:
        """Nearest-cell value at world x, y (NaN where there is no tile)."""
        x = np.asarray(x, np.float64)
        y = np.asarray(y, np.float64)
        shape = np.broadcast(x, y).shape
        x, y = np.broadcast_to(x, shape).ravel(), np.broadcast_to(y, shape).ravel()
        ix = np.floor(x / self.cell).astype(np.int64)
        iy = np.floor(y / self.cell).astype(np.int64)
        T = self.tile
        tx, ty = np.floor_divide(ix, T), np.floor_divide(iy, T)
        out = np.full(len(x), np.nan, np.float64)
        if len(x) == 0:
            return out.reshape(shape)
        kk = tx * 1_000_003 + ty
        for u in np.unique(kk):
            sel = np.where(kk == u)[0]
            k = (int(tx[sel[0]]), int(ty[sel[0]]))
            t = self.tiles.get(k)
            if t is None or layer not in t:
                continue
            out[sel] = t[layer][iy[sel] - k[1] * T, ix[sel] - k[0] * T]
        return out.reshape(shape)

    def set_cells(self, x, y, layer: str, values, only_if=None) -> None:
        """Write values at world x, y; ``only_if(old, new)`` can veto per cell."""
        x = np.asarray(x, np.float64).ravel()
        y = np.asarray(y, np.float64).ravel()
        v = np.broadcast_to(np.asarray(values), x.shape)
        ix = np.floor(x / self.cell).astype(np.int64)
        iy = np.floor(y / self.cell).astype(np.int64)
        T = self.tile
        tx, ty = np.floor_divide(ix, T), np.floor_divide(iy, T)
        kk = tx * 1_000_003 + ty
        for u in np.unique(kk):
            sel = np.where(kk == u)[0]
            k = (int(tx[sel[0]]), int(ty[sel[0]]))
            t = self.tiles.get(k)
            if t is None:
                continue
            r, c = iy[sel] - k[1] * T, ix[sel] - k[0] * T
            if only_if is not None:
                ok = only_if(t[layer][r, c], v[sel])
                r, c, vals = r[ok], c[ok], v[sel][ok]
            else:
                vals = v[sel]
            t[layer][r, c] = vals

    def iter_valid(self, layers: Iterable[str]):
        """Yield (x, y, {layer: values}) per tile for cells with an elevation."""
        T = self.tile
        for k, t in sorted(self.tiles.items()):
            ok = np.isfinite(t["z"])
            r, c = np.nonzero(ok)
            x = (k[0] * T + c + 0.5) * self.cell
            y = (k[1] * T + r + 0.5) * self.cell
            yield x, y, {name: t[name][r, c] for name in layers}

    def stats(self) -> dict:
        n = sum(int(np.isfinite(t["z"]).sum()) for t in self.tiles.values())
        lab = np.zeros(5, np.int64)
        zmin, zmax = math.inf, -math.inf
        for t in self.tiles.values():
            ok = np.isfinite(t["z"])
            if ok.any():
                zmin = min(zmin, float(np.nanmin(t["z"])))
                zmax = max(zmax, float(np.nanmax(t["z"])))
            lab += np.bincount(t["label"][ok].astype(np.int64).clip(0, 4), minlength=5)
        a = self.cell * self.cell
        return {"cells": n, "area_m2": n * a, "z_min": zmin if n else None,
                "z_max": zmax if n else None,
                "area_by_label_m2": {LABELS[i]: float(lab[i] * a) for i in range(5)}}

    # --- persistence ----------------------------------------------------------------

    def save(self, folder: Path) -> None:
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        for k, t in self.tiles.items():
            np.savez_compressed(folder / ("tile_%d_%d.npz" % k), **t)
        (folder / "grid.json").write_text(json.dumps(
            {"cell": self.cell, "tile": self.tile, "tiles": sorted(self.tiles)}))

    @classmethod
    def load(cls, folder: Path) -> "TerrainGrid":
        folder = Path(folder)
        meta = json.loads((folder / "grid.json").read_text())
        g = cls(meta["cell"], meta["tile"])
        for k in meta["tiles"]:
            d = np.load(folder / ("tile_%d_%d.npz" % tuple(k)))
            g.tiles[tuple(k)] = {name: d[name] for name in d.files}
        return g

    # --- overview images --------------------------------------------------------------

    def mosaic(self, layer: str, max_px: int = 4096, how: str = "mean"):
        """Downsampled north-up image of one layer and its extent (x0, y0, x1, y1)."""
        if not self.tiles:
            return np.zeros((1, 1), np.float32), (0, 0, 0, 0)
        ks = np.array(list(self.tiles))
        T = self.tile
        ntx = ks[:, 0].max() - ks[:, 0].min() + 1
        nty = ks[:, 1].max() - ks[:, 1].min() + 1
        f = 1
        while max(ntx, nty) * T / f > max_px and f < T:
            f *= 2
        s = T // f
        img = np.full((nty * s, ntx * s), np.nan, np.float32)
        for k, t in self.tiles.items():
            a = t[layer].astype(np.float32)
            if f > 1:
                blk = a.reshape(s, f, s, f)
                if how == "nearest":        # for categories (labels)
                    a = blk[:, f // 2, :, f // 2]
                elif how == "max":
                    with np.errstate(all="ignore"):
                        a = np.where(np.isfinite(blk).any((1, 3)),
                                     np.nanmax(np.where(np.isfinite(blk), blk, -np.inf), (1, 3)),
                                     np.nan)
                else:
                    with np.errstate(all="ignore"):
                        cnt = np.isfinite(blk).sum((1, 3))
                        a = np.where(cnt > 0, np.nansum(blk, (1, 3)) / np.maximum(cnt, 1), np.nan)
            r0 = (k[1] - ks[:, 1].min()) * s
            c0 = (k[0] - ks[:, 0].min()) * s
            img[r0:r0 + s, c0:c0 + s] = a
        span = T * self.cell
        ext = (float(ks[:, 0].min() * span), float(ks[:, 1].min() * span),
               float((ks[:, 0].max() + 1) * span), float((ks[:, 1].max() + 1) * span))
        return img[::-1], ext


# --- per-tile processing -----------------------------------------------------------

def _window(raw: Dict[Key, Dict[str, np.ndarray]], k: Key, T: int, h: int, name: str):
    """One layer of tile ``k`` with an ``h``-cell border taken from its neighbours."""
    out = np.full((T + 2 * h, T + 2 * h), np.nan, np.float32)
    spans = {-1: (slice(T - h, T), 0), 0: (slice(0, T), h), 1: (slice(0, h), h + T)}
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            t = raw.get((k[0] + dx, k[1] + dy))
            if t is None:
                continue
            (sr, r0), (sc, c0) = spans[dy], spans[dx]
            blk = t[name][sr, sc]
            out[r0:r0 + blk.shape[0], c0:c0 + blk.shape[1]] = blk
    return out


def _shift(a: np.ndarray, dr: int, dc: int) -> np.ndarray:
    """out[r, c] = a[r + dr, c + dc] (NaN outside)."""
    out = np.full_like(a, np.nan)
    R, C = a.shape
    rs, re = max(0, -dr), min(R, R - dr)
    cs, ce = max(0, -dc), min(C, C - dc)
    out[rs:re, cs:ce] = a[rs + dr:re + dr, cs + dc:ce + dc]
    return out


def fill_holes(z: np.ndarray, radius: int) -> Tuple[np.ndarray, np.ndarray]:
    """Fill empty cells surrounded by data (3+ of 4 quadrants within ``radius``)."""
    ok = np.isfinite(z)
    num = np.zeros_like(z, dtype=np.float64)
    den = np.zeros_like(z, dtype=np.float64)
    quad = np.zeros((4,) + z.shape, bool)
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if dr == 0 and dc == 0:
                continue
            s = _shift(z, dr, dc)
            v = np.isfinite(s)
            w = 1.0 / (dr * dr + dc * dc)
            num += np.where(v, s, 0) * w
            den += v * w
            q = (0 if dr >= 0 else 2) + (0 if dc >= 0 else 1)
            quad[q] |= v
    can = (~ok) & (quad.sum(0) >= 3) & (den > 0)
    out = z.copy()
    out[can] = (num[can] / den[can]).astype(z.dtype)
    return out, can


def _box(a: np.ndarray, r: int) -> np.ndarray:
    """Sum over a (2r+1)^2 window (zero padding), via an integral image."""
    p = np.pad(a, r + 1)
    ii = p.cumsum(0).cumsum(1)
    w = 2 * r + 1
    R, C = a.shape
    return (ii[w:w + R, w:w + C] - ii[0:R, w:w + C] - ii[w:w + R, 0:C] + ii[0:R, 0:C])


def plane_fit(z: np.ndarray, cell: float, win: int):
    """Local plane per cell over a win x win window: slope (dz/dx, dz/dy), RMS residual."""
    r = win // 2
    R, C = z.shape
    ok = np.isfinite(z).astype(np.float64)
    zz = np.where(ok > 0, z, 0.0).astype(np.float64)
    yy, xx = np.mgrid[0:R, 0:C].astype(np.float64) * cell
    S1 = _box(ok, r)
    Sx, Sy = _box(xx * ok, r), _box(yy * ok, r)
    Sxx, Syy, Sxy = _box(xx * xx * ok, r), _box(yy * yy * ok, r), _box(xx * yy * ok, r)
    Sz, Sxz, Syz = _box(zz, r), _box(xx * zz, r), _box(yy * zz, r)
    Szz = _box(zz * zz, r)
    A = np.stack([np.stack([S1, Sx, Sy], -1), np.stack([Sx, Sxx, Sxy], -1),
                  np.stack([Sy, Sxy, Syy], -1)], -2)
    bvec = np.stack([Sz, Sxz, Syz], -1)
    good = (S1 >= 6) & (ok > 0)
    coef = np.full((R, C, 3), np.nan)
    if good.any():
        Ag = A[good]
        det = np.linalg.det(Ag)
        solv = np.abs(det) > 1e-9
        sol = np.full((len(Ag), 3), np.nan)
        sol[solv] = np.linalg.solve(Ag[solv], bvec[good][solv][..., None])[..., 0]
        coef[good] = sol
    a, b, c = coef[..., 0], coef[..., 1], coef[..., 2]
    sse = (Szz - 2 * (a * Sz + b * Sxz + c * Syz)
           + a * a * S1 + b * b * Sxx + c * c * Syy
           + 2 * a * b * Sx + 2 * a * c * Sy + 2 * b * c * Sxy)
    rms = np.sqrt(np.maximum(sse, 0) / np.maximum(S1, 1))
    return b, c, rms


def smooth3(z: np.ndarray) -> np.ndarray:
    k = {(-1, -1): 1, (-1, 0): 2, (-1, 1): 1, (0, -1): 2, (0, 0): 4, (0, 1): 2,
         (1, -1): 1, (1, 0): 2, (1, 1): 1}
    num = np.zeros(z.shape)
    den = np.zeros(z.shape)
    for (dr, dc), w in k.items():
        s = _shift(z, dr, dc)
        v = np.isfinite(s)
        num += np.where(v, s, 0) * w
        den += v * w
    out = np.where(np.isfinite(z) & (den > 0), num / np.maximum(den, 1e-12), np.nan)
    return out.astype(np.float32)


def remove_raised(z: np.ndarray, r: int, height: float, rounds: int = 2) -> np.ndarray:
    """Drop cells standing ``height`` above the mean surface around them.

    Ground segmentation can let the flat roof of a car or a low wall through;
    within a window much larger than the object, its cells stick out above the
    surrounding road, while slopes and curbs do not.
    """
    z = z.copy()
    for _ in range(rounds):
        ok = np.isfinite(z)
        if r <= 0 or not ok.any():
            break
        s = _box(np.where(ok, z, 0.0).astype(np.float64), r)
        c = _box(ok.astype(np.float64), r)
        mean = np.where(c > 0, s / np.maximum(c, 1), np.nan)
        bad = ok & (z > mean + height)
        if not bad.any():
            break
        z[bad] = np.nan
    return z


def _process_tile(raw, k: Key, T: int, h: int, cell: float, fill_r: int, win: int,
                  obj_r: int = 20, obj_h: float = 0.4):
    zr = remove_raised(_window(raw, k, T, h, "z_raw"), obj_r, obj_h)
    zf, filled = fill_holes(zr, fill_r)
    gx, gy, rough = plane_fit(zf, cell, win)
    # texture at two scales: bumps between cells (plane residual) and the
    # spread of points inside each cell (grass, gravel); sensor noise adds to both
    sd = _window(raw, k, T, h, "std").astype(np.float64)
    okd = np.isfinite(sd)
    r_ = win // 2
    mstd2 = _box(np.where(okd, sd * sd, 0.0), r_) / np.maximum(_box(okd.astype(float), r_), 1)
    rough = np.sqrt(rough * rough + mstd2)
    z = smooth3(zf)
    inner = (slice(h, h + T), slice(h, h + T))
    t = {name: raw[k][name] for name in RAW_LAYERS}
    t["z_raw"] = zr[inner]
    t["z"] = z[inner]
    t["filled"] = filled[inner].astype(np.float32)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    aspect = (np.degrees(np.arctan2(-gx, -gy)) + 360.0) % 360.0   # downhill direction, from north
    t["slope_deg"] = slope[inner].astype(np.float32)
    t["aspect_deg"] = aspect[inner].astype(np.float32)
    t["roughness"] = rough[inner].astype(np.float32)
    t["roughness"][~np.isfinite(t["z"])] = np.nan
    lab = np.zeros((T, T), np.float32)
    lab[np.isfinite(t["z"])] = 4
    t["label"] = lab
    return t
