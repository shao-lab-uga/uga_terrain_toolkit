"""Tidy the surface after the roads are found, so the map reads as drivable.

* the junction (inside its curb-fitted outline) is labelled road;
* small gaps in the scan are filled: up to ``hole_m`` (6 m) across anywhere, and up
  to ``road_hole_m`` across on roads and in the junction (parked cars, people
  and the like leave holes there). Filled cells are marked in the ``filled``
  layer (value 2), so accuracy figures can still leave them out, and they are
  emptied again before the roads are found the next time;
* speckle in the labels (single cells of "other ground" inside a road, or
  sidewalk specks in the grass) is removed with a majority filter.

Only cells with no lidar ground get new heights; measured heights are never
changed.
"""
from __future__ import annotations

from typing import Dict, Sequence, Tuple

import numpy as np

from uga_terrain.grid import TerrainGrid, _box

TIDY_FILLED = 2.0      # value in the "filled" layer of cells filled here (1 = filled while gridding)


def _assemble(grid: TerrainGrid, layers: Sequence[str]):
    ks = np.array(list(grid.tiles))
    T = grid.tile
    kx0, ky0 = int(ks[:, 0].min()), int(ks[:, 1].min())
    nx, ny = int(ks[:, 0].max()) - kx0 + 1, int(ks[:, 1].max()) - ky0 + 1
    out = {}
    for name in layers:
        a = np.full((ny * T, nx * T), np.nan, np.float32)
        for k, t in grid.tiles.items():
            r0, c0 = (k[1] - ky0) * T, (k[0] - kx0) * T
            a[r0:r0 + T, c0:c0 + T] = t[name]
        out[name] = a
    return out, (kx0, ky0)


def _scatter(grid: TerrainGrid, arrays: Dict[str, np.ndarray], origin: Tuple[int, int]) -> None:
    T = grid.tile
    for k, t in grid.tiles.items():
        r0, c0 = (k[1] - origin[1]) * T, (k[0] - origin[0]) * T
        for name, a in arrays.items():
            t[name] = a[r0:r0 + T, c0:c0 + T].astype(np.float32).copy()


def _nearest_along(valid: np.ndarray, z: np.ndarray, axis: int, reverse: bool):
    """Distance (cells) to and height of the nearest valid cell in one direction."""
    v, a = valid, z
    if axis == 0:
        v, a = v.T, a.T
    if reverse:
        v, a = v[:, ::-1], a[:, ::-1]
    n = v.shape[1]
    idx = np.where(v, np.arange(n)[None, :], -1)
    last = np.maximum.accumulate(idx, axis=1)
    dist = np.where(last >= 0, np.arange(n)[None, :] - last, np.iinfo(np.int32).max).astype(np.float64)
    val = np.take_along_axis(a, np.maximum(last, 0), axis=1).astype(np.float64)
    if reverse:
        dist, val = dist[:, ::-1], val[:, ::-1]
    if axis == 0:
        dist, val = dist.T, val.T
    return dist, val


def fill_gaps(z: np.ndarray, limit_cells: np.ndarray):
    """Fill empty cells that have ground on all four sides (left, right, up,
    down) within ``limit_cells`` (per cell). Heights blend the four nearest
    measured cells by inverse distance. Returns (z, filled mask)."""
    valid = np.isfinite(z)
    num = np.zeros(z.shape, np.float64)
    den = np.zeros(z.shape, np.float64)
    ok = ~valid
    for axis in (0, 1):
        for rev in (False, True):
            d, v = _nearest_along(valid, z, axis, rev)
            ok &= d <= limit_cells
            w = 1.0 / np.maximum(d, 1.0)
            good = d < np.iinfo(np.int32).max
            num += np.where(good, v * w, 0.0)
            den += np.where(good, w, 0.0)
    out = z.copy()
    out[ok] = (num[ok] / den[ok]).astype(z.dtype)
    return out, ok


def _inside(poly: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Even-odd point-in-polygon for arrays of points."""
    inside = np.zeros(x.shape, bool)
    px, py = poly[:, 0], poly[:, 1]
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi, xj, yj = px[i], py[i], px[j], py[j]
        cross = (yi > y) != (yj > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xc = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= cross & (x < xc)
        j = i
    return inside


def _count8(mask: np.ndarray) -> np.ndarray:
    return _box(mask.astype(np.float64), 1) - mask


def untidy(grid: TerrainGrid) -> None:
    """Empty the cells an earlier :func:`tidy` filled (so road finding sees only lidar)."""
    for t in grid.tiles.values():
        m = t["filled"] == TIDY_FILLED
        if m.any():
            for name in ("z", "slope_deg", "aspect_deg"):
                t[name][m] = np.nan
            t["filled"][m] = 0.0


def tidy(grid: TerrainGrid, juncs, cfg: dict) -> dict:
    """Label the junction as road, fill gaps and remove label speckle (in place)."""
    if not grid.tiles:
        return {}
    tc = cfg.get("tidy") or {}
    if tc.get("enabled", True) is False:
        return {}
    cell = grid.cell
    A, origin = _assemble(grid, ("z", "label", "filled"))
    z, lab, filled = A["z"], A["label"], A["filled"]
    R, C = z.shape
    x0, y0 = origin[0] * grid.tile * cell, origin[1] * grid.tile * cell
    # junction areas, from their curb-fitted outlines
    junc = np.zeros((R, C), bool)
    for jn in juncs or []:
        ol = np.asarray(getattr(jn, "outline", None) or [], np.float64)
        if len(ol) < 3:
            continue
        c0 = max(int((ol[:, 0].min() - x0) / cell), 0)
        c1 = min(int((ol[:, 0].max() - x0) / cell) + 1, C)
        r0 = max(int((ol[:, 1].min() - y0) / cell), 0)
        r1 = min(int((ol[:, 1].max() - y0) / cell) + 1, R)
        if c1 <= c0 or r1 <= r0:
            continue
        cc, rr = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
        junc[r0:r1, c0:c1] |= _inside(ol, x0 + (cc + 0.5) * cell, y0 + (rr + 0.5) * cell)
    # road area: road cells and the junction, closed by about 1 m so speckle and gaps join up
    rc = max(1, int(round(1.0 / cell)))
    road = (lab == 1) | junc
    area = _box(road.astype(np.float64), rc) > 0.5
    area = _box((~area).astype(np.float64), rc) < 0.5
    area |= junc
    # gaps
    hole = float(tc.get("hole_m", 6.0)) / 2 / cell
    road_hole = float(tc.get("road_hole_m", 15.0)) / 2 / cell
    lim = np.where(area, road_hole, hole)
    was = np.isfinite(z)
    z2, new = fill_gaps(z, lim)
    z[:] = z2
    filled[new] = TIDY_FILLED
    lab[new] = 4.0
    # labels: the junction and the road area are road (curbs stay curbs)
    valid = np.isfinite(z)
    lab[valid & area & ((lab == 4) | (lab == 0) | np.isnan(lab))] = 1.0
    lab[valid & junc & (lab == 2)] = 1.0
    # speckle: a cell whose 8 neighbours are mostly one other surface takes it
    changed = 0
    for _ in range(2):
        counts = {k: _count8(valid & (lab == k)) for k in (1, 2, 4)}
        for k, n in counts.items():
            flip = valid & (lab != k) & (lab != 3) & (n >= 6)
            changed += int(flip.sum())
            lab[flip] = float(k)
    lab[~valid] = 0.0
    out = {"z": z, "label": lab, "filled": filled}
    if new.any():
        # slope and aspect of the filled cells from the filled heights (roughness stays unknown)
        A2, _ = _assemble(grid, ("slope_deg", "aspect_deg"))
        gy, gx = np.gradient(z.astype(np.float64), cell)
        A2["slope_deg"][new] = np.degrees(np.arctan(np.hypot(gx, gy)))[new]
        A2["aspect_deg"][new] = ((np.degrees(np.arctan2(-gx, -gy)) + 360.0) % 360.0)[new]
        out.update(A2)
    _scatter(grid, out, origin)
    a = cell * cell
    return {"filled_m2": float(new.sum() * a), "filled_road_m2": float((new & area).sum() * a),
            "junction_m2": float((junc & valid).sum() * a), "relabelled_cells": changed,
            "had_ground_m2": float(was.sum() * a)}
