"""Output files: point cloud (PLY), elevation rasters (TIFF), CSVs, preview images."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

from uga_terrain.grid import LABELS, TerrainGrid
from uga_terrain.roads import Road

LABEL_RGB = {0: (0, 0, 0), 1: (90, 90, 96), 2: (230, 200, 90), 3: (230, 80, 60),
             4: (110, 170, 90)}


# --- point cloud ------------------------------------------------------------------------

def write_ground_ply(grid: TerrainGrid, path: Path) -> int:
    """One point per elevation cell: x y z, intensity, label, colour by label."""
    xs, ys, zs, it, lab = [], [], [], [], []
    for x, y, v in grid.iter_valid(("z", "intensity", "label")):
        xs.append(x)
        ys.append(y)
        zs.append(v["z"])
        it.append(np.nan_to_num(v["intensity"]))
        lab.append(np.nan_to_num(v["label"]).astype(np.uint8))
    if not xs:
        n = 0
        arr = np.zeros(0)
    else:
        x, y, z = np.concatenate(xs), np.concatenate(ys), np.concatenate(zs)
        lab_ = np.concatenate(lab)
        n = len(x)
        rgb = np.array([LABEL_RGB.get(i, (0, 0, 0)) for i in range(5)], np.uint8)[lab_.clip(0, 4)]
        dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                       ("label", "u1"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
        arr = np.zeros(n, dt)
        arr["x"], arr["y"], arr["z"] = x, y, z
        arr["intensity"] = np.concatenate(it)
        arr["label"] = lab_
        arr["red"], arr["green"], arr["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    header = ("ply\nformat binary_little_endian 1.0\n"
              "comment UGA terrain toolkit ground map, local ENU metres\n"
              "comment label: 1 road, 2 sidewalk, 3 curb, 4 other ground\n"
              "element vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
              "property float intensity\nproperty uchar label\nproperty uchar red\n"
              "property uchar green\nproperty uchar blue\nend_header\n" % n)
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if n:
            f.write(arr.tobytes())
    return n


def read_ply_points(path: Path, max_points: int = 0) -> np.ndarray:
    """x, y, z, label from a PLY written by :func:`write_ground_ply`."""
    raw = Path(path).read_bytes()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    n = int([ln for ln in raw[:end].decode().splitlines() if ln.startswith("element vertex")][0].split()[-1])
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                   ("label", "u1"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    arr = np.frombuffer(raw[end:end + n * dt.itemsize], dt)
    if max_points and n > max_points:
        arr = arr[:: int(math.ceil(n / max_points))]
    return arr


# --- rasters -----------------------------------------------------------------------------

def write_rasters(grid: TerrainGrid, folder: Path, layers: Sequence[str]) -> List[str]:
    """Float32 TIFF per tile and layer, each with a .tfw world file (local ENU metres)."""
    from PIL import Image
    folder = Path(folder)
    written = []
    T = grid.tile
    for layer in layers:
        d = folder / layer
        d.mkdir(parents=True, exist_ok=True)
        for (tx, ty), t in sorted(grid.tiles.items()):
            a = np.asarray(t[layer], np.float32)[::-1]       # north-up rows
            name = "tile_%d_%d" % (tx, ty)
            Image.fromarray(a, mode="F").save(d / (name + ".tif"))
            x0 = tx * T * grid.cell
            y1 = (ty + 1) * T * grid.cell
            (d / (name + ".tfw")).write_text("%.6f\n0\n0\n%.6f\n%.6f\n%.6f\n" % (
                grid.cell, -grid.cell, x0 + grid.cell / 2, y1 - grid.cell / 2))
            written.append(str(d / (name + ".tif")))
    return written


# --- colour maps and previews ----------------------------------------------------------------

_STOPS = {
    "terrain": [(0.0, (40, 70, 160)), (0.25, (60, 150, 200)), (0.45, (90, 180, 110)),
                (0.65, (220, 210, 120)), (0.85, (170, 110, 70)), (1.0, (250, 250, 250))],
    "heat": [(0.0, (20, 40, 90)), (0.35, (40, 150, 160)), (0.65, (240, 200, 70)),
             (1.0, (220, 50, 40))],
    "gray": [(0.0, (20, 20, 20)), (1.0, (245, 245, 245))],
}


def colormap(v: np.ndarray, lo: float, hi: float, name: str) -> np.ndarray:
    stops = _STOPS[name]
    f = np.clip((v - lo) / max(hi - lo, 1e-9), 0, 1)
    xs = [s for s, _ in stops]
    rgb = np.stack([np.interp(f, xs, [c[k] for _, c in stops]) for k in range(3)], -1)
    return rgb


def hillshade(z: np.ndarray, cell: float, az: float = 315.0, alt: float = 45.0) -> np.ndarray:
    zf = np.where(np.isfinite(z), z, np.nanmean(z) if np.isfinite(z).any() else 0)
    gy, gx = np.gradient(zf, cell)
    gy = -gy          # image rows run south
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    a, e = math.radians(az), math.radians(alt)
    hs = np.sin(e) * np.cos(slope) + np.cos(e) * np.sin(slope) * np.cos(a - aspect)
    return np.clip(hs, 0, 1)


def previews(grid: TerrainGrid, folder: Path, max_px: int = 4096) -> Dict[str, dict]:
    """PNG preview per layer for the app, with value ranges for legends."""
    from PIL import Image
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    out: Dict[str, dict] = {}
    z, ext = grid.mosaic("z", max_px)
    ok = np.isfinite(z)
    px = (ext[2] - ext[0]) / max(z.shape[1], 1)

    def save(name, rgb, meta):
        a = np.zeros(z.shape + (4,), np.uint8)
        a[..., :3] = np.clip(np.nan_to_num(rgb), 0, 255).astype(np.uint8)
        a[..., 3] = np.where(meta.pop("_mask"), 255, 0)
        Image.fromarray(a, "RGBA").save(folder / ("%s.png" % name), optimize=True)
        meta.update({"file": "%s.png" % name, "extent": ext, "width": z.shape[1],
                     "height": z.shape[0]})
        out[name] = meta

    if ok.any():
        lo, hi = np.nanpercentile(z, [1, 99])
        hs = hillshade(z, px)
        rgb = colormap(z, lo, hi, "terrain") * (0.45 + 0.55 * hs[..., None])
        save("elevation", rgb, {"_mask": ok, "min": float(lo), "max": float(hi), "unit": "m",
                                "label": "Elevation"})
        s, _ = grid.mosaic("slope_deg", max_px)
        save("slope", colormap(s, 0, 15, "heat"), {"_mask": ok & np.isfinite(s), "min": 0,
                                                   "max": 15, "unit": "deg", "label": "Slope"})
        r, _ = grid.mosaic("roughness", max_px)
        save("roughness", colormap(r, 0, 0.06, "heat"), {"_mask": ok & np.isfinite(r), "min": 0,
                                                       "max": 0.06, "unit": "m",
                                                       "label": "Roughness (RMS)"})
        it, _ = grid.mosaic("intensity", max_px)
        if np.isfinite(it).any():
            ilo, ihi = np.nanpercentile(it, [2, 98])
            save("intensity", colormap(it, ilo, ihi, "gray"),
                 {"_mask": ok & np.isfinite(it), "min": float(ilo), "max": float(ihi),
                  "unit": "", "label": "Lidar intensity"})
        lab, _ = grid.mosaic("label", max_px, how="max")
        li = np.nan_to_num(lab).astype(int).clip(0, 4)
        lut = np.array([LABEL_RGB[i] for i in range(5)], float)
        save("labels", lut[li] * (0.7 + 0.3 * hs[..., None]),
             {"_mask": ok, "classes": {str(k): {"name": v, "rgb": LABEL_RGB[k]}
                                       for k, v in LABELS.items() if k}, "label": "Surface"})
    return out


# --- tables -------------------------------------------------------------------------------------

PROFILE_COLS = [
    ("road", None), ("s_m", None), ("x", None), ("y", None), ("elevation_m", "z"),
    ("grade_pct", "grade_pct"), ("bank_deg", "bank_deg"),
    ("cross_slope_left_pct", "cross_slope_left_pct"),
    ("cross_slope_right_pct", "cross_slope_right_pct"),
    ("curvature_1pm", "curvature"), ("road_width_left_m", "road_l"),
    ("road_width_right_m", "road_r"), ("curb_height_left_m", "curb_h_l"),
    ("curb_height_right_m", "curb_h_r"), ("verge_left_m", "verge_l"),
    ("verge_right_m", "verge_r"), ("sidewalk_left_m", "walk_l"),
    ("sidewalk_right_m", "walk_r"), ("sidewalk_height_left_m", "walk_h_l"),
    ("sidewalk_height_right_m", "walk_h_r"), ("road_roughness_m", "roughness"),
    ("vehicle_grade_pct", "vehicle_grade_pct"), ("vehicle_bank_deg", "vehicle_bank_deg"),
]


def write_profile_csv(roads: Sequence[Road], path: Path) -> int:
    n = 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([c for c, _ in PROFILE_COLS])
        for r in roads:
            for i in range(len(r.s)):
                row = [r.id, r.s[i], r.x[i], r.y[i]]
                for _c, key in PROFILE_COLS[4:]:
                    v = r.data.get(key)
                    row.append("" if v is None or not np.isfinite(v[i]) else round(float(v[i]), 4))
                row[1:4] = [round(float(row[1]), 3), round(float(row[2]), 3), round(float(row[3]), 3)]
                w.writerow(row)
                n += 1
    return n


def write_trajectory_csv(rows: List[dict], path: Path) -> None:
    cols = ["recording", "frame", "t", "x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg",
            "gnss_x", "gnss_y", "gnss_z"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in cols})


def road_json(roads: Sequence[Road], every: int = 1) -> List[dict]:
    out = []
    for r in roads:
        sel = slice(None, None, max(1, every))
        d = {"id": r.id, "source": r.source, "length": r.length,
             "s": np.round(r.s[sel], 2).tolist(), "x": np.round(r.x[sel], 3).tolist(),
             "y": np.round(r.y[sel], 3).tolist(),
             "heading": np.round(r.heading[sel], 4).tolist()}
        for k, v in r.data.items():
            d[k] = [None if not np.isfinite(a) else round(float(a), 4) for a in v[sel]]
        out.append(d)
    return out


def save_json(obj, path: Path) -> None:
    Path(path).write_text(json.dumps(obj, indent=1, default=float))
