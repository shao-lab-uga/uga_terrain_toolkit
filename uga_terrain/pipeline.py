"""From .mcap recordings to a terrain map, road profiles and an OpenDRIVE file.

Steps (each can be re-run on its own once the ones before it have run):

1. ``scan``   per recording: poses from GNSS/INS (+ lidar odometry), ground
              points of every lidar frame cached in the vehicle frame.
2. ``map``    all cached ground placed with the final poses and merged into
              one elevation map (all recordings together).
3. ``roads``  roads, curbs and sidewalks from cross-sections along the drives.
4. ``export`` OpenDRIVE, CSVs, PLY point cloud, rasters, preview images.

A project lives in ``<work_dir>/projects/<name>/``.
"""
from __future__ import annotations

import hashlib
import json
import re
import shutil
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from uga_terrain import export, geo, poses, roads as roads_mod, xodr
from uga_terrain.accumulate import VoxelAccumulator
from uga_terrain.config import work_dir
from uga_terrain.grid import TerrainGrid
from uga_terrain.ground import GroundSegmenter
from uga_terrain.mcap_io import inspect, iter_messages, stamp_of
from uga_terrain.odometry import LidarOdometry, fuse, resolve_method
from uga_terrain.pointcloud import box_mask, decode_pointcloud2, range_filter, voxel_down
from uga_terrain.progress import Progress, maybe

STEPS = ("scan", "map", "roads", "export")


def _slug(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip()).strip("._")
    return s or "map"


class Project:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.meta_path = self.root / "project.json"
        self.meta = json.loads(self.meta_path.read_text()) if self.meta_path.is_file() else {}

    @property
    def name(self) -> str:
        return self.meta.get("name") or self.root.name

    @property
    def recordings(self) -> List[str]:
        return list(self.meta.get("recordings") or [])

    def save(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        self.meta_path.write_text(json.dumps(self.meta, indent=1))

    @classmethod
    def create(cls, cfg: dict, name: str, recordings: Sequence[str]) -> "Project":
        root = work_dir(cfg) / "projects" / _slug(name)
        p = cls(root)
        if not p.meta:
            p.meta = {"name": name, "created": time.strftime("%Y-%m-%d %H:%M:%S")}
        p.meta["recordings"] = [str(Path(r).expanduser()) for r in recordings]
        p.save()
        return p

    @classmethod
    def open(cls, cfg: dict, name: str) -> "Project":
        root = work_dir(cfg) / "projects" / _slug(name)
        if not (root / "project.json").is_file():
            raise FileNotFoundError("no map called %r" % name)
        return cls(root)

    def cache(self, i: int) -> Path:
        return self.root / "cache" / ("rec_%02d" % i)

    @property
    def out(self) -> Path:
        return self.root / "outputs"

    def status(self) -> dict:
        st = {s: (self.root / (".done_%s" % s)).is_file() for s in STEPS}
        summary = self.out / "summary.json"
        return {"name": self.name, "slug": self.root.name, "recordings": self.recordings,
                "created": self.meta.get("created"), "origin": self.meta.get("origin"),
                "steps": st, "built": self.meta.get("built"),
                "summary": json.loads(summary.read_text()) if summary.is_file() else None}

    def mark(self, step: str, done: bool = True) -> None:
        f = self.root / (".done_%s" % step)
        if done:
            f.write_text(time.strftime("%Y-%m-%d %H:%M:%S"))
        elif f.exists():
            f.unlink()


def list_projects(cfg: dict) -> List[dict]:
    root = work_dir(cfg) / "projects"
    if not root.is_dir():
        return []
    out = []
    for d in sorted(root.iterdir()):
        if (d / "project.json").is_file():
            try:
                out.append(Project(d).status())
            except (OSError, ValueError):
                continue
    return out


# --- step 1: scan --------------------------------------------------------------------------

_SCAN_KEYS = ("topics", "frames", "pose", "lidar", "odometry", "ground")
SCAN_VERSION = 2   # bump when scan output changes, so old caches are rebuilt


def _scan_key(path: str, cfg: dict, origin) -> str:
    st = Path(path).stat()
    blob = json.dumps({"v": SCAN_VERSION, "p": str(path), "size": st.st_size, "mtime": int(st.st_mtime),
                       "origin": origin, "cfg": {k: cfg.get(k) for k in _SCAN_KEYS}},
                      sort_keys=True, default=str)
    return hashlib.sha1(blob.encode()).hexdigest()[:16]


def _rot_angle(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def scan_recording(path: str, cfg: dict, origin, out: Path, progress: Progress,
                   label: str = "") -> dict:
    topics = cfg.get("topics") or {}
    frames_cfg = cfg.get("frames") or {}
    lid = cfg.get("lidar") or {}
    gcfg = cfg.get("ground") or {}
    lidar_topic = topics.get("lidar", "/lidar/points")
    progress.set("read poses", 0, 1, "%s GNSS / INS" % label)
    edges = poses.read_static_tf(path, topics.get("tf_static", "/tf_static"))
    base_f, lidar_f = frames_cfg.get("base", "base_footprint"), frames_cfg.get("lidar", "lidar")
    T_bl = np.eye(4) if base_f == lidar_f else poses.lookup(edges, base_f, lidar_f)
    if T_bl is None and frames_cfg.get("base_to_lidar"):
        bl = frames_cfg["base_to_lidar"]
        T_bl = geo.make_T(geo.quat_to_mat(bl["rotation_xyzw"]), bl["translation"])
        progress.note("%s: no %s -> %s in /tf_static; using frames.base_to_lidar from Settings"
                      % (Path(path).name, base_f, lidar_f))
    if T_bl is None:
        raise RuntimeError(
            "%s: no static transform from %s to %s (frames found: %s). Set frames.base / "
            "frames.lidar in Settings." % (Path(path).name, base_f, lidar_f,
                                            ", ".join(sorted(edges)) or "none"))
    fixes, imus, nav_topic = poses.read_gnss_imu(path, cfg)
    if len(fixes) == 0:
        raise RuntimeError("%s: no GNSS fixes on %s" % (
            Path(path).name, ", ".join(poses.navsat_candidates(cfg))))
    if nav_topic != topics.get("navsat"):
        progress.note("%s: GNSS read from %s (%s not in this recording)"
                      % (Path(path).name, nav_topic, topics.get("navsat")))
    track = poses.build_track(fixes, imus, origin, cfg)
    yi = getattr(track, "yaw_info", {}) or {}
    if yi.get("source") == "measured":
        progress.note("%s: INS heading lined up with the GNSS course: %+.1f deg "
                      "(spread %.1f deg over %d samples)" % (Path(path).name, yi["offset_deg"],
                                                             yi["spread_deg"], yi["samples"]))
    elif yi.get("source") == "fallback":
        progress.note("%s: the car never drove fast enough to check the INS heading; using "
                      "pose.imu_yaw_offset_fallback_deg = %.1f" % (Path(path).name,
                                                                   yi["offset_deg"]))
    seg = GroundSegmenter(cfg, T_bl[:3, 3])
    method = resolve_method(cfg)
    odo = LidarOdometry(cfg, T_bl) if method == "kiss_icp" else None
    info = inspect(path, scan=False)
    ch = info.channels.get(lidar_topic)
    total = (ch.count if ch and ch.count else
             max(1, int((track.t[-1] - track.t[0]) * 10)))
    use_header = str((cfg.get("pose") or {}).get("time_source") or "header") == "header"
    min_move = float(lid.get("min_motion_m") or 0.0)
    min_turn = float(lid.get("min_turn_deg") or 0.0)
    fstep = max(1, int(lid.get("frame_step") or 1))
    gmax = float(lid.get("ground_max_range") or 40.0)
    gvox = float(gcfg.get("voxel") or 0.1)
    ego = lid.get("ego_box")
    fdir = out / "frames"
    if fdir.exists():
        shutil.rmtree(fdir)
    fdir.mkdir(parents=True)
    ts, Gs, Bs, oks = [], [], [], []
    heights = []
    anchor = None
    last_G = None
    n_seen = 0
    n_icp_reset = 0
    for t_log, _topic, msg in iter_messages(path, [lidar_topic]):
        n_seen += 1
        if n_seen % 10 == 0 or n_seen == 1:
            progress.set("scan lidar", n_seen, total, "%s frame %d, kept %d" % (label, n_seen, len(ts)))
        progress.check()
        if (n_seen - 1) % fstep:
            continue
        t = stamp_of(msg, t_log) if use_header else t_log
        if not track.covers(t, 0.2):
            continue
        p, q = track.interp(t)
        G = poses.pose_matrix(p, q)[0]
        if last_G is not None:
            d = np.linalg.norm(G[:3, 3] - last_G[:3, 3])
            ang = _rot_angle(last_G[:3, :3].T @ G[:3, :3])
            if d < min_move and ang < min_turn:
                continue
        pts = range_filter(decode_pointcloud2(msg), float(lid.get("min_range") or 2.0),
                           float(lid.get("max_range") or 80.0))
        if len(pts) < 100:
            continue
        base = pts.copy()
        base[:, :3] = geo.transform(T_bl, pts[:, :3].astype(np.float64))
        keep = ~box_mask(base[:, :3], ego)
        base, pts = base[keep], pts[keep]
        gmask = seg.split(base)
        g = base[gmask]
        g = g[np.hypot(g[:, 0], g[:, 1]) <= gmax]
        g = voxel_down(g, gvox)
        k = len(ts)
        np.save(fdir / ("%06d.npy" % k), g.astype(np.float32))
        near = g[(np.hypot(g[:, 0], g[:, 1]) > 3) & (np.hypot(g[:, 0], g[:, 1]) < 10)]
        if len(near) > 20:
            heights.append(float(np.median(near[:, 2])))
        B = G
        ok = False
        if odo is not None:
            guess = np.linalg.inv(last_G) @ G if last_G is not None else None
            rel = odo.step(pts[:, :3], guess)
            if rel is None:
                odo.reset()
                rel = odo.step(pts[:, :3], None)
                anchor = None
            if rel is not None:
                if anchor is None:
                    anchor = G @ np.linalg.inv(rel)
                B = anchor @ rel
                ok = True
                if Bs and oks[-1]:
                    # lidar odometry must agree with GNSS/INS on this step's motion;
                    # long featureless roads can make it slide (or stick)
                    d_icp = np.linalg.inv(Bs[-1]) @ B
                    d_gnss = guess
                    dt_err = np.linalg.norm(d_icp[:3, 3] - d_gnss[:3, 3])
                    da_err = _rot_angle(d_icp[:3, :3].T @ d_gnss[:3, :3])
                    move = np.linalg.norm(d_gnss[:3, 3])
                    if dt_err > max(0.3, 0.25 * move) or da_err > 2.0:
                        anchor = G @ np.linalg.inv(rel)
                        B = G
                        n_icp_reset += 1
                elif Bs:
                    anchor = G @ np.linalg.inv(rel)
                    B = G
        ts.append(t)
        Gs.append(G)
        Bs.append(B)
        oks.append(ok)
        last_G = G
    if not ts:
        raise RuntimeError("%s: no lidar frames on %s overlap the GNSS track"
                           % (Path(path).name, lidar_topic))
    t_arr = np.asarray(ts)
    G_arr, B_arr = np.asarray(Gs), np.asarray(Bs)
    w = np.interp(t_arr, track.t, track.gnss_ok.astype(float))
    sigma = float((cfg.get("odometry") or {}).get("fuse_sigma_s") or 8.0)
    T_final = fuse(t_arr, B_arr, G_arr, w, sigma) if odo is not None and any(oks) else G_arr
    if odo is not None and n_icp_reset > 0.3 * len(ts):
        progress.note("%s: lidar odometry disagreed with GNSS/INS on %d of %d frames; those "
                      "frames use GNSS/INS poses" % (Path(path).name, n_icp_reset, len(ts)))
    np.savez_compressed(out / "frames.npz", t=t_arr, G=G_arr, B=B_arr, T=T_final,
                        icp_ok=np.asarray(oks))
    dev = np.linalg.norm(T_final[:, :3, 3] - G_arr[:, :3, 3], axis=1)
    stats = {
        "recording": str(path), "frames": len(ts), "lidar_messages": n_seen,
        "duration_s": float(t_arr[-1] - t_arr[0]),
        "distance_m": float(np.sum(np.linalg.norm(np.diff(T_final[:, :3, 3], axis=0), axis=1))),
        "pose_source": track.source, "odometry": method, "odometry_resets": n_icp_reset,
        "ground_method": seg.method,
        "gnss_vs_final_m": {"median": float(np.median(dev)), "max": float(np.max(dev))},
        "ground_height_under_car_m": float(np.median(heights)) if heights else None,
    }
    return stats


def step_scan(project: Project, cfg: dict, progress: Progress) -> List[dict]:
    recs = project.recordings
    if not recs:
        raise RuntimeError("this map has no recordings")
    origin = project.meta.get("origin")
    out = []
    for i, path in enumerate(recs):
        if not Path(path).is_file():
            raise FileNotFoundError("recording not found: %s" % path)
        if origin is None:
            fixes, _, _ = poses.read_gnss_imu(path, cfg)
            if len(fixes) == 0:
                continue
            origin = list(poses.first_fix(fixes))
            project.meta["origin"] = origin
            project.save()
        cdir = project.cache(i)
        key = _scan_key(path, cfg, origin)
        meta_f = cdir / "meta.json"
        if meta_f.is_file() and (cdir / "frames.npz").is_file():
            meta = json.loads(meta_f.read_text())
            if meta.get("key") == key:
                progress.note("%s: using cached scan" % Path(path).name)
                out.append(meta["stats"])
                continue
        cdir.mkdir(parents=True, exist_ok=True)
        label = "[%d/%d] %s" % (i + 1, len(recs), Path(path).name)
        stats = scan_recording(path, cfg, tuple(origin), cdir, progress, label)
        meta_f.write_text(json.dumps({"key": key, "stats": stats}, indent=1))
        out.append(stats)
        h = stats.get("ground_height_under_car_m")
        if h is not None and abs(h) > 0.3:
            progress.note("%s: ground under the car sits %.2f m from the base frame; check "
                          "frames.base and /tf_static" % (Path(path).name, h))
    if origin is None:
        raise RuntimeError("no recording has GNSS fixes")
    # drop caches of recordings no longer in the map
    for d in (project.root / "cache").glob("rec_*"):
        if int(d.name[4:]) >= len(recs):
            shutil.rmtree(d, ignore_errors=True)
    project.meta["scan"] = out
    project.save()
    return out


# --- step 2: map --------------------------------------------------------------------------

def step_map(project: Project, cfg: dict, progress: Progress) -> TerrainGrid:
    g = cfg.get("grid") or {}
    acc = VoxelAccumulator(float(g.get("cell") or 0.2), float(g.get("z_bin") or 0.1))
    recs = project.recordings
    for i in range(len(recs)):
        fz = project.cache(i) / "frames.npz"
        if not fz.is_file():
            continue
        T = np.load(fz)["T"]
        for k in range(len(T)):
            if k % 20 == 0:
                progress.set("merge ground", k, len(T), "[%d/%d] frame %d" % (i + 1, len(recs), k))
                progress.check()
            f = project.cache(i) / "frames" / ("%06d.npy" % k)
            if not f.is_file():
                continue
            pts = np.load(f).astype(np.float64)
            if len(pts):
                pts[:, :3] = geo.transform(T[k], pts[:, :3])
                acc.add(pts)
    progress.set("merge ground", 1, 1, "%d height bins" % len(acc))
    cells = acc.cells(min_points=2)
    grid = TerrainGrid.from_cells(cells, cfg, progress)
    gdir = project.root / "grid"
    if gdir.exists():
        shutil.rmtree(gdir)
    grid.save(gdir)
    return grid


# --- step 3: roads ---------------------------------------------------------------------------

def step_roads(project: Project, cfg: dict, progress: Progress, grid: Optional[TerrainGrid] = None):
    grid = grid or TerrainGrid.load(project.root / "grid")
    for t in grid.tiles.values():      # labels are recomputed from scratch
        t["label"] = np.where(np.isfinite(t["z"]), 4, 0).astype(np.float32)
    paths, frames = [], {}
    for i, rec in enumerate(project.recordings):
        fz = project.cache(i) / "frames.npz"
        if not fz.is_file():
            continue
        T = np.load(fz)["T"]
        name = "%d:%s" % (i, Path(rec).name)
        paths.append((name, T[:, :2, 3]))
        frames[name] = T
    progress.set("find roads", 0, 1, "%d drives" % len(paths))
    found = roads_mod.split_roads(paths, cfg)
    for j, r in enumerate(found):
        progress.set("find roads", j, len(found), "road %d, %.0f m" % (r.id, r.length))
        progress.check()
        T = frames[r.source]
        xy = T[:, :2, 3]
        idx = np.empty(len(r.s), np.int64)
        for a in range(0, len(r.s), 512):
            P = np.stack([r.x[a:a + 512], r.y[a:a + 512]], 1)
            idx[a:a + 512] = np.argmin(((P[:, None, :] - xy[None, :, :]) ** 2).sum(-1), 1)
        rpy = geo.mat_to_rpy(T[idx, :3, :3])
        roads_mod.analyse_road(r, grid, cfg, vehicle_rpy=(rpy[:, 0], rpy[:, 1]))
        roads_mod.paint_labels(r, grid)
    gdir = project.root / "grid"
    grid.save(gdir)
    export.save_json(export.road_json(found), project.root / "roads.json")
    return grid, found


def load_roads(project: Project) -> List[roads_mod.Road]:
    f = project.root / "roads.json"
    if not f.is_file():
        return []
    out = []
    for d in json.loads(f.read_text()):
        r = roads_mod.Road(d["id"], d["source"], np.asarray(d["s"], float),
                           np.asarray(d["x"], float), np.asarray(d["y"], float),
                           np.asarray(d["heading"], float))
        skip = {"id", "source", "length", "s", "x", "y", "heading"}
        r.data = {k: np.asarray([np.nan if v is None else v for v in vals], float)
                  for k, vals in d.items() if k not in skip}
        out.append(r)
    return out


# --- step 4: export ---------------------------------------------------------------------------

def step_export(project: Project, cfg: dict, progress: Progress,
                grid: Optional[TerrainGrid] = None, found=None) -> dict:
    grid = grid or TerrainGrid.load(project.root / "grid")
    found = found if found is not None else load_roads(project)
    out = project.out
    out.mkdir(parents=True, exist_ok=True)
    origin = tuple(project.meta.get("origin") or (0.0, 0.0, 0.0))
    progress.set("export", 0, 5, "OpenDRIVE")
    name = _slug(project.name)
    xinfo = xodr.write_xodr(found, origin, cfg, out / ("%s.xodr" % name), name=name)
    progress.set("export", 1, 5, "road profile CSV")
    export.write_profile_csv(found, out / "road_profile.csv")
    rows = []
    for i, rec in enumerate(project.recordings):
        fz = project.cache(i) / "frames.npz"
        if not fz.is_file():
            continue
        d = np.load(fz)
        rpy = np.degrees(geo.mat_to_rpy(d["T"][:, :3, :3]))
        for k in range(len(d["t"])):
            rows.append({"recording": Path(rec).name, "frame": k, "t": "%.6f" % d["t"][k],
                         "x": round(float(d["T"][k, 0, 3]), 4), "y": round(float(d["T"][k, 1, 3]), 4),
                         "z": round(float(d["T"][k, 2, 3]), 4),
                         "roll_deg": round(float(rpy[k, 0]), 4),
                         "pitch_deg": round(float(rpy[k, 1]), 4),
                         "yaw_deg": round(float(rpy[k, 2]), 4),
                         "gnss_x": round(float(d["G"][k, 0, 3]), 4),
                         "gnss_y": round(float(d["G"][k, 1, 3]), 4),
                         "gnss_z": round(float(d["G"][k, 2, 3]), 4)})
    export.write_trajectory_csv(rows, out / "trajectory.csv")
    progress.set("export", 2, 5, "ground point cloud")
    n_pts = export.write_ground_ply(grid, out / "ground.ply")
    progress.set("export", 3, 5, "elevation rasters")
    rdir = out / "rasters"
    if rdir.exists():
        shutil.rmtree(rdir)
    export.write_rasters(grid, rdir, ("z", "slope_deg", "roughness", "label", "intensity"))
    progress.set("export", 4, 5, "previews")
    prev = export.previews(grid, out / "previews")
    export.save_json(prev, out / "previews" / "previews.json")
    stats = grid.stats()
    summary = {
        "name": project.name, "origin_lla": origin, "cell_m": grid.cell,
        "bounds": grid.bounds(), "terrain": stats, "roads": xinfo,
        "road_list": [{"id": r.id, "source": r.source, "length_m": round(r.length, 1),
                       "mean_grade_pct": _nanmean(r.data.get("grade_pct")),
                       "max_abs_grade_pct": _nanmax_abs(r.data.get("grade_pct")),
                       "mean_bank_deg": _nanmean(r.data.get("bank_deg")),
                       "max_abs_bank_deg": _nanmax_abs(r.data.get("bank_deg")),
                       "median_width_m": _nanmedian(r.data.get("road_l"), r.data.get("road_r")),
                       "sidewalk_left_pct": _pct_pos(r.data.get("walk_l")),
                       "sidewalk_right_pct": _pct_pos(r.data.get("walk_r"))}
                      for r in found],
        "scan": project.meta.get("scan"), "ground_points": n_pts,
        "files": sorted(str(p.relative_to(out)) for p in out.iterdir() if p.is_file()),
    }
    export.save_json(summary, out / "summary.json")
    progress.set("export", 5, 5, "done")
    return summary


def _nanmean(v):
    v = np.asarray(v if v is not None else [], float)
    return float(np.nanmean(v)) if np.isfinite(v).any() else None


def _nanmax_abs(v):
    v = np.asarray(v if v is not None else [], float)
    return float(np.nanmax(np.abs(v))) if np.isfinite(v).any() else None


def _nanmedian(a, b):
    if a is None or b is None:
        return None
    v = np.asarray(a, float) + np.asarray(b, float)
    return float(np.nanmedian(v)) if np.isfinite(v).any() else None


def _pct_pos(v):
    v = np.asarray(v if v is not None else [], float)
    return float(100.0 * np.mean(np.nan_to_num(v) > 0)) if len(v) else None


# --- driver ---------------------------------------------------------------------------------------

def build(project: Project, cfg: dict, progress: Optional[Progress] = None,
          start: str = "scan") -> dict:
    """Run the steps from ``start`` to the end."""
    progress = maybe(progress)
    if start not in STEPS:
        raise ValueError("start must be one of %s" % ", ".join(STEPS))
    todo = STEPS[STEPS.index(start):]
    for s in todo:
        project.mark(s, False)
    grid = None
    found = None
    t0 = time.time()
    if "scan" in todo:
        step_scan(project, cfg, progress)
        project.mark("scan")
    if "map" in todo:
        grid = step_map(project, cfg, progress)
        project.mark("map")
    if "roads" in todo:
        grid, found = step_roads(project, cfg, progress, grid)
        project.mark("roads")
    summary = step_export(project, cfg, progress, grid, found)
    project.mark("export")
    project.meta["built"] = time.strftime("%Y-%m-%d %H:%M:%S")
    project.meta["build_seconds"] = round(time.time() - t0, 1)
    project.save()
    return summary


def steps_available(project: Project) -> List[str]:
    """Steps a rebuild may start from (the ones whose inputs exist).

    An imported map without its scan cache can still redo roads and export."""
    ok = ["scan"]
    if all((project.cache(i) / "frames.npz").is_file() for i in range(len(project.recordings))):
        ok.append("map")
    if (project.root / "grid" / "grid.json").is_file():
        ok.append("roads")
        if (project.root / "roads.json").is_file():
            ok.append("export")
    return ok
