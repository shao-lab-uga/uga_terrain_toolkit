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
import math
import re
import shutil
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from uga_terrain import export, geo, junction_shape, junctions as junctions_mod, poses, register, roads as roads_mod, xodr
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
                "ranges": [self.range_of(r) for r in self.recordings],
                "steps": st, "built": self.meta.get("built"),
                "road_names": self.road_names(),
                "summary": json.loads(summary.read_text()) if summary.is_file() else None}

    def road_names(self) -> Dict[int, str]:
        """Names typed in for roads, by road id (roads without one are left out)."""
        return {int(k): v for k, v in (self.meta.get("road_names") or {}).items() if v}

    def set_road_names(self, names: dict) -> None:
        cur = {str(k): v for k, v in (self.meta.get("road_names") or {}).items()}
        for k, v in names.items():
            v = " ".join(str(v or "").split())[:80]
            if v:
                cur[str(int(k))] = v
            else:
                cur.pop(str(int(k)), None)
        self.meta["road_names"] = cur
        self.save()
        apply_road_names(self)

    def edits(self) -> dict:
        """Changes made in the lane editor: per road (type, speed, unit, one_way,
        lanes: {lane id: {type, width}}), removed_roads, removed_turns."""
        e = dict(self.meta.get("edits") or {})
        e.setdefault("roads", {})
        e.setdefault("removed_roads", [])
        e.setdefault("removed_turns", [])
        return e

    def set_edits(self, e: dict) -> dict:
        clean = {"roads": {}, "removed_roads": sorted({int(v) for v in e.get("removed_roads") or []}),
                 "removed_turns": sorted({str(v) for v in e.get("removed_turns") or []})}
        for rid, d in (e.get("roads") or {}).items():
            d = {k: v for k, v in (d or {}).items() if v not in (None, "", {})}
            if d.get("lanes"):
                d["lanes"] = {str(int(k)): {kk: vv for kk, vv in (v or {}).items() if vv not in (None, "")}
                              for k, v in d["lanes"].items()}
                d["lanes"] = {k: v for k, v in d["lanes"].items() if v}
                if not d["lanes"]:
                    del d["lanes"]
            if d:
                clean["roads"][str(int(rid))] = d
        self.meta["edits"] = clean
        self.save()
        return clean

    def range_of(self, recording: str) -> Optional[dict]:
        return (self.meta.get("ranges") or {}).get(str(recording))

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
        if not d.name.startswith(".") and (d / "project.json").is_file():
            try:
                out.append(Project(d).status())
            except (OSError, ValueError):
                continue
    return out


# --- step 1: scan --------------------------------------------------------------------------

_SCAN_KEYS = ("topics", "frames", "pose", "lidar", "odometry", "ground")
SCAN_VERSION = 4   # bump when scan output changes, so old caches are rebuilt


def clean_range(r) -> Optional[dict]:
    """A part of a recording: {"unit": "s" | "frame", "start": a, "end": b}.

    Seconds count from the first lidar scan; frames are lidar scan numbers
    starting at 0. Either end may be None (open). None means the whole recording."""
    if not r:
        return None
    unit = "frame" if str(r.get("unit")) == "frame" else "s"
    a = r.get("start")
    b = r.get("end")
    a = None if a in (None, "") else float(a)
    b = None if b in (None, "") else float(b)
    if a is None and b is None:
        return None
    if a is not None and b is not None and b <= a:
        raise ValueError("the end of the range must come after its start")
    return {"unit": unit, "start": a, "end": b}


def range_text(r: Optional[dict]) -> str:
    if not r:
        return "whole recording"
    u = "s" if r["unit"] == "s" else ""
    a = "start" if r["start"] is None else ("%g%s" % (r["start"], u) if u else "frame %d" % r["start"])
    b = "end" if r["end"] is None else ("%g%s" % (r["end"], u) if u else "frame %d" % r["end"])
    return "%s to %s" % (a, b)


def _scan_key(path: str, cfg: dict, origin, rng: Optional[dict] = None) -> str:
    st = Path(path).stat()
    blob = json.dumps({"v": SCAN_VERSION, "p": str(path), "size": st.st_size, "mtime": int(st.st_mtime),
                       "origin": origin, "range": rng, "cfg": {k: cfg.get(k) for k in _SCAN_KEYS}},
                      sort_keys=True, default=str)
    return hashlib.sha1(blob.encode()).hexdigest()[:16]


def _rot_angle(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def _deskew(xyz: np.ndarray, dt: np.ndarray, t: float, G: np.ndarray, T_bl: np.ndarray,
            track, bin_s: float = 0.004) -> np.ndarray:
    """Lidar-frame points measured at t + dt -> base frame at time t."""
    out = np.empty_like(xyz)
    G_inv = np.linalg.inv(G)
    b = np.round(dt / bin_s).astype(np.int64)
    for k in np.unique(b):
        m = b == k
        p, q = track.interp(t + k * bin_s)   # Ouster stamps a scan at its first column
        M = G_inv @ poses.pose_matrix(p, q)[0] @ T_bl
        out[m] = geo.transform(M, xyz[m])
    return out


def scan_recording(path: str, cfg: dict, origin, out: Path, progress: Progress,
                   label: str = "", rng: Optional[dict] = None) -> dict:
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
        progress.note("%s: GNSS read from %s (%s is missing or stops early in this recording)"
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
    t_first = None
    deskew = bool(lid.get("deskew", True))
    n_deskew = 0
    T_bl_inv = np.linalg.inv(T_bl)
    for t_log, _topic, msg in iter_messages(path, [lidar_topic]):
        n_seen += 1
        if rng:
            # only the chosen part of the recording
            if t_first is None:
                t_first = stamp_of(msg, t_log) if use_header else t_log
            pos_ = (n_seen - 1) if rng["unit"] == "frame" else \
                (stamp_of(msg, t_log) if use_header else t_log) - t_first
            if rng["end"] is not None and pos_ > rng["end"]:
                break
            if rng["start"] is not None and pos_ < rng["start"]:
                continue
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
        pts = range_filter(decode_pointcloud2(msg, with_time=deskew),
                           float(lid.get("min_range") or 2.0), float(lid.get("max_range") or 80.0))
        if len(pts) < 100:
            continue
        base = pts[:, :4].copy()
        if deskew and np.isfinite(pts[:, 4]).all():
            # the lidar spins for ~0.1 s while the car moves; put every point
            # where it was at the scan's time stamp, using the GNSS/INS motion
            base[:, :3] = _deskew(pts[:, :3].astype(np.float64), pts[:, 4].astype(np.float64),
                                  t, G, T_bl, track)
            n_deskew += 1
        else:
            base[:, :3] = geo.transform(T_bl, pts[:, :3].astype(np.float64))
        pts = pts[:, :4].copy()
        pts[:, :3] = geo.transform(T_bl_inv, base[:, :3])
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
        if rng:
            raise RuntimeError("%s: no usable lidar scans in %s (the recording has %d scans). "
                               "Check the range, or the car may not move in that part."
                               % (Path(path).name, range_text(rng), n_seen))
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
        "deskewed_frames": n_deskew,
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
        rng = project.range_of(path)
        key = _scan_key(path, cfg, origin, rng)
        meta_f = cdir / "meta.json"
        if meta_f.is_file() and (cdir / "frames.npz").is_file():
            meta = json.loads(meta_f.read_text())
            if meta.get("key") == key:
                progress.note("%s: using cached scan" % Path(path).name)
                out.append(meta["stats"])
                continue
        cdir.mkdir(parents=True, exist_ok=True)
        label = "[%d/%d] %s" % (i + 1, len(recs), Path(path).name)
        if rng:
            progress.note("%s: mapping %s" % (Path(path).name, range_text(rng)))
        stats = scan_recording(path, cfg, tuple(origin), cdir, progress, label, rng)
        stats["range"] = rng
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
    align = {}
    if len(recs) > 1:
        align = register.align_project(project, cfg, progress)
        for i, rec in enumerate(recs):
            if str(i) in align and i > 0:
                progress.note("Lining up drives: " + register.summary_text(Path(rec).name, align[str(i)]))
    for i in range(len(recs)):
        T = register.poses_of(project, i, align)
        if T is None:
            continue
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
    align = register.load(project)
    for i, rec in enumerate(project.recordings):
        T = register.poses_of(project, i, align)
        if T is None:
            continue
        name = "%d:%s" % (i, Path(rec).name)
        paths.append((name, T[:, :2, 3]))
        frames[name] = T
    progress.set("find roads", 0, 1, "%d drives" % len(paths))
    juncs = junctions_mod.find(paths, cfg)

    def analyse(found):
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
    for attempt in range(2):
        pieces, passes = junctions_mod.cut(paths, juncs)
        found = roads_mod.split_roads(pieces, cfg)
        analyse(found)
        # a side street the drives went straight past shows as the road edge
        # opening up on one or both sides
        new = junctions_mod.openings(found, juncs, cfg) if attempt == 0 else []
        if not new:
            break
        juncs = juncs + new
    if juncs and (cfg.get("junctions") or {}).get("fit_to_curbs", True):
        # the junction's edge follows the curbs, and streets nobody drove into get a short road
        progress.set("find roads", 0, 1, "fitting junctions to the curbs")
        lines = [np.stack([r.x, r.y], 1) for r in found]
        for jn in juncs:
            res = junction_shape.fit(jn, junction_shape.leg_ends(jn, found), grid, cfg, lines)
            if res:
                jn.outline, jn.legs = res["outline"], res["legs"]
                ol = np.asarray(jn.outline)
                jn.r = float(np.max(np.hypot(ol[:, 0] - jn.x, ol[:, 1] - jn.y)))
        pieces, passes = junctions_mod.cut(paths, juncs)
        found = roads_mod.split_roads(pieces, cfg)
        analyse(found)
        for jn in juncs:
            # leg rows of driven roads point at the new road ids
            remap = {}
            for lg in jn.legs:
                if not lg.get("stub"):
                    new = next((e["road"] for e in junction_shape.leg_ends(jn, found)
                                if abs(math.remainder(e["h_out"] - math.radians(lg["theta_deg"]), 2 * math.pi))
                                < math.radians(30)), lg.get("road"))
                    remap[lg.get("road")] = new
                    lg["road"] = new
            for lg in jn.legs:
                if lg.get("stub") and lg.get("opposite") is not None:
                    lg["opposite"] = remap.get(lg["opposite"], lg["opposite"])
        stubs = junction_shape.stub_roads(juncs, grid, cfg, max([r.id for r in found] + [0]) + 1, found)
        found += stubs
        for st in stubs:
            progress.note("Junction %d: a street nobody drove into, %.0f m wide; added as road %d (%.0f m)." % (
                st.meta["junction_start"], st.data["road_l"][0] * 2, st.id, st.length))
    _driven_movements(juncs, found, passes)
    for jn in juncs:
        legs = [r.id for r in found if jn.id in (r.meta.get("junction_start"), r.meta.get("junction_end"))]
        progress.note("Junction %d (%s): %d roads meet there%s." % (
            jn.id, ", ".join(jn.why), len(legs),
            "; driven: " + ", ".join("road %s to road %s" % tuple(k.split(">")) for k in jn.movements)
            if jn.movements else ""))
    export.save_json([jn.to_json() for jn in juncs], project.root / "junctions.json")
    for j, r in enumerate(found):
        roads_mod.paint_labels(r, grid)
        if (cfg.get("lanes") or {}).get("detect_paint", True) and not r.meta.get("stub"):
            progress.set("find lanes", j, len(found), "road %d: looking for lane paint" % r.id)
            _detect_lanes(project, r, frames, cfg, progress)
    if (cfg.get("lanes") or {}).get("detect_paint", True):
        _street_lanes(project, found, juncs, frames, cfg, progress)
    if juncs and (cfg.get("lanes") or {}).get("detect_markings", True):
        from uga_terrain import markings
        jids = {jn.id for jn in juncs}
        for j, r in enumerate(found):
            if not jids & {r.meta.get("junction_start"), r.meta.get("junction_end")}:
                continue
            progress.set("find lanes", j, len(found), "road %d: stop lines and crosswalks" % r.id)
            try:
                ms = markings.detect(r, jids, lambda P: _scans_near(project, frames, P, cfg), cfg)
            except Exception as e:
                progress.note("Road %d: looking for stop lines and crosswalks failed (%s)." % (r.id, e))
                continue
            r.meta["markings"] = ms
            if ms:
                progress.note("Road %d: %s at the junction." % (r.id, ", ".join(
                    "%s %.1f m deep" % (m["kind"].replace("_", " "), m["depth"]) for m in ms)))
    gdir = project.root / "grid"
    grid.save(gdir)
    export.save_json(export.road_json(found), project.root / "roads.json")
    return grid, found


def _driven_movements(juncs, found, passes) -> None:
    """Which leg a drive came in on and which it left by, per junction."""
    for jn in juncs:
        legs = []
        for r in found:
            if r.meta.get("junction_end") == jn.id:
                legs.append((r.id, float(r.heading[-1])))            # heading into the junction
            if r.meta.get("junction_start") == jn.id:
                legs.append((r.id, float(r.heading[0]) + math.pi))
        if len(legs) < 2:
            continue
        for jid, h_in, h_out, _name, pxy in passes:
            if jid != jn.id:
                continue
            a = min(legs, key=lambda l: abs(math.remainder(l[1] - h_in, 2 * math.pi)))
            b = min(legs, key=lambda l: abs(math.remainder(l[1] + math.pi - h_out, 2 * math.pi)))
            if a[0] != b[0] and abs(math.remainder(a[1] - h_in, 2 * math.pi)) < math.radians(45) \
                    and abs(math.remainder(b[1] + math.pi - h_out, 2 * math.pi)) < math.radians(45):
                k = "%d>%d" % (a[0], b[0])
                jn.movements[k] = jn.movements.get(k, 0) + 1
                jn.paths.setdefault(k, np.round(np.asarray(pxy, float), 3).tolist())


def _street_lanes(project: Project, found, juncs, frames: dict, cfg: dict, progress: Progress) -> None:
    """One lane model per street through a junction: two legs that continue
    each other (straight on) get their lanes found together, so the centre
    line and lane widths match on both sides of the junction."""
    from uga_terrain import lanes
    step = float((cfg.get("roads") or {}).get("station_step") or 1.0)
    done = set()
    for jn in juncs:
        legs = []
        for r in found:
            for contact in ("start", "end"):
                if r.meta.get("junction_" + contact) == jn.id:
                    h_in = float(r.heading[-1]) if contact == "end" else float(r.heading[0]) + math.pi
                    legs.append((r, contact, h_in))
        pairs = []
        for A, ca, ha in legs:
            for B, cb, hb in legs:
                if A is B:
                    continue
                dev = abs(math.remainder((hb + math.pi) - ha, 2 * math.pi))
                if dev < math.radians(25):
                    pairs.append((dev, A, ca, B, cb))
        for dev, A, ca, B, cb in sorted(pairs, key=lambda p: p[0]):
            if A.id in done or B.id in done:
                continue
            # A in travel order towards the junction, B away from it
            fa, fb = ca == "start", cb == "end"
            ax, ay, ah = (A.x[::-1], A.y[::-1], A.heading[::-1] + math.pi) if fa else (A.x, A.y, A.heading)
            bx, by, bh = (B.x[::-1], B.y[::-1], B.heading[::-1] + math.pi) if fb else (B.x, B.y, B.heading)

            def side(r, flip):
                rl = r.data.get("road_l", np.full(len(r.s), np.nan))
                rr = r.data.get("road_r", np.full(len(r.s), np.nan))
                return (rr[::-1], rl[::-1]) if flip else (rl, rr)
            al, ar = side(A, fa)
            bl, br = side(B, fb)
            gap = math.hypot(bx[0] - ax[-1], by[0] - ay[-1])
            nb = max(1, int(round(gap / step)))
            f = np.arange(1, nb) / nb
            gx = ax[-1] + f * (bx[0] - ax[-1])
            gy = ay[-1] + f * (by[0] - ay[-1])
            gh = np.full(len(f), math.atan2(by[0] - ay[-1], bx[0] - ax[-1]))
            x = np.concatenate([ax, gx, bx])
            y = np.concatenate([ay, gy, by])
            h = np.unwrap(np.concatenate([ah, gh, bh]))
            s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
            nan = np.full(len(f), np.nan)
            street = roads_mod.Road(-1, A.source, s, x, y, h,
                                    {"road_l": np.concatenate([al, nan, bl]),
                                     "road_r": np.concatenate([ar, nan, br])})
            na = len(ax)
            n_gap = len(f)
            P = np.stack([x, y], 1)
            lo, hi = P.min(0) - 45.0, P.max(0) + 45.0
            rng = float((cfg.get("lanes") or {}).get("paint_range", 20.0)) + 5.0

            def scans():
                for name, T in frames.items():
                    fdir = project.cache(int(name.split(":", 1)[0])) / "frames"
                    for k in range(len(T)):
                        c = T[k, :2, 3]
                        if not ((c >= lo).all() and (c <= hi).all()):
                            continue
                        if np.min(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1])) > rng:
                            continue
                        fp = fdir / ("%06d.npy" % k)
                        if fp.is_file():
                            g = np.load(fp)
                            if g.shape[1] >= 4:
                                yield T[k], g
            progress.set("find lanes", 0, 1, "street through junction %d: roads %d and %d" % (jn.id, A.id, B.id))
            try:
                lay = lanes.detect(street, scans, cfg, blank=[(s[na - 1] + 0.5, s[na + n_gap] - 0.5)])
            except Exception as e:
                progress.note("Roads %d and %d: joint lane detection failed (%s)." % (A.id, B.id, e))
                continue
            bounds = lay.pop("bounds")
            for r, sl, flip in ((A, slice(0, na), fa), (B, slice(na + n_gap, len(s)), fb)):
                bs = [b[sl] for b in bounds]
                types, marks, centre = list(lay["types"]), list(lay["marks"]), int(lay["centre"])
                if flip:
                    bs = [-b[::-1] for b in reversed(bs)]
                    types, marks = types[::-1], marks[::-1]
                    centre = len(bs) - 1 - centre
                for k in [k for k in r.data if k.startswith("lb")]:
                    del r.data[k]
                for k, b in enumerate(bs):
                    r.data["lb%d" % k] = b
                r.meta["lanes"] = dict(lay, types=types, marks=marks, centre=centre,
                                       street_with=(B.id if r is A else A.id))
            done.update((A.id, B.id))
            n_drive = sum(t == "driving" for t in lay["types"])
            progress.note("Roads %d and %d are one street through junction %d: %d driving lane(s), %s." % (
                A.id, B.id, jn.id, n_drive, "from %d painted line(s)" % len(lay["lines"])
                if lay["source"] == "paint" else "no lane paint found, so the width is split evenly"))


def _scans_near(project: Project, frames: dict, P: np.ndarray, cfg: dict):
    """Lidar frames (pose, points with intensity) taken near the points P."""
    rng = float((cfg.get("lanes") or {}).get("paint_range", 20.0)) + 5.0
    lo, hi = P.min(0) - rng, P.max(0) + rng
    for name, T in frames.items():
        fdir = project.cache(int(name.split(":", 1)[0])) / "frames"
        for k in range(len(T)):
            c = T[k, :2, 3]
            if not ((c >= lo).all() and (c <= hi).all()):
                continue
            if np.min(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1])) > rng:
                continue
            fp = fdir / ("%06d.npy" % k)
            if fp.is_file():
                g = np.load(fp)
                if g.shape[1] >= 4:
                    yield T[k], g


def load_junctions(project: Project):
    f = project.root / "junctions.json"
    if not f.is_file():
        return []
    return [junctions_mod.Junction.from_json(d) for d in json.loads(f.read_text())]


def _detect_lanes(project: Project, road, frames: dict, cfg: dict, progress: Progress) -> None:
    """Lane lines from lidar intensity, using every drive that saw the road;
    the lane model goes into road.data (boundaries lb0..lbN) and road.meta["lanes"]."""
    from uga_terrain import lanes
    lo = np.array([road.x.min(), road.y.min()]) - 45.0
    hi = np.array([road.x.max(), road.y.max()]) + 45.0
    rng = float((cfg.get("lanes") or {}).get("paint_range", 20.0)) + 5.0
    P = np.stack([road.x, road.y], 1)

    def scans():
        for name, T in frames.items():
            fdir = project.cache(int(name.split(":", 1)[0])) / "frames"
            for k in range(len(T)):
                c = T[k, :2, 3]
                if not ((c >= lo).all() and (c <= hi).all()):
                    continue
                if np.min(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1])) > rng:
                    continue
                f = fdir / ("%06d.npy" % k)
                if f.is_file():
                    g = np.load(f)
                    if g.shape[1] >= 4:
                        yield T[k], g
    try:
        lay = lanes.detect(road, scans, cfg)
    except Exception as e:  # keep the build going with evenly split lanes
        progress.note("Road %d: lane detection failed (%s); lanes are split evenly." % (road.id, e))
        return
    for k in [k for k in road.data if k.startswith("lb")]:
        del road.data[k]
    for k, b in enumerate(lay.pop("bounds")):
        road.data["lb%d" % k] = b
    road.meta["lanes"] = lay
    n_drive = sum(t == "driving" for t in lay["types"])
    progress.note("Road %d: %d driving lane(s), %s." % (
        road.id, n_drive, "from %d painted line(s)" % len(lay["lines"]) if lay["source"] == "paint"
        else "no lane paint found, so the width is split evenly"))


def load_roads(project: Project) -> List[roads_mod.Road]:
    f = project.root / "roads.json"
    if not f.is_file():
        return []
    out = []
    for d in json.loads(f.read_text()):
        r = roads_mod.Road(d["id"], d["source"], np.asarray(d["s"], float),
                           np.asarray(d["x"], float), np.asarray(d["y"], float),
                           np.asarray(d["heading"], float))
        skip = {"id", "source", "length", "s", "x", "y", "heading", "meta"}
        r.meta = d.get("meta") or {}
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
    names = project.road_names()
    xinfo = write_xodr(project, cfg, grid, found)
    progress.set("export", 1, 5, "road profile CSV")
    export.write_profile_csv(found, out / "road_profile.csv")
    rows = []
    for i, rec in enumerate(project.recordings):
        fz = project.cache(i) / "frames.npz"
        if not fz.is_file():
            continue
        d = dict(np.load(fz))
        d["T"] = register.poses_of(project, i)
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
        "road_list": [{"id": r.id, "name": names.get(r.id), "source": r.source, "length_m": round(r.length, 1),
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
    # check the .xodr as a simulator would read it, replaying the car's own path
    chk = check_xodr(project)
    summary["xodr_check"] = chk["counts"]
    export.save_json(summary, out / "summary.json")
    for f in chk["findings"]:
        if f["level"] != "info":
            progress.note("OpenDRIVE check %s%s: %s" % (f["level"], " (road %s)" % f["road"] if f["road"] else "", f["msg"]))
    progress.set("export", 5, 5, "done")
    return summary


def write_xodr(project: Project, cfg: dict, grid: Optional[TerrainGrid] = None, found=None) -> dict:
    """The .xodr from the roads, junctions and the lane editor's changes."""
    grid = grid or TerrainGrid.load(project.root / "grid")
    found = found if found is not None else load_roads(project)
    origin = tuple(project.meta.get("origin") or (0.0, 0.0, 0.0))
    name = _slug(project.name)
    project.out.mkdir(parents=True, exist_ok=True)
    return xodr.write_xodr(found, origin, cfg, project.out / ("%s.xodr" % name), name=name,
                           road_names=project.road_names(), junctions=load_junctions(project),
                           ground=lambda x, y: grid.sample(x, y, "z"), edits=project.edits())


def check_xodr(project: Project) -> dict:
    from uga_terrain import xodr_check
    out = project.out
    chk = xodr_check.check(out / ("%s.xodr" % _slug(project.name)), [out / "trajectory.csv"])
    export.save_json(chk, out / "xodr_check.json")
    return chk


def apply_edits(project: Project, cfg: dict) -> dict:
    """Rewrite and re-check the .xodr after the lane editor changed something
    (no rebuild needed)."""
    xinfo = write_xodr(project, cfg)
    chk = check_xodr(project)
    sj = project.out / "summary.json"
    if sj.is_file():
        summary = json.loads(sj.read_text())
        summary["roads"] = xinfo
        summary["xodr_check"] = chk["counts"]
        export.save_json(summary, sj)
    return {"xodr": xinfo, "check": chk["counts"]}


def apply_road_names(project: Project) -> None:
    """Put the current road names into an already exported .xodr and summary.json,
    so renaming a road does not need a rebuild."""
    names = project.road_names()
    out = project.out
    sj = out / "summary.json"
    if sj.is_file():
        summary = json.loads(sj.read_text())
        for r in summary.get("road_list") or []:
            r["name"] = names.get(r["id"])
        export.save_json(summary, sj)
    xf = out / ("%s.xodr" % _slug(project.name))
    if xf.is_file():
        import xml.etree.ElementTree as ET
        tree = ET.parse(xf)
        for r in tree.getroot().findall("road"):
            rid = int(r.get("id"))
            r.set("name", names.get(rid) or "road_%d" % rid)
        tree.write(xf, encoding="utf-8", xml_declaration=True)


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
