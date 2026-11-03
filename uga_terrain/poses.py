"""Vehicle poses from GNSS (NavSatFix) + INS orientation (Imu), and /tf_static."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from uga_terrain import geo
from uga_terrain.mcap_io import iter_messages, stamp_of


@dataclass
class PoseTrack:
    """World (local ENU) <- base poses, sorted by time."""
    t: np.ndarray        # (N,) seconds
    pos: np.ndarray      # (N, 3) east, north, up of the base frame
    quat: np.ndarray     # (N, 4) x y z w, base -> world
    gnss_ok: np.ndarray  # (N,) bool, a real fix backs this sample
    source: str = "navsat_imu"

    def __len__(self) -> int:
        return len(self.t)

    def interp(self, ts) -> Tuple[np.ndarray, np.ndarray]:
        ts = np.atleast_1d(np.asarray(ts, dtype=np.float64))
        if len(self.t) == 1:
            return (np.repeat(self.pos, len(ts), 0), np.repeat(self.quat, len(ts), 0))
        j = np.clip(np.searchsorted(self.t, ts) - 1, 0, len(self.t) - 2)
        t0, t1 = self.t[j], self.t[j + 1]
        f = np.clip((ts - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)
        pos = self.pos[j] * (1 - f)[:, None] + self.pos[j + 1] * f[:, None]
        quat = geo.slerp(self.quat[j], self.quat[j + 1], f)
        return pos, quat

    def covers(self, t: float, slack: float = 0.5) -> bool:
        return len(self.t) > 0 and self.t[0] - slack <= t <= self.t[-1] + slack


# --- /tf_static ---------------------------------------------------------------

def read_static_tf(path: str, topic: str = "/tf_static") -> Dict[str, Tuple[str, np.ndarray]]:
    """child frame -> (parent frame, 4x4 parent <- child)."""
    edges: Dict[str, Tuple[str, np.ndarray]] = {}
    for _t, _topic, msg in iter_messages(path, [topic]):
        for tr in msg.transforms:
            r, p = tr.transform.rotation, tr.transform.translation
            T = geo.make_T(geo.quat_to_mat([r.x, r.y, r.z, r.w]), [p.x, p.y, p.z])
            edges[tr.child_frame_id.lstrip("/")] = (tr.header.frame_id.lstrip("/"), T)
        if edges:
            break  # latched topic: the first message has the whole tree
    return edges


def lookup(edges: Dict[str, Tuple[str, np.ndarray]], parent: str, child: str
           ) -> Optional[np.ndarray]:
    """4x4 parent <- child through the static tree, or None."""
    def chain(frame):
        out = [(frame, np.eye(4))]
        T = np.eye(4)
        seen = {frame}
        while frame in edges:
            p, Tpc = edges[frame]
            T = Tpc @ T
            frame = p
            if frame in seen:
                break
            seen.add(frame)
            out.append((frame, T.copy()))
        return dict(out)
    a, b = chain(parent), chain(child)
    for frame, T_root_child in b.items():
        if frame in a:
            return np.linalg.inv(a[frame]) @ T_root_child
    return None


# --- GNSS / INS ------------------------------------------------------------------

def _frd_to_flu(q):
    """OXTS driver: orientation of a forward-right-down body in ENU -> forward-left-up."""
    x, y, z, w = q
    return (w, z, -y, -x)


def read_gnss_imu(path: str, cfg: dict):
    topics = cfg.get("topics") or {}
    pcfg = cfg.get("pose") or {}
    nav_t, imu_t = topics.get("navsat"), topics.get("imu")
    use_header = str(pcfg.get("time_source") or "header") == "header"
    frd = str(pcfg.get("imu_convention") or "enu_frd").lower() == "enu_frd"
    fixes, imus = [], []
    for t_log, topic, msg in iter_messages(path, [x for x in (nav_t, imu_t) if x]):
        t = stamp_of(msg, t_log) if use_header else t_log
        if topic == nav_t:
            st = getattr(getattr(msg, "status", None), "status", 0)
            vals = (msg.latitude, msg.longitude, msg.altitude)
            if st >= 0 and np.isfinite(vals).all() and abs(msg.latitude) > 1e-9:
                fixes.append((t,) + tuple(vals))
        else:
            o = msg.orientation
            q = (o.x, o.y, o.z, o.w)
            if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.0) > 0.1:
                continue
            imus.append((t,) + (_frd_to_flu(q) if frd else q))
    return np.asarray(sorted(fixes), float).reshape(-1, 4), \
        np.asarray(sorted(imus), float).reshape(-1, 5)


def build_track(fixes: np.ndarray, imus: np.ndarray, origin, cfg: dict) -> PoseTrack:
    """Positions from the fixes, orientation from the INS (or the GNSS track)."""
    if len(fixes) == 0:
        raise ValueError("no GNSS fixes in this recording")
    enu = geo.lla_to_enu(fixes[:, 1], fixes[:, 2], fixes[:, 3], origin)
    tf = fixes[:, 0]
    if len(imus) >= 2:
        t = imus[:, 0]
        keep = (t >= tf[0] - 0.05) & (t <= tf[-1] + 0.05)
        t, quat = t[keep], imus[keep, 1:5]
        if len(t) < 2:
            t, quat = tf, np.repeat(imus[:1, 1:5], len(tf), 0)
        pos = np.stack([np.interp(t, tf, enu[:, k]) for k in range(3)], 1)
        dt = np.abs(t[:, None] - tf[None, :]).min(1) if len(tf) < 20000 else np.zeros(len(t))
        ok = dt < 0.5
        source = "navsat_imu"
    else:
        t, pos = tf, enu
        yaw = np.zeros(len(t))
        d = np.diff(enu[:, :2], axis=0)
        moving = np.hypot(d[:, 0], d[:, 1]) > 0.05
        last = 0.0
        for i in range(1, len(t)):
            if moving[i - 1]:
                last = float(np.arctan2(d[i - 1, 1], d[i - 1, 0]))
            yaw[i] = last
        yaw[0] = yaw[1] if len(yaw) > 1 else 0.0
        quat = geo.mat_to_quat(geo.rpy_to_mat(np.zeros_like(yaw), np.zeros_like(yaw), yaw))
        ok = np.ones(len(t), bool)
        source = "navsat_track"
    off = np.asarray((cfg.get("pose") or {}).get("gnss_offset_base") or [0, 0, 0], float)
    if np.any(off):
        pos = pos - np.einsum("nij,j->ni", geo.quat_to_mat(quat), off)
    order = np.argsort(t, kind="stable")
    t, pos, quat, ok = t[order], pos[order], quat[order], ok[order]
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    return PoseTrack(t[keep], pos[keep], quat[keep], ok[keep], source)


def first_fix(fixes: np.ndarray) -> Tuple[float, float, float]:
    return float(fixes[0, 1]), float(fixes[0, 2]), float(fixes[0, 3])


def pose_matrix(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """(N, 4, 4) world <- base."""
    T = np.repeat(np.eye(4)[None], len(pos), 0)
    T[:, :3, :3] = geo.quat_to_mat(quat)
    T[:, :3, 3] = pos
    return T


def track_rows(T: np.ndarray) -> List[dict]:
    rpy = geo.mat_to_rpy(T[:, :3, :3])
    return [{"x": float(T[i, 0, 3]), "y": float(T[i, 1, 3]), "z": float(T[i, 2, 3]),
             "roll": float(rpy[i, 0]), "pitch": float(rpy[i, 1]), "yaw": float(rpy[i, 2])}
            for i in range(len(T))]
