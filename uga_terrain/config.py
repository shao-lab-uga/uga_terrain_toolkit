"""Configuration: configs/default.yaml overlaid with configs/local.yaml."""
from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def default_path() -> Path:
    return repo_root() / "configs" / "default.yaml"


def local_path() -> Path:
    return repo_root() / "configs" / "local.yaml"


def deep_merge(base: dict, over: Optional[dict]) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _read(path: Path) -> dict:
    if not path.is_file():
        return {}
    with open(path) as f:
        rec = yaml.safe_load(f) or {}
    return rec if isinstance(rec, dict) else {}


def load_config(path: Optional[str] = None) -> Dict[str, Any]:
    cfg = deep_merge(_read(default_path()), _read(local_path()))
    if path:
        cfg = deep_merge(cfg, _read(Path(path)))
    if os.environ.get("UGA_TERRAIN_WORK_DIR"):
        cfg["work_dir"] = os.environ["UGA_TERRAIN_WORK_DIR"]
    return cfg


def _diff(cur: dict, base: dict) -> dict:
    """The parts of ``cur`` that differ from ``base``."""
    out = {}
    for k, v in cur.items():
        b = base.get(k) if isinstance(base, dict) else None
        if isinstance(v, dict) and isinstance(b, dict):
            d = _diff(v, b)
            if d:
                out[k] = d
        elif v != b:
            out[k] = copy.deepcopy(v)
    return out


def save_local(changes: dict, replace: bool = False) -> Dict[str, Any]:
    """Merge ``changes`` into configs/local.yaml (or replace it) and return the new config.

    Only values that differ from configs/default.yaml are written, so later
    updates to the defaults still reach settings the user never changed."""
    cur = deep_merge({} if replace else _read(local_path()), changes)
    if os.environ.get("UGA_TERRAIN_WORK_DIR") and cur.get("work_dir") == os.environ["UGA_TERRAIN_WORK_DIR"]:
        cur.pop("work_dir")
    cur = _diff(cur, _read(default_path()))
    if cur:
        with open(local_path(), "w") as f:
            yaml.safe_dump(cur, f, sort_keys=False)
    elif local_path().is_file():
        local_path().unlink()
    return load_config()


def work_dir(cfg: dict) -> Path:
    p = Path(os.path.expanduser(str(cfg.get("work_dir") or "~/uga_terrain_work")))
    p.mkdir(parents=True, exist_ok=True)
    return p


def data_folders(cfg: dict):
    return [Path(os.path.expanduser(str(p))) for p in (cfg.get("data_folders") or [])]
