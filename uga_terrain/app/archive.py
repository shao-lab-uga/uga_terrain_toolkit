"""Map packages (export / import) and snapshots (manual saves).

A map folder holds project.json, the .done_* step marks, grid/, roads.json,
outputs/ and cache/ (per-frame scan results, only needed to rebuild the
elevation map without the recordings). Snapshots live in snapshots/<id>/ and
hold everything except cache/ and other snapshots, plus the settings in use.
"""
from __future__ import annotations

import json
import re
import shutil
import time
import zipfile
from pathlib import Path
from typing import List, Optional

from uga_terrain.pipeline import _slug

SKIP_ALWAYS = {"snapshots"}
MANIFEST = "terrainmap.json"


def _entries(root: Path, skip: set):
    for f in sorted(root.rglob("*")):
        rel = f.relative_to(root)
        if rel.parts and rel.parts[0] in skip:
            continue
        if f.is_file():
            yield f, rel


def export_project(root: Path, dest: str, include_cache: bool = False) -> None:
    root = Path(root)
    skip = set(SKIP_ALWAYS) | (set() if include_cache else {"cache"})
    meta = json.loads((root / "project.json").read_text())
    with zipfile.ZipFile(dest, "w", zipfile.ZIP_DEFLATED, allowZip64=True) as z:
        z.writestr(MANIFEST, json.dumps({"format": 1, "slug": root.name, "name": meta.get("name"),
                                         "exported": time.strftime("%Y-%m-%d %H:%M:%S"),
                                         "cache": include_cache}, indent=1))
        for f, rel in _entries(root, skip):
            z.write(f, rel.as_posix())


def import_project(zip_path: str, projects: Path, fallback_name: str = "map") -> str:
    """Unpack an exported map into ``projects``; returns the new map's folder name."""
    projects = Path(projects)
    projects.mkdir(parents=True, exist_ok=True)
    try:
        z = zipfile.ZipFile(zip_path)
    except zipfile.BadZipFile:
        raise ValueError("that file is not a .zip exported from this app")
    with z:
        names = z.namelist()
        if "project.json" not in names:
            raise ValueError("that zip has no project.json; export the map from this app first")
        info = {}
        if MANIFEST in names:
            info = json.loads(z.read(MANIFEST))
        meta = json.loads(z.read("project.json"))
        name = meta.get("name") or info.get("name") or fallback_name
        base, k = name, 2
        while (projects / _slug(name)).exists():
            name, k = "%s %d" % (base, k), k + 1
        slug = _slug(name)
        dest = projects / slug
        tmp = projects / (".import_" + slug)
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir()
        try:
            for n in names:
                if n == MANIFEST or n.endswith("/"):
                    continue
                parts = Path(n).parts
                if Path(n).is_absolute() or ".." in parts:
                    raise ValueError("unsafe path in zip: %s" % n)
                out = tmp.joinpath(*parts)
                out.parent.mkdir(parents=True, exist_ok=True)
                with z.open(n) as src, open(out, "wb") as dst:
                    shutil.copyfileobj(src, dst)
            meta["name"] = name
            meta["imported"] = time.strftime("%Y-%m-%d %H:%M:%S")
            (tmp / "project.json").write_text(json.dumps(meta, indent=1))
            tmp.rename(dest)
        except Exception:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
    return slug


# --- snapshots -----------------------------------------------------------------------

def _snap_dir(root: Path) -> Path:
    return Path(root) / "snapshots"


def list_snapshots(root: Path) -> List[dict]:
    d = _snap_dir(root)
    out = []
    if d.is_dir():
        for s in sorted(d.iterdir(), reverse=True):
            f = s / "snapshot.json"
            if f.is_file():
                try:
                    out.append(json.loads(f.read_text()))
                except ValueError:
                    continue
    return out


def _size(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def save_snapshot(root: Path, label: str, settings_yaml: str) -> dict:
    root = Path(root)
    sid = time.strftime("%Y%m%d-%H%M%S")
    dest = _snap_dir(root) / sid
    k = 2
    while dest.exists():
        dest = _snap_dir(root) / ("%s_%d" % (sid, k))
        k += 1
    dest.mkdir(parents=True)
    for f, rel in _entries(root, {"snapshots", "cache"}):
        out = dest / "files" / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, out)
    (dest / "settings.yaml").write_text(settings_yaml)
    meta = json.loads((root / "project.json").read_text())
    info = {"id": dest.name, "label": label or "Saved", "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
            "built": meta.get("built"), "recordings": len(meta.get("recordings") or []),
            "size": _size(dest)}
    (dest / "snapshot.json").write_text(json.dumps(info, indent=1))
    return info


def _snap(root: Path, sid: str) -> Path:
    if not re.fullmatch(r"[0-9_-]+", sid or ""):
        raise FileNotFoundError("no snapshot %s" % sid)
    d = _snap_dir(root) / sid
    if not (d / "snapshot.json").is_file():
        raise FileNotFoundError("no snapshot %s" % sid)
    return d


def restore_snapshot(root: Path, sid: str) -> Optional[str]:
    """Put the map back the way it was; returns the snapshot's settings YAML.

    The scan cache is kept (it depends only on the recordings and settings)."""
    root = Path(root)
    snap = _snap(root, sid)
    for f in list(root.iterdir()):
        if f.name in ("snapshots", "cache"):
            continue
        shutil.rmtree(f) if f.is_dir() else f.unlink()
    src = snap / "files"
    for f in sorted(src.rglob("*")):
        if f.is_file():
            out = root / f.relative_to(src)
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, out)
    s = snap / "settings.yaml"
    return s.read_text() if s.is_file() else None


def delete_snapshot(root: Path, sid: str) -> None:
    shutil.rmtree(_snap(Path(root), sid))
