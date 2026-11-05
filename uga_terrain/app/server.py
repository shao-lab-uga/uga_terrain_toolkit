"""The terrain toolkit app: a local web server and its browser UI."""
from __future__ import annotations

import io
import json
import os
import re
import shutil
import struct
import tempfile
import time
import zipfile
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml
from fastapi import Body, FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from uga_terrain import __version__, config as config_mod, mcap_io, pipeline, poses
from uga_terrain.app import archive
from uga_terrain.app.jobs import JobManager
from uga_terrain.grid import LABELS, TerrainGrid
from uga_terrain.ground import patchwork_available
from uga_terrain.odometry import kiss_available

STATIC = Path(__file__).parent / "static"


def _scan_folders(cfg) -> List[dict]:
    out = []
    seen = set()
    for root in config_mod.data_folders(cfg):
        if not root.is_dir():
            continue
        for p in sorted(root.rglob("*.mcap")):
            if p in seen or len(p.relative_to(root).parts) > 4:
                continue
            seen.add(p)
            st = p.stat()
            out.append({"name": p.name, "path": str(p), "folder": str(root),
                        "rel": str(p.relative_to(root)), "size": st.st_size,
                        "mtime": st.st_mtime})
    return out


def _cleanup(path: str):
    from starlette.background import BackgroundTask
    return BackgroundTask(lambda: os.path.exists(path) and os.unlink(path))


def create_app(crash_log: Optional[str] = None) -> FastAPI:
    app = FastAPI(title="UGA Terrain Toolkit")
    jobs = JobManager(crash_log)
    grids: dict = {}
    scans: dict = {}     # accuracy cross-sections: ground points of the open map

    def cfg():
        return config_mod.load_config()

    def project(slug: str) -> pipeline.Project:
        root = config_mod.work_dir(cfg()) / "projects" / slug
        if not (root / "project.json").is_file():
            raise HTTPException(404, "no map %s" % slug)
        return pipeline.Project(root)

    def grid_of(p: pipeline.Project) -> TerrainGrid:
        gj = p.root / "grid" / "grid.json"
        if not gj.is_file():
            raise HTTPException(404, "this map has not been built yet")
        key = (str(p.root), gj.stat().st_mtime)
        g = grids.get(key)
        if g is None:
            grids.clear()
            g = grids[key] = TerrainGrid.load(p.root / "grid")
        return g

    @app.get("/")
    def index():
        # stamp the script and style URLs with their file times, so a browser
        # never keeps running an old app.js after an update
        html = (STATIC / "index.html").read_text()
        for name in ("app.js", "style.css"):
            v = int((STATIC / name).stat().st_mtime)
            html = html.replace('/static/%s"' % name, '/static/%s?v=%d"' % (name, v))
        html = html.replace("{{version}}", __version__)
        return Response(html, media_type="text/html", headers={"Cache-Control": "no-store"})

    @app.get("/api/state")
    def state():
        c = cfg()
        return {"version": __version__, "work_dir": str(config_mod.work_dir(c)),
                "data_folders": [str(p) for p in config_mod.data_folders(c)],
                "kiss_icp": kiss_available(), "patchworkpp": patchwork_available(),
                "crash_log": crash_log, "job": jobs.snapshot()}

    # --- settings ---------------------------------------------------------------------
    @app.post("/api/quit")
    def quit_app(request: Request):
        """Lets a newly started copy of the app replace this one (local only)."""
        if request.client and request.client.host not in ("127.0.0.1", "::1", "localhost"):
            raise HTTPException(403, "only from this computer")
        if jobs.busy():
            raise HTTPException(409, "a job is running")
        import signal
        import threading
        threading.Timer(0.3, os.kill, (os.getpid(), signal.SIGTERM)).start()
        return {"quitting": True}

    @app.get("/api/settings")
    def get_settings():
        c = cfg()
        return {"config": c, "yaml": yaml.safe_dump(c, sort_keys=False)}

    @app.put("/api/settings")
    def put_settings(body: dict = Body(...)):
        if "yaml" in body:
            try:
                new = yaml.safe_load(body["yaml"]) or {}
            except yaml.YAMLError as e:
                raise HTTPException(400, "settings are not valid YAML: %s" % e)
            if not isinstance(new, dict):
                raise HTTPException(400, "settings must be a mapping")
            if body.get("keep_work_dir"):
                new["work_dir"] = cfg().get("work_dir")
            c = config_mod.save_local(new, replace=True)
            return {"config": c, "yaml": yaml.safe_dump(c, sort_keys=False)}
        else:
            changes = body.get("changes") or {}
        c = config_mod.save_local(changes)
        return {"config": c, "yaml": yaml.safe_dump(c, sort_keys=False)}

    # --- recordings -------------------------------------------------------------------
    @app.get("/api/recordings")
    def recordings():
        return {"recordings": _scan_folders(cfg()),
                "folders": [str(p) for p in config_mod.data_folders(cfg())]}

    @app.post("/api/recordings/inspect")
    def inspect_recording(body: dict = Body(...)):
        path = body.get("path")
        if not path or not Path(path).is_file():
            raise HTTPException(404, "file not found")
        info = mcap_io.inspect(path, scan=bool(body.get("scan", True))).to_dict()
        t = cfg().get("topics") or {}
        have = {x["topic"] for x in info["topics"]}
        need = {}
        for k, v in t.items():
            if k == "navsat":
                v = next((c for c in poses.navsat_candidates(cfg()) if c in have), v)
            need[k] = {"topic": v, "present": v in have}
        info["needed"] = need
        return info

    # --- maps ---------------------------------------------------------------------------
    @app.get("/api/maps")
    def maps():
        return {"maps": pipeline.list_projects(cfg())}

    @app.post("/api/maps")
    def create_map(body: dict = Body(...)):
        name = (body.get("name") or "").strip()
        recs = body.get("recordings") or []
        if not name:
            raise HTTPException(400, "give the map a name")
        if not recs:
            raise HTTPException(400, "pick at least one recording")
        p = pipeline.Project.create(cfg(), name, recs)
        return p.status()

    @app.put("/api/maps/{slug}")
    def update_map(slug: str, body: dict = Body(...)):
        p = project(slug)
        if jobs.busy():
            raise HTTPException(409, "wait for the running job to finish")
        if "recordings" in body:
            recs = [str(r) for r in body["recordings"]]
            if not recs:
                raise HTTPException(400, "a map needs at least one recording")
            old = p.recordings
            if recs[:len(old)] != old[:len(recs)] or len(recs) < len(old):
                # order changed or recordings removed: cached scans no longer line up
                shutil.rmtree(p.root / "cache", ignore_errors=True)
            p.meta["recordings"] = recs
            p.meta["ranges"] = {k: v for k, v in (p.meta.get("ranges") or {}).items() if k in recs}
            p.save()
        if "road_names" in body:
            try:
                p.set_road_names(dict(body["road_names"] or {}))
            except (TypeError, ValueError):
                raise HTTPException(400, "road names must be {road id: name}")
        if "ranges" in body:
            # {recording path: {"unit", "start", "end"} or null}; a changed range
            # gets a fresh scan on the next build (the cache key includes it)
            ranges = dict(p.meta.get("ranges") or {})
            for rec, r in (body.get("ranges") or {}).items():
                if rec not in p.recordings:
                    raise HTTPException(400, "%s is not in this map" % rec)
                try:
                    r = pipeline.clean_range(r)
                except (TypeError, ValueError) as e:
                    raise HTTPException(400, str(e))
                if r != ranges.get(rec):
                    shutil.rmtree(p.cache(p.recordings.index(rec)), ignore_errors=True)
                if r:
                    ranges[rec] = r
                else:
                    ranges.pop(rec, None)
            p.meta["ranges"] = ranges
            p.save()
            for s_ in ("scan", "map", "roads", "export"):
                p.mark(s_, False)
        return p.status()

    @app.delete("/api/maps/{slug}")
    def delete_map(slug: str):
        """Delete a map's folder: outputs, scan cache and its snapshots. The
        recordings themselves are never touched."""
        p = project(slug)
        j = jobs.snapshot()
        if jobs.busy() and j and (j.get("args") or {}).get("slug") == slug:
            raise HTTPException(409, "stop the job running on this map first")
        grids.clear()
        scans.clear()
        trash = p.root.with_name(".deleting_" + p.root.name)
        p.root.rename(trash)
        shutil.rmtree(trash, ignore_errors=True)
        return {"deleted": slug}

    @app.get("/api/maps/{slug}")
    def map_status(slug: str):
        p = project(slug)
        st = p.status()
        st["can_start"] = pipeline.steps_available(p)
        return st

    @app.post("/api/maps/{slug}/build")
    def build(slug: str, body: dict = Body(default={})):
        p = project(slug)
        start = body.get("start") or "scan"
        if start not in pipeline.steps_available(p):
            raise HTTPException(400, "run the earlier steps first")
        try:
            return jobs.start("build", cfg(), {"name": p.name, "slug": slug, "start": start})
        except RuntimeError as e:
            raise HTTPException(409, str(e))

    # --- accuracy ------------------------------------------------------------------------
    @app.post("/api/maps/{slug}/accuracy")
    def start_accuracy(slug: str):
        p = project(slug)
        if not (p.root / "grid" / "grid.json").is_file():
            raise HTTPException(400, "build the map first")
        try:
            return jobs.start("accuracy", cfg(), {"name": p.name, "slug": slug})
        except RuntimeError as e:
            raise HTTPException(409, str(e))

    @app.get("/api/maps/{slug}/accuracy")
    def get_accuracy(slug: str):
        f = project(slug).root / "accuracy.json"
        return Response(f.read_text() if f.is_file() else "null", media_type="application/json")

    @app.get("/api/maps/{slug}/xodr_check")
    def xodr_check_result(slug: str):
        """OpenDRIVE checks of the exported .xodr (run now if missing or stale)."""
        from uga_terrain import export, xodr_check
        p = project(slug)
        xs = sorted(p.out.glob("*.xodr")) if p.out.is_dir() else []
        if not xs:
            return Response("null", media_type="application/json")
        f = p.out / "xodr_check.json"
        if not f.is_file() or f.stat().st_mtime < xs[0].stat().st_mtime:
            tr = p.out / "trajectory.csv"
            export.save_json(xodr_check.check(xs[0], [tr] if tr.is_file() else []), f)
        return Response(f.read_text(), media_type="application/json")

    @app.get("/api/maps/{slug}/lanes")
    def lanes_geometry(slug: str):
        """Lanes, connecting lanes and centre lines of the exported .xodr, for the map."""
        from uga_terrain import export, xodr_check
        p = project(slug)
        xs = sorted(p.out.glob("*.xodr")) if p.out.is_dir() else []
        if not xs:
            return Response("null", media_type="application/json")
        f = p.out / "lanes_geometry.json"
        if not f.is_file() or f.stat().st_mtime < xs[0].stat().st_mtime:
            export.save_json(xodr_check.lane_geometry(xs[0]), f)
        return Response(f.read_text(), media_type="application/json")

    @app.get("/api/maps/{slug}/section")
    def road_section(slug: str, road: int, s: float, half_width: float = Query(15.0, ge=2, le=40)):
        """Raw ground points of every scan in a thin slice across a road."""
        from uga_terrain import accuracy
        p = project(slug)
        rd = [r for r in pipeline.load_roads(p) if r.id == road]
        if not rd:
            raise HTTPException(404, "no road %d" % road)
        fz = [p.cache(i) / "frames.npz" for i in range(len(p.recordings))] + [p.root / "align.json"]
        key = (str(p.root),) + tuple(f.stat().st_mtime if f.is_file() else 0 for f in fz)
        if key not in scans:
            scans.clear()
            try:
                xyz, _rng, scan, _rec = accuracy.load_scans(p)
            except RuntimeError as e:
                raise HTTPException(400, str(e))
            scans[key] = (xyz.astype(np.float32), scan)
        xyz, scan = scans[key]
        return accuracy.section(p, rd[0], s, xyz, scan, half_width=half_width)

    @app.get("/api/job")
    def job():
        return {"job": jobs.snapshot()}

    @app.post("/api/job/stop")
    def stop():
        jobs.stop()
        return {"job": jobs.snapshot()}

    # --- results ------------------------------------------------------------------------
    @app.get("/api/maps/{slug}/previews")
    def previews(slug: str):
        p = project(slug)
        f = p.out / "previews" / "previews.json"
        if not f.is_file():
            return {}
        d = json.loads(f.read_text())
        if "relief" not in d and (p.root / "grid" / "grid.json").is_file() and not jobs.busy():
            # map built before the shaded-relief layer existed: add it now
            from uga_terrain import export
            d = export.previews(grid_of(p), p.out / "previews")
            export.save_json(d, f)
        return d

    @app.get("/api/maps/{slug}/preview/{name}")
    def preview_png(slug: str, name: str):
        f = project(slug).out / "previews" / Path(name).name
        if not f.is_file():
            raise HTTPException(404, "no preview")
        return FileResponse(f, headers={"Cache-Control": "no-cache"})

    @app.get("/api/maps/{slug}/roads")
    def roads(slug: str):
        f = project(slug).root / "roads.json"
        return Response(f.read_text() if f.is_file() else "[]", media_type="application/json")

    @app.get("/api/maps/{slug}/edits")
    def get_edits(slug: str):
        from uga_terrain import xodr as xodr_mod
        p = project(slug)
        c = cfg()
        roads = [{"id": int(d["id"]), "stub": bool((d.get("meta") or {}).get("stub")),
                  "details": xodr_mod.road_details(c, p.edits(), d["id"])}
                 for d in json.loads((p.root / "roads.json").read_text())] if (p.root / "roads.json").is_file() else []
        return {"edits": p.edits(), "roads": roads, "road_types": list(xodr_mod.ROAD_TYPES),
                "lane_types": list(xodr_mod.LANE_TYPES),
                "defaults": xodr_mod.road_details(c, None, -1)}

    @app.put("/api/maps/{slug}/edits")
    def put_edits(slug: str, body: dict = Body(...)):
        p = project(slug)
        if jobs.busy():
            raise HTTPException(409, "wait for the running job to finish")
        if not (p.root / "roads.json").is_file():
            raise HTTPException(400, "build the map first")
        p.set_edits(body.get("edits") or {})
        try:
            res = pipeline.apply_edits(p, cfg())
        except Exception as e:  # keep the app up; say what went wrong
            raise HTTPException(500, "the .xodr could not be rewritten: %s" % e)
        return dict(res, edits=p.edits())

    @app.get("/api/maps/{slug}/junctions")
    def junctions(slug: str):
        f = project(slug).root / "junctions.json"
        if not f.is_file():
            return []
        return [{k: v for k, v in j.items() if k != "paths"} for j in json.loads(f.read_text())]

    @app.get("/api/maps/{slug}/drives")
    def drives(slug: str):
        p = project(slug)
        out = []
        for i, rec in enumerate(p.recordings):
            fz = p.cache(i) / "frames.npz"
            if not fz.is_file():
                continue
            from uga_terrain import register
            T = register.poses_of(p, i)
            step = max(1, len(T) // 3000)
            out.append({"recording": Path(rec).name,
                        "x": np.round(T[::step, 0, 3], 2).tolist(),
                        "y": np.round(T[::step, 1, 3], 2).tolist()})
        return out

    @app.get("/api/maps/{slug}/sample")
    def sample(slug: str, x: float, y: float):
        g = grid_of(project(slug))
        vals = {}
        for layer in ("z", "slope_deg", "aspect_deg", "roughness", "intensity", "n", "label",
                      "filled"):
            v = float(g.sample(np.array([x]), np.array([y]), layer)[0])
            vals[layer] = None if not np.isfinite(v) else v
        lab = vals.get("label")
        vals["surface"] = LABELS.get(int(lab), "none") if lab is not None else "none"
        return vals

    @app.get("/api/maps/{slug}/cloud")
    def cloud(slug: str, max_points: int = Query(600000, ge=1000, le=5000000)):
        from uga_terrain.export import read_ply_points
        f = project(slug).out / "ground.ply"
        if not f.is_file():
            raise HTTPException(404, "build the map first")
        a = read_ply_points(f, max_points)
        xyz = np.stack([a["x"], a["y"], a["z"]], 1).astype("<f4")
        head = struct.pack("<I", len(a))
        return Response(head + xyz.tobytes() + a["label"].astype("u1").tobytes(),
                        media_type="application/octet-stream")

    @app.get("/api/maps/{slug}/surface")
    def surface(slug: str, max_px: int = Query(900, ge=64, le=2048)):
        """Height grid for the 3D view: u32 w, u32 h, f32 x0 y0 x1 y1, f32 z[h*w]
        (NaN = no ground, rows north to south), u8 label[h*w]."""
        g = grid_of(project(slug))
        z, ext = g.mosaic("z", max_px)
        lab, _ = g.mosaic("label", max_px, how="nearest")
        lab = np.nan_to_num(lab, nan=0).astype("u1")
        h, w = z.shape
        head = struct.pack("<II4f", w, h, *ext)
        return Response(head + z.astype("<f4").tobytes() + lab.tobytes(),
                        media_type="application/octet-stream")

    @app.get("/api/maps/{slug}/files")
    def files(slug: str):
        out = project(slug).out
        rows = []
        if out.is_dir():
            for f in sorted(out.iterdir()):
                if f.is_file():
                    rows.append({"name": f.name, "size": f.stat().st_size,
                                 "mtime": f.stat().st_mtime})
        has_rasters = (out / "rasters").is_dir()
        return {"folder": str(out), "files": rows, "rasters": has_rasters}

    @app.get("/api/maps/{slug}/files/{name}")
    def download(slug: str, name: str):
        f = project(slug).out / Path(name).name
        if not f.is_file():
            raise HTTPException(404, "no such file")
        return FileResponse(f, filename=f.name)

    @app.get("/api/maps/{slug}/rasters.zip")
    def rasters_zip(slug: str):
        p = project(slug)
        rdir = p.out / "rasters"
        if not rdir.is_dir():
            raise HTTPException(404, "build the map first")
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(rdir.rglob("*")):
                if f.is_file():
                    z.write(f, f.relative_to(rdir))
        buf.seek(0)
        return StreamingResponse(buf, media_type="application/zip", headers={
            "Content-Disposition": 'attachment; filename="%s_rasters.zip"' % p.root.name})

    # --- export / import / snapshots -----------------------------------------------------
    def _no_job():
        if jobs.busy():
            raise HTTPException(409, "wait for the running job to finish")

    @app.get("/api/maps/{slug}/export")
    def export_map(slug: str, cache: bool = False):
        p = project(slug)
        tmp = tempfile.NamedTemporaryFile(suffix=".zip", delete=False)
        tmp.close()
        archive.export_project(p.root, tmp.name, include_cache=cache)
        name = "%s_%s.terrainmap.zip" % (p.root.name, time.strftime("%Y%m%d-%H%M"))
        return FileResponse(tmp.name, filename=name, media_type="application/zip",
                            background=_cleanup(tmp.name))

    @app.post("/api/maps/import")
    async def import_map(request: Request, filename: str = "map.zip"):
        _no_job()
        tmp = tempfile.NamedTemporaryFile(suffix=".zip", delete=False)
        try:
            with tmp:
                async for chunk in request.stream():
                    tmp.write(chunk)
            try:
                slug = archive.import_project(tmp.name, config_mod.work_dir(cfg()) / "projects",
                                              fallback_name=Path(filename).stem)
            except ValueError as e:
                raise HTTPException(400, str(e))
        finally:
            os.unlink(tmp.name)
        return pipeline.Project(config_mod.work_dir(cfg()) / "projects" / slug).status()

    @app.get("/api/maps/{slug}/snapshots")
    def snapshots(slug: str):
        return {"snapshots": archive.list_snapshots(project(slug).root)}

    @app.post("/api/maps/{slug}/snapshots")
    def save_snapshot(slug: str, body: dict = Body(default={})):
        _no_job()
        p = project(slug)
        return archive.save_snapshot(p.root, (body.get("label") or "").strip(),
                                     yaml.safe_dump(cfg(), sort_keys=False))

    @app.post("/api/maps/{slug}/snapshots/{sid}/restore")
    def restore_snapshot(slug: str, sid: str, body: dict = Body(default={})):
        _no_job()
        p = project(slug)
        settings_yaml = archive.restore_snapshot(p.root, sid)
        if body.get("settings", True) and settings_yaml:
            new = yaml.safe_load(settings_yaml) or {}
            new["work_dir"] = cfg().get("work_dir")   # never move the maps folder on a restore
            config_mod.save_local(new, replace=True)
        grids.clear()
        return p.status()

    @app.delete("/api/maps/{slug}/snapshots/{sid}")
    def delete_snapshot(slug: str, sid: str):
        archive.delete_snapshot(project(slug).root, sid)
        return {"ok": True}

    @app.get("/api/settings/export")
    def export_settings():
        return Response(yaml.safe_dump(cfg(), sort_keys=False), media_type="application/x-yaml",
                        headers={"Content-Disposition":
                                 'attachment; filename="uga_terrain_settings.yaml"'})

    @app.exception_handler(FileNotFoundError)
    async def not_found(_req, exc):
        return JSONResponse({"detail": str(exc)}, status_code=404)

    @app.exception_handler(Exception)
    async def err(_req, exc):
        return JSONResponse({"detail": str(exc)}, status_code=500)

    @app.middleware("http")
    async def no_stale_static(request, call_next):
        resp = await call_next(request)
        if request.url.path.startswith("/static/") and not request.url.path.startswith("/static/vendor/"):
            resp.headers["Cache-Control"] = "no-cache"
        return resp

    app.mount("/static", StaticFiles(directory=STATIC), name="static")
    return app


def serve(host: str = "127.0.0.1", port: int = 8090) -> None:
    import faulthandler
    import uvicorn
    c = config_mod.load_config()
    wd = config_mod.work_dir(c)
    crash = str(wd / "app_crash.log")
    f = open(crash, "a")
    f.write("\n=== app started %s ===\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
    f.flush()
    faulthandler.enable(file=f, all_threads=True)
    serve.crash_file = f  # keep open
    # jobs run in a fresh Python process; make it load this same copy of the code
    # even when the environment was installed from another copy of the folder
    code_root = str(Path(__file__).resolve().parents[2])
    pp = os.environ.get("PYTHONPATH", "")
    if code_root not in pp.split(os.pathsep):
        os.environ["PYTHONPATH"] = code_root + (os.pathsep + pp if pp else "")
    print("UGA terrain toolkit  http://%s:%d   (v%s)" % (host, port, __version__))
    print("code from            %s" % code_root)
    print("recordings from      %s" % ", ".join(str(p) for p in config_mod.data_folders(c)))
    print("maps and outputs in  %s" % wd)
    print("crash log            %s" % crash)
    uvicorn.run(create_app(crash), host=host, port=port, log_level="warning",
                loop="asyncio", http="h11")


if __name__ == "__main__":
    serve(os.environ.get("UGA_TERRAIN_HOST", "127.0.0.1"),
          int(os.environ.get("UGA_TERRAIN_PORT", "8090")))
