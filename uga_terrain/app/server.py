"""The terrain toolkit app: a local web server and its browser UI."""
from __future__ import annotations

import io
import json
import os
import struct
import time
import zipfile
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml
from fastapi import Body, FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from uga_terrain import __version__, config as config_mod, mcap_io, pipeline, poses
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


def create_app(crash_log: Optional[str] = None) -> FastAPI:
    app = FastAPI(title="UGA Terrain Toolkit")
    jobs = JobManager(crash_log)
    grids: dict = {}

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
        return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})

    @app.get("/api/state")
    def state():
        c = cfg()
        return {"version": __version__, "work_dir": str(config_mod.work_dir(c)),
                "data_folders": [str(p) for p in config_mod.data_folders(c)],
                "kiss_icp": kiss_available(), "patchworkpp": patchwork_available(),
                "crash_log": crash_log, "job": jobs.snapshot()}

    # --- settings ---------------------------------------------------------------------
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
            changes = new
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
        info = mcap_io.inspect(path, scan=True).to_dict()
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
                import shutil
                shutil.rmtree(p.root / "cache", ignore_errors=True)
            p.meta["recordings"] = recs
            p.save()
        return p.status()

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
        f = project(slug).out / "previews" / "previews.json"
        if not f.is_file():
            return {}
        return json.loads(f.read_text())

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

    @app.get("/api/maps/{slug}/drives")
    def drives(slug: str):
        p = project(slug)
        out = []
        for i, rec in enumerate(p.recordings):
            fz = p.cache(i) / "frames.npz"
            if not fz.is_file():
                continue
            T = np.load(fz)["T"]
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

    @app.exception_handler(Exception)
    async def err(_req, exc):
        return JSONResponse({"detail": str(exc)}, status_code=500)

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
    print("UGA terrain toolkit  http://%s:%d" % (host, port))
    print("recordings from      %s" % ", ".join(str(p) for p in config_mod.data_folders(c)))
    print("maps and outputs in  %s" % wd)
    print("crash log            %s" % crash)
    uvicorn.run(create_app(crash), host=host, port=port, log_level="warning",
                loop="asyncio", http="h11")


if __name__ == "__main__":
    serve(os.environ.get("UGA_TERRAIN_HOST", "127.0.0.1"),
          int(os.environ.get("UGA_TERRAIN_PORT", "8090")))
