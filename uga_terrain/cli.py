"""Command line entry points.

    uga-terrain app                      start the app (default)
    uga-terrain build --name NAME a.mcap b.mcap [--start map]
    uga-terrain inspect file.mcap
"""
from __future__ import annotations

import argparse
import json
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="uga-terrain", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    a = sub.add_parser("app", help="start the app in your browser")
    a.add_argument("--host", default=None)
    a.add_argument("--port", type=int, default=None)
    a.add_argument("--no-browser", action="store_true")
    b = sub.add_parser("build", help="build a map without the app")
    b.add_argument("--name", required=True, help="map name")
    b.add_argument("--start", default="scan", choices=["scan", "map", "roads", "export"])
    b.add_argument("files", nargs="*", help=".mcap recordings (omit to rebuild an existing map)")
    i = sub.add_parser("inspect", help="list a recording's topics and check the needed ones")
    i.add_argument("file")
    args = ap.parse_args(argv)

    from uga_terrain import config as config_mod
    cfg = config_mod.load_config()

    if args.cmd in (None, "app"):
        from uga_terrain.app.server import serve
        app_cfg = cfg.get("app") or {}
        host = getattr(args, "host", None) or app_cfg.get("host", "127.0.0.1")
        port = getattr(args, "port", None) or int(app_cfg.get("port", 8090))
        from uga_terrain.app.launch import claim_port
        port = claim_port(host, port)
        if not getattr(args, "no_browser", False) and app_cfg.get("open_browser", True):
            import threading
            import webbrowser
            threading.Timer(1.5, webbrowser.open, ("http://%s:%d" % (host, port),)).start()
        serve(host, port)
        return 0

    if args.cmd == "inspect":
        from uga_terrain import mcap_io
        info = mcap_io.inspect(args.file, scan=True).to_dict()
        have = {t["topic"] for t in info["topics"]}
        from uga_terrain.poses import navsat_candidates
        for k, v in (cfg.get("topics") or {}).items():
            if k == "navsat":
                v = next((c for c in navsat_candidates(cfg) if c in have), v)
            print("%-10s %-24s %s" % (k, v, "ok" if v in have else "MISSING"))
        print(json.dumps(info, indent=1, default=str))
        return 0

    if args.cmd == "build":
        from uga_terrain import pipeline
        from uga_terrain.progress import Progress
        if args.files:
            p = pipeline.Project.create(cfg, args.name, args.files)
        else:
            p = pipeline.Project.open(cfg, args.name)
        summary = pipeline.build(p, cfg, Progress(echo=True), start=args.start)
        print("outputs in", p.out)
        print(json.dumps({k: summary.get(k) for k in ("terrain", "roads")}, indent=1, default=str))
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
