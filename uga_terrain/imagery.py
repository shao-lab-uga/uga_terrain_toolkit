"""Aerial photo under the map, downloaded by the map's GPS position.

The default source is the USGS National Map orthoimagery service, which is free and
public domain for the United States. The photo is asked for in plain latitude /
longitude (north up), so over the few hundred metres of a map it lines up with the
map's east / north axes; its corners are converted back to map metres and saved
beside the other 2D map layers as ``satellite.jpg`` + ``satellite.json``.
"""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from uga_terrain import geo

DEFAULTS = {
    "url": ("https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/export"
            "?bbox={west},{south},{east},{north}&bboxSR=4326&imageSR=4326"
            "&size={width},{height}&format=jpg&f=image"),
    "label": "Aerial photo (USGS)",
    "credit": "Imagery: USGS The National Map (public domain)",
    "m_per_px": 0.25,
    "max_px": 4096,
    "timeout": 30.0,
}


def _settings(cfg) -> dict:
    s = dict(DEFAULTS)
    s.update({k: v for k, v in ((cfg or {}).get("imagery") or {}).items() if v is not None})
    return s


def _download(url: str, timeout: float) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "uga-terrain-toolkit"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def _image_kind(data: bytes) -> Optional[str]:
    if data[:3] == b"\xff\xd8\xff":
        return "jpg"
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return "png"
    return None


def request_box(extent, origin, cfg=None) -> dict:
    """Latitude / longitude box and pixel size covering the map ``extent`` [x0, y0, x1, y1]."""
    s = _settings(cfg)
    x0, y0, x1, y1 = (float(v) for v in extent)
    xs = np.array([x0, x1, x1, x0])
    ys = np.array([y0, y0, y1, y1])
    lla = geo.enu_to_lla(xs, ys, np.zeros(4), origin)
    box = {"west": float(lla[:, 1].min()), "east": float(lla[:, 1].max()),
           "south": float(lla[:, 0].min()), "north": float(lla[:, 0].max())}
    w_m, h_m = x1 - x0, y1 - y0
    m = max(float(s["m_per_px"]), max(w_m, h_m) / float(s["max_px"]))
    box["width"] = max(16, int(math.ceil(w_m / m)))
    box["height"] = max(16, int(math.ceil(h_m / m)))
    return box


def box_extent(box, origin) -> list:
    """Map metres [x0, y0, x1, y1] of a latitude / longitude box (edges averaged)."""
    lat = np.array([box["south"], box["south"], box["north"], box["north"]])
    lon = np.array([box["west"], box["east"], box["east"], box["west"]])
    enu = geo.lla_to_enu(lat, lon, np.zeros(4), origin)
    x0 = (enu[0, 0] + enu[3, 0]) / 2
    x1 = (enu[1, 0] + enu[2, 0]) / 2
    y0 = (enu[0, 1] + enu[1, 1]) / 2
    y1 = (enu[2, 1] + enu[3, 1]) / 2
    return [round(float(v), 3) for v in (x0, y0, x1, y1)]


def load(prev_dir: Path, origin) -> Optional[dict]:
    """The saved aerial-photo layer, if there is one for this map origin."""
    f = Path(prev_dir) / "satellite.json"
    if not f.is_file():
        return None
    d = json.loads(f.read_text())
    if not origin or not d.get("origin") or \
            max(abs(a - b) for a, b in zip(d["origin"][:2], origin[:2])) > 1e-7 or \
            not (Path(prev_dir) / d.get("file", "")).is_file():
        return None
    return d


def fetch(prev_dir: Path, extent, origin, cfg=None,
          fetcher: Optional[Callable[[str, float], bytes]] = None) -> dict:
    """Download the photo for ``extent`` and save it in ``prev_dir``. Raises RuntimeError
    with a readable message when it cannot be fetched."""
    if not origin:
        raise RuntimeError("this map has no GPS position, so there is no aerial photo to line up")
    s = _settings(cfg)
    origin = tuple(float(v) for v in origin)
    box = request_box(extent, origin, cfg)
    url = s["url"].format(**box)
    try:
        data = (fetcher or _download)(url, float(s["timeout"]))
    except urllib.error.HTTPError as e:
        raise RuntimeError("the imagery server answered %d %s" % (e.code, e.reason))
    except (urllib.error.URLError, OSError) as e:
        why = getattr(e, "reason", e)
        raise RuntimeError("could not reach the imagery server (%s). Is this computer online?" % why)
    kind = _image_kind(data)
    if kind is None:
        text = data[:300].decode("utf-8", "replace").strip()
        raise RuntimeError("the imagery server did not send a picture: %s" % (text or "empty answer"))
    prev_dir = Path(prev_dir)
    prev_dir.mkdir(parents=True, exist_ok=True)
    for old in prev_dir.glob("satellite.*"):
        if old.suffix in (".jpg", ".png"):
            old.unlink()
    name = "satellite." + kind
    (prev_dir / name).write_bytes(data)
    d = {"file": name, "extent": box_extent(box, origin), "width": box["width"], "height": box["height"],
         "label": s["label"], "credit": s["credit"], "origin": list(origin),
         "lonlat_box": [box["west"], box["south"], box["east"], box["north"]]}
    (prev_dir / "satellite.json").write_text(json.dumps(d, indent=1))
    return d
