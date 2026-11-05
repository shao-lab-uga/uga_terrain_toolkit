"""Aerial photo layer: asked for over the map's area and placed back in map metres."""
import io
import json

import numpy as np
import pytest

from uga_terrain import geo, imagery
from uga_terrain.config import load_config

ORIGIN = (33.946971571916656, -83.37608003272604, 185.7)
EXTENT = [-102.4, -102.4, 102.4, 204.8]


def _jpg(w, h):
    from PIL import Image
    b = io.BytesIO()
    Image.fromarray(np.zeros((h, w, 3), np.uint8)).save(b, "JPEG")
    return b.getvalue()


def test_fetch_places_the_photo_in_map_metres(tmp_path):
    cfg = load_config()
    seen = []

    def fake(url, timeout):
        seen.append(url)
        return _jpg(32, 48)

    d = imagery.fetch(tmp_path, EXTENT, ORIGIN, cfg, fetcher=fake)
    assert "bbox=" in seen[0] and "size=820,1229" in seen[0]
    # the box covers the map and maps back onto it within a few cm
    assert all(abs(a - b) < 0.05 for a, b in zip(d["extent"], EXTENT)), d["extent"]
    w, s, e, n = d["lonlat_box"]
    back = geo.lla_to_enu(np.array([s, n]), np.array([w, e]), np.zeros(2), ORIGIN)
    assert abs(back[0, 0] - EXTENT[0]) < 0.05 and abs(back[1, 1] - EXTENT[3]) < 0.05
    assert (tmp_path / "satellite.jpg").is_file()
    assert imagery.load(tmp_path, ORIGIN)["file"] == "satellite.jpg"
    assert imagery.load(tmp_path, (34.0, -83.0, 0.0)) is None      # map moved: photo no longer fits
    assert json.loads((tmp_path / "satellite.json").read_text())["credit"].startswith("Imagery")


def test_fetch_explains_failures(tmp_path):
    with pytest.raises(RuntimeError, match="did not send a picture"):
        imagery.fetch(tmp_path, EXTENT, ORIGIN, None, fetcher=lambda u, t: b'{"error": "bad"}')

    def offline(url, timeout):
        raise OSError("network unreachable")
    with pytest.raises(RuntimeError, match="online"):
        imagery.fetch(tmp_path, EXTENT, ORIGIN, None, fetcher=offline)
    with pytest.raises(RuntimeError, match="GPS"):
        imagery.fetch(tmp_path, EXTENT, None, None, fetcher=offline)
