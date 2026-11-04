"""Map export/import and snapshots."""
import json

import pytest

from uga_terrain.app import archive


def _fake_map(root):
    root.mkdir(parents=True)
    (root / "project.json").write_text(json.dumps({"name": "Campus", "recordings": ["a.mcap"]}))
    (root / "roads.json").write_text("[1]")
    (root / "grid").mkdir()
    (root / "grid" / "grid.json").write_text("{}")
    (root / "cache" / "rec_00").mkdir(parents=True)
    (root / "cache" / "rec_00" / "frames.npz").write_bytes(b"x" * 100)


def test_export_import_round_trip(tmp_path):
    projects = tmp_path / "projects"
    _fake_map(projects / "Campus")
    z = tmp_path / "m.zip"
    archive.export_project(projects / "Campus", str(z), include_cache=False)
    slug = archive.import_project(str(z), projects)
    assert slug == "Campus_2"
    new = projects / slug
    assert json.loads((new / "project.json").read_text())["name"] == "Campus 2"
    assert (new / "roads.json").read_text() == "[1]"
    assert not (new / "cache").exists()


def test_import_rejects_other_zips(tmp_path):
    import zipfile
    z = tmp_path / "x.zip"
    with zipfile.ZipFile(z, "w") as f:
        f.writestr("hello.txt", "hi")
    with pytest.raises(ValueError):
        archive.import_project(str(z), tmp_path / "projects")


def test_snapshot_restore(tmp_path):
    root = tmp_path / "Campus"
    _fake_map(root)
    info = archive.save_snapshot(root, "before", "grid:\n  cell: 0.2\n")
    (root / "roads.json").write_text("[2]")
    (root / "extra.txt").write_text("new")
    assert archive.restore_snapshot(root, info["id"]).startswith("grid")
    assert (root / "roads.json").read_text() == "[1]"
    assert not (root / "extra.txt").exists()
    assert (root / "cache" / "rec_00" / "frames.npz").is_file()
    assert [s["label"] for s in archive.list_snapshots(root)] == ["before"]
    archive.delete_snapshot(root, info["id"])
    assert archive.list_snapshots(root) == []
