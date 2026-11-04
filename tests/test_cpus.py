import os

import pytest

from uga_terrain import cpus


def test_parse_and_resolve():
    assert cpus.resolve("all") is None
    assert cpus.resolve("16-18,20") == {16, 17, 18, 20}
    with pytest.raises(ValueError):
        cpus.resolve("fast ones")


@pytest.mark.skipif(not hasattr(os, "sched_setaffinity"), reason="Linux only")
def test_apply_restricts_this_process():
    before = os.sched_getaffinity(0)
    try:
        one = min(before)
        msg = cpus.apply(str(one))
        assert os.sched_getaffinity(0) == {one} and "1 of" in msg
        assert "using all cores" in (cpus.apply("e-cores") or "using all cores")
    finally:
        os.sched_setaffinity(0, before)
