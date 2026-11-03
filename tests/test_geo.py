import numpy as np

from uga_terrain import geo


def test_enu_round_trip():
    origin = (33.95, -83.37, 200.0)
    e, n, u = 120.0, -45.0, 3.0
    lla = geo.enu_to_lla(e, n, u, origin)
    back = geo.lla_to_enu(lla[0], lla[1], lla[2], origin)
    assert np.allclose(np.ravel(back), [e, n, u], atol=1e-4)


def test_rotation_round_trips():
    R = geo.rpy_to_mat(0.05, -0.1, 1.2)
    assert np.allclose(geo.mat_to_rpy(R), [0.05, -0.1, 1.2])
    assert np.allclose(geo.quat_to_mat(geo.mat_to_quat(R)), R)
    assert np.allclose(geo.rotvec_to_mat(geo.mat_to_rotvec(R)), R)


def test_wrap_angle():
    assert np.isclose(geo.wrap_angle(3 * np.pi), np.pi) or np.isclose(geo.wrap_angle(3 * np.pi), -np.pi)
