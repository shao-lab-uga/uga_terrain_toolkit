"""A synthetic street for tests: graded, banked road, curbs, verge, sidewalks.

Coordinates: the car drives along +x at y = 0 (the reference line). Left is +y.
"""
from __future__ import annotations

import math

import numpy as np

GRADE = 0.04            # road rises 4 % along x
BANK_DEG = 3.0          # left side higher
ROAD_L, ROAD_R = 4.0, 3.5
CURB = 0.15
VERGE_L = 1.5           # grass between the left curb and the left sidewalk
WALK_L, WALK_R = 1.8, 2.0
GRASS = 0.045           # RMS roughness of grass (m)


def surface(x, y, rng=None):
    """Height and label (1 road, 2 sidewalk, 3 curb face, 4 other) of the ground."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    tb = math.tan(math.radians(BANK_DEG))
    z = GRADE * x + tb * np.clip(y, -ROAD_R, ROAD_L)
    lab = np.full(x.shape, 4, np.int8)
    road = (y <= ROAD_L) & (y >= -ROAD_R)
    lab[road] = 1
    # left: curb, verge (grass), sidewalk, grass
    left = y > ROAD_L
    z = z + np.where(left, CURB, 0.0)
    wl0 = ROAD_L + VERGE_L
    walk_l = (y >= wl0) & (y <= wl0 + WALK_L)
    lab[walk_l] = 2
    # right: curb then sidewalk straight away
    right = y < -ROAD_R
    z = z + np.where(right, CURB, 0.0)
    walk_r = (y <= -ROAD_R) & (y >= -ROAD_R - WALK_R)
    lab[walk_r] = 2
    if rng is not None:
        noise = np.where(lab == 4, GRASS, 0.006)
        z = z + rng.normal(0, 1, x.shape) * noise
    else:
        # fixed fine texture so ray-cast grass is rough too (RMS ~ GRASS)
        tex = np.sin(31.0 * x + 3.0 * np.sin(7.0 * y)) * np.sin(29.0 * y + 2.0 * np.sin(5.0 * x))
        z = z + np.where(lab == 4, 2.0 * GRASS * tex, 0.0)
    return z, lab


def ground_points(x0=0.0, x1=120.0, ymax=12.0, density=150.0, seed=0):
    rng = np.random.default_rng(seed)
    n = int((x1 - x0) * 2 * ymax * density)
    x = rng.uniform(x0, x1, n)
    y = rng.uniform(-ymax, ymax, n)
    z, lab = surface(x, y, rng)
    return np.stack([x, y, z, np.full(n, 20.0)], 1), lab
