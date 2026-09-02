"""Shared builders for the raven_nav unit tests (pure numpy, no ROS)."""
from __future__ import annotations

import numpy as np

from raven_nav.behaviors.common import TickContext


def ctx(**kw) -> TickContext:
    """A TickContext with sane defaults; override any field by keyword."""
    base = dict(
        cur_pose=np.zeros(3),
        now=0.0,
        query_labels=['person', 'car', 'road'],
        target_objects=['person'],
        min_altitude=1.5,
        max_altitude=100.0,
    )
    base.update(kw)
    if 'cur_pose' in base:
        base['cur_pose'] = np.asarray(base['cur_pose'], dtype=float)
    return TickContext(**base)


def rays(origins, dirs, scores) -> dict:
    """kwargs for ctx(): ray arrays as (N,3)/(N,3)/(N,Q)."""
    o = np.asarray(origins, dtype=float).reshape(-1, 3)
    d = np.asarray(dirs, dtype=float).reshape(-1, 3)
    d = d / np.linalg.norm(d, axis=1, keepdims=True)
    return dict(ray_origins=o, ray_dirs=d,
                ray_scores=np.asarray(scores, dtype=float).reshape(o.shape[0], -1))


def frontier_cloud_flu(points_flu) -> np.ndarray:
    """FLU points -> the (N,6) RDF cloud the node hands the behaviour.
    Inverse of x=z, y=-x, z=-y, i.e. RDF = (-y_flu, -z_flu, x_flu)."""
    p = np.asarray(points_flu, dtype=float).reshape(-1, 3)
    out = np.zeros((p.shape[0], 6), dtype=float)
    out[:, 0] = -p[:, 1]
    out[:, 1] = -p[:, 2]
    out[:, 2] = p[:, 0]
    return out


def voxel_box(center, half_extent=1.0, step=0.5) -> np.ndarray:
    """A solid FLU voxel block centred on `center`, on the 0.5 m grid."""
    c = np.asarray(center, dtype=float)
    r = np.arange(-half_extent, half_extent + 1e-9, step)
    g = np.stack(np.meshgrid(r, r, r, indexing='ij'), axis=-1).reshape(-1, 3)
    return c[None, :] + g


def scores_for(n: int, q: int, col: int, value: float,
               other: float = 0.01) -> np.ndarray:
    s = np.full((n, q), other, dtype=float)
    s[:, col] = value
    return s
