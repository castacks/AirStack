"""Transport-neutral helpers for read-only AirStack environment evidence."""
from __future__ import annotations

import math
from typing import Iterable


def finite_xyz_bounds(points: Iterable[tuple[float, float, float]]) -> dict[str, float] | None:
    """Return finite axis-aligned bounds while ignoring malformed/non-finite points."""
    bounds: list[float] | None = None
    for point in points:
        try:
            x, y, z = (float(point[index]) for index in range(3))
        except (IndexError, TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if bounds is None:
            bounds = [x, x, y, y, z, z]
        else:
            bounds = [
                min(bounds[0], x), max(bounds[1], x),
                min(bounds[2], y), max(bounds[3], y),
                min(bounds[4], z), max(bounds[5], z),
            ]
    if bounds is None:
        return None
    return dict(zip(("min_x", "max_x", "min_y", "max_y", "min_z", "max_z"), bounds))
