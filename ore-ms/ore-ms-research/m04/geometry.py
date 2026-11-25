"""Deterministic first hits for opaque, world-aligned planar rectangles."""
from dataclasses import dataclass
import math

PARALLEL = 1e-12
MIN_RANGE = 1e-8
DEPTH_TIE = 1e-9


def dot(a, b):
    return math.fsum(x * y for x, y in zip(a, b, strict=True))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def finite_vector(vector, size):
    if len(vector) != size or not all(math.isfinite(value) for value in vector):
        raise ValueError("invalid vector")


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    origin: tuple
    axes: tuple

    def __post_init__(self):
        if (not isinstance(self.width, int) or not isinstance(self.height, int)
                or min(self.width, self.height) < 1):
            raise ValueError("invalid image dimensions")
        finite_vector((self.fx, self.fy, self.cx, self.cy), 4)
        finite_vector(self.origin, 3)
        if min(self.fx, self.fy) <= 0 or len(self.axes) != 3:
            raise ValueError("invalid focal length or orientation")
        for axis in self.axes:
            finite_vector(axis, 3)
        for i, first in enumerate(self.axes):
            for j, second in enumerate(self.axes):
                if abs(dot(first, second) - int(i == j)) > 1e-12:
                    raise ValueError("camera axes must be orthonormal")
        if dot(cross(*self.axes[:2]), self.axes[2]) < 1 - 1e-12:
            raise ValueError("camera axes must be right-handed")

    def world_to_camera(self, point):
        finite_vector(point, 3)
        offset = tuple(p - o for p, o in zip(point, self.origin, strict=True))
        return tuple(dot(axis, offset) for axis in self.axes)

    def project(self, point):
        x, y, z = self.world_to_camera(point)
        if z <= 0:
            raise ValueError("projection requires positive camera depth")
        return self.fx * x / z + self.cx, self.fy * y / z + self.cy

    def projection_jacobian(self, point):
        x, y, z = self.world_to_camera(point)
        if z <= 0:
            raise ValueError("projection requires positive camera depth")
        return ((self.fx / z, 0, -self.fx * x / z**2),
                (0, self.fy / z, -self.fy * y / z**2))

    def ray(self, column, row):
        finite_vector((column, row), 2)
        camera = ((column - self.cx) / self.fx, (row - self.cy) / self.fy, 1)
        norm = math.hypot(*camera)
        return tuple(dot(camera, coordinates) / norm for coordinates in zip(*self.axes))


@dataclass(frozen=True)
class Rectangle:
    id: str
    center: tuple
    half: tuple
    material: str
    target: bool = False
    velocity: tuple = (0, 0, 0)

    def __post_init__(self):
        finite_vector(self.center, 3)
        finite_vector(self.half, 2)
        finite_vector(self.velocity, 3)
        if min(self.half) <= 0 or not self.id:
            raise ValueError("invalid rectangle dimensions or ID")

    def at(self, time):
        if not math.isfinite(time):
            raise ValueError("time must be finite")
        return tuple(center + velocity * time
                     for center, velocity in zip(self.center, self.velocity, strict=True))


def intersect(origin, direction, center, half):
    """Return ray parameter for a unit direction, or None."""
    if abs(direction[2]) <= PARALLEL:
        return None
    distance = (center[2] - origin[2]) / direction[2]
    if distance < MIN_RANGE:
        return None
    if all(value - extent <= o + distance * d <= value + extent
           for o, d, value, extent in zip(origin[:2], direction[:2], center[:2], half, strict=True)):
        return distance
    return None


def hits(camera, direction, surfaces):
    """Return target-only range, nearest eligible surface, and first-hit range."""
    candidates = []
    target_range = None
    for primitive, center in surfaces:
        distance = intersect(camera.origin, direction, center, primitive.half)
        if distance is not None:
            candidates.append((primitive, distance))
            if primitive.target:
                target_range = distance
    if not candidates:
        return target_range, None, None
    nearest = min(distance for _, distance in candidates)
    tied = ((primitive, distance) for primitive, distance in candidates
            if distance - nearest <= DEPTH_TIE)
    first, distance = min(tied, key=lambda hit: (hit[0].target, hit[0].id))
    return target_range, first, distance
