"""
mesh_damage.py — structural damage applied to a building's actual geometry.

The asset-swap route (`disaster_stage.apply_to_buildings`) can only ruin a
building for which a same-footprint ruin asset exists. Where none does, the
generator has always fallen back to tilting and sinking the intact model, which
reads as "the building is drunk" rather than "the building failed". This
deforms the mesh instead.

PORTED FROM `scenegen/damage.py`, NOT IMPORTED
----------------------------------------------
That module is Blender-based and lives in a separate repo. `bpy` cannot run
inside Isaac Sim — not for the reason usually given (bpy 5.0.1 and the 4.5 LTS
line *do* ship cp311 wheels matching Kit's 3.11.13), but because bpy statically
links its own USD and bundles oneTBB 12, while Isaac ships `libusd_*.so` and
TBB 2 with a malloc proxy. Two USD runtimes and two allocators in one address
space is a crash waiting to happen. It is also GPL against AirStack's MIT.

None of that matters, because the operators barely need Blender. Every deformation
in `damage.py` is expressed as `deform(objs, fn)` where `fn` maps an ``(N, 3)``
world-space array to another — pure numpy. The entire Blender dependency was
`get_verts`/`set_verts`, seventeen lines. Swap those for a `UsdGeom.Mesh`
adapter and the operator maths ports verbatim, which is what this module is.

WHAT DID NOT COME ACROSS
------------------------
`voronoi_fracture` and `boolean_cut` need bmesh topology surgery, and `settle`
needs a rigid-body solver — Isaac has PhysX for that, and the generator already
marks props `settle=True` for it. `punch_hole` deletes faces, which is possible
on `faceVertexIndices` but is topology work rather than a vertex map, so it is
left for later. What remains is the set that makes a building *look* failed:
racking, collapse, spalling, blast displacement and scorching.

TWO THINGS THE OPERATORS ASSUME
-------------------------------
* **World space.** Points are read through the prim's local-to-world transform
  and written back through its inverse, so an epicentre is a position in the
  scene and not in some asset's authoring frame.
* **`Bounds` is a snapshot.** Deformation moves geometry out from under it, so
  a normalized point that sat on a wall before a `lean` may sit in open air
  afterwards. Re-measure between large operators; the profiles below do.
"""

import math

import numpy as np

# pxr is the one hard dependency, and it is present everywhere this runs:
# Isaac's Kit python, and `usd-core` on the host for the offline tests.
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

# ---------------------------------------------------------------------------
# the USD adapter — the seventeen lines that were Blender-specific
# ---------------------------------------------------------------------------


def deinstance(root: "Usd.Prim") -> int:
    """Make every instance under *root* editable. Returns how many were opened.

    Authoring to an instance proxy raises — its geometry lives in a shared
    prototype, and a per-instance opinion is exactly what USD instancing exists
    to forbid. The AEC packs ship *internally* instanced (a brownstone is
    `/World/Brownstone02_Instanced/...` with 307 meshes behind one instance
    root), so deforming one fails on the first `set_points` unless the
    instances are opened first.

    **This costs memory**, which is the whole point of instancing: each opened
    instance stops sharing its prototype's geometry. The generated scene
    already OOM-killed once at 89.1M points, so open instances only on the
    buildings actually being deformed — which is what `apply_to_stage` does —
    and never wholesale.

    Same constraint as the placement-level one in
    `scene_generator.apply_placements`: geometry you intend to edit per-prim
    cannot be instanced.
    """
    opened = 0
    # Repeat: opening an instance can expose nested instances beneath it.
    for _ in range(8):
        found = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
                 if p.IsInstance()]
        if not found:
            break
        for p in found:
            try:
                p.SetInstanceable(False)
                opened += 1
            except Exception:
                pass
    return opened


def mesh_prims(root: "Usd.Prim") -> list:
    """Every `UsdGeom.Mesh` at or under *root*.

    Traverses instance proxies because generated scenes reference their assets
    and the geometry lives inside the referenced layer. This is the same
    descent `generate_city_v2.prune_prims` and `apply_surface_overrides` do;
    they are the prior art for reaching into a placed asset.
    """
    if not root or not root.IsValid():
        return []
    out = []
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if p.GetTypeName() == "Mesh":
            out.append(p)
    return out


def _world_matrix(prim) -> np.ndarray:
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    return np.array(m, dtype=np.float64).reshape(4, 4)


def get_points(prim) -> np.ndarray:
    """World-space points of *prim*, shape (N, 3). Empty array if it has none.

    USD is row-vector: ``world = local @ M[:3, :3] + M[3, :3]``.
    """
    attr = UsdGeom.Mesh(prim).GetPointsAttr()
    pts = attr.Get() if attr else None
    if not pts:
        return np.zeros((0, 3), dtype=np.float64)
    local = np.array(pts, dtype=np.float64)
    M = _world_matrix(prim)
    return local @ M[:3, :3] + M[3, :3]


def set_points(prim, world: np.ndarray) -> None:
    """Write world-space points back through the prim's transform.

    Authors an *override* on the points attribute — the mesh lives in a
    referenced layer, and an opinion in a stronger layer composes over it.
    This is why damaged buildings cannot be instanceable: an instance's
    descendants live in a shared prototype and take no per-instance opinion.
    """
    M = _world_matrix(prim)
    Minv = np.linalg.inv(M)
    local = world @ Minv[:3, :3] + Minv[3, :3]
    UsdGeom.Mesh(prim).GetPointsAttr().Set(
        Vt_Vec3fArray(local))


def Vt_Vec3fArray(arr: np.ndarray):
    """(N, 3) float array -> Vt.Vec3fArray, the type the attribute wants."""
    from pxr import Vt
    return Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in arr])


def deform(prims, fn) -> None:
    """Apply ``fn(world_xyz) -> world_xyz`` to every prim's points."""
    for prim in prims:
        v = get_points(prim)
        if len(v):
            set_points(prim, fn(v))


class Bounds:
    """World-space AABB, plus normalized-building-space conversion.

    Normalized space puts (0, 0, 0) at the centre of the footprint at ground
    level: x, y in [-1, 1] and z in [0, 1] cover the building. Sizes are
    fractions of `radius`, which stops "a 1 m crack" meaning something
    different on every asset — the library spans 15 m houses to 90 m towers.
    """

    def __init__(self, lo, hi):
        self.lo = np.asarray(lo, dtype=np.float64)
        self.hi = np.asarray(hi, dtype=np.float64)

    @property
    def dims(self):
        return self.hi - self.lo

    @property
    def center(self):
        return (self.lo + self.hi) * 0.5

    @property
    def radius(self) -> float:
        return max(float(np.linalg.norm(self.dims)) * 0.5, 1e-6)

    @property
    def base_z(self) -> float:
        return float(self.lo[2])

    @property
    def height(self) -> float:
        return max(float(self.dims[2]), 1e-6)

    def to_world(self, p) -> np.ndarray:
        px, py, pz = p
        half = self.dims * 0.5
        c = self.center
        return np.array([c[0] + px * half[0],
                         c[1] + py * half[1],
                         self.lo[2] + pz * self.dims[2]], dtype=np.float64)

    def frac(self, f: float) -> float:
        return f * self.radius


def bounds_of(prims) -> "Bounds | None":
    """AABB over every prim's world points, or None when there is no geometry."""
    lo = np.array([np.inf] * 3)
    hi = np.array([-np.inf] * 3)
    seen = False
    for prim in prims:
        v = get_points(prim)
        if not len(v):
            continue
        seen = True
        lo = np.minimum(lo, v.min(axis=0))
        hi = np.maximum(hi, v.max(axis=0))
    return Bounds(lo, hi) if seen else None


# ---------------------------------------------------------------------------
# noise — ported verbatim; real damage is spatially correlated, and per-vertex
# random jitter just looks like sandpaper
# ---------------------------------------------------------------------------


def _lattice(points: np.ndarray, freq: float, seed: int, res: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    grid = rng.random((res, res, res)) * 2.0 - 1.0
    p = points * freq
    i0 = np.floor(p).astype(np.int64)
    t = p - i0
    t = t * t * (3.0 - 2.0 * t)                      # smoothstep
    c = [None] * 8
    for k, (dx, dy, dz) in enumerate([(a, b, cc) for a in (0, 1)
                                      for b in (0, 1) for cc in (0, 1)]):
        idx = (i0 + np.array([dx, dy, dz])) % res
        c[k] = grid[idx[:, 0], idx[:, 1], idx[:, 2]]
    tx, ty, tz = t[:, 0], t[:, 1], t[:, 2]
    c00 = c[0] * (1 - tz) + c[1] * tz
    c01 = c[2] * (1 - tz) + c[3] * tz
    c10 = c[4] * (1 - tz) + c[5] * tz
    c11 = c[6] * (1 - tz) + c[7] * tz
    c0 = c00 * (1 - ty) + c01 * ty
    c1 = c10 * (1 - ty) + c11 * ty
    return c0 * (1 - tx) + c1 * tx


def value_noise(points: np.ndarray, freq: float, seed: int,
                octaves: int = 3, res: int = 24) -> np.ndarray:
    """Scalar noise in roughly [-1, 1] sampled at *points* (N, 3)."""
    total = np.zeros(len(points))
    amp_sum = 0.0
    for o in range(octaves):
        total += (0.5 ** o) * _lattice(points, freq * (2 ** o),
                                       seed + o * 977, res)
        amp_sum += 0.5 ** o
    return total / max(amp_sum, 1e-9)


def _falloff(t: np.ndarray, kind: str = "smooth") -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    if kind == "linear":
        return 1.0 - t
    if kind == "sharp":
        return (1.0 - t) ** 3
    if kind == "shell":          # peaks at the wavefront, not at the source
        return np.exp(-((t - 0.6) ** 2) / 0.045)
    return 1.0 - t * t * (3.0 - 2.0 * t)


# ---------------------------------------------------------------------------
# operators
# ---------------------------------------------------------------------------


def lean(prims, bounds: Bounds, angle_deg: float, direction_deg: float = 0.0,
         profile: str = "linear") -> None:
    """Tilt off vertical — foundation failure or storey racking.

    Displacement grows with height, so the footprint stays put and the roof
    moves furthest. `profile`: 'linear' racks uniformly, 'soft_story' puts
    nearly all of it in the bottom fifth (the classic weak-ground-floor
    failure), 'whip' accelerates toward the top (tall, flexible structures).
    """
    a, d = math.radians(angle_deg), math.radians(direction_deg)
    dirx, diry = math.cos(d), math.sin(d)
    h, z0 = bounds.height, bounds.base_z

    def fn(v):
        t = np.clip((v[:, 2] - z0) / h, 0.0, 1.0)
        if profile == "soft_story":
            s = np.clip(t / 0.2, 0.0, 1.0)
        elif profile == "whip":
            s = t ** 2
        else:
            s = t
        out = v.copy()
        drift = math.tan(a) * h * s
        out[:, 0] += drift * dirx
        out[:, 1] += drift * diry
        return out

    deform(prims, fn)


def pancake(prims, bounds: Bounds, z_lo: float, z_hi: float,
            collapse: float = 0.7, spread: float = 0.15, seed: int = 0) -> None:
    """Crush a horizontal band and drop everything above it.

    Storey collapse. The band between *z_lo* and *z_hi* (normalized height) is
    compressed to ``1 - collapse`` of its thickness and everything above
    translates down by the height lost, so the upper storeys land on the rubble
    instead of floating. *spread* bulges the crushed band outward, as material
    squeezes out sideways.
    """
    h, z0 = bounds.height, bounds.base_z
    wz_lo, wz_hi = z0 + z_lo * h, z0 + z_hi * h
    band = max(wz_hi - wz_lo, 1e-9)
    lost = band * collapse
    cx, cy = bounds.center[0], bounds.center[1]

    def fn(v):
        out = v.copy()
        z = v[:, 2]
        inside = (z >= wz_lo) & (z <= wz_hi)
        above = z > wz_hi
        out[inside, 2] = wz_lo + (z[inside] - wz_lo) * (1.0 - collapse)
        out[above, 2] = z[above] - lost
        if spread:
            frac = np.zeros(len(v))
            frac[inside] = 1.0 - np.abs((z[inside] - wz_lo) / band - 0.5) * 2.0
            n = value_noise(v / bounds.radius, freq=3.0, seed=seed) * 0.5 + 1.0
            push = frac * spread * n
            out[:, 0] += (v[:, 0] - cx) * push
            out[:, 1] += (v[:, 1] - cy) * push
        return out

    deform(prims, fn)


def crumble(prims, bounds: Bounds, amount: float = 0.01, freq: float = 6.0,
            seed: int = 0, height_bias: float = 0.0) -> None:
    """Roughen surfaces with coherent noise — spalled concrete, buckled walls.

    *amount* is a fraction of the building radius. *height_bias* > 0 puts the
    damage up high (wind, blast), < 0 puts it at the base (ground shaking).
    """
    amp = bounds.frac(amount)
    h, z0, r = bounds.height, bounds.base_z, bounds.radius

    def fn(v):
        p = v / r
        n = np.stack([value_noise(p, freq, seed + i * 131) for i in range(3)],
                     axis=1)
        w = np.ones(len(v))
        if height_bias:
            t = np.clip((v[:, 2] - z0) / h, 0.0, 1.0)
            w = t ** 2 if height_bias > 0 else (1.0 - t) ** 2
            w = 1.0 - abs(height_bias) + abs(height_bias) * w
        return v + n * amp * w[:, None]

    deform(prims, fn)


def shockwave(prims, bounds: Bounds, epicenter, radius: float = 0.5,
              strength: float = 0.12, falloff: str = "smooth",
              seed: int = 0, roughness: float = 0.35,
              floor_z: float = None) -> None:
    """Push geometry radially away from a blast point.

    *epicenter* is in normalized building space; *radius* and *strength* are
    fractions of the building radius. Use ``falloff="shell"`` for a wavefront
    that has already passed the source — peak displacement at a ring rather
    than at the centre.

    *floor_z* clamps the result so nothing is driven below it. Without it a
    blast seated near the base pushes the geometry under it straight down and
    the building sinks — measured at **-1.29 m** on a real brownstone, which
    reads as the building being swallowed rather than blown. The ground is
    there; pass ``bounds.base_z``.
    """
    c = bounds.to_world(epicenter)
    R, S, r_norm = bounds.frac(radius), bounds.frac(strength), bounds.radius

    def fn(v):
        d = v - c
        dist = np.linalg.norm(d, axis=1)
        w = _falloff(dist / max(R, 1e-9), falloff)
        if roughness:
            w = w * (value_noise(v / r_norm, freq=5.0, seed=seed) * roughness
                     + 1.0)
        out = v + (d / np.maximum(dist, 1e-9)[:, None]) * (w * S)[:, None]
        if floor_z is not None:
            np.maximum(out[:, 2], floor_z, out=out[:, 2])
        return out

    deform(prims, fn)


def scorch(prims, strength: float = 0.85) -> None:
    """Darken the bound materials — soot, approximately.

    Reimplemented on `UsdShade` rather than ported: the Blender version walks
    material node trees. This follows the pattern already in
    `generate_city_v2.apply_surface_overrides` — resolve the bound material,
    find its surface shader, set inputs — so there is one way in this codebase
    to retune a placed asset's material, not two.

    Authored once per *material*, not per prim: the material lives in the
    referenced layer and is shared, so a second visit would compound the tint.
    """
    seen = set()
    w = max(0.0, min(1.0, strength))
    for prim in prims:
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        key = str(mat.GetPrim().GetPath())
        if key in seen:
            continue
        seen.add(key)
        shader = mat.ComputeSurfaceSource()[0]
        if not shader:
            continue
        _tint(shader, "diffuseColor", w)
        _tint(shader, "baseColor", w)
        _raise(shader, "roughness", w)


def _tint(shader, name, w):
    inp = shader.GetInput(name)
    if not inp:
        return
    cur = inp.Get()
    if cur is None:
        return
    try:
        inp.Set(Gf.Vec3f(*[float(c) * (1.0 - 0.8 * w) for c in cur]))
    except Exception:
        pass


def _raise(shader, name, w):
    inp = shader.GetInput(name)
    if not inp:
        inp = shader.CreateInput(name, Sdf.ValueTypeNames.Float)
        inp.Set(0.5)
    cur = inp.Get()
    if cur is None:
        return
    try:
        inp.Set(float(min(1.0, float(cur) + 0.4 * w)))
    except Exception:
        pass


# ---------------------------------------------------------------------------
# profiles — the layer between the disaster taxonomy and the operators
#
# The disaster axis (earthquake / tornado / explosion / flood / hurricane) is
# the stable public vocabulary and is NOT changing. These are the mechanisms it
# composes: each takes an intensity in [0, 1] and a seed, and is a pure
# composition of the operators above. Adding a disaster type is a mapping
# change, not a new operator set.
#
# `scenegen`'s own presets were deliberately not ported: all five call
# `voronoi_fracture`, `punch_hole` and `settle`, so none survives the port
# intact, and they were composed for a different vocabulary anyway.
# ---------------------------------------------------------------------------


def structural_collapse(prims, intensity: float, seed: int = 0,
                        story_bias: str = "soft_story") -> None:
    """Fails in place: racks, pancakes a storey, spalls. Serves earthquake."""
    b = bounds_of(prims)
    if b is None:
        return
    rng = np.random.default_rng(seed)
    lean(prims, b, angle_deg=2.0 + 9.0 * intensity,
         direction_deg=float(rng.uniform(0, 360)), profile=story_bias)
    b = bounds_of(prims) or b            # re-measure: lean moved the geometry
    lo = float(rng.uniform(0.0, 0.45))
    pancake(prims, b, z_lo=lo, z_hi=lo + 0.18,
            collapse=0.25 + 0.6 * intensity, spread=0.10 + 0.10 * intensity,
            seed=seed)
    b = bounds_of(prims) or b
    crumble(prims, b, amount=0.004 + 0.016 * intensity, seed=seed,
            height_bias=-0.6)            # shaking damages the base


def blast(prims, intensity: float, seed: int = 0, epicenter=None) -> None:
    """Sharp radial: thrown outward, scorched. Serves explosion."""
    b = bounds_of(prims)
    if b is None:
        return
    rng = np.random.default_rng(seed)
    if epicenter is None:
        ang = float(rng.uniform(0, 2 * math.pi))
        epicenter = (0.9 * math.cos(ang), 0.9 * math.sin(ang),
                     float(rng.uniform(0.1, 0.4)))
    shockwave(prims, b, epicenter, radius=0.45 + 0.35 * intensity,
              strength=0.04 + 0.16 * intensity, falloff="shell", seed=seed,
              floor_z=b.base_z)
    b = bounds_of(prims) or b
    crumble(prims, b, amount=0.006 + 0.020 * intensity, seed=seed + 7,
            height_bias=0.3)
    scorch(prims, strength=0.3 + 0.6 * intensity)


def wind_shear(prims, intensity: float, seed: int = 0,
               direction_deg: float = None) -> None:
    """Torn apart rather than settled. Serves tornado and hurricane."""
    b = bounds_of(prims)
    if b is None:
        return
    rng = np.random.default_rng(seed)
    if direction_deg is None:
        direction_deg = float(rng.uniform(0, 360))
    lean(prims, b, angle_deg=3.0 + 14.0 * intensity,
         direction_deg=direction_deg, profile="whip")
    b = bounds_of(prims) or b
    crumble(prims, b, amount=0.008 + 0.026 * intensity, seed=seed,
            height_bias=0.8)             # the wind works on the upper storeys


def inundation(prims, intensity: float, seed: int = 0) -> None:
    """Little structural loss; the damage is at the waterline. Serves flood."""
    b = bounds_of(prims)
    if b is None:
        return
    crumble(prims, b, amount=0.003 + 0.008 * intensity, seed=seed,
            height_bias=-0.9)            # scour and staining low down
    scorch(prims, strength=0.15 + 0.25 * intensity)   # silt line, not soot


PROFILES = {
    "structural_collapse": structural_collapse,
    "blast": blast,
    "wind_shear": wind_shear,
    "inundation": inundation,
}

# Which mechanism each disaster type reaches for. The taxonomy is fixed; this
# is the only place that needs an entry when one is added.
PROFILE_FOR_DISASTER = {
    "earthquake": "structural_collapse",
    "explosion": "blast",
    "tornado": "wind_shear",
    "hurricane": "wind_shear",
    "flood": "inundation",
    "none": None,
}


def apply_profile(prims, disaster_type: str, intensity: float,
                  seed: int = 0) -> str | None:
    """Run the profile for *disaster_type*. Returns its name, or None."""
    name = PROFILE_FOR_DISASTER.get(str(disaster_type).lower())
    if not name or intensity <= 0.0 or not prims:
        return None
    PROFILES[name](prims, float(np.clip(intensity, 0.0, 1.0)), seed=seed)
    return name


def apply_to_stage(stage, config: dict, placements: list) -> dict:
    """Deform the buildings the asset-swap route could not ruin.

    Runs after `apply_placements`, because it needs real prims: the deformation
    authors a `points` override on geometry inside a referenced layer, and
    there is nothing to override until the reference is composed.

    `disaster_stage.apply_to_buildings` marks the candidates with
    `_mesh_damage` — the local field intensity at that building — which it sets
    only where no ruin asset fitted the footprint. Buildings that *did* get a
    ruin swapped in already look ruined and are left alone; deforming them too
    would be doing the same job twice.

    Returns ``{profile_name: count}``.
    """
    from scene_generator import _stage

    dis = _stage(config, "disaster")
    dtype = str(config.get("disaster_type")
                or config.get("locale_disaster_type") or "").lower()
    if not dtype:
        # Compiled configs do not carry the type name — infer it from the
        # profile the field shape implies, falling back to the generic
        # collapse, which is the right read for "something knocked it down".
        dtype = str(dis.get("type") or "earthquake").lower()

    seed = int(config.get("seed", 0))
    tally: dict = {}
    for i, p in enumerate(placements):
        inten = p.get("_mesh_damage")
        if not inten or not p.get("prim_path"):
            continue
        prim = stage.GetPrimAtPath(p["prim_path"])
        if not prim or not prim.IsValid():
            continue
        # Open any instances first: a placement whose asset is internally
        # instanced (the AEC packs are) cannot take a per-prim points opinion
        # until it is. Scoped to the buildings actually being deformed, since
        # this is what instancing was saving.
        deinstance(prim)
        prims = mesh_prims(prim)
        if not prims:
            continue
        name = apply_profile(prims, dtype, float(inten), seed=seed + i * 31)
        if name:
            tally[name] = tally.get(name, 0) + 1

    if tally:
        print("[mesh_damage] deformed "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    return tally
