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

THE THREE KINDS OF DAMAGE, AND WHY ALL THREE ARE NEEDED
-------------------------------------------------------
* **Move vertices** — `lean`, `pancake`, `crumble`, `shockwave`. Cheap, and
  they never change the face count, which is also their limit: a building that
  has only been deformed still has every wall it started with. On its own this
  reads as *squashed*, not *broken*.
* **Delete faces** — `punch_hole`, over `delete_faces`. This is what opens a
  roof, blows out a facade, and makes a torn edge. Ported from the prototype's
  `punch_hole`; the bookkeeping it needs on USD is documented at
  `delete_faces`.
* **Cut the mesh apart** — `fracture` / `fracture_to_stage`, Voronoi cells that
  become their own prims and are dropped by PhysX. Ported from
  `voronoi_fracture` + `settle`.

`boolean_cut` did not come across (it needs a watertight solid, which building
assets are not, and the prototype only offers it as an optimisation over
`punch_hole`). Neither did `refine` — see `punch_hole` for what replaces it.

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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

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
    descent `generate_scene.prune_prims` and `apply_surface_overrides` do;
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
# topology — removing material
#
# Deforming a building cannot open it. A roof comes off, a facade blows out and
# a wall goes missing by DELETING FACES, and on USD that is bookkeeping rather
# than geometry: `faceVertexCounts` and `faceVertexIndices` renumber the faces,
# and everything indexed *by* face has to follow or the asset comes apart in
# ways that look worse than the damage —
#
#   * **faceVarying primvars**, `st` above all: one value per face-vertex, so a
#     stale array slides every UV onto the wrong corner and the brick texture
#     crawls across the facade. Indexed primvars carry the shift in their
#     `indices`, flat ones in their values, and both forms occur in the packs.
#   * **`GeomSubset` children**: face-index sets, and the mechanism that binds
#     a material to *part* of a mesh. The AEC packs lean on them heavily, so a
#     stale subset paints the windows with the roof material.
#   * **`normals`**, whenever they are authored faceVarying or uniform.
#
# `delete_faces` is the single place that knows all of this; nothing else may
# touch the face arrays.
# ---------------------------------------------------------------------------


def _face_arrays(prim):
    """``(faceVertexCounts, faceVertexIndices)`` as int arrays, or (None, None)."""
    mesh = UsdGeom.Mesh(prim)
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not counts or not idx:
        return None, None
    counts = np.asarray(counts, dtype=np.int64)
    if (counts <= 0).any() or int(counts.sum()) != len(idx):
        return None, None            # malformed; leave it alone
    return counts, np.asarray(idx, dtype=np.int64)


def _fv_starts(counts: np.ndarray) -> np.ndarray:
    """Offset of each face's first entry in the flattened face-vertex array."""
    return np.concatenate([[0], np.cumsum(counts)[:-1]])


def _filter_by_element(attr, primvar, interpolation, keep, fv_keep) -> None:
    """Drop the elements of a face- or face-vertex-indexed array.

    Vertex-, varying- and constant-interpolated data is indexed by *point*,
    which face deletion does not renumber, so it is left alone — the orphaned
    points stay in `points` and cost nothing but a little memory.
    """
    if interpolation == UsdGeom.Tokens.faceVarying:
        mask = fv_keep
    elif interpolation == UsdGeom.Tokens.uniform:
        mask = keep
    else:
        return

    # An INDEXED primvar keeps its value palette and renumbers its `indices`;
    # a flat one carries the values themselves. Filtering the wrong one of the
    # two silently corrupts the other.
    if primvar is not None and primvar.IsIndexed():
        ind = primvar.GetIndices()
        if ind is None or len(ind) != len(mask):
            return
        primvar.SetIndices(
            Vt.IntArray([int(v) for v in np.asarray(ind)[mask]]))
        return

    vals = attr.Get() if attr else None
    if vals is None or len(vals) != len(mask):
        return
    try:
        attr.Set(type(vals)([v for v, k in zip(vals, mask) if k]))
    except Exception:
        pass


def _renumber_subsets(prim, keep: np.ndarray) -> None:
    """Rewrite every face-`GeomSubset` under *prim* into the new numbering."""
    new_of = np.cumsum(keep) - 1
    for child in prim.GetChildren():
        if not child.IsA(UsdGeom.Subset):
            continue
        sub = UsdGeom.Subset(child)
        et = sub.GetElementTypeAttr().Get()
        if et and et != UsdGeom.Tokens.face:
            continue
        ind = sub.GetIndicesAttr().Get()
        if ind is None:
            continue
        a = np.asarray(ind, dtype=np.int64)
        a = a[(a >= 0) & (a < len(keep))]
        a = a[keep[a]]
        sub.GetIndicesAttr().Set(Vt.IntArray([int(v) for v in new_of[a]]))


def delete_faces(prim, keep, deactivate: bool = True) -> int:
    """Keep only the faces *keep* marks. Returns how many were removed.

    Authors overrides on the topology attributes, exactly as `set_points` does
    on `points` — so the same rule applies: the prim must not be instanced.

    When nothing survives the prim is DEACTIVATED rather than left holding
    empty topology, the same retirement `fracture_to_stage` gives a source it
    has consumed. Empty face arrays are legal USD but some renderers dislike
    them, and an invisible prim still costs traversal.

    *deactivate* turns that off, and the caller that needs it is
    `fracture_to_stage`: deactivating a prim takes its whole subtree with it,
    and the fragments are authored as CHILDREN of the building. On a
    multi-mesh asset that is harmless — the fragments hang off the building
    Xform and the consumed prim is one of its mesh siblings — but half the
    library is a SINGLE mesh referenced straight at the placement path, and
    there the consumed prim *is* the fragments' parent. Retiring it deleted
    the rubble it had just produced, so a whole-building fracture on those
    assets made the building vanish entirely: no ruin, no debris field,
    nothing but the ground. Callers that author under the prim pass False and
    get the empty-topology form instead.
    """
    counts, idx = _face_arrays(prim)
    if counts is None:
        return 0
    keep = np.asarray(keep, dtype=bool)
    if len(keep) != len(counts):
        return 0
    n_gone = int((~keep).sum())
    if n_gone == 0:
        return 0

    mesh = UsdGeom.Mesh(prim)
    if n_gone == len(counts):
        if deactivate:
            try:
                prim.SetActive(False)
                return n_gone
            except Exception:
                pass
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([]))
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray([]))
        return n_gone

    fv_keep = np.repeat(keep, counts)
    mesh.GetFaceVertexCountsAttr().Set(
        Vt.IntArray([int(c) for c in counts[keep]]))
    mesh.GetFaceVertexIndicesAttr().Set(
        Vt.IntArray([int(i) for i in idx[fv_keep]]))

    _filter_by_element(mesh.GetNormalsAttr(), None,
                       mesh.GetNormalsInterpolation(), keep, fv_keep)
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        _filter_by_element(pv.GetAttr(), pv, pv.GetInterpolation(),
                           keep, fv_keep)
    _renumber_subsets(prim, keep)
    return n_gone


def _face_geometry(prim):
    """Per-face ``(centroid, span)`` in world space, or ``(None, None)``.

    *span* is the face's largest bounding-box edge — `punch_hole` uses it to
    refuse to swallow a face far bigger than the hole itself.
    """
    counts, idx = _face_arrays(prim)
    if counts is None:
        return None, None
    pts = get_points(prim)
    if not len(pts) or int(idx.max()) >= len(pts):
        return None, None
    fv = pts[idx]
    starts = _fv_starts(counts)
    cen = np.add.reduceat(fv, starts, axis=0) / counts[:, None]
    hi = np.maximum.reduceat(fv, starts, axis=0)
    lo = np.minimum.reduceat(fv, starts, axis=0)
    return cen, (hi - lo).max(axis=1)


def surface_points(prims, n: int, bounds: Bounds, seed: int = 0,
                   z_range=None, facing=None) -> list:
    """*n* points that actually lie ON the building, in normalized space.

    A point drawn from the bounding box lands in the hollow interior most of
    the time, where there is no geometry to remove and the hole silently does
    nothing — the same reason `_seed_points` samples the surface. Face
    centroids are guaranteed to be on a wall or a roof.

    *z_range* restricts sampling to a normalized height band, which is how the
    wind profile attacks only the top of a building. *facing* is an ``(x, y)``
    bearing that biases sampling to the faces pointing that way — the windward
    side — expressed loosely enough that a little damage still lands elsewhere.
    """
    if n <= 0:
        return []
    pool = [c for c in (_face_geometry(p)[0] for p in prims)
            if c is not None and len(c)]
    if not pool:
        return []
    pts = np.concatenate(pool, axis=0)

    half = bounds.dims * 0.5
    half = np.where(np.abs(half) < 1e-9, 1e-9, half)
    c = bounds.center
    norm = np.empty_like(pts)
    norm[:, 0] = (pts[:, 0] - c[0]) / half[0]
    norm[:, 1] = (pts[:, 1] - c[1]) / half[1]
    norm[:, 2] = (pts[:, 2] - bounds.lo[2]) / max(bounds.dims[2], 1e-9)

    def _narrow(mask):
        # Never narrow to nothing: an empty pool means no damage at all, which
        # is a worse answer than damage in a slightly wrong place.
        return norm[mask] if mask.any() else norm

    if z_range is not None:
        norm = _narrow((norm[:, 2] >= z_range[0]) & (norm[:, 2] <= z_range[1]))
    if facing is not None:
        d = norm[:, 0] * float(facing[0]) + norm[:, 1] * float(facing[1])
        norm = _narrow(d > -0.2)

    rng = np.random.default_rng(seed)
    pick = rng.choice(len(norm), size=min(int(n), len(norm)), replace=False)
    return [tuple(norm[i]) for i in pick]


def punch_hole(prims, bounds: Bounds, epicenter, radius: float = 0.18,
               seed: int = 0, ragged: float = 0.45,
               max_face_span: float = 3.0) -> int:
    """Delete the faces near a point — the blast hole, the missing wall.

    Works on any topology (unlike a boolean difference, which needs a closed
    solid): it removes faces whose centroid falls inside a noise-perturbed
    sphere. *ragged* modulates the radius with coherent noise so the opening
    has a torn edge rather than a suspiciously circular one.

    WHAT REPLACES `refine`
    ----------------------
    The prototype subdivides large faces before testing them, because on a
    coarse asset one face can be bigger than the whole hole and the centroid
    test then either misses or eats an entire wall. Subdividing here would mean
    inventing faces — and every faceVarying primvar and `GeomSubset` on them —
    which is a great deal of machinery for a mesh that is about to be rubble.
    Instead *max_face_span* refuses any face whose bounding box is more than
    that multiple of the hole across. A small hole on a coarse wall then does
    nothing, which is the honest outcome; a large tear still takes the wall.
    """
    c = bounds.to_world(epicenter)
    R = bounds.frac(radius)
    if R <= 0.0:
        return 0
    removed = 0
    for prim in prims:
        cen, span = _face_geometry(prim)
        if cen is None:
            continue
        thresh = np.full(len(cen), R)
        if ragged:
            thresh = R * (1.0 + value_noise(cen / bounds.radius, 4.0, seed)
                          * ragged)
        kill = (np.linalg.norm(cen - c, axis=1) < thresh) & \
               (span <= max_face_span * R)
        if kill.any():
            removed += delete_faces(prim, ~kill)
    return removed


# ---------------------------------------------------------------------------
# topology — adding thickness
#
# THE PROBLEM THIS SOLVES
# -----------------------
# Much of the library is a HOLLOW SHELL: a single surface with no thickness at
# all, because nothing ever needed the inside of a wall. Measured, an objaverse
# bungalow encloses 51 m³ against the 820 m³ a 0.25 m slab of its own surface
# area would have — i.e. essentially none. That assumption survives right up
# until the wall is broken, and then it is the single thing that makes
# procedural damage read as fake:
#
#   * `punch_hole` deletes faces, and the opening it leaves has a knife edge.
#     A real breach shows the depth of the wall — that band of exposed material
#     around the hole is most of what tells you the wall was thick. Worse, a
#     shell is one-sided, so through the hole you see the *backfaces* of the far
#     wall, which most renderers draw as black or drop entirely. The building
#     reads as a paper cutout, not as masonry.
#   * `fracture` clips the shell into Voronoi cells, and a cell of a zero-
#     thickness surface is a zero-thickness surface. The "rubble" is a drift of
#     curved sheets — visually, confetti with a brick texture on it. No amount
#     of tuning the cell count fixes that, because there is no volume to cut.
#
# So: give the shell volume BEFORE breaking it. `solidify` extrudes every
# surface inward along its own normal and caps the open edges, turning the
# shell into a slab of *wall_m* metres. Fracture then cuts a solid and returns
# chunks; a hole punched through it shows a reveal on every edge.
#
# NOT EVERY ASSET NEEDS IT, AND THE OPERATOR CHECKS
# -------------------------------------------------
# The library holds both kinds. The nine `urban` / `urban_intact` buildings are
# closed masses enclosing 7-34x the volume a slab of their surface would, so
# `solidify` declines all nine and the tally reports nothing thickened — which
# is the operator working. The same enclosed volume that diagnoses a shell is
# what refuses a solid, and it costs nothing extra: it falls out of the normal
# accumulation by the divergence theorem.
#
# WHY INWARD, AND HOW THAT DIRECTION IS FOUND
# -------------------------------------------
# Outward would inflate the building past its own footprint — the layout stage
# has already packed these to setbacks and a building that grows by 0.25 m on
# every face starts intersecting its neighbours. Inward eats into interior
# space that is empty anyway.
#
# Which way is "in" cannot be read off the winding, because the packs are not
# consistently wound and USD's `orientation` metadata is unreliable on
# converted assets. Nor is one sign per mesh enough: measured on a bungalow,
# the roof and the ground slab each carry faces of BOTH windings, so whichever
# global sign is picked, part of the mesh extrudes the wrong way — visible
# without measuring anything, as the building's bounding box growing by most of
# the wall thickness. The direction is therefore settled PER POINT, by flipping
# each normal to face away from the building's centre. That ignores winding
# entirely and is right for the shapes this deals with: a roof plane thickens
# downward, a ground slab upward, a facade inward.
#
# WHY THIS IS NOT FREE, AND WHAT BOUNDS IT
# ----------------------------------------
# It doubles the point count of every mesh it touches and adds a rim quad per
# boundary edge. The generated scene has OOM-killed at 89M points, so this is
# on a budget exactly as fracture is (`apply_to_stage` spends both on the
# worst-hit buildings), and `max_points` skips any single mesh too big to be
# worth it.
# ---------------------------------------------------------------------------


def _weld(pts: np.ndarray, tol: float) -> np.ndarray:
    """Map each point to a representative id shared by points at one position.

    Assets arrive with their vertices split along every UV seam and material
    boundary, so the "same" corner of a wall is several points. Normals
    averaged without welding are then wrong at exactly the seams — the extrude
    tears open along them — and boundary-edge detection breaks completely,
    because a shared edge looks like two one-sided edges and every seam grows a
    rim. Rounding to *tol* is enough: these are seams, not near-misses.
    """
    if not len(pts):
        return np.zeros(0, dtype=np.int64)
    q = np.round(pts / max(tol, 1e-9)).astype(np.int64)
    _, inv = np.unique(q, axis=0, return_inverse=True)
    return inv.reshape(-1)


def _surface_normals(pts: np.ndarray, counts: np.ndarray, idx: np.ndarray,
                     tol: float, ref_centre=None):
    """``(unit_normals (P,3), median_edge, enclosed_volume, area)``.

    Returns ``(None, 0.0, 0.0, 0.0)`` when there is nothing triangulable.

    Normals are area-weighted (the cross product is not normalised before
    accumulation, so a big face counts for more than a sliver), averaged across
    welded position, and oriented OUTWARD — away from *ref_centre*, which
    should be the building's centre rather than the mesh's. Measuring about
    each mesh's own centroid works only for a mesh that wraps a volume by
    itself, and an asset is not built that way: it is split into wall panels,
    roof planes and floor slabs.

    *enclosed_volume* falls straight out of the same sum. By the divergence
    theorem a closed surface encloses ``V = Σ (c - c0)·n dA / 3``, which is the
    accumulator divided by six — so the caller gets, for free, the one number
    that says whether this mesh is a hollow shell or already a solid.
    """
    tri, _src = _triangulate(counts)
    if not len(tri):
        return None, 0.0, 0.0, 0.0
    p = idx[tri]                                    # (T, 3) point indices
    a, b, c = pts[p[:, 0]], pts[p[:, 1]], pts[p[:, 2]]
    fn = np.cross(b - a, c - a)                     # |fn| = 2 * area
    cen = (a + b + c) / 3.0
    area = float(np.linalg.norm(fn, axis=1).sum()) * 0.5

    w = _weld(pts, tol)
    acc = np.zeros((int(w.max()) + 1, 3))
    for k in range(3):
        np.add.at(acc, w[p[:, k]], fn)

    centre = (pts.mean(axis=0) if ref_centre is None
              else np.asarray(ref_centre, dtype=np.float64))
    flux = float((fn * (cen - centre)).sum())
    if flux < 0.0:
        acc = -acc                                  # normals pointed inward

    n = acc[w]
    mag = np.linalg.norm(n, axis=1)
    # A point where opposing faces cancel exactly leaves a zero normal; leaving
    # it at zero simply means that point does not move, which is a degenerate
    # rim quad and not a corrupt mesh.
    n = np.where(mag[:, None] > 1e-12, n / np.maximum(mag, 1e-12)[:, None], 0.0)

    # PER-POINT orientation, because a per-mesh one is not enough. These assets
    # are not consistently wound — measured on a bungalow, the roof and the
    # ground slab each carry faces of both windings, so whichever global sign
    # is chosen, part of the mesh extrudes the wrong way and the building's
    # bounding box grows by most of the wall thickness. Flipping each point
    # individually to face away from the building centre does not care about
    # winding at all, and is right for the shapes this module deals with: a
    # roof plane thickens downward, a ground slab upward, a facade inward.
    radial = pts - centre
    rmag = np.linalg.norm(radial, axis=1)
    rhat = radial / np.maximum(rmag, 1e-12)[:, None]
    d = np.einsum("ij,ij->i", n, rhat)
    # Below the threshold the normal is essentially tangential, the radial test
    # says nothing, and the accumulated orientation above is the better guess.
    n = np.where((d < -0.15)[:, None], -n, n)

    edge = np.linalg.norm(b - a, axis=1)
    med = float(np.median(edge[edge > 1e-9])) if (edge > 1e-9).any() else 0.0
    return n, med, abs(flux) / 6.0, area


def _boundary_edges(counts: np.ndarray, idx: np.ndarray, w: np.ndarray):
    """``(slot_a, slot_b, face)`` for every edge used by exactly one face.

    Slots index the FLATTENED face-vertex array, not the point array — the rim
    has to inherit faceVarying data (`st` above all) from the corner it grew
    out of, and only a slot identifies that. Edges are keyed on WELDED point
    ids so that a wall meeting a floor counts as shared, and the pair is
    canonicalised so the two faces that share an edge agree on its key.
    """
    starts = _fv_starts(counts)
    # Faces are contiguous in the flattened array, so the slots ARE arange.
    sa = np.arange(int(counts.sum()))
    # Next slot around each face, wrapping at the face's last corner.
    nxt = sa + 1
    nxt[starts + counts - 1] = starts
    face = np.repeat(np.arange(len(counts)), counts)

    ea, eb = w[idx[sa]], w[idx[nxt]]
    key = np.stack([np.minimum(ea, eb), np.maximum(ea, eb)], axis=1)
    _, first, n_use = np.unique(key, axis=0, return_index=True,
                                return_counts=True)
    lone = first[n_use == 1]
    # A degenerate edge (both ends welded to one point) is not a boundary, it
    # is a collapsed sliver, and rimming it emits zero-area quads.
    lone = lone[ea[lone] != eb[lone]]
    return sa[lone], nxt[lone], face[lone]


def _gather_by_element(attr, primvar, interpolation, face_src, fv_src,
                       n_points: int) -> None:
    """Re-index a primvar onto the solidified topology.

    The mirror of `_filter_by_element`, and it has the same three cases and the
    same indexed/flat split. The difference is that solidify GROWS the arrays
    rather than shrinking them, so every new element names the old one it
    inherits from: `face_src` for uniform data, `fv_src` for faceVarying.

    Point-indexed data (`vertex` / `varying`) is not gathered but DOUBLED — the
    inner shell has one new point per original point, in the same order, so the
    array that indexes them is just itself twice over. Getting this wrong is
    silent: the attribute keeps its old length, USD decides the interpolation
    no longer matches, and the primvar is dropped at render time.
    """
    if interpolation == UsdGeom.Tokens.faceVarying:
        take = fv_src
    elif interpolation == UsdGeom.Tokens.uniform:
        take = face_src
    elif interpolation in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying):
        take = None
    else:
        return                                       # constant — nothing to do

    if primvar is not None and primvar.IsIndexed():
        ind = primvar.GetIndices()
        if ind is None:
            return
        a = np.asarray(ind, dtype=np.int64)
        if take is None:
            if len(a) != n_points // 2:
                return
            a = np.concatenate([a, a])
        else:
            if len(a) <= int(take.max()):
                return
            a = a[take]
        primvar.SetIndices(Vt.IntArray([int(v) for v in a]))
        return

    vals = attr.Get() if attr else None
    if vals is None:
        return
    try:
        if take is None:
            if len(vals) != n_points // 2:
                return
            attr.Set(type(vals)(list(vals) + list(vals)))
        else:
            if len(vals) <= int(take.max()):
                return
            attr.Set(type(vals)([vals[int(i)] for i in take]))
    except Exception:
        pass


def _grow_subsets(prim, n_faces: int, rim_face: np.ndarray) -> None:
    """Extend every face-`GeomSubset` onto the inner shell and the rim.

    A subset is how a material is bound to part of a mesh, so if it does not
    follow, the inside of every wall and every torn edge renders with whichever
    material happened to be bound on the prim itself — which on the AEC packs
    is usually the roof. Face `f` gains its inner twin at `f + n_faces` and
    whichever rim quads grew off it.
    """
    for child in prim.GetChildren():
        if not child.IsA(UsdGeom.Subset):
            continue
        sub = UsdGeom.Subset(child)
        et = sub.GetElementTypeAttr().Get()
        if et and et != UsdGeom.Tokens.face:
            continue
        ind = sub.GetIndicesAttr().Get()
        if ind is None:
            continue
        a = np.asarray(ind, dtype=np.int64)
        a = a[(a >= 0) & (a < n_faces)]
        mine = np.isin(rim_face, a)
        out = np.concatenate([a, a + n_faces,
                              2 * n_faces + np.nonzero(mine)[0]])
        sub.GetIndicesAttr().Set(Vt.IntArray([int(v) for v in out]))


def solidify(prim, thickness: float, ref_centre=None, ref_box=None,
             weld_tol: float = 1e-4, max_span_frac: float = 0.25,
             solid_ratio: float = 0.5, max_points: int = 200_000) -> int:
    """Extrude *prim*'s shell inward into a slab. Returns faces added, or 0.

    *ref_centre* / *ref_box* describe the whole building in world space; pass
    them whenever the prim is one mesh of many, which on a real asset it always
    is. See `_surface_normals` and the direction note below for why a per-mesh
    guess is not good enough.

    *thickness* is in WORLD METRES, deliberately — a wall is about 0.25 m thick
    whether it is on a bungalow or a tower, so this is the one quantity in the
    module that is NOT a fraction of the building radius.

    It is still capped, because assets carry window glass, railings, gutters
    and signage as their own thin meshes and extruding a 5 cm mullion by 25 cm
    turns a facade into a wall of blocks. The cap is *max_span_frac* of the
    mesh's SECOND-SMALLEST bounding-box dimension. Both halves of that are
    deliberate: the smallest dimension is useless because a flat wall panel is
    zero-thickness by definition and would cap to nothing, while the second
    smallest is the panel's short side — 3 m for a wall, 5 cm for a mullion,
    which is exactly the distinction wanted. Median edge length was the obvious
    alternative and is wrong: it measures how finely a surface is tessellated,
    not how thin the thing is, so it throttled dense wall meshes to 9 cm while
    leaving coarse ones at full thickness.

    ALREADY-SOLID MESHES ARE SKIPPED
    --------------------------------
    Not every asset in the library is a shell, and thickening one that is not
    buys nothing and costs double the points. The library really does span both
    kinds — measured: an objaverse bungalow encloses 51 m³ against the 820 m³
    a 0.25 m slab of its own surface area would have, i.e. a true shell, while
    a Nucleus `BG_Building_*` is a closed mass enclosing thirty times that.
    So the enclosed volume `_surface_normals` returns decides: at or above
    *solid_ratio* of the slab volume there is already material there.

    WHICH WAY IS IN
    ---------------
    The reverse of the outward normal, oriented per point — see
    `_surface_normals`. *ref_box* is then a backstop rather than the mechanism:
    if a mesh would still inflate the building, its offset is scaled back until
    it does not, because a wall that thickens OUTWARD is the one artifact this
    operator must never introduce. The layout stage has already packed these to
    setbacks, and a building that grows on every face starts intersecting its
    neighbours.

    Authors overrides on the same attributes `delete_faces` does, and carries
    the same warning: the prim must not be instanced.

    NORMALS ARE DROPPED, NOT REBUILT
    --------------------------------
    Authored `normals` cannot survive this. The inner shell needs them negated,
    the rim needs values that exist nowhere in the source, and a `vertex`-
    interpolated array cannot express a crease between a wall and its own rim
    at all. Blocking the attribute makes the renderer compute them from the
    topology, which is right everywhere and costs a little smoothing detail on
    assets that shipped custom normals. A stale array would instead light the
    inside of the wall as though it were the outside.
    """
    counts, idx = _face_arrays(prim)
    if counts is None:
        return 0
    world = get_points(prim)
    n_pts = len(world)
    if n_pts < 3 or n_pts > max_points or int(idx.max()) >= n_pts:
        return 0

    outward, med_edge, volume, area = _surface_normals(
        world, counts, idx, weld_tol, ref_centre)
    if outward is None or med_edge <= 0.0:
        return 0
    span = float(np.sort(world.max(axis=0) - world.min(axis=0))[1])
    t = min(float(thickness), max_span_frac * span)
    if t <= 1e-6:
        return 0
    if volume >= solid_ratio * area * t:
        return 0                                  # already has material in it

    inward = -outward
    if ref_box is not None:
        # Clamp any point that would push past the building's own box back
        # onto it. Per-axis and per-point, so a mesh is never rejected whole
        # for one stray corner.
        lo, hi = np.asarray(ref_box[0]), np.asarray(ref_box[1])
        moved = world + inward * t
        over = np.maximum(moved - hi, 0.0) + np.minimum(moved - lo, 0.0)
        inward = inward - over / t

    n_faces = len(counts)
    n_fv = int(counts.sum())
    starts = _fv_starts(counts)

    # --- the three sets of faces -------------------------------------------
    # Outer: untouched. Inner: same faces, reversed winding, on the offset
    # points. Reversing is not optional — an offset copy with the original
    # winding faces the same way the outer shell does, so the slab is
    # back-to-front on the inside and renders inside-out through every breach.
    rev = np.concatenate([np.arange(s + c - 1, s - 1, -1)
                          for s, c in zip(starts, counts)])
    inner_idx = idx[rev] + n_pts

    sa, sb, rim_face = _boundary_edges(counts, idx, _weld(world, weld_tol))
    # Quad (a, a', b', b) — the winding that makes its normal point out of the
    # surface rather than back into the slab. See the module note above.
    rim_idx = np.stack([idx[sa], idx[sa] + n_pts,
                        idx[sb] + n_pts, idx[sb]], axis=1).reshape(-1)
    rim_fv = np.stack([sa, sa, sb, sb], axis=1).reshape(-1)

    mesh = UsdGeom.Mesh(prim)
    new_world = np.concatenate([world, world + inward * t])
    set_points(prim, new_world)
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(
        [int(c) for c in counts] + [int(c) for c in counts]
        + [4] * len(rim_face)))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(
        [int(i) for i in np.concatenate([idx, inner_idx, rim_idx])]))

    face_src = np.concatenate([np.arange(n_faces), np.arange(n_faces),
                               rim_face])
    fv_src = np.concatenate([np.arange(n_fv), rev, rim_fv])

    # See the docstring: computed normals beat a stale or invented array.
    try:
        mesh.GetNormalsAttr().Block()
    except Exception:
        pass
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        if pv.GetBaseName() == "normals":
            continue
        _gather_by_element(pv.GetAttr(), pv, pv.GetInterpolation(),
                           face_src, fv_src, len(new_world))
    _grow_subsets(prim, n_faces, rim_face)
    return n_faces + len(rim_face)


def solidify_prims(prims, thickness: float, bounds: Bounds = None,
                   **kw) -> dict:
    """`solidify` every prim against one shared centre.

    Returns ``{"meshes": n, "faces": n}``. *bounds* is measured off the prims
    when not given; pass the caller's if it already has one, since every
    operator here re-measures and it is the expensive part on a big asset.
    """
    b = bounds if bounds is not None else bounds_of(prims)
    centre = None if b is None else b.center
    box = None if b is None else (b.lo, b.hi)
    done = faces = 0
    for prim in prims:
        added = solidify(prim, thickness, ref_centre=centre, ref_box=box, **kw)
        if added:
            done += 1
            faces += added
    return {"meshes": done, "faces": faces}


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


# Albedo inputs worth darkening, by shading model. UsdPreviewSurface is what
# the Objaverse→USD conversion writes; the `diffuse_*` names are OmniPBR/MDL,
# which is what the Nucleus and AEC packs bind inside Isaac. `diffuse_tint` is
# already a multiplier over the albedo texture, so it is the ideal place to put
# soot when it exists.
_ALBEDO_INPUTS = ("diffuseColor", "baseColor",
                  "diffuse_tint", "diffuse_color_constant", "albedo_add")
_ROUGHNESS_INPUTS = ("roughness", "reflection_roughness_constant")


def scorch(prims, strength: float = 0.85, bounds: Bounds = None,
           epicenter=None, radius: float = 0.6) -> None:
    """Darken the bound materials — soot.

    *bounds* and *epicenter* make it local: a material's weight comes from the
    closest prim bound to it, so a blast sooties the wall it happened against
    and not the far side of the building. Omit them and the whole asset chars,
    which is what a fire does.

    WHY SETTING THE INPUT IS NOT ENOUGH
    -----------------------------------
    This used to be `inp.Set(darker_colour)` on `diffuseColor` and it did
    **nothing at all** on almost every real asset. Building assets drive albedo
    from an image texture, which in USD means `diffuseColor` is *connected* to
    a `UsdUVTexture`, and a connection beats a value — the authored constant is
    simply never consulted. The prototype hit the identical wall in Blender
    ("overwriting `default_value` silently does nothing: the constant is
    ignored while the link supplies the colour") and solved it by splicing a
    multiply node into the graph.

    The USD equivalent needs no new node: `UsdUVTexture` has an `inputs:scale`
    that multiplies whatever it sampled, so soot goes there. Only the RGB
    components are scaled — the fourth is alpha, and a texture whose `.a` drives
    opacity would otherwise turn the building transparent as it charred.

    ROUGHNESS IS ONLY TOUCHED WHEN IT IS NOT CONNECTED, which is the
    prototype's rule and not timidity: these assets pack roughness and metallic
    into two channels of ONE texture, so scaling that texture to roughen the
    surface would silently rewrite metallic too.

    Authored once per *material*: the material lives in the referenced layer
    and is shared between prims, so a second visit would compound the tint.
    """
    w_max = max(0.0, min(1.0, strength))
    if w_max <= 0.0:
        return

    # Per material, the strongest weight any prim bound to it earns.
    weights: dict = {}
    mats: dict = {}
    centre = R = None
    if bounds is not None and epicenter is not None:
        centre = bounds.to_world(epicenter)
        R = max(bounds.frac(radius), 1e-9)

    for prim in prims:
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        key = str(mat.GetPrim().GetPath())
        w = w_max
        if centre is not None:
            v = get_points(prim)
            if not len(v):
                continue
            near = float(np.linalg.norm(v - centre, axis=1).min())
            w = w_max * float(_falloff(np.array([near / R]))[0])
        if w <= 0.01:
            continue
        mats[key] = mat
        weights[key] = max(weights.get(key, 0.0), w)

    for key, mat in mats.items():
        for shader in _surface_shaders(mat):
            _soot(shader, weights[key])


def _surface_shaders(material) -> list:
    """The material's surface shader in every render context it declares.

    `ComputeSurfaceSource()` with no argument resolves the universal context,
    which finds the UsdPreviewSurface. Isaac's assets additionally (sometimes
    only) carry an `mdl` surface, and that is the one the RTX renderer actually
    shades with — so a scorch that checked only the universal context would
    look right in usdview and change nothing in the sim.
    """
    out, seen = [], set()
    for ctx in ("", "mdl"):
        shader = None
        try:
            shader = material.ComputeSurfaceSource(ctx)[0]
        except TypeError:
            # Some USD builds take a context VECTOR rather than a token. Only
            # the signature mismatch is caught: a blanket `except Exception`
            # here is what hid this call being wrong in the first place, and a
            # scorch that resolves no shader fails completely silently.
            try:
                shader = material.ComputeSurfaceSource(
                    [ctx] if ctx else [])[0]
            except Exception:
                shader = None
        if shader and str(shader.GetPath()) not in seen:
            seen.add(str(shader.GetPath()))
            out.append(shader)
    return out


def _soot(shader, w: float) -> None:
    """Darken one shader's albedo by *w*, and roughen it where it is safe to."""
    keep = 1.0 - 0.8 * float(w)
    for name in _ALBEDO_INPUTS:
        inp = shader.GetInput(name)
        if inp:
            _scale_input(inp, keep)
    for name in _ROUGHNESS_INPUTS:
        inp = shader.GetInput(name)
        if inp and not inp.HasConnectedSource():
            cur = inp.Get()
            if cur is not None:
                try:
                    inp.Set(float(min(1.0, float(cur) + 0.4 * w)))
                except Exception:
                    pass


def _scale_input(inp, keep: float) -> None:
    """Multiply a shader input by *keep*, through a texture if it goes to one."""
    if inp.HasConnectedSource():
        source = inp.GetConnectedSource()
        if not source:
            return
        tex = UsdShade.Shader(source[0].GetPrim())
        if not tex or tex.GetIdAttr().Get() != "UsdUVTexture":
            return          # some other node; nothing safe to scale here
        scale = tex.GetInput("scale")
        if not scale:
            scale = tex.CreateInput("scale", Sdf.ValueTypeNames.Float4)
            scale.Set(Gf.Vec4f(1.0, 1.0, 1.0, 1.0))
        cur = scale.Get() or Gf.Vec4f(1.0, 1.0, 1.0, 1.0)
        # RGB only. The fourth component is alpha, and a texture whose `.a`
        # drives opacity would fade the building out as it charred.
        try:
            scale.Set(Gf.Vec4f(float(cur[0]) * keep, float(cur[1]) * keep,
                               float(cur[2]) * keep, float(cur[3])))
        except Exception:
            pass
        return

    cur = inp.Get()
    if cur is None:
        return
    try:
        inp.Set(Gf.Vec3f(*[float(c) * keep for c in cur]))
    except Exception:
        try:
            inp.Set(float(cur) * keep)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# profiles — the layer between the disaster taxonomy and the operators
#
# The disaster axis (earthquake / tornado / hurricane / fire / explosion /
# flood) is the stable public vocabulary. These are the mechanisms it
# composes: each takes an intensity in [0, 1] and a seed, and is a pure
# composition of the operators above. Adding a disaster type is a mapping
# change, not a new operator set.
#
# `scenegen`'s own presets could not be ported when this module was written:
# all five call `voronoi_fracture`, `punch_hole` and `settle`, none of which
# existed here. All three do now, so `burnout` IS the prototype's `fire`, and
# the rest have been folded into the profiles below rather than transcribed —
# the prototype's `shelling` is `structural_collapse`'s `spall` mode with more
# openings, and its `windstorm` is `wind_shear`.
# ---------------------------------------------------------------------------


# How a building fails in an earthquake — the real modes, not one recipe.
#
# A single lean+pancake+crumble applied to every building makes a street of
# identical casualties. Real post-quake blocks are mixed: most buildings are
# standing with cracked facades, some have lost a storey, a few are down
# entirely, and which one you get is mostly a function of intensity.
#
# Weights are (base, per-intensity slope) and are renormalised, so at low
# severity the street is mostly racked and spalled and at high severity the
# collapse modes dominate. Nothing here needs a new operator — the modes differ
# in WHICH storey fails, how many, and how hard the surface is worked.
_QUAKE_MODES = {
    # tilts and stands. The common outcome, and the one that was missing:
    # previously every building lost a storey.
    "racking":    (0.45, -0.40),
    # facade shatters, structure holds. Heavy surface work, no collapse.
    "spall":      (0.30, -0.20),
    # ground floor goes, upper storeys ride down intact. The classic.
    "soft_story": (0.10, +0.25),
    # a middle floor fails instead — reads differently from the street.
    "mid_story":  (0.08, +0.18),
    # several bands crush; the building squats.
    "total":      (-0.05, +0.30),
    # one side drops and the rest leans into it.
    "partial":    (0.07, +0.17),
}


def _pick_quake_mode(intensity: float, rng) -> str:
    w = {k: max(0.0, base + slope * intensity)
         for k, (base, slope) in _QUAKE_MODES.items()}
    total = sum(w.values()) or 1.0
    r = float(rng.random()) * total
    for name, weight in w.items():
        r -= weight
        if r <= 0.0:
            return name
    return "racking"


def _holes(prims, bounds: Bounds, n: int, radius: float, seed: int,
           z_range=None, facing=None, ragged: float = 0.5,
           spread: float = 0.5) -> int:
    """Punch *n* holes at points sampled off the surface. Returns faces removed.

    The pairing of `surface_points` with `punch_hole` is the whole idiom — a
    hole centred anywhere else is centred in the building's hollow interior and
    removes nothing. *spread* varies the radius around *radius* so the openings
    do not all come out the same size.
    """
    if n <= 0 or radius <= 0.0:
        return 0
    rng = np.random.default_rng(seed + 5)
    removed = 0
    for k, p in enumerate(surface_points(prims, int(n), bounds, seed=seed,
                                         z_range=z_range, facing=facing)):
        r = radius * (1.0 - spread + 2.0 * spread * float(rng.random()))
        removed += punch_hole(prims, bounds, p, radius=r, seed=seed + k * 17,
                              ragged=ragged)
    return removed


def structural_collapse(prims, intensity: float, seed: int = 0,
                        story_bias: str = None, mode: str = None,
                        report: dict = None) -> None:
    """Fails in place — but not the same way twice. Serves earthquake.

    Picks a failure mode (see `_QUAKE_MODES`) rather than always racking,
    pancaking one storey and spalling lightly. *mode* forces one, which is
    what the tests use.

    Each mode ends by REMOVING material as well as moving it. Deformation alone
    left a "collapsed" building with a complete envelope — every wall still
    there, just bent — which is the single biggest reason the damage read as
    squashed rather than broken. Where the openings go is part of what
    distinguishes the modes: a soft storey blows out at the ground floor, a
    partial collapse opens the failed side, a spall pits the whole facade.
    """
    b = bounds_of(prims)
    if b is None:
        return
    rng = np.random.default_rng(seed)
    mode = mode or _pick_quake_mode(intensity, rng)
    # The mode is not just this function's business: how much of the building
    # comes apart has to agree with how it failed, or a building that racked
    # and stood gets shattered to its foundations anyway. `apply_to_stage`
    # reads this back and feeds it to `fracture_plan`.
    if report is not None:
        report["mode"] = mode
    direction = float(rng.uniform(0, 360))
    hole_r = 0.05 + 0.09 * intensity
    facing = (math.cos(math.radians(direction)),
              math.sin(math.radians(direction)))

    # Surface damage is the part that reads as "earthquake" at any distance,
    # and it was far too subtle — 0.4-2% of radius. Every mode now works the
    # surface harder, and `spall` makes it the whole story.
    spall_amount = 0.010 + 0.030 * intensity
    lean_deg = 1.5 + 6.0 * intensity

    if mode == "racking":
        # Drifted off vertical and stayed up. Soft-storey drift concentrates
        # the lean at the base, which is what a weak ground floor does.
        lean(prims, b, angle_deg=lean_deg * 1.4, direction_deg=direction,
             profile="soft_story" if rng.random() < 0.5 else "linear")
        b = bounds_of(prims) or b
        crumble(prims, b, amount=spall_amount * 0.8, seed=seed,
                height_bias=-0.4, freq=7.0)
        # Still standing, so only shed cladding — a few openings low down.
        _holes(prims, b, 1 + int(round(3 * intensity)), hole_r * 0.7, seed,
               z_range=(0.0, 0.55))

    elif mode == "spall":
        # Structure holds; the cladding does not. Two octaves of surface work
        # at different frequencies so it reads as broken material rather than
        # as noise.
        lean(prims, b, angle_deg=lean_deg * 0.3, direction_deg=direction)
        b = bounds_of(prims) or b
        crumble(prims, b, amount=spall_amount * 1.6, seed=seed,
                height_bias=-0.3, freq=5.0)
        crumble(prims, b, amount=spall_amount * 0.7, seed=seed + 401,
                height_bias=0.2, freq=13.0)
        # The cladding is what fails here, so: many small openings, everywhere.
        _holes(prims, b, 3 + int(round(10 * intensity)), hole_r * 0.55, seed,
               ragged=0.6)

    elif mode == "soft_story":
        lean(prims, b, angle_deg=lean_deg, direction_deg=direction,
             profile="soft_story")
        b = bounds_of(prims) or b
        pancake(prims, b, z_lo=0.0, z_hi=0.16,
                collapse=0.55 + 0.35 * intensity,
                spread=0.14 + 0.14 * intensity, seed=seed)
        b = bounds_of(prims) or b
        crumble(prims, b, amount=spall_amount, seed=seed, height_bias=-0.5)
        # The ground floor is gone, so open it: the storey that crushed is the
        # one you can see through afterwards.
        _holes(prims, b, 2 + int(round(4 * intensity)), hole_r * 1.3, seed,
               z_range=(0.0, 0.22))

    elif mode == "mid_story":
        lo = float(rng.uniform(0.30, 0.60))
        lean(prims, b, angle_deg=lean_deg * 0.8, direction_deg=direction,
             profile="linear")
        b = bounds_of(prims) or b
        pancake(prims, b, z_lo=lo, z_hi=lo + 0.15,
                collapse=0.5 + 0.4 * intensity,
                spread=0.16 + 0.14 * intensity, seed=seed)
        b = bounds_of(prims) or b
        crumble(prims, b, amount=spall_amount, seed=seed, height_bias=0.2)
        _holes(prims, b, 2 + int(round(4 * intensity)), hole_r * 1.3, seed,
               z_range=(max(0.0, lo - 0.05), lo + 0.20))

    elif mode == "total":
        # Several bands crush in sequence, each re-measured, so the building
        # squats rather than telescoping through itself.
        for i in range(3):
            b = bounds_of(prims) or b
            lo = 0.05 + 0.30 * i
            pancake(prims, b, z_lo=lo, z_hi=lo + 0.22,
                    collapse=0.45 + 0.35 * intensity,
                    spread=0.18 + 0.16 * intensity, seed=seed + i * 97)
        b = bounds_of(prims) or b
        lean(prims, b, angle_deg=lean_deg * 1.6, direction_deg=direction,
             profile="linear")
        b = bounds_of(prims) or b
        crumble(prims, b, amount=spall_amount * 1.5, seed=seed, height_bias=0.0)
        # Nothing intact is left; the envelope should be full of gaps.
        _holes(prims, b, 4 + int(round(9 * intensity)), hole_r * 1.5, seed,
               ragged=0.6)

    else:   # "partial" — one side drops, the rest leans into the gap
        lo = float(rng.uniform(0.0, 0.35))
        pancake(prims, b, z_lo=lo, z_hi=lo + 0.20,
                collapse=0.4 + 0.4 * intensity,
                spread=0.24 + 0.20 * intensity, seed=seed)
        b = bounds_of(prims) or b
        # A large whip-profile lean after the collapse throws the standing part
        # over the failed side, which is what reads as a partial collapse.
        lean(prims, b, angle_deg=lean_deg * 2.2, direction_deg=direction,
             profile="whip")
        b = bounds_of(prims) or b
        crumble(prims, b, amount=spall_amount * 1.3, seed=seed,
                height_bias=0.4)
        # Open the side that dropped, not the side still holding it up.
        _holes(prims, b, 3 + int(round(6 * intensity)), hole_r * 1.8, seed,
               z_range=(lo, min(1.0, lo + 0.55)), facing=facing, ragged=0.6)


def blast_epicenter(prims, bounds: Bounds, seed: int = 0):
    """Where the charge sat, in normalized building space.

    Ported from the prototype's reasoning, both halves of which matter. It must
    be a point ON the building: a geometric guess at some radius and bearing
    lands in empty air whenever the footprint is not roughly circular — an
    L-plan or a long slab leaves most of its bounding box empty — and the blast
    then quietly does nothing. And it should be the most PERIPHERAL such point:
    a charge sits against an exterior wall, while a blast centred inside the
    building frees nearly every fragment at once and the whole thing
    disintegrates instead of losing a corner.
    """
    pts = (surface_points(prims, 24, bounds, seed=seed, z_range=(0.0, 0.45))
           or surface_points(prims, 24, bounds, seed=seed))
    if pts:
        return max(pts, key=lambda p: math.hypot(p[0], p[1]))
    ang = float(np.random.default_rng(seed).uniform(0, 2 * math.pi))
    return (0.9 * math.cos(ang), 0.9 * math.sin(ang), 0.25)


def blast(prims, intensity: float, seed: int = 0, epicenter=None) -> None:
    """A breach, then radial displacement and soot. Serves explosion."""
    b = bounds_of(prims)
    if b is None:
        return
    if epicenter is None:
        epicenter = blast_epicenter(prims, b, seed)
    # The breach comes FIRST, on undeformed geometry: the epicentre was chosen
    # against the surface as it stands, and `shockwave` moves that surface out
    # from under it.
    punch_hole(prims, b, epicenter, radius=0.06 + 0.22 * intensity, seed=seed,
               ragged=0.5)
    shockwave(prims, b, epicenter, radius=0.45 + 0.35 * intensity,
              strength=0.04 + 0.16 * intensity, falloff="shell", seed=seed,
              floor_z=b.base_z)
    b = bounds_of(prims) or b
    crumble(prims, b, amount=0.006 + 0.020 * intensity, seed=seed + 7,
            height_bias=0.3)
    # Local to the breach: a charge blackens the wall it went off against, not
    # the far side of the building. `burnout` is the one that chars everything.
    scorch(prims, strength=0.3 + 0.6 * intensity, bounds=b,
           epicenter=epicenter, radius=0.6 + 0.4 * intensity)


def wind_shear(prims, intensity: float, seed: int = 0,
               direction_deg: float = None, peel: float = None) -> None:
    """The top comes off. Serves tornado and hurricane.

    Wind loads a building at the roofline, and it fails from the top down —
    roof covering, then roof structure, then the top storey's walls. So the
    material removal is concentrated in a band at the top (*peel*, a fraction
    of the height) and biased to the windward side, and the lean uses the
    `whip` profile that accelerates upward.

    The lean is deliberately smaller than it was. A building that survived a
    storm is not visibly tilted — the tell is that its top is missing, which
    took `punch_hole` to express at all. The matching `fracture_plan` then
    shatters the same band into pieces that travel downwind.
    """
    b = bounds_of(prims)
    if b is None:
        return
    rng = np.random.default_rng(seed)
    if direction_deg is None:
        direction_deg = float(rng.uniform(0, 360))
    if peel is None:
        peel = 0.12 + 0.30 * intensity
    facing = (math.cos(math.radians(direction_deg)),
              math.sin(math.radians(direction_deg)))

    lean(prims, b, angle_deg=1.5 + 6.0 * intensity,
         direction_deg=direction_deg, profile="whip")
    b = bounds_of(prims) or b
    crumble(prims, b, amount=0.008 + 0.026 * intensity, seed=seed,
            height_bias=0.8)             # the wind works on the upper storeys
    b = bounds_of(prims) or b
    _holes(prims, b, 2 + int(round(6 * intensity)),
           0.08 + 0.20 * intensity, seed,
           z_range=(max(0.0, 1.0 - peel), 1.0), facing=facing, ragged=0.55)
    # A little damage lower down, on the windward face only — the storm took
    # windows and cladding off the side it hit, not off the whole building.
    _holes(prims, b, int(round(3 * intensity)), 0.05 + 0.06 * intensity,
           seed + 313, z_range=(0.1, max(0.2, 1.0 - peel)), facing=facing)


def burnout(prims, intensity: float, seed: int = 0) -> None:
    """A burnt-out shell: the roof is consumed, the walls stand. Serves fire.

    Ported from the prototype's `fire`, whose reasoning is the whole profile:
    fire destroys roofs and floors — timber, spanning, under load — long before
    it drops masonry walls, so the SILHOUETTE is the tell. Openings go in the
    top, nothing is thrown anywhere, and the charring is applied globally
    rather than from a point because the whole structure burns.

    That makes it the opposite of `wind_shear`, which also takes the top off:
    wind tears a band away and carries it downwind, fire consumes it in place
    and drops it inward. Same silhouette from a distance, opposite debris.
    """
    b = bounds_of(prims)
    if b is None:
        return
    # Open the roof up — a few large holes rather than many small ones, which
    # is what a collapsed roof deck reads as from the air.
    _holes(prims, b, 2 + int(round(5 * intensity)),
           0.08 + 0.16 * intensity, seed, z_range=(0.55, 1.0), ragged=0.6)
    b = bounds_of(prims) or b
    crumble(prims, b, amount=0.004 + 0.012 * intensity, freq=8.0, seed=seed + 3,
            height_bias=0.6)
    # Charred throughout, not from an epicentre: no `bounds`/`epicenter`, so
    # every material on the building takes the full weight.
    scorch(prims, strength=0.45 + 0.50 * intensity)


def inundation(prims, intensity: float, seed: int = 0) -> None:
    """Little structural loss; the damage is at the waterline. Serves flood."""
    b = bounds_of(prims)
    if b is None:
        return
    crumble(prims, b, amount=0.003 + 0.008 * intensity, seed=seed,
            height_bias=-0.9)            # scour and staining low down
    scorch(prims, strength=0.15 + 0.25 * intensity)   # silt line, not soot
    # Undermined footings and doors stoved in: a couple of small openings at
    # the waterline, and nothing above it. Water does not take a roof off.
    _holes(prims, b, int(round(2 * intensity)), 0.04 + 0.04 * intensity,
           seed + 77, z_range=(0.0, 0.2))


PROFILES = {
    "structural_collapse": structural_collapse,
    "blast": blast,
    "wind_shear": wind_shear,
    "burnout": burnout,
    "inundation": inundation,
}

# Which mechanism each disaster type reaches for. Adding a type is an entry
# here plus one in `fracture_plan` — no new operators, which is the point of
# keeping the profiles a layer below the taxonomy.
PROFILE_FOR_DISASTER = {
    "earthquake": "structural_collapse",
    "explosion": "blast",
    "tornado": "wind_shear",
    "hurricane": "wind_shear",
    "fire": "burnout",
    "flood": "inundation",
    "none": None,
}


def apply_profile(prims, disaster_type: str, intensity: float,
                  seed: int = 0, report: dict = None, **kw) -> str | None:
    """Run the profile for *disaster_type*. Returns its name, or None.

    *report*, when given, collects whatever the profile chose that the caller
    needs — today just the earthquake's failure mode, which `fracture_plan`
    has to agree with.
    """
    name = PROFILE_FOR_DISASTER.get(str(disaster_type).lower())
    if not name or intensity <= 0.0 or not prims:
        return None
    if report is not None and name == "structural_collapse":
        kw["report"] = report
    PROFILES[name](prims, float(np.clip(intensity, 0.0, 1.0)), seed=seed, **kw)
    return name


# ---------------------------------------------------------------------------
# WHAT SHATTERING MEANS FOR EACH DISASTER
#
# `fracture_to_stage` takes a height band, a fineness and a bearing. Which
# three you choose IS the difference between the disaster types once a building
# is in pieces, and getting them from config alone — one `fracture:` block
# copied identically into every preset, which is what the compiler emitted —
# meant every disaster shattered buildings the same way and only the debris
# around them differed.
#
#   earthquake  the whole building comes apart and drops where it stood
#   tornado     a deep peel off the top, thrown hard along the track
#   hurricane   a shallower peel, thrown along the storm bearing
#   fire        the roof and the floors under it, dropped straight in
#   explosion   the whole building, cut finest at the breach, thrown outward
#   flood       nothing. Water does not shatter masonry.
#
# The config block still wins where it sets a key — see `apply_to_stage` — so a
# preset can still override any of this; it just no longer has to supply it.
# ---------------------------------------------------------------------------

#: Keys a config `mesh_damage.fracture` block may override on a plan.
PLAN_KEYS = ("z_range", "n_cells", "focus", "focus_bias", "throw", "lift",
             "keep_base", "direction_deg")


#: How much of a building each earthquake failure mode actually frees, as
#: `keep_base` — the fraction of the height that stays standing on its footings.
#: A building that racked and stood must not be shattered to its foundations
#: just because it was in an earthquake; the mode already decided how it failed,
#: and the fracture has to agree with it or the six modes make no difference
#: once the pieces hit the ground.
_QUAKE_KEEP_BASE = {
    "racking":    0.80,     # cracked, standing. Almost nothing comes loose.
    "spall":      0.75,     # the cladding goes, the frame does not.
    "soft_story": 0.30,     # ground floor gone, the storeys above ride down.
    "mid_story":  0.40,     # one floor fails; below it is untouched.
    "partial":    0.22,     # one side drops.
    "total":      0.08,     # everything.
}


def fracture_plan(disaster_type: str, intensity: float,
                  heading_deg: float = None,
                  mode: str = None) -> dict | None:
    """Kwargs for `fracture_to_stage`, or None if this type does not shatter."""
    i = float(np.clip(intensity, 0.0, 1.0))
    d = str(disaster_type).lower()

    if d == "earthquake":
        # Whole-building, seeded at the base where the shaking is worst, and
        # dropped rather than thrown. How much comes loose is the failure
        # mode's call, softened a little by intensity.
        keep = _QUAKE_KEEP_BASE.get(mode, 0.45)
        return {"z_range": None,
                "n_cells": int(round(14 + 34 * i)),
                "focus": (0.0, 0.0, 0.05), "focus_bias": 0.35,
                "throw": 0.02 + 0.05 * i, "lift": 0.0,
                "keep_base": max(0.05, keep - 0.15 * i),
                "direction_deg": None}

    if d in ("tornado", "hurricane"):
        # Only the top comes apart, which is both what wind does and what makes
        # this affordable: the band is a fifth of the building, so it needs a
        # fifth of the cells and leaves the rest standing as its own prims.
        # `keep_base=0` because everything inside the band is loose by
        # definition — it is the part that tore off.
        gale = d == "tornado"
        peel = (0.18 + 0.42 * i) if gale else (0.10 + 0.25 * i)
        return {"z_range": (1.0 - peel, 1.0),
                "n_cells": int(round(8 + 22 * i)),
                "focus": (0.0, 0.0, 1.0), "focus_bias": 0.30,
                "throw": (0.20 + 0.45 * i) if gale else (0.10 + 0.25 * i),
                "lift": 0.05 + 0.15 * i,
                "keep_base": 0.0, "direction_deg": heading_deg}

    if d == "fire":
        # The roof and the floors under it are consumed, and they come down
        # WHERE THEY WERE — no throw, no lift, no bearing. `keep_base=0` because
        # everything inside the consumed band is loose by definition: the
        # timber holding it up is what burned. The masonry below is untouched,
        # which is the whole silhouette.
        consumed = 0.30 + 0.35 * i
        return {"z_range": (1.0 - consumed, 1.0),
                "n_cells": int(round(10 + 26 * i)),
                "focus": (0.0, 0.0, 1.0), "focus_bias": 0.35 + 0.30 * i,
                "throw": 0.0, "lift": 0.0,
                "keep_base": 0.0, "direction_deg": None}

    if d == "explosion":
        # `focus` is replaced with the real epicentre by `apply_to_stage`, so
        # the cut is finest exactly where the breach was punched.
        return {"z_range": None,
                "n_cells": int(round(18 + 42 * i)),
                "focus": (0.0, 0.0, 0.18), "focus_bias": 0.30 + 0.40 * i,
                "throw": 0.08 + 0.22 * i, "lift": 0.03 + 0.10 * i,
                "keep_base": 0.30 - 0.25 * i, "direction_deg": None}

    return None


def _heading_of(dis: dict):
    """The storm's bearing in degrees, or None where the type has none.

    A windstorm's damage all faces the same way and its debris all travels the
    same way; that shared bearing is the tornado signature, and the difference
    between a track and a bomb site.

    It has to be looked for in TWO places. `compile_tornado` writes
    `heading_deg` inside the `field` block — the corridor's bearing is a
    property of the field — and nothing ever wrote it at the top level, so
    reading only there silently returned None for every scene ever generated
    and each building picked its own random bearing. The unit test could not
    catch it: it passes `direction_deg` in directly.
    """
    for src in (dis, dis.get("field") or {}):
        v = src.get("heading_deg")
        if v is not None:
            return float(v)
    return None


def apply_to_stage(stage, config: dict, placements: list) -> dict:
    """Wreck the buildings the asset-swap route did not ruin.

    Runs after `apply_placements`, because it needs real prims: the damage
    authors overrides on `points` and on the topology of geometry inside a
    referenced layer, and there is nothing to override until the reference is
    composed.

    `disaster_stage.apply_to_buildings` marks the candidates with
    `_mesh_damage`, the local field intensity at that building — every building
    it decided to damage rather than swap, plus every one it wanted to destroy
    but had no ruin of the right footprint for. Buildings that *did* get a ruin
    swapped in already look ruined and are left alone; wrecking them too would
    be doing the same job twice.

    Returns ``{"tally": ..., "fragments": [...], "loose": [...]}``. `loose` is
    the subset of the fragments that came free of the structure, and the ONLY
    ones the caller may hand to the settle pass — see `fracture_to_stage` for
    what settling all of them costs.
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
    heading = _heading_of(dis)

    fcfg = (dis.get("mesh_damage") or {}).get("fracture") or {}
    # BUDGET. Each fragment is its own prim, and the settle pass gives every
    # one a collider and a rigid body. At ~50 cells a building, fracturing all
    # 167 that a severity-0.6 detailed downtown marks would author ~8,700 prims
    # and ask PhysX to rest them — on a scene that has OOM-killed at 89M
    # points. So shattering is a budget rather than a thing every damaged
    # building gets.
    #
    # It is spent on the WORST-HIT buildings, not on whichever ones the
    # placement list reached first. Placement order is essentially the packing
    # order, so the old first-come rule handed the budget to one corner of the
    # map and left the epicentre — the one place a viewer looks — deformed but
    # whole. The per-building seed still keys off the placement index, so
    # ranking changes which buildings shatter and never how one of them does.
    cands = [(i, p) for i, p in enumerate(placements)
             if p.get("_mesh_damage") and p.get("prim_path")]
    rank = sorted(cands, key=lambda ip: (-float(ip[1]["_mesh_damage"]), ip[0]))
    shatter = set()
    if fcfg.get("enabled", True):
        shatter = {i for i, _ in rank[:int(fcfg.get("max_buildings", 40))]}

    # Wall thickness, on the same budget and the same ranking. Doubling the
    # point count of every damaged building in a downtown is not affordable —
    # see the section header on `solidify` — and the buildings worth spending
    # it on are the same worst-hit ones fracture is spent on, so a shattered
    # building is never left as paper while an untouched one is solid.
    tcfg = (dis.get("mesh_damage") or {}).get("thickness") or {}
    wall_m = float(tcfg.get("wall_m", 0.25))
    thicken = set()
    if tcfg.get("enabled", True) and wall_m > 0.0:
        thicken = {i for i, _ in rank[:int(tcfg.get("max_buildings", 40))]}
    solid_kw = {k: tcfg[k] for k in ("max_points", "solid_ratio",
                                     "max_span_frac", "weld_tol")
                if k in tcfg}

    tally: dict = {}
    fragments: list = []
    loose: list = []
    for i, p in cands:
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
        inten = float(p["_mesh_damage"])
        bseed = seed + i * 31

        kw = {}
        if dtype in ("tornado", "hurricane") and heading is not None:
            kw["direction_deg"] = heading
        b = bounds_of(prims)
        if dtype == "explosion" and b is not None:
            # Chosen once, against the surface as it stands, and reused as the
            # fracture focus — so the mesh is cut finest where it was breached.
            kw["epicenter"] = blast_epicenter(prims, b, bseed)

        report: dict = {}
        name = apply_profile(prims, dtype, inten, seed=bseed, report=report,
                             **kw)
        if not name:
            continue
        tally[name] = tally.get(name, 0) + 1

        # Thickness. Runs AFTER the profile and BEFORE the fracture, and both
        # halves of that matter:
        #   * after, because the profile is what OPENS the building — every
        #     mode ends by punching holes — and `solidify` rims whatever edges
        #     it finds. Thicken first and the holes are punched back through
        #     both shells, leaving the same knife edge this exists to remove.
        #   * before, because the fracture cuts whatever the prims hold. Cut a
        #     shell and every fragment is a zero-thickness sheet; cut a slab
        #     and the rubble has chunks in it.
        # One pass therefore serves both, and nothing downstream needs to know.
        if i in thicken:
            b = bounds_of(prims)
            got = solidify_prims(prims, wall_m, bounds=b, **solid_kw)
            if got["meshes"]:
                tally["thickened"] = tally.get("thickened", 0) + 1

        # Shattering. Runs AFTER the vertex profile so the fragments are cut
        # from already-failed geometry, and takes the failure mode that profile
        # chose so the two agree about how much of the building survived.
        plan = fracture_plan(dtype, inten, heading_deg=heading,
                             mode=report.get("mode")) if i in shatter else None
        if plan is None:
            continue
        if fcfg.get("cells"):                 # legacy spelling of `n_cells`
            lo, hi = list(fcfg["cells"])[:2]
            plan["n_cells"] = int(round(lo + (hi - lo) * inten))
        plan.update({k: v for k, v in fcfg.items() if k in PLAN_KEYS})
        if "epicenter" in kw:
            plan["focus"] = tuple(kw["epicenter"])

        b = bounds_of(prims)                  # the profile moved the geometry
        if b is None:
            continue
        cut = fracture_to_stage(stage, prim, b, seed=bseed, **plan)
        fragments.extend(cut["paths"])
        loose.extend(cut["loose"])
        if cut["paths"]:
            tally["fragments"] = tally.get("fragments", 0) + len(cut["paths"])
            tally["loose"] = tally.get("loose", 0) + len(cut["loose"])
            tally["shattered"] = tally.get("shattered", 0) + 1

    if tally:
        print("[mesh_damage] deformed "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    return {"tally": tally, "fragments": fragments, "loose": loose}


# ---------------------------------------------------------------------------
# fracture
#
# Ported from `scenegen/damage.py:voronoi_fracture`, which builds each Voronoi
# cell by copying the source mesh and bisecting it with the perpendicular
# bisector plane between its seed and each nearby seed — it does not use
# Blender's cell-fracture addon (the `bpy` wheel does not ship one), so the
# algorithm was already explicit and only the bisection needed replacing.
#
# WHY THE CLIP IS HAND-WRITTEN
# ----------------------------
# `trimesh.intersections.slice_mesh_plane` does exactly this, and Isaac's Kit
# python already ships trimesh 4.5.1 — but importing it pulls in `shapely`,
# which Isaac does not have, and capping additionally wants a triangulation
# engine it does not have either. Adding both means a Dockerfile change and an
# image rebuild for what is fifty lines of Sutherland-Hodgman. So the clip is
# here, in numpy, and works in Kit today.
#
# Fragments come out as OPEN SHELLS — the cut face is not capped. That is what
# the prototype degrades to as well ("on non-manifold source meshes some caps
# will fail and those fragments stay shells — visually fine for rubble"), and
# building assets are non-manifold to begin with, so a cap is ill-defined: an
# open shell has no well-defined inside.
# ---------------------------------------------------------------------------


class Soup:
    """An unwelded triangle soup, plus the shading data a fragment needs.

    ``verts`` (V, 3) world-space, ``uv`` (V, 2), ``faces`` (T, 3) into verts,
    ``fmat`` (T,) ids into ``mats``, a list of material paths.

    UNWELDED — three vertices per triangle, never shared. That triples the
    vertex count, and buys the only thing that makes fragments look like part
    of the building they came from: `st` is authored *faceVarying* on real
    assets, so one point carries a different UV in each face that uses it and
    there is no per-point UV to interpolate. Splitting first makes every
    attribute per-vertex, after which the clip interpolates position and UV
    with the same parameter and fragments come out textured.

    Fragments cut from a welded soup would have to fall back to a flat colour,
    which is what "shattered building" looked like before: white confetti.
    """

    __slots__ = ("verts", "uv", "faces", "fmat", "mats")

    def __init__(self, verts, uv, faces, fmat, mats):
        self.verts, self.uv = verts, uv
        self.faces, self.fmat, self.mats = faces, fmat, mats

    def __len__(self):
        return len(self.faces)


def _clip_by_plane(verts, faces, normal, origin, uv=None, fmat=None):
    """Keep the half of a triangle soup on the negative side of a plane.

    Sutherland-Hodgman per triangle: a triangle straddling the plane is cut to
    a 3- or 4-gon and fan-triangulated. Makes no topology assumptions, so open
    shells and duplicated vertices pass through unharmed.

    *uv* rides along and is interpolated at each crossing with the same
    parameter as the position; *fmat* is per-face and is inherited by every
    triangle a cut face fans into. Both may be None, which is the plain
    positions-only form the tests exercise.

    The result is COMPACTED — vertices no longer referenced are dropped. That
    is not tidiness: `fracture` clips the same soup a dozen times in a row, and
    without compaction each pass carries the full original vertex array through
    every later one, so the cost never falls as the cell shrinks.
    """
    empty = np.zeros((0, 3), dtype=np.int64)
    if len(faces) == 0:
        return verts, faces, uv, fmat

    n = np.asarray(normal, dtype=np.float64)
    n = n / max(np.linalg.norm(n), 1e-12)
    d = (verts - np.asarray(origin, dtype=np.float64)) @ n     # signed distance

    inside = d[faces] <= 0.0               # (F, 3)
    n_in = inside.sum(axis=1)

    whole = n_in == 3                      # wholly inside — untouched
    keep = faces[whole]
    keep_mat = fmat[whole] if fmat is not None else None
    cut = np.nonzero((n_in == 1) | (n_in == 2))[0]

    new_v, new_uv, new_f, new_m = [], [], [], []
    base = len(verts)
    for fi in cut:
        tri = faces[fi]
        poly, poly_uv = [], []

        def _add(p, q=None, t=None):
            if q is None:
                poly.append(verts[p])
                if uv is not None:
                    poly_uv.append(uv[p])
            else:
                poly.append(verts[p] + t * (verts[q] - verts[p]))
                if uv is not None:
                    poly_uv.append(uv[p] + t * (uv[q] - uv[p]))

        for k in range(3):
            a, b = tri[k], tri[(k + 1) % 3]
            da, db = d[a], d[b]
            if da <= 0.0:
                _add(a)
            if (da < 0.0) < (db < 0.0) or (da > 0.0) > (db > 0.0):
                if abs(db - da) > 1e-12:            # crossing -> split vertex
                    _add(a, b, da / (da - db))
        if len(poly) < 3:
            continue
        start = base + len(new_v)
        new_v.extend(poly)
        new_uv.extend(poly_uv)
        for k in range(1, len(poly) - 1):           # fan
            new_f.append([start, start + k, start + k + 1])
            if fmat is not None:
                new_m.append(fmat[fi])

    if new_v:
        verts = np.concatenate([verts, np.asarray(new_v, dtype=np.float64)])
        if uv is not None:
            uv = np.concatenate([uv, np.asarray(new_uv, dtype=np.float64)])
        nf = np.asarray(new_f, dtype=np.int64)
        keep = np.concatenate([keep, nf]) if len(keep) else nf
        if fmat is not None:
            nm = np.asarray(new_m, dtype=np.int64)
            keep_mat = (np.concatenate([keep_mat, nm])
                        if keep_mat is not None and len(keep_mat) else nm)

    if len(keep) == 0:
        return verts[:0], empty, (uv[:0] if uv is not None else None), keep_mat

    used = np.unique(keep)
    remap = np.full(len(verts), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    return (verts[used], remap[keep],
            uv[used] if uv is not None else None, keep_mat)


def _seed_points(bounds: Bounds, n: int, seed: int, focus=None,
                 focus_bias: float = 0.0, surface=None,
                 surface_frac: float = 0.75) -> np.ndarray:
    """Voronoi seeds in world space.

    *surface*, if given, is an (M, 3) array of points ON the mesh; that
    fraction of the seeds is drawn from it. This matters more than it sounds:
    a building is a thin SHELL inside a mostly-empty bounding box, so seeding
    the box uniformly puts most seeds in interior air and leaves one enormous
    cell holding the bulk of the geometry — measured on a brownstone, the
    largest fragment was 31k of 61k triangles, i.e. not shattered at all.
    Seeding from the surface divides the shell instead of the air.

    *focus_bias* then pulls seeds toward *focus*, because real damage is finely
    pulverised at the source and coarsely cracked further out.
    """
    rng = np.random.default_rng(seed)
    box = rng.random((n, 3)) * (bounds.hi - bounds.lo) + bounds.lo
    if surface is not None and len(surface) and surface_frac > 0:
        k = int(round(n * float(np.clip(surface_frac, 0.0, 1.0))))
        if k:
            pick = rng.integers(0, len(surface), size=k)
            # jitter off the surface so cuts do not all land exactly on it
            jitter = (rng.random((k, 3)) - 0.5) * bounds.frac(0.04)
            box[:k] = surface[pick] + jitter
    pts = box
    if focus is not None and focus_bias > 0:
        f = bounds.to_world(focus)
        pull = rng.random((n, 1)) ** 2 * focus_bias
        pts = pts * (1 - pull) + f * pull
    return pts


def _triangulate(counts: np.ndarray):
    """``(tri_slots, tri_face)`` for a face-vertex layout.

    *tri_slots* (T, 3) indexes the FLATTENED face-vertex array rather than the
    point array, so the caller can pull both point indices and faceVarying data
    through the same triangulation. *tri_face* (T,) is each triangle's source
    face, which is how uniform data and material bindings follow.

    Vectorised per distinct face size — real assets are all quads and triangles,
    so this is two passes, against one Python iteration per face before.
    """
    starts = _fv_starts(counts)
    slots, src = [], []
    for c in np.unique(counts):
        c = int(c)
        if c < 3:
            continue
        sel = np.nonzero(counts == c)[0]
        s = starts[sel]
        for k in range(1, c - 1):
            slots.append(np.stack([s, s + k, s + k + 1], axis=1))
            src.append(sel)
    if not slots:
        return np.zeros((0, 3), dtype=np.int64), np.zeros(0, dtype=np.int64)
    return np.concatenate(slots), np.concatenate(src)


#: Primvar names that hold UVs when the type alone does not say so.
_UV_NAMES = ("st", "st0", "st_0", "uv", "UVMap", "map1")


def _uv_face_varying(prim, counts: np.ndarray, idx: np.ndarray):
    """The prim's UVs as one value per face-vertex, (n_fv, 2), or None.

    Normalising every interpolation to faceVarying here means the soup builder
    does not have to care which of the four an asset happened to use.
    """
    chosen = None
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        tn = pv.GetTypeName()
        role = getattr(tn, "role", "")
        if role == "TextureCoordinate" or pv.GetBaseName() in _UV_NAMES:
            chosen = pv
            if role == "TextureCoordinate":
                break
    if chosen is None:
        return None
    vals = chosen.Get()
    if vals is None or not len(vals):
        return None
    try:
        vals = np.asarray(vals, dtype=np.float64).reshape(-1, 2)
    except ValueError:
        return None
    if chosen.IsIndexed():
        ind = chosen.GetIndices()
        if ind is None:
            return None
        vals = vals[np.clip(np.asarray(ind, dtype=np.int64), 0, len(vals) - 1)]

    n_fv = int(counts.sum())
    interp = chosen.GetInterpolation()
    if interp == UsdGeom.Tokens.faceVarying:
        return vals if len(vals) == n_fv else None
    if interp in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying):
        return vals[idx] if int(idx.max()) < len(vals) else None
    if interp == UsdGeom.Tokens.uniform:
        return (np.repeat(vals, counts, axis=0)
                if len(vals) == len(counts) else None)
    if interp == UsdGeom.Tokens.constant:
        return np.repeat(vals[:1], n_fv, axis=0)
    return None


def _face_material_ids(prim, n_faces: int, table: list, index: dict):
    """(n_faces,) ids into *table* — the material bound to each face.

    The prim-level binding is the default; every face-`GeomSubset` overrides it
    for its own faces, which is how the AEC packs give one mesh a brick wall
    and a glass window. A fragment that ignored subsets would be bound to
    whichever material happened to sit at the top.
    """
    def _id(mat):
        p = (str(mat.GetPath())
             if mat and mat.GetPrim().IsValid() else "")
        if p not in index:
            index[p] = len(table)
            table.append(p)
        return index[p]

    base = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    out = np.full(n_faces, _id(base), dtype=np.int64)
    for child in prim.GetChildren():
        if not child.IsA(UsdGeom.Subset):
            continue
        sub = UsdGeom.Subset(child)
        et = sub.GetElementTypeAttr().Get()
        if et and et != UsdGeom.Tokens.face:
            continue
        mat = UsdShade.MaterialBindingAPI(child).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        ind = sub.GetIndicesAttr().Get()
        if ind is None:
            continue
        a = np.asarray(ind, dtype=np.int64)
        out[a[(a >= 0) & (a < n_faces)]] = _id(mat)
    return out


def _mesh_soup(prims, z_range=None, bounds: "Bounds | None" = None):
    """``(Soup, taken)`` — the prims' geometry as one soup, and what it took.

    *z_range* is a normalized height band of *bounds*; only faces whose
    centroid falls inside it enter the soup. *taken* is a per-prim boolean mask
    over that prim's faces saying which ones did, so `fracture_to_stage` can
    delete exactly the geometry it turned into fragments and leave the rest of
    the building standing as its own original, fully textured prims.

    That band is what makes "the roof tore off" expressible: without it the
    only fracture available is all-or-nothing, and a hurricane that shatters a
    tower to its foundations is the wrong picture.
    """
    V, UV, F, M, taken = [], [], [], [], []
    table, index = [], {}
    off = 0
    lo_z = hi_z = None
    if z_range is not None and bounds is not None:
        lo_z = bounds.lo[2] + float(z_range[0]) * bounds.dims[2]
        hi_z = bounds.lo[2] + float(z_range[1]) * bounds.dims[2]

    for prim in prims:
        counts, idx = _face_arrays(prim)
        pts = get_points(prim) if counts is not None else None
        if counts is None or not len(pts) or int(idx.max()) >= len(pts):
            taken.append(None)
            continue

        take = np.ones(len(counts), dtype=bool)
        if lo_z is not None:
            cz = (np.add.reduceat(pts[idx][:, 2], _fv_starts(counts)) / counts)
            take = (cz >= lo_z) & (cz <= hi_z)
        taken.append(take)
        if not take.any():
            continue

        uv_fv = _uv_face_varying(prim, counts, idx)
        mats = _face_material_ids(prim, len(counts), table, index)
        slots, src = _triangulate(counts)
        if not len(slots):
            continue
        sel = take[src]
        slots, src = slots[sel], src[sel]
        if not len(slots):
            continue

        flat = slots.reshape(-1)
        V.append(pts[idx[flat]])
        UV.append(uv_fv[flat] if uv_fv is not None
                  else np.zeros((len(flat), 2)))
        F.append(np.arange(off, off + len(flat),
                           dtype=np.int64).reshape(-1, 3))
        M.append(mats[src])
        off += len(flat)

    if not V:
        return Soup(np.zeros((0, 3)), np.zeros((0, 2)),
                    np.zeros((0, 3), dtype=np.int64),
                    np.zeros(0, dtype=np.int64), table), taken
    return Soup(np.concatenate(V), np.concatenate(UV), np.concatenate(F),
                np.concatenate(M), table), taken


def _fracture_soup(soup: Soup, n_cells: int = 40, seed: int = 0, focus=None,
                   focus_bias: float = 0.0, neighbors: int = 12,
                   min_faces: int = 4) -> list:
    """Shatter a soup into Voronoi cells. Returns a list of `Soup` fragments.

    Each cell is the source clipped by the perpendicular bisector between its
    seed and each of its *neighbors* nearest seeds — the prototype's
    construction, with the bisection swapped for `_clip_by_plane`.

    Seeds are drawn against the SOUP's own bounding box rather than the
    building's. With a height band that is the difference between seeding the
    torn-off roof and seeding the whole tower, of which the band is a tenth.

    Cells with fewer than *min_faces* triangles are dropped: a Voronoi cell in
    empty interior space clips down to nothing, and emitting those as prims
    costs draw calls for invisible geometry.
    """
    if len(soup) == 0 or n_cells < 2:
        return []
    box = Bounds(soup.verts.min(axis=0), soup.verts.max(axis=0))
    centroids = soup.verts[soup.faces].mean(axis=1)
    pts = _seed_points(box, int(n_cells), seed, focus, focus_bias,
                       surface=centroids)
    d2 = np.sum((pts[:, None, :] - pts[None, :, :]) ** 2, axis=-1)
    np.fill_diagonal(d2, np.inf)
    order = np.argsort(d2, axis=1)[:, :max(1, neighbors)]

    out = []
    for i, p in enumerate(pts):
        cv, cf, cuv, cm = soup.verts, soup.faces, soup.uv, soup.fmat
        for j in order[i]:
            q = pts[j]
            nrm = q - p
            if np.linalg.norm(nrm) < 1e-9:
                continue
            cv, cf, cuv, cm = _clip_by_plane(cv, cf, nrm, (p + q) * 0.5,
                                             cuv, cm)
            if len(cf) == 0:
                break
        if len(cf) >= min_faces:
            out.append(Soup(cv, cuv, cf, cm, soup.mats))
    return out


def fracture(prims, bounds: Bounds, n_cells: int = 40, seed: int = 0,
             focus=None, focus_bias: float = 0.0, neighbors: int = 12,
             min_faces: int = 4, z_range=None) -> list:
    """Shatter *prims* into Voronoi cells. Returns a list of `Soup` fragments.

    The read-only half of `fracture_to_stage` — nothing is authored and the
    source is left alone, which is what makes it testable without a stage.
    """
    soup, _ = _mesh_soup(prims, z_range, bounds)
    return _fracture_soup(soup, n_cells, seed, focus, focus_bias,
                          neighbors, min_faces)


def _bind_materials(stage, mesh, frag: Soup) -> None:
    """Give a fragment back every material its faces came from.

    Together with the `st` primvar authored alongside it, this is what stops
    shattered geometry rendering as untextured white — which read as confetti
    rather than as a building coming apart.

    The majority material is bound on the prim and each remaining one gets a
    `GeomSubset`, which is what the prototype's "fragments inherit the source's
    materials" amounts to on USD. Binding only the majority is tempting and
    wrong: it holds up on a tower cut into 50 cells, where a cell really is one
    material, but a house cut into 40 spans roof, wall and trim in nearly every
    cell — the damage gallery showed a whole street of them rendering in one
    flat off-white. Subsets carry no geometry, so the cost is a prim each and
    nothing is paid at all by the single-material fragments that skip it.
    """
    prim = mesh.GetPrim()
    if frag.fmat is None or not len(frag.fmat) or not frag.mats:
        return

    def _material(i):
        path = frag.mats[int(i)]
        if not path:
            return None
        mat = UsdShade.Material.Get(stage, path)
        return mat if mat and mat.GetPrim().IsValid() else None

    ids, counts = np.unique(frag.fmat, return_counts=True)
    ids = ids[np.argsort(-counts)]                  # majority first
    top = _material(ids[0])
    if top is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(top)
    for i in ids[1:]:
        mat = _material(i)
        if mat is None:
            continue
        faces = np.nonzero(frag.fmat == i)[0]
        sub = UsdGeom.Subset.CreateGeomSubset(
            mesh, f"mat_{int(i)}", UsdGeom.Tokens.face,
            Vt.IntArray([int(x) for x in faces]), "materialBind")
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)


def fracture_to_stage(stage, root_prim, bounds: Bounds, n_cells: int = 40,
                      seed: int = 0, focus=None, focus_bias: float = 0.0,
                      direction_deg: float = None, throw: float = 0.0,
                      lift: float = 0.0, keep_base: float = 0.35,
                      z_range=None) -> list:
    """Shatter *root_prim* in place and author the fragments. Returns paths.

    Fragments are authored under `<root>/fragments/`, carrying their `st` and a
    material binding so they still look like the building they came from.

    Their points are written in the ROOT's local space, not in world space.
    Everything upstream of here works in world space — that is the operators'
    whole convention — but a fragment is a child of the building prim and so
    inherits the building's transform, and `apply_placements` gives that prim
    the placement's translate, rotate and scale. Authoring world points under
    it applies all three a second time: on the centimetre-authored AEC packs
    (`scale: 0.01`) that puts the debris a hundred times too small and a long
    way from the building it came off.

    WHAT IS CONSUMED
    ----------------
    *z_range* is the normalized height band that comes apart. Only the faces in
    it are turned into fragments, and only those faces are removed from the
    source — so a wind profile can take the top fifth of a building and leave
    the remaining four fifths standing as their original prims, with their own
    materials, UVs and subsets untouched. With ``z_range=None`` the whole
    building is consumed and its source meshes end up empty, at which point
    `delete_faces` deactivates them (they live in a referenced layer, where
    `RemovePrim` does not compose — the trap `prune_prims` documents).

    WHERE THE PIECES GO, AND WHICH OF THEM FALL
    -------------------------------------------
    *keep_base* is the fraction of the building's height below which a fragment
    is still standing on its own footings. Those come back as ANCHORED; the
    rest come back as LOOSE, are thrown by *throw* along *direction_deg*, and
    are lofted by *lift* (both fractions of the building radius) — the
    windstorm signature, where everything loose travels one bearing rather than
    radiating symmetrically.

    That split is the whole reason intensity means anything, and it is the
    prototype's `partition` in one line. **Only the loose fragments may be
    settled.** Hand every fragment to PhysX and gravity levels the building
    whatever the severity was — a 0.3 earthquake and a 0.9 one both end as the
    same flat pile, and a severity sweep stops comparing anything. The
    anchored ones are cut where the geometry was and belong exactly there.

    Returns ``{"paths": [...], "loose": [...]}``; `loose` is the subset the
    caller should mark for the settle pass, and it is a subset of `paths`.
    """
    prims = mesh_prims(root_prim)
    soup, taken = _mesh_soup(prims, z_range, bounds)
    frags = _fracture_soup(soup, n_cells=n_cells, seed=seed, focus=focus,
                           focus_bias=focus_bias)
    if not frags:
        # Nothing came apart. Leave the source intact — deleting the band with
        # no fragments to replace it would just punch the top off the building.
        return {"paths": [], "loose": []}

    rng = np.random.default_rng(seed + 13)
    if direction_deg is None:
        direction_deg = float(rng.uniform(0.0, 360.0))
    ang = math.radians(direction_deg)
    dirx, diry = math.cos(ang), math.sin(ang)
    z_keep = bounds.base_z + keep_base * bounds.height
    push, up = bounds.frac(throw), bounds.frac(lift)

    scope = root_prim.GetPath().AppendChild("fragments")
    UsdGeom.Scope.Define(stage, scope)
    root_inv = np.linalg.inv(_world_matrix(root_prim))
    paths, loose = [], []
    for i, fr in enumerate(frags):
        v = fr.verts
        # Standing on its own footings, or freed? A piece whose lowest point is
        # still down in the base of the building has something under it.
        anchored = float(v[:, 2].min()) <= z_keep
        if not anchored and (push > 0.0 or up > 0.0):
            # Height-scaled, as the prototype has it: what started high had
            # further to travel and was less restrained on the way.
            t = np.clip((v[:, 2] - z_keep) / max(bounds.height, 1e-6), 0.0, 1.0)
            v = v.copy()
            v[:, 0] += dirx * push * t
            v[:, 1] += diry * push * t
            v[:, 2] += up * t

        p = scope.AppendChild(f"frag_{i:03d}")
        mesh = UsdGeom.Mesh.Define(stage, p)
        local = v @ root_inv[:3, :3] + root_inv[3, :3]
        mesh.GetPointsAttr().Set(
            Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in local]))
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(fr.faces)))
        mesh.GetFaceVertexIndicesAttr().Set(
            Vt.IntArray([int(x) for x in fr.faces.reshape(-1)]))
        # The soup is unwelded, so UVs are already one per vertex.
        if fr.uv is not None and len(fr.uv) == len(v):
            pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
            pv.Set(Vt.Vec2fArray([Gf.Vec2f(float(a), float(b))
                                  for a, b in fr.uv]))
        _bind_materials(stage, mesh, fr)
        paths.append(str(p))
        if not anchored:
            loose.append(str(p))

    # Only now: the soup had to be built from the faces before they went.
    # `deactivate` is refused for the prim the fragments were authored under —
    # see `delete_faces`; retiring it would take them with it.
    for prim, take in zip(prims, taken):
        if take is not None and take.any():
            delete_faces(prim, ~take, deactivate=prim != root_prim)
    return {"paths": paths, "loose": loose}
