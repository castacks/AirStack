"""
mesh_damage.py — what a disaster does to a building's actual geometry.

THE MODEL, IN FOUR STAGES
-------------------------
A building is not damaged by a list of effects. It is damaged by *losing
structural material in a region of space*, and everything else — the missing
storey, the torn-off roof, the blown-out corner, the pile in the street —
follows from which region and how much. So there is one pipeline, and the only
thing a disaster type gets to choose is the shape of that region:

  1. **THICKEN.** Give the shell volume (`solidify`). Almost every asset in
     this library is a zero-thickness surface, and a fragment cut from one is a
     sheet of paper. Nothing downstream can look like broken masonry until the
     wall has a thickness to break through.

  2. **FAIL.** Evaluate a **failure field** over world space — a scalar
     ``damage(p) ∈ [0, 1]`` saying how much of the material at *p* has lost its
     integrity, plus an ``ejecta(p)`` saying where that material was thrown.
     **This is the whole of the disaster specialization** (`field_earthquake`,
     `field_tornado`, `field_hurricane`, `field_fire`,
     `field_flood`) — see `Failure`.

  3. **PROPAGATE.** Cut the mesh along a crack network seeded from the failure
     field: dense where damage is high, absent where it is zero. Voronoi cell
     fracture with capped cuts, over a solid, gives **closed 3D fragments**
     (`fracture_to_stage`). Failure also propagates *structurally*: a fragment
     whose supporting column has failed comes free even if its own material did
     not — which is what makes a lost ground floor bring the storeys above it
     down without anything having to name "soft-storey collapse".

  4. **SETTLE.** Released fragments are handed to PhysX as rigid bodies by the
     caller; anchored ones stay exactly where they were cut. `apply_to_stage`
     returns the split, `generate_scene` marks the loose ones `settle: True`.

WHAT WAS REMOVED, AND WHY
-------------------------
An earlier version of this module had a second, parallel vocabulary: `lean`,
`pancake`, `crumble`, `shockwave` and `punch_hole`, composed into five hand-
tuned "profiles" and six enumerated earthquake "failure modes". Every one of
them was a guess at the *appearance* of a failure rather than a statement about
it, they did not compose (a leaned building had to be re-measured before it
could be pancaked), and they were redundant with the fracture that ran after
them. The failure field says the same things and more, once:

  * a soft storey is a horizontal band of high damage — which falls out of
    noise with a storey-height grain, not out of a mode table;
  * a blown-out corner is a ball of damage centred on the charge;
  * a torn-off roof is damage that grows with height;
  * "the building leaned" is what PhysX does to a stack of released fragments.

THE TWO INVARIANTS THIS MODULE OWES THE GENERATOR
-------------------------------------------------
* **Severity has to mean something.** The released fraction is a threshold on
  the field, so it grows smoothly with intensity: a 0.3 event frees a few
  pieces of the weakest storey, a 0.9 event frees most of the building. If
  every fragment were settled instead, gravity would level every building at
  every severity and a severity sweep would compare nothing.
* **Layout is not touched.** Nothing here moves a building, changes a
  footprint, or draws from the layout RNG. Damage is authored as overrides on
  geometry that `apply_placements` has already placed.

TWO THINGS THE CODE ASSUMES
---------------------------
* **World space.** Points are read through the prim's local-to-world transform
  and written back through its inverse, so an epicentre is a position in the
  scene and not in some asset's authoring frame.
* **`Bounds` is a snapshot.** `solidify` moves geometry, so the bounds are
  re-measured after it and before the field is built.

WHY THIS IS NOT BLENDER
-----------------------
`scenegen/damage.py` is the Blender prototype some of this descends from. `bpy`
cannot run inside Isaac Sim: it statically links its own USD and bundles oneTBB
12 while Isaac ships `libusd_*.so` and TBB 2 with a malloc proxy, and two USD
runtimes plus two allocators in one address space is a crash waiting to happen.
It is also GPL against AirStack's MIT. None of that matters, because the maths
is numpy and the only Blender-shaped dependency was reading and writing points.
"""

import math
import os
import time
from collections import namedtuple

import numpy as np

# pxr is the one hard dependency, and it is present everywhere this runs:
# Isaac's Kit python, and `usd-core` on the host for the offline tests.
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

# ---------------------------------------------------------------------------
# 0. THE USD ADAPTER
#
# Everything above this line is numpy on (N, 3) world-space arrays. These are
# the dozen functions that make a `UsdGeom.Mesh` look like one.
# ---------------------------------------------------------------------------


def stable_seed(*parts, mod: int = 9973) -> int:
    """A reproducible small integer from *parts*. Use instead of `hash()`.

    PYTHON'S `hash()` IS SALTED PER PROCESS for str and bytes, so
    ``abs(hash((asset, level))) % 9973`` draws a DIFFERENT seed every run. Every
    caller of it was quietly opting out of the one invariant this generator
    rests on — config plus seed reproduces the scene — and the symptom is the
    least alarming one possible: the output looks fine, it is just never the
    same twice, so a severity sweep compares two different cities and an
    archetype re-bake silently replaces the library.

    `crc32` is not a hash function in the security sense and does not need to
    be. It needs to be cheap, well spread over small inputs, and identical in
    every process, which it is and `hash()` is not.
    """
    import zlib

    key = "\x1f".join(str(p) for p in parts).encode("utf-8")
    return int(zlib.crc32(key) % int(mod))


def deinstance(root: "Usd.Prim") -> int:
    """Make every instance under *root* editable. Returns how many were opened.

    Authoring to an instance proxy raises — its geometry lives in a shared
    prototype, and a per-instance opinion is exactly what USD instancing exists
    to forbid. The AEC packs ship *internally* instanced (a brownstone is
    `/World/Brownstone02_Instanced/...` with 307 meshes behind one instance
    root), so damaging one fails on the first `set_points` unless the instances
    are opened first.

    **This costs memory**, which is the whole point of instancing: each opened
    instance stops sharing its prototype's geometry. The generated scene
    already OOM-killed once at 89.1M points, so open instances only on the
    buildings actually being damaged — which is what `apply_to_stage` does —
    and never wholesale.
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
    and the geometry lives inside the referenced layer.
    """
    if not root or not root.IsValid():
        return []
    return [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
            if p.GetTypeName() == "Mesh"]


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


# VT ARRAYS ARE BUILT WITH `FromNumpy`, NEVER WITH A LIST COMPREHENSION.
#
# `Vt.Vec3fArray([Gf.Vec3f(*p) for p in arr])` is one Python call and one
# temporary object PER VERTEX, and a fractured building has millions of them:
# profiled on one `BG_Building_F` at `partial_collapse`, the three conversions
# these helpers replace cost 107 s of a 428 s fracture — a quarter of it, spent
# entirely on marshalling. `FromNumpy` hands the buffer over in one call.
#
# It is strict about dtype (int32 / float32) and about shape, so the cast is
# done here rather than at each call site.

def _vt_int(arr) -> Vt.IntArray:
    """Any integer sequence -> Vt.IntArray."""
    return Vt.IntArray.FromNumpy(np.ascontiguousarray(arr, dtype=np.int32))


def _vec3f(arr: np.ndarray):
    """(N, 3) float array -> Vt.Vec3fArray, the type the attribute wants."""
    return Vt.Vec3fArray.FromNumpy(
        np.ascontiguousarray(arr, dtype=np.float32).reshape(-1, 3))


def _vec2f(arr: np.ndarray):
    """(N, 2) float array -> Vt.Vec2fArray, for UVs."""
    return Vt.Vec2fArray.FromNumpy(
        np.ascontiguousarray(arr, dtype=np.float32).reshape(-1, 2))


def set_points(prim, world: np.ndarray) -> None:
    """Write world-space points back through the prim's transform.

    Authors an *override* on the points attribute — the mesh lives in a
    referenced layer, and an opinion in a stronger layer composes over it.
    This is why damaged buildings cannot be instanceable: an instance's
    descendants live in a shared prototype and take no per-instance opinion.
    """
    M = _world_matrix(prim)
    Minv = np.linalg.inv(M)
    UsdGeom.Mesh(prim).GetPointsAttr().Set(
        _vec3f(world @ Minv[:3, :3] + Minv[3, :3]))


class Bounds:
    """World-space AABB, plus normalized-building-space conversion.

    Normalized space puts (0, 0, 0) at the centre of the footprint at ground
    level: x, y in [-1, 1] and z in [0, 1] cover the building. Sizes are
    fractions of `radius`, which stops "a 3 m fragment" meaning something
    different on every asset — the library spans 15 m houses to 90 m towers.
    """

    __slots__ = ("lo", "hi")

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
        """A normalized ``(u, v, t)`` as a world point."""
        px, py, pz = p
        half = self.dims * 0.5
        c = self.center
        return np.array([c[0] + px * half[0],
                         c[1] + py * half[1],
                         self.lo[2] + pz * self.dims[2]], dtype=np.float64)

    def frac(self, f: float) -> float:
        """A length given as a fraction of the building, in metres."""
        return float(f) * self.radius


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


def plan_coords(bounds: Bounds, pts: np.ndarray):
    """World points as ``(u, v, t)``: plan in [-1, 1], height in [0, 1].

    The failure fields are written in these coordinates so that the same field
    means the same thing on a bungalow and on a tower — which is what makes a
    severity sweep comparable across a mixed street.
    """
    half = bounds.dims * 0.5
    half = np.where(np.abs(half) < 1e-9, 1e-9, half)
    c = bounds.center
    u = (pts[:, 0] - c[0]) / half[0]
    v = (pts[:, 1] - c[1]) / half[1]
    t = (pts[:, 2] - bounds.lo[2]) / max(bounds.dims[2], 1e-9)
    return u, v, t


# ---------------------------------------------------------------------------
# 1. TOPOLOGY BOOKKEEPING
#
# Two operations rewrite a mesh's face arrays: `delete_faces` (fracture
# consumes the geometry it turned into fragments) and `subdivide` (a resolution
# floor, so a fragment is not shaped by the model's tessellation). USD indexes
# three things BY FACE, and all three go silently wrong if the arrays are
# renumbered without them:
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
# These two functions are the only place that knows this; nothing else may
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


def face_centroids(prim) -> "np.ndarray | None":
    """Per-face centroid in world space, (F, 3), or None.

    The one place a field is sampled against a mesh: a face is damaged when its
    centroid is, and Voronoi seeds are drawn from these points because a
    building is a thin shell inside a mostly-empty box (see `fracture_seeds`).
    """
    counts, idx = _face_arrays(prim)
    if counts is None:
        return None
    pts = get_points(prim)
    if not len(pts) or int(idx.max()) >= len(pts):
        return None
    return (np.add.reduceat(pts[idx], _fv_starts(counts), axis=0)
            / counts[:, None])


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
            _vt_int(np.asarray(ind)[mask]))
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
        sub.GetIndicesAttr().Set(_vt_int(new_of[a]))


def delete_faces(prim, keep, deactivate: bool = True) -> int:
    """Keep only the faces *keep* marks. Returns how many were removed.

    Authors overrides on the topology attributes, exactly as `set_points` does
    on `points` — so the same rule applies: the prim must not be instanced.

    When nothing survives the prim is DEACTIVATED rather than left holding
    empty topology. Empty face arrays are legal USD but some renderers dislike
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
    nothing but the ground.
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
        # …and empty its subsets with it. Left alone they keep indexing the
        # faces that are gone, and every USD consumer that checks says so —
        # Blender logs "UsdGeomSubset 'Section0' contains invalid indices"
        # for a prim that now draws nothing. Noise, but the kind that hides
        # the warning that matters.
        for sub in prim.GetChildren():
            if sub.IsA(UsdGeom.Subset):
                UsdGeom.Subset(sub).GetIndicesAttr().Set(Vt.IntArray([]))
        return n_gone

    fv_keep = np.repeat(keep, counts)
    mesh.GetFaceVertexCountsAttr().Set(
        _vt_int(counts[keep]))
    mesh.GetFaceVertexIndicesAttr().Set(
        _vt_int(idx[fv_keep]))

    _filter_by_element(mesh.GetNormalsAttr(), None,
                       mesh.GetNormalsInterpolation(), keep, fv_keep)
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        _filter_by_element(pv.GetAttr(), pv, pv.GetInterpolation(),
                           keep, fv_keep)
    _renumber_subsets(prim, keep)
    return n_gone


def _flatten_primvar(pv) -> None:
    """Resolve an indexed primvar to a flat one, in place.

    Subdivision has to *interpolate* faceVarying data — a midpoint's `st` is
    the average of the two corners' — and an index cannot be averaged. Values
    can. Flattening first is the standard way out and costs only memory.
    """
    if not pv.IsIndexed():
        return
    ind = pv.GetIndices()
    vals = pv.Get()
    if ind is None or vals is None:
        return
    try:
        pv.GetAttr().Set(type(vals)([vals[int(i)] for i in ind]))
        pv.BlockIndices()
    except Exception:
        pass


def _pv_array(vals):
    """A Vt array as ``(np_values, rebuild)``; rebuild puts numpy back as Vt."""
    a = np.asarray(vals)
    t = type(vals)
    if a.ndim == 1:
        return a.astype(np.float64), lambda x: t([type(vals[0])(v) for v in x])
    return a.astype(np.float64), lambda x: t([tuple(v) for v in x])


def _resubset(prim, face_src: np.ndarray) -> None:
    """Remap every face-`GeomSubset` onto the subdivided faces."""
    for child in prim.GetChildren():
        if not child.IsA(UsdGeom.Subset):
            continue
        sub = UsdGeom.Subset(child)
        if sub.GetElementTypeAttr().Get() != UsdGeom.Tokens.face:
            continue
        old = sub.GetIndicesAttr().Get()
        if old is None:
            continue
        want = np.zeros(int(face_src.max()) + 1, dtype=bool)
        keep = np.asarray([int(i) for i in old], dtype=np.int64)
        keep = keep[keep <= int(face_src.max())]
        want[keep] = True
        new = np.nonzero(want[face_src])[0]
        sub.GetIndicesAttr().Set(_vt_int(new))


def subdivide(prim, max_edge: float, max_points: int = 400_000,
              max_rounds: int = 4) -> int:
    """Split faces until no edge is longer than *max_edge*. Returns faces added.

    WHY DAMAGE NEEDS A RESOLUTION FLOOR
    -----------------------------------
    Building assets are modelled for rendering, not for being broken: measured
    on the packs in use here, `BG_Building_D` carries triangles up to 255 m²
    with a 22 m edge. `solidify` extrudes such a face into a single 22 m slab,
    and a Voronoi cell cut out of two enormous triangles is itself two enormous
    triangles — which is what makes rubble read as flat shards no matter how
    many cells are asked for. The fracture's *silhouettes* are exact at any
    tessellation (`_clip_by_plane` creates vertices on the cut), but the
    interior detail is not, and neither is the rim `solidify` builds.

    So this runs first, in WORLD METRES, and `max_edge` therefore means the
    same size of rubble on a tower and on a shed.

    PRIMVARS ARE CARRIED AS VALUES, NOT AS INDICES
    ----------------------------------------------
    Each round rebuilds the topology, so any index into the ORIGINAL arrays
    stops meaning anything after the first one. Deferring the primvar rebuild
    to a final gather over such indices is what put `primvars:st` at its
    original length on a mesh with 7x the face-vertices — USD then dropped the
    UVs as inconsistent with the interpolation. So the values themselves are
    interpolated in step with the geometry, every round.

    WHAT IT DOES NOT DO
    -------------------
    Only faces that need it are split, and split whole (1 -> 4 on a triangle),
    with midpoints shared through an edge map. A split face beside an unsplit
    one therefore leaves a **T-junction**. That is deliberate: making it
    conforming needs red-green refinement and neighbour propagation, and the
    unsplit neighbour is by definition already below `max_edge`. Uniform
    subdivision would avoid it and quadruple every asset — 471k triangles to
    1.9M on the largest building here — which the point budget exists to stop.

    Quads and n-gons are triangulated on the way through; the mesh comes out
    all-triangles. Indexed primvars are flattened (see `_flatten_primvar`).
    """
    counts, idx = _face_arrays(prim)
    if counts is None:
        return 0
    pts = get_points(prim)
    if len(pts) < 3 or int(idx.max()) >= len(pts):
        return 0

    slots, face_src = _triangulate(counts)
    if not len(slots):
        return 0

    mesh = UsdGeom.Mesh(prim)
    n_fv0, n_pt0, n_face0 = int(counts.sum()), len(pts), len(counts)

    # --- primvars, as values laid out the way the geometry is ---------------
    fv_sets, pt_sets, uni_sets = [], [], []
    entries = [(mesh.GetNormalsAttr(), None, mesh.GetNormalsInterpolation())]
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        _flatten_primvar(pv)
        entries.append((pv.GetAttr(), pv, pv.GetInterpolation()))
    for attr, pv, interp in entries:
        vals = attr.Get() if attr else None
        if vals is None or not len(vals):
            continue
        try:
            arr, rebuild = _pv_array(vals)
        except Exception:
            continue
        if interp == UsdGeom.Tokens.faceVarying and len(arr) == n_fv0:
            fv_sets.append([attr, rebuild, arr[slots]])      # (T, 3, ...)
        elif interp in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying) \
                and len(arr) == n_pt0:
            pt_sets.append([attr, rebuild, arr])             # (P, ...)
        elif interp == UsdGeom.Tokens.uniform and len(arr) == n_face0:
            uni_sets.append([attr, rebuild, arr])            # (F, ...)

    pt = idx[slots]                             # (T, 3) point indices
    parent = face_src.copy()                    # (T,) original face
    added = 0

    for _round in range(int(max_rounds)):
        a, b, c = pts[pt[:, 0]], pts[pt[:, 1]], pts[pt[:, 2]]
        elen = np.stack([np.linalg.norm(b - a, axis=1),
                         np.linalg.norm(c - b, axis=1),
                         np.linalg.norm(a - c, axis=1)], axis=1)
        need = elen.max(axis=1) > float(max_edge)
        if not need.any():
            break
        if len(pts) + int(need.sum()) * 3 > int(max_points):
            break

        sel = np.nonzero(need)[0]
        keep = np.nonzero(~need)[0]

        # Shared midpoints, keyed on the unordered point pair, so two triangles
        # that both split an edge get the SAME new vertex and stay welded.
        mid: dict = {}
        new_pt_pairs = []
        e_of = np.empty((len(sel), 3), dtype=np.int64)
        for r, t in enumerate(sel):
            for k, (i, j) in enumerate(((0, 1), (1, 2), (2, 0))):
                pa, pb = int(pt[t, i]), int(pt[t, j])
                key = (min(pa, pb), max(pa, pb))
                if key not in mid:
                    mid[key] = len(pts) + len(new_pt_pairs)
                    new_pt_pairs.append((pa, pb))
                e_of[r, k] = mid[key]

        if new_pt_pairs:
            pr = np.asarray(new_pt_pairs, dtype=np.int64)
            pts = np.concatenate([pts, (pts[pr[:, 0]] + pts[pr[:, 1]]) / 2.0])
            for st in pt_sets:                       # vertex/varying follow
                st[2] = np.concatenate(
                    [st[2], (st[2][pr[:, 0]] + st[2][pr[:, 1]]) / 2.0])

        # Corner order of the four children, as (corner, edge) slots. `None`
        # in the first slot means "the midpoint of edge 01/12/20".
        CHILD = (((0, None), (None, 0), (None, 2)),
                 ((1, None), (None, 1), (None, 0)),
                 ((2, None), (None, 2), (None, 1)),
                 ((None, 0), (None, 1), (None, 2)))
        EDGE_ENDS = ((0, 1), (1, 2), (2, 0))

        new_pt = [pt[keep]]
        new_par = [parent[keep]]
        new_fv = [[st[2][keep]] for st in fv_sets]
        for child in CHILD:
            blk = np.empty((len(sel), 3), dtype=np.int64)
            for k, (sc, se) in enumerate(child):
                blk[:, k] = (pt[sel, sc] if sc is not None else e_of[:, se])
            new_pt.append(blk)
            new_par.append(parent[sel])
            for si, st in enumerate(fv_sets):
                arr3 = st[2]
                cols = []
                for sc, se in child:
                    if sc is not None:
                        cols.append(arr3[sel, sc])
                    else:
                        i, j = EDGE_ENDS[se]
                        cols.append((arr3[sel, i] + arr3[sel, j]) / 2.0)
                new_fv[si].append(np.stack(cols, axis=1))

        pt = np.concatenate(new_pt)
        parent = np.concatenate(new_par)
        for si, st in enumerate(fv_sets):
            st[2] = np.concatenate(new_fv[si])
        added += 3 * len(sel)

    if not added:
        return 0

    n_tri = len(pt)
    mesh.GetFaceVertexCountsAttr().Set(_vt_int(np.full(n_tri, 3)))
    mesh.GetFaceVertexIndicesAttr().Set(
        _vt_int(pt.reshape(-1)))
    set_points(prim, pts)

    for attr, rebuild, arr in fv_sets:
        try:
            attr.Set(rebuild(arr.reshape(-1, *arr.shape[2:])))
        except Exception:
            pass
    for attr, rebuild, arr in pt_sets:
        try:
            attr.Set(rebuild(arr))
        except Exception:
            pass
    for attr, rebuild, arr in uni_sets:
        try:
            attr.Set(rebuild(arr[parent]))
        except Exception:
            pass
    _resubset(prim, parent)
    return added


# ---------------------------------------------------------------------------
# 2. STAGE ONE — THICKNESS
#
# THE PROBLEM THIS SOLVES
# -----------------------
# Much of the library is a HOLLOW SHELL: a single surface with no thickness at
# all, because nothing ever needed the inside of a wall. Measured, an objaverse
# bungalow encloses 51 m³ against the 820 m³ a 0.25 m slab of its own surface
# area would have — i.e. essentially none. That assumption survives right up
# until the wall is broken, and then it is the single thing that makes
# procedural damage read as fake: a Voronoi cell of a zero-thickness surface is
# a zero-thickness surface, so the "rubble" is a drift of curved sheets —
# visually, confetti with a brick texture on it. No cell count fixes that,
# because there is no volume to cut.
#
# It is also what makes stage three's promise possible at all. A **closed**
# fragment requires a closed input: cut a solid with capped planes and every
# piece is a solid. Cut a shell and every piece is a shell.
#
# WHY INWARD, AND HOW THAT DIRECTION IS FOUND
# -------------------------------------------
# Outward would inflate the building past its own footprint — the layout stage
# has already packed these to setbacks and a building that grows by 0.5 m on
# every face starts intersecting its neighbours. Inward eats into interior
# space that is empty anyway.
#
# Which way is "in" cannot be read off the winding, because the packs are not
# consistently wound and USD's `orientation` metadata is unreliable on
# converted assets. Nor is one sign per mesh enough: measured on a bungalow,
# the roof and the ground slab each carry faces of BOTH windings, so whichever
# global sign is picked, part of the mesh extrudes the wrong way — visible as
# the building's bounding box growing by most of the wall thickness. The
# direction is therefore settled PER POINT, by flipping each normal to face
# away from the building's centre. That ignores winding entirely and is right
# for the shapes this deals with: a roof plane thickens downward, a ground slab
# upward, a facade inward.
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
        primvar.SetIndices(_vt_int(a))
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
    follow, the inside of every wall and every cut edge renders with whichever
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
        sub.GetIndicesAttr().Set(_vt_int(out))


def solidify(prim, thickness: float, ref_centre=None, ref_box=None,
             weld_tol: float = 1e-4, max_span_frac: float = 0.35,
             max_points: int = 800_000, orient=None) -> int:
    """Extrude *prim*'s shell inward into a slab. Returns faces added, or 0.

    *ref_centre* / *ref_box* describe the whole building in world space; pass
    them whenever the prim is one mesh of many, which on a real asset it always
    is. See `_surface_normals` and the section header for why a per-mesh guess
    is not good enough.

    *thickness* is in WORLD METRES, deliberately — a wall is about half a metre
    thick whether it is on a bungalow or a tower, so this is the one quantity
    in the module that is NOT a fraction of the building radius.

    It is still capped, because assets carry window glass, railings, gutters
    and signage as their own thin meshes and extruding a 5 cm mullion by 50 cm
    turns a facade into a wall of blocks. The cap is *max_span_frac* of the
    mesh's SECOND-SMALLEST bounding-box dimension. Both halves of that are
    deliberate: the smallest dimension is useless because a flat wall panel is
    zero-thickness by definition and would cap to nothing, while the second
    smallest is the panel's short side — 3 m for a wall, 5 cm for a mullion,
    which is exactly the distinction wanted. Median edge length was the obvious
    alternative and is wrong: it measures how finely a surface is tessellated,
    not how thin the thing is, so it throttled dense wall meshes to 9 cm while
    leaving coarse ones at full thickness.

    EVERY MESH IS THICKENED; THE ASSET SET SAYS OTHERWISE
    -----------------------------------------------------
    This does not try to work out whether a mesh is already solid. Two attempts
    at that are in the history and both were wrong in practice — the enclosed
    volume (which for a closed building is the air in its rooms, so every
    Nucleus building was declared solid and left a paper balloon) and then a
    ray probe (right, but a guess about art remade on every run).

    An asset's author knows the answer, so it is declared: `solid: true` on the
    entry in the asset pack, read by `scene_generator.solid_assets` and applied
    by `apply_to_stage`, which simply does not call this for those. Unmarked
    means shell, because that is what nearly all of this library is.

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
    if n_pts < 3 or int(idx.max()) >= n_pts:
        return 0
    # THE CAP IS A BUDGET, AND IT WAS SET BELOW THE LIBRARY IT SERVES.
    #
    # At 200_000 this refused every merged AEC/Nucleus building in `urban.yaml`
    # — they are single meshes of 100k-340k faces and 470k+ points — so the
    # whole city library silently skipped stage one and fractured as paper,
    # which is the exact artifact this function exists to prevent. Nothing in
    # the report said so: `thickened=0` reads identically to "the asset set
    # declared it solid".
    #
    # Measured on `SM_MERGED_BP_MBuilding02` (471,324 points): 6.1 s and one
    # doubling of the point count. That is a real cost but it is linear and it
    # is affordable, so the cap belongs above it rather than below. It stays
    # finite because the failure past a few million points is an OOM kill, not
    # a slow run.
    if n_pts > max_points:
        print("[mesh_damage] solidify DECLINED {0}: {1} points over the {2} "
              "cap — this mesh stays a shell".format(
                  prim.GetPath(), n_pts, max_points), flush=True)
        return 0

    outward, med_edge, _volume, _area = _surface_normals(
        world, counts, idx, weld_tol, ref_centre)
    if outward is None or med_edge <= 0.0:
        return 0
    if orient is not None:
        # An oracle that actually knows which side is outside, supplied by a
        # caller that can afford one. The radial fallback below `_surface_
        # normals` only fires on normals leaning inward by more than ~8.6
        # degrees, so a normal TANGENTIAL to the radial direction — the top
        # face of a window sill, +Z on a facade that faces +X — is decided by a
        # sign averaged over the whole prim, which for a prim holding a wall
        # and a hundred pieces of trim is arbitrary. Those points extrude the
        # wrong way and the new skin erupts through the facade (measured on an
        # Objaverse house: 19.6% of vertices pushed a full thickness outward,
        # 414 of 1216 pieces). Nothing here depends on it — omit it and the
        # heuristic stands, which is what the in-sim disaster pipeline does,
        # since this file may not import anything beyond numpy and pxr.
        outward = outward * np.asarray(
            orient(world, outward), dtype=np.float64).reshape(-1)[:, None]
    span = float(np.sort(world.max(axis=0) - world.min(axis=0))[1])
    t = min(float(thickness), max_span_frac * span)
    if t <= 1e-6:
        return 0

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
    # back-to-front on the inside and renders inside-out through every cut.
    rev = np.concatenate([np.arange(s + c - 1, s - 1, -1)
                          for s, c in zip(starts, counts)])
    inner_idx = idx[rev] + n_pts

    sa, sb, rim_face = _boundary_edges(counts, idx, _weld(world, weld_tol))
    # Quad (a, a', b', b) — the winding that makes its normal point out of the
    # surface rather than back into the slab.
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
                   orient=None,
                   **kw) -> dict:
    """`solidify` every prim against one shared centre.

    Returns ``{"meshes": n, "faces": n}``. *bounds* is measured off the prims
    when not given; pass the caller's if it already has one, since measuring is
    the expensive part on a big asset.
    """
    b = bounds if bounds is not None else bounds_of(prims)
    centre = None if b is None else b.center
    box = None if b is None else (b.lo, b.hi)
    done = faces = 0
    for prim in prims:
        added = solidify(prim, thickness, ref_centre=centre, ref_box=box,
                         orient=orient, **kw)
        if added:
            done += 1
            faces += added
    return {"meshes": done, "faces": faces}


# ---------------------------------------------------------------------------
# 3. STAGE TWO — THE FAILURE FIELD
#
# This is where a disaster becomes a disaster. Everything else in this module
# is the same for all six types.
#
# A field answers two questions about any point in the scene:
#
#     damage(p)  how much of the material here has failed, 0 to 1
#     ejecta(p)  where the event threw it, as a world-space displacement
#
# and carries one scalar, `char`, for how sooty the event leaves the surfaces.
#
# WHY A FIELD AND NOT A LIST OF EFFECTS
# -------------------------------------
# Because damage composes and effects do not. Two effects applied in sequence
# each have to re-measure the geometry the other moved, and their interaction
# is whatever the ordering happened to produce. Two fields add. A field can be
# sampled anywhere — at a face centroid to decide whether it is consumed, at a
# fragment centroid to decide whether it is released, down a column to decide
# whether it is still supported — and it gives a consistent answer every time,
# because it is a function of position and nothing else.
#
# It is also the only form in which "the same event, harder" is a well-defined
# statement, which is the invariant the whole generator rests on.
#
# HOW THE SHAPES ARE BUILT
# ------------------------
# From four primitives, in normalized building coordinates (`plan_coords`):
# a height profile, a plan-radial or plan-directional weight, a ball around a
# point, and `_grain` — coherent noise with an explicit world-space cell size
# per axis. The last one is what stops every field looking analytic: real
# failure follows the material's weak planes, so an earthquake's grain is
# storey-shaped (wide in plan, thin in z) and a blast front's is small and
# isotropic.
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
                octaves: int = 3, res: int = 32) -> np.ndarray:
    """Scalar noise in roughly [-1, 1] sampled at *points* (N, 3)."""
    total = np.zeros(len(points))
    amp_sum = 0.0
    for o in range(octaves):
        total += (0.5 ** o) * _lattice(points, freq * (2 ** o),
                                       seed + o * 977, res)
        amp_sum += 0.5 ** o
    return total / max(amp_sum, 1e-9)


def _grain(points: np.ndarray, size, seed: int, octaves: int = 3) -> np.ndarray:
    """Coherent noise in [-1, 1] with a world-space feature *size* per axis.

    *size* is a length in metres, or three of them. Anisotropy is the point:
    an earthquake's weak level is a horizontal slab through the whole plan, so
    its grain is metres thick in z and tens of metres wide in x and y, and the
    damaged regions come out as storeys without anything enumerating storeys.
    """
    s = np.asarray(size, dtype=np.float64).reshape(-1)
    if s.size == 1:
        s = np.repeat(s, 3)
    return value_noise(points / np.maximum(s, 1e-6), 1.0, seed,
                       octaves=octaves)


def _ramp(x, lo: float, hi: float) -> np.ndarray:
    """Smoothstep: 0 at or below *lo*, 1 at or above *hi*."""
    t = np.clip((np.asarray(x, dtype=np.float64) - lo) / ((hi - lo) or 1e-9),
                0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _no_ejecta(p: np.ndarray) -> np.ndarray:
    return np.zeros_like(p)


class Failure:
    """A disaster's spatial signature on one building.

    * ``damage(p)`` — (N, 3) world points -> (N,) in [0, 1]. How much of the
      material at each point has lost its structural integrity. Everything
      downstream is a threshold or a weight on this.
    * ``ejecta(p)`` — (N, 3) -> (N, 3) world metres. The displacement the event
      imparts to material that comes free there: radial for a blast,
      downwind-and-up for a storm, zero for a fire or a collapse, which drop
      what they consume where it stood.
    * ``char`` — 0 to 1, how much soot the event leaves on the surfaces.

    Both callables must be pure functions of position, vectorised, and seeded
    only through the closure — `release_mask` samples `damage` several times
    per fragment and would otherwise not be reproducible.
    """

    __slots__ = ("name", "damage", "ejecta", "char")

    def __init__(self, name: str, damage, ejecta=None, char: float = 0.0):
        self.name = str(name)
        self.damage = damage
        self.ejecta = ejecta if ejecta is not None else _no_ejecta
        self.char = float(char)

    def __repr__(self):
        return f"Failure({self.name!r}, char={self.char:.2f})"


def field_earthquake(bounds: Bounds, intensity: float, seed: int = 0,
                     asymmetry: float = 0.55, heading_deg=None,
                     grain_frac: float = 0.45, shear_band: float = 1.0,
                     shear_at: float = 0.0, storey_band: float = 0.0,
                     storey_at: float = 0.0, **_) -> Failure:
    """Ground shaking. Demand is highest at the base and fails a weak storey.

    A building resists lateral ground motion in shear, and the shear it has to
    carry is the accumulated inertia of everything above — so demand is maximal
    at the ground and falls off with height. That single fact is the field's
    height profile, and it is why an earthquake building settles onto its own
    footprint rather than losing its top.

    Where along that profile it actually fails is a property of the structure,
    not of the shaking: real buildings fail at a *level*, because a storey is
    where the columns are and any one of them can be the weak one. So the noise
    that modulates the profile is storey-shaped — the width of the building in
    plan, a storey high in z. Horizontal bands of failed material fall out of
    it, and with them the soft-storey collapse, the mid-storey collapse and the
    total collapse that an earlier version of this module had to enumerate as
    six named modes with a weighted draw between them.

    Nothing is thrown: shaking does not impart a direction.

    ASYMMETRY — WHY HALF A BUILDING COMES DOWN
    ------------------------------------------
    The height profile and the storey grain are both uniform IN PLAN, so on
    their own they can only produce failures that span the whole footprint: the
    building loses a storey, or it loses all of them. That is a real earthquake
    failure but it is not the only one, and it is not the common one on a large
    plan. A wing drops while the rest stands; a corner shears off; one side
    settles onto softer ground and the frame tears down the line between them.

    Three things make that happen in the real event and all of them are
    directional: ground motion arrives as a wave and reaches one side first,
    liquefaction is patchy under a big footprint, and a plan-asymmetric frame
    responds in torsion. So the field carries a bearing and a demand gradient
    along it. *asymmetry* is how much of the demand that gradient owns — 0
    restores the old plan-uniform behaviour exactly, 1 means the sheltered side
    sees no demand at all.

    The grain also shrinks in plan (*grain_frac* of the building radius rather
    than 1.6x it), so the storey bands break up laterally instead of running
    clean through. Together these are what let one half of a tower pancake
    while the other half stands cracked — and the exposed floor plates on the
    tear line are the cross-section you can actually see into.
    """
    i = float(np.clip(intensity, 0.0, 1.0))
    z0, h, r = bounds.base_z, bounds.height, bounds.radius
    storey = float(np.clip(h / 8.0, 2.0, 5.0))       # metres, a plausible one

    # The bearing the demand climbs along. Seeded rather than random so a
    # severity sweep keeps the same building failing on the same side, which
    # is the invariant the whole generator rests on.
    if heading_deg is None:
        ang = float(np.random.default_rng(seed + 977).uniform(0.0, 2.0 * np.pi))
    else:
        ang = math.radians(float(heading_deg))
    ux, uy = math.cos(ang), math.sin(ang)
    cx, cy = float(bounds.center[0]), float(bounds.center[1])
    reach = max(1e-6, float(r))
    a = float(np.clip(asymmetry, 0.0, 1.0))
    gp = max(0.05, float(grain_frac)) * r
    band = max(1e-3, float(shear_band))
    at = float(shear_at)

    def damage(p):
        t = np.clip((p[:, 2] - z0) / h, 0.0, 1.0)
        shear = 1.0 - 0.75 * t
        # -1 on the sheltered side, +1 on the exposed one.
        s = ((p[:, 0] - cx) * ux + (p[:, 1] - cy) * uy) / reach
        # A SHEAR PLANE, NOT A GRADIENT. `band` is how much of the plan the
        # transition takes: 1.0 is the old linear ramp across the whole
        # footprint, small values are a line. A gradient cannot produce the
        # failure people picture when they say "half of it came down", because
        # every intermediate value is *some* damage — the far side still
        # cleared `support` (measured 0.15-0.34 against a 0.12 threshold), so
        # it was cut into fragments too and the boundary read as a smear of
        # rubble rather than as an edge.
        #
        # With a narrow band and `asymmetry` at 1.0 the sheltered side
        # evaluates to ~0, falls under `support`, and is never taken into the
        # soup at all: it survives as its own original, fully textured prim
        # with a clean vertical face where the cut stopped. That face is the
        # cross-section.
        e = np.clip((s - at) / band, -1.0, 1.0)
        side = 0.5 * (e + 1.0)
        side = side * side * (3.0 - 2.0 * side)          # smoothstep
        lean = (1.0 - a) + a * side
        weak = 0.55 + 0.75 * _grain(p, (gp, gp, storey), seed + 11)
        out = 1.95 * i * shear * lean * np.clip(weak, 0.0, None)
        # A STOREY BAND — the soft-storey mechanism, stated rather than hoped
        # for. The height profile plus the storey grain make a band SOMEWHERE
        # likely; they cannot put it at the ground floor on demand, and the
        # rung called `soft_storey` is a claim about exactly that. Everything
        # outside a `storey_band`-tall window centred on `storey_at` (a
        # fraction of the height) is suppressed, so the ground floor is
        # pulverised and the mass above it is left whole to come down on it.
        if storey_band and storey_band > 0.0:
            win = np.exp(-((t - float(storey_at)) /
                           (float(storey_band) * 0.5)) ** 2)
            out = out * win
        return np.clip(out, 0.0, 1.0)

    return Failure("earthquake", damage, char=0.06 * i)


def _field_wind(name: str, bounds: Bounds, intensity: float, seed: int,
                heading_deg, exponent: float, amp: float,
                throw: float, lift: float, char: float = 0.0,
                windward: float = 0.45, floor: float = 0.0) -> Failure:
    """The shared shape of a windstorm: it fails a building from the top down.

    Wind speed rises with height through the boundary layer and the load goes
    as its square, so demand *grows* with height — the exact opposite of an
    earthquake, and the reason the two are distinguishable from a drone at
    100 m even though both leave a broken building. The roof goes first (it is
    the lightest thing, held down only by its own weight, and it is where the
    uplift is), then the top storey's walls behind it.

    The pressure is on the WINDWARD face, so damage is biased to the plan
    hemisphere the storm came from, and everything that comes free travels the
    SAME bearing. That shared bearing is the whole signature: it is what makes
    a tornado track read as a track rather than as a row of bomb sites.

    *windward* is HOW MUCH of the demand that bias owns, on the same reading as
    `field_earthquake`'s `asymmetry`: the leeward face sees ``1 - 2*windward``
    of the load, so 0.5 fails one side of the building and nothing else, and 0
    is plan-uniform. Plan-uniform is the right shape for a direct hit — a house
    under the vortex is loaded from every side in turn and does not have a
    sheltered face — which is why it is a parameter and not the 0.45 that was
    baked in.

    *floor* is how much of the load reaches the GROUND. The height profile is
    ``t ** exponent``, which is zero at the sill for any positive exponent, so
    on its own it cannot fail the bottom course of a wall however hard it is
    driven — and a building hit head-on by a tornado fails at the sill plate
    first, from suction and from debris impact rather than from wind speed.
    Without this the top rung of the tornado ladder came out as a house with
    its roof and upper walls gone standing on an untouched ground floor, which
    is the rung below it. 0 is the pure profile and the default.
    """
    i = float(np.clip(intensity, 0.0, 1.0))
    w = float(np.clip(windward, 0.0, 0.5))
    fl = float(np.clip(floor, 0.0, 1.0))
    rng = np.random.default_rng(seed + 5)
    if heading_deg is None:
        heading_deg = float(rng.uniform(0.0, 360.0))
    ang = math.radians(float(heading_deg))
    dx, dy = math.cos(ang), math.sin(ang)
    r = bounds.radius

    def damage(p):
        u, v, t = plan_coords(bounds, p)
        prof = fl + (1.0 - fl) * np.clip(t, 0.0, 1.0) ** exponent
        # The storm travels along +d, so it strikes the face at -d.
        wind = np.clip((1.0 - w) + w * np.clip(-(u * dx + v * dy), -1.0, 1.0),
                       0.0, None)
        gust = 0.75 + 0.45 * _grain(p, 0.6 * r, seed + 31)
        return np.clip(amp * prof * wind * np.clip(gust, 0.0, None), 0.0, 1.0)

    def ejecta(p):
        d = damage(p)
        out = np.zeros_like(p)
        out[:, 0] = dx * throw * d
        out[:, 1] = dy * throw * d
        out[:, 2] = lift * d
        return out

    return Failure(name, damage, ejecta, char=char)


def field_tornado(bounds: Bounds, intensity: float, seed: int = 0,
                  heading_deg=None, exponent=None, amp=None, throw=None,
                  lift=None, windward: float = 0.45, floor: float = 0.0,
                  **_) -> Failure:
    """A narrow, violent corridor: a deep peel thrown a long way along track.

    Every shape knob defaults to the intensity-derived value it has always had
    and can be overridden by a caller that knows which MECHANISM it is asking
    for — see `disaster/tornado.py`, which is the only one that does. `throw`
    and `lift` are given as fractions of the building (`Bounds.frac`) when
    overridden, so "thrown its own length downwind" means the same on a
    bungalow and on a tower.
    """
    i = float(np.clip(intensity, 0.0, 1.0))
    return _field_wind("tornado", bounds, i, seed, heading_deg,
                       exponent=(2.4 - 1.2 * i) if exponent is None
                       else float(exponent),
                       amp=(0.90 + 0.55 * i) if amp is None else float(amp),
                       throw=bounds.frac(0.22 + 0.50 * i if throw is None
                                         else float(throw)),
                       lift=bounds.frac(0.06 + 0.16 * i if lift is None
                                        else float(lift)),
                       windward=float(windward), floor=float(floor))


def field_hurricane(bounds: Bounds, intensity: float, seed: int = 0,
                    heading_deg=None, **_) -> Failure:
    """Sustained and broad: a shallower peel, carried a shorter way.

    Same mechanism as a tornado and a deliberately different shape — the higher
    exponent keeps the failure in the top tenth or so rather than the top half,
    and the smaller throw leaves the debris against the building instead of a
    hundred metres downwind.
    """
    i = float(np.clip(intensity, 0.0, 1.0))
    return _field_wind("hurricane", bounds, i, seed, heading_deg,
                       exponent=3.6 - 1.6 * i,
                       amp=0.70 + 0.50 * i,
                       throw=bounds.frac(0.10 + 0.25 * i),
                       lift=bounds.frac(0.02 + 0.06 * i))


def field_fire(bounds: Bounds, intensity: float, seed: int = 0,
               **_) -> Failure:
    """A burnt-out shell: the roof and floors are consumed, the walls stand.

    Fire destroys what spans — timber roof decks and floors, in tension, under
    load — long before it drops masonry that is only ever in compression. So
    the field is high at the top and high in the INTERIOR, and it falls away
    toward the envelope. From above, a fire and a tornado leave the same
    silhouette; the difference is that the tornado's material is a hundred
    metres downwind and the fire's is inside the building, which is why this
    field has no ejecta at all.
    """
    i = float(np.clip(intensity, 0.0, 1.0))
    r = bounds.radius
    exponent = 2.4 - 1.1 * i
    amp = 0.75 + 0.50 * i

    def damage(p):
        u, v, t = plan_coords(bounds, p)
        prof = np.clip(t, 0.0, 1.0) ** exponent
        # 1 on the envelope, 0 well inside it.
        shell = _ramp(np.maximum(np.abs(u), np.abs(v)), 0.55, 0.95)
        smoke = 0.80 + 0.35 * _grain(p, 0.5 * r, seed + 41)
        return np.clip(amp * prof * (1.0 - 0.55 * shell)
                       * np.clip(smoke, 0.0, None), 0.0, 1.0)

    return Failure("fire", damage, char=0.45 + 0.50 * i)


def field_flood(bounds: Bounds, intensity: float, seed: int = 0,
                **_) -> Failure:
    """Scour and hydrostatic load at the waterline, and nothing above it.

    Water is heavy and undermines footings, but a masonry wall is a poor thing
    to knock down with it — so this field is deliberately the weakest of the
    six, and in practice it releases nothing: measured, the field peaks around
    0.5 at the top of the severity range, which a *fragment's mean* never
    clears. A flooded building is therefore cut by nothing and thickened by
    nothing (`damage_building` dismisses it before either), and what severity
    changes is the silt line here and the debris and asset swaps elsewhere in
    the disaster stage.

    This says the same thing the old categorical "flood never shatters" did,
    but says it as a *quantity* rather than as an exemption — so if the release
    threshold is ever lowered, or a timber-framed asset pack arrives that should
    wash out, the field is already the right shape and only the numbers move.
    """
    i = float(np.clip(intensity, 0.0, 1.0))
    r = bounds.radius
    depth = 0.08 + 0.30 * i                # waterline, normalized height
    amp = 0.12 + 0.42 * i

    def damage(p):
        _u, _v, t = plan_coords(bounds, p)
        below = _ramp(depth - t, -0.03, 0.06)
        scour = 0.70 + 0.50 * _grain(p, (0.35 * r, 0.35 * r, 0.6 * r),
                                     seed + 53)
        return np.clip(amp * below * np.clip(scour, 0.0, None), 0.0, 1.0)

    return Failure("flood", damage, char=0.10 + 0.20 * i)


#: The disaster axis, as fields. Adding a type is one entry here and one
#: function above — there is nothing else to write, because the pipeline is
#: the same for all of them.
FAILURE_FIELDS = {
    "earthquake": field_earthquake,
    "tornado": field_tornado,
    "hurricane": field_hurricane,
    "fire": field_fire,
    "flood": field_flood,
    "none": None,
}


def sector_mask(bounds: Bounds, heading_deg: float, share: float,
                soft: float = 0.18):
    """``f(p) -> 0..1`` selecting a wedge of the PLAN, *share* of it wide.

    A rung that says "one side collapsed while the other cracked" needs the
    two mechanisms to land on different parts of the footprint, and the plan
    angle about the centre is the cheapest description of "a side" that does
    not depend on the building being a box. `share` of 1.0 is everything;
    `soft` is how much of the wedge is a feathered edge, so two mechanisms
    meeting do not leave a hard seam of undamaged material between them.
    """
    cx, cy, _ = bounds.center
    th = math.radians(float(heading_deg))
    half = math.pi * float(np.clip(share, 0.0, 1.0))

    def mask(p):
        if half >= math.pi - 1e-9:
            return np.ones(len(p))
        a = np.arctan2(p[:, 1] - cy, p[:, 0] - cx) - th
        a = np.abs(np.arctan2(np.sin(a), np.cos(a)))       # to [0, pi]
        e = np.clip((half - a) / max(half * float(soft), 1e-6), 0.0, 1.0)
        return e * e * (3.0 - 2.0 * e)                     # smoothstep

    return mask


def compose(name: str, parts: list, char: float = 0.0) -> Failure:
    """One `Failure` out of several, each restricted to a region of the plan.

    *parts* is ``[(Failure, mask_or_None), ...]`` and the result takes the
    MAXIMUM across them — a piece of building fails in whichever way fails it
    first, and two mechanisms overlapping should not add up to more than
    either could do alone.

    WHY A RUNG IS NOT ONE MECHANISM
    -------------------------------
    A ladder rung is a severity, and a real building at a given severity shows
    several kinds of failure at once: a wing pancakes, the wing beside it loses
    its ground floor and leans onto the wreck, and the far end merely cracks.
    Modelling a rung as a single named mode forces a choice between those and
    makes the ladder non-monotonic in appearance — `soft_storey` came out
    looking worse than `partial_collapse` because the two recipes were tuned
    independently against different fields.

    Composing instead means severity does the one thing it should: it decides
    HOW MANY mechanisms are in play and how severe the worst of them is, and
    the rung name is just the label of the worst one.
    """
    def damage(p):
        out = np.zeros(len(p))
        for fail, mask in parts:
            d = fail.damage(p)
            if mask is not None:
                d = d * mask(p)
            out = np.maximum(out, d)
        return out

    def ejecta(p):
        out = np.zeros((len(p), 3))
        for fail, mask in parts:
            e = fail.ejecta(p)
            if mask is not None:
                e = e * mask(p)[:, None]
            out = np.where(np.abs(e) > np.abs(out), e, out)
        return out

    return Failure(name, damage, ejecta, char=char)


def failure_field(disaster_type: str, bounds: Bounds, intensity: float,
                  seed: int = 0, **kw) -> "Failure | None":
    """The field for *disaster_type*, or None when it does not damage buildings."""
    fn = FAILURE_FIELDS.get(str(disaster_type).lower())
    if fn is None or bounds is None or float(intensity) <= 0.0:
        return None
    return fn(bounds, float(intensity), int(seed), **kw)


def surface_points(prims, n: int, bounds: Bounds, seed: int = 0,
                   z_range=None) -> list:
    """*n* points that actually lie ON the building, in normalized space.

    A point drawn from the bounding box lands in the hollow interior most of
    the time. Face centroids are guaranteed to be on a wall or a roof.
    *z_range* restricts sampling to a normalized height band.
    """
    if n <= 0:
        return []
    pool = [c for c in (face_centroids(p) for p in prims)
            if c is not None and len(c)]
    if not pool:
        return []
    pts = np.concatenate(pool, axis=0)
    u, v, t = plan_coords(bounds, pts)
    norm = np.stack([u, v, t], axis=1)

    if z_range is not None:
        mask = (t >= z_range[0]) & (t <= z_range[1])
        # Never narrow to nothing: an empty pool would mean no damage at all,
        # which is a worse answer than damage in a slightly wrong place.
        if mask.any():
            norm = norm[mask]

    rng = np.random.default_rng(seed)
    pick = rng.choice(len(norm), size=min(int(n), len(norm)), replace=False)
    return [tuple(norm[i]) for i in pick]


def release_column(points: np.ndarray, failure: Failure, bounds: Bounds,
                   samples: int = 6) -> np.ndarray:
    """The worst damage anywhere in the column of material below each point.

    **This is the propagation.** A piece of a building is held up by everything
    between it and the ground, so when that path fails, the piece comes free
    whether or not its own material was touched. Sampling the field straight
    down from each point and taking the maximum is the cheapest honest form of
    that statement, and it is what turns a lost ground floor into a collapsed
    building instead of a floating one.

    Six samples is enough because the fields are smooth in z at the scale that
    matters — the narrowest feature any of them has is a storey (`_grain` in
    `field_earthquake`), and a building is more than six storeys only at the
    top of the library.
    """
    n = max(int(samples), 1)
    out = np.zeros(len(points))
    z0 = bounds.base_z
    for k in range(n):
        f = k / max(n - 1, 1)
        q = points.copy()
        q[:, 2] = z0 + (points[:, 2] - z0) * f
        out = np.maximum(out, failure.damage(q))
    return out


# ---------------------------------------------------------------------------
# 4. STAGE THREE — PROPAGATION BY CELL FRACTURE
#
# The crack network is a Voronoi diagram whose seeds are drawn from the failure
# field, so cells are small where the material was pulverised and large — or
# absent — where it was not. Each cell is the source geometry clipped by the
# perpendicular bisector between its seed and each nearby seed.
#
# WHY THE FRAGMENTS ARE CLOSED
# ----------------------------
# Three things together, and all three are necessary:
#
#   * the input is a solid, because stage one made it one;
#   * every clip is CAPPED (`_cap_fan`), so the cross-section a plane leaves
#     is filled rather than left as an open mouth;
#   * a cell interior to the fractured region is bounded by nothing but those
#     capped planes and the closed surface it was cut from.
#
# The exception is the boundary of the fractured region itself, where faces
# stop being consumed and the cell has an open edge against the geometry that
# stayed behind. That is by construction the low-damage end of the field, which
# is below the release threshold — so the fragments with an open face are
# exactly the ones that never move, and the open face is flush against the mesh
# it was cut from. Everything that is thrown or settled is closed.
#
# WHY THE CLIP IS HAND-WRITTEN
# ----------------------------
# `trimesh.intersections.slice_mesh_plane` does exactly this, and Isaac's Kit
# python already ships trimesh 4.5.1 — but importing it pulls in `shapely`,
# which Isaac does not have, and capping additionally wants a triangulation
# engine it does not have either. Adding both means a Dockerfile change and an
# image rebuild for what is fifty lines of Sutherland-Hodgman.
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

    def centroids(self) -> np.ndarray:
        """Per-triangle centroid, (T, 3) — where the field is sampled."""
        return self.verts[self.faces].mean(axis=1)


def _cap_fan(verts, faces, normal, origin, tol_rel: float = 1e-9):
    """Close a cut: one triangle per in-plane boundary edge, fanned to a point.

    Returns ``(centre_xyz, tris)`` where *tris* indexes ``verts`` except for
    index ``-1``, which stands for the centre the caller has to append. Empty
    when there is nothing to close.

    WHY THIS IS THE LOAD-BEARING PART OF "CLOSED FRAGMENTS"
    -------------------------------------------------------
    Without a cap every fragment is an open shell: measured, 30 of 30 were
    non-watertight, so looking into a fracture surface showed the hollow
    between the two sheets stage one had just built. The rubble had thickness
    from the outside and none at the cut, which is exactly where a viewer
    looks.

    WHY IT IS A FAN AND NOT A POLYGON FILL
    --------------------------------------
    The obvious construction — chain the cut segments into loops, then fill
    each loop — is what this replaces, and it failed on real geometry for three
    separate reasons, all of which are gone here:

      * a vertex lying exactly ON the plane records no crossing, so the chain
        broke and the whole loop was dropped, uncapped;
      * a cross-section with more than one loop through a shared welded point
        chained into the wrong one;
      * a cut through a **solidified** wall has an ANNULAR cross-section, and
        an annulus is not a loop that a centroid fan can fill.

    A fan needs none of it. Every edge of the kept surface that lies in the
    plane is part of the cut's boundary — that is a local test, no chaining —
    and each one gets a triangle back to a single shared centre, wound the
    opposite way round. Every such edge is then used exactly twice with
    opposite orientation, which IS closure, and the fan triangles share their
    spoke edges pairwise. Holes take care of themselves: an inner ring runs the
    other way round, so its fan is negatively oriented and cancels the outer
    fan over the hole, which is the standard signed-area argument for filling a
    polygon with holes from an arbitrary point.

    The cost is coplanar overlapping triangles inside a hollow cross-section.
    They are invisible (they are inside a solid), they cancel in every integral
    anyone computes over the surface, and they are the price of never leaving a
    hole. Finding the boundary edges of the KEPT geometry rather than tracking
    crossings during the clip is also what makes the coplanar cases — a face
    lying in the plane, an edge lying in the plane — behave.

    HOW CLOSED, MEASURED
    --------------------
    On a solidified box cut into ~34 cells across four seeds: 126 of 133
    fragments have no open edge at all, and 111 open edges remain out of
    ~100,000. Uncapped, the same geometry gives **zero** closed fragments and
    1,925 open edges on one building alone.

    On real objaverse houses the rate is 0.05% to 4% of edges left open, and
    most fragments are still perfectly closed. The residue is degenerate
    slivers the clipper drops (`len(poly) < 3`), cuts leaving fewer than three
    boundary edges, and the small amount of non-manifold geometry `solidify`
    could not resolve — that pass takes a raw asset from 5,953 open edges of
    41,891 (14%) down to 191 of 137,719 (0.14%), and what it cannot fix is what
    shows up here. These are slivers, not holes.

    WHAT IT COSTS
    -------------
    The cap is the module's dominant expense, and it scales with how finely the
    asset is tessellated rather than with anything this code chooses: a plane
    through a 95,000-triangle bungalow crosses a great many edges, and each one
    becomes a triangle. Budget roughly **35-50 s and ~1.2M triangles per
    building** at 32 cells on that library, which is time-parity with the
    unclosed implementation this replaces (38.9 s) at ~2.6x its geometry. That
    ratio is the price of solid rubble, and `apply_to_stage`'s per-scene budget
    is what keeps it affordable — see the note there.

    *tol_rel* is what counts as "in the plane", relative to the geometry's own
    scale. It matters in one direction only: a crossing point computed as
    ``a + t (b - a)`` lands within a few ULP of the plane rather than exactly
    on it, so **zero tolerance caps nothing** (0 of 133 closed). The result is
    flat from 1e-12 to 1e-8 and then degrades fast — at 1e-5, vertices that are
    merely *near* the plane get fanned and closure falls to 91 of 133.
    """
    empty = (np.zeros(3), np.zeros((0, 3), dtype=np.int64))
    if len(faces) == 0:
        return empty
    n = np.asarray(normal, dtype=np.float64)
    n = n / max(np.linalg.norm(n), 1e-12)
    d = (verts - np.asarray(origin, dtype=np.float64)) @ n
    tol = tol_rel * max(1.0, float(np.abs(verts).max()))
    on = np.abs(d) <= tol
    if on.sum() < 3:
        return empty

    ea = faces[:, [0, 1, 2]].reshape(-1)
    eb = faces[:, [1, 2, 0]].reshape(-1)
    sel = on[ea] & on[eb]
    a, b = ea[sel], eb[sel]
    if len(a) < 3:
        return empty
    # A collapsed edge is not a boundary; fanning it emits a zero-area sliver.
    live = (np.abs(verts[a] - verts[b]).max(axis=1) > tol)
    a, b = a[live], b[live]
    if len(a) < 3:
        return empty

    centre = verts[np.unique(np.concatenate([a, b]))].mean(axis=0)
    # (centre, b, a) — the segment reversed, which is the winding that makes
    # the cap face +n, i.e. out of the half being kept. See the module note.
    area = np.cross(verts[b] - centre, verts[a] - centre) @ n
    if float(area.sum()) < 0.0:
        a, b = b, a                      # source wound inside out; take it back
    tris = np.stack([np.full(len(a), -1, dtype=np.int64), b, a], axis=1)
    return centre, tris


def _clip_by_plane(verts, faces, normal, origin, uv=None, fmat=None,
                   cap: bool = False):
    """Keep the half of a triangle soup on the negative side of a plane.

    Sutherland-Hodgman per triangle: a triangle straddling the plane is cut to
    a 3- or 4-gon and fan-triangulated. Makes no topology assumptions, so open
    shells and duplicated vertices pass through unharmed.

    *uv* rides along and is interpolated at each crossing with the same
    parameter as the position; *fmat* is per-face and is inherited by every
    triangle a cut face fans into. Both may be None, which is the plain
    positions-only form the tests exercise.

    The result is COMPACTED — vertices no longer referenced are dropped. That
    is not tidiness: a cell is clipped a dozen times in a row, and without
    compaction each pass carries the full original vertex array through every
    later one, so the cost never falls as the cell shrinks.
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

    # THE CUT FACES, ALL AT ONCE.
    #
    # Sutherland-Hodgman is a per-polygon walk, but every polygon here is a
    # TRIANGLE against ONE plane, so the walk is three fixed edge slots and the
    # whole thing is a masked gather. It used to be a Python loop with an
    # `_add` closure appending one vertex at a time: profiled on a single
    # `BG_Building_F`, that closure was 24.1M calls and 127 s of a 428 s
    # fracture, with `_clip_by_plane` itself another 100 s on top — 63% of the
    # time, on one core, while the other 23 sat idle. Cells are clipped by a
    # dozen planes each and every cell pays it.
    #
    # The output order is the part that has to be preserved exactly, because
    # the fan below and the caller's UV interpolation both depend on it: each
    # edge slot k contributes vertex `a` (if inside) and then the crossing
    # point (if the edge crosses), in that order. Interleaving the two (F, 3)
    # masks into one (F, 6) mask reproduces it, and `np.nonzero` on a 2-D mask
    # yields row-major order, i.e. slot order within each face.
    base = len(verts)
    if len(cut):
        tri = faces[cut]                              # (F, 3)
        ia, ib = tri, tri[:, [1, 2, 0]]
        da, db = d[ia], d[ib]                         # (F, 3)

        inside_a = da <= 0.0
        # A SIGN CHANGE IN EITHER DIRECTION IS A CROSSING.
        #
        # This used to be `(da < 0) < (db < 0) or (da > 0) > (db > 0)`, and
        # both of those clauses are true only for outside -> inside — so a
        # straddling triangle recorded ONE crossing where it has two. A
        # triangle with one vertex inside then clipped to two points and was
        # dropped by the `< 3` guard below; one with two inside came out as a
        # triangle instead of a quad. Every cut face in every cell was
        # affected.
        denom = da - db
        cross = ((da < 0.0) != (db < 0.0)) & (np.abs(denom) > 1e-12)
        t = da / np.where(cross, denom, 1.0)          # masked; never 0/0

        va, vb = verts[ia], verts[ib]                 # (F, 3, 3)
        pts_a = va
        pts_x = va + t[:, :, None] * (vb - va)
        cand = np.stack([pts_a, pts_x], axis=2).reshape(len(cut), 6, 3)
        take = np.stack([inside_a, cross], axis=2).reshape(len(cut), 6)

        # A face that clips to fewer than three points contributes NOTHING —
        # not a face and not its vertices. Dropping the rows before the gather
        # is what keeps that true.
        cnt = take.sum(axis=1)
        ok = cnt >= 3
        take[~ok] = False
        cnt = cnt[ok]

        if len(cnt):
            rows, _cols = np.nonzero(take)
            new_v = cand[take]                        # (sum cnt, 3)
            ends = np.cumsum(cnt)
            starts = base + ends - cnt                # first vertex of each

            # Fan: (start, start+k, start+k+1) for k in 1 .. cnt-2.
            ntri = cnt - 2
            rep = np.repeat(starts, ntri)
            k = (np.arange(int(ntri.sum()))
                 - np.repeat(np.cumsum(ntri) - ntri, ntri) + 1)
            nf = np.stack([rep, rep + k, rep + k + 1], axis=1)

            verts = np.concatenate([verts, new_v])
            if uv is not None:
                ua, ub = uv[ia], uv[ib]
                uv_c = np.stack([ua, ua + t[:, :, None] * (ub - ua)],
                                axis=2).reshape(len(cut), 6, uv.shape[1])
                uv = np.concatenate([uv, uv_c[take]])
            keep = np.concatenate([keep, nf]) if len(keep) else nf
            if fmat is not None:
                nm = np.repeat(fmat[cut][ok], ntri)
                keep_mat = (np.concatenate([keep_mat, nm])
                            if keep_mat is not None and len(keep_mat) else nm)

    if cap and len(keep):
        centre, cf = _cap_fan(verts, keep, n, origin)
        if len(cf):
            # `-1` in the fan is the shared centre, appended here so the fan
            # can be built without knowing where the vertex array will end.
            cf = np.where(cf < 0, len(verts), cf)
            verts = np.concatenate([verts, centre[None, :]])
            if uv is not None:
                # The centre sits in the middle of the cut, so the middle of
                # the UVs around it is the only defensible value; the cut face
                # has no authored parameterisation of its own.
                uv = np.concatenate([uv, uv[np.unique(cf[:, 1:])].mean(
                    axis=0, keepdims=True)])
            keep = np.concatenate([keep, cf])
            if fmat is not None:
                # The interior takes the majority material of the piece it
                # closes; there is no separate "cut face" material in the
                # source, and inventing one would not bind to anything.
                mid = (int(np.bincount(keep_mat).argmax())
                       if keep_mat is not None and len(keep_mat) else 0)
                keep_mat = np.concatenate(
                    [keep_mat if keep_mat is not None else np.zeros(0, np.int64),
                     np.full(len(cf), mid, dtype=np.int64)])

    if len(keep) == 0:
        return verts[:0], empty, (uv[:0] if uv is not None else None), keep_mat

    # Compacted with a mask rather than `np.unique`, which sorts: this runs a
    # dozen times per cell on arrays of tens of thousands of faces, and O(V + T)
    # beats O(T log T) by enough to matter at that count.
    seen = np.zeros(len(verts), dtype=bool)
    seen[keep.reshape(-1)] = True
    used = np.nonzero(seen)[0]
    remap = np.empty(len(verts), dtype=np.int64)
    remap[used] = np.arange(len(used))
    return (verts[used], remap[keep],
            uv[used] if uv is not None else None, keep_mat)


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


def _mesh_soup(prims, select=None, masks=None):
    """``(Soup, taken)`` — the prims' geometry as one soup, and what it took.

    *select* maps an (F, 3) array of world-space face centroids to a boolean
    mask; only the faces it accepts enter the soup. *taken* is the per-prim
    mask, so `fracture_to_stage` can delete exactly the geometry it turned into
    fragments and leave the rest of the building standing as its own original,
    fully textured prims.

    That predicate is how the failure field decides what comes apart. With
    ``select=None`` the whole building is consumed.

    *masks* names the faces directly, one boolean array per prim, for the
    caller that has already decided — `fracture_to_stage` uses it to lift an
    orphaned slab out of the source it was left on, where the deciding was
    done by the support graph and not by the field.
    """
    V, UV, F, M, taken = [], [], [], [], []
    table, index = [], {}
    off = 0

    for prim in prims:
        counts, idx = _face_arrays(prim)
        pts = get_points(prim) if counts is not None else None
        if counts is None or not len(pts) or int(idx.max()) >= len(pts):
            taken.append(None)
            continue

        if masks is not None:
            take = np.asarray(masks[len(taken)], dtype=bool)
        elif select is None:
            take = np.ones(len(counts), dtype=bool)
        else:
            cen = (np.add.reduceat(pts[idx], _fv_starts(counts), axis=0)
                   / counts[:, None])
            take = np.asarray(select(cen), dtype=bool)
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


#: How much faster seed density grows than damage does. Above 1 the worst-hit
#: place is unambiguously the finest, which is what makes a blast crater read
#: as pulverised against the coarse cracking further out.
SEED_GAMMA = 1.5

#: How strongly a corner REPELS a seed. See `cornerness` for why the sign is
#: this way round: a Voronoi seed is the middle of a fragment, not a crack, so
#: seeding the corners buries them inside intact cells. At 0.9 a fully turning
#: neighbourhood is a tenth as likely to hold a seed as flat wall, which puts
#: the pair of seeds that straddle an edge onto the flat either side of it and
#: their bisector — the crack — along the edge.
CORNER_AVOID = 0.9

#: Floor under that, so a corner is unlikely to be seeded rather than
#: forbidden. Zero would mean a building whose damaged region is ALL corner —
#: a truss, a railing, a parapet run — draws no seeds at all and never breaks.
CORNER_FLOOR = 0.1


def cornerness(soup: Soup, cell_m: float) -> np.ndarray:
    """Per-face 0..1 measure of how much the surface turns nearby.

    Concrete does not crack at random: it breaks at the geometry that
    concentrates stress, which on a building is corners, reveals, parapets and
    the edges of openings — flat wall between them comes off in slabs. So the
    corners and the edge seams are where the cracks should BE.

    WHICH IS WHY THE SEEDS ARE PUSHED AWAY FROM THEM, NOT TOWARD THEM
    ----------------------------------------------------------------
    This had the sign the wrong way round, and the render is what says so: the
    corners and the roof-to-wall seams were the parts that stayed whole. A
    Voronoi seed is the CENTRE of a fragment; the cracks are the bisectors
    halfway between seeds. Seeding a corner therefore does the opposite of what
    it sounds like — it puts the corner in the middle of a cell, wraps it in
    one intact lump of geometry, and guarantees the one place the building
    should have split is the one place it cannot.

    Turn it around and the mechanism works with the cut rather than against it:
    seeds land on the flat wall either side of an edge, the bisector between
    them falls on the edge, and the building comes apart along its seams. See
    `CORNER_AVOID`.

    The measure is the spread of face normals in a neighbourhood, and the
    neighbourhood is a VOXEL rather than a k-d query: candidates run to
    millions of faces on a real tower and a radius search over them costs more
    than the fracture it is feeding. Binning is O(faces) and the quantity
    wanted — "does the surface turn within a fragment's width of here" — is
    exactly what a fragment-sized voxel answers.

    1 - |mean(n)| over the voxel: 0 where every face agrees (flat wall), and
    rising wherever two or more orientations meet.
    """
    v = soup.verts[soup.faces]
    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-12
    n = np.where(ok[:, None], n / np.where(ok, ln, 1.0)[:, None], 0.0)

    cen = v.mean(axis=1)
    step = max(float(cell_m), 1e-3)
    key = np.floor((cen - cen.min(axis=0)) / step).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)

    tot = np.zeros((inv.max() + 1, 3))
    np.add.at(tot, inv, n)
    cnt = np.bincount(inv, minlength=len(tot)).astype(float)
    mean = np.linalg.norm(tot, axis=1) / np.maximum(cnt, 1.0)
    return np.clip(1.0 - mean, 0.0, 1.0)[inv]


#: HOW A MATERIAL BREAKS, as the shape of the unit it breaks INTO.
#:
#: This is the `block` column of the `Material` table `disaster/earthquake.py`
#: carried before it was deleted, and which `quake.py`'s header lists as debt
#: item 1: "a brick fragment came out a lump, a timber a plank, a steel member
#: a beam". Everything in this module was isotropic until now, so every
#: building broke like the same substance — masonry — and a timber-framed
#: house came apart into rubble that reads as concrete.
#:
#: `wall_m`  how thick the material actually is. A masonry wall is half a
#:           metre; a stud wall is a 90 mm stud with sheathing and siding on
#:           it, so ~0.15 m, and the difference is the single most visible
#:           thing about a pile of house debris.
#: `grain`   the ASPECT of a fragment, as a per-axis multiplier on the cut
#:           spacing. Normalised to unit geometric mean by `grain_vec`, so it
#:           changes a fragment's SHAPE and not its size. (1, 1, 1) is the
#:           isotropic lump this module has always made. Timber is long along
#:           the run of the framing and short across it.
#: `fragment_m` the fragment size the material wants, when a caller has no
#:           opinion of its own.
Material = namedtuple("Material", "wall_m grain fragment_m")

MATERIALS = {
    # Brick, block, concrete: a lump, and a thick one.
    "masonry": Material(0.50, (1.0, 1.0, 1.0), 2.0),
    # Stud framing, sheathing, siding, rafters. Long, thin and flat: a plank.
    # The long axis is the framing run, the short one is across the boards,
    # and the thin one comes free from `wall_m` rather than from the grain —
    # a fragment cut out of a 0.15 m wall is 0.15 m thick whatever the cut
    # does, which is why `grain`'s z is not the plank's thickness.
    "timber": Material(0.15, (2.6, 1.0, 0.55), 1.2),
    # A beam: long in one direction and stubby in the other two.
    "steel": Material(0.30, (3.2, 0.8, 0.8), 2.4),
}

#: What an asset is made of when nothing says otherwise. Masonry, because that
#: is what every number in this module meant before the table existed.
DEFAULT_MATERIAL = "masonry"


def material(name) -> Material:
    """The `Material` for *name*, falling back to `DEFAULT_MATERIAL`.

    Unknown degrades rather than raising, for the reason `levels.NONE_LADDER`
    does: a typo in an asset set should not fail a bake an hour in.
    """
    if isinstance(name, Material):
        return name
    return MATERIALS.get(str(name or DEFAULT_MATERIAL).lower(),
                         MATERIALS[DEFAULT_MATERIAL])


def grain_vec(grain) -> np.ndarray:
    """A grain as a (3,) array normalised to unit geometric mean.

    Normalised because the grain must say what SHAPE a fragment is and not how
    big it is — `fragment_m` is the one place size is decided, and an
    un-normalised grain would silently scale it. With the geometric mean at 1
    the cell volume is unchanged and only the aspect moves.
    """
    g = np.asarray(grain if grain is not None else (1.0, 1.0, 1.0),
                   dtype=np.float64).reshape(3)
    g = np.maximum(np.abs(g), 1e-6)
    return g / float(np.prod(g) ** (1.0 / 3.0))


def fracture_seeds(soup: Soup, failure: Failure, fragment_m: float,
                   seed: int, max_cells: int, spacing_frac: float = 0.7,
                   oversample: int = 8, grain=None) -> np.ndarray:
    """Voronoi seeds, (S, 3) world space, drawn against the failure field.

    THE SIZE OF A FRAGMENT IS THE SPACING BETWEEN CRACKS
    ----------------------------------------------------
    So *fragment_m* — a length in world metres, because a chunk of wall is a
    metre or two across whether it fell off a bungalow or a tower — sets the
    minimum distance between seeds directly, and the number of cells is
    whatever fits. That is the right way round, and the reverse is a trap this
    got wrong first: deriving the spacing from a target *count* means deriving
    it from the damaged surface AREA, and on real geometry that number is
    meaningless. Measured on an objaverse bungalow, the soup came to 17,988 m²
    on a 12 x 11 m footprint — forty-five times its envelope, because the asset
    carries interior walls, doubled shells and detail geometry, and because
    stage one had just doubled all of it. The spacing that fell out was 11.85 m,
    wider than the house, so **two seeds** survived and the "shattered" bungalow
    came apart into two pieces. Area is not a quantity these assets can be
    asked for; a length is.

    TWO MORE THINGS THIS HAS TO GET RIGHT
    -------------------------------------
    * **Seed the material, not the air.** A building is a thin shell inside a
      mostly-empty bounding box, so seeding the box uniformly puts most seeds
      in interior air and leaves one enormous cell holding the bulk of the
      geometry — measured on a brownstone, the largest fragment was 31k of 61k
      triangles, i.e. not shattered at all. Candidates are therefore face
      centroids, which are on the geometry by construction.
    * **Cracks are finer where the damage is worse**, because comminution
      scales with the energy that went into the material: the same wall is
      cracked into slabs at the edge of a blast and pulverised at its centre.
      So the spacing is a *radius per seed* that shrinks with damage, and the
      thinning honours the larger of any pair — the standard variable-radius
      Poisson rule.

    WHEN THE BUDGET IS THE BINDING CONSTRAINT, THE CELLS GET BIGGER
    ---------------------------------------------------------------
    Not fewer. If *max_cells* would be exceeded the whole spacing is scaled up
    and the thinning re-run, so the seeds still **cover** the damaged region at
    a coarser size.

    Truncating the list instead is the trap, and it is worth spelling out
    because the failure looks like a fracture bug rather than a seeding one.
    Candidates were once visited worst-damage first so that a truncated budget
    would at least buy resolution at the epicentre — but an earthquake's field
    saturates at the base, so all 72 seeds packed into one corner of the ground
    floor at 0.7 m spacing and **nothing above it got a seed at all**. The
    unseeded four fifths of the building then fell into whichever peripheral
    cell was nearest, and those cells came out holding 13,000 to 32,000
    triangles each — bigger than the building, overlapping each other, and
    taking 70 seconds to cut. Coverage first, resolution second.
    """
    if len(soup) == 0 or fragment_m <= 0.0 or max_cells < 2:
        return np.zeros((0, 3))
    cen = soup.centroids()
    d = np.clip(failure.damage(cen), 0.0, 1.0)
    # Damage says WHERE the building failed; cornerness says where, within
    # that, the crack actually runs. Multiplying keeps the first as the gate —
    # an undamaged wall is still not seeded — and lets the second push the
    # seeds off the seams so the cut lands on them (see `cornerness`).
    w = (d ** SEED_GAMMA) * np.maximum(
        CORNER_FLOOR, 1.0 - CORNER_AVOID * cornerness(soup, fragment_m))
    nz = np.nonzero(w > 1e-6)[0]
    if len(nz) < 2:
        return np.zeros((0, 3))

    # THE THINNING HAPPENS IN GRAIN SPACE. The Poisson rule below rejects a
    # candidate that is within `radius` of one already kept, and doing that on
    # world distance would space the seeds isotropically — which is exactly
    # what makes every fragment a lump. Measuring the same rejection on
    # `(p - q) / g` lets seeds sit closer together across the grain than along
    # it, and the Voronoi cells that fall out of it are elongated by `g`.
    g = grain_vec(grain)
    rng = np.random.default_rng(int(seed) + 7)
    k = int(min(len(nz), max(int(max_cells) * int(oversample), 2)))
    # Weighted and UNSORTED: the draw already puts more candidates where the
    # damage is, and the per-seed radius below turns that into density. Any
    # further ordering biases coverage instead of resolution.
    pick = rng.choice(nz, size=k, replace=False, p=w[nz] / w[nz].sum())
    base_r = float(fragment_m) * (1.0 - 0.60 * d[pick]) * float(spacing_frac)
    jitter = rng.random((k, 3)) - 0.5

    kept = np.zeros((0, 3))
    scale = 1.0
    for _ in range(6):
        radius = base_r * scale
        # A little jitter off the surface so cuts do not all land exactly on it.
        cand = cen[pick] + jitter * radius[:, None] * 0.5
        pts = np.empty((k, 3))
        rad = np.empty(k)
        n = 0
        for q, r in zip(cand, radius):
            if n:
                lim = np.maximum(rad[:n], r)
                if ((((pts[:n] - q) / g) ** 2).sum(axis=1)
                        < lim * lim).any():
                    continue
            pts[n], rad[n] = q, r
            n += 1
        kept = pts[:n]
        if n <= max_cells:
            break
        # Cell area goes as the square of the spacing, so this lands close to
        # the budget in one step; the loop is for the rounding.
        scale *= math.sqrt(n / float(max_cells)) * 1.02
    return kept[:int(max_cells)]


def _fracture_soup(soup: Soup, seeds: np.ndarray, neighbors: int = 24,
                   min_faces: int = 4, cap: bool = True,
                   with_index: bool = False) -> list:
    """Clip *soup* into one `Soup` per seed. Cells too small to see are dropped.

    Each cell is its candidate geometry clipped by the perpendicular bisector
    between its seed and each other seed, nearest first — which is not cosmetic,
    since the nearest bisector removes the most and every later plane then walks
    a smaller soup.

    WHERE IT STOPS, AND WHY IT IS NOT A FIXED NEIGHBOUR COUNT
    ---------------------------------------------------------
    A cell contained in ``ball(p, R)`` cannot be touched by the bisector with
    any seed further than ``2R`` away — the plane sits at half that distance,
    outside the ball. Since the seeds are visited in order of distance, the
    first one that fails the test ends the loop, and the result is the exact
    Voronoi cell rather than an approximation of it.

    This began as "clip by the twelve nearest seeds", and a fixed count is
    wrong in both directions on a field-driven seeding, where density varies by
    design. Too few and the cell is never bounded: measured on an objaverse
    bungalow, 72 cells came to **1.9 million triangles** against a 95,000
    triangle source, because each peripheral cell kept most of the building.
    Too many and every cell pays for planes that cannot reach it. The radius
    test needs no number at all — *neighbors* survives only as a ceiling.

    Cells with fewer than *min_faces* triangles are dropped: a cell whose seed
    landed in interior air clips down to nothing, and emitting those as prims
    costs draw calls for invisible geometry.

    DO NOT PRE-CULL THE FACE SET. IT IS THE OBVIOUS OPTIMISATION AND IT IS UNSOUND
    -----------------------------------------------------------------------------
    Every cell starts from the whole soup, which is O(cells x faces) and is the
    dominant cost of the whole module. The tempting fix is to first drop, per
    cell, the faces lying wholly beyond one of its bisectors — exact as a
    *set* operation, since such a face cannot touch the cell, and it does cut
    the time roughly in half.

    It also destroys the capping completely: measured on a solidified box,
    closure went from **62 of 70 fragments to 0 of 70**, with or without the
    early exit, and whether 16 planes were culled or all 48.

    The reason is that the cap is incremental. `_cap_fan` closes a cut by
    fanning the edges that lie in the plane, which requires the cross-section
    at that moment to be a closed loop — and it is one only because the
    geometry being cut is itself closed. Culling opens it: whole faces vanish,
    leaving a jagged boundary that lies on no plane, so the first cut's
    cross-section is a truncated chain, its fan has unpaired spokes, and the
    later planes do not put them back. The closure of the *final* cell depends
    on the closure of every intermediate one.

    So the geometry each plane sees has to stay closed all the way through, and
    the only sound speed-ups are ones that do not remove faces: clipping
    nearest-first (below), stopping as soon as no remaining seed can reach the
    cell (below), and making the clip itself cheaper (`_clip_by_plane`).
    """
    if len(soup) == 0 or len(seeds) < 2:
        return []
    d2 = np.sum((seeds[:, None, :] - seeds[None, :, :]) ** 2, axis=-1)
    np.fill_diagonal(d2, np.inf)
    order = np.argsort(d2, axis=1)[:, :max(1, int(neighbors))]

    out = []
    for i, p in enumerate(seeds):
        cv, cf, cuv, cm = soup.verts, soup.faces, soup.uv, soup.fmat
        for j in order[i]:
            if len(cf) == 0:
                break
            q = seeds[j]
            gap = float(np.linalg.norm(q - p))
            if gap < 1e-9:
                continue
            # `_clip_by_plane` compacts, so every vertex here is still in play.
            if gap > 2.0 * float(np.linalg.norm(cv - p, axis=1).max()):
                break
            cv, cf, cuv, cm = _clip_by_plane(cv, cf, q - p, (p + q) * 0.5,
                                             cuv, cm, cap=cap)
        if len(cf) >= min_faces:
            frag = Soup(cv, cuv, cf, cm, soup.mats)
            out.append((i, frag) if with_index else frag)
    return out


def _kd_groups(seeds: np.ndarray, n: int) -> np.ndarray:
    """Split *seeds* into *n* spatially compact groups. Returns labels (S,).

    Repeated median splits along the widest axis — a k-d tree stopped early.
    Median rather than a grid because the seeds are field-driven and therefore
    wildly non-uniform: a grid over an earthquake's seeding puts almost every
    seed in the ground-floor cells and leaves the rest empty, which is the
    imbalance the whole hierarchy exists to avoid.
    """
    lab = np.zeros(len(seeds), dtype=np.int64)
    k = 1
    while k < int(n):
        for g in range(k):
            m = np.nonzero(lab == g)[0]
            if len(m) < 2:
                continue
            pts = seeds[m]
            ax = int(np.argmax(pts.max(axis=0) - pts.min(axis=0)))
            hi = m[pts[:, ax] > np.median(pts[:, ax])]
            lab[hi] = g + k
        k *= 2
    return lab


def _fracture_hier(soup: Soup, seeds: np.ndarray, neighbors: int = 24,
                   min_faces: int = 4, cap: bool = True, branch: int = 8,
                   leaf: int = 12, flat_below: int = 64,
                   _depth: int = 0) -> list:
    """`_fracture_soup`, but in O(faces x branch x depth) instead of O(faces x cells).

    WHY THIS EXISTS
    ---------------
    `_fracture_soup` starts every cell from the WHOLE soup — see the note there
    on why pre-culling faces is unsound — so its cost is the product of the two
    numbers that both want to be large: cells and faces. That product is what
    capped rubble size to a fraction of the building. On the 91 m tower,
    1.5M faces at 140 cells already took ~300 s, and rubble of a fixed
    ~2 m size on that tower is thousands of cells, i.e. hours.

    THE DECOMPOSITION, AND WHY IT KEEPS THE CAPS VALID
    ---------------------------------------------------
    Cut the soup by a handful of COARSE seeds first. Each resulting block is a
    closed solid — that is exactly the guarantee `_fracture_soup` already
    provides — so each block can be handed back to the same cutter as if it
    were the original soup, with only the fine seeds that live inside it. The
    capping is incremental and needs its input closed; a block is closed, so
    the recursion is sound where face-culling was not.

    Cells straddling a block boundary are split by it. That is a real
    difference from the flat cutter and it is the price: a boundary reads as
    one more crack, at a scale coarser than the rubble, which is what a
    building does anyway.

    TWO STOPPING RULES, NOT ONE
    ---------------------------
    A block is a coarse cell, so its cross-section is more complicated than a
    fragment's and `_cap_fan` declines more often on it — measured on the test
    box, level-1 blocks come out only 25% closed. That does not propagate as
    badly as it sounds (the fine cuts inside an open block still cap), but it
    is not free either: at 30 cells the hierarchy closed 53% against the flat
    cutter's 70%. At 200 cells it closed 84% against 82% and ran 2.4x faster,
    because the blocks are then a small fraction of the work and the fine cells
    dominate.

    So there are two thresholds. `flat_below` is the entry gate: a fracture
    small enough that the hierarchy cannot pay for itself takes the exact flat
    path it always did. `leaf` is where the RECURSION stops once it has been
    entered, and it is much smaller — the leaf cost is `leaf x faces_in_leaf`
    summed over leaves, i.e. `leaf x faces`, so it is the term that dominates a
    deep tree and wants to be small.

    Each level costs `branch x faces`, and there are `log_branch(cells)`
    levels, so the total is ~`branch x log(cells) x faces` — 3,000 cells on
    1.5M faces goes from 3000xF to about 32xF.
    """
    n = len(seeds)
    stop = max(int(flat_below), 2) if _depth == 0 else max(int(leaf), 2)
    if n <= stop or len(soup) == 0:
        return _fracture_soup(soup, seeds, neighbors, min_faces, cap=cap)

    b = min(int(branch), n)
    lab = _kd_groups(seeds, b)
    centres = np.array([seeds[lab == g].mean(axis=0)
                        for g in range(b) if (lab == g).any()])
    if len(centres) < 2:
        return _fracture_soup(soup, seeds, neighbors, min_faces, cap=cap)

    # REASSIGN TO THE NEAREST CENTRE. A k-d group's own centroid is not
    # necessarily the closest one to every seed in it, and a seed sitting
    # outside the block it was handed to contributes no cell at all — its
    # region silently merges into a neighbour and one fragment comes out
    # block-sized. Nearest-centre labelling is what makes "every seed is
    # inside its own block" true rather than usually true.
    lab = np.argmin(((seeds[:, None, :] - centres[None, :, :]) ** 2)
                    .sum(axis=-1), axis=1)

    out = []
    # `min_faces=1` here: a block is scaffolding, not a fragment, and dropping
    # a thin one throws away every fine cell inside it.
    for g, block in _fracture_soup(soup, centres, neighbors, 1, cap=cap,
                                   with_index=True):
        mine = seeds[lab == g]
        if len(mine) < 2:
            if len(block) >= min_faces:
                out.append(block)
            continue
        out.extend(_fracture_hier(block, mine, neighbors, min_faces, cap,
                                  branch, leaf, flat_below, _depth + 1))
    return out


def fracture(prims, bounds: Bounds, failure: Failure, seed: int = 0,
             fragment_m: float = 2.5, max_cells: int = 32,
             support: float = 0.12, neighbors: int = 24,
             min_faces: int = 4, cap: bool = True, grain=None) -> list:
    """Shatter *prims* along *failure*. Returns a list of `Soup` fragments.

    The read-only half of `fracture_to_stage` — nothing is authored and the
    source is left alone, which is what makes it testable without a stage.
    """
    soup, _taken = _mesh_soup(
        prims, lambda cen: failure.damage(cen) >= support)
    seeds = fracture_seeds(soup, failure, fragment_m, seed, max_cells,
                           grain=grain)
    g = grain_vec(grain)
    if np.allclose(g, 1.0):
        return _fracture_hier(soup, seeds, neighbors, min_faces, cap=cap)
    scaled = Soup(soup.verts / g, soup.uv, soup.faces, soup.fmat, soup.mats)
    return [Soup(f.verts * g, f.uv, f.faces, f.fmat, f.mats)
            for f in _fracture_hier(scaled, seeds / g, neighbors, min_faces,
                                    cap=cap)]


def _bind_materials(stage, mesh, frag: Soup) -> None:
    """Give a fragment back every material its faces came from.

    Together with the `st` primvar authored alongside it, this is what stops
    shattered geometry rendering as untextured white — which read as confetti
    rather than as a building coming apart.

    The majority material is bound on the prim and each remaining one gets a
    `GeomSubset`. Binding only the majority is tempting and wrong: it holds up
    on a tower cut into 50 cells, where a cell really is one material, but a
    house cut into 40 spans roof, wall and trim in nearly every cell — the
    damage gallery showed a whole street of them rendering in one flat
    off-white. Subsets carry no geometry, so the cost is a prim each and
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
            _vt_int(faces), "materialBind")
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)


# ---------------------------------------------------------------------------
# SUPPORT — what is still standing, and what was left hanging in the air
#
# Two kinds of thing can float, and only one of them was ever checked:
#
#   * a FRAGMENT with nothing left under it. `release_column` asks whether the
#     FIELD failed the material below a piece, which is a statement about the
#     damage and not about what is still there: the field below a fragment can
#     be pristine while every piece that was actually holding it up has been
#     thrown clear.
#   * the RETAINED remainder — the faces whose damage never reached `support`,
#     so they were never cut at all. On any BANDED mechanism that is most of
#     the building: `soft_storey` fails a window at the base and leaves the
#     twenty storeys above it untouched, and untouched is not the same as
#     supported. Those faces used to enter the graph as unconditional ROOTS,
#     i.e. as ground, which is exactly backwards — and is why a roof plate hung
#     in the air over the rubble on every heavy rung of the ladder.
#
# So both are entities in ONE graph, and the graph is resolved on GEOMETRY
# rather than on the field: an entity stands if it reaches the ground, or if it
# rests on something that does. Everything else falls, and the retained parts
# that fall are cut out of their source prims and handed to the settle as
# rigid bodies — a soft storey going means the block above it comes down whole,
# which is the one thing that rung is supposed to look like.
#
# WHAT THIS GRAPH DOES NOT ASK is whether the load path it found could carry
# the load. It is a reachability test: connected to the ground, therefore
# standing. A real structure also has to survive the MOMENT, and the case where
# the difference shows is a cantilever — `shear_off` on BG_Building_C takes a
# bite out of the middle of the tower and leaves the mass above the bite
# hanging off the core beside it, which is a legal load path and not a building
# anyone would leave standing. Fixing it wants a span test (free length against
# depth, per component) rather than a better contact test, so it is written
# down here rather than bolted onto the traversal.
#
# CONTACT IS VOXEL OCCUPANCY, NOT AABB OVERLAP. A box is a usable stand-in for
# a 2 m Voronoi chunk and a hopeless one for the L-shaped top twenty storeys of
# a tower: that box spans the whole plan, overlaps every fragment in the storey
# below it, and one surviving chunk anywhere underneath declares the block
# supported. Surfaces are sampled at a fixed areal density and hashed into a
# grid instead, so "rests on" is asked of the material and not of its extent.
# ---------------------------------------------------------------------------

#: Grid edge for the support graph, in metres. Small against the 2 m rubble
#: the pipeline makes, so resting on something means resting on it rather than
#: being somewhere above it; large enough that a 96 m tower is a few hundred
#: cells a side.
SUPPORT_CELL_M = 0.5

#: Ceiling on surface samples per entity set, so a 300k-face tower cannot turn
#: the support pass into the expensive stage. Over it the density is scaled
#: down uniformly — coarser, not partial.
SUPPORT_SAMPLE_CAP = 1_500_000


def _surface_samples(verts, faces, step, seed=0, cap=SUPPORT_SAMPLE_CAP):
    """Points covering a triangle soup at ~one per *step* squared, plus verts.

    Vertices alone are not a sampling. `subdivide` floors the edge length at
    4 m, so one wall triangle can span sixty support cells and occupy none of
    them between its corners — which reads as a hole in the material and frees
    whatever was resting there. Area-proportional barycentric samples make the
    density a property of the GRID rather than of the tessellation.
    """
    if not len(faces):
        return np.zeros((0, 3)), np.zeros(0, dtype=np.int64)
    v = verts[faces]
    area = 0.5 * np.linalg.norm(np.cross(v[:, 1] - v[:, 0],
                                         v[:, 2] - v[:, 0]), axis=1)
    per = max(float(step) * float(step) * 0.5, 1e-9)
    n = np.clip(np.ceil(area / per), 1, 4096).astype(np.int64)
    tot = int(n.sum())
    if cap and tot > cap:
        n = np.maximum(1, (n * (float(cap) / float(tot))).astype(np.int64))
    idx = np.repeat(np.arange(len(faces), dtype=np.int64), n)
    rng = np.random.default_rng(int(seed) + 17)
    r = rng.random((len(idx), 2))
    su = np.sqrt(r[:, 0])
    b = np.stack([1.0 - su, su * (1.0 - r[:, 1]), su * r[:, 1]], axis=1)
    pts = (v[idx] * b[:, :, None]).sum(axis=1)
    own = np.repeat(np.arange(len(faces), dtype=np.int64), 3)
    return (np.vstack([pts, v.reshape(-1, 3)]),
            np.concatenate([idx, own]))


class _Grid:
    """A fixed voxel grid over a point set, with points packed to int64 keys."""

    __slots__ = ("org", "step", "shape")

    def __init__(self, lo, hi, step):
        self.org = np.asarray(lo, dtype=float) - float(step)
        self.step = float(step)
        span = np.asarray(hi, dtype=float) + float(step) - self.org
        self.shape = np.maximum(2, np.ceil(span / self.step) + 2).astype(
            np.int64)

    def ijk(self, pts):
        v = np.floor((np.asarray(pts, dtype=float) - self.org)
                     / self.step).astype(np.int64)
        return np.clip(v, 0, self.shape - 1)

    def pack(self, ijk):
        return ((ijk[:, 0] * self.shape[1] + ijk[:, 1]) * self.shape[2]
                + ijk[:, 2])

    def keys(self, pts):
        return self.pack(self.ijk(pts))

    def unpack(self, keys):
        iz = keys % self.shape[2]
        rest = keys // self.shape[2]
        return np.stack([rest // self.shape[1], rest % self.shape[1], iz],
                        axis=1)

    def shift(self, keys, offsets):
        """Every *keys* moved by each (dx, dy, dz), out-of-range dropped."""
        base = self.unpack(keys)
        out = []
        for d in offsets:
            q = base + np.asarray(d, dtype=np.int64)
            ok = ((q >= 0) & (q < self.shape)).all(axis=1)
            if ok.any():
                out.append(self.pack(q[ok]))
        return np.unique(np.concatenate(out)) if out else np.zeros(
            0, dtype=np.int64)


#: The 13 lexicographically-positive offsets of the 26-neighbourhood: one per
#: undirected voxel adjacency, so connectivity is found without doing each
#: edge twice.
_ADJ26 = [(dx, dy, dz)
          for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
          if (dx, dy, dz) > (0, 0, 0)]

#: Where a supporter may sit relative to what it holds up: one cell below it,
#: allowing a cell of lateral slop, or in the same cell. Not above, and not a
#: whole cell to the side at the SAME level — a fragment is held up by what is
#: beneath it, not by what it stands beside, and the difference is the whole
#: reason a wall pinned between two others still falls when the floor goes.
_HOLD = [(0, 0, 0)] + [(dx, dy, 1) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]


def _voxel_components(grid, keys):
    """26-connected component label per input key.

    Hooking plus pointer-jumping rather than a union-find loop: the retained
    remainder of a real tower is a few hundred thousand voxels and this stays
    in numpy, where a Python loop over the same edges does not.
    """
    uniq = np.unique(keys)
    if not len(uniq):
        return np.zeros(0, dtype=np.int64), 0
    base = grid.unpack(uniq)
    ei, ej = [], []
    for d in _ADJ26:
        q = base + np.asarray(d, dtype=np.int64)
        ok = ((q >= 0) & (q < grid.shape)).all(axis=1)
        if not ok.any():
            continue
        nk = grid.pack(q[ok])
        j = np.searchsorted(uniq, nk)
        j = np.clip(j, 0, len(uniq) - 1)
        hit = uniq[j] == nk
        if hit.any():
            ei.append(np.nonzero(ok)[0][hit])
            ej.append(j[hit])
    lab = np.arange(len(uniq), dtype=np.int64)
    if ei:
        a, b = np.concatenate(ei), np.concatenate(ej)
        for _ in range(64):
            prev = lab
            nxt = lab.copy()
            np.minimum.at(nxt, a, lab[b])
            np.minimum.at(nxt, b, lab[a])
            lab = nxt[nxt]                       # pointer jumping
            if np.array_equal(lab, prev):
                break
    _, lab = np.unique(lab, return_inverse=True)
    return lab[np.searchsorted(uniq, keys)], int(lab.max()) + 1


def _standing(grid, keys, ent, low, ground, n_ent, forced_free):
    """Fixpoint of "reaches the ground". Bool over entities.

    *keys* / *ent* / *low* are per-SAMPLE: which cell it is in, which entity it
    belongs to, and whether it is in the bottom of that entity's own extent —
    the last is what keeps a same-cell contact from meaning "held up by the
    wall next to me" everywhere up a facade.
    """
    ks, es, lw = keys, ent, low
    standing = np.asarray(ground, dtype=bool) & ~forced_free
    for _ in range(int(n_ent) + 1):
        if not standing.any():
            break
        # Every cell a standing entity could be holding something up in.
        held = grid.shift(np.unique(ks[standing[es]]), _HOLD)
        if not len(held):
            break
        j = np.clip(np.searchsorted(held, ks), 0, len(held) - 1)
        cand = np.unique(es[(held[j] == ks) & lw])
        nxt = standing.copy()
        nxt[cand] = True
        nxt &= ~forced_free
        if nxt.sum() == standing.sum():
            break
        standing = nxt
    return standing


def unsupported(frags: list, freed: np.ndarray, bounds: Bounds,
                retained=None, tol: float = 0.35,
                cell_m: float = SUPPORT_CELL_M, seed: int = 0):
    """Free everything with no load path to the ground. See the note above.

    *retained* is ``(points, sample_face, n_faces)`` for the geometry the cut
    left on the source prims — surface samples, which face each came off, and
    how many retained faces there are. It is grouped into connected components
    and each component stands or falls on its own.

    Returns ``(freed, orphan_face)``: the fragment release mask, extended with
    everything the graph could not hold up, and an int array over the retained
    faces giving the id of the orphaned component each belongs to, or -1 for a
    face that is still standing.
    """
    n = len(frags)
    ret_pts = ret_face = None
    n_ret_faces = 0
    if retained is not None:
        ret_pts, ret_face, n_ret_faces = retained
    if not n:
        return freed, np.full(int(n_ret_faces), -1, dtype=np.int64)

    freed = np.asarray(freed, dtype=bool).copy()
    step = max(float(cell_m), 1e-3)

    # --- sample every entity's surface --------------------------------------
    pts, ent = [], []
    for i, f in enumerate(frags):
        p, _ = _surface_samples(f.verts, f.faces, step, seed=seed + i)
        if len(p):
            pts.append(p)
            ent.append(np.full(len(p), i, dtype=np.int64))
    if not pts:
        return freed, np.full(int(n_ret_faces), -1, dtype=np.int64)

    all_pts = np.vstack(pts + ([ret_pts] if ret_pts is not None
                               and len(ret_pts) else []))
    lo = np.minimum(all_pts.min(axis=0), [0.0, 0.0, float(bounds.base_z)])
    grid = _Grid(lo, all_pts.max(axis=0), step)

    # --- the retained remainder, split into pieces that stand or fall -------
    comp = np.zeros(0, dtype=np.int64)
    n_comp = 0
    if ret_pts is not None and len(ret_pts):
        comp, n_comp = _voxel_components(grid, grid.keys(ret_pts))
        pts.append(ret_pts)
        ent.append(n + comp)
    pts = np.vstack(pts)
    ent = np.concatenate(ent)
    n_ent = n + n_comp

    # --- who is where, and which samples are at an entity's own base --------
    keys = grid.keys(pts)
    z = pts[:, 2]
    base = np.full(n_ent, np.inf)
    np.minimum.at(base, ent, z)
    top = np.full(n_ent, -np.inf)
    np.maximum.at(top, ent, z)
    band = np.maximum(step, 0.25 * (top - base))
    low = z <= base[ent] + band[ent]
    ground = base <= float(bounds.base_z) + float(tol)

    forced = np.zeros(n_ent, dtype=bool)
    forced[:n] = freed                      # the cut already let these go
    standing = _standing(grid, keys, ent, low, ground, n_ent, forced)

    freed |= ~standing[:n]
    orphan_face = np.full(int(n_ret_faces), -1, dtype=np.int64)
    if n_comp and ret_face is not None and len(ret_face):
        fell = ~standing[n:]
        # A face belongs to the component its samples landed in; the first is
        # as good as any, since a face is far smaller than a component.
        face_comp = np.full(int(n_ret_faces), -1, dtype=np.int64)
        face_comp[ret_face[::-1]] = comp[::-1]
        keep = (face_comp >= 0) & fell[np.maximum(face_comp, 0)]
        orphan_face[keep] = face_comp[keep]
    return freed, orphan_face


def fracture_to_stage(stage, root_prim, bounds: Bounds, failure: Failure,
                      seed: int = 0, fragment_m: float = 2.5,
                      max_cells: int = 32,
                      support: float = 0.12, release: float = 0.50,
                      collapse: float = 0.80, neighbors: int = 24,
                      min_faces: int = 4, cap: bool = True,
                      shrink: float = 1.0, grain=None,
                      gap_m: float = 0.0) -> dict:
    """Break *root_prim* along *failure* and author the fragments.

    Returns ``{"paths": [...], "loose": [...], "cells": n}``; `loose` is the
    subset of `paths` that came free of the structure, and the only fragments
    the caller may hand to the settle pass.

    THE THREE THRESHOLDS
    --------------------
    *support* is where material is damaged enough to be worth cutting at all.
    Faces below it stay on the source prims, with their own materials, UVs and
    subsets untouched — so a hurricane that fails the top tenth of a tower pays
    for the top tenth and leaves the other nine as they were.

    *release* is where a fragment's own material has failed enough that it is
    no longer attached to anything. *collapse* is the higher bar the material
    *underneath* it has to clear before it comes free for lack of support (see
    `release_column`). Two thresholds rather than one because the two are not
    the same event: a roof that has failed is off the building, but a wall does
    not fall just because the storey below it is cracked — it falls when that
    storey is gone.

    If nothing anywhere reaches *release*, nothing is authored at all. An
    anchored fragment is cut where the geometry was and is geometrically
    identical to the source it replaced, so a building that cracks without
    losing anything should be left alone rather than re-authored into pieces
    that look exactly like it did.

    WHAT IS CONSUMED, AND WHERE THE PIECES GO
    -----------------------------------------
    Only the faces the soup took are removed from the source; with the whole
    building consumed its meshes end up empty, at which point `delete_faces`
    deactivates them (they live in a referenced layer, where `RemovePrim` does
    not compose — the trap `prune_prims` documents).

    Released fragments are translated RIGIDLY by `failure.ejecta` at their own
    centroid. Rigidly, because a fragment is a rigid body — the earlier
    per-vertex, height-scaled throw sheared every piece as it moved it.

    `slabs` is the subset of `loose` that was never cut: whole pieces of the
    building that lost their load path. See the SUPPORT note above, and
    `quake.collapse`, which is where they stop being treated as rubble.
    """
    prims = mesh_prims(root_prim)
    if not prims:
        return {"paths": [], "loose": [], "cells": 0}

    soup, taken = _mesh_soup(
        prims, lambda cen: failure.damage(cen) >= support)
    if len(soup) == 0:
        return {"paths": [], "loose": [], "cells": 0}

    # Nothing would come free, so nothing is worth authoring — see above.
    if float(np.max(failure.damage(soup.centroids()))) < release:
        return {"paths": [], "loose": [], "cells": 0}

    seeds = fracture_seeds(soup, failure, fragment_m, seed, max_cells,
                           grain=grain)
    if len(seeds) < 2:
        return {"paths": [], "loose": [], "cells": 0}
    # THE WHOLE CUT RUNS IN GRAIN SPACE, and then comes back. Dividing every
    # vertex and seed by `g` turns the anisotropic partition we want into the
    # isotropic one this cutter already makes, and a diagonal affine map takes
    # planes to planes — so every bisector is still a plane, every closed
    # fragment is still closed, and `_cap_fan`, `_clip_by_plane`, `_kd_groups`
    # and the radius-based early exit all keep working untouched. That is why
    # the anisotropy is a change of coordinates here rather than a rewrite of
    # the cutter: there is no version of "clip by an ellipsoid" to write.
    g = grain_vec(grain)
    if not np.allclose(g, 1.0):
        soup = Soup(soup.verts / g, soup.uv, soup.faces, soup.fmat, soup.mats)
        frags = _fracture_hier(soup, seeds / g, neighbors, min_faces, cap=cap)
        frags = [Soup(f.verts * g, f.uv, f.faces, f.fmat, f.mats)
                 for f in frags]
        soup = Soup(soup.verts * g, soup.uv, soup.faces, soup.fmat, soup.mats)
    else:
        frags = _fracture_hier(soup, seeds, neighbors, min_faces, cap=cap)
    if not frags:
        # Nothing came apart. Leave the source intact — deleting the damaged
        # region with no fragments to replace it would just punch a hole.
        return {"paths": [], "loose": [], "cells": 0}

    # --- release: which fragments are still part of the building ------------
    cen = np.array([f.centroids().mean(axis=0) for f in frags])
    own = np.array([float(np.mean(failure.damage(f.centroids())))
                    for f in frags])
    below = release_column(cen, failure, bounds)
    freed = (own >= release) | (below >= collapse)
    # Nothing may be left hanging in the air — see the SUPPORT note above. The
    # faces the soup did not take are still on the building, and the graph has
    # to decide for each connected piece of them whether it is holding the
    # building up or being held up by it.
    ret_pts, ret_face, ret_of, n_ret = _retained_samples(
        prims, taken, max(float(SUPPORT_CELL_M), 1e-3), seed=seed)
    freed, orphan = unsupported(
        frags, freed, bounds,
        retained=(ret_pts, ret_face, n_ret) if n_ret else None, seed=seed)
    slabs = _orphan_masks(orphan, ret_of, prims, taken)
    if not freed.any() and not slabs:
        # Nothing moved, so nothing is worth authoring: an anchored fragment is
        # cut where the geometry was and is indistinguishable from the source
        # it would replace. Checked here rather than only on the field's
        # maximum above because a fragment is judged on its MEAN damage, which
        # a single hot face can clear without any piece of the building doing.
        return {"paths": [], "loose": [], "cells": 0}
    push = failure.ejecta(cen)

    scope = root_prim.GetPath().AppendChild("fragments")
    UsdGeom.Scope.Define(stage, scope)
    root_inv = np.linalg.inv(_world_matrix(root_prim))
    paths, loose = [], []
    for i, fr in enumerate(frags):
        v = fr.verts
        if freed[i] and np.abs(push[i]).max() > 1e-9:
            v = v + push[i]
        # SHRINK ABOUT THE CENTROID, LOOSE PIECES ONLY.
        #
        # Voronoi cells share exact boundaries, so a freshly cut building is a
        # set of rigid bodies all touching along every face. PhysX reads that
        # as pervasive penetration and resolves it with a separating impulse —
        # capped here to a shove by `MaxDepenetrationVelocity`, which means the
        # pile does not explode but also never comes apart: measured on the
        # 96 m tower, 30 of 30 bodies were still grinding after 1500 steps.
        # A hairline inset gives the solver a legal starting configuration.
        # `vtk_fracture` has had this since the beginning (`shrink=0.97`);
        # this pipeline never did, and that is the whole difference between a
        # collapse and a cracked building that sags.
        #
        # ANCHORED FRAGMENTS ARE NOT INSET. They are static geometry standing
        # in for the source they replaced, so shrinking them opens visible
        # seams across a wall that never moved. Only what is about to be
        # simulated needs the gap.
        # AN ABSOLUTE GAP, NOT A PROPORTIONAL ONE, once the fragments stop
        # being lumps. `shrink` takes a fixed fraction off every axis, so the
        # gap it opens is proportional to that axis's own extent — fine while
        # a fragment is roughly cubic, and wrong the moment `grain` makes it a
        # plank: a 4.8 m x 0.9 m board inset by 3% gets 7 cm of clearance
        # along its length and 1.4 cm across its thin axis, which is where its
        # neighbours actually are. Measured, that is enough to keep the pile
        # interpenetrating: `walls_breached` went from -1.1 m (masonry) to
        # +2.5 m (timber) on the same house and seed, i.e. the plank pile
        # launching where the lump pile settled.
        #
        # `gap_m` insets every axis by the SAME distance instead, which is
        # what the solver needs and what a fraction can only approximate on a
        # shape it was tuned for. Off by default so `shrink` keeps its
        # long-standing meaning for the isotropic path.
        if freed[i] and gap_m and gap_m > 0.0:
            c = v.mean(axis=0)
            d = v - c
            ext = np.abs(d).max(axis=0)
            # FLOORED AT 0.75, and the floor is the whole lesson. Without it
            # the clamp sat at 0.2, so a fragment cut from a 0.15 m stud wall
            # lost 80% of its thin axis and came out a 3 cm sliver — and a
            # convex hull that thin is degenerate enough that PhysX resolves
            # its contacts with impulses that throw the pile. Measured on the
            # Colbert at `walls_breached`: +5.0 m with the proportional inset,
            # +12.8 m with an unfloored absolute one. The gap is meant to give
            # the solver a legal start, not to whittle the geometry away.
            f = np.clip(1.0 - float(gap_m) / np.maximum(ext, 1e-9), 0.75, 1.0)
            v = c + d * f
        elif freed[i] and shrink != 1.0:
            c = v.mean(axis=0)
            v = c + (v - c) * float(shrink)
        p = _author_soup(stage, scope.AppendChild(f"frag_{i:03d}"),
                         fr, v, root_inv)
        paths.append(p)
        if freed[i]:
            loose.append(p)

    # --- the orphans: whole uncut pieces with nothing under them any more ----
    #
    # Authored as ONE body each and NOT inset, because a slab is not rubble:
    # it is the part of the building that never failed, and it should land
    # looking like the part of the building that never failed.
    slab_paths = []
    for k, masks in enumerate(slabs):
        soup, _ = _mesh_soup(prims, masks=masks)
        if not len(soup):
            continue
        p = _author_soup(stage, scope.AppendChild(f"slab_{k:03d}"),
                         soup, soup.verts, root_inv)
        paths.append(p)
        loose.append(p)
        slab_paths.append(p)

    # Only now: the soup had to be built from the faces before they went.
    # `deactivate` is refused for the prim the fragments were authored under —
    # see `delete_faces`; retiring it would take them with it.
    gone = [np.zeros(0, dtype=bool)] * len(prims)
    for masks in slabs:
        for i, m in enumerate(masks):
            gone[i] = m if not len(gone[i]) else (gone[i] | m)
    for i, (prim, take) in enumerate(zip(prims, taken)):
        drop = None if take is None else take.copy()
        if len(gone[i]):
            drop = gone[i] if drop is None else (drop | gone[i])
        if drop is not None and drop.any():
            delete_faces(prim, ~drop, deactivate=prim != root_prim)

    # WHAT IS STILL STANDING, BY PATH. The retained remainder the graph kept is
    # the part of the building that did not fall, and the pile has to land ON
    # it. It is not in `paths` — it was never cut, so it has no fragment — and
    # leaving it out of the settle is why a sheared-off wing fell straight
    # through the half that was supposed to be holding: measured on
    # BG_Building_C's `shear_off`, the settle ran with 825 rigid bodies and TWO
    # static colliders, then spent its whole 4,000-step budget with 566 bodies
    # still moving.
    # THE ROOT PRIM IS IN THIS LIST when the root is itself the mesh, which the
    # merged AEC assets are. That is only safe because `settle._apply_collider`
    # skips prims that already carry a rigid body: the fragments are authored
    # as the root's own children, so a subtree walk reaches every one of them.
    standing = []
    for prim in prims:
        p = prim.GetPrim() if hasattr(prim, "GetPrim") else prim
        if not p.IsActive():
            continue
        fc = face_centroids(p)
        if fc is not None and len(fc):
            standing.append(str(p.GetPath()))
    return {"paths": paths, "loose": loose, "cells": len(frags),
            "slabs": slab_paths, "standing": standing}


def _author_soup(stage, path, soup: Soup, verts: np.ndarray, root_inv):
    """One `Soup` as a `UsdGeom.Mesh` under *path*. Returns the path as a str.

    Points are written in the ROOT's local space, not in world space.
    Everything upstream works in world space — that is this module's whole
    convention — but a fragment is a child of the building prim and inherits
    its transform, and `apply_placements` gives that prim the placement's
    translate, rotate and scale. Authoring world points under it applies all
    three a second time: on the centimetre-authored AEC packs (`scale: 0.01`)
    that puts the debris a hundred times too small and a long way from the
    building it came off.
    """
    mesh = UsdGeom.Mesh.Define(stage, path)
    local = verts @ root_inv[:3, :3] + root_inv[3, :3]
    mesh.GetPointsAttr().Set(_vec3f(local))
    mesh.GetFaceVertexCountsAttr().Set(_vt_int(np.full(len(soup.faces), 3)))
    mesh.GetFaceVertexIndicesAttr().Set(
        _vt_int(soup.faces.reshape(-1)))
    # See the note in `fill_interior`: a fragment is looked at from both sides
    # once the building is open.
    mesh.CreateDoubleSidedAttr(True)
    # The soup is unwelded, so UVs are already one per vertex.
    if soup.uv is not None and len(soup.uv) == len(verts):
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(_vec2f(soup.uv))
    _bind_materials(stage, mesh, soup)
    return str(path)


def _retained_samples(prims, taken, step, seed=0):
    """Surface samples of everything the cut LEFT on the source prims.

    ``(pts, sample_face, face_owner, n_faces)`` where *sample_face* indexes a
    flat list of every retained face across every prim and *face_owner* is the
    ``(prim, face)`` that list came from — which is what lets an orphaned
    component be cut back out of the prims it was spread over.
    """
    pts, sfa, own = [], [], []
    base = 0
    for pi, (prim, take) in enumerate(zip(prims, taken)):
        counts, idx = _face_arrays(prim)
        if counts is None:
            continue
        pt = get_points(prim)
        if not len(pt) or int(idx.max()) >= len(pt):
            continue
        keep = np.ones(len(counts), dtype=bool) if take is None else ~take
        if not keep.any():
            continue
        slots, src = _triangulate(counts)
        if not len(slots):
            continue
        sel = keep[src]
        slots, src = slots[sel], src[sel]
        if not len(slots):
            continue
        # Retained faces are numbered densely, so a sample can name one
        # without carrying the prim's own face numbering around.
        num = np.full(len(counts), -1, dtype=np.int64)
        num[keep] = base + np.arange(int(keep.sum()), dtype=np.int64)
        v = pt[idx[slots.reshape(-1)]]
        f = np.arange(len(slots) * 3, dtype=np.int64).reshape(-1, 3)
        p, tri = _surface_samples(v, f, step, seed=seed + pi)
        if len(p):
            pts.append(p)
            sfa.append(num[src[tri]])
        own.append(np.stack([np.full(int(keep.sum()), pi, dtype=np.int64),
                             np.nonzero(keep)[0]], axis=1))
        base += int(keep.sum())
    if not pts:
        return None, None, np.zeros((0, 2), dtype=np.int64), 0
    return (np.vstack(pts), np.concatenate(sfa), np.vstack(own), base)


def _orphan_masks(orphan, ret_of, prims, taken):
    """Per-component, per-prim face masks for the retained pieces that fell."""
    if orphan is None or not len(orphan) or not len(ret_of):
        return []
    sizes = []
    for prim in prims:
        counts, _ = _face_arrays(prim)
        sizes.append(0 if counts is None else len(counts))
    out = []
    for c in np.unique(orphan[orphan >= 0]):
        rows = ret_of[np.nonzero(orphan == c)[0]]
        masks = []
        for pi, n in enumerate(sizes):
            m = np.zeros(n, dtype=bool)
            mine = rows[rows[:, 0] == pi]
            if len(mine) and n:
                m[mine[:, 1]] = True
            masks.append(m)
        if any(m.any() for m in masks):
            out.append(masks)
    return out

# ---------------------------------------------------------------------------
# soot — the one thing damage does to materials rather than to geometry
#
# Kept out of the four stages because it is not structural: it changes what the
# surface looks like, not what is there. It is still driven by the same field,
# so a blast blackens the wall it went off against and not the far side of the
# building, and a fire chars everything because its field reaches everything.
# ---------------------------------------------------------------------------

# Albedo inputs worth darkening, by shading model. UsdPreviewSurface is what
# the Objaverse→USD conversion writes; the `diffuse_*` names are OmniPBR/MDL,
# which is what the Nucleus and AEC packs bind inside Isaac.
_ALBEDO_INPUTS = ("diffuseColor", "baseColor",
                  "diffuse_tint", "diffuse_color_constant", "albedo_add")
_ROUGHNESS_INPUTS = ("roughness", "reflection_roughness_constant")


def scorch(prims, failure: Failure, samples: int = 256) -> int:
    """Darken each bound material in proportion to how hard the field hit it.

    Returns how many materials were touched. Authored once per *material*: the
    material lives in the referenced layer and is shared between prims, so a
    second visit would compound the tint.

    WHY SETTING THE INPUT IS NOT ENOUGH
    -----------------------------------
    This used to be `inp.Set(darker_colour)` on `diffuseColor` and it did
    **nothing at all** on almost every real asset. Building assets drive albedo
    from an image texture, which in USD means `diffuseColor` is *connected* to
    a `UsdUVTexture`, and a connection beats a value — the authored constant is
    simply never consulted. The USD way out needs no new node: `UsdUVTexture`
    has an `inputs:scale` that multiplies whatever it sampled, so soot goes
    there. Only the RGB components are scaled — the fourth is alpha, and a
    texture whose `.a` drives opacity would otherwise turn the building
    transparent as it charred.

    ROUGHNESS IS ONLY TOUCHED WHEN IT IS NOT CONNECTED, which is not timidity:
    these assets pack roughness and metallic into two channels of ONE texture,
    so scaling that texture to roughen the surface would silently rewrite
    metallic too.
    """
    w_max = float(np.clip(failure.char, 0.0, 1.0))
    if w_max <= 0.0:
        return 0

    weights: dict = {}
    mats: dict = {}
    for prim in prims:
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        v = get_points(prim)
        if not len(v):
            continue
        if len(v) > samples:
            step = max(len(v) // samples, 1)
            v = v[::step]
        w = w_max * float(np.max(np.clip(failure.damage(v), 0.0, 1.0)))
        if w <= 0.01:
            continue
        key = str(mat.GetPrim().GetPath())
        mats[key] = mat
        weights[key] = max(weights.get(key, 0.0), w)

    for key, mat in mats.items():
        for shader in _surface_shaders(mat):
            _soot(shader, weights[key])
    return len(mats)


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
                shader = material.ComputeSurfaceSource([ctx] if ctx else [])[0]
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
# THE DRIVER — the four stages on one building, and on a whole scene
# ---------------------------------------------------------------------------

#: Fracture knobs a config `mesh_damage.fracture` block may override. Every one
#: has a defensible default, so a preset never has to supply any of them; what
#: it does have to supply is the budget (`max_buildings`), because that is a
#: property of the scene rather than of the disaster.
FRACTURE_KEYS = ("fragment_m", "max_cells", "support", "release",
                 "collapse", "neighbors", "min_faces", "cap", "shrink",
                 "grain", "gap_m")


# ---------------------------------------------------------------------------
# THE BUDGET — how thick a wall is and how many cells to spend on one building
#
# Neither of these is a statement about a disaster: a wall is half a metre
# thick whether it is shaken or blown over, and the cell count is set by the
# rubble size the script asked for and by what PhysX can afford. They live here
# so every per-disaster script gets the same answer.
#
# NOTE: `disaster/quake.py` still carries its own identical copies, written
# before there was a second script. They should be deleted in favour of these.
# ---------------------------------------------------------------------------

#: Metres of wall on a REFERENCE-sized building. A wall is about half a metre
#: thick on a bungalow and on a tower alike, which is why `solidify` takes an
#: absolute length — but the two do not read the same. On a 12 m house 0.5 m is
#: masonry; on a 96 m tower it is 0.5% of the span, a foil skin inside a 10 m
#: Voronoi cell, and the fragments come out looking hollow. So the wall grows
#: with the building, slowly (square root, not linear — a skyscraper has
#: thicker walls than a house, not eight times thicker) and to a ceiling.
WALL_M = 0.5
WALL_M_MAX = 2.0

#: Building radius `WALL_M` is quoted for.
REF_RADIUS_M = 12.0

#: Compute ceiling on cells for ONE building. Not a design number: the cut is
#: near-linear in cells since `_fracture_hier`, but the SETTLE is not — every
#: cell is a rigid body with a cooked collider and PhysX cost climbs with the
#: contacts between them. Set from measurement; see `tools/quake_preview.py`.
MAX_CELLS_CAP = 1200


def wall_for(root_prim, wall_m=WALL_M, ref_m=REF_RADIUS_M, cap=WALL_M_MAX):
    """Wall thickness for this building's size. See `WALL_M`."""
    prims = mesh_prims(root_prim)
    b = bounds_of(prims) if prims else None
    if b is None or not b.radius:
        return float(wall_m)
    scale = math.sqrt(max(1.0, float(b.radius) / float(ref_m)))
    return float(min(float(cap), float(wall_m) * scale))


def cells_for(root_prim, fragment_m=2.0, cap=None):
    """How many cells this building needs to break into *fragment_m* rubble.

    Derived from the building's own envelope area — facades plus roof, which is
    the surface a Voronoi cut actually tiles — divided by the area one fragment
    covers. So the COUNT varies with the building and the SIZE does not, which
    is the way round rubble works.

    `cap` is a compute ceiling, not a design choice. When it binds,
    `fracture_seeds` widens the spacing and the rubble gets coarser — the
    failure this parameterisation exists to make visible rather than silent, so
    callers report it.
    """
    prims = mesh_prims(root_prim)
    b = bounds_of(prims) if prims else None
    top = int(MAX_CELLS_CAP if cap is None else cap)
    if b is None:
        return top
    w, d, h = (float(x) for x in b.dims)
    area = 2.0 * (w + d) * h + w * d
    want = int(round(area / max(float(fragment_m) ** 2, 1e-6)))
    return int(max(24, min(top, want)))


def damage_building(stage, root_prim, disaster_type: str, intensity: float,
                    seed: int = 0, wall_m: float = 0.5, solid: bool = False,
                    max_edge_m: float = 4.0, sub_max_points: int = 400_000,
                    heading_deg=None, solid_kw: dict = None,
                    field_fn=None, field_kw: dict = None, interior: bool = True,
                    interior_kw: dict = None,
                    **fracture_kw) -> dict:
    """Run the four stages on one building. Returns a small report.

    ``{"field": name|None, "thickened": n_meshes, "cells": n,
       "paths": [...], "loose": [...]}``

    ORDER IS THE ARGUMENT, NOT AN IMPLEMENTATION DETAIL
    ---------------------------------------------------
    * **subdivide first**, because everything after it works at the resolution
      it is handed: `solidify` extrudes a 22 m triangle into a 22 m slab, and a
      cell cut from two enormous triangles is two enormous triangles.
    * **field before thickness**, so the building can be dismissed before the
      expensive stage rather than after it. Thickening exists to serve the
      fracture; on a building the field will never break — the far edge of an
      epicentre falloff, or anything a flood touched — it would double the
      point count of the geometry for nothing, on a scene that has OOM-killed
      at 89M points. The probe is the field's own maximum over the faces,
      which is an upper bound on any fragment's mean, so it can only dismiss
      buildings that were going to produce nothing.
    * **the field is rebuilt after thickening**, because it is anchored to the
      bounds and `solidify` moves geometry. Inward, so they barely change —
      but "barely" is not "not at all" and a field that disagrees with the
      geometry it is cutting is a bug waiting for an asset with a thin roof.
    * **scorch before fracture**, because fracture deactivates the source prims
      it consumes and there would be nothing left to read a material binding
      off. The fragments bind the same materials, so they inherit the soot.
    """
    out = {"field": None, "thickened": 0, "cells": 0, "paths": [], "loose": [],
           "dismissed": False}
    deinstance(root_prim)
    prims = mesh_prims(root_prim)
    if not prims:
        return out
    b = bounds_of(prims)
    if b is None:
        return out

    # --- resolution floor ---------------------------------------------------
    if max_edge_m and max_edge_m > 0.0:
        for mp in prims:
            subdivide(mp, float(max_edge_m), max_points=int(sub_max_points))
        b = bounds_of(prims) or b

    # --- stage two: the failure field ---------------------------------------
    kw = dict(field_kw or {})
    if heading_deg is not None:
        kw["heading_deg"] = float(heading_deg)

    # `field_fn` is a FACTORY, not a field: the field is anchored to the
    # bounds and `solidify` moves them, so it has to be rebuildable. That is
    # the same reason the built-in path calls `failure_field` twice below.
    def _field(bb):
        if field_fn is not None:
            return field_fn(bb)
        return failure_field(disaster_type, bb, intensity, seed, **kw)

    fail = _field(b)
    if fail is None:
        return out
    out["field"] = fail.name

    scorch(prims, fail)

    release = float(fracture_kw.get("release", 0.50))
    peak = 0.0
    for mp in prims:
        cen = face_centroids(mp)
        if cen is not None and len(cen):
            peak = max(peak, float(np.max(fail.damage(cen))))
    if peak < release:
        out["dismissed"] = True         # nothing here will ever come free
        return out

    # --- stage one: thickness -----------------------------------------------
    if wall_m and wall_m > 0.0 and not solid:
        got = solidify_prims(prims, float(wall_m), bounds=b,
                             **(solid_kw or {}))
        out["thickened"] = got["meshes"]
        if got["meshes"]:
            b = bounds_of(prims) or b
            fail = _field(b)

    # --- stage one and a half: an inside ------------------------------------
    # After thickening (so the slabs are not extruded a second time) and
    # before the field is used to cut, so the cells see slabs and columns as
    # ordinary geometry. Bounds do not change: everything added is inside.
    # ON FOR EVERY DISASTER, not just the earthquake path. A building opened
    # by wind or fire is looked into exactly as much as one opened by shaking,
    # and a hollow shell reads as paper through any of those holes.
    # `fill_interior` declines on anything under its own size floor, so a shed
    # still costs nothing.
    if interior:
        out["interior"] = fill_interior(stage, root_prim, b,
                                        **(interior_kw or {}))

    # --- stage three: propagation -------------------------------------------
    cut = fracture_to_stage(stage, root_prim, b, fail, seed=seed,
                            **fracture_kw)
    out["paths"] = cut["paths"]
    out["loose"] = cut["loose"]
    out["cells"] = cut["cells"]
    # The uncut pieces that lost their load path — see the SUPPORT note. The
    # caller needs them by name: they are the one part of `loose` that must
    # not be consumed and must not get a convex-hull collider.
    out["slabs"] = cut.get("slabs") or []
    # The uncut material the support graph KEPT — static colliders for the
    # settle, so the pile lands on the part of the building still there.
    out["standing"] = cut.get("standing") or []
    # Stage four — settling the loose fragments — is PhysX's, and it belongs to
    # the caller: `generate_scene` gives each loose path a placement marked
    # `settle: True`, which is how every other approximated-pose prop reaches
    # `scene_prep.settle_rigid_props`.
    return out


def _heading_of(dis: dict):
    """The storm's bearing in degrees, or None where the type has none.

    A windstorm's damage all faces the same way and its debris all travels the
    same way; that shared bearing is the tornado signature, and the difference
    between a track and a bomb site.

    It has to be looked for in TWO places. `compile_tornado` writes
    `heading_deg` inside the `field` block — the corridor's bearing is a
    property of the field — and nothing ever wrote it at the top level, so
    reading only there silently returned None for every scene ever generated
    and each building picked its own random bearing.
    """
    for src in (dis, dis.get("field") or {}):
        v = src.get("heading_deg")
        if v is not None:
            return float(v)
    return None


#: ONE SCRIPT PER DISASTER, resolved lazily by name.
#:
#: `mesh_damage` is the API — fields, solidify, interior, the cutter, the
#: support graph — and a per-disaster script is what says what that disaster
#: MEANS: which mechanisms are in play at which rung, how big the rubble is,
#: how much is pulverised. The registry is here rather than an import because
#: the dependency runs one way: a script imports the API, never the reverse,
#: and hard-importing `quake` from this module would be a cycle.
#:
#: Anything not listed falls through to `damage_building` at a continuous
#: intensity, which is where every disaster started and where the ones without
#: a script of their own still are.
DAMAGE_SCRIPTS = {"earthquake": ("disaster.quake", "at_level"),
                  "tornado": ("disaster.tornado", "at_level")}


def damage_one(stage, prim, dtype: str, intensity: float, seed: int = 0,
               settle_it: bool = False, **kw) -> dict:
    """Damage one building the way *dtype* means it. The single entry point.

    Routes to the disaster's own script when it has one (`DAMAGE_SCRIPTS`),
    quantising the continuous local intensity to a LADDER RUNG on the way —
    a script is written in terms of kinds of damage, not magnitudes, and
    `levels.level_at` is the one place that conversion lives.
    """
    entry = DAMAGE_SCRIPTS.get(str(dtype).lower())
    if entry is None:
        # No script, so there is no ladder to rescale and `material` means
        # only the fragment SHAPE. Translated here rather than forwarded:
        # `damage_building` passes what it does not name to
        # `fracture_to_stage`, which has no `material` and would raise.
        mat = material(kw.pop("material", None))
        kw.setdefault("grain", mat.grain)
        kw.setdefault("wall_m", mat.wall_m)
        return damage_building(stage, prim, dtype, float(intensity),
                               seed=int(seed), **kw)

    import importlib

    from . import levels as L
    mod = importlib.import_module(entry[0])
    level = L.level_at(dtype, float(intensity), L.STRUCTURE).name
    return getattr(mod, entry[1])(stage, prim, level, seed=int(seed),
                                  settle_it=bool(settle_it), **kw)


def apply_to_stage(stage, config: dict, placements: list) -> dict:
    """Damage the buildings the asset-swap route did not ruin.

    Runs after `apply_placements`, because it needs real prims: the damage
    authors overrides on `points` and on the topology of geometry inside a
    referenced layer, and there is nothing to override until the reference is
    composed.

    `disaster_stage.apply_to_buildings` marks the candidates with
    `_mesh_damage`, the local field intensity at that building — every building
    it decided to damage rather than swap, plus every one it wanted to destroy
    but had no ruin of the right footprint for. Buildings that *did* get a ruin
    swapped in already look ruined and are left alone.

    Returns ``{"tally": ..., "fragments": [...], "loose": [...]}``.

    THE BUDGET, AND WHAT IT COSTS
    -----------------------------
    Thickening doubles a mesh's point count and every fragment is a prim that
    the settle pass gives a collider and a rigid body. At ~32 cells a building,
    doing this to all 167 buildings a severity-0.6 detailed urban marks
    would author ~5,300 prims on a scene that has OOM-killed at 89M points. So
    the pipeline is a budget rather than something every marked building gets.

    **The budget is worth measuring before raising.** On the suburban objaverse
    library one damaged building costs roughly 35-50 s and ~1.2M triangles of
    fragments — the closed-fragment cap is what dominates both, and neither
    scales with anything this module chooses (see `_cap_fan`). At the 50-80
    `max_buildings` the compiled presets currently carry, a severe scene is
    therefore tens of minutes and tens of millions of triangles. If a scene is
    slow or tight on memory, `max_buildings` is the knob, then `max_cells`.

    It is spent on the WORST-HIT buildings, not on whichever ones the placement
    list reached first: placement order is essentially packing order, so a
    first-come rule hands the budget to one corner of the map and leaves the
    epicentre — the one place a viewer looks — untouched. Buildings outside the
    budget keep the tilt-and-sink `disaster_stage` already gave them.

    The per-building seed keys off the placement index, so re-ranking changes
    *which* buildings are damaged and never *how* one of them is.
    """
    from scene_generator import _stage, solid_assets

    dis = _stage(config, "disaster")
    dtype = str(config.get("disaster_type")
                or config.get("locale_disaster_type") or "").lower()
    if not dtype:
        # Compiled configs do not carry the type name at the top level — take
        # it from the disaster block, falling back to the generic ground
        # failure, which is the right read for "something knocked it down".
        dtype = str(dis.get("type") or "earthquake").lower()
    if FAILURE_FIELDS.get(dtype) is None:
        return {"tally": {}, "fragments": [], "loose": [],
                "slabs": []}

    seed = int(config.get("seed", 0))
    heading = _heading_of(dis)

    cfg = dis.get("mesh_damage") or {}
    fcfg = cfg.get("fracture") or {}
    tcfg = cfg.get("thickness") or {}
    scfg = cfg.get("subdivide") or {}

    if not cfg.get("enabled", True) or not fcfg.get("enabled", True):
        return {"tally": {}, "fragments": [], "loose": [],
                "slabs": []}

    # One budget for one pipeline. `fracture.max_buildings` and
    # `thickness.max_buildings` are the spellings the compiler emits today and
    # are read as aliases; they were always set to the same number, because
    # thickening a building nobody was going to break is wasted memory and
    # breaking one that was left as paper is the artifact stage one exists to
    # prevent.
    budget = int(cfg.get("max_buildings")
                 or fcfg.get("max_buildings")
                 or tcfg.get("max_buildings") or 40)

    # AN ENVIRONMENT CAP, because the budget and the damage PATH are set in
    # different places and a preset's budget is written for the path it
    # expects. `urban_quake_showcase` carries `max_buildings: 60` because it
    # was written around a baked archetype library, where the budget costs
    # nothing; running it with `SCENE_ARCHETYPES=0` hands all fourteen of its
    # wrecked buildings to live mesh damage instead, and the scene that was
    # meant to load in minutes takes hours. This bounds that without editing a
    # preset — a knob for looking at a scene, not a scene parameter, which is
    # why it lives in the environment and not in the config.
    env_budget = os.environ.get("SCENE_DAMAGE_BUDGET", "").strip()
    if env_budget:
        try:
            budget = max(0, int(env_budget))
        except ValueError:
            print(f"[mesh_damage] SCENE_DAMAGE_BUDGET={env_budget!r} is not an "
                  f"integer — keeping the config's {budget}")

    # WHAT THE BUILDINGS ARE MADE OF. A property of the PLACE, not of the
    # event — a suburb is timber frame whether it was shaken or blown down —
    # so `compile_disaster` sets it from the locale and both the wall
    # thickness and the fragment shape follow from it. Masonry when nothing
    # says otherwise, which is what every number in this module meant before
    # `MATERIALS` existed.
    mat = material(cfg.get("material"))
    wall_m = (float(tcfg.get("wall_m", mat.wall_m))
              if tcfg.get("enabled", True) else 0.0)
    solid_kw = {k: tcfg[k] for k in ("max_points", "max_span_frac", "weld_tol")
                if k in tcfg}
    # Assets whose art already has material in the walls. Declared in the asset
    # set, not detected — see `scene_generator.solid_assets`.
    solid_usds = solid_assets(config)

    sub_edge = (float(scfg.get("max_edge_m", 4.0))
                if scfg.get("enabled", True) else 0.0)
    sub_max_points = int(scfg.get("max_points", 400_000))
    frac_kw = {k: fcfg[k] for k in FRACTURE_KEYS if k in fcfg}

    cands = [(i, p) for i, p in enumerate(placements)
             if p.get("_mesh_damage") and p.get("prim_path")]
    rank = sorted(cands, key=lambda ip: (-float(ip[1]["_mesh_damage"]), ip[0]))
    if cands:
        print(f"[mesh_damage] {len(cands)} building(s) marked for damage, "
              f"budget {budget}"
              + (f" (SCENE_DAMAGE_BUDGET={env_budget})" if env_budget else "")
              + (f" — {len(cands) - budget} keep the tilt-and-sink stand-in"
                 if len(cands) > budget else ""), flush=True)

    tally: dict = {}
    fragments: list = []
    loose: list = []
    slabs: list = []
    # Live damage is tens of seconds A BUILDING and prints nothing until the
    # whole pass is done, so a scene that is working looks identical to one
    # that has hung. One line per building, before and after.
    work = rank[:budget]
    for n, (i, p) in enumerate(work, 1):
        prim = stage.GetPrimAtPath(p["prim_path"])
        if not prim or not prim.IsValid():
            continue
        t_one = time.time()
        print(f"[mesh_damage] {n}/{len(work)} "
              f"{os.path.basename(str(p.get('usd', '')))} "
              f"{p.get('_damage_level', '?')} "
              f"(intensity {float(p['_mesh_damage']):.2f}) ...", flush=True)
        if str(dtype).lower() in DAMAGE_SCRIPTS:
            # The script owns the DEFAULTS for fragment size, thresholds and
            # consume; the config still overrides them where it says so, or a
            # compiled preset's `mesh_damage.fracture` block would silently
            # stop meaning anything the moment a disaster grew a script.
            # `settle_it=False`: the scene settles every loose prim once, in
            # `scene_prep`, not per building.
            # `heading_deg` reaches a script too. A windstorm's whole
            # signature is that every building on the track fails on the same
            # side and throws its debris the same way; without the scene's
            # bearing each one picks its own at random and the track reads as
            # a row of unrelated bomb sites. Scripts that have no bearing
            # (`quake`) accept and ignore it.
            got = damage_one(stage, prim, dtype, float(p["_mesh_damage"]),
                             seed=seed + i * 31, solid=p.get("usd") in solid_usds,
                             max_edge_m=sub_edge, solid_kw=solid_kw,
                             heading_deg=heading, wall_m=wall_m,
                             material=cfg.get("material") or DEFAULT_MATERIAL,
                             **frac_kw)
        else:
            got = damage_building(
                stage, prim, dtype, float(p["_mesh_damage"]), seed=seed + i * 31,
                wall_m=wall_m, solid=p.get("usd") in solid_usds,
                max_edge_m=sub_edge, sub_max_points=sub_max_points,
                heading_deg=heading, solid_kw=solid_kw,
                # No ladder here, so the material reaches the cut as the only
                # thing it can say without one: the SHAPE of a fragment.
                **{"grain": mat.grain, **frac_kw})
        print(f"[mesh_damage] {n}/{len(work)} done in "
              f"{time.time() - t_one:.0f}s  "
              f"cells={got.get('cells', 0)} loose={len(got.get('loose') or ())}",
              flush=True)
        if got["field"] is None:
            continue
        tally[got["field"]] = tally.get(got["field"], 0) + 1
        if got["dismissed"]:
            # The field never reaches the release threshold anywhere on this
            # building, so it was dropped before the expensive stages.
            tally["unbroken"] = tally.get("unbroken", 0) + 1
        elif got["thickened"]:
            tally["thickened"] = tally.get("thickened", 0) + 1
        elif wall_m > 0.0:
            tally["already_solid"] = tally.get("already_solid", 0) + 1
        if got["paths"]:
            fragments.extend(got["paths"])
            loose.extend(got["loose"])
            slabs.extend(got.get("slabs") or ())
            tally["shattered"] = tally.get("shattered", 0) + 1
            tally["fragments"] = tally.get("fragments", 0) + len(got["paths"])
            tally["loose"] = tally.get("loose", 0) + len(got["loose"])
            if got.get("slabs"):
                tally["slabs"] = tally.get("slabs", 0) + len(got["slabs"])

    if tally:
        print("[mesh_damage] "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    # `slabs` is a subset of `loose`: uncut pieces that lost their load path.
    # Reported separately because they are not debris — nothing threw them —
    # so anything measuring where an event put the material has to leave them
    # out. See `fracture_to_stage` and the SUPPORT note.
    return {"tally": tally, "fragments": fragments, "loose": loose,
            "slabs": slabs}


# ---------------------------------------------------------------------------
# 1b. STAGE ONE AND A HALF — GIVE A BIG BUILDING AN INSIDE
#
# `solidify` extrudes the outer SURFACE inward and stops, because that is all
# the surface knows about. On a house that is enough: the shell is the
# building. On a tower it is not, and the measurement says so — after
# thickening, a fragment of `SM_MERGED_BP_MBuilding02` was 11% material by
# bounding box and none of the 40 sampled were watertight. The asset is a
# facade with nothing behind it, so a Voronoi cell 10 m across holds one
# curved 0.5 m panel and a great deal of air, and a cross-section through it
# shows an empty box instead of stacked floors.
#
# Real towers are mostly floor. Slabs every storey, a column grid carrying
# them, and the shell hung off the outside — so that is what this adds, as
# ordinary geometry, before the fracture runs. The cells then cut through
# slabs and columns like they cut through the walls, and a piece that comes
# away carries a bit of floor with it.
#
# WHERE THE PLAN COMES FROM. Not the bounding box — these buildings have
# courtyards, setbacks and L-shaped wings, and a slab spanning the bbox fills
# the courtyard with concrete. The footprint is read off the shell itself as a
# coarse occupancy grid, so the interior is only built where the building
# actually is.
# ---------------------------------------------------------------------------


def _box(cx, cy, cz, sx, sy, sz):
    """One axis-aligned box: (8, 3) verts and (12, 3) OUTWARD-wound triangles."""
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    v = np.array([[cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz],
                  [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz],
                  [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz],
                  [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz]],
                 dtype=np.float64)
    f = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                  [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6],
                  [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7]], dtype=np.int64)
    return v, f


def _plan_interior(P, lo, hi, cell):
    """Enclosed plan cells for one z-band: `(inside_mask, nx, ny)`.

    The occupancy built from shell points is a RING, not a plan — these
    buildings are hollow, so cells "containing points" outline the walls and
    leave the rooms empty. The interior is the free space the OUTSIDE cannot
    reach, so the border is flooded inward and whatever the flood never
    touched is enclosed by wall. An open courtyard reaches the border through
    its own gap and correctly stays empty.
    """
    nx = max(1, int(np.ceil((hi[0] - lo[0]) / cell)))
    ny = max(1, int(np.ceil((hi[1] - lo[1]) / cell)))
    occ = np.zeros((nx, ny), dtype=bool)
    if len(P):
        ix = np.clip(((P[:, 0] - lo[0]) / cell).astype(int), 0, nx - 1)
        iy = np.clip(((P[:, 1] - lo[1]) / cell).astype(int), 0, ny - 1)
        occ[ix, iy] = True

    outside = np.zeros_like(occ)
    stack = []
    for gx in range(nx):
        for gy in (0, ny - 1):
            if not occ[gx, gy] and not outside[gx, gy]:
                outside[gx, gy] = True
                stack.append((gx, gy))
    for gy in range(ny):
        for gx in (0, nx - 1):
            if not occ[gx, gy] and not outside[gx, gy]:
                outside[gx, gy] = True
                stack.append((gx, gy))
    while stack:
        gx, gy = stack.pop()
        for ax, ay in ((gx - 1, gy), (gx + 1, gy), (gx, gy - 1), (gx, gy + 1)):
            if 0 <= ax < nx and 0 <= ay < ny \
                    and not occ[ax, ay] and not outside[ax, ay]:
                outside[ax, ay] = True
                stack.append((ax, ay))
    return (~occ) & (~outside), nx, ny


def _dominant_material(prims):
    """The material covering the most faces across *prims*, or None.

    The interior is new geometry with no binding of its own, so it renders as
    untextured white — which is what "the floors are white" was. Reusing the
    building's own biggest material is what makes the inside read as the same
    construction as the outside, and it is already on the stage with its
    textures resolved.

    IT HAS TO WALK THE SUBSETS. These assets bind nothing at the mesh level:
    `SM_MERGED_BP_MBuilding02` is one `LOD0` mesh carrying thirteen
    `GeomSubset`s — facades, roof, first floor — each with its own material,
    and `ComputeBoundMaterial` on the mesh itself returns None. Reading only
    the mesh binding is why the first attempt still came out white.
    """
    weight = {}
    keep = {}
    for prim in prims:
        counts, _ = _face_arrays(prim)
        if counts is None:
            continue
        img = UsdGeom.Imageable(prim)
        subs = UsdGeom.Subset.GetAllGeomSubsets(img)
        if subs:
            for sub in subs:
                mat = UsdShade.MaterialBindingAPI(
                    sub.GetPrim()).ComputeBoundMaterial()[0]
                if not mat or not mat.GetPrim().IsValid():
                    continue
                idx = sub.GetIndicesAttr().Get() or []
                key = str(mat.GetPrim().GetPath())
                weight[key] = weight.get(key, 0) + len(idx)
                keep[key] = mat
        else:
            mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if mat and mat.GetPrim().IsValid():
                key = str(mat.GetPrim().GetPath())
                weight[key] = weight.get(key, 0) + len(counts)
                keep[key] = mat
    if not weight:
        return None
    return keep[max(weight, key=weight.get)]


def _triplanar_uv(verts, faces, uv_m):
    """Face-varying `st` by box projection, one UV per corner.

    A material without UVs on flat generated geometry samples its texture at a
    single point and comes out a solid colour. Projecting on whichever axis
    each triangle faces, at a real-world tile size, gives the slabs and
    columns the same texel density as the facade they sit behind.
    """
    tri = verts[faces]                                   # (F, 3, 3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    axis = np.argmax(np.abs(n), axis=1)                  # dominant axis per tri
    u = np.where(axis[:, None] == 0, tri[:, :, 1], tri[:, :, 0])
    v = np.where(axis[:, None] == 2, tri[:, :, 1], tri[:, :, 2])
    return np.stack([u / float(uv_m), v / float(uv_m)], axis=-1).reshape(-1, 2)


def interior_fill(prims, bounds: Bounds, storey_m: float = 3.5,
                  cell_m: float = 3.0) -> float:
    """How much of this building's inside is ALREADY modelled. 0..1.

    A hollow shell and a fully modelled building need opposite treatment, and
    the library holds both: some assets are a facade with nothing behind it,
    others ship real floor plates and partitions. Filling the second kind
    authors slabs through geometry that is already there — doubled floors,
    columns driven through walls, and a fracture that cuts both.

    Measured per storey on the plan grid `_plan_interior` already builds: take
    the enclosed footprint, ERODE IT BY ONE CELL to drop the perimeter wall
    (which every building has, hollow or not, and which is therefore not
    evidence of anything), and ask how much of what is left already holds
    geometry. A shell scores near zero because its core is empty; a modelled
    building scores high because its slabs and partitions are in the core.

    Averaged over storeys rather than taken at one height: a floored building
    is nearly solid in the bands where a slab sits and nearly empty between
    them, so any single band is a coin flip.
    """
    if not prims or bounds is None:
        return 0.0
    pts = [get_points(q) for q in prims]
    pts = [q for q in pts if q is not None and len(q)]
    if not pts:
        return 0.0
    P = np.vstack(pts)

    lo, hi = bounds.lo, bounds.hi
    cell = max(1.0, float(cell_m))
    storey = max(2.0, float(storey_m))
    levels = np.arange(bounds.base_z + storey, float(hi[2]) - 0.5 * storey,
                       storey)
    if not len(levels):
        return 0.0

    scores = []
    for cz in levels:
        band = P[(P[:, 2] >= cz - storey * 0.5) & (P[:, 2] < cz + storey * 0.5)]
        if len(band) < 8:
            continue
        inside, nx, ny = _plan_interior(band, lo, hi, cell)
        # `inside` is enclosed AND free; the enclosed footprint is that plus
        # whatever the flood could not reach because geometry occupies it.
        occ = np.zeros((nx, ny), dtype=bool)
        ix = np.clip(((band[:, 0] - lo[0]) / cell).astype(int), 0, nx - 1)
        iy = np.clip(((band[:, 1] - lo[1]) / cell).astype(int), 0, ny - 1)
        occ[ix, iy] = True
        enclosed = inside | occ

        core = enclosed.copy()                     # erode by one cell
        for sh, ax in ((1, 0), (-1, 0), (1, 1), (-1, 1)):
            core &= np.roll(enclosed, sh, axis=ax)
        core[0, :] = core[-1, :] = core[:, 0] = core[:, -1] = False
        if core.sum() < 4:
            continue
        scores.append(float((core & occ).sum()) / float(core.sum()))
    return float(np.mean(scores)) if scores else 0.0


def fill_interior(stage, root_prim, bounds: Bounds, storey_m: float = 3.5,
                  cell_m: float = 3.0, slab_m: float = 0.30,
                  column_m: float = 0.9, column_every: int = 3,
                  inset: float = 0.0, uv_m: float = 4.0,
                  min_radius_m: float = 25.0, already_filled: float = 0.35,
                  max_boxes: int = 20000) -> int:
    """Author floor slabs and a column grid inside *root_prim*. Returns boxes.

    `solidify` extrudes the outer SURFACE inward and stops, because the
    surface is all it knows about. On a house that is the building. On a tower
    it is a facade with nothing behind it — measured on
    `SM_MERGED_BP_MBuilding02`, a thickened fragment was 11% material by
    bounding box and none of 40 sampled were watertight — so a Voronoi cell
    10 m across holds one curved panel and a lot of air, and a cross-section
    through it shows an empty box rather than stacked floors.

    Only for buildings with an inside worth modelling, and only for the ones
    that do not already have one. Two gates:

    * *min_radius_m* skips houses, whose shell genuinely is the building.
    * *already_filled* skips assets that ship their own floors and partitions
      — `interior_fill` measures that. Authoring into one of those puts slabs
      through geometry that is already there: doubled floors, columns driven
      through walls, and a fracture that cuts both.

    The threshold sits in a wide empty gap rather than on a guess. Measured
    over the urban library: BG_Building_C 0.000, BG_Building_F 0.000,
    MBuilding01 0.005, BG_Building_A 0.013 — all hollow shells; MBuilding02
    0.120 and BG_Building_E 0.139, which read high off setbacks and structural
    cores but are still shells (a thickened MBuilding02 fragment was 11%
    material by bounding box); and MBuilding05 at 0.667, which really does
    have floor plates. Nothing in the library lands between 0.14 and 0.67.

    THE PLAN IS PER STOREY, NOT PER BUILDING
    ----------------------------------------
    One plan for the whole height is the bug that put slabs through thin air.
    These towers set back, notch and open out as they rise: a cell enclosed at
    the second floor can be outdoors at the twentieth, and filling it at every
    level pushes floors out through a concave elevation. So the occupancy is
    rebuilt from the points in EACH storey's own z-band, and a level only gets
    floor where that level actually encloses space.

    Two things make the result read as building rather than as scaffolding:
    it is bound to the building's own dominant material (`_dominant_material`)
    rather than left white, and it carries projected UVs (`_triplanar_uv`) so
    that material has somewhere to land.
    """
    prims = mesh_prims(root_prim)
    if not prims or bounds is None or bounds.radius < float(min_radius_m):
        return 0
    if interior_fill(prims, bounds, storey_m, cell_m) >= float(already_filled):
        return 0

    pts = [get_points(p) for p in prims]
    pts = [q for q in pts if q is not None and len(q)]
    if not pts:
        return 0
    P = np.vstack(pts)

    lo, hi = bounds.lo, bounds.hi
    cell = max(1.0, float(cell_m))
    storey = max(2.0, float(storey_m))
    z0, z1 = bounds.base_z, float(hi[2])
    levels = np.arange(z0 + storey, z1 - 0.5 * storey, storey)
    if not len(levels):
        return 0

    sxy = max(0.5, cell - 2.0 * float(inset))
    V, F, base, n_box = [], [], 0, 0
    for li, cz in enumerate(levels):
        band = P[(P[:, 2] >= cz - storey) & (P[:, 2] < cz + storey)]
        if not len(band):
            continue
        inside, nx, ny = _plan_interior(band, lo, hi, cell)
        cells = np.argwhere(inside)
        for gx, gy in cells:
            if n_box >= max_boxes:
                break
            cx = lo[0] + (gx + 0.5) * cell
            cy = lo[1] + (gy + 0.5) * cell
            v, f = _box(cx, cy, float(cz), sxy, sxy, float(slab_m))
            V.append(v); F.append(f + base); base += 8; n_box += 1
            # COLUMNS ON A COARSER LATTICE. One per floor cell is a forest at
            # a 3 m grid; every `column_every` cells is a structural bay.
            # Storey-tall, not full height: a single post through the whole
            # building is one rigid strut, and with a blast under it the tower
            # was levered UPWARD (mean displacement came out +1.77 m).
            if gx % int(column_every) or gy % int(column_every):
                continue
            if n_box >= max_boxes:
                break
            v, f = _box(cx, cy, float(cz) - storey * 0.5,
                        float(column_m), float(column_m), storey)
            V.append(v); F.append(f + base); base += 8; n_box += 1
        if n_box >= max_boxes:
            break

    if not V:
        return 0
    world = np.vstack(V)
    faces = np.vstack(F)

    inv = np.linalg.inv(_world_matrix(root_prim))
    local = world @ inv[:3, :3] + inv[3, :3]
    path = root_prim.GetPath().AppendChild("interior")
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(_vec3f(local))
    mesh.CreateFaceVertexCountsAttr(_vt_int(np.full(len(faces), 3)))
    mesh.CreateFaceVertexIndicesAttr(
        _vt_int(faces.reshape(-1)))
    mesh.CreateExtentAttr([Gf.Vec3f(*map(float, local.min(0))),
                           Gf.Vec3f(*map(float, local.max(0)))])
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    # DOUBLE-SIDED, because a cross-section is a view from the INSIDE. USD
    # defaults to single-sided and the renderer then drops the back of every
    # wall you are looking through the tear line at, which reads as the
    # building being transparent where it should be showing its own interior
    # face.
    mesh.CreateDoubleSidedAttr(True)

    uv = _triplanar_uv(local, faces, uv_m)
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set(_vec2f(uv))

    mat = _dominant_material(prims)
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return n_box
