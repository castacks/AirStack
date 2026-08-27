"""fracture stage — cut the house's real geometry into irregular fragments.

WHY THIS AND NOT THE ALTERNATIVES
---------------------------------
Scattering kit panels reads as "rectangles toppled onto each other", because
that is what it is. Generated convex chunks (`disaster.rubble`) fix the shape
but throw the building away: they know nothing about the wall they came from,
so a fragment never carries a window reveal, a corner, or a board edge.

The alternatives were checked before settling here:

  NVIDIA Blast    the purpose-built destruction library. NOT in this Kit
                  build, and not available as an Isaac Sim 5.1 extension.
  voxel / VDB     robust, but it resamples the surface — a 5 cm voxel turns
                  crisp panel edges into blobs, and finer costs memory fast.
  pre-fractured   authoring fragments in Houdini/Blender and importing them.
                  Best-looking option, but it is per-asset manual work and
                  cannot follow procedurally generated geometry.
  Voronoi + CSG   THIS. Cut the actual mesh with real boolean intersection.

VORONOI FRACTURE
----------------
Seed points are scattered through the mesh. Each seed's Voronoi cell is the
set of points closer to it than to any other seed — a convex polyhedron, built
here by starting from an oversized box and slicing it with the perpendicular
bisector plane against every other seed. Intersecting the mesh with that cell
gives one fragment, and the cells tile space, so the fragments partition the
mesh exactly: on a test panel with a window opening, 12 fragments recovered
100.0% of the original volume, all watertight.

Seeding is what controls the look. Uniform seeds give blocky shards; seeds
pulled toward a point give a fragmentation focus; seeds stretched along an
axis give long splinters, which is what burnt timber framing actually breaks
into.

DEPENDENCIES
------------
`trimesh` with `manifold3d` as the boolean engine, plus `shapely` for the
capped slicing. manifold3d and shapely are pip installs added for this; they
are not in the stock Isaac Sim image.

COST
----
N seeds costs N*(N-1) plane slices plus N booleans per module. Fine at N<=12
per module for a handful of modules; it is not something to run over a suburb
without baking the result to disk first.
"""

import math
import os
import subprocess
import sys

import numpy as np

# THREE, NOT TWO. `slice_mesh_plane(cap=True)` triangulates the cap polygon
# through trimesh's `triangulate_polygon`, which needs `mapbox_earcut` (or
# `triangle`); without it every capped slice logs "try running pip install
# mapbox-earcut" and comes back empty, and `vegetation.wood_debris` then has
# no stock to cut. A fresh container from the current image has none of them.
_DEPS = ("manifold3d", "shapely", "mapbox_earcut")


def ensure_deps(verbose=True):
    """Install the boolean/slicing backends if this image predates them.

    They are in the Dockerfile now, but `airstack down` destroys the container
    and an image built before that change comes back without them. Installing
    on demand keeps the bench runnable without forcing a rebuild first; once
    the image is rebuilt this is a no-op that costs one import each.
    """
    missing = []
    for m in _DEPS:
        try:
            __import__(m)
        except ImportError:
            missing.append(m)
    if not missing:
        return True
    if verbose:
        print("[fracture] installing {0} (not in this image yet)"
              .format(", ".join(missing)))
    try:
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "--no-cache-dir",
             "--disable-pip-version-check", "-q"] + missing)
    except Exception as exc:
        print("[fracture] could not install {0}: {1}".format(missing, exc))
        return False
    return True





# ---------------------------------------------------------------------------
# Plane slicing — the hot loop, and the one worth a second backend
# ---------------------------------------------------------------------------
#
# EVERY CUT IN THIS FILE GOES THROUGH ONE FUNCTION, and it is where the build
# time is. A Voronoi fracture is N*(N-1) plane slices; a tree bole is another
# ten before any debris is cut; a 250 m block runs that over hundreds of
# trees. Measured on the mini scene, mesh slicing is the single longest phase
# and it is entirely CPU: trimesh is numpy, and its capping is shapely.
#
# `slice_mesh_plane` and VTK's `vtkClipPolyData` agree on the convention that
# matters — both KEEP the side the normal points to — so they are swappable.
# VTK's is C++ where trimesh's is Python, which is the whole argument.
#
# CAPPING IS THE PART TO GET RIGHT. The naive VTK recipe caps every boundary
# edge of the result, and these are kit meshes: open shells with no back
# faces, so that would close holes the asset was authored with and turn a
# hollow panel into a solid one. `vtkCutter` instead returns exactly the
# plane's intersection with the surface, which is the cut face and nothing
# else — the same thing trimesh caps and no more.

_BACKEND = os.environ.get("FRACTURE_BACKEND", "auto").strip().lower()
_VTK = None


def _vtk():
    """The vtk module, or None. Cached, including the failure."""
    global _VTK
    if _VTK is None:
        if _BACKEND == "trimesh":
            _VTK = False
        else:
            try:
                import vtk as _v
                _VTK = _v
            except Exception:
                _VTK = False
                if _BACKEND == "vtk":
                    print("[fracture] FRACTURE_BACKEND=vtk but vtk will not "
                          "import; falling back to trimesh")
    return _VTK or None


def ensure_vtk(verbose=True):
    """Install VTK if it is not here. Returns True when the backend is live."""
    if _vtk() is not None:
        return True
    if _BACKEND == "trimesh":
        return False
    if verbose:
        print("[fracture] installing vtk (C++ plane clipping backend)")
    try:
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "--no-cache-dir",
             "--disable-pip-version-check", "-q", "vtk"])
    except Exception as exc:
        print("[fracture] could not install vtk: {0}".format(exc))
        return False
    global _VTK
    _VTK = None
    return _vtk() is not None


def _to_vtk(mesh):
    import vtk
    from vtk.util import numpy_support as ns

    v = np.ascontiguousarray(np.asarray(mesh.vertices, dtype=np.float64))
    f = np.asarray(mesh.faces, dtype=np.int64)
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(v, deep=True))
    cells = np.hstack([np.full((len(f), 1), 3, dtype=np.int64), f]).ravel()
    ca = vtk.vtkCellArray()
    ca.SetCells(len(f), ns.numpy_to_vtkIdTypeArray(cells, deep=True))
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetPolys(ca)
    return pd


def _from_vtk(pd):
    import trimesh
    import vtk
    from vtk.util import numpy_support as ns

    tf = vtk.vtkTriangleFilter()
    tf.SetInputData(pd)
    tf.Update()
    out = tf.GetOutput()
    if out.GetNumberOfPoints() == 0 or out.GetNumberOfPolys() == 0:
        return None
    v = ns.vtk_to_numpy(out.GetPoints().GetData())
    polys = out.GetPolys()
    try:
        # VTK 9 keeps connectivity and offsets separately; after the triangle
        # filter every cell is 3 long, so the connectivity reshapes directly.
        conn = ns.vtk_to_numpy(polys.GetConnectivityArray())
        f = conn.reshape(-1, 3)
    except AttributeError:
        raw = ns.vtk_to_numpy(polys.GetData()).reshape(-1, 4)
        f = raw[:, 1:]
    if not len(f):
        return None
    return trimesh.Trimesh(vertices=np.asarray(v, dtype=float),
                           faces=np.asarray(f, dtype=np.int64), process=False)


def _vtk_slice(mesh, normal, origin, cap=True):
    """`slice_mesh_plane` on VTK. Keeps the side *normal* points to."""
    import vtk

    pd = _to_vtk(mesh)
    plane = vtk.vtkPlane()
    plane.SetOrigin(float(origin[0]), float(origin[1]), float(origin[2]))
    plane.SetNormal(float(normal[0]), float(normal[1]), float(normal[2]))

    clip = vtk.vtkClipPolyData()
    clip.SetInputData(pd)
    clip.SetClipFunction(plane)
    clip.Update()
    kept = clip.GetOutput()
    if kept.GetNumberOfPolys() == 0:
        return None

    if cap:
        # THE CUT FACE ONLY — see the note above about open shells.
        cut = vtk.vtkCutter()
        cut.SetInputData(pd)
        cut.SetCutFunction(plane)
        cut.Update()
        strip = vtk.vtkStripper()
        strip.SetInputConnection(cut.GetOutputPort())
        strip.Update()
        loops = strip.GetOutput()
        if loops.GetNumberOfLines():
            face = vtk.vtkPolyData()
            face.SetPoints(loops.GetPoints())
            face.SetPolys(loops.GetLines())
            fill = vtk.vtkTriangleFilter()
            fill.SetInputData(face)
            fill.Update()
            app = vtk.vtkAppendPolyData()
            app.AddInputData(kept)
            app.AddInputData(fill.GetOutput())
            app.Update()
            clean = vtk.vtkCleanPolyData()
            clean.SetInputConnection(app.GetOutputPort())
            clean.Update()
            kept = clean.GetOutput()
    return _from_vtk(kept)


def slice_plane(mesh, normal, origin, cap=True):
    """Cut *mesh* with a plane, keeping the side *normal* points to.

    One entry point for every cut in this module, so the backend is a single
    switch rather than a search-and-replace. Falls back to trimesh whenever
    VTK is absent or returns nothing usable — a slice that comes back empty
    is a legitimate answer (the plane missed the mesh), so the fallback only
    fires on an EXCEPTION, never on an empty result, or a fragment that
    genuinely got cut away would be silently resurrected by the other backend.
    """
    from trimesh.intersections import slice_mesh_plane

    v = _vtk()
    if v is not None:
        try:
            return _vtk_slice(mesh, normal, origin, cap=cap)
        except Exception as exc:
            print("[fracture] vtk slice failed ({0}); using trimesh"
                  .format(exc))
    return slice_mesh_plane(mesh, plane_normal=normal, plane_origin=origin,
                            cap=cap)


def _tri(counts, indices):
    """Fan-triangulate USD faceVertexCounts/Indices into an (n, 3) array."""
    faces = []
    k = 0
    for c in counts:
        c = int(c)
        if c >= 3:
            for i in range(1, c - 1):
                faces.append((indices[k], indices[k + i], indices[k + i + 1]))
        k += c
    return np.asarray(faces, dtype=np.int64)


def prim_to_mesh(stage, prim_path):
    """Every Mesh under *prim_path*, welded into one world-space trimesh."""
    import trimesh
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        return None

    xf = UsdGeom.XformCache()
    verts, faces, base = [], [], 0
    # TRAVERSE INSTANCE PROXIES. A plain PrimRange stops at an instance
    # boundary, so anything the suburb marks instanceable — fences, street
    # furniture — looked like it had no geometry at all and silently produced
    # no fragments.
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not cnt or not idx:
            continue
        f = _tri(cnt, idx)
        if not len(f):
            continue
        mat = np.array(xf.GetLocalToWorldTransform(p), dtype=float)
        v = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
        v = v @ mat[:3, :3] + mat[3, :3]        # USD matrices are row-vector
        verts.append(v)
        faces.append(f + base)
        base += len(v)

    if not verts:
        return None
    mesh = trimesh.Trimesh(vertices=np.vstack(verts),
                           faces=np.vstack(faces), process=True)
    return mesh if len(mesh.faces) else None


def _seeds(mesh, n, rng, mode="uniform", focus=None, axis=None,
           aspect=None):
    """Voronoi seed points. Their arrangement is what decides the break.

    Modes: `uniform` (isotropic rubble), `char` (the alligator grid of burnt
    timber), `splinter` (long shards), `focus` (denser toward a point) and
    `plank` (an anisotropic LATTICE, so the cells are rectangular boards —
    what a wall blown apart by wind comes to pieces as; see that branch).

    `axis` OVERRIDES the "long axis is the grain" assumption, and a tree is
    why it exists. Black_Oak's woody bole measures 20.6 x 19.8 x 17.2 m —
    its limbs reach further than it is tall — so `argmax(span)` picks X and
    `splinter` produces HORIZONTAL shards out of a standing trunk. The grain
    of a trunk is Z whatever its crown does, so the caller says so.

    `char` is the one matched to real burnt timber, and it is NOT what
    intuition suggests. Work on the topology of crack patterns in charred wood
    (Bartlett et al., arXiv 1604.01249) finds the cracks run mainly
    PERPENDICULAR to the fibres — across the grain, where the wood is
    strongest — rather than along it, because charring drives the stress
    thermally rather than by drying shrinkage. Combined with sparser splits
    along the grain, that is what produces the familiar alligator grid.

    So `char` seeds a grid that is DENSE along the long axis (many closely
    spaced cross-cuts) and SPARSE across it (few long splits), which puts the
    cell walls where the cracks actually are.
    """
    lo, hi = mesh.bounds
    span = np.maximum(hi - lo, 1e-6)
    axis = int(np.argmax(span)) if axis is None else int(axis)
    others = [a for a in range(3) if a != axis]

    if mode == "char":
        # Cross-cuts roughly every 0.6 of the short dimension, which is the
        # aspect real checking settles at.
        n_long = max(2, int(round(np.sqrt(n * span[axis]
                                          / max(span[others[0]], 1e-6)))))
        n_long = min(n_long, n)
        n_across = max(1, int(round(n / n_long)))
        pts = []
        for i in range(n_long):
            for j in range(n_across):
                q = np.empty(3)
                q[axis] = lo[axis] + span[axis] * (i + 0.5) / n_long
                q[others[0]] = (lo[others[0]]
                                + span[others[0]] * (j + 0.5) / n_across)
                q[others[1]] = lo[others[1]] + span[others[1]] * rng.random()
                # Jitter, or the grid reads as machined.
                q += (rng.random(3) - 0.5) * span * np.array(
                    [0.55 if a == axis else 0.35 for a in range(3)]) / max(
                        n_long, 2)
                pts.append(q)
        return np.asarray(pts)

    if mode == "plank":
        # BOARDS, NOT CHIPS — the tornado mode. A Voronoi diagram of a REGULAR
        # LATTICE is a grid of boxes, exactly: every cell is the set of points
        # nearer its own lattice site than any other, and for a rectangular
        # lattice those bisectors are three families of parallel planes. So
        # anisotropic lattice SPACING gives anisotropic BOXES, and spacing that
        # is long along `axis` and short across it gives planks.
        #
        # This is the opposite construction to `char`, which is also a lattice
        # but a DENSE-along / SPARSE-across one, because a crack pattern runs
        # across the grain. A wall coming apart in a wind does not crack, it
        # DELAMINATES: the sheathing lets go along its fasteners and what is
        # left is boards and panels the size the mill cut them, which is why
        # `tornado.jpeg` shows a debris field of pale rectangles rather than
        # the black gravel a burnt house leaves.
        #
        # THE JITTER IS SMALL AND IT IS NOT OPTIONAL. Zero jitter gives cells
        # that are identical to the millimetre, and a hundred identical boxes
        # reads as a crate of bricks. 0.18 of the cell pitch keeps every piece
        # recognisably a board and no two the same.
        cross = [a for a in range(3) if a != axis]
        # ASPECT IS DRAWN PER MODULE, NOT FIXED. A single `ar` gives every
        # module in the building the same board proportion, and the pieces
        # come out visibly interchangeable. `aspect` is a (lo, hi) range so a
        # caller can say what KIND of board this module sheds — a wall sheds
        # long studs (high ratio), a roof sheds SHEETS (low ratio) — and
        # within that, each module still draws its own.
        alo, ahi = ((3.5, 7.0) if aspect is None
                    else (float(aspect[0]), float(aspect[1])))
        ar = alo + (ahi - alo) * float(rng.random())
        s_l, s_a, s_b = span[axis], span[cross[0]], span[cross[1]]
        # Cell edge lengths (e_l, e_a, e_b) with e_l = ar * e_cross, and
        # (s_l/e_l) * (s_a/e_a) * (s_b/e_b) = n.
        # Build the lattice for MORE sites than asked, because ~30% of them
        # are deleted below to merge cells. Without this the dropout silently
        # under-delivers by a third.
        target = max(1, int(round(n / 0.70)))
        e_c = max(1e-4, (s_l * s_a * s_b / (ar * target)) ** (1.0 / 3.0))

        def _counts(edge):
            return {axis: max(1, int(round(s_l / (ar * edge)))),
                    cross[0]: max(1, int(round(s_a / edge))),
                    cross[1]: max(1, int(round(s_b / edge)))}

        # THE `max(1, ...)` CLAMPS OVERSHOOT, AND ON A THIN MODULE THEY
        # OVERSHOOT HARD. `e_c` solves the cell count exactly, but a wall is
        # 0.2 m thick and a roof panel 0.1 m, so the across-thickness count
        # rounds to 0 and is clamped up to 1 — which multiplies the product by
        # 1/0.34 rather than leaving it alone. Two clamped axes and a request
        # for 14 cells comes back as 50.
        #
        # That is not a cosmetic overshoot. Every extra cell is a rigid body
        # in the settle, and the archetype bake settles every house in the
        # library at once: a 4x overshoot across 40 damaged archetypes is tens
        # of thousands of bodies, which is where PhysX stops being fast and
        # starts being a reason the bake did not finish. Grow the cell until
        # the product is within `cap` of what was asked for.
        cap = 1.6
        counts = _counts(e_c)
        for _ in range(6):
            prod = counts[0] * counts[1] * counts[2]
            if prod <= cap * target:
                break
            e_c *= (prod / (cap * target)) ** (1.0 / 3.0)
            counts = _counts(e_c)
        pts = []
        for i in range(counts[0]):
            for j in range(counts[1]):
                for k in range(counts[2]):
                    idx = (i, j, k)
                    q = np.empty(3)
                    for a in range(3):
                        step = span[a] / counts[a]
                        # 0.34 rather than 0.18. The jitter is the ONLY thing
                        # separating one cell from the next in a regular
                        # lattice, and at 0.18 a roof plate came apart into
                        # two dozen boards identical to within a few
                        # centimetres — reported from the assembled scene as
                        # "a lot of the roof plate, very repetitive".
                        q[a] = (lo[a] + step * (idx[a] + 0.5)
                                + (rng.random() - 0.5) * step * 0.34)
                    pts.append(q)

        # DROP SITES TO MERGE CELLS — the move that actually buys diversity.
        #
        # Jitter alone perturbs a cell's POSITION and barely its SIZE: a
        # lattice of N sites gives N cells of about one lattice volume each,
        # however hard you shake them. Deleting a share of the sites is what
        # changes that — every cell adjacent to a hole expands to absorb it,
        # so what comes out is a MIX of single-width boards, double-width
        # ones, and the occasional big panel where two holes fell together.
        #
        # That is also what a building actually sheds. Sheathing does not
        # disintegrate into a uniform grid of identical rectangles; it tears
        # along some fastener lines and not others, so a debris field is
        # mostly standard sections with a minority of larger pieces still
        # joined. `keep` is the share of lattice sites that survive.
        #
        # The lattice was built for `n / keep` sites (see `target` above) so
        # the surviving count still lands near what the caller asked for.
        keep = 0.70
        if len(pts) > 3:
            m = max(2, int(round(len(pts) * keep)))
            sel = rng.permutation(len(pts))[:m]
            pts = [pts[int(i)] for i in sel]
        return np.asarray(pts)

    if mode == "splinter":
        p = rng.uniform(lo, hi, size=(n, 3))
        for a in others:
            mid = 0.5 * (lo[a] + hi[a])
            p[:, a] = mid + (p[:, a] - mid) * 0.28
        return p
    if mode == "focus" and focus is not None:
        p = rng.uniform(lo, hi, size=(n, 3))
        t = rng.random((n, 1)) ** 1.8
        return p * t + np.asarray(focus, dtype=float) * (1.0 - t)
    return rng.uniform(lo, hi, size=(n, 3))


def roughen(mesh, rng, amount=0.035):
    """Break the flat faces and clean edges a Voronoi cut leaves behind.

    A convex cell is bounded by perfect planes meeting at perfect edges, which
    is what reads as "polygonal". Charred timber breaks with a fibrous, ragged
    face — so every vertex is displaced by band-limited noise keyed off its
    own position, which roughens the cut surfaces and ragged-edges the
    silhouette without moving the piece or changing its size much.

    Keyed off position rather than per-vertex random so that vertices which
    coincide on a shared edge move together and the piece does not tear open.
    """
    v = np.asarray(mesh.vertices, dtype=float)
    if not len(v):
        return mesh
    scale = float(np.max(mesh.extents)) * float(amount)
    ph = rng.random(3) * 100.0
    f = 6.5 / max(float(np.max(mesh.extents)), 1e-6)
    d = np.stack([np.sin(v[:, 1] * f + ph[0]) * np.cos(v[:, 2] * f + ph[1]),
                  np.sin(v[:, 2] * f + ph[1]) * np.cos(v[:, 0] * f + ph[2]),
                  np.sin(v[:, 0] * f + ph[2]) * np.cos(v[:, 1] * f + ph[0])], -1)
    d += 0.45 * np.stack([np.sin(v[:, 1] * f * 2.7 + ph[2]),
                          np.sin(v[:, 2] * f * 2.7 + ph[0]),
                          np.sin(v[:, 0] * f * 2.7 + ph[1])], -1)
    mesh.vertices = v + d * scale
    return mesh


# ---------------------------------------------------------------------------
# A COHERENT roughening field — the fix for "the intact roof is a crack mosaic"
# ---------------------------------------------------------------------------
#
# `roughen` above draws a NEW phase per fragment and derives its frequency and
# its amplitude from THAT FRAGMENT's own size. For a pile of debris that is
# right: every piece is scarred differently and no two look alike. For a piece
# that is only PARTLY broken it is wrong twice over:
#
#   * two cells that shared a Voronoi face are displaced by different noise,
#     so the face opens up — every cell boundary in the SURVIVING part of a
#     slab becomes a visible crack, and a roof judged cell-by-cell comes out
#     as a mosaic of flat polygons (the user's "unnatural rectangular/square
#     parts broken off");
#   * the amplitude is a FRACTION of the piece, so a 5 m cell is displaced by
#     0.06 m at rough=0.012 and a 0.4 m cell by 0.005 m — the big surviving
#     cells get the big scars, which is backwards.
#
# `noise_field` returns one function of WORLD POSITION with an amplitude in
# METRES and a fixed wavelength, so vertices that coincide stay coincident and
# every piece is scarred at the same physical scale.
def noise_field(rng, amp_m=0.03, lam_m=1.1):
    """(phase, angular frequency, amplitude) for `roughen_field`."""
    return (np.asarray(rng.random(3) * 100.0, dtype=float),
            6.2832 / max(float(lam_m), 1e-3), float(amp_m))


def roughen_field(mesh, field):
    """`roughen`, but from a shared position-keyed field (see `noise_field`)."""
    if field is None:
        return mesh
    ph, f, amp = field
    v = np.asarray(mesh.vertices, dtype=float)
    if not len(v):
        return mesh
    d = np.stack([np.sin(v[:, 1] * f + ph[0]) * np.cos(v[:, 2] * f + ph[1]),
                  np.sin(v[:, 2] * f + ph[1]) * np.cos(v[:, 0] * f + ph[2]),
                  np.sin(v[:, 0] * f + ph[2]) * np.cos(v[:, 1] * f + ph[0])], -1)
    d += 0.45 * np.stack([np.sin(v[:, 1] * f * 2.7 + ph[2]),
                          np.sin(v[:, 2] * f * 2.7 + ph[0]),
                          np.sin(v[:, 0] * f * 2.7 + ph[1])], -1)
    mesh.vertices = v + d * amp
    return mesh


def inset(mesh, gap_m):
    """Pull every face `gap_m` toward the centroid, PER AXIS.

    `fracture_mesh` insets by a RATIO (`shrink=0.97`), which is 0.6 cm on a
    0.4 m chip and 8 cm on a 5 m slab cell: on a big surviving slab that ratio
    is what draws the long dark crack lines the user called out. A gap in
    metres is the same hairline whatever the cell is, and 0 means the cells
    butt exactly — which is what a slab that did NOT break should look like.
    """
    v = np.asarray(mesh.vertices, dtype=float)
    if not len(v) or gap_m <= 0.0:
        return mesh
    c = np.asarray(mesh.centroid, dtype=float)
    half = np.maximum(np.asarray(mesh.extents, dtype=float) * 0.5, 1e-4)
    s = np.clip(1.0 - float(gap_m) / half, 0.25, 1.0)
    mesh.vertices = c + (v - c) * s
    return mesh


def _cell(mesh, pts, i):
    """The Voronoi cell of seed `i` clipped out of `mesh`, or None.

    NEAREST-FIRST WITH AN EARLY-OUT. The straightforward loop slices against
    all N-1 other seeds, so a fracture costs N*(N-1) plane cuts and refining
    anything is unaffordable. But the bisector against seed q sits at |q-p|/2
    from p, so once the cell's own circumradius r is below that the plane
    cannot touch it — and with the seeds sorted by distance, no later one can
    either. The result is EXACTLY the same partition (this is the standard
    clipping bound, not an approximation); it just stops early, at ~15-25 cuts
    per cell however many seeds there are, which is what makes the two-scale
    fracture below affordable.
    """
    p = np.asarray(pts[i], dtype=float)
    d = np.linalg.norm(np.asarray(pts, dtype=float) - p, axis=1)
    frag = mesh
    for j in np.argsort(d):
        j = int(j)
        if j == i or d[j] < 1e-9:
            continue
        v = np.asarray(frag.vertices, dtype=float)
        if not len(v):
            return None
        if 0.5 * float(d[j]) > float(np.max(np.linalg.norm(v - p, axis=1))):
            break
        q = np.asarray(pts[j], dtype=float)
        try:
            frag = slice_plane(frag, -(q - p) / float(d[j]), (p + q) * 0.5,
                               cap=True)
        except Exception:
            return None
        if frag is None or not len(frag.faces):
            return None
    return frag


def fracture_mesh(mesh, n_pieces, rng, mode="uniform", focus=None,
                  min_volume_frac=0.004, rough=0.035, shrink=0.97,
                  consume=0.30, consume_pool=1.25, axis=None, aspect=None,
                  max_piece_m=None, verbose=False):
    """Split one trimesh into Voronoi fragments. Returns a list of trimeshes.

    SLICING, NOT BOOLEANS. The first version intersected the mesh with a cell
    polyhedron via manifold3d, which needs watertight input — and kit panels
    are game art: open shells with no back faces, so `mesh.volume` is garbage
    and every boolean returned nothing. Clipping the mesh directly by each
    bisector plane with `cap=True` produces the identical Voronoi partition,
    caps the open edges as it goes, and never needs the mesh to be manifold.
    """
    if mesh is None or not len(mesh.faces):
        return []

    # Bounding-box volume, because a non-watertight shell has no real one.
    # This floor also caps how fine a cut is worth making: past ~20 seeds
    # every new cell falls under it and is discarded, so raising the seed
    # count alone stops shrinking the median. Lowering it does produce finer
    # rubble — measured 0.34 -> 0.27 m3 median — but that read as gravel
    # rather than as a collapsed house, so it stays where it was.
    bbox_vol = float(np.prod(np.maximum(mesh.extents, 1e-6)))
    keep_min = bbox_vol * float(min_volume_frac)

    pts = _seeds(mesh, int(n_pieces), rng, mode=mode, focus=focus,
                 axis=axis, aspect=aspect)
    keep, out = [], []
    # WHY A MODULE YIELDED NOTHING is reported, always. The earthquake bake
    # lost ten of sixteen style rows to fractures that came back empty with
    # no line in any log: `slice_plane` failures were swallowed here and
    # `verbose=False` hid the count. One line per empty module is cheap.
    n_exc = n_empty = n_cull = 0
    first_exc = None
    _P = np.asarray(pts, dtype=float)
    for i, p in enumerate(pts):
        frag = mesh
        # NEAREST-FIRST, WITH THE CLIPPING EARLY-OUT (see `_cell`): the
        # bisector against a seed |q-p| away sits |q-p|/2 from p, so once the
        # cell's own circumradius is smaller than that the plane cannot touch
        # it and — the seeds being sorted — nor can any later one. Same
        # partition, ~15-25 cuts per cell instead of N-1, which is what makes
        # the refinement in `fracture_split` affordable at all.
        _d = np.linalg.norm(_P - np.asarray(p, dtype=float), axis=1)
        for j in np.argsort(_d):
            j = int(j)
            if i == j:
                continue
            q = pts[j]
            n = float(_d[j])
            if n < 1e-9:
                continue
            _v = np.asarray(frag.vertices, dtype=float)
            if len(_v) and 0.5 * n > float(np.max(np.linalg.norm(
                    _v - np.asarray(p, dtype=float), axis=1))):
                break
            try:
                frag = slice_plane(frag, -(q - p) / n, (p + q) * 0.5, cap=True)
            except Exception as exc:
                frag = None
                n_exc += 1
                if first_exc is None:
                    first_exc = "{0}: {1}".format(type(exc).__name__, exc)
            if frag is None or not len(frag.faces):
                if frag is not None:
                    n_empty += 1
                break
        if frag is None or not len(frag.faces):
            continue
        if float(np.prod(np.maximum(frag.extents, 1e-9))) < keep_min:
            n_cull += 1
            continue
        keep.append(roughen(frag, rng, amount=rough))
    if not keep:
        print("[fracture] EMPTY: {0} seeds -> 0 fragments (slice exceptions {1}, "
              "empty slices {2}, culled {3}; faces {4}, extents {5}, watertight "
              "{6}){7}".format(len(pts), n_exc, n_empty, n_cull, len(mesh.faces),
                               np.round(mesh.extents, 2), mesh.is_watertight,
                               "  first: " + first_exc if first_exc else ""))

    # WOOD BURNS AWAY, it does not merely break. A collapsed timber building
    # leaves far less material than it was built from, so a share of the
    # fragments are never authored at all.
    #
    # WEIGHTED TOWARD THE BIG PIECES. This has been all three ways round, and
    # the reasoning that looks physical is the one that looks wrong:
    #
    #   small-biased   "a splinter is consumed, a beam chars" — true of the
    #                  material, but it strips out exactly the rubble that
    #                  makes a pile read as rubble and leaves a heap of slabs.
    #   uniform        better, but the slabs are still the thing you notice.
    #   large-biased   what actually reads. A big surviving panel is a wall
    #                  that fell over; take those out and what is left is
    #                  debris. It is also not unphysical — a large panel has
    #                  the most surface exposed to the fire and the least
    #                  chance of being shielded by anything else in the pile.
    #
    # Candidates are drawn from the large end and then chosen among, so it is
    # a bias rather than a rule and the odd big piece still survives.
    if consume > 0.0 and keep:
        vols = np.array([float(np.prod(np.maximum(f.extents, 1e-9)))
                         for f in keep])
        order = list(np.argsort(vols)[::-1])          # largest first
        n_drop = int(round(len(keep) * float(consume)))
        # `consume_pool` IS THE STRENGTH OF THE BIAS. Candidates are the
        # largest `n_drop * consume_pool` fragments and `n_drop` of those are
        # then chosen at random, so 1.0 removes exactly the biggest pieces and
        # larger values let progressively smaller ones into the draw. 1.25
        # leaves a noticeable share of big panels standing; drop it toward 1.0
        # when the pile still reads as slabs rather than debris.
        pool = order[:min(len(order),
                          max(n_drop, int(n_drop * float(consume_pool))))]
        rng.shuffle(pool)
        dropped = set(pool[:n_drop])
        keep = [f for i, f in enumerate(keep) if i not in dropped]

    # AN ABSOLUTE CEILING ON A PIECE, not a share of them.
    #
    # `consume` is a probabilistic bias and it cannot guarantee anything: a kit
    # wall is an open SHELL, so every cell of it is a thin PLATE, and a total
    # collapse that keeps even a handful of 2-3 m plates reads as broken
    # crockery rather than rubble however good the heap under them is (the
    # round-2 review of the DG5 city: "a sea of large thin flat plates"). A
    # collapse recipe passes `max_piece_m` and nothing bigger than that is
    # authored; the material is in the heap, which is what carries the mass.
    if max_piece_m and keep:
        lim = float(max_piece_m)
        ext = np.array([float(np.max(f.extents)) for f in keep])
        under = [f for f, e in zip(keep, ext) if e <= lim]
        # A FLOOR, OR THE CAP EATS THE WHOLE MODULE. A kit wall cut into a
        # dozen cells has cells 1.2-1.5 m long, so a 1.1 m cap culled nearly
        # all of them — and `fracture_prim` returns early on an empty list
        # WITHOUT deactivating the source, so the wall stayed whole and the
        # building came out as intact floor plates hanging in the air
        # (A_fin3_urm DG5). The cap is for the outliers; it may never take
        # more than half.
        floor = max(3, int(round(len(keep) * 0.5)))
        if len(under) < floor:
            order = list(np.argsort(ext))
            under = [keep[int(i)] for i in order[:floor]]
        keep = under

    for frag in keep:
        # SHRINK ABOUT THE CENTROID. Voronoi cells share exact boundaries, so
        # after roughening pushes vertices outward every fragment overlaps its
        # neighbours. PhysX resolves that penetration with a large separating
        # impulse and the pile detonates. A ~3% inset restores a hairline gap
        # so the solver starts from a legal configuration.
        c = np.asarray(frag.centroid, dtype=float)
        frag.vertices = c + (np.asarray(frag.vertices, dtype=float) - c) * shrink
        out.append(frag)

    if verbose:
        print("[fracture]   {0} seeds -> {1} fragments (src {2} faces, "
              "watertight={3})".format(len(pts), len(out), len(mesh.faces),
                                       mesh.is_watertight))
    return out


# ---------------------------------------------------------------------------
# TWO-SCALE FRACTURE ALONG A BREAK LINE
# ---------------------------------------------------------------------------
#
# THE PROBLEM THIS SOLVES. `fracture_partial` and the earthquake path's
# `_break_split` both fracture a piece once and then decide, CELL BY CELL,
# which cells come away. The surviving edge is therefore a chain of whole
# Voronoi cell walls — and a 20 m roof cut with the 10-15 seeds those callers
# can afford has 4-5 m cells, so the "ragged" edge is a chain of 4 m FLAT
# PLANES. From 20 m up that reads as a ruler-straight shear line with a few
# triangular flakes, which is exactly what the review called out.
#
# Refining the whole piece is not the fix: N seeds cost N*(N-1) plane cuts
# (before `_cell`'s early-out), and the part of the slab that did NOT break
# then comes out as a crack mosaic of small polygons — a different wrong look.
#
# So the cut is made at TWO SCALES. Coarse cells everywhere; then every cell
# the break line actually crosses is re-fractured on its own, with the seeds
# packed ALONG the line at `edge_cell_m`, and only then judged. What comes out
# is metre-scale cells away from the break and 0.3 m teeth at it, for the cost
# of a dozen small extra fractures. Two further touches finish the edge:
#
#   CHEW — a share of the small static cells touching the line come away
#   anyway (a bite out of the stub) and a share of the loose ones that
#   STRADDLE the line stay put (a piece left hanging). Without it the edge is
#   still a clean partition of the cells, just a finer one.
#
#   GAPS IN METRES — see `inset`. Statics butt with a 0.8 mm hairline so the
#   surviving slab reads as one slab, a `crack_frac` share of the cells at the
#   rim open a 1 cm crack that radiates out of the break, and only the loose
#   pieces take the full separation PhysX needs.
SPLIT_EDGE_CELL_M = 0.35     # tooth size at the break line
SPLIT_BAND_M = 0.9           # how far from the line the fine cells reach
SPLIT_EDGE_MAX = 16          # edge seeds per refined cell (prim-count ceiling)
SPLIT_REFINE_MAX = 12        # cells refined per call (time ceiling)


def _inside_convex(cell, P):
    """Boolean mask of the points of P inside a CONVEX cell, by its own face
    planes. Cheap and exact for a Voronoi cell; meaningless for the open
    shells the kit façades are, which is why the caller falls back to the
    bounding box when this keeps almost nothing."""
    try:
        fn = np.asarray(cell.face_normals, dtype=float)
        fc = np.asarray(cell.triangles_center, dtype=float)
        if not len(fn):
            return np.zeros(len(P), dtype=bool)
        off = np.einsum("ij,ij->i", fn, fc)
        return np.all(P.dot(fn.T) - off[None, :] <= 1e-6, axis=1)
    except Exception:
        return np.zeros(len(P), dtype=bool)


def _probe(cell, judge):
    """(n_true, n_samples) of `judge` over the cell: its centroid and its
    extreme vertices. A cell with a mixed verdict is one the break line
    crosses."""
    v = np.asarray(cell.vertices, dtype=float)
    if not len(v):
        return 0, 1
    if len(v) > 14:
        v = v[::max(1, len(v) // 14)]
    P = np.vstack([np.asarray(cell.centroid, dtype=float)[None, :], v])
    t = 0
    for q in P:
        if judge(q):
            t += 1
    return t, len(P)


def _refine_cell(cell, judge, rng, edge_cell_m, edge_max, sub_min,
                 samples=240, bulk=3):
    """Re-fracture ONE cell with the seeds packed along the break line."""
    lo, hi = cell.bounds
    P = rng.uniform(lo, hi, size=(int(samples), 3))
    ins = _inside_convex(cell, P)
    if int(ins.sum()) >= 24:
        P = P[ins]
    lab = np.array([bool(judge(q)) for q in P], dtype=bool)
    A, B = P[lab], P[~lab]
    if not len(A) or not len(B):
        return []
    near = float(edge_cell_m) * 2.0
    band = []
    for g, o in ((A, B), (B, A)):
        dd = np.linalg.norm(g[:, None, :] - o[None, :, :], axis=2).min(axis=1)
        if len(g[dd < near]):
            band.append(g[dd < near])
    if not band:
        return []
    band = np.vstack(band)
    # A POISSON PICK, NOT A COUNT. Spreading the edge seeds at `edge_cell_m`
    # fixes the TOOTH SIZE and lets the number follow from how much of the
    # line runs through this cell — a cell the line only clips gets three
    # seeds, one it runs the length of gets `edge_max`. Every extra cell is a
    # prim and, on the loose side, a rigid body, so the cap matters.
    sep = float(edge_cell_m) * 0.85
    keep = np.zeros((0, 3))
    for i in rng.permutation(len(band)):
        q = band[int(i)]
        if len(keep) and float(np.min(np.linalg.norm(keep - q, axis=1))) < sep:
            continue
        keep = np.vstack([keep, q[None, :]])
        if len(keep) >= int(edge_max):
            break
    if len(keep) < 2:
        return []
    rest = np.vstack([A, B])
    seeds = [keep[i] for i in range(len(keep))]
    for i in rng.permutation(len(rest))[:int(bulk)]:
        seeds.append(rest[int(i)])
    seeds = np.asarray(seeds, dtype=float)
    subs = []
    for i in range(len(seeds)):
        f = _cell(cell, seeds, i)
        if f is None or not len(f.faces):
            continue
        if float(np.prod(np.maximum(f.extents, 1e-9))) < sub_min:
            continue
        subs.append(f)
    return subs


def fracture_split(mesh, n_pieces, judge, rng,
                   edge_cell_m=SPLIT_EDGE_CELL_M, edge_max=SPLIT_EDGE_MAX,
                   refine_max=SPLIT_REFINE_MAX, refine=True,
                   rough_m=0.03, rough_lam_m=1.1,
                   gap_loose_m=0.008, gap_static_m=0.0008,
                   gap_crack_m=0.010, crack_frac=0.18,
                   chew_out=0.18, chew_in=0.12, edge_consume=0.5,
                   max_loose_m=None,
                   min_volume_frac=0.0015, mode="uniform", focus=None,
                   axis=None, aspect=None, verbose=False):
    """Fracture `mesh` and split the fragments by `judge(point) -> bool`
    (True = this point is on the side that COMES AWAY), resolving the break
    line at a second, much finer scale. Returns (static_meshes, loose_meshes).

    `judge` is evaluated in whatever frame the mesh is in — `prim_to_mesh`
    returns world space, so the earthquake judges work in world coordinates.
    """
    if mesh is None or not len(mesh.faces):
        return [], []
    bbox_vol = float(np.prod(np.maximum(mesh.extents, 1e-6)))
    keep_min = bbox_vol * float(min_volume_frac)
    sub_min = (0.35 * float(edge_cell_m)) ** 3

    pts = _seeds(mesh, int(n_pieces), rng, mode=mode, focus=focus,
                 axis=axis, aspect=aspect)
    field = noise_field(rng, amp_m=rough_m, lam_m=rough_lam_m)

    cells = []
    for i in range(len(pts)):
        f = _cell(mesh, pts, i)
        if f is None or not len(f.faces):
            continue
        if float(np.prod(np.maximum(f.extents, 1e-9))) < keep_min:
            continue
        cells.append(f)
    if not cells:
        return [], []

    straddle, plain = [], []
    for c in cells:
        t, n = _probe(c, judge)
        (straddle if 0 < t < n else plain).append((c, bool(t * 2 >= n)))
    # biggest straddlers first: their cell walls are the long straight ones,
    # and if the ceiling bites it should bite the cells that matter least.
    straddle.sort(key=lambda cb: -float(np.prod(np.maximum(cb[0].extents, 1e-9))))

    # (mesh, loose, at_edge, straddles)
    out = [(c, s, False, False) for c, s in plain]
    n_ref = 0
    for k, (c, side) in enumerate(straddle):
        subs = (_refine_cell(c, judge, rng, edge_cell_m, edge_max, sub_min)
                if (refine and k < int(refine_max)) else [])
        if not subs:
            out.append((c, side, True, True))
            continue
        n_ref += 1
        for s in subs:
            t, n = _probe(s, judge)
            out.append((s, bool(t * 2 >= n), True, 0 < t < n))

    statics, looses = [], []
    n_eaten = 0
    for msh, loose, at_edge, strad in out:
        if at_edge:
            if not loose and rng.random() < float(chew_out):
                loose = True          # a tooth bitten out of the stub
            elif loose and strad and rng.random() < float(chew_in):
                loose = False         # a piece left hanging over the break
        # EVERY REFINED LOOSE CELL IS A RIGID BODY. The refinement is there to
        # make the SURVIVING edge read; the pieces that came off it are debris
        # among a lot of other debris, and keeping all of them tripled the
        # body count of a bench (840 -> 2243 on one commercial row, settle
        # 42 s -> 99 s). A share of the small ones is simply not authored.
        if loose and at_edge and rng.random() < float(edge_consume):
            n_eaten += 1
            continue
        # AND NO 8 m SPLINTERS. `plank` seeding on a whole roof box produces
        # the odd cell that runs most of the length of the slab; on the ground
        # it reads as a scaffold board, not as debris, and it is exactly the
        # "large clean pale sheet" the DG5 review objected to. A loose piece
        # longer than `max_loose_m` is under the pile, so it is not authored.
        if (loose and max_loose_m
                and float(np.max(msh.extents)) > float(max_loose_m)):
            n_eaten += 1
            continue
        roughen_field(msh, field)
        if loose:
            inset(msh, gap_loose_m)
            looses.append(msh)
        else:
            inset(msh, gap_crack_m if (at_edge and rng.random() < float(crack_frac))
                  else gap_static_m)
            statics.append(msh)
    if verbose:
        print("[fracture]   split: {0} coarse ({1} crossed, {2} refined) -> "
              "{3} static + {4} loose ({5} edge chips dropped)".format(
                  len(cells), len(straddle), n_ref, len(statics), len(looses),
                  n_eaten))
    return statics, looses


def _face_normals(mesh):
    """faceVarying normals: the geometric normal of each triangle, three times.

    WITHOUT THESE A FRAGMENT IS SMOOTH-SHADED. A fractured piece has no
    authored normals, so Hydra averages them at the vertices and a flat cut
    face comes out as a pillow — and where a fractured STRIP butts against an
    authored `_box` remainder (which does carry faceVarying normals) the two
    shade differently, so the join shows as a thin light line straight across
    the roof even when the geometry matches to a millimetre. That line was
    still visible after the gap and the scar amplitude had both been taken
    down; it is a shading seam, not a crack.
    """
    from pxr import Gf, Vt
    v = np.asarray(mesh.vertices, dtype=float)
    f = np.asarray(mesh.faces)
    if not len(f):
        return None
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = n / np.maximum(ln, 1e-12)
    n = np.repeat(n, 3, axis=0)
    return Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in n])


def _write_mesh(stage, path, mesh, centre_on_centroid=True):
    """One trimesh -> one UsdGeom.Mesh, optionally centred on its centroid."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    v = np.asarray(mesh.vertices, dtype=float)
    c = np.asarray(mesh.centroid, dtype=float) if centre_on_centroid \
        else np.zeros(3)
    v = v - c
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in v]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(mesh.faces)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(
        [int(x) for x in np.asarray(mesh.faces).ravel()]))
    nrm = _face_normals(mesh)
    if nrm is not None:
        m.CreateNormalsAttr(nrm)
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(0))),
                        Gf.Vec3f(*map(float, v.max(0)))])
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(*c))
    return path


def fracture_partial(stage, prim_path, out_parent, n_pieces, rng,
                     cut_frac=0.5, mode="char", rough=0.035, shrink=0.97,
                     ragged=0.22, deactivate=True, consume=0.30, axis=None,
                     min_volume_frac=0.004, aspect=None, max_piece_m=None):
    """Break a wall so that only its upper part comes down.

    NO STRAIGHT CUT. The first version sliced the module at a flat plane and
    fractured what was above it, which left the surviving stub with a razor
    edge across the top — the one thing a failed wall never has.

    Instead the WHOLE wall is fractured, and each fragment is then judged
    against a break line that wanders with horizontal position: fragments whose
    centre sits below it stay as static geometry, the rest come down. The
    surviving edge is therefore made of real cell boundaries, so it is ragged
    by construction and no two walls fail alike.

    `ragged` is how much the break line wanders, as a fraction of the wall's
    height. 0 gives a level break; 0.22 gives a plausibly collapsed one.

    `consume` is exposed because the default is tuned for a TIMBER WALL, where
    a third of the material genuinely burns away. A standing trunk is not
    that: dropping 30% of a stub's cells punches holes through the middle of
    it and leaves the sections above hanging. Trunks pass something near zero.

    Returns `(static_paths, loose_paths)`.
    """
    mesh = prim_to_mesh(stage, prim_path)
    if mesh is None:
        return [], []

    frags = fracture_mesh(mesh, n_pieces, rng, mode=mode, rough=rough,
                          shrink=shrink, consume=consume, axis=axis,
                          aspect=aspect, min_volume_frac=min_volume_frac,
                          max_piece_m=max_piece_m)
    if not frags:
        return [], []

    lo, hi = mesh.bounds
    height = max(1e-6, float(hi[2] - lo[2]))
    base_z = float(lo[2] + height * float(cut_frac))
    # The wall's long axis — the break wanders along it.
    span = hi - lo
    axis = int(np.argmax(np.abs(span[:2])))
    a0, a1 = float(lo[axis]), float(hi[axis])
    phase = float(rng.random()) * 6.283
    freq = 1.4 + 2.2 * float(rng.random())

    def break_z(p):
        t = (float(p[axis]) - a0) / max(1e-6, a1 - a0)
        wobble = (math.sin(t * freq * 6.283 + phase) * 0.6
                  + math.sin(t * freq * 2.7 + phase * 1.7) * 0.4)
        return base_z + wobble * height * float(ragged)

    statics, loose = [], []
    for i, f in enumerate(frags):
        c = np.asarray(f.centroid, dtype=float)
        path = "{0}/frag_{1:03d}".format(out_parent, i)
        _write_mesh(stage, path, f)
        (statics if c[2] < break_z(c) else loose).append(path)

    # A stub with nothing above it is just an intact wall, and a wall with no
    # stub is a full collapse — neither is what this is for. If the break line
    # happened to fall outside the fragment spread, split by height instead.
    if not loose or not statics:
        order = sorted(range(len(frags)),
                       key=lambda i: float(frags[i].centroid[2]))
        keep = max(1, int(len(order) * float(cut_frac)))
        allp = ["{0}/frag_{1:03d}".format(out_parent, i) for i in range(len(frags))]
        statics = [allp[i] for i in order[:keep]]
        loose = [allp[i] for i in order[keep:]]

    if deactivate:
        src = stage.GetPrimAtPath(prim_path)
        if src and src.IsValid():
            src.SetActive(False)
    return statics, loose


def fracture_prim(stage, prim_path, out_parent, n_pieces, rng,
                  mode="uniform", rough=0.035, shrink=0.97,
                  deactivate=True, verbose=True, consume=0.30,
                  consume_pool=1.25, axis=None, min_volume_frac=0.004,
                  aspect=None, max_piece_m=None):
    """Fracture a placed module in the stage. Returns the new prim paths.

    Fragments are authored around their OWN centroid with the centroid in the
    transform, not baked into the points — otherwise every piece would rotate
    about the world origin once physics takes hold, which sends the pile into
    orbit.

    `min_volume_frac` IS EXPOSED FOR THIN PANELS. The cull compares a
    fragment's BOUNDING-BOX volume against this fraction of the source
    module's, which is generous to a stubby chunk and hostile to a sheet: a
    2.4 x 1.2 x 0.015 m piece of sheathing has a hundredth the bbox volume of
    a cube with the same longest edge, so the 0.004 default discards exactly
    the broken panels a wind-damage field is made of. `disaster.wind_flow`
    passes 0.0008.
    """
    from pxr import Gf, Sdf, UsdGeom, Vt

    mesh = prim_to_mesh(stage, prim_path)
    if mesh is None:
        if verbose:
            print("[fracture] no readable mesh under {0}".format(prim_path))
        return []
    frags = fracture_mesh(mesh, n_pieces, rng, mode=mode, rough=rough,
                          shrink=shrink, consume=consume,
                          consume_pool=consume_pool, axis=axis,
                          aspect=aspect, min_volume_frac=min_volume_frac,
                          max_piece_m=max_piece_m, verbose=verbose)
    if not frags:
        if verbose:
            print("[fracture] {0}: {1} faces, watertight={2}, extents={3} "
                  "-> no fragments".format(prim_path.rsplit("/", 1)[-1],
                                           len(mesh.faces), mesh.is_watertight,
                                           np.round(mesh.extents, 2)))
        return []

    UsdGeom.Scope.Define(stage, Sdf.Path(out_parent))
    made = []
    for i, f in enumerate(frags):
        c = np.asarray(f.centroid, dtype=float)
        v = np.asarray(f.vertices, dtype=float) - c
        path = "{0}/frag_{1:03d}".format(out_parent, i)
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(Vt.Vec3fArray(
            [Gf.Vec3f(*map(float, q)) for q in v]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(f.faces)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(
            [int(x) for x in np.asarray(f.faces).ravel()]))
        nrm = _face_normals(f)
        if nrm is not None:
            m.CreateNormalsAttr(nrm)
            m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        m.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(0))),
                            Gf.Vec3f(*map(float, v.max(0)))])
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(*c))
        made.append(path)

    if deactivate:
        # Not RemovePrim: the module is a reference and RemovePrim cannot
        # delete across one. Deactivating composes and is reversible.
        src = stage.GetPrimAtPath(prim_path)
        if src and src.IsValid():
            src.SetActive(False)
    return made
