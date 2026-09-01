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
    # ONE installer at a time: two bakes starting together in a fresh
    # container both ran this, one imported a half-installed mapbox_earcut,
    # and every slice in that process died with "No available triangulation
    # engine" (169 EMPTY modules in one style, 2026-08-27). The lock lives in
    # /tmp of the container; the loser re-checks the imports after waiting.
    import fcntl
    try:
        with open("/tmp/fracture_deps.lock", "w") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            still = []
            for m in missing:
                try:
                    __import__(m)
                except ImportError:
                    still.append(m)
            if still:
                subprocess.check_call(
                    [sys.executable, "-m", "pip", "install", "--no-cache-dir",
                     "--disable-pip-version-check", "-q"] + still)
            fcntl.flock(lk, fcntl.LOCK_UN)
    except Exception as exc:
        print("[fracture] could not install {0}: {1}".format(missing, exc))
        return False
    # prove the engine trimesh caps with is importable in THIS process
    for m in missing:
        try:
            __import__(m)
        except ImportError as exc:
            print("[fracture] {0} still not importable after install: {1}".format(m, exc))
            return False
    # AND PROVE TRIMESH CAN SEE IT. `trimesh.creation` resolves its
    # triangulation engines ONCE, at import, into a module-level `_engines`
    # list; a pip install that lands after trimesh was first imported changes
    # nothing, and every capped slice in the process then dies with
    # "ValueError: No available triangulation engine!" — 169 empty modules in
    # one style on 2026-08-27, and six more in the commercial re-bake at 09:11
    # where `shapely` was the package installed late. Purging trimesh from
    # sys.modules makes the next `import trimesh` (they are all function-local
    # in this file) re-resolve the engines.
    _reload_trimesh(verbose=verbose)
    return True


def _triangulation_ok():
    """True when trimesh has a live cap-triangulation engine RIGHT NOW."""
    try:
        import trimesh.creation as tc
    except Exception:
        return False
    eng = getattr(tc, "_engines", None)
    if eng is None:
        return True                     # a trimesh version without the table
    return any(bool(ok) for _, ok in eng)


def _reload_trimesh(verbose=True):
    """Drop trimesh (and the modules it probes) so the next import re-resolves
    its triangulation engines. Returns True when capping will work."""
    import importlib
    if _triangulation_ok():
        return True
    importlib.invalidate_caches()
    for name in [k for k in list(sys.modules)
                 if k == "trimesh" or k.startswith("trimesh.")
                 or k in ("shapely", "mapbox_earcut", "manifold3d")
                 or k.startswith("shapely.")]:
        sys.modules.pop(name, None)
    ok = _triangulation_ok()
    if verbose:
        print("[fracture] trimesh reloaded after install; cap engine {0}"
              .format("live" if ok else "STILL DEAD — caps will be skipped"))
    return ok





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
_N_CAPFAIL = 0
# `contour` = vtkContourTriangulator (correct for sections with holes, can
# hang on degenerate contours); `fan` = stripper + triangle filter (fast,
# fills a hole solid, never hangs). See the note in `_vtk_slice`.
_CAP = os.environ.get("FRACTURE_CAP", "fan").strip().lower()
_VTK = None

# ---------------------------------------------------------------------------
# THE VTK BOUNDARY IS NOT BOUNDS-CHECKED, AND THAT IS WHAT SEGFAULTED
# ---------------------------------------------------------------------------
#
# The one hard crash this module has produced took the whole GAC fire bench
# down three times on 2026-08-30 (SM_Building_09 F6). Kit's crash dump
# 70cfa653-7231-48aa-81e159b6-f5f287c9 has both halves of it:
#
#   py-spy      _vtk_slice (fracture.py:329)   <- `strip.Update()`
#               slice_plane <- fracture_mesh <- fracture_prim
#               <- urban_fire.r_fire_collapse   (a `_break` of a wall module,
#                                                NOT the roof-lid shatter)
#   native      001 vtkPolyData::GetPointCells(long long, vtkIdList*)
#               002 vtkStripper::RequestData(...)
#
# `vtkStripper` builds a link table sized by the polydata's POINT COUNT and
# then walks it with ids taken FROM THE CELLS. `vtkPolyData::GetPointCells`
# validates neither the id nor the table, so ONE cell naming a point the
# array does not have reads off the end of the heap. Reproduced exactly, and
# only, by that condition: `tools/_vtk_shell_probe.py`'s `oob` case — a
# stripper fed one line cell whose second id is `npoints + 5` — dies with that
# stack, while duplicate segments, degenerate segments, one-point cells,
# high-degree junctions, 2,560-segment contours and contours past
# `MaximumLength` all survive. Non-finite points do not crash it either, but
# they PROPAGATE: a NaN vertex comes back out of `vtkClipPolyData` as a NaN
# fragment (measured), and NaN in a mesh is what makes a later locator index
# wild in the first place.
#
# Nothing in this file is supposed to build such a mesh and none of the 254
# pieces of SM_Building_09 does when it is cut offline — but the failure mode
# is a SIGSEGV that takes an eight-minute bake with it, while the cost of the
# check is one numpy pass per cut and the cost of failing it is one fragment's
# cap. So both ends of the boundary are checked from here on: what goes into
# VTK (`_to_vtk`), what comes back (`_from_vtk`), and what the stripper is
# asked to walk (`_strip_input`). `FRACTURE_VTK_GUARD=0` restores the round-3
# behaviour for a comparison.
_VTK_GUARD = os.environ.get("FRACTURE_VTK_GUARD", "1").strip() not in (
    "0", "false", "no")
_N_GUARD = 0                 # how many times the guard has actually fired


def _guard_note(what, n):
    """Report the first few guard hits. Silence would hide a real defect
    upstream — the guard is a seatbelt, not a fix for whoever built the mesh."""
    global _N_GUARD
    _N_GUARD += 1
    if _N_GUARD <= 5:
        print("[fracture] VTK guard: dropped {0} {1} (a segfault avoided; "
              "see the note above `_VTK_GUARD`)".format(n, what))


def _vtk_arrays(mesh):
    """(vertices, faces) that `vtkStripper` cannot walk off the end of.

    Drops, in this order: faces with a negative or out-of-range index, faces
    that name the same vertex twice (a zero-area cell the cutter turns into a
    degenerate contour segment), and faces touching a non-finite point.
    Returns `(v, f)` with `f` possibly empty. On clean input — every kit
    module and every sliced GAC piece measured — NOTHING is dropped and the
    arrays are the caller's own, so the polydata built from them is identical
    to what the unguarded code built.
    """
    v = np.ascontiguousarray(np.asarray(mesh.vertices, dtype=np.float64))
    f = np.asarray(mesh.faces, dtype=np.int64)
    if not _VTK_GUARD or not len(f) or not len(v):
        return v, f
    n = len(v)
    # FAST PATH FIRST — this runs on EVERY cut, and a Voronoi fracture is
    # hundreds to thousands of them. Five scalar reductions (no temporary
    # boolean arrays, no per-face mask) answer "is anything wrong at all";
    # only a mesh that fails one of them pays for the full diagnosis. Measured
    # over 254 pieces x 8 seeds: no measurable change in wall time.
    if (int(f.min()) >= 0 and int(f.max()) < n
            and not (f[:, 0] == f[:, 1]).any()
            and not (f[:, 1] == f[:, 2]).any()
            and not (f[:, 2] == f[:, 0]).any()
            and np.isfinite(v.sum())):
        return v, f
    ok = ((f >= 0).all(axis=1) & (f < n).all(axis=1)
          & (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 2] != f[:, 0]))
    if not np.isfinite(v).all():
        bad = ~np.isfinite(v).all(axis=1)
        # `f` may already be out of range here, so index the mask safely
        ok &= ~bad[np.clip(f, 0, max(n - 1, 0))].any(axis=1)
    if ok.all():
        return v, f
    _guard_note("face(s) VTK cannot index", int((~ok).sum()))
    return v, f[ok]


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

    v, f = _vtk_arrays(mesh)          # see the `_VTK_GUARD` note above
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
    f = np.asarray(f, dtype=np.int64)
    if _VTK_GUARD and len(f):
        # THE OTHER END OF THE BOUNDARY. A polydata that came back naming a
        # point it does not have would be handed straight back to `_to_vtk` by
        # the next cut in `_cell`, and that is the mesh `vtkStripper` dies on.
        # Catch it where it is produced, not two cuts later.
        if int(f.min()) < 0 or int(f.max()) >= len(v):
            ok = (f >= 0).all(axis=1) & (f < len(v)).all(axis=1)
            _guard_note("face(s) VTK returned naming a point it did not make",
                        int((~ok).sum()))
            f = f[ok]
            if not len(f):
                return None
    return trimesh.Trimesh(vertices=np.asarray(v, dtype=float),
                           faces=f, process=False)


def _strip_input(pd):
    """The contour `vtkStripper` may walk, or None to skip the cap.

    TWO JOBS, AND THE FIRST ONE IS THE CRASH (see the `_VTK_GUARD` note):

      1. REFUSE what makes `vtkPolyData::GetPointCells` read off the end of the
         link table — a line naming a point the output does not have, or a
         non-finite point (which is how a locator index goes wild in the first
         place). The cost is one numpy reduction over the contour, which is two
         orders of magnitude smaller than the mesh that produced it; the cost
         of failing is ONE fragment's cut face, against a SIGSEGV that takes
         the whole bake.

      2. DROP degenerate and DUPLICATE segments, and only if there are any.
         `vtkCutter` emits one segment per crossed triangle, so two triangles
         lying in the same place — GAC models its posters and signs as
         separate quads ON the wall plane, and `gac_storey_slice.clip` had to
         defeat a merging locator for exactly that reason — give the same
         segment twice. MEASURED on a 64-segment ring
         (`tools/_vtk_shell_probe.py`, case `dup_segs`): duplicated, the
         stripper returns ONE polyline of nonsense and the fan cap comes back
         with ZERO triangles instead of 62. So this is a cap that works rather
         than a cap that silently vanishes — and it removes the last contour
         topology the stripper is visibly wrong on.

    Returns `pd` ITSELF when the contour is already clean, which every kit
    module and every sliced GAC piece measured is, so the stripper sees byte
    for byte what it saw before this function existed.
    """
    import vtk
    from vtk.util import numpy_support as ns

    if pd is None or pd.GetNumberOfPoints() <= 0:
        return None
    lines = pd.GetLines()
    if lines is None or lines.GetNumberOfCells() == 0:
        return None
    if not _VTK_GUARD:
        return pd
    try:
        conn = ns.vtk_to_numpy(lines.GetConnectivityArray())
        off = ns.vtk_to_numpy(lines.GetOffsetsArray())
    except Exception:
        return pd                    # a VTK without the split cell arrays
    npts = int(pd.GetNumberOfPoints())
    if len(conn) and (int(conn.max()) >= npts or int(conn.min()) < 0):
        _guard_note("contour segment(s) naming a point the cut did not make",
                    int(len(conn)))
        return None
    try:
        xyz = ns.vtk_to_numpy(pd.GetPoints().GetData())
    except Exception:
        xyz = None
    if xyz is not None and len(xyz) and not np.isfinite(xyz).all():
        _guard_note("non-finite contour point(s)",
                    int((~np.isfinite(xyz).all(axis=1)).sum()))
        return None
    # -- 2. duplicates, ONLY on the all-2-point contour a cutter produces
    if len(off) < 2 or not np.all(np.diff(off) == 2):
        return pd
    seg = conn.reshape(-1, 2)
    key = np.sort(seg, axis=1)
    good = key[:, 0] != key[:, 1]
    if good.all():
        _u, first = np.unique(key, axis=0, return_index=True)
        if len(first) == len(seg):
            return pd                              # already clean: unchanged
        keep = np.zeros(len(seg), dtype=bool)
        keep[first] = True
    else:
        _u, first = np.unique(key[good], axis=0, return_index=True)
        idx = np.where(good)[0][first]
        keep = np.zeros(len(seg), dtype=bool)
        keep[idx] = True
    seg = seg[keep]
    _guard_note("duplicate/degenerate contour segment(s)",
                int((~keep).sum()))
    if not len(seg):
        return None
    cells = np.hstack([np.full((len(seg), 1), 2, dtype=np.int64),
                       seg.astype(np.int64)]).ravel()
    ca = vtk.vtkCellArray()
    ca.SetCells(len(seg), ns.numpy_to_vtkIdTypeArray(
        np.ascontiguousarray(cells), deep=True))
    out = vtk.vtkPolyData()
    out.SetPoints(pd.GetPoints())
    out.SetLines(ca)
    return out


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
        filled = None
        # vtkContourTriangulator IS THE RIGHT FILTER AND IT IS NOT THE DEFAULT.
        # It fills nested closed contours with the odd-even rule, so a section
        # with a HOLE in it — a cut through a wall with a window, or through
        # any solidified module, whose section is a ring — comes out as a ring
        # instead of the outer loop filled solid PLUS the inner loop filled
        # solid on top, which is what `vtkStripper` + `vtkTriangleFilter`
        # gives. On clean input it also costs the same: measured over 4 cuts
        # of a commercial wall module, 0.5 ms/cap either way.
        #
        # BUT IT CAN HANG. On the first solid bench (T_sol1_urm / T_sol1_rc,
        # 2026-08-27) both runs sat in `ct.Update()` for 13+ minutes on a
        # single cut — py-spy has them at `_vtk_slice` line 305 sample after
        # sample — where the equivalent round-2 column took 41 s END TO END.
        # The trigger is a DEGENERATE contour: the flat back plane a solidified
        # module is built on makes coincident and self-overlapping section
        # segments where a bay passes in front of the wall behind it, and the
        # triangulator's loop assembly does not terminate usefully on those.
        # There is no way to time-box a VTK `Update()`, so it is opt-in:
        # FRACTURE_CAP=contour to use it, `fan` (the default) for the filter
        # that cannot hang.
        if _CAP == "contour":
            try:
                ct = vtk.vtkContourTriangulator()
                ct.SetInputConnection(cut.GetOutputPort())
                ct.Update()
                out = ct.GetOutput()
                if out.GetNumberOfPolys():
                    filled = out
            except Exception:
                filled = None
        if filled is None:
            # `SetInputData`, NOT `SetInputConnection`: the stripper must walk
            # EXACTLY the polydata `_strip_input` just validated, not whatever
            # a re-pull of the cutter's pipeline would hand it. The cutter is
            # already up to date (`cut.Update()` above), so the result is the
            # same and the check is no longer advisory.
            src = _strip_input(cut.GetOutput())
            loops = None
            if src is not None:
                strip = vtk.vtkStripper()
                strip.SetInputData(src)
                strip.Update()
                loops = strip.GetOutput()
            if loops is not None and loops.GetNumberOfLines():
                face = vtk.vtkPolyData()
                face.SetPoints(loops.GetPoints())
                face.SetPolys(loops.GetLines())
                fill = vtk.vtkTriangleFilter()
                fill.SetInputData(face)
                fill.Update()
                filled = fill.GetOutput()
        if filled is not None and filled.GetNumberOfPolys():
            app = vtk.vtkAppendPolyData()
            app.AddInputData(kept)
            app.AddInputData(filled)
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
    try:
        return slice_mesh_plane(mesh, plane_normal=normal, plane_origin=origin,
                                cap=cap)
    except Exception as exc:
        # AN UNCAPPED FRAGMENT BEATS NO FRAGMENT. The cap is what needs
        # shapely + earcut, and when the engine is dead (or the section
        # polygon is degenerate) the exception used to propagate, every cell
        # of the module came back None, `fracture_prim` returned [] and — this
        # is the damaging part — returned WITHOUT deactivating the source, so
        # the wall stayed whole in an overturn. Retry once without the cap:
        # the piece is then open on its cut face, which is a worse-looking
        # fragment and a far better outcome than an undamaged building.
        if not cap:
            raise
        global _N_CAPFAIL
        _N_CAPFAIL += 1
        if _N_CAPFAIL <= 3:
            print("[fracture] cap failed ({0}); slicing uncapped".format(exc))
        return slice_mesh_plane(mesh, plane_normal=normal,
                                plane_origin=origin, cap=False)


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
        # A NON-FINITE PRIM IS DROPPED, NOT MERGED. A zero or NaN component in
        # a composed transform (a referenced Xform asset whose op the caller
        # authored from a solver result, say) turns the whole merged mesh into
        # NaN, every extent and centroid with it, and the first VTK locator
        # that hashes those coordinates indexes wherever the cast lands — the
        # class of defect that ends in the SIGSEGV described above `_to_vtk`.
        # One prim lost beats a fracture that cannot be reasoned about.
        if not np.isfinite(v.sum()):
            print("[fracture] {0}: non-finite points after transform; that "
                  "prim is dropped".format(str(p.GetPath())))
            continue
        verts.append(v)
        faces.append(f + base)
        base += len(v)

    if not verts:
        return None
    mesh = trimesh.Trimesh(vertices=np.vstack(verts),
                           faces=np.vstack(faces), process=True)
    return mesh if len(mesh.faces) else None


# ---------------------------------------------------------------------------
# SOLIDIFY — close an open shell into a solid before it is cut
# ---------------------------------------------------------------------------
#
# WHAT THE KIT ART ACTUALLY IS. Round 3 started from "the walls have zero
# thickness". They do not — `tools/_t_shell_probe.py` on `bld_commercial_DG0`
# and a ray probe through each module say:
#
#   * every one of the 114 meshes is OPEN (`is_watertight` False), which is why
#     the pipeline believed they were sheets;
#   * but a wall module is DOUBLE-SIDED: on the commercial kit wall, 27.8 m2 of
#     face points out (y -9.37..-8.58, the relieved façade) and 28.0 m2 points
#     in (y -8.58..-8.30). Measured wall thickness, median 0.14-0.68 m; the
#     plain storey band is exactly 0.70 m. The material is THERE;
#   * what is missing is the RIM. 134 of 4433 edges on the main wall component
#     are used by one face only — the top, bottom and side edges are never
#     closed, and `trimesh.repair.fill_holes` will not close them (it only
#     handles 3- and 4-vertex holes; measured: `ok=False`, still not
#     watertight).
#
# WHY AN OPEN RIM WRECKS THE FRACTURE. `slice_plane(cap=True)` caps a cut by
# taking the cross-section and triangulating it. On a closed mesh that section
# is a closed loop and the cap is the real cut face. On a mesh with an open rim
# it is a set of OPEN POLYLINES, and vtkStripper + vtkTriangleFilter fan them
# into enormous flat triangles that span the whole fragment. That is what the
# round-2 fragments are: renders of twelve cells off one commercial wall show
# every one carrying two or three 2-3 m triangular sheets and `is_watertight`
# False on all twelve. It is also, precisely, the user's round-3 note that the
# breakage "looks very TRIANGULAR" — those triangles are not a break pattern,
# they are failed caps.
#
# WHAT THIS DOES, per connected component:
#
#   CAP    the component is a closed volume with a few small holes (a column
#          with open ends, a cornice band open where it was cut from the strip)
#          -> fan-fill each boundary loop and keep the real geometry, real
#          thickness and real window reveals. Accepted only when the caps come
#          to less than `cap_area_frac` of the component: capping the module's
#          whole front rim would hang a 4 x 7 m sheet across the façade.
#
#   EXTRUDE  the component is a sheet, or its rim is too big to cap -> keep the
#          FRONT surface (the faces pointing away from `ref`), translate a copy
#          of it onto a plane `thickness_m` behind the innermost front point,
#          and stitch the two with side quads along every boundary edge. The
#          façade relief is kept exactly; the openings keep their holes and
#          their loops become REVEAL faces `thickness_m` deep, which is what
#          makes a broken window edge show wall depth; the back is flat, so it
#          cannot fold through itself the way a normal-offset does (measured:
#          a per-vertex normal offset on this art produced 0.9 m spikes out of
#          the top and bottom rims, because 65% of the wall's vertices sit on
#          folds sharper than 60 degrees).
#
#   DROP   too small to matter and not closeable. It has to be dropped, not
#          left: ONE open sheet anywhere in the mesh puts open polylines back
#          into every cut and the fan caps come back with them.
#
# THICKNESS IS NOT ONE NUMBER. A Boston/NYC URM bearing wall is 3-4 wythes at
# ~110 mm a wythe (_plans/earthquake_research.md, damage 12: "outer wythe
# ~110 mm thick"), so 0.33-0.44 m; a parapet is one or two wythes; an RC infill
# panel is a single block leaf ~0.20 m; a floor slab is 150-250 mm (research
# §3.9); and GLASS IS 6-12 mm — a pane must stay a plate or a curtain wall
# turns into a brick wall. The role table lives in `quake_flow`, which is where
# the role is known; this module only takes metres.


def _weld(mesh, tol=1e-5):
    """Merge vertices by position. Kit art splits them at every UV seam, so a
    wall that is one surface arrives as a thousand loose triangles and EVERY
    edge looks like a boundary edge."""
    import trimesh
    v = np.asarray(mesh.vertices, dtype=float)
    f = np.asarray(mesh.faces, dtype=np.int64)
    key = np.round(v / float(tol)).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True,
                              return_inverse=True)
    inv = np.asarray(inv).ravel()
    nf = inv[f]
    ok = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 2] != nf[:, 0])
    return trimesh.Trimesh(vertices=v[first], faces=nf[ok], process=False)


def _labels(faces, n_vert):
    """Connected-component label per face, by shared vertex. Union-find, so it
    needs neither scipy nor networkx — neither is guaranteed in the Kit python
    and `trimesh.graph` silently needs one of them."""
    parent = np.arange(int(n_vert), dtype=np.int64)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for tri in faces:
        r = find(int(tri[0]))
        for k in (1, 2):
            q = find(int(tri[k]))
            if q != r:
                parent[q] = r
    roots = np.array([find(int(t[0])) for t in faces], dtype=np.int64)
    _, lab = np.unique(roots, return_inverse=True)
    return np.asarray(lab).ravel()


def _fix_winding(faces):
    """Flip faces until neighbours traverse their shared edge in OPPOSITE
    directions — the definition of a consistently wound surface. Plain BFS
    over edge adjacency; numpy + dicts, no scipy, no networkx."""
    f = np.array(faces, dtype=np.int64, copy=True)
    edge = {}
    for i, tri in enumerate(f):
        a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
        for u, w in ((a, b), (b, c), (c, a)):
            edge.setdefault((u, w) if u < w else (w, u), []).append(i)
    seen = np.zeros(len(f), dtype=bool)

    def travels(i, key):
        tri = f[i]
        a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
        for u, w in ((a, b), (b, c), (c, a)):
            if (u, w) == key or (w, u) == key:
                return u < w
        return None

    for start in range(len(f)):
        if seen[start]:
            continue
        seen[start] = True
        stack = [start]
        while stack:
            i = stack.pop()
            tri = f[i]
            a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
            for u, w in ((a, b), (b, c), (c, a)):
                key = (u, w) if u < w else (w, u)
                mine = u < w
                for j in edge.get(key, ()):
                    if j == i or seen[j]:
                        continue
                    seen[j] = True
                    if travels(j, key) == mine:      # same way round: flipped
                        f[j] = f[j][::-1]
                    stack.append(j)
    return f


def _boundary(faces):
    """The DIRECTED boundary edges of a face set — those used by one face."""
    de = np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    key = np.sort(de, axis=1)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True,
                            return_counts=True)
    return de[cnt[np.asarray(inv).ravel()] == 1]


def _loops(bound):
    """Chain directed boundary edges into cycles. Non-manifold rims give short
    broken chains; those are returned too and the caller judges them."""
    nxt = {}
    for a, b in bound:
        nxt.setdefault(int(a), []).append(int(b))
    out, used = [], set()
    for a0 in list(nxt):
        for b0 in list(nxt.get(a0, ())):
            if (a0, b0) in used:
                continue
            loop, a, b = [a0], a0, b0
            while True:
                used.add((a, b))
                loop.append(b)
                if b == a0:
                    break
                nx = [q for q in nxt.get(b, ()) if (b, q) not in used]
                if not nx:
                    break
                a, b = b, nx[0]
                if len(loop) > 20000:
                    break
            if loop[0] == loop[-1] and len(loop) > 3:
                out.append(loop[:-1])
    return out


def _fan(v, loop):
    """Triangle fan from a loop's centroid. Faces + the one new vertex."""
    c = v[loop].mean(0)
    n = len(v)
    tri = [(loop[i], loop[(i + 1) % len(loop)], n) for i in range(len(loop))]
    return np.asarray(tri, dtype=np.int64), c


def _tri_area(v, f):
    if not len(f):
        return 0.0
    return float(np.linalg.norm(np.cross(v[f[:, 1]] - v[f[:, 0]],
                                         v[f[:, 2]] - v[f[:, 0]]),
                                axis=1).sum() * 0.5)


def _front_axis(v, f, ref):
    """The component's OUTWARD axis: the dominant direction of its face
    normals (the largest eigenvector of the area-weighted normal covariance —
    for a slab that is the slab normal, for a sill it is up), oriented away
    from `ref`. A near-vertical axis with no useful `ref` cue points UP, so a
    sill or a cornice thickens downward, which is the way one hangs."""
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    a = np.linalg.norm(n, axis=1)
    nh = n / np.maximum(a[:, None], 1e-16)
    M = (nh[:, :, None] * nh[:, None, :] * a[:, None, None]).sum(0)
    w, vec = np.linalg.eigh(M)
    u = vec[:, int(np.argmax(w))]
    u = u / max(float(np.linalg.norm(u)), 1e-12)
    c = v[np.unique(f)].mean(0)
    d = 0.0 if ref is None else float(np.dot(u, np.asarray(ref, dtype=float) * -1.0 + c))
    if abs(d) < 1e-3:
        d = float(u[2])          # vertical-ish and no cue: outward = up
    return u if d > 0 else -u


def _extrude_front(v, f, u, t, front_cos=0.10, flip_back=False):
    """Front faces -> a solid `t` deep, its back on a single flat plane.

    A FLAT BACK, not a normal offset. The kit walls are relieved by up to
    0.8 m; offsetting every vertex along its own normal makes the offset
    surface fold through itself at every sharp fold (65% of this wall's
    vertices), and mitreing the folds fires 0.9 m spikes out of the rim. A
    common back plane cannot self-intersect, and the piece comes out at least
    `t` thick everywhere and thicker under a projecting bay — which is what a
    bay is."""
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    nh = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-16)
    d = nh @ u
    ff = f[d > float(front_cos)]
    if len(ff) < 2:
        # NO OUTWARD FACE AT ALL. Either the component is the INNER lining of a
        # wall whose outer sheet is a component of its own (the storey band is
        # literally two parallel quads 0.70 m apart) — in which case it must be
        # dropped, because the outer sheet already becomes the solid and a
        # second slab would sit in the room behind it — or the art is simply
        # wound inward and this sheet IS the façade. The caller resolves that:
        # it drops these on the first pass and only flips them if the WHOLE
        # module came out empty.
        if not flip_back:
            return None, None
        ff = f[d < -float(front_cos)][:, ::-1]
        if len(ff) < 2:
            return None, None
    vi = np.unique(ff)
    s = v @ u
    back = float(s[vi].min()) - float(t)
    v2 = v + u[None, :] * (back - s)[:, None]
    nv = len(v)
    faces = [ff, ff[:, ::-1] + nv]
    b = _boundary(ff)
    if len(b):
        a0, b0 = b[:, 0], b[:, 1]
        faces.append(np.column_stack([a0, a0 + nv, b0 + nv]))
        faces.append(np.column_stack([a0, b0 + nv, b0]))
    return np.vstack([v, v2]), np.vstack(faces)


SOLID_MIN_AREA = 0.03        # m2: below this a component is dropped, not
                             # solidified — a bolt head is not a wall, and an
                             # open one left in would break every cut cap
SOLID_CAP_FRAC = 0.20        # fan caps may not exceed this share of the
                             # component's area (above it the "hole" is the
                             # module's own front rim, and capping it hangs a
                             # sheet across the façade)
SOLID_FRONT_COS = 0.10       # a face counts as front-facing above this
SOLID_FEATURE_FRAC = 0.9     # never thicker than this times the component's
                             # own second-smallest extent


def is_shell(mesh, min_open=1):
    """True when the mesh still has boundary edges after welding — i.e. it is
    open, so cutting it produces fan caps rather than real cut faces."""
    if mesh is None or not len(mesh.faces):
        return False
    m = _weld(mesh)
    f = np.asarray(m.faces, dtype=np.int64)
    return len(f) > 0 and len(_boundary(f)) >= int(min_open)


def solidify(mesh, thickness_m, inward=True, ref=None,
             min_area_m2=SOLID_MIN_AREA, cap_area_frac=SOLID_CAP_FRAC,
             front_cos=SOLID_FRONT_COS, feature_frac=SOLID_FEATURE_FRAC,
             weld_tol=1e-5, verbose=False):
    """Close an open shell into a solid at least `thickness_m` thick.

    CONTRACT (agents P and G build on this):
      * ANY mesh, any thickness, world units in metres. A mesh that is already
        watertight comes back UNCHANGED (`is` the same object) — that is how
        the authored `_box` slabs pass through. `thickness_m <= 0` also returns
        it unchanged, which is how a caller disables the whole thing.
      * The OUTER surface never moves. Whatever the component's front faces
        were, they are still exactly where they were, with their relief, their
        openings and their UVs; the new material is added behind them. A
        solidified module therefore still lines up with the intact one beside
        it.
      * `ref` is a point INSIDE the building (a mass centre is what
        `quake_flow` passes). Every component is thickened away from it. Each
        component decides independently, so a sill thickens downward while the
        wall beside it thickens inward. `ref=None` falls back to each
        component's own dominant normal, which for kit art is arbitrary — pass
        a ref for anything whose outer face has to stay put.
      * WINDOW AND DOOR OPENINGS SURVIVE as openings, and their boundary loops
        become reveal faces `thickness_m` deep. That is the point: a broken
        opening then shows wall depth instead of a paper edge.
      * Components below `min_area_m2`, and components that cannot be closed,
        are DROPPED. They cannot be passed through: one open sheet anywhere
        puts the fan caps back into every cut.
      * Returns a new trimesh. The input is not modified. `.is_watertight` is
        normally True; it can be False on art with non-manifold rims, and the
        slicing path tolerates that (see `fracture_mesh`) — but the caps get
        worse the further from watertight it is, so `verbose=True` reports it.
    """
    import trimesh

    t = float(thickness_m)
    if mesh is None or not len(mesh.faces) or t <= 0.0:
        return mesh
    if mesh.is_watertight:
        return mesh

    m = _weld(mesh, tol=weld_tol)
    v = np.asarray(m.vertices, dtype=float)
    f = np.asarray(m.faces, dtype=np.int64)
    if not len(f):
        return mesh
    # WINDING FIRST. `_extrude_front` picks the front faces by the sign of
    # `normal . outward`, and that sign comes from the face winding — so a
    # component with one flipped triangle loses that triangle from the front
    # set and comes out with a hole in it, which is the one defect that puts
    # the fan caps back. Measured 0 disagreeing pairs on the commercial kit
    # wall (the art is clean), but `prim_to_mesh` welds several placed prims
    # into one mesh and nothing guarantees they agree.
    f = _fix_winding(f)
    lab = _labels(f, len(v))

    def build(flip_back):
        V, F = [v], []
        st = dict(cap=0, ext=0, keep=0, drop=0, a_drop=0.0)
        for c in np.unique(lab):
            fc = f[lab == c]
            area = _tri_area(v, fc)
            b = _boundary(fc)
            if not len(b):                    # already closed — a real solid
                F.append(fc)
                st["keep"] += 1
                continue
            if area < float(min_area_m2):
                st["drop"] += 1
                st["a_drop"] += area
                continue
            # -- CAP: a volume with holes in it. Cheap, and it keeps the real
            #    geometry, so it is tried first.
            loops = _loops(b)
            caps, cverts, cap_area = [], [], 0.0
            for lp in loops:
                tri, cen = _fan(v, lp)
                tri = tri.copy()
                tri[tri == len(v)] = len(v) + len(cverts)
                cverts.append(cen)
                caps.append(tri)
                cap_area += _tri_area(np.vstack([v, np.asarray(cverts)]), tri)
            if (loops and sum(len(lp) for lp in loops) == len(b)
                    and cap_area <= float(cap_area_frac) * area):
                base = sum(len(q) for q in V)
                caps = [q + np.where(q >= len(v), base - len(v), 0) for q in caps]
                V.append(np.asarray(cverts, dtype=float))
                F.append(fc)
                F += caps
                st["cap"] += 1
                continue
            # -- EXTRUDE: a sheet, or a rim too big to cap.
            vi = np.unique(fc)
            ext = np.sort(v[vi].max(0) - v[vi].min(0))
            tc = min(t, float(feature_frac) * max(float(ext[1]), 1e-3))
            u = _front_axis(v, fc, ref)
            vv, ffc = _extrude_front(v, fc, u, tc, front_cos=front_cos,
                                     flip_back=flip_back)
            if ffc is None:
                st["drop"] += 1
                st["a_drop"] += area
                continue
            base = sum(len(q) for q in V)
            ffc = np.where(ffc >= len(v), ffc - len(v) + base, ffc)
            V.append(vv[len(v):])
            F.append(ffc)
            st["ext"] += 1
        return V, F, st

    V, F, st = build(False)
    if not F:
        # EVERY component faced the wrong way: the art is wound inward, so the
        # sheets ARE the façade. Redo, flipping them.
        V, F, st = build(True)
    if not F:
        return mesh
    # process=False ON PURPOSE. Merging by position would weld the back
    # vertices of two front faces that sit at DIFFERENT depths but the same
    # position on the flat back plane — which is every relief ridge and every
    # bay on this art — and that turns a clean closed surface into a
    # non-manifold one (measured on the commercial wall: 0 open edges and
    # `is_watertight` still False, purely from edges shared by four faces).
    # Only the degenerate triangles are dropped.
    VV = np.vstack(V)
    FF = np.vstack(F)
    a = np.linalg.norm(np.cross(VV[FF[:, 1]] - VV[FF[:, 0]],
                                VV[FF[:, 2]] - VV[FF[:, 0]]), axis=1)
    FF = FF[a > 1e-10]
    out = trimesh.Trimesh(vertices=VV, faces=FF, process=False)
    if not len(out.faces):
        return mesh
    if verbose:
        print("[fracture]   solidify {0:.3f} m: {1} -> {2} faces; {3} parts "
              "({4} capped, {5} extruded, {6} already closed, {7} dropped "
              "= {8:.2f} m2); watertight {9}".format(
                  t, len(mesh.faces), len(out.faces), len(np.unique(lab)),
                  st["cap"], st["ext"], st["keep"], st["drop"], st["a_drop"],
                  out.is_watertight))
    return out


# ---------------------------------------------------------------------------
# _p_  ROUND 3: SEEDING FROM THE MECHANISM, NOT FROM A RANDOM CLOUD
# ---------------------------------------------------------------------------
#
# THE DIAGNOSIS (agent R, _plans/eq_round3_R.md §0). A 3-D Voronoi of random
# seeds cut out of a wall whose thickness (0.02-0.4 m) is far under the cell
# pitch (0.3-1.0 m) can only produce THIN POLYGONAL PLATES with acute corners
# — the "TRIANGULAR" look the user reported. Nothing about brick or concrete
# produced it; the seeder did. Measured demolition debris is BLOCKY: mean
# Flakiness Index 7.3-12.9 %, Elongation Index 13.1-23.9 %, 55-74 %
# equidimensional, and "almost no particles that can be characterised as
# blades". The two legitimate triangles are both METRES across (an infill
# quadrant cut by a stair-stepped X, a wall-corner rocking wedge).
#
# So the seeds have to sit on the planes of weakness:
#   `brick`  — a running-bond lattice. Cell faces land on MORTAR JOINTS,
#              because the pitch is GIVEN (a brick has a size) rather than
#              solved from the seed count.
#   `prism`  — ONE seed layer at the mid-plane of a member, so every cell
#              spans the full thickness and every cut face is normal to the
#              surface. A mode-I crack in a plate runs through the thickness;
#              it does not shave a flake off the middle of it.
#
# Both are lattices with SMALL jitter, and the size variety comes from site
# DROPOUT (`keep`) — a cell next to a deleted site absorbs it exactly, so what
# comes out is 2-8-brick clusters among single bricks rather than a machined
# grid. That is `plank`'s trick, and it is the only one of `plank`'s that
# survives here: `plank`'s 0.34-of-pitch jitter would move a seed most of a
# brick and destroy the courses.

# US modular brick 194 x 92 x 57 mm actual with a 9.5 mm (3/8 in) joint, and
# heritage (NZ/UK/AU) 230 x 110 x 76 mm — research §11. (stretcher, course,
# wythe) pitch in metres: three modular courses are 8 in exactly.
BRICK_MODULAR = (0.203, 0.0677, 0.092)
BRICK_HERITAGE = (0.240, 0.086, 0.110)
BRICK_JITTER = 0.10          # x the BRICK pitch, not the cell pitch. Half a
                             # mortar joint; anything more interpenetrates the
                             # units and the courses stop reading.
BRICK_KEEP = 0.62            # share of lattice sites that survive (R: 0.55-0.70)
BRICK_CAP = 1.7              # cells may overshoot the caller's budget by this
                             # much before the cluster is grown. A TRUE single
                             # -brick lattice on one 4 x 3 m module is ~3500
                             # cells = 3500 rigid bodies; see `_p_brick_seeds`.


def _p_axes(span, thin=None):
    """(thin, course, length) axis indices for a wall-like box: the thinnest
    axis is the wythe direction, Z is the course direction if it is not the
    thin one, and what is left runs along the wall."""
    t = int(np.argmin(span)) if thin is None else int(thin)
    rest = [a for a in range(3) if a != t]
    c = 2 if 2 in rest else int(rest[int(np.argmin([span[a] for a in rest]))])
    l = [a for a in rest if a != c][0]
    return t, c, l


BRICK_BLOCKY = 1.5           # a cluster may be at most this many times the
                             # wall's own thickness across, so a "brick
                             # cluster" cannot come out a plate
BRICK_NMAX = 2.4             # ...but only as far as this multiple of the
                             # caller's budget. Past it the body count wins.


def _p_brick_seeds(mesh, n, rng, pitch=None, keep=None, jitter=None,
                   leaves=1, cap=BRICK_CAP, blocky_m=None,
                   blocky=BRICK_BLOCKY, n_max=BRICK_NMAX):
    """A RUNNING-BOND lattice. Cells are whole brick clusters; their faces are
    mortar joints.

    THE PITCH IS GIVEN, NOT SOLVED. That is the whole point and it is the one
    line that separates this from `plank`: a brick is 203 x 67.7 mm whatever
    the caller's seed budget is, so the cell edges are `m x half-stretcher` by
    `k x course` — integer multiples — and a break through the lattice is a
    STAIRCASE quantised to the grid rather than a continuous wobble.

    WHAT IS CAPPED, AND WHY (be blunt about this — it is the one honest
    compromise in the mode). One kit wall module is ~4 x 3 m; a true single
    -brick lattice through 0.38 m of wall is 44 courses x 20 stretchers x 4
    wythes = ~3500 cells, i.e. ~3500 rigid bodies from ONE module, and a
    commercial building has ninety of them. So the caller's `n` still sets the
    budget and the CLUSTER is grown until the lattice fits it — but the
    cluster is always an integer number of courses and half-stretchers, so
    every cell is still a whole number of bricks and every face is still a
    joint. The single bricks that dominate a real pile by count (60-80 %) come
    from the authored heap (`quake_flow._heap` / `_a_lump`), which is where the
    mass of the pile lives anyway.

    `leaves` cuts the wall through its thickness into that many leaves — 1
    (the default) keeps a cell the full depth of the wall, which is what keeps
    it blocky. 2 is the cavity wall whose outer leaf peels off outwards.
    """
    lo, hi = mesh.bounds
    span = np.maximum(hi - lo, 1e-6)
    p_l, p_c, p_t = [float(q) for q in (pitch or BRICK_MODULAR)]
    keep = BRICK_KEEP if keep is None else float(keep)
    jit = BRICK_JITTER if jitter is None else float(jitter)
    t, c, l = _p_axes(span)
    # The wythe count the wall actually has, so a "leaf" is a real leaf.
    n_t = max(1, min(int(leaves), int(round(span[t] / max(p_t, 1e-4)))))
    # Build for MORE sites than asked: `keep` deletes a third of them.
    target = max(2.0, float(n) / max(0.2, keep)) / float(n_t)
    unit_l, unit_c = p_l * 0.5, p_c        # half a stretcher, one course
    edge = math.sqrt(max(span[l] * span[c], 1e-6) / target)
    m_l = max(1, int(round(edge / unit_l)))
    k_c = max(1, int(round(edge / unit_c)))
    for _ in range(8):
        n_l = max(1, int(math.ceil(span[l] / (m_l * unit_l))))
        n_c = max(1, int(math.ceil(span[c] / (k_c * unit_c))))
        if n_l * n_c <= cap * target:
            break
        # grow the cluster on whichever axis has the finer cell, so the
        # cluster stays square-ish rather than turning into a course-high band
        if m_l * unit_l <= k_c * unit_c:
            m_l += 1
        else:
            k_c += 1
    # A CELL MUST NOT BE MUCH WIDER THAN THE WALL IS DEEP. 0.9 m of face on
    # 0.38 m of masonry is c/b = 0.42, which the acceptance test calls flaky
    # and is right to: it is a PLATE of brickwork, not a lump of it. So the
    # cluster is shrunk toward `blocky` x the wall thickness — as far as
    # `n_max` x the caller's budget and no further, because a 4 x 7 m module
    # at 0.5 m cells is ~110 rigid bodies and a building has ninety modules.
    # `blocky_m` is the MATERIAL thickness (the caller's `solid_m`), NOT the
    # module's bbox: a kit wall with a projecting bay measures 1.07 m across
    # and is 0.38 m of brick. (Measured on P_urm1 before this clamp existed:
    # brick-mode Flakiness Index 28.6 %, equidimensional 42 %.)
    lim = float(blocky) * float(blocky_m if blocky_m else span[t])
    if lim > unit_l:
        for _ in range(24):
            if m_l * unit_l <= lim and k_c * unit_c <= lim:
                break
            nm, nk = m_l, k_c
            if m_l * unit_l > lim and (m_l * unit_l >= k_c * unit_c
                                       or k_c * unit_c <= lim):
                nm = m_l - 1
            else:
                nk = k_c - 1
            if nm < 1 or nk < 1:
                break
            tot = (max(1, int(math.ceil(span[l] / (nm * unit_l))))
                   * max(1, int(math.ceil(span[c] / (nk * unit_c)))))
            if tot > n_max * target:
                break
            m_l, k_c = nm, nk
    cell_l, cell_c = m_l * unit_l, k_c * unit_c
    n_l = max(1, int(math.ceil(span[l] / cell_l)))
    n_c = max(1, int(math.ceil(span[c] / cell_c)))
    pts = []
    for j in range(n_c):
        # RUNNING BOND: alternate cell rows shift by exactly one half
        # stretcher. Because every run is an integer number of half
        # stretchers, the shift lands the head joints of one row on the middle
        # of the units below — which is what a bricklayer does and what makes
        # the break line step instead of running straight down a perpend.
        off = unit_l if (j % 2) else 0.0
        # THE COURSE JITTER IS DRAWN PER ROW, NOT PER SITE. A bed joint is one
        # continuous plane across the wall: jittering each site's height
        # separately tilts every cell boundary in the row by a different
        # amount and the courses stop reading as courses. Head joints do vary
        # from unit to unit, so the stretcher jitter stays per site.
        cz = (lo[c] + min(span[c], cell_c * (j + 0.5))
              + (rng.random() - 0.5) * 2.0 * jit * unit_c)
        for i in range(n_l + 1):
            cx = lo[l] + off + cell_l * (i + 0.5)
            if cx > hi[l] + cell_l * 0.25:
                continue
            for k in range(n_t):
                q = np.empty(3)
                q[l] = cx + (rng.random() - 0.5) * 2.0 * jit * unit_l
                q[c] = cz
                # NO jitter through the thickness: the seed sits on the wall's
                # mid-plane (or the leaf's), so the bisector planes are
                # perpendicular to the face and the cell keeps the full wall
                # depth instead of being shaved into a plate.
                q[t] = lo[t] + span[t] * (k + 0.5) / n_t
                pts.append(q)
    if len(pts) > 3 and keep < 1.0:
        m = max(2, int(round(len(pts) * keep)))
        sel = rng.permutation(len(pts))[:m]
        pts = [pts[int(i)] for i in sel]
    return np.asarray(pts)


PRISM_AR_MAX = 2.2           # in-plane cell aspect ceiling: no sticks


def _p_prism_seeds(mesh, n, rng, keep=None, jitter=None, aspect=None,
                   axis=None, pitch=None, ar_max=PRISM_AR_MAX):
    """ONE seed layer at the mid-plane: cells are PRISMS through the member.

    "Never put a 3-D seed inside a member thinner than the cell pitch" (R §2)
    — that one line is what produced the shards. With every seed coplanar,
    every bisector plane is perpendicular to that plane, so every cut face is
    normal to the member's surface, which is what a mode-I crack in a plate
    does: it runs THROUGH the thickness.

    `aspect` (lo, hi) stretches the in-plane lattice along its long axis, for
    a slab that should raft along the beam lines rather than dice.
    """
    lo, hi = mesh.bounds
    span = np.maximum(hi - lo, 1e-6)
    t = int(np.argmin(span)) if axis is None else int(axis)
    u, v = [a for a in range(3) if a != t]
    if span[u] < span[v]:
        u, v = v, u                      # u is the long in-plane axis
    keep = 1.0 if keep is None else float(keep)
    jit = 0.26 if jitter is None else float(jitter)
    ar = 1.0
    if aspect is not None:
        ar = float(aspect[0]) + (float(aspect[1]) - float(aspect[0])) * float(
            rng.random())
    target = max(2.0, float(n) / max(0.2, keep))
    if pitch is not None:
        e_u, e_v = float(pitch[0]), float(pitch[1])
    else:
        # (s_u/e_u) * (s_v/e_v) = target with e_u = ar * e_v
        e_v = max(1e-3, math.sqrt(span[u] * span[v] / (ar * target)))
        e_u = ar * e_v
    n_u = max(1, int(round(span[u] / e_u)))
    n_v = max(1, int(round(span[v] / e_v)))
    # NO STICKS. On a member with extreme in-plane proportions the count solve
    # rounds the short way down to ONE and the cells come out as bars:
    # measured on bench P_urm1, prism fragments of 11.47 x 0.41 x 0.28 m, a
    # fan of pale dowels on the crown of the DG5 heap. Grow the count on
    # whichever axis carries the longer cell until the in-plane aspect is
    # under `ar_max`. Capped at 4x the budget so a 12 m x 0.3 m bar cannot
    # turn into a hundred bodies.
    if aspect is None:
        for _ in range(48):
            cu, cv = span[u] / n_u, span[v] / n_v
            if max(cu, cv) <= float(ar_max) * max(min(cu, cv), 1e-9):
                break
            if n_u * n_v >= 4.0 * target:
                break
            if cu > cv:
                n_u += 1
            else:
                n_v += 1
    pts = []
    for i in range(n_u):
        for j in range(n_v):
            q = np.empty(3)
            su, sv = span[u] / n_u, span[v] / n_v
            q[u] = lo[u] + su * (i + 0.5) + (rng.random() - 0.5) * su * jit * 2.0
            q[v] = lo[v] + sv * (j + 0.5) + (rng.random() - 0.5) * sv * jit * 2.0
            q[t] = lo[t] + span[t] * 0.5
            pts.append(q)
    if len(pts) > 3 and keep < 1.0:
        m = max(2, int(round(len(pts) * keep)))
        sel = rng.permutation(len(pts))[:m]
        pts = [pts[int(i)] for i in sel]
    return np.asarray(pts)


# SLIVER REJECTION (R §2.4). With sorted cell axes a >= b >= c, require
# b/a >= 0.6 and c/b >= 0.5, merging failures into the largest neighbour —
# which for a Voronoi diagram is EXACTLY "delete the seed": the neighbouring
# cells expand to absorb the vacated region with no gap and no overlap.
#
# THE c/b RULE HAS A CARVE-OUT AND IT IS DELIBERATE. A 5 x 4 x 0.2 m slab raft
# fails c/b by a mile, and R wants those: "a collapsed RC building has >= 3
# pieces > 2 m holding 25-40 % of the volume", slabs "hinge and hang". The
# flakiness data behind the 0.5 is crushed demolition waste screened to 50 mm,
# so it says nothing about the large end. So: b/a (no blades, no needles) is
# enforced on EVERY piece; c/b (no flakes) only under `sliver_max_m`, i.e. on
# the debris that has to read as rubble.
# THE SEED PASS IS OFF BY DEFAULT AND THE FRAGMENT CULL IS ON.
# `_p_sliver_seeds` costs a whole extra cell pass (~+50 % on a fracture) and
# on a `brick` or `prism` LATTICE it now almost never finds anything, because
# the only needles a lattice makes are the ribbons the module boundary clips —
# and `_p_sliver_cull` removes those at the end for free, along with the ones
# the break-line refinement makes, which the seed pass cannot see at all.
# `EQ_SLIVER_SEEDS=1` turns it back on (it merges rather than deletes, so it
# is the better answer when a hole in a static would show).
SLIVER_SEED_PASS = os.environ.get("EQ_SLIVER_SEEDS", "0").strip() not in (
    "0", "", "false", "no")
SLIVER_BA = 0.6              # b/a: needles. ENFORCED (seeds and fragments).
SLIVER_CB = 0.5              # c/b: flakes. NOT enforced by rejection — see
SLIVER_MAX_M = 1.2           # `_p_sliver_seeds`. It is set by the cell size
                             # against the material thickness instead
                             # (`BRICK_BLOCKY`). These two remain as the
                             # acceptance-test vocabulary and `_p_is_sliver`.


def _p_is_sliver(extents, ba=SLIVER_BA, cb=SLIVER_CB, max_m=SLIVER_MAX_M):
    a, b, c = sorted((float(q) for q in extents), reverse=True)
    if a < 1e-6:
        return True
    if b / a < float(ba):
        return True
    return a <= float(max_m) and b > 1e-9 and (c / b) < float(cb)


def _p_cover_shell(mesh, axis, cover_m):
    """The outer `cover_m` of a member, as its own mesh.

    "Spalling is typically within the cover" — ACI 318 §20.6 gives 38 mm on
    beams and columns, 19 mm on slabs, measured to the tie, so THE SPALL SHELL
    IS THE COVER and nothing thicker ever flakes off a frame. Round 2 broke
    the whole 0.20 m member for a spall and shed slabs of wall.

    This is the ONE legitimate flake in the whole round: 19-38 mm thick,
    0.1-0.5 m across, slightly curved because it follows the bar cage. It is
    also the one place sliver rejection must be turned OFF — by construction
    a cover flake fails c/b, and it is right to.

    `axis` is the piece's outward normal (`describe()`'s `e["out"]`).
    """
    if mesh is None or not len(mesh.faces) or not cover_m:
        return None
    u = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(u))
    if n < 1e-9:
        return None
    u = u / n
    v = np.asarray(mesh.vertices, dtype=float)
    if not len(v):
        return None
    d = v.dot(u)
    if float(d.max() - d.min()) <= float(cover_m) * 1.2:
        return None                     # already thinner than its own cover
    o = u * (float(d.max()) - float(cover_m))
    try:
        # `slice_plane` keeps the side the normal points TO (see `_cell`).
        out = slice_plane(mesh, u, o, cap=True)
    except Exception:
        return None
    return out if (out is not None and len(out.faces)) else None


SLIVER_MAX_DROP = 0.40       # a rejection pass may never take more than this


def _p_sliver_cull(frags, sliver, verbose=False, where="",
                   max_drop=SLIVER_MAX_DROP):
    """Drop the finished fragments that are still needles.

    `_p_sliver_seeds` works on SEEDS, which is the right place for it — the
    neighbours absorb the vacated region exactly. But it cannot see two things
    that come later: the cells the module's own boundary clipped into ribbons,
    and the sub-cells the break-line refinement makes out of random points. On
    bench P_rc1 that produced 30.91 x 0.25 x 0.23 m bars off a pancaked slab
    strip — a scrapyard of dowels on top of the pile — and the same shape in
    the URM heap at 11.5 m.

    So the b/a rule is enforced once more at the end, on the geometry rather
    than on the seeds. A dropped needle leaves a hairline notch in a static
    and one less stick in the debris; both are better than the stick.

    `max_drop` IS NOT OPTIONAL AND IT IS NOT A TUNING KNOB. Before the
    `_seeds` axis bug (§3.0 of the notes) was found, every prism cell of a
    pancaked facade was a needle, so this cull removed ALL of them —
    `fracture_prim` then returns early on the empty list **without
    deactivating the source**, and bench P_rc4 rendered a DG5 "pancake" as an
    INTACT six-storey office standing on a rubble skirt. When a predicate
    wants to reject most of the input, the predicate is wrong for that input;
    the safe answer is to drop only the worst `max_drop` share and let the
    rest through."""
    if not sliver or not frags:
        return frags, 0
    sv = (tuple(sliver) if isinstance(sliver, (tuple, list))
          else (SLIVER_BA, SLIVER_CB, SLIVER_MAX_M))
    ratios = []
    for f in frags:
        try:
            a, b, _c = sorted((float(q) for q in np.asarray(
                f.extents, dtype=float)), reverse=True)
            ratios.append(1e9 if a <= 1e-6 else b / a)
        except Exception:
            ratios.append(1e9)
    ba = float(sv[0])
    n_bad = sum(1 for q in ratios if q < ba)
    cap = int(math.floor(float(max_drop) * len(frags)))
    if n_bad > cap:
        # only the WORST `cap` go; see the docstring for the DG5 office that
        # stayed standing when this rule did not exist
        thresh = sorted(ratios)[max(0, cap - 1)] if cap > 0 else -1.0
        ba = min(ba, thresh if cap > 0 else -1.0)
        if verbose:
            print("[fracture]   sliver cull CAPPED{0}: {1}/{2} were needles, "
                  "dropping {3}".format((" " + where) if where else "",
                                        n_bad, len(frags), cap))
    out, n = [], 0
    for f, q in zip(frags, ratios):
        if q < ba:
            n += 1
            continue
        out.append(f)
    if n and verbose:
        print("[fracture]   sliver cull{0}: {1} needles dropped".format(
            (" " + where) if where else "", n))
    return out, n


def _p_sliver_seeds(mesh, pts, rng, ba=SLIVER_BA, cb=SLIVER_CB,
                    max_m=SLIVER_MAX_M, min_keep=3, max_drop=0.35,
                    verbose=False):
    """Delete the seeds whose cell is a NEEDLE and return the rest.

    NEEDLES ONLY (b/a), NOT FLAKES (c/b), AND THIS IS THE BUG THAT COST THE
    MOST THIS ROUND. A `prism` cell is thin BY CONSTRUCTION — the member is
    0.2-0.4 m thick and the cell is 0.8 m across, so c/b is ~0.25 and every
    single cell fails the flakiness rule. Running the full predicate here
    therefore deleted the entire lattice down to `min_keep`, and three
    surviving seeds in a 4 x 3.5 m module gave cells 3.89 m long with
    a/b = 10: on bench P_rc3, 98 % of the pancake facade fragments were
    sticks, and the DG5 pile rendered as a scrapyard of dowels. The rejection
    made exactly the shape it was written to remove.

    Flakiness is not something to reject after the fact; it is set by the CELL
    SIZE against the MATERIAL THICKNESS, which is what `BRICK_BLOCKY` does in
    `_p_brick_seeds`. What rejection is good for is the degenerate ribbon a
    module boundary clips out, and that is a b/a failure.

    `max_drop` is the second half of the lesson: a rejection pass may never
    remove more than this share of the seeds. If it wants to, the predicate is
    wrong for this input and the safe answer is to leave the lattice alone.

    MERGING INTO THE LARGEST NEIGHBOUR IS THE SAME THING AS DELETING THE SEED.
    In a Voronoi diagram the region a deleted site occupied is redistributed
    to exactly its neighbours, in proportion to how close each is — no gap, no
    overlap, no extra code. So the whole of R's "merge failures into the
    largest neighbour" is one boolean mask over the seed array.

    Costs ONE extra cell pass, so it is opt-in (`sliver=` on the fracture
    calls). On a `brick` or `prism` lattice it usually removes only the
    boundary cells the module's own edge clipped into wafers — which is the
    right answer for those too: the last part-column of bricks goes with the
    column beside it."""
    P = np.asarray(pts, dtype=float)
    if len(P) < int(min_keep) + 1:
        return P
    bad = []
    for i in range(len(P)):
        f = _cell(mesh, P, i)
        if f is None or not len(f.faces):
            continue                      # empty cells are culled anyway
        e = np.sort(np.asarray(f.extents, dtype=float))[::-1]
        if e[0] > 1e-6 and (e[1] / e[0]) < float(ba):
            bad.append(i)
    if not bad or len(P) - len(bad) < int(min_keep):
        return P
    if len(bad) > float(max_drop) * len(P):
        if verbose:
            print("[fracture]   sliver pass DECLINED: {0}/{1} seeds would go, "
                  "over the {2:.0%} ceiling".format(len(bad), len(P), max_drop))
        return P
    if verbose:
        print("[fracture]   sliver pass: {0}/{1} seeds merged away".format(
            len(bad), len(P)))
    m = np.ones(len(P), dtype=bool)
    m[np.asarray(bad, dtype=int)] = False
    return P[m]


# --- _p_ FRAGMENT SHAPE DUMP -------------------------------------------
# `EQ_DUMP_FRAGS=1` on the bench line writes one JSON line per authored
# fragment to `EQ_DUMP_FRAGS_PATH` (default /tmp/eq_frags.jsonl), which
# `scene_gen/tools/test_break_shape.py` reads on the HOST and scores against
# R §5. Off by default and costs nothing when off.
_P_DUMP = os.environ.get("EQ_DUMP_FRAGS", "").strip() not in ("", "0", "false", "no")
_P_DUMP_PATH = (os.environ.get("EQ_DUMP_FRAGS_PATH", "").strip()
                # SNAP_DIR is what eq_bench.sh mounts through to the host, so
                # the dump lands beside the PNGs with no extra plumbing.
                or (os.path.join(os.environ["SNAP_DIR"], "frags.jsonl")
                    if os.environ.get("SNAP_DIR", "").strip()
                    else "/tmp/eq_frags.jsonl"))


def _p_dump(frags, tag="", mode="", loose=True, src=None):
    """Record fragment shape statistics. `perp`/`face` are the area-weighted
    shares of the piece's surface that lie within 20 deg of perpendicular to,
    and of parallel to, its own thinnest axis — for a brick cell or a prism
    those two account for nearly all of it, and whatever is left over is the
    oblique shard face this round exists to remove."""
    if not _P_DUMP or not frags:
        return
    import json
    rows = []
    for f in frags:
        try:
            ext = np.asarray(f.extents, dtype=float)
            a, b, c = sorted((float(q) for q in ext), reverse=True)
            t = int(np.argmin(ext))
            fn = np.asarray(f.face_normals, dtype=float)
            ar = np.asarray(f.area_faces, dtype=float)
            dot = np.abs(fn[:, t])
            tot = float(ar.sum()) or 1.0
            rows.append({
                "tag": tag, "mode": mode, "loose": bool(loose),
                "a": round(a, 4), "b": round(b, 4), "c": round(c, 4),
                "vol": round(float(np.prod(np.maximum(ext, 1e-9))), 6),
                "perp": round(float(ar[dot <= 0.342].sum()) / tot, 4),
                "face": round(float(ar[dot >= 0.940].sum()) / tot, 4),
                "src": ([round(float(q), 3) for q in src]
                        if src is not None else None),
                "nf": int(len(fn))})
        except Exception:
            continue
    if not rows:
        return
    try:
        with open(_P_DUMP_PATH, "a") as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")
    except Exception as exc:
        print("[fracture] _p_dump failed: {0}".format(exc))


def _seeds(mesh, n, rng, mode="uniform", focus=None, axis=None,
           aspect=None, brick=None, keep=None, jitter=None, leaves=1,
           blocky_m=None):
    """Voronoi seed points. Their arrangement is what decides the break.

    Modes: `uniform` (isotropic rubble), `char` (the alligator grid of burnt
    timber), `splinter` (long shards), `focus` (denser toward a point),
    `plank` (an anisotropic LATTICE, so the cells are rectangular boards —
    what a wall blown apart by wind comes to pieces as; see that branch),
    and the two round-3 masonry/concrete modes `brick` and `prism` (see
    `_p_brick_seeds` / `_p_prism_seeds`).

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
    # _p_: KEEP THE CALLER'S `axis` BEFORE IT IS OVERWRITTEN. The next line
    # resolves `axis` to the LONG axis for `char`/`splinter`/`plank`, whose
    # meaning of `axis` is "the grain". `prism`'s meaning is the opposite —
    # the THIN axis, the one seeds must NOT be spread along — so reading the
    # resolved value handed it the long axis and every prism cell came out
    # spanning the whole member: 3.89 m facade sticks at a/b = 10, 11.5 m
    # partition bars, and a DG5 pancake pile that rendered as a scrapyard of
    # dowels (benches P_rc1..P_rc4).
    _axis_in = axis
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

    # --- _p_ ROUND 3: the two modes that come from a MECHANISM ------------
    if mode == "brick":
        return _p_brick_seeds(mesh, n, rng, brick, keep=keep, jitter=jitter,
                              leaves=leaves, blocky_m=blocky_m)
    if mode == "prism":
        return _p_prism_seeds(mesh, n, rng, keep=keep, jitter=jitter,
                              aspect=aspect, axis=_axis_in)
    # --- end _p_ ----------------------------------------------------------

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
                  max_piece_m=None, verbose=False,
                  brick=None, keep_frac=None, jitter=None, leaves=1,
                  sliver=None, dump_tag="", blocky_m=None):
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
    # THE ENTRY POINT EVERY CUT GOES THROUGH. `_vtk_arrays` drops individual
    # bad faces silently at the VTK boundary; a mesh that is non-finite as a
    # WHOLE is a defect upstream and worth one line, because every derived
    # number below (`extents`, `bounds`, the seed cloud, `centroid`) is then
    # NaN and the fracture returns nothing for no visible reason.
    if not np.isfinite(np.asarray(mesh.vertices, dtype=float).sum()):
        print("[fracture] REFUSED: {0} vertices, {1} faces, non-finite "
              "coordinates".format(len(mesh.vertices), len(mesh.faces)))
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
                 axis=axis, aspect=aspect, brick=brick, keep=keep_frac,
                 jitter=jitter, leaves=leaves, blocky_m=blocky_m)
    # _p_ round 3: drop the seeds whose cell would come out a blade or a
    # wafer and let their neighbours absorb them (see `_p_sliver_seeds`).
    if sliver and SLIVER_SEED_PASS:
        _sv = (tuple(sliver) if isinstance(sliver, (tuple, list))
               else (SLIVER_BA, SLIVER_CB, SLIVER_MAX_M))
        pts = _p_sliver_seeds(mesh, pts, rng, ba=_sv[0], cb=_sv[1],
                              max_m=_sv[2], verbose=verbose)
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
    keep, _n_sliv = _p_sliver_cull(keep, sliver, verbose=verbose,
                                   where="(mesh)")            # _p_
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
    _p_dump(out, dump_tag, mode, loose=True,      # _p_ (no-op unless dumping)
            src=mesh.extents)
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
                 samples=240, bulk=3, brick=None, flat_axis=None):
    """Re-fracture ONE cell with the seeds packed along the break line.

    `brick` (a (stretcher, course, wythe) pitch) makes the refinement sit on
    the SINGLE-BRICK lattice instead of on random points — which is what makes
    the tooth at a masonry break line a brick rather than a polygon, and what
    leaves the odd single brick protruding. It is affordable exactly here and
    nowhere else: one coarse cell is ~0.5 m, so the lattice inside it is a few
    dozen sites, where the same pitch over a whole 4 x 3 m module is ~3500
    (see `_p_brick_seeds`).

    `flat_axis` does the same job for `prism`: the refinement seeds are
    collapsed onto ONE plane through the member, so the teeth at the break
    line are prisms through the full thickness like every other cell. Without
    it the coarse cut is prismatic and the refined EDGE — the part anyone
    actually looks at — goes back to a 3-D Voronoi of random points in a
    0.2 m plate, i.e. shards. Measured on P_urm1 before this line existed:
    prism fragments came out at Elongation Index 68 % and 22 % blades."""
    lo, hi = cell.bounds
    if brick:                                                          # _p_
        P = _p_brick_seeds(cell, max(24, int(edge_max) * 4), rng,
                           pitch=brick, keep=1.0, leaves=1)
        if len(P) < 8:
            P = rng.uniform(lo, hi, size=(int(samples), 3))
    else:
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
    if flat_axis is not None:                                          # _p_
        a = int(flat_axis)
        seeds[:, a] = 0.5 * (float(lo[a]) + float(hi[a]))
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
                   axis=None, aspect=None, solid_m=None, solid_ref=None,
                   brick=None, keep_frac=None, jitter=None, leaves=1,
                   sliver=None, cover_m=None, cover_axis=None,
                   dump_tag="", blocky_m=None, verbose=False):
    """Fracture `mesh` and split the fragments by `judge(point) -> bool`
    (True = this point is on the side that COMES AWAY), resolving the break
    line at a second, much finer scale. Returns (static_meshes, loose_meshes).

    `judge` is evaluated in whatever frame the mesh is in — `prim_to_mesh`
    returns world space, so the earthquake judges work in world coordinates.
    """
    if mesh is None or not len(mesh.faces):
        return [], []
    if solid_m:
        mesh = solidify(mesh, solid_m, ref=solid_ref, verbose=verbose)
    if cover_m and cover_axis is not None:                          # _p_
        _sh = _p_cover_shell(mesh, cover_axis, cover_m)
        if _sh is not None:
            mesh = _sh
    bbox_vol = float(np.prod(np.maximum(mesh.extents, 1e-6)))
    keep_min = bbox_vol * float(min_volume_frac)
    sub_min = (0.35 * float(edge_cell_m)) ** 3

    pts = _seeds(mesh, int(n_pieces), rng, mode=mode, focus=focus,
                 axis=axis, aspect=aspect, brick=brick, keep=keep_frac,
                 jitter=jitter, leaves=leaves, blocky_m=blocky_m)
    if sliver and SLIVER_SEED_PASS:                                # _p_
        _sv = (tuple(sliver) if isinstance(sliver, (tuple, list))
               else (SLIVER_BA, SLIVER_CB, SLIVER_MAX_M))
        pts = _p_sliver_seeds(mesh, pts, rng, ba=_sv[0], cb=_sv[1],
                              max_m=_sv[2], verbose=verbose)
    field = noise_field(rng, amp_m=rough_m, lam_m=rough_lam_m)
    # `prism` seeds one layer at the member's mid-plane; the refinement has to
    # honour the same plane or the break line goes back to shards.
    _flat = int(np.argmin(mesh.extents)) if mode == "prism" else None    # _p_

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
        subs = (_refine_cell(c, judge, rng, edge_cell_m, edge_max, sub_min,
                             brick=brick, flat_axis=_flat)
                if (refine and k < int(refine_max)) else [])
        if not subs:
            out.append((c, side, True, True))
            continue
        n_ref += 1
        for s in subs:
            t, n = _probe(s, judge)
            out.append((s, bool(t * 2 >= n), True, 0 < t < n))

    statics, looses = [], []
    n_eaten = n_sliv = 0
    _sv_ba = ((tuple(sliver)[0] if isinstance(sliver, (tuple, list))
               else SLIVER_BA) if sliver else None)
    # THE SAME CEILING AS `_p_sliver_cull`, and for the same reason: a cull
    # that wants to take most of the piece means the predicate is wrong for
    # this input, and an emptied `_break_split` leaves the wall standing.
    if _sv_ba is not None:
        _r = []
        for _m, _l, _ae, _s in out:
            _x = np.sort(np.asarray(_m.extents, dtype=float))[::-1]
            _r.append(1e9 if _x[0] <= 1e-6 else _x[1] / _x[0])
        _cap = int(math.floor(SLIVER_MAX_DROP * len(_r)))
        if sum(1 for q in _r if q < _sv_ba) > _cap:
            _sv_ba = (sorted(_r)[_cap - 1] if _cap > 0 else None)
            if verbose:
                print("[fracture]   split sliver cull CAPPED at {0}/{1}".format(
                    _cap, len(_r)))
    for msh, loose, at_edge, strad in out:
        # _p_ FINAL NEEDLE CULL — see `_p_sliver_cull` for why the seed pass
        # is not enough.
        if _sv_ba is not None:
            _e = np.sort(np.asarray(msh.extents, dtype=float))[::-1]
            if _e[0] > 1e-6 and (_e[1] / _e[0]) < _sv_ba:
                n_sliv += 1
                continue
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
              "{3} static + {4} loose ({5} edge chips, {6} needles dropped)"
              .format(len(cells), len(straddle), n_ref, len(statics),
                      len(looses), n_eaten, n_sliv))
    _p_dump(statics, dump_tag, mode, loose=False,   # _p_ (no-op unless dumping)
            src=mesh.extents)
    _p_dump(looses, dump_tag, mode, loose=True, src=mesh.extents)
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


def face_subset(stage, path, axis, cos=0.30, name="core", mesh=None):
    """A `materialBind` GeomSubset of every face that is NOT the façade.

    A solidified fragment carries two kinds of face: the outer ones, which are
    the module's own cladding and must keep the cladding material, and the
    ones this pipeline invented — the cut faces, the back, and the reveals
    round an opening — which are the inside of the wall and must read as brick
    or concrete core. A prim takes one material, so the core faces go into a
    subset the caller binds separately.

    `axis` is the module's OUTWARD direction; a face is façade when its normal
    is within `acos(cos)` of it. Returns the subset's prim, or None when the
    split is degenerate (all faces one way — a fragment entirely off the back,
    which the caller should simply bind to the core material whole).
    """
    from pxr import UsdGeom, Vt

    pr = stage.GetPrimAtPath(path)
    if not pr or not pr.IsValid():
        return None
    m = UsdGeom.Mesh(pr)
    if mesh is not None:
        # PASS THE TRIMESH IN when you have it. Reading points and indices
        # back out of USD and converting the Vt arrays is the expensive half
        # of this function, and every caller in this pipeline has just
        # authored the prim from a trimesh it still holds.
        v = np.asarray(mesh.vertices, dtype=float)
        f = np.asarray(mesh.faces, dtype=np.int64)
    else:
        pts = m.GetPointsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        if (not pts or not idx or not cnt
                or int(min(cnt)) != 3 or int(max(cnt)) != 3):
            return None
        v = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
        f = np.asarray(idx, dtype=np.int64).reshape(-1, 3)
    if len(f) < 2:
        return None
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-16)
    u = np.asarray(axis, dtype=float)
    u = u / max(float(np.linalg.norm(u)), 1e-12)
    sel = np.where((n @ u) <= float(cos))[0]
    if not len(sel) or len(sel) == len(f):
        return None
    try:
        sub = UsdGeom.Subset.CreateGeomSubset(
            m, name, UsdGeom.Tokens.face,
            Vt.IntArray([int(i) for i in sel]), "materialBind",
            UsdGeom.Tokens.nonOverlapping)
    except Exception:
        return None
    return sub.GetPrim()


def fracture_partial(stage, prim_path, out_parent, n_pieces, rng,
                     cut_frac=0.5, mode="char", rough=0.035, shrink=0.97,
                     ragged=0.22, deactivate=True, consume=0.30, axis=None,
                     min_volume_frac=0.004, aspect=None, max_piece_m=None,
                     solid_m=None, solid_ref=None, **kw):
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
    if solid_m:
        mesh = solidify(mesh, solid_m, ref=solid_ref)

    frags = fracture_mesh(mesh, n_pieces, rng, mode=mode, rough=rough,
                          shrink=shrink, consume=consume, axis=axis,
                          aspect=aspect, min_volume_frac=min_volume_frac,
                          max_piece_m=max_piece_m, **kw)
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
                  aspect=None, max_piece_m=None, solid_m=None, solid_ref=None,
                  **kw):
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
    if solid_m:
        mesh = solidify(mesh, solid_m, ref=solid_ref, verbose=verbose)
    frags = fracture_mesh(mesh, n_pieces, rng, mode=mode, rough=rough,
                          shrink=shrink, consume=consume,
                          consume_pool=consume_pool, axis=axis,
                          aspect=aspect, min_volume_frac=min_volume_frac,
                          max_piece_m=max_piece_m, verbose=verbose, **kw)
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


# ---------------------------------------------------------------------------
# chip_box — irregular versions of the AUTHORED rectangular solids
# ---------------------------------------------------------------------------
# WHY THIS EXISTS. Everything above cuts REAL building geometry. But three
# populations in the earthquake path are authored as literal cuboids and never
# see a fracture at all: `quake_rubble_usd._box` (lintels, quoins, joists,
# column stubs), the floor plates `quake_flow._box` builds for the fit-out and
# `quake_collapse` then drops whole, and the prism/plank cells a `_break_box`
# leaves behind. On the first 500 m OSMO scene the user's verdict was blunt:
# "a lot of perfect rectangular debris. While they should look rectangular,
# they shouldn't look perfect ... use VTK to cause chips ... random from small
# to very large chips ... warped? bent?"
#
# So: take a rectangular solid, bite corners out of it with oblique VTK plane
# clips, roughen the cast faces, and (timber/metal only) bend it.
#
# THE SHAPE OF A CHIP IS THE WHOLE TRICK. An infinite plane cut is only LOCAL
# when its normal has a substantial component on ALL THREE axes: the removed
# region is then bounded on every axis by depth/|n_i|, i.e. a corner wedge. Let
# one component go to ~0 and the same cut bevels the entire edge run instead —
# which is the "gift box with a bite taken out" look this is trying to avoid.
# `_chip_normal` therefore draws |n_i| >= 0.40 for a corner chip and only
# occasionally (`bevel_p`) lets one component collapse, for the one big
# diagonal shear a broken slab really does have.
#
# `ends` is the second user note ("the actual breaking should not be clean"):
# on a BAR-shaped piece the two faces normal to the long axis are the BREAK,
# not a cast surface, so that share of the chips is aimed at them with the long
# axis dominant and both cross-axes still substantial — several overlapping
# corner bites on one end read as a snapped, stepped fracture instead of a saw
# cut.
#
# SAFETY. Only ever call this on a solid THIS codebase authored (a box, or a
# Voronoi cell of one). It must NEVER be pointed at a clipped shell: that is
# the `vtkStripper::GetPointCells` SIGSEGV in the round-4 catalogue, and
# `quake_sliced`'s standing rule. The guard (`FRACTURE_VTK_GUARD`) is honoured
# on both ends of the VTK boundary here exactly as `_vtk_slice` honours it, and
# every candidate cut is REJECTED unless it comes back capped, closed and
# inside the volume budget — so a chip that goes wrong costs one chip, never
# the piece.
#
# `QC_CHIP=0` turns the whole thing off; call sites are written so that with it
# off they take their pre-chip code path byte for byte.


def chips_enabled():
    """Is chipping on? `QC_CHIP=0` (or false/no/off) turns it off.

    Read from the environment on every call rather than cached at import so a
    test — and a bake driver's `EXTRA_ENV` — can flip it without reloading the
    module. One dict lookup per piece.
    """
    return os.environ.get("QC_CHIP", "1").strip().lower() not in (
        "0", "false", "no", "off")


def stable_seed(*parts):
    """A 32-bit seed from an object's own identity — never a shared rng draw.

    The chip call sites are EMITTERS with no rng of their own, and threading
    one through would make every piece's shape depend on how many pieces were
    authored before it (and would shift a recipe's whole random stream the
    moment chipping is switched on). Hashing the piece's own identity —
    prim path, or (tag, index, kind, size, position) — gives the same shape for
    the same plan every time and costs the surrounding code nothing.
    """
    import zlib
    key = "|".join(("{0:.4f}".format(q) if isinstance(q, float) else str(q))
                   for q in parts)
    return int(zlib.crc32(key.encode("utf-8")) & 0xFFFFFFFF)


class _Arrays(object):
    """The two attributes `_to_vtk`/`_vtk_arrays` actually read.

    Lets the chip path reuse the guard machinery verbatim without dragging in
    trimesh — which the offline preview/test environment does not install.
    """

    __slots__ = ("vertices", "faces")

    def __init__(self, v, f):
        self.vertices = v
        self.faces = f


def _pd_arrays(pd):
    """(vertices, faces) out of a vtkPolyData. `_from_vtk` without trimesh."""
    import vtk
    from vtk.util import numpy_support as ns

    tf = vtk.vtkTriangleFilter()
    tf.SetInputData(pd)
    tf.Update()
    out = tf.GetOutput()
    if out.GetNumberOfPoints() == 0 or out.GetNumberOfPolys() == 0:
        return None
    v = np.asarray(ns.vtk_to_numpy(out.GetPoints().GetData()), dtype=float)
    polys = out.GetPolys()
    try:
        f = ns.vtk_to_numpy(polys.GetConnectivityArray()).reshape(-1, 3)
    except AttributeError:
        f = ns.vtk_to_numpy(polys.GetData()).reshape(-1, 4)[:, 1:]
    f = np.asarray(f, dtype=np.int64)
    if not len(f):
        return None
    if _VTK_GUARD:
        ok = (f >= 0).all(axis=1) & (f < len(v)).all(axis=1)
        if not ok.all():
            _guard_note("chip face(s) VTK returned naming a point it did "
                        "not make", int((~ok).sum()))
            f = f[ok]
            if not len(f):
                return None
    if not np.isfinite(v).all():
        _guard_note("non-finite chip vertex/vertices",
                    int((~np.isfinite(v).all(axis=1)).sum()))
        return None
    return v, f


def _chip_clip(v, f, normal, origin):
    """ONE capped plane cut of a closed solid, arrays in / arrays out.

    Keeps the side `normal` points to, exactly like `slice_plane`. Returns
    None — never a half-open solid — when the cap cannot be built, because an
    uncapped chip is a hole in a piece that physics and the renderer both see.
    """
    if _vtk() is None:
        return None
    import vtk

    pd = _to_vtk(_Arrays(v, f))
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

    cut = vtk.vtkCutter()
    cut.SetInputData(pd)
    cut.SetCutFunction(plane)
    cut.Update()
    # `SetInputData` on the validated polydata, never `SetInputConnection` —
    # same reason as `_vtk_slice`: the stripper must walk EXACTLY what
    # `_strip_input` just checked.
    src = _strip_input(cut.GetOutput())
    if src is None:
        return None
    strip = vtk.vtkStripper()
    strip.SetInputData(src)
    strip.Update()
    loops = strip.GetOutput()
    if loops is None or not loops.GetNumberOfLines():
        return None
    face = vtk.vtkPolyData()
    face.SetPoints(loops.GetPoints())
    face.SetPolys(loops.GetLines())
    fill = vtk.vtkTriangleFilter()
    fill.SetInputData(face)
    fill.Update()
    filled = fill.GetOutput()
    if not filled.GetNumberOfPolys():
        return None

    app = vtk.vtkAppendPolyData()
    app.AddInputData(kept)
    app.AddInputData(filled)
    app.Update()
    # NO `vtkCleanPolyData` HERE, unlike `_vtk_slice`. The clipper and the
    # cutter interpolate the SAME edge with the same formula, so the cap's
    # rim points come back bit-identical to the body's and `weld_arrays`
    # (which the caller runs anyway, to drop the degenerate faces the append
    # can leave) merges them for free. Measured over 24 seeds x 5 kinds:
    # zero open edges either way, and one fewer VTK `Update()` per chip is
    # ~15 % of the whole helper's wall time.
    return _pd_arrays(app.GetOutput())


def mesh_volume(v, f):
    """Signed-tetrahedron volume of a closed triangle mesh, absolute."""
    v = np.asarray(v, dtype=float)
    f = np.asarray(f, dtype=np.int64)
    if len(f) < 4 or len(v) < 4:
        return 0.0
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    return abs(float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0)


def weld_arrays(v, f, tol=1e-6):
    """Merge vertices closer than `tol` and drop the degenerate faces."""
    v = np.asarray(v, dtype=float)
    f = np.asarray(f, dtype=np.int64)
    if not len(v) or not len(f):
        return v, f
    key = np.round(v / float(tol)).astype(np.int64)
    _u, idx, inv = np.unique(key, axis=0, return_index=True,
                             return_inverse=True)
    inv = np.asarray(inv).ravel()
    vv = v[idx]
    ff = inv[f]
    ok = ((ff[:, 0] != ff[:, 1]) & (ff[:, 1] != ff[:, 2])
          & (ff[:, 2] != ff[:, 0]))
    return vv, ff[ok]


def open_edge_count(f):
    """How many undirected edges are used by exactly ONE face.

    Zero on a closed solid. This is the acceptance test every candidate chip
    has to pass — a cut whose cap came back short opens the piece, and an open
    piece is a hole in the render and a bad convex hull for physics.
    """
    f = np.asarray(f, dtype=np.int64)
    if len(f) < 4:
        return 3 * len(f)
    e = np.sort(np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _u, cnt = np.unique(e, axis=0, return_counts=True)
    return int((cnt == 1).sum())


def box_arrays(sx, sy, sz, bottom=False, seg=(1, 1, 1)):
    """A closed, SHARED-VERTEX triangulated box, optionally tessellated.

    `bottom=True` puts the local origin at the bottom-centre
    (`quake_rubble_usd._box`'s convention); otherwise the box is centred
    (`quake_flow._box`'s). `seg` is the segment count per axis — a warp or a
    roughening pass has nothing to displace on an 8-corner box, and a
    STRUCTURED grid is the cheap way to get segments where the piece is long
    without paying for them through its thickness (uniform subdivision of a
    3.5 x 0.1 x 0.2 m joist spends three quarters of its triangles on the
    section).
    """
    nx, ny, nz = [max(1, int(s)) for s in seg]
    hx, hy = float(sx) / 2.0, float(sy) / 2.0
    z0 = 0.0 if bottom else -float(sz) / 2.0
    xs = np.linspace(-hx, hx, nx + 1)
    ys = np.linspace(-hy, hy, ny + 1)
    zs = np.linspace(z0, z0 + float(sz), nz + 1)
    idx, pts, tris = {}, [], []

    def gid(i, j, k):
        r = idx.get((i, j, k))
        if r is None:
            r = len(pts)
            idx[(i, j, k)] = r
            pts.append((xs[i], ys[j], zs[k]))
        return r

    def quad(a, b, c, d):
        tris.append((a, b, c))
        tris.append((a, c, d))

    for i in range(nx):
        for j in range(ny):
            quad(gid(i, j, 0), gid(i, j + 1, 0),
                 gid(i + 1, j + 1, 0), gid(i + 1, j, 0))            # -z
            quad(gid(i, j, nz), gid(i + 1, j, nz),
                 gid(i + 1, j + 1, nz), gid(i, j + 1, nz))          # +z
    for i in range(nx):
        for k in range(nz):
            quad(gid(i, 0, k), gid(i + 1, 0, k),
                 gid(i + 1, 0, k + 1), gid(i, 0, k + 1))            # -y
            quad(gid(i, ny, k), gid(i, ny, k + 1),
                 gid(i + 1, ny, k + 1), gid(i + 1, ny, k))          # +y
    for j in range(ny):
        for k in range(nz):
            quad(gid(0, j, k), gid(0, j, k + 1),
                 gid(0, j + 1, k + 1), gid(0, j + 1, k))            # -x
            quad(gid(nx, j, k), gid(nx, j + 1, k),
                 gid(nx, j + 1, k + 1), gid(nx, j, k + 1))          # +x
    return np.asarray(pts, dtype=float), np.asarray(tris, dtype=np.int64)


def auto_seg(sx, sy, sz, cell_m=None, cap=4, budget=96):
    """Segment counts that make the cells roughly square and never explode.

    `cap=4` and a quarter-of-the-long-axis cell keep a chipped piece in the
    40-140 triangle band. Going finer buys nothing visible (the chips, not the
    tessellation, are what breaks the silhouette) and costs both authoring
    time and stage size on a population of thousands.
    """
    ext = np.abs(np.asarray([sx, sy, sz], dtype=float))
    if cell_m is None:
        cell_m = max(float(ext.max()) / 4.0, 1e-3)
    seg = tuple(int(np.clip(round(float(e) / float(cell_m)), 1, cap))
                for e in ext)
    # AND A HARD TRIANGLE BUDGET. A near-cube (a quoin, a column stub) has no
    # short axis to collapse, so the same cell size that gives a 3.5 m joist
    # 36 triangles gives it 132 — for no visible gain, on a population of
    # thousands. Coarsen until the base box fits `budget`.
    while (4 * (seg[0] * seg[1] + seg[0] * seg[2] + seg[1] * seg[2])
           > int(budget)) and max(seg) > 1:
        cell_m *= 1.25
        seg = tuple(int(np.clip(round(float(e) / float(cell_m)), 1, cap))
                    for e in ext)
    return seg


def subdivide_to_budget(v, f, budget=260):
    """Uniform 4-to-1 midpoint subdivision while the triangle count fits.

    RUN AFTER THE CHIPS, not before. A chip's cut face is one flat polygon
    with vertices only on its rim, so a roughening pass applied to the box
    beforehand cannot touch it — and a break face that is perfectly flat is
    the "clean break" the user rejected. Subdividing the CHIPPED solid puts
    vertices in the middle of every face, cast and cut alike, which is what
    `roughen_arrays` then has to work with.

    Shared edge midpoints (the dict), so the solid stays watertight.
    """
    v = [tuple(float(q) for q in p) for p in np.asarray(v, dtype=float)]
    f = [tuple(int(q) for q in t) for t in np.asarray(f, dtype=np.int64)]
    while len(f) and 4 * len(f) <= int(budget):
        mid = {}

        def m(i, j):
            k = (i, j) if i < j else (j, i)
            r = mid.get(k)
            if r is None:
                a, b = v[i], v[j]
                r = len(v)
                v.append(((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5,
                          (a[2] + b[2]) * 0.5))
                mid[k] = r
            return r

        nf = []
        for a, b, c in f:
            ab, bc, ca = m(a, b), m(b, c), m(c, a)
            nf += [(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)]
        f = nf
    return np.asarray(v, dtype=float), np.asarray(f, dtype=np.int64)


# ---------------------------------------------------------------------------
# ROUND 6: THE TWO THINGS A HALF-SPACE CLIP CANNOT DO
# ---------------------------------------------------------------------------
#
# THE MEASUREMENT THAT FORCED THIS. `tools/pillar_break_bench.py` renders the
# production chip path on the two shapes the user keeps pointing at, and
# samples the solid on a grid (`vtkSelectEnclosedPoints`) to get the CROSS-
# SECTION FILL at 44 stations along a piece. On a 0.40 x 0.40 x 3.50 m concrete
# column through `_CHIP_KIND["column"]`, round 5 gives:
#
#     station     1   2   3  ...  22  ...  41  42  43  44
#     fill %     22  42  61      92       91  92  88  72
#
# — the two END stations lose material and THE OTHER FORTY DO NOT. The render
# agrees: the shaft is a dead-straight prism for 3.2 of its 3.5 m and the piece
# reads, correctly, as "a perfect cuboid with the top corner knocked off". That
# is the user's complaint, made three review rounds running, and no amount of
# retuning `depth_frac` fixes it, because:
#
#   A PLANE CLIP CAN ONLY REMOVE MATERIAL AT AN EXTREME CORNER. `_one_chip`
#   cuts at `n * (hi - d)` where `hi` is the piece's MAXIMUM projection along
#   the drawn normal, so whatever normal is drawn, the removed region contains
#   the vertex that is extreme along it. On a convex box that is one of the 8
#   corners — and all 8 corners of a slender column are at its two ends.
#
# So two mechanisms are added here, and they are the whole of the round-6 fix:
#
#   `subdivide_long_edges`  refine by EDGE LENGTH, not uniformly. The old
#                           `subdivide_to_budget` spends its budget splitting a
#                           column's 0.4 m section edges exactly as hard as its
#                           3.5 m flank edges, so the shaft ends up with four
#                           stations and there is nothing to displace.
#
#   `gouge_arrays`          bite localized scallops out of the surface, at a
#                           STATION CHOSEN ALONG THE PIECE rather than at a
#                           corner. This is the only mechanism in this file
#                           that can take a quarter of the section out of the
#                           middle of a column, or a big scallop out of the
#                           middle of a slab's edge instead of sawing its
#                           corners off into a stop-sign.
#
# Both are off unless asked for (`gouges=(0, 0)` reproduces round 5), and both
# are driven from the `_CHIP_*` spec dicts, so a caller that does not set the
# new keys is byte-for-byte unchanged.


def subdivide_long_edges(v, f, target_m, budget=900, max_passes=5):
    """Split only the edges longer than `target_m`. Stays watertight.

    RED-GREEN REFINEMENT with a SHARED midpoint table: a triangle with 1, 2 or
    3 over-long edges becomes 2, 3 or 4 triangles, and the neighbour across a
    split edge splits that same edge at the same midpoint — so there is no
    T-junction, no open edge, and no vertex moves.

    WHY NOT `subdivide_to_budget`. That is a uniform 4-to-1 split, so it can
    only ever land on 12, 48, 192, 768 ... triangles — it QUANTIZES the budget
    into powers of four and strands whatever is left. Measured on a
    0.4 x 0.4 x 3.5 m column at budget 760: uniform stops at 192 triangles with
    a longest edge of 0.88 m, because the next step would be 768; this reaches
    512 triangles and 0.45 m. That difference is the whole gouge pass — a
    scallop 0.3 m across has nothing to displace at 0.88 m spacing. On the
    2.5 x 1.6 x 0.12 m plate at the same budget it is 192/0.74 m against
    472/0.37 m.

    It is also anisotropic where the piece is: an edge already under
    `target_m` is never split, so a short section axis stops paying once it is
    fine enough while a long one keeps refining. (Face diagonals are long on
    a slender piece, so their midpoints do land on the section — the split is
    edge-length-driven, not axis-driven.)
    """
    v = [tuple(float(q) for q in p) for p in np.asarray(v, dtype=float)]
    f = [tuple(int(q) for q in t) for t in np.asarray(f, dtype=np.int64)]
    t2 = float(target_m) ** 2
    for _ in range(int(max_passes)):
        if not f:
            break
        long_e = set()
        for tri in f:
            for i, j in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
                k = (i, j) if i < j else (j, i)
                if k in long_e:
                    continue
                a, b = v[i], v[j]
                if ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
                        + (a[2] - b[2]) ** 2) > t2:
                    long_e.add(k)
        if not long_e:
            break
        # STOP BEFORE OVERSHOOTING, not after: each split edge adds one
        # triangle to each of the (at most two) faces using it, so the pass
        # costs at most 2 * len(long_e) triangles.
        if len(f) + 2 * len(long_e) > int(budget):
            break
        mid = {}

        def m(i, j):
            k = (i, j) if i < j else (j, i)
            r = mid.get(k)
            if r is None:
                a, b = v[i], v[j]
                r = len(v)
                v.append(((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5,
                          (a[2] + b[2]) * 0.5))
                mid[k] = r
            return r

        def is_long(i, j):
            return ((i, j) if i < j else (j, i)) in long_e

        nf = []
        for a, b, c in f:
            e = (is_long(a, b), is_long(b, c), is_long(c, a))
            n = sum(e)
            if n == 0:
                nf.append((a, b, c))
            elif n == 3:
                p, q, r = m(a, b), m(b, c), m(c, a)
                nf += [(a, p, r), (p, b, q), (r, q, c), (p, q, r)]
            elif n == 1:
                # rotate so the split edge is (a, b)
                if e[1]:
                    a, b, c = b, c, a
                elif e[2]:
                    a, b, c = c, a, b
                p = m(a, b)
                nf += [(a, p, c), (p, b, c)]
            else:
                # rotate so the UNSPLIT edge is (c, a) -> splits are ab and bc
                if not e[0]:
                    a, b, c = b, c, a
                elif not e[1]:
                    a, b, c = c, a, b
                p, q = m(a, b), m(b, c)
                nf += [(p, b, q), (a, p, q), (a, q, c)]
        f = nf
    return np.asarray(v, dtype=float), np.asarray(f, dtype=np.int64)


def _gouge_faces(ext, rng, n):
    """`n` distinct (axis, sign) faces to bite, drawn without replacement.

    DISTINCT, because the acceptance test for this whole exercise is "no piece
    is left with all its faces pristine": drawing the same flank three times
    leaves three others untouched.

    WEIGHTED BY THE SQUARE ROOT OF THE FACE AREA, not by the area. Straight
    area is right for a column (its four 1.4 m2 flanks should outrank its two
    0.16 m2 ends) and badly wrong for a slab, where the two 4 m2 decks outrank
    the rim 21:1 — and a deck gouge can only ever be as deep as the plate is
    thick, so the slab spent its whole budget on shallow dishes and came back
    looking like a crumpled pillow rather than a slab with its edge bitten
    out. The square root keeps the column's preference (2.9:1) while giving the
    rim a real share."""
    # NEVER GOUGE INTO A THIN AXIS. A gouge is a bite to depth `h`, and `h` is
    # a fraction of the extent along the axis it bites into — so on the 12 cm
    # deck of a slab the deepest possible gouge is 5 cm, which does not read as
    # a bite out of a plate. It reads as the plate BEING BENT, and a run of
    # them reads as crumpled cloth: that is exactly what the v2/v3 bench sheets
    # showed for `plate`, decks warped into a pillow while the edge runs stayed
    # smooth. Concrete does not bend. A slab's damage belongs on its RIM, so
    # the two faces normal to an axis under a third of the piece's median
    # extent are dropped from the draw entirely.
    thin = 0.35 * float(np.median(np.asarray(ext, dtype=float)))
    cand = []
    for ax in range(3):
        if float(ext[ax]) < thin:
            continue
        oa, ob = [q for q in range(3) if q != ax]
        a = math.sqrt(float(ext[oa] * ext[ob]))
        for sgn in (1.0, -1.0):
            cand.append([ax, sgn, max(a, 1e-9)])
    if not cand:
        return []
    out = []
    for k in range(int(min(n, len(cand)))):
        pool = cand
        if k == 0:
            # THE FIRST GOUGE GOES ON A FLANK, NOT AN END. On a bar the two
            # end faces are already the only thing the plane clips can reach,
            # and a draw that happens to send every gouge there leaves the
            # shaft a prism again — measured over 14 column seeds, one came
            # back with its middle at 0.91 of section while the other 13 were
            # 0.19-0.81. Guaranteeing one flank bite is cheaper than widening
            # the acceptance test to admit the piece that fails it.
            la = int(np.argmax(ext))
            flanks = [c for c in cand if c[0] != la]
            if flanks and float(ext[la]) >= 2.2 * float(
                    np.sort(np.asarray(ext, dtype=float))[1]):
                pool = flanks
        tot = sum(c[2] for c in pool)
        x = rng.random() * tot
        for c in pool:
            x -= c[2]
            if x <= 0.0:
                break
        cand.remove(c)
        out.append(tuple(c[:2]))
    return out


def gouge_arrays(v, rng, n_gouges, depth_frac=(0.10, 0.30), big_p=0.28,
                 big_frac=(0.34, 0.55), radius_mul=(0.9, 2.4),
                 elong=(1.0, 1.9), station=(0.14, 0.86), taper=1.15,
                 vol_frac=0.30, piece_vol=None):
    """Bite `n_gouges` localized scallops INTO a closed solid. Points only.

    THE MECHANISM THE CLIPS DO NOT HAVE (see the section note above): each
    gouge is anchored at a STATION chosen along the face it sits on — by
    default the middle 72 % of it — so a slender piece finally loses material
    somewhere other than its two ends.

    Per gouge:
      * a face `(axis, sign)`, drawn without replacement, weighted by area;
      * a depth `h`, a fraction of the piece's extent ALONG THAT AXIS, drawn
        log-uniformly over `depth_frac` with a `big_p` tail into `big_frac` —
        which is the user's "random from small to VERY LARGE": on a 0.4 m
        column the tail takes 13-22 cm, a third to a half of the section;
      * an elliptical footprint, `radius_mul` x the depth, stretched `elong`
        along the face's longer axis and clamped to fit the face, with a
        RAGGED rim (two azimuthal harmonics) so the scar outline is not a
        circle;
      * a near-CONICAL profile (`taper` > 1 sharpens the rim) rather than a
        smooth bump — spalled concrete has an edge to it.

    A vertex only moves if it is on the outer half of the piece along that
    axis, and never past `0.15 x extent` from the far surface, so the solid
    cannot be pushed through itself. Topology is untouched: no face is added,
    removed or reordered, so a watertight input stays watertight.

    Deterministic in `rng`. Returns a new array; the input is not modified.
    """
    v = np.array(np.asarray(v, dtype=float), copy=True)
    n_gouges = int(n_gouges)
    if len(v) < 8 or n_gouges <= 0:
        return v
    lo, hi = v.min(0), v.max(0)
    ext = np.maximum(hi - lo, 1e-9)
    if piece_vol is None:
        piece_vol = float(np.prod(ext))
    # ONE VOLUME BUDGET, SHARED. Without it a gouge on a long face is sized in
    # metres off that face's own extent and a couple of them eat the piece: a
    # 2.5 m slab edge scallop drawn at 0.30 of 2.5 m is 0.75 m deep. The budget
    # is spent by SHRINKING THE FOOTPRINT, never the depth — a shallow wide
    # dish reads as a dent, a deep narrow one reads as a bite, and the bite is
    # the thing that was missing.
    cap = float(vol_frac) * float(max(piece_vol, 1e-9)) / max(n_gouges, 1)
    for ax, sgn in _gouge_faces(ext, rng, n_gouges):
        oa, ob = [q for q in range(3) if q != ax]
        if ext[oa] < ext[ob]:
            oa, ob = ob, oa                      # oa is the face's LONG axis
        lo, hi = v.min(0), v.max(0)
        ext = np.maximum(hi - lo, 1e-9)
        d0, d1 = max(float(depth_frac[0]), 1e-4), max(float(depth_frac[1]), 1e-3)
        q = d0 * (d1 / d0) ** rng.random()
        if rng.random() < float(big_p):
            q = rng.uniform(*big_frac)
        h = min(float(q), 0.45) * float(ext[ax])
        # THE FOOTPRINT IS TIED TO THE DEPTH, and only then to the face.
        #
        # Both clamps are load-bearing and they fail in opposite directions.
        # Allowing a radius up to the face's own extent turned a column's
        # gouges into 1.4 m vertical creases — the v2 bench renders read as
        # DRAPERY, not as spalled concrete, because a scallop four times
        # longer than it is deep is a fold. Capping it at "half the extent"
        # instead does the opposite harm on the 0.12 m rim of a slab: the
        # footprint then sits inside the thickness and carves a groove along
        # the middle of the edge rather than taking the edge out. So the cap
        # is `3.2 x depth` (a bite is about as wide as it is deep, give or
        # take), widened to at least 6 % of the face so a shallow gouge is not
        # a pinprick, and finally limited by the face itself.
        hi_u = float(min(0.95 * ext[oa], max(3.2 * h, 0.06 * float(ext[oa]))))
        hi_v = float(min(0.95 * ext[ob], max(3.2 * h, 0.06 * float(ext[ob]))))
        ru = float(np.clip(h * rng.uniform(*radius_mul) * rng.uniform(*elong),
                           min(0.25 * float(ext[oa]), hi_u), hi_u))
        rv = float(np.clip(h * rng.uniform(*radius_mul),
                           min(0.25 * float(ext[ob]), hi_v), hi_v))
        est = 0.5 * math.pi * ru * rv * h
        if est > cap:
            # SPEND THE OVERRUN ON `h` AND `ru`, NEVER ON `rv`. `rv` runs on
            # the face's SHORT axis and is usually already pinned to it — on a
            # slab's 0.12 m rim it is the thickness — and shrinking it is what
            # turns a scallop out of the edge into a groove along the middle
            # of it. Halving the overrun between the depth and the long
            # footprint axis keeps the removed volume exact either way.
            k = math.sqrt(cap / est)
            ru = max(ru * k, 0.08 * float(ext[oa]))
            h *= k
        # keep the footprint on the face: a gouge centred a radius off the end
        # of a thin axis is half a gouge
        mu, mv = min(0.49, ru / ext[oa] * 0.55), min(0.49, rv / ext[ob] * 0.55)
        cu = lo[oa] + ext[oa] * float(np.clip(rng.uniform(*station),
                                              mu, 1.0 - mu))
        cv = lo[ob] + ext[ob] * float(np.clip(rng.uniform(*station),
                                              mv, 1.0 - mv))
        ph1, ph2 = rng.uniform(0, 6.2832), rng.uniform(0, 6.2832)
        k1, k2 = rng.choice((2, 3, 4)), rng.choice((5, 6, 7))

        du = (v[:, oa] - cu) / ru
        dv = (v[:, ob] - cv) / rv
        # A SUPER-ELLIPSE, NOT A CIRCLE, and on an axis the footprint already
        # spans, no falloff at all. A radial profile tapers towards the rim in
        # EVERY direction — including across a slab's 12 cm thickness, where
        # the footprint covers the whole face — so the bite came out
        # lens-shaped: deepest at mid-thickness, shallower at the deck and the
        # soffit. That is a rounded edge, which is why the v3/v4 plate sheets
        # still read as soft. p=3.5 keeps a flat floor and a sharp rim, and
        # zeroing a spanned axis makes the gouge a straight-through bite of the
        # rim rather than a groove inside it.
        if rv >= 0.85 * float(ext[ob]):
            dv = np.zeros_like(dv)
        if ru >= 0.85 * float(ext[oa]):
            du = np.zeros_like(du)
        pw = 3.5
        t = (np.abs(du) ** pw + np.abs(dv) ** pw) ** (1.0 / pw)
        th = np.arctan2(dv, du)
        # RAGGED RIM: the same radius in every direction is a drill hole.
        t = t * (1.0 + 0.30 * np.sin(k1 * th + ph1)
                 + 0.16 * np.sin(k2 * th + ph2))
        w = np.clip(1.0 - t, 0.0, 1.0) ** float(taper)
        # only the outer part of the piece moves, full effect at the surface
        s = v[:, ax] * sgn
        s_out = float(s.max())
        reach = max(1.7 * h, 0.30 * float(ext[ax]))
        g = np.clip((s - (s_out - reach)) / max(reach, 1e-9), 0.0, 1.0)
        floor = float(s.min()) + 0.15 * float(ext[ax])
        v[:, ax] = sgn * np.maximum(s - h * w * g, np.minimum(s, floor))
    return v


def roughen_arrays(v, rng, amp_m, lam_scale=5.5, lam_m=None):
    # `amp_m` may be a scalar or a PER-AXIS triple — see the note in `chip_box`
    # about why a plate must not be roughened across its thickness.
    """`roughen`'s band-limited positional noise, on a raw point array.

    Keyed off POSITION (not per-vertex random) so vertices that coincide on a
    shared edge move together and the solid does not tear open — the same
    reason `roughen` above is written that way.

    `lam_m` (round 6) sets the wavelength in METRES instead of deriving it from
    the piece's LONGEST extent. That derivation is wrong for a bar: on a
    0.4 x 0.4 x 3.5 m column `lam_scale / max_extent` puts the wavelength at
    2.9 m, so the "roughening" is one gentle 4 cm bow over the whole shaft
    rather than surface relief, and it contributes nothing to the piece
    reading as broken. Callers that want relief pass a wavelength scaled to
    the piece's SECTION.
    """
    v = np.asarray(v, dtype=float)
    amp = np.asarray(amp_m, dtype=float)
    if amp.ndim == 0:
        amp = np.repeat(amp, 3)
    if not len(v) or not (amp > 0.0).any():
        return v
    ext = v.max(0) - v.min(0)
    L = float(max(ext.max(), 1e-6))
    ph = np.asarray([rng.random() * 100.0 for _ in range(3)], dtype=float)
    fq = (float(lam_scale) / L if lam_m is None
          else 6.2831853 / max(float(lam_m), 1e-6))
    d = np.stack([np.sin(v[:, 1] * fq + ph[0]) * np.cos(v[:, 2] * fq + ph[1]),
                  np.sin(v[:, 2] * fq + ph[1]) * np.cos(v[:, 0] * fq + ph[2]),
                  np.sin(v[:, 0] * fq + ph[2]) * np.cos(v[:, 1] * fq + ph[0])],
                 -1)
    d += 0.45 * np.stack([np.sin(v[:, 1] * fq * 2.7 + ph[2]),
                          np.sin(v[:, 2] * fq * 2.7 + ph[0]),
                          np.sin(v[:, 0] * fq * 2.7 + ph[1])], -1)
    return v + d * amp[None, :]


def warp_mesh(points, rng, warp_m=0.0, twist_deg=0.0, axis=None):
    """Bend and twist a piece by a SMOOTH low-frequency field. Points only.

    A sawn plank that has been through a collapse is bowed and slightly
    wound; a lump of concrete is not, which is why this is opt-in per kind
    rather than part of `chip_box`'s default. The field is a parabolic sag
    (zero at both ends, so the piece keeps its length) plus a half-amplitude
    sine so the bow is not symmetric, with the twist linear in the same
    parameter. Amplitude is clamped to a quarter of the long span: past that
    a "bend" is a fold.

    Deterministic in `rng`; call it AFTER any anisotropic scaling, because
    `warp_m` is metres of real displacement.
    """
    v = np.array(points, dtype=float, copy=True)
    if len(v) < 4 or (warp_m <= 0.0 and abs(twist_deg) < 1e-6):
        return v
    lo, hi = v.min(0), v.max(0)
    ext = hi - lo
    la = int(np.argmax(ext)) if axis is None else int(axis) % 3
    ta = int(np.argmin(ext))
    if ta == la:
        ta = (la + 1) % 3
    ma = 3 - la - ta
    L = float(max(ext[la], 1e-6))
    u = (v[:, la] - lo[la]) / L
    s = 2.0 * u - 1.0
    ph = rng.uniform(0.0, 6.2831853)
    sgn = 1.0 if rng.random() < 0.5 else -1.0
    amp = float(min(float(warp_m), 0.25 * L))
    bow = 1.0 - s * s
    sc = np.sin(6.2831853 * u + ph)
    if amp > 0.0:
        v[:, ta] += sgn * amp * (0.75 * bow + 0.35 * sc)
        v[:, ma] += sgn * amp * 0.30 * (bow * math.cos(ph) + 0.25 * sc)
    if abs(twist_deg) > 1e-6:
        th = math.radians(float(twist_deg)) * s * sgn
        c0 = 0.5 * (lo + hi)
        a = v[:, ta] - c0[ta]
        b = v[:, ma] - c0[ma]
        ct, st = np.cos(th), np.sin(th)
        v[:, ta] = c0[ta] + a * ct - b * st
        v[:, ma] = c0[ma] + a * st + b * ct
    return v


def _chip_normal(rng, kind, la=None, la_sign=None):
    """A unit normal for one chip. See the section note for why |n_i| matters.

      corner  every component >= 0.40 -> a bounded corner wedge
      bevel   ONE component collapses  -> the long diagonal shear a big slab
              really does break along (drawn only at `bevel_p`)
      end     the long axis dominates, both cross axes still substantial ->
              a bite out of the BREAK face of a bar, stepped rather than sawn
    """
    sgn = np.asarray([1.0 if rng.random() < 0.5 else -1.0 for _ in range(3)])
    w = np.asarray([rng.uniform(0.40, 1.0) for _ in range(3)])
    if kind == "end" and la is not None:
        # LONG-DOMINANT, so the new face FACES ALONG THE BAR — it is part of
        # the break, not a shave off the flank. What stops it being one clean
        # saw cut is the DEPTH rule in `_one_chip`, which sizes the bite so it
        # only reaches part way across the section (see `end_relief` there):
        # long-dominant plus a shallow bite is a STEP on the break face, and
        # three or four of those on the same end, at different cross signs and
        # reliefs, are what a snapped bar end actually looks like.
        w = np.asarray([rng.uniform(0.25, 0.60) for _ in range(3)])
        w[la] = rng.uniform(0.60, 0.90)
        if la_sign is not None:
            # THE SAME END, deliberately. Several bites at ONE end face, with
            # only the cross-axis signs varying, overlap into a stepped,
            # irregular break; the same bites spread over both ends and four
            # corners each just bevel the bar.
            sgn[la] = float(la_sign)
    elif kind == "bevel":
        w[rng.randrange(3)] = rng.uniform(0.04, 0.18)
    n = sgn * w
    return n / max(float(np.linalg.norm(n)), 1e-9)


def _chip_depth(rng, depth_frac, big_p):
    """Chip depth as a fraction of the piece's own span along the cut normal.

    LOG-uniform, not uniform: the user asked for "random from small to very
    large chips", and a uniform draw over (0.04, 0.22) gives almost no small
    ones — every piece then loses a similar-sized corner and the population
    reads processed. Log-uniform puts the mass at the small end and
    `big_p` reserves a real chunk-loss tail on top of it.
    """
    lo = max(float(depth_frac[0]), 1e-4)
    hi = max(float(depth_frac[1]), lo * 1.001)
    d = lo * (hi / lo) ** rng.random()
    if rng.random() < float(big_p):
        d = hi * rng.uniform(1.0, 2.0)
    return float(min(d, 0.42))


def chip_box(points=None, faces=None, rng=None, chips=(2, 6),
             depth_frac=(0.04, 0.22), warp_m=0.0, sizes=None, bottom=False,
             rough_frac=0.0, seg=None, twist_deg=0.0, ends=0.0,
             big_p=0.16, bevel_p=0.22, min_loss=0.05, max_loss=0.27,
             bar_aspect=2.2, max_grow=0.35, rough_budget=260,
             bites=(0, 0), bite_frac=(0.34, 0.95),
             gouges=(0, 0), gouge_depth=(0.10, 0.30), gouge_big_p=0.28,
             gouge_big_frac=(0.34, 0.55), gouge_budget=760,
             gouge_vol_frac=0.30, rough_lam_frac=None):
    """Turn a rectangular solid into a broken-looking one. Arrays in/out.

    TWO INPUT FORMS, one output form:

      chip_box(sizes=(sx, sy, sz), rng=r, ...)      builds the box itself
      chip_box(points, faces, r, ...)               chips a solid you have

    Returns `(points, faces)` — float64 `(n, 3)` and int64 `(m, 3)` triangles.
    Always returns arrays; never raises for bad input.

    PASSES THROUGH UNCHANGED, printing nothing, when:
      * `QC_CHIP=0` (`chips_enabled()` is False) — the caller is expected to
        take its own pre-chip path in that case, this is only a backstop;
      * `rng` is None;
      * the input is not a closed triangle solid (fewer than 4 faces, any
        open edge). THIS IS THE SAFETY RULE: a clipped shell is open, and a
        clipped shell is what segfaults `vtkStripper` (round-4 catalogue).
      * VTK is not importable. There is no trimesh fallback here on purpose —
        a chip is cosmetic, and a half-chipped solid is worse than a box.

    ARGUMENTS
      chips        (lo, hi) inclusive count of chips ATTEMPTED. A chip that
                   comes back open, empty, or outside the volume budget is
                   dropped, so the achieved count can be lower.
      depth_frac   (lo, hi) chip depth as a fraction of the piece's span along
                   that chip's own normal. Log-uniform between them, with a
                   `big_p` tail above `hi`.
      ends         0..1 — EXTRA cuts, `clip(round(chips * ends), 2, 4)` of
                   them, aimed at the BREAK faces of a bar-shaped piece (longest
                   extent >= `bar_aspect` x the next); most land on one end,
                   at different cross signs, so they overlap into a stepped
                   fracture. Ignored on a piece that is not bar-shaped.
      rough_frac   band-limited surface noise, as a fraction of the piece's
                   SMALLEST extent (so a plate wobbles by a bit of its
                   thickness, not by metres). Applied AFTER the chips, on a
                   solid subdivided to `rough_budget` triangles, so the BREAK
                   faces get relief too — a fresh concrete fracture is not a
                   plane.
      warp_m       peak bend, metres (see `warp_mesh`). Timber and metal only.
      twist_deg    peak twist about the long axis, degrees.
      seg          segment counts for the built box; defaults from `auto_seg`
                   when the piece is going to be roughened or warped (both
                   need vertices to displace) and (1, 1, 1) when it is not.
      max_grow     how far the finished piece may exceed `sizes` on any axis
                   before it is scaled back (sizes-mode only). Roughening and
                   warping both push the bbox out without costing volume;
                   forcing an exact fit is what pays for that twice.
      min_loss/max_loss
                   volume budget, as a fraction of the INPUT volume. Chips
                   that would push past `max_loss` are rejected; if the whole
                   pass came in under `min_loss` extra chips are attempted at
                   escalating depth, so a chipped piece is always visibly
                   chipped and never sliced in half.

    ROUND 6 — the two knobs the "still perfect cuboids" complaint needed. Both
    default OFF, so a caller that does not set them gets round-5 output byte
    for byte; the `_CHIP_*` spec tables in `quake_rubble_usd` / `quake_collapse`
    set them for the quake populations.

      bites        (lo, hi) count of VERY LARGE corner bites, sized by the
                   CROSS-SECTION rather than by the span along the cut normal
                   (see `_one_chip`'s `bite` branch for why that distinction is
                   the whole point). `bite_frac` is the share of the tightest
                   cross-section reach each one takes; near 1.0 the corner
                   comes off across the whole member.
      gouges       (lo, hi) count of localized scallops bitten out of the
                   surface by `gouge_arrays`, at a station chosen ALONG the
                   piece. THE ONLY MECHANISM HERE THAT DAMAGES THE MIDDLE OF A
                   SLENDER PIECE — every plane clip is anchored at an extreme
                   corner, and a column's corners are all at its two ends, so
                   without this the shaft is a perfect prism no matter what the
                   volume loss says. `gouge_depth` / `gouge_big_frac` /
                   `gouge_big_p` are the small-to-very-large draw.
      gouge_budget triangle budget for the anisotropic refinement a gouge needs
                   to have something to displace (`subdivide_long_edges`).
                   Raising it is what a gouge costs; 760 puts ~13 cm stations
                   on a 3.5 m column.
      rough_lam_frac
                   roughening wavelength as a fraction of the piece's SHORTEST
                   extent. `None` keeps round 5's `lam_scale / longest extent`,
                   which on a bar is a 3 m wavelength — one gentle bow, not
                   surface relief.

    COST: 4.4-8.7 ms per piece on an idle machine — 2-6 corner chips, 2-4 end
    steps, one subdivision to `rough_budget` and the roughening pass, ending at
    66-256 triangles (measured over 40 seeds x 5 kinds; the numbers per kind
    are in the v9 README). Deterministic in `rng` for a given input.
    """
    if sizes is not None:
        sx, sy, sz = [float(q) for q in sizes]
        if seg is None:
            # ONLY THE WARP needs segments up front: it is applied last, to
            # whatever vertices exist, and a bow through eight box corners is
            # a wedge, not a bow. Roughening does NOT need them any more — it
            # runs after `subdivide_to_budget` — and pre-tessellating for it
            # would just make every chip cut a 96-triangle box instead of a
            # 12-triangle one.
            seg = (auto_seg(sx, sy, sz)
                   if (warp_m > 0.0 or abs(twist_deg) > 1e-6) else (1, 1, 1))
        v, f = box_arrays(sx, sy, sz, bottom=bottom, seg=seg)
    else:
        v = np.asarray(points, dtype=float)
        f = np.asarray(faces, dtype=np.int64)
        if v.ndim != 2 or v.shape[1] != 3 or f.ndim != 2 or f.shape[1] != 3:
            return np.asarray(points, dtype=float), np.asarray(faces,
                                                               dtype=np.int64)
        v = v.copy()

    if rng is None or not chips_enabled():
        return v, f
    if len(f) < 4 or len(v) < 4 or not np.isfinite(v).all():
        return v, f
    if open_edge_count(f) > 0:
        return v, f                     # not a solid: never cut it
    if _vtk() is None:
        return v, f

    v0 = mesh_volume(v, f)
    if v0 <= 0.0:
        return v, f

    ext = v.max(0) - v.min(0)
    order = np.argsort(ext)[::-1]
    la = int(order[0])
    is_bar = float(ext[order[0]]) >= float(bar_aspect) * float(
        max(ext[order[1]], 1e-9))
    n_chips = rng.randint(int(chips[0]), int(chips[1]))
    # END STEPS ARE EXTRA, not a share of `chips`. They barely cost any volume
    # (a 3 cm step across half a section), so spending the corner-chip budget
    # on them would leave the piece looking un-chipped everywhere else and
    # then trip the `min_loss` rescue into adding the corner chips back.
    n_end = (int(np.clip(round(n_chips * float(ends)), 2, 4))
             if (is_bar and ends > 0.0) else 0)
    main_end = 1.0 if rng.random() < 0.5 else -1.0
    # BOTH ENDS, always: a bar that broke out of a floor broke at both ends,
    # and leaving one of them a single flat plane is the clean saw cut the
    # user objected to. The far end gets one step, the near end the rest — a
    # break is rarely symmetrical.
    n_far = 1 if n_end >= 2 else 0
    n_bite = (rng.randint(int(bites[0]), int(bites[1]))
              if int(bites[1]) > 0 else 0)
    kinds = ([("end", main_end)] * (n_end - n_far)
             + [("end", -main_end)] * n_far
             + [("bite", None)] * n_bite
             + [(("bevel" if rng.random() < float(bevel_p) else "corner"), None)
                for _ in range(n_chips)])
    rng.shuffle(kinds)

    floor_v = (1.0 - float(max_loss)) * v0
    for kind, la_sign in kinds:
        v, f = _one_chip(v, f, rng, kind, la, depth_frac, big_p, floor_v,
                         la_sign=la_sign, bite_frac=bite_frac)

    # Under-chipped? A piece the draws happened to leave nearly intact is the
    # gift box the whole exercise is about, so force more bites at escalating
    # depth rather than shipping it.
    tries = 0
    while mesh_volume(v, f) > (1.0 - float(min_loss)) * v0 and tries < 6:
        boost = 1.0 + 0.55 * tries
        v, f = _one_chip(v, f, rng, "corner" if tries % 2 else "end", la,
                         (depth_frac[0] * boost, depth_frac[1] * boost),
                         big_p, floor_v,
                         la_sign=(None if tries % 2 else main_end))
        tries += 1

    # ROUGHEN LAST, ON A SUBDIVIDED SOLID. Chipping a smooth box and leaving
    # it there gives big flat facets meeting at clean edges — better than a
    # cuboid, still not a broken casting. Subdividing the chipped piece and
    # then displacing every vertex by a band-limited positional field puts
    # relief on the cast faces AND on the break faces, which is the half of
    # "the actual breaking should not be clean" the chip planes cannot do on
    # their own. It also costs no volume worth speaking of (the field is
    # zero-mean), unlike a fit-to-size, so the chip budget still means what it
    # says.
    n_gouge = (rng.randint(int(gouges[0]), int(gouges[1]))
               if int(gouges[1]) > 0 else 0)
    if rough_frac > 0.0 or n_gouge > 0:
        if n_gouge > 0:
            # ANISOTROPIC REFINEMENT, and the target edge is solved from the
            # budget rather than guessed: a surface of area A carries roughly
            # 2A/t^2 triangles at edge length t, so t = sqrt(2A/budget) lands
            # near the budget in one shot instead of over/undershooting by a
            # factor of four the way a uniform 4-to-1 split does.
            area = _tri_area(v, f)
            tgt = float(np.clip(math.sqrt(2.0 * max(area, 1e-9)
                                          / max(int(gouge_budget), 8)),
                                0.02, 1.0))
            v, f = subdivide_long_edges(v, f, tgt, budget=int(gouge_budget))
            v = gouge_arrays(v, rng, n_gouge, depth_frac=gouge_depth,
                             big_p=gouge_big_p, big_frac=gouge_big_frac,
                             vol_frac=float(gouge_vol_frac),
                             piece_vol=mesh_volume(v, f))
        else:
            v, f = subdivide_to_budget(v, f, budget=rough_budget)
        if rough_frac > 0.0:
            thin = float(max(ext.min(), 1e-6))
            lam = (None if rough_lam_frac is None
                   else max(float(rough_lam_frac) * thin, 1e-3))
            amp = np.repeat(min(float(rough_frac) * thin, 0.30 * thin), 3)
            if rough_lam_frac is not None:
                # ACROSS THE THIN AXIS, ALMOST NOTHING. The round-5 wavelength
                # came off the piece's LONGEST extent, so on a 2.5 m plate the
                # noise was one 2 m bow and its amplitude did not read as
                # texture at all. Shortening it to a fraction of the SECTION
                # (which is what makes it read on a column) turned the same
                # amplitude into a 20 cm corrugation, and 1.7 cm of corrugation
                # on a 12 cm plate is not spalled concrete — it is crumpled
                # foil. Measured on the 2.5 x 1.6 x 0.12 m plate: the pass grew
                # the thickness 35 % (0.120 -> 0.162) and put 6 cm of relief on
                # the deck, and the bench sheets read as cloth. Concrete plates
                # do not bend, so the displacement along the piece's thinnest
                # axis is cut to a third: the rim still gets relief, the deck
                # stays a deck.
                amp[int(np.argmin(ext))] *= 0.35
            v = roughen_arrays(v, rng, amp, lam_scale=7.5, lam_m=lam)

    if sizes is not None and max_grow is not None:
        # A SAFETY NET WITH SLACK, not a hard fit. `roughen_arrays`' field is
        # smooth and zero-mean, so it costs almost no VOLUME — but it does
        # push the bbox out by about its own amplitude on each side, and
        # `warp_mesh` bows across the thin axis on top of that. Scaling the
        # piece back to exactly `sizes` would pay for that in volume twice
        # over (measured: a 14 %-roughened lintel lost 48 % of its volume to
        # the fit, against 19 % to the chips it was supposed to lose it to).
        # So the piece is allowed to breathe `max_grow`, and only a genuinely
        # runaway axis is pulled back — the planner's `bury` is a fraction of
        # the piece's own rotated thickness, which tolerates a few per cent
        # and does not tolerate a piece twice the size it asked for.
        want = np.abs(np.asarray(sizes, dtype=float)) * (1.0 + float(max_grow))
        ext = np.maximum(v.max(0) - v.min(0), 1e-9)
        sc = np.minimum(1.0, want / ext)
        if (sc < 1.0 - 1e-9).any():
            c = 0.5 * (v.max(0) + v.min(0))
            v = c + (v - c) * sc

    # AND THE BUDGET IS MADE EXACT, last. `roughen_arrays`' field is zero-mean
    # in DISPLACEMENT, which is not the same as zero-mean in volume: the
    # surface integral of `d . n` over a piece this small comes out several
    # points either way, and measured over 40 seeds it pushed a column to 34 %
    # loss and a slab to 0.4 %. Both are outside the band the caller asked for,
    # and both are fixed by a uniform scale of a few per cent about the bbox
    # centre — invisible, shape-preserving, and it turns `min_loss`/`max_loss`
    # from a tendency into a guarantee. (It can put the bbox a few per cent
    # past `max_grow`; that is the cheaper of the two errors.)
    vol = mesh_volume(v, f)
    lo_t, hi_t = (1.0 - float(max_loss)) * v0, (1.0 - float(min_loss)) * v0
    if vol > 0.0 and (vol < lo_t or vol > hi_t):
        k = (min(max(vol, lo_t), hi_t) / vol) ** (1.0 / 3.0)
        c = 0.5 * (v.max(0) + v.min(0))
        v = c + (v - c) * k

    # THE WARP IS APPLIED AFTER THE CLAMP, deliberately. A bowed 3.2 m board
    # displaced 4 cm HAS a bbox 4 cm wider across the bow — that is what a bow
    # is — so taxing it against `sizes` would either undo the bend or charge
    # the piece a quarter of its volume for having one (measured: the joist
    # row lost 60 % when the clamp ran last).
    if warp_m > 0.0 or abs(twist_deg) > 1e-6:
        v = warp_mesh(v, rng, warp_m=warp_m, twist_deg=twist_deg, axis=la)
    return v, f


def _one_chip(v, f, rng, kind, la, depth_frac, big_p, floor_v, la_sign=None,
              end_relief=(0.25, 0.90), bite_frac=(0.34, 0.95)):
    """Attempt one chip; return the chipped arrays or the originals."""
    n = _chip_normal(rng, kind, la=la, la_sign=la_sign)
    proj = v @ n
    hi = float(proj.max())
    span = hi - float(proj.min())
    if span <= 1e-6:
        return v, f
    if kind == "bite":
        # A VERY LARGE CORNER BITE, SIZED BY THE SECTION. `depth_frac * span`
        # (the `else` branch) measures the depth against the piece's extent
        # ALONG THE NORMAL, which on a 3.5 m column is dominated by its LENGTH
        # — so `depth_frac=(0.025, 0.15)` never meant "2.5-15 % of the
        # section", and there was no way to ask for the big bite the user did
        # ask for ("random from small to VERY LARGE chips"). Sizing `d` off the
        # tightest reach across the two SMALLEST axes makes `bite_frac` mean
        # exactly "this share of the cross-section goes", and a draw near 1.0
        # takes the whole corner off across the member.
        ext = v.max(0) - v.min(0)
        order = np.argsort(ext)
        # THE TWO LARGEST AXES, not the two smallest. The reach of a plane cut
        # along axis i is `d / |n_i|`, so whichever axis is SHORTEST is the one
        # the cut passes clean through — sizing `d` by it makes the bite
        # vanish. On a 2.5 x 1.6 x 0.12 m plate the shortest axis is the
        # 12 cm thickness, and sizing off it gave a 4 cm nick; sizing off the
        # plan gives a half-metre corner chunk that cuts through the thickness,
        # which is what a broken slab corner is. On a column the two largest
        # are the length and one section axis, so the bite still reaches
        # 32-78 % across the member.
        a, b = int(order[2]), int(order[1])
        cut = min(float(ext[a]) * max(abs(float(n[a])), 1e-3),
                  float(ext[b]) * max(abs(float(n[b])), 1e-3))
        d = float(max(cut, 1e-6)) * rng.uniform(*bite_frac)
    elif kind == "end" and la is not None:
        # RELIEF, not depth-along-the-normal. The cut reaches `d/|n_i|` along
        # axis i, so sizing `d` by the SECTION (`relief` x the tightest
        # `ext_i * |n_i|` over the two cross axes) is what keeps the bite to a
        # fraction of the section instead of taking the whole end off in one
        # plane — a 3.2 x 0.10 x 0.20 m joist gets 2-6 cm steps on its break
        # face. Sized by `span` (the `else` branch) the same plane would
        # remove 0.3 m of bar: a clean angled cut, which is what the user
        # rejected.
        ext = v.max(0) - v.min(0)
        cut = min(float(ext[i]) * max(abs(float(n[i])), 1e-3)
                  for i in range(3) if i != la)
        d = float(max(cut, 1e-6)) * rng.uniform(*end_relief)
    else:
        d = _chip_depth(rng, depth_frac, big_p) * span
    d = float(np.clip(d, 0.004 * span, 0.42 * span))
    got = _chip_clip(v, f, -n, n * (hi - d))
    if got is None:
        return v, f
    nv, nf = weld_arrays(got[0], got[1])
    if len(nf) < 4 or len(nv) < 4 or not np.isfinite(nv).all():
        return v, f
    if open_edge_count(nf) > 0:
        return v, f                     # the cap came back short — drop it
    vol = mesh_volume(nv, nf)
    if vol < floor_v or vol <= 0.0:
        return v, f                     # would breach the volume budget
    return nv, nf
