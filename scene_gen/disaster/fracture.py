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
