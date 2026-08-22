"""rubble stage — debris geometry that is not a rectangle.

THE PROBLEM WITH KIT PIECES
---------------------------
`damage.break_up` makes debris by scattering the kit's own wall panels. It is
cheap and the material matches, but every piece is a clean rectangular slab, so
a collapsed house reads as a stack of intact panels rather than as wreckage.
Some of the U-shaped modules survive whole and are recognisable as doorways
lying on their side.

There is no fracture engine in this build — no Blast — and trimesh's mesh
slicing needs `shapely`, which is not installed either. So debris is BUILT
rather than cut: irregular convex solids generated from jittered point clouds
with `scipy.spatial.ConvexHull`, which needs nothing that is not already here.

THE TECHNIQUES, so they can be compared side by side
----------------------------------------------------
    kit        the existing approach — whole kit panels. The baseline.
    chunk      compact irregular solids. Broken masonry, roof fragments.
    splinter   long thin shards with one dominant axis. Broken framing timber,
               which is what most of a burnt timber-frame house actually is.
    shard      flat angular plates. Sheet material — cladding, ply, plasterboard.
    graded     a size-graded mix of the three, many small and few large, which
               is how real debris fields are distributed.

CHIPPED EDGES
-------------
A raw convex hull is faceted but its edges are still clean. `chip` pushes
vertices along their own normal by noise, which breaks the silhouette and
reads as splintering at any distance where the piece is more than a few pixels.
"""

import math

import numpy as np
from scipy.spatial import ConvexHull

TECHNIQUES = ("kit", "chunk", "splinter", "shard", "graded")

# Rough proportions per style, before per-piece jitter. Metres.
_PROFILE = {
    "chunk":    ((0.9, 1.0, 0.7), 14, 0.16),
    "splinter": ((3.4, 0.16, 0.22), 10, 0.10),
    "shard":    ((1.5, 1.3, 0.10), 12, 0.13),
}


def chunk_mesh(rng, style="chunk", size=1.0, chip=0.18):
    """One irregular convex solid. Returns (points, counts, indices).

    Points are drawn from a Gaussian cloud scaled to the style's proportions,
    so the hull is irregular in a way that still has a dominant shape — a
    splinter stays long, a shard stays flat. `chip` then displaces each hull
    vertex outward along its own direction, which is what stops the result
    looking like a cut gemstone.
    """
    prop, n_pts, base_chip = _PROFILE.get(style, _PROFILE["chunk"])
    scale = np.array(prop, dtype=float) * float(size)

    # /2.6 because a Gaussian cloud spans roughly +-2.6 sigma, so without
    # it `size=1.0` produced 4-8 m pieces rather than 1 m ones.
    p = rng.normal(size=(n_pts, 3)) * scale / 2.6
    # Squash one side at random so pieces are not centrally symmetric.
    p[:, rng.integers(0, 3)] *= rng.uniform(0.45, 1.0)

    hull = ConvexHull(p)
    verts = p[hull.vertices]
    remap = {old: i for i, old in enumerate(hull.vertices)}
    faces = np.array([[remap[i] for i in s] for s in hull.simplices])

    # Chip the edges.
    c = float(chip if chip is not None else base_chip)
    if c > 0.0:
        d = verts - verts.mean(0)
        norm = np.linalg.norm(d, axis=1, keepdims=True) + 1e-9
        verts = verts + (d / norm) * rng.normal(scale=c, size=(len(verts), 1)) * scale.mean()

    counts = np.full(len(faces), 3, dtype=int)
    return verts, counts, faces.ravel()


def write_mesh(stage, path, verts, counts, indices, display_color=None):
    """Author a UsdGeom.Mesh from raw arrays."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, v))
                                         for v in verts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([int(c) for c in counts]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in indices]))
    lo = Gf.Vec3f(*map(float, np.min(verts, 0)))
    hi = Gf.Vec3f(*map(float, np.max(verts, 0)))
    mesh.CreateExtentAttr([lo, hi])
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    if display_color is not None:
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*display_color)])
    return mesh


# Size grading: how many pieces of each scale. Many small, few large — the
# distribution real debris fields follow, and the thing that stops a pile
# looking like a handful of identical rocks.
_GRADE = ((0.28, 0.55), (0.45, 0.30), (0.85, 0.15))


def scatter(stage, parent_path, centre, rng, technique="graded", count=48,
            radius_m=7.5, size_m=1.0):
    """Generate and place a debris field. Returns the prim paths created."""
    from pxr import Gf, Sdf, UsdGeom

    if technique == "kit":
        return []

    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    styles = ({"chunk": ["chunk"], "splinter": ["splinter"],
               "shard": ["shard"]}.get(technique)
              or ["splinter", "splinter", "chunk", "shard"])

    made = []
    for i in range(int(count)):
        # numpy Generator throughout, not Python's `random` — `chunk_mesh`
        # needs `normal`/`integers`, and mixing the two APIs is what made
        # `randrange` blow up here.
        style = styles[int(rng.integers(0, len(styles)))]
        if technique == "graded":
            r = rng.random()
            acc = 0.0
            grade = _GRADE[0][0]
            for g, w in _GRADE:
                acc += w
                if r <= acc:
                    grade = g
                    break
        else:
            grade = rng.uniform(0.35, 0.9)

        verts, counts, idx = chunk_mesh(
            rng, style=style, size=size_m * grade,
            chip=rng.uniform(0.10, 0.26))

        path = "{0}/debris_{1:03d}".format(parent_path, i)
        mesh = write_mesh(stage, path, verts, counts, idx)

        ang = rng.uniform(0.0, 2.0 * math.pi)
        rad = radius_m * math.sqrt(rng.random())
        x = centre[0] + math.cos(ang) * rad
        y = centre[1] + math.sin(ang) * rad
        z = float(np.max(verts[:, 2]) - np.min(verts[:, 2])) * 0.5 + 0.02

        xf = UsdGeom.Xformable(mesh)
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(rng.uniform(0, 360),
                                         rng.uniform(0, 360),
                                         rng.uniform(0, 360)))
        made.append(path)
    return made
