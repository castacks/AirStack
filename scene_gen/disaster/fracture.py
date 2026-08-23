"""fracture — cut a solid into chunks along Voronoi cells.

The engine, with no opinion about what caused the damage. A caller supplies
``seed_fn`` to say WHERE the fragments should be small, which is the whole hook
a damage model needs — see ``earthquake.shake``."""

from __future__ import annotations

import numpy as np
import trimesh
from scipy.spatial import ConvexHull, HalfspaceIntersection, cKDTree

from .solids import close_directly, solidify

# ---------------------------------------------------------------------------
# Geometry — no Isaac, no USD.
#
# scipy's half-space convention is `A @ x + b <= 0`, one row per plane, so a
# "cell" here is just a stack of those rows handed to HalfspaceIntersection
# together with a point known to be inside it (its own seed).
# ---------------------------------------------------------------------------


def hull_halfspaces(points: np.ndarray) -> np.ndarray:
    """The convex hull of *points* as half-spaces, already `Ax + b <= 0`.

    Bounding the Voronoi cells by the hull rather than by the object keeps the
    cell construction convex and cheap. The object's real (possibly very
    non-convex) shape is imposed afterwards by boolean intersection.
    """
    return ConvexHull(points).equations


def _cell(seed, others, bound, inset):
    """One Voronoi cell clipped to *bound*: `(vertices, bisector_planes)`.

    `inset` moves each bisector toward its own seed by that many metres, which
    is what opens the hairline gap between neighbours. It is applied to the
    bisectors only — `bound` is passed through untouched, so nothing shrinks
    the object's own surface.

    The normalised bisector planes come back with the vertices because they are
    exactly the set of planes a *new* (interior) face can lie on, which is how
    `classify_faces` tells cut from original without any geometric guesswork.
    """
    d = others - seed
    n = np.linalg.norm(d, axis=1, keepdims=True)
    n[n == 0] = 1.0
    unit = d / n
    mid = (others + seed) * 0.5
    off = -(unit * mid).sum(1) + inset
    planes = np.column_stack([unit, off])
    try:
        pts = HalfspaceIntersection(np.vstack([bound, planes]), seed).intersections
    except Exception:
        # A seed left outside its own clipped cell (possible right at the
        # boundary after relaxation) gives the intersection no interior point.
        return None, planes
    return pts, planes


def seed_points(parts, n: int, rng) -> np.ndarray:
    """*n* points uniformly inside the solid, by rejection sampling.

    Spread across parts in proportion to volume, so a thickened building seeds
    its big wall slabs more heavily than its trim and the chunk sizes stay even.
    """
    vols = np.array([abs(p.volume) for p in parts], dtype=np.float64)
    share = np.maximum((vols / vols.sum() * n).astype(int), 0)
    share[np.argmax(vols)] += n - share.sum()

    got = []
    for part, want in zip(parts, share):
        have = 0
        guard = 0
        while have < want and guard < 40:
            # volume_mesh draws in the AABB and keeps what is inside, so the
            # yield is the volume-to-AABB ratio; oversample rather than loop.
            batch = trimesh.sample.volume_mesh(part, max(4 * (want - have), 32))
            guard += 1
            if len(batch):
                got.append(batch)
                have += len(batch)
    pts = np.vstack(got) if got else np.zeros((0, 3))
    return pts[rng.permutation(len(pts))[:n]]


class SurfaceLookup:
    """Nearest point on the *original* surface, with its UV and material.

    Needed because neither the boolean nor the voxel remesh carries appearance
    through. Brute-force closest-point over every source triangle is
    O(queries x triangles) and does not finish on a real asset, so a KD-tree
    over triangle centroids proposes `k` candidates per query and exact
    point-triangle distance decides among those.
    """

    def __init__(self, vertices: np.ndarray, faces: np.ndarray,
                 tri_uv: np.ndarray, tri_mat: np.ndarray):
        self.tris = vertices[faces]
        self.tri_uv, self.tri_mat = tri_uv, tri_mat
        self.tree = cKDTree(self.tris.mean(axis=1))
        self.mesh = trimesh.Trimesh(vertices, faces, process=False)

    def query(self, points: np.ndarray, normals=None, offset: float = 0.0,
              k: int = 12):
        """`(uv, material_id)` for each query point.

        Ray cast inward from just outside the point, and take the first hit:
        the first surface a ray meets coming from outside *is* the one you can
        see, which is exactly the definition of "outer appearance" wanted here.

        Closest-triangle is the obvious alternative and is not good enough on
        real art. Building assets are layered — this house is a full
        construction model, with shingles over building paper over OSB
        sheathing over studs over drywall, several surfaces inside one voxel of
        each other. The resampled surface sits somewhere in that stack, so
        nearest-triangle picks an inner layer almost as often as the outer one.
        Measured on 400 roof samples whose right answer is asphalt shingle:
        closest-triangle 50%, ray cast 92%. (An outwardness heuristic was tried
        in between and scored 37% — worse than plain distance, because it let
        trim geometry 19 cm away outrank the shingle actually underfoot.)

        Rays that hit nothing — concave pockets, or an origin pushed inside
        some other part of the object — fall back to closest-triangle.
        """
        n = len(points)
        tri_id = np.full(n, -1, dtype=np.int64)
        hit = np.zeros((n, 3))

        if normals is not None and offset > 0:
            loc, i_ray, i_tri = self.mesh.ray.intersects_location(
                points + normals * offset, -normals, multiple_hits=False)
            if len(i_ray):
                tri_id[i_ray] = i_tri
                hit[i_ray] = loc

        miss = np.nonzero(tri_id < 0)[0]
        if len(miss):
            kk = min(k, len(self.tris))
            _, cand = self.tree.query(points[miss], k=kk)
            cand = np.atleast_2d(cand.reshape(len(miss), kk))
            flat = trimesh.triangles.closest_point(
                self.tris[cand.ravel()], np.repeat(points[miss], kk, axis=0))
            dist = np.linalg.norm(
                flat - np.repeat(points[miss], kk, axis=0), axis=1)
            best = np.argmin(dist.reshape(len(miss), kk), axis=1)
            tri_id[miss] = cand[np.arange(len(miss)), best]
            hit[miss] = flat.reshape(len(miss), kk, 3)[np.arange(len(miss)), best]

        bary = trimesh.triangles.points_to_barycentric(self.tris[tri_id], hit)
        uv = np.einsum("ij,ijk->ik", bary, self.tri_uv[tri_id])
        travel = np.linalg.norm(hit - points, axis=1)
        return uv, self.tri_mat[tri_id], travel


def classify_faces(chunk: trimesh.Trimesh, planes: np.ndarray, tol: float):
    """True for faces created by the cut, False for original surface.

    Exact rather than heuristic: a cut face lies in one of the cell's bisector
    planes by construction, so testing the face centroid against those planes
    decides it outright.
    """
    c = chunk.triangles.mean(axis=1)
    on = np.abs(c @ planes[:, :3].T + planes[:, 3]) < tol
    return on.any(axis=1)


def fracture_solid(parts, n_chunks=80, seed=0, gap=0.004, relax=2, quiet=False,
                   seed_fn=None):
    """Cut a solid into convex-cell chunks: a list of `(mesh, planes)`.

    `parts` is a list of closed meshes rather than one, because thickening a
    shell asset yields hundreds of separate slabs and fusing them into a single
    manifold is both expensive and unnecessary — a cell is intersected against
    each part it overlaps and the results are collected. An AABB test skips the
    parts a cell cannot touch, which is nearly all of them.

    `relax` iterations of Lloyd's algorithm even the chunks out — raw uniform
    seeds leave a long tail of slivers, which look wrong and give PhysX
    degenerate inertia tensors. Relaxation runs against the hull-clipped cell
    (cheap, closed form) rather than the true chunk, because doing it against
    the boolean result would mean a full fracture per iteration for a
    second-order improvement.
    """
    rng = np.random.default_rng(seed)
    np.random.seed(seed)  # trimesh.sample draws from the global stream
    bound = hull_halfspaces(np.vstack([p.vertices for p in parts]))
    # `seed_fn` lets a caller place the seeds itself instead of scattering them
    # uniformly. Where the seeds are IS the fracture pattern, so this is the
    # whole hook a damage model needs: crowd them where the structure failed
    # and the cells there come out small, leave a region unseeded and it
    # survives as one big block. See tools/earthquake_damage.py.
    pts = (seed_fn or seed_points)(parts, n_chunks, rng)
    if not len(pts):
        return []

    for _ in range(relax):
        moved = []
        for i, p in enumerate(pts):
            v, _ = _cell(p, np.delete(pts, i, axis=0), bound, 0.0)
            moved.append(p if v is None or not len(v) else v.mean(axis=0))
        pts = np.asarray(moved)

    boxes = np.array([p.bounds for p in parts])          # (P, 2, 3)
    chunks = []
    for i, p in enumerate(pts):
        v, planes = _cell(p, np.delete(pts, i, axis=0), bound, gap * 0.5)
        if v is None or len(v) < 4:
            continue
        cell = trimesh.convex.convex_hull(v)
        lo, hi = cell.bounds
        near = np.nonzero(~((boxes[:, 0] > hi).any(1) | (boxes[:, 1] < lo).any(1)))[0]

        pieces = []
        for j in near:
            try:
                got = trimesh.boolean.intersection([parts[j], cell])
            except Exception:
                continue
            if not got.is_empty and got.volume > 0:
                pieces.append(got)
        if not pieces:
            continue

        # One cell is one chunk, even when it catches several parts that do
        # not touch — a scrap of wall and the trim beside it. They are all
        # bounded by the same convex cell, so they stay together and fly
        # together, which is what a fractured building does. Splitting here
        # instead gave 2635 bodies from 30 cells on the test house: a cloud of
        # confetti, each scrap its own rigid body.
        merged = trimesh.util.concatenate(pieces)
        if merged.volume > 0 and len(merged.faces) >= 4:
            chunks.append((merged, planes))
    if not quiet:
        print(f"[fracture] {len(chunks)} chunks from {len(pts)} cells", flush=True)
    return chunks


def build_chunks(source, n_chunks=80, seed=0, gap=0.004, relax=2,
                 voxel_res=192, lift=0.02, quiet=False, force_voxel=False,
                 interior_tile=0.5, seed_fn=None):
    """Full geometry pipeline: solidify, fracture, reproject appearance.

    `source` is a `Source`. Returns dicts carrying everything the USD writer
    needs and nothing it does not: local points, triangles, per-face-vertex
    UVs, a per-face material id (-1 for cut faces) and the world centroid.
    """
    surface = trimesh.Trimesh(source.vertices, source.faces, process=False)
    parts = getattr(source, "parts", None)
    if parts:
        tier = "thickened"
    else:
        parts, tier = solidify(surface, voxel_res, quiet, force_voxel)

    # Placement waits until here because tier 3 invents its own bounds: the
    # marching-cubes surface overshoots the source by up to a voxel, so a lift
    # applied to the source still left the object 25 mm into the ground.
    allb = np.array([p.bounds for p in parts])
    lo, hi = allb[:, 0].min(axis=0), allb[:, 1].max(axis=0)
    shift = np.array([0.0, 0.0, 0.0])
    shift[:2] = -(lo + hi)[:2] * 0.5
    shift[2] = lift - lo[2]
    for p in parts:
        p.apply_translation(shift)
    verts = source.vertices + shift
    span = float((hi - lo).max())

    if not quiet:
        vol = sum(p.volume for p in parts)
        nf = sum(len(p.faces) for p in parts)
        print(f"[fracture] solid via {tier}, {len(parts)} part(s), "
              f"volume {vol:.2f} m^3, {nf} faces", flush=True)

    look = SurfaceLookup(verts, source.faces, source.tri_uv, source.tri_mat)
    tol = max(span * 1e-4, 1e-6)
    # Ray origins start this far out along the face normal. Scaled to the
    # solidifier's own error, since that is how far the resampled surface can
    # sit from the original: too small and the origin starts under the skin,
    # too large and it can wander inside a neighbouring part of the object.
    # Ray origins start this far out along the face normal, and `skin` is how
    # far off the original surface a face may sit and still count as being ON
    # it. Both scale with how badly the solidifier moved the surface — which is
    # a whole voxel for tier 5 and NOTHING for every other tier, where the
    # outer faces are the original triangles.
    #
    # Sharing tier 5's numbers with the rest was a real bug, not a tuning nit.
    # At a 83 mm offset a ray fired from a wall meets the window trim standing
    # in front of it before it meets the wall, 83 mm of travel against a 21 mm
    # skin, so the wall was ruled "not the original surface" and painted with
    # the white interior material. Most of the house rendered as bare white
    # with a few correctly-textured shards, which read as the shards being the
    # defect when they were the only part working.
    if tier.startswith("voxelised"):
        offset = 2.0 * span / float(voxel_res)
        skin = 0.25 * offset
    else:
        offset = max(2e-3, 2e-4 * span)
        skin = 0.5 * offset

    out = []
    for piece, planes in fracture_solid(parts, n_chunks, seed, gap, relax,
                                       quiet, seed_fn):
        inner = classify_faces(piece, planes, tol)
        # UVs are per face-vertex: a vertex shared by a cut face and an
        # original one needs a different value on each, which vertex
        # interpolation cannot express.
        corners = piece.triangles.reshape(-1, 3)
        corner_n = np.repeat(piece.face_normals, 3, axis=0)
        uv, mat, travel = look.query(corners, corner_n, offset)
        uv = uv.reshape(-1, 3, 2)
        mat = mat.reshape(-1, 3)
        # A face that is neither a cut plane nor anywhere near the original
        # surface is one the solidifier ADDED — the inner skin of a thickened
        # slab, or the rim joining it to the outer skin. Texturing it from
        # whatever a ray happens to hit paints interior layers onto surfaces
        # that never had them; it is interior, so it goes white.
        inner |= travel.reshape(-1, 3).min(axis=1) > skin

        # One material per face: the corner materials agree except on faces
        # straddling a material seam, where the majority is the sane pick.
        face_mat = np.array([np.bincount(m).argmax() for m in mat])
        face_mat[inner] = -1

        # Cut faces get a planar projection instead of the source UVs, which
        # mean nothing on a surface that did not exist a moment ago. Pinning
        # them to (0,0) — the old behaviour, fine for flat white — samples one
        # pixel of a texture and returns a flat smear. Projecting on each
        # face's dominant axis keeps the grain the right size, and using WORLD
        # coordinates means neighbouring chunks share one continuous grain, so
        # a fracture looks split rather than independently wallpapered.
        if inner.any():
            axis = np.argmax(np.abs(piece.face_normals), axis=1)
            plane = {0: (1, 2), 1: (0, 2), 2: (0, 1)}
            proj = np.zeros((len(piece.faces), 3, 2))
            for a, (i, j) in plane.items():
                pick = axis == a
                if pick.any():
                    proj[pick] = piece.triangles[pick][:, :, [i, j]]
            uv[inner] = proj[inner] / max(interior_tile, 1e-6)

        centroid = piece.center_mass
        out.append({
            "points": piece.vertices - centroid,
            "faces": piece.faces,
            "uv": uv,
            "face_mat": face_mat,
            "centroid": centroid,
        })
    return out
