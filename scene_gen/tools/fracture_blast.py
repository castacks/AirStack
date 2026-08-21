#!/usr/bin/env python3
"""fracture_blast.py — anything, already broken, and it knows it.

    cd AirStack
    # the default cylinder
    UV_ENV_FILE=$PWD/.env.host uv run python scene_gen/tools/fracture_blast.py

    # any asset an asset-pack YAML could name
    ... fracture_blast.py --asset objaverse://6644de89c2f0449db3de934744162b63
    ... fracture_blast.py --asset omniverse://host/Library/Stages/foo.usd
    ... fracture_blast.py --asset airstack://scene_gen/assets/aec/thing.usd

(`isaacsim` is a default dependency *group*, so plain `uv run` already has it;
`.env.host` supplies the EULA acceptance Kit otherwise prompts for on stdin.)

Opens a local Isaac Sim window (orbit/pan as usual) holding what looks like one
solid object. It is not: it is ~80 chunks cut from the object along shared
Voronoi planes and set down in exactly the pose the cut left them in. Press
SPACE and they fly.

    SPACE   detonate — radial impulse from the blast centre
    R       re-form (poses restored, velocities zeroed, held again)
    ESC     quit

SOURCE ASSETS
-------------
`--asset` takes the same strings an asset-set YAML does, resolved through the
generator's own `scene_generator._expand_scheme`, so there is one spelling of
an asset in this repo and this tool did not invent a second:

    objaverse://<32-hex-uid>   downloaded and converted on demand via
                               `objaverse_assets.ensure` — same cache the
                               scene generator reads
    omniverse://…              read straight off Nucleus. Works because this
                               runs inside Kit; `tools/localize_nucleus_assets`
                               exists precisely because usd-core on the host
                               cannot resolve these
    airstack://<repo path>     repo-local packs
    /abs/path.usd              a plain file

`--target-size` rescales the asset to that many metres on its largest plan
dimension, mirroring `target-size-m` in the asset sets, so a blast tuned on the
cylinder still reads correctly on a house.

SOLIDIFYING WITHOUT RESAMPLING
------------------------------
Fracturing into *solid* chunks needs a closed volume to cut, and art assets are
overwhelmingly not one. Tiers, cheapest first; the first three never touch the
outer surface:

1. already watertight    cut it directly
2. small holes           `fill_holes` + `fix_normals`, then as above
3. several closed solids cut the parts directly — see below
4. open shells           `thicken()` — extrude each shell INWARD into a slab
5. `--force-voxel`       voxelise + flood-fill + re-surface + snap (diagnostic)

Tier 3 exists because `is_watertight` asks "is this ONE closed surface?", which
is stricter than what fracturing needs: "does this bound a volume?" A bag of
separate closed solids does. Six quads arranged as a cube are one solid once
their corners weld, and an asset can be hundreds of closed parts that were
simply never joined. Demanding one connected surface sent 4 of the 29 assets
that used to reach tier 4 off to be thickened when they were already solid,
discarding the exact outer surface they arrived with. Cutting takes a list of
parts natively, so there is nothing to fuse — the parts are cut as they are.

The library splits cleanly around `MULTI_SOLID_AREA`: these assets carry
95.0-98.9% of their area in closed parts, the next asset down is 79.7%, and the
buildings are 20-40%. The open remainder is dropped and the percentage
reported, so it is never silent.

**Tier 3 is the one that matters**, because tier 4 is what a shell asset used
to fall to and it visibly wrecked the model: marking surface voxels on both
sides inflates the solid by half a voxel, and a binary grid holds no sub-voxel
information, so flat faces re-surface as staircases. Fattened silhouettes,
rounded corners, sawtooth along every window and roof edge.

Extruding inward has none of that, because it *adds* rather than resamples: the
original triangles stay exactly where they are and become the outer skin, and
all new geometry (an inner skin and a rim) is hidden inside. Measured on the
test house, the bounding box is identical before and after to three decimals,
and it is ~3.5x faster than tier 4 besides.

The extrusion is `disaster/mesh_damage.py:solidify_prims`, already in this repo
for exactly this reason — its comment reads "a CLOSED fragment needs a closed
input". It handles the two traps: welding UV-split vertices before averaging
normals (unwelded, 4.8% of components come out closed; welded, 47%), and
choosing "inward" per *point*, because winding is unreliable on converted
assets.

Only components that come out closed are kept — 1218 of 2570 on the test house,
holding 33.8 m^3 against ~25 m^3 predicted for a hollow envelope of this size.
The kept set's bounding box still matches the source exactly, so the discarded
remainder is interior layer scrap with no enclosed volume, not structure.

A thickened building is HOLLOW, so its chunks are slab fragments rather than
solid blocks. `--fill` asks for solid blocks by fusing the parts and keeping
only the outermost boundary sheet. It works on an asset that cleanly encloses
voids and **does not work on a layered shell asset**, where the extruded layers
overlap and the union self-intersects (the test house fragments into 29316
sheets whose largest is an inverted volume). Every failure path there falls
back to hollow and says so, because a broken solid silently yields zero chunks.

KNOWN ARTEFACT: white shards around window trim. `solidify_prims` picks the
extrusion direction per point by flipping away from the building centre, and
its radial test only fires on normals leaning inward by more than ~8.6 degrees.
A normal TANGENTIAL to that direction — the top face of a window sill, +Z on a
facade facing +X — therefore falls back to a sign averaged over the whole prim,
which for a prim holding a wall and a hundred pieces of trim is arbitrary.
Those points extrude outward and their new skin erupts through the facade:
19.6% of vertices displaced by a full thickness, on 414 of 1216 pieces. It is
geometric, not shading — colouring those faces white (correct, they are not
original surface) leaves them in place, and it is identical at 12, 4 and 2 cm.

FOUR FIXES WERE TRIED AND NONE BEAT THE BASELINE. Recorded so the next attempt
starts further along:

1. Orientation per connected COMPONENT rather than per prim, from each piece's
   own enclosed volume (divergence theorem). Principled, but only 222 of 1821
   components are closed, so almost everything still fell through to the
   radial test: 19.6% -> 18.6%. Not worth a per-triangle union-find in code
   that runs in-sim on 200k-point meshes.
2. A ray oracle injected into `solidify` — cast both ways, call the side with
   the FARTHER first hit "outward". Fired on 94.7% of points and flipped
   52.7% of them, and made the render clearly worse. The rule is wrong for a
   SANDWICHED layer: OSB has siding 1 cm one way and drywall 10 cm the other,
   so it flips and drives its skin out through the siding.
3. Culling interior layers by "does a ray along the normal escape". No epsilon
   works. Too small and a roof's own paper blocks its shingles, so the roof
   culls as interior and vanishes (60 of 126 m^2 surviving); too large and the
   paper escapes past the shingles and both are kept, rendering as brown
   patches over black.
4. Culling by true first-hit visibility — parallel ray sweeps in from 14
   directions, keep the first face each ray meets. Exact, no epsilon, and it
   does select single outermost layers (4006 of 26935 faces). But the kept
   roof sheets then fail to close under extrusion and get dropped, so the roof
   vanishes again. The unsolved part is CLOSURE of a culled open sheet, not
   the culling; that is where a fifth attempt should start.

KEEPING THE TEXTURES THROUGH ALL THAT
-------------------------------------
Boolean intersection discards UVs (trimesh hands the result back as
`ColorVisuals`), and tier-3 solidification discards the original vertices
outright. So the outer appearance is *reprojected* rather than carried:

- **Which faces are outer** is decided exactly, not by guessing. A chunk is the
  object intersected with one convex Voronoi cell, so every new face lies on
  one of that cell's bisector planes — those planes are known analytically, and
  a face whose centroid sits on one is interior. Everything else survived from
  the original surface.
- **What those faces look like** is resolved by casting a ray inward from just
  outside the face and taking the first hit — the first surface a ray meets
  coming from outside is the one you can see. Its barycentric coordinates
  interpolate the source `st`, recovering both the UV and which source material
  the face belonged to, so a multi-material asset stays multi-material.

  Closest-triangle is the obvious alternative and is not good enough. Real
  building assets are layered: this house is a full construction model, with
  shingles over building paper over OSB sheathing over studs over drywall, all
  within a voxel of each other, so nearest-triangle picks an interior layer
  almost as often as the exterior. On 400 roof samples whose right answer is
  asphalt shingle: closest-triangle 50%, ray cast 92%. It is the difference
  between a house blotched with drywall and one that looks like the source.
  (Ray casting wants `embreex`; without it trimesh falls back to a much slower
  pure-Python intersector.)
- **The materials themselves** are not re-authored. The source asset is
  referenced onto the stage under an invisible prim and the chunks bind to its
  existing `Material` prims, so shader networks and textures arrive intact.

Interior faces get a plain white material, per the brief.

Cut faces are inset by `--gap`/2 along interior planes only, so neighbours
never touch and the outer surface stays on the original geometry.

WHAT HOLDS IT TOGETHER UNTIL YOU PRESS SPACE
--------------------------------------------
The chunks are real rigid bodies from the first frame, but they are held
**kinematic** until the blast. Merely switching gravity off is not enough,
which is worth knowing because it is the obvious thing to reach for: bodies
still accumulate small solver and depenetration impulses, and with no gravity
to absorb them the drift never decays — measured at 98 mm over two seconds
with the object resting on the ground, and still 38 mm lifted clear. Both are
plainly visible. A kinematic body ignores contacts and forces outright, and
measures 0.00 mm. SPACE flips every chunk to dynamic and assigns velocities in
the same breath, so gravity and the blast arrive together.

`--lift` (2 cm of ground clearance) keeps the switch to dynamic starting from
clean, non-coincident contacts.

The geometry half imports no `pxr` and no `isaacsim`; `--dump` exercises it
(load, solidify, fracture, reproject) and writes a GLB, no sim involved.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import trimesh
from scipy.spatial import ConvexHull, HalfspaceIntersection, cKDTree

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


# Interior (fracture-face) look. The folder holds texture maps only — no .mdl
# and no .usd — so the material is built here against the JPGs in place.
# Kit resolves `omniverse://` at render time; nothing is downloaded.
INTERIOR_TEXTURE = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                    "SEI-COA/StandaloneMaterials/OSB/osb_board_tjzjedni_2k/"
                    "OSB_Board_tjzjedni_2K_BaseColor.jpg")
INTERIOR_ROUGHNESS = INTERIOR_TEXTURE.replace("BaseColor", "Roughness")


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


def _mesh_soup(stage, root_path):
    """Every mesh under *root_prim* as one welded triangle soup, world space.

    Geometry only — no UVs. The appearance reference is the *pristine* source
    read before thickening; this is only the thing that gets cut.
    """
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(root_path) if isinstance(root_path, str) \
        else root_path
    verts, faces, off = [], [], 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        xf = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()))
        p = np.array(pts, dtype=np.float64) @ xf[:3, :3] + xf[3, :3]
        tris, _ = _triangulate(np.array(counts), np.array(idx))
        if not len(tris):
            continue
        faces.append(tris + off)
        verts.append(p)
        off += len(p)
    if not verts:
        return None
    soup = trimesh.Trimesh(np.vstack(verts), np.vstack(faces), process=False)
    # Welding is not optional: assets arrive with vertices split along every UV
    # seam, so without it the "same" wall is thousands of disjoint scraps and
    # nothing is closed (measured: 4.8% of components watertight unwelded,
    # 47% welded).
    soup.merge_vertices()
    return soup


def visible_faces(soup, spacing, n_views=14):
    """Mask of faces a camera outside the object can actually see.

    Sweeps parallel rays in from `n_views` directions and keeps the FIRST face
    each ray meets. That is what separates a wall's siding from the sheathing,
    paper and drywall stacked behind it: only the outermost sheet is ever hit.

    First-hit from outside, rather than "does a ray along the normal escape",
    because the escape test needs an epsilon to step over coincident layers and
    no epsilon works — too small and a roof's own paper blocks its shingles,
    too large and both survive. Nothing is coincident seen from far enough away.
    """
    seen = np.zeros(len(soup.faces), dtype=bool)
    lo, hi = soup.bounds
    centre = (lo + hi) * 0.5
    radius = float(np.linalg.norm(hi - lo)) * 0.5 + spacing

    i = np.arange(n_views) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n_views)
    theta = np.pi * (1.0 + 5.0 ** 0.5) * i
    dirs = np.column_stack([np.cos(theta) * np.sin(phi),
                            np.sin(theta) * np.sin(phi), np.cos(phi)])

    grid = np.arange(-radius, radius + spacing, spacing)
    gu, gv = np.meshgrid(grid, grid)
    gu, gv = gu.ravel(), gv.ravel()
    for d in dirs:
        tmp = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else \
            np.array([1.0, 0.0, 0.0])
        u = np.cross(d, tmp)
        u /= np.linalg.norm(u)
        v = np.cross(d, u)
        origins = centre - d * radius * 1.5 + gu[:, None] * u + gv[:, None] * v
        _, _, i_tri = soup.ray.intersects_location(
            origins, np.broadcast_to(d, origins.shape), multiple_hits=False)
        if len(i_tri):
            seen[i_tri] = True
    return seen


def _demanifold(mesh):
    """Drop faces sitting on edges shared by more than two faces.

    Art carries T-junctions — an internal web meeting a wall along an edge
    already used by two faces. The extrusion inherits them (8 such edges on one
    wall became 17 on its slab), and a non-manifold slab is refused by
    manifold3d outright, so the wall silently vanished from the scene.
    """
    m = mesh.copy()
    for _ in range(3):
        _, inv, cnt = np.unique(m.edges_sorted, axis=0,
                                return_inverse=True, return_counts=True)
        bad = np.nonzero(cnt > 2)[0]
        if not len(bad):
            break
        drop = np.unique(np.nonzero(np.isin(inv, bad))[0] // 3)
        keep = np.ones(len(m.faces), dtype=bool)
        keep[drop] = False
        m.update_faces(keep)
        m.remove_unreferenced_vertices()
    return m


def _close_sheet(comp, visible, thickness, centre):
    """Every way we know to turn one open component into closed slabs.

    No single strategy closes everything, so they are tried in order of how
    much of the component they preserve and the first that works wins:

    1. the component as it stands
    2. only its visible faces — first-hit faces are a single layer by
       construction, which extrudes cleanly when a two-sided shell will not
    3. de-manifolded and split, taking whichever pieces close

    A component that survives none of these is reported rather than dropped
    quietly; that is how a missing wall stayed missing for two rounds.
    """
    def ok(slab):
        return slab is not None and slab.is_volume and slab.volume > 1e-9

    slab = _sheet_slab(comp, thickness, centre)
    if ok(slab):
        return [slab]

    if visible is not None and visible.any() and not visible.all():
        sub = comp.submesh([np.nonzero(visible)[0]], append=False)
        if sub:
            sub = sub[0]
            sub.merge_vertices()
            slab = _sheet_slab(sub, thickness, centre)
            if ok(slab):
                return [slab]

    out = []
    for piece in _demanifold(comp).split(only_watertight=False):
        piece.merge_vertices()
        slab = _sheet_slab(piece, thickness, centre)
        if ok(slab):
            out.append(slab)
    if out:
        return out

    # Last resort, for the handful that resist everything: voxelise this one
    # component. Surface voxels alone already form a slab about a pitch thick,
    # so a sheet comes back closed no matter how its rim pinches or how many
    # non-manifold edges it carries. It resamples, which is what the rest of
    # this file exists to avoid — but it is applied to single components (5 of
    # ~1800 on the test house), and snapping pulls the result back onto the
    # original surface, so the cost is local and small. Losing a whole gable
    # wall to a rim that would not close is the alternative.
    try:
        # Pitch is tied to the COMPONENT'S SIZE, not to the extrusion
        # thickness. Using the thickness meant a 5 mm pitch on detail parts
        # whatever their extent, so a one-metre component became a 200^3 grid:
        # measured at 6.9 s per component, 241 s over the 35 that reach here,
        # which was 70% of the entire run. Capping the longest axis at 64 cells
        # makes it ~0.05 s. The slab can come out thicker than asked when the
        # cap binds, which is a fair trade for a last-resort path.
        pitch = max(thickness, float(comp.extents.max()) / 64.0)
        grid = comp.voxelized(pitch).fill()
        slab = grid.marching_cubes
        # `marching_cubes` returns the surface in VOXEL INDEX space and drops
        # the grid transform; without this the slab is hundreds of units wide
        # and nowhere near the object.
        slab.apply_transform(grid.transform)
        trimesh.repair.fix_normals(slab)
        if ok(slab):
            return [slab]
    except Exception as exc:                      # pragma: no cover - reporting
        print(f"[fracture] voxel fallback failed: {type(exc).__name__}: {exc}",
              flush=True)
    return []


def _sheet_slab(sheet, thickness, centre):
    """Extrude one open sheet inward into a closed slab, or None.

    Purely additive: the sheet's own triangles are reused vertex-for-vertex as
    the outer skin, so the visible surface is never moved. New geometry is the
    offset inner skin (wound backwards, or the slab renders inside-out) and a
    rim quad along every free edge.

    "Inward" is decided by an area-weighted vote of the sheet's own normals
    against the direction out of the building. That test is only reliable when
    the sheet is big enough to have a definite facing, which is exactly why
    small detail is excluded upstream rather than fixed here.
    """
    comp = sheet.copy()
    trimesh.repair.fix_winding(comp)
    fn = comp.face_normals * trimesh.triangles.area(comp.triangles)[:, None]
    sign = 1.0 if float((fn * (comp.triangles.mean(axis=1) - centre)).sum()) \
        >= 0.0 else -1.0

    # The sign is applied to the offset rather than by calling `invert()`, and
    # that is not a style choice. `invert()` negates the vertex normals only if
    # they are ALREADY CACHED; it leaves the face-normal cache stale either way.
    # Reading `face_normals` for the vote and `vertex_normals` after the invert
    # — exactly this order — recomputes the vertex normals from the stale face
    # normals and hands back the wrong direction. It sent 60% of big-sheet area
    # outward, so walls extruded through their own siding and the house
    # rendered as its own white inner skin.
    n_v = len(comp.vertices)
    inner = comp.vertices - comp.vertex_normals * (sign * thickness)

    lone = trimesh.grouping.group_rows(comp.edges_sorted, require_count=1)
    if not len(lone):
        return None
    be = comp.edges[lone]
    rim = np.vstack([
        np.column_stack([be[:, 0], be[:, 1], be[:, 1] + n_v]),
        np.column_stack([be[:, 0], be[:, 1] + n_v, be[:, 0] + n_v]),
    ])

    slab = trimesh.Trimesh(
        np.vstack([comp.vertices, inner]),
        np.vstack([comp.faces, comp.faces[:, ::-1] + n_v, rim]),
        process=False)
    slab.merge_vertices()
    trimesh.repair.fix_normals(slab)
    return slab


def thicken(stage, root_path, thickness, detail_area=0.25,
            detail_thickness=0.005, quiet=False):
    """Solidify by extruding only what needs it. Returns closed parts.

    Three cases, and only one of them touches the geometry:

    * **already solid — left alone.** A third of this house by area is closed
      already; the whole roof is. Extruding a closed solid does not thicken it,
      it wrecks it (the test cone went 23.7 -> 46.2 m^3 when it was thickened
      needlessly). These pass through vertex-for-vertex.
    * **big sheets — extruded inward.** Walls and facades. Their facing is
      unambiguous, so the direction is safe to decide.
    * **small detail — barely extruded, or dropped.** Window trim, shutters,
      sills. Individually tiny, collectively 83% of the sheet COUNT but only
      4.8% of sheet AREA, and every one of them an independent chance to guess
      a direction wrong. They were the entire source of the white shards: a
      sheet lying tangential to the direction out of the building has no
      definite facing, so the vote that works for a wall is a coin toss for a
      sill. Below `detail_area` they get `detail_thickness` (millimetres, so a
      wrong guess is invisible) or nothing at all at 0.

    Dropping detail costs less than it sounds: appearance is reprojected from
    the *original* surface, which still carries the trim, so a dropped sill
    still paints onto the wall behind it. Only its relief is lost.
    """
    soup = _mesh_soup(stage, root_path)
    if soup is None:
        return []
    centre = soup.bounds.mean(axis=0)

    # Which sheets are even on the outside. A wall assembly stacks siding,
    # paper, sheathing and drywall; thickening all four makes them fight for
    # the outermost surface, and an inner layer extruded 4 cm wins about as
    # often as not, which is why the walls came back white instead of blue.
    # Buried sheets are invisible by definition, so dropping them costs nothing.
    seen = visible_faces(soup, max(0.5 * thickness, 5e-3))

    # Components as explicit FACE INDEX groups, so visibility can be read off
    # per component. Walking `split()` and slicing `seen` with a running offset
    # looks equivalent and is not: `split` gathers each component's faces by
    # index and does not hand them back as contiguous runs in the original
    # order, so the offsets drift and every component is scored with some other
    # component's visibility. It is what buried an exterior wall at "0%
    # visible" and left a hole straight into the house.
    groups = trimesh.graph.connected_components(
        soup.face_adjacency, nodes=np.arange(len(soup.faces)), min_len=1)

    parts = []
    kept = big = small = dropped = failed = buried = 0
    lost = 0.0
    for idx in groups:
        comp = soup.submesh([idx], append=False)[0]
        # FRACTION visible, not "any face visible". Lap siding is modelled as
        # dozens of small boards, and at any finite ray spacing a few rays slip
        # between them and strike the sheathing behind. On "any" that counted
        # the sheathing as exterior, so it was extruded the full thickness
        # while the siding boards — small enough to be detail — got millimetres,
        # and the sheathing came out through the siding.
        vis = float(seen[idx].mean()) > 0.15
        comp.merge_vertices()
        trimesh.repair.fix_normals(comp)
        if comp.is_volume and comp.volume > 1e-9:
            parts.append(comp)                       # already solid: no-op
            kept += 1
            continue
        # A buried sheet is thinned, never dropped. Dropping was tempting —
        # it is invisible, so who cares — but "buried" is decided by a ray
        # sample, and a wall that happens to sit behind its own siding boards
        # or a porch roof scores under the threshold and disappears, leaving a
        # hole you can see straight through into the house. Thinning gets the
        # same benefit (a 5 mm sheet cannot punch out through the layer in
        # front of it) with no way to lose a wall.
        t = thickness if (vis and comp.area >= detail_area) else detail_thickness
        if not vis:
            buried += 1
        if t <= 0.0:
            dropped += 1
            continue
        slabs = _close_sheet(comp, seen[idx], t, centre)
        if slabs:
            parts.extend(slabs)
            big += comp.area >= detail_area
            small += comp.area < detail_area
        else:
            failed += 1
            lost += comp.area
    if not quiet:
        vol = sum(p.volume for p in parts)
        print(f"[fracture] solid {kept} kept as-is | {big} sheets extruded "
              f"{thickness*100:.0f} cm | {small} details at "
              f"{detail_thickness*1000:.0f} mm | {buried} buried, "
              f"{dropped} dropped, {failed} failed ({lost:.1f} m^2 lost) "
              f"-> {len(parts)} parts, {vol:.1f} m^3", flush=True)
    return parts


def fill_cavities(parts, quiet=False):
    """Fuse the parts and discard interior boundaries, leaving a solid mass.

    A thickened building is *hollow* — chunks cut from it are slab fragments.
    Filling would make them solid blocks instead. The intent is that the fused
    surface splits into disconnected sheets, one bounding the outside and one
    bounding each sealed void, so keeping the outer sheet fills the rooms.

    THIS ONLY WORKS ON A CLEANLY NESTED SOLID, WHICH A THICKENED SHELL ASSET IS
    NOT. Layers of a wall assembly overlap once extruded, so the union
    self-intersects: on the test house it fragments into 29316 sheets and the
    largest is an inverted volume. Every failure mode here falls back to hollow
    rather than returning a broken solid, because a broken one silently yields
    zero chunks. A watertight asset that genuinely encloses voids will fill.
    """
    if len(parts) < 2:
        return parts
    try:
        fused = trimesh.boolean.union(parts)
    except Exception as exc:
        if not quiet:
            print(f"[fracture] fill: union failed ({exc}); staying hollow",
                  flush=True)
        return parts

    sheets = fused.split(only_watertight=False)
    if len(sheets) < 2:
        if not quiet:
            print("[fracture] fill: nothing sealed to fill; staying hollow",
                  flush=True)
        return parts

    outer = max(sheets, key=lambda c: float(np.prod(c.extents)))
    trimesh.repair.fix_normals(outer)
    if not (outer.is_volume and outer.volume > 0):
        if not quiet:
            print(f"[fracture] fill: fused surface is not cleanly nested "
                  f"({len(sheets)} sheets, outer volume {outer.volume:.1f} "
                  f"m^3); staying hollow", flush=True)
        return parts
    if not quiet:
        print(f"[fracture] filled: {len(sheets)} sheets -> outer only, "
              f"{outer.volume:.1f} m^3", flush=True)
    return [outer]


# Fraction of surface area that must already be closed for an asset to count
# as a pile of solids rather than an open shell. The library splits cleanly
# either side of this: the multi-solid assets measure 95.0-98.9%, the next one
# down is 79.7%, and the buildings are 20-40%.
MULTI_SOLID_AREA = 0.90


def close_directly(mesh: trimesh.Trimesh):
    """`(parts, tier)` if the mesh already bounds a volume, else None.

    The tiers that need no help. All of them leave the outer surface exactly as
    authored — nothing here resamples, extrudes or invents anything.
    """
    if mesh.is_watertight and mesh.volume > 0:
        return [mesh], "watertight"

    patched = mesh.copy()
    patched.merge_vertices()
    trimesh.repair.fill_holes(patched)
    trimesh.repair.fix_normals(patched)
    if patched.is_watertight and patched.volume > 0:
        return [patched], "hole-filled"

    # SEVERAL CLOSED SOLIDS THAT SIMPLY ARE NOT ONE CONNECTED SURFACE.
    # `is_watertight` asks "is this one closed surface?", which is stricter than
    # what fracturing needs — "does this bound a volume?" A bag of separate
    # closed solids does, and answering the strict question sent 4 of the 29
    # assets that reach here off to be thickened or voxelised when they were
    # already solid, throwing away the exact outer surface they arrived with.
    # Cutting handles a list of parts natively, so there is nothing to fuse.
    welded = mesh.copy()
    welded.merge_vertices()
    parts = welded.split(only_watertight=False)
    if len(parts) > 1:
        area = sum(p.area for p in parts)
        shut = [p for p in parts
                if p.is_watertight and abs(p.volume) > 1e-12]
        if shut and sum(p.area for p in shut) >= MULTI_SOLID_AREA * max(area, 1e-12):
            solid = []
            for p in shut:
                trimesh.repair.fix_normals(p)
                if p.is_volume and p.volume > 1e-12:
                    solid.append(p)
            if solid:
                kept = sum(p.area for p in solid)
                return solid, ("multi-solid, %.1f%% of area dropped as open "
                               "scrap" % (100 * (1 - kept / area)))
    return None


def solidify(mesh: trimesh.Trimesh, voxel_res: int = 160, quiet=False,
             force_voxel: bool = False):
    """A closed volume to cut, by the cheapest tier that works.

    See the module docstring: watertight passes through untouched, small holes
    are filled, and open shells are voxelised and flood-filled. Returns
    `(solid, tier)`.
    """
    if not force_voxel:
        direct = close_directly(mesh)
        if direct is not None:
            return direct

    if not quiet:
        print(f"[fracture] open shell — voxelising at {voxel_res} to build an "
              f"interior", flush=True)
    pitch = mesh.extents.max() / float(voxel_res)
    grid = mesh.voxelized(pitch).fill()
    solid = grid.marching_cubes
    # `marching_cubes` hands back the surface in VOXEL INDEX space and drops
    # the grid's transform, so without this the solid is `voxel_res` units
    # wide instead of its real size, in the wrong place. It fails loudly on
    # volume (a 8 m house measured 1.05e6 m^3) and silently on everything else.
    solid.apply_transform(grid.transform)
    trimesh.repair.fix_normals(solid)
    trimesh.repair.fix_normals(solid)
    return [solid], "voxelised"


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


# ---------------------------------------------------------------------------
# Source loading — USD, and Kit for `omniverse://`
# ---------------------------------------------------------------------------


class Source:
    """Triangles of the source asset, in world space, with appearance."""

    def __init__(self, vertices, faces, tri_uv, tri_mat, mat_paths):
        self.vertices, self.faces = vertices, faces
        self.tri_uv, self.tri_mat, self.mat_paths = tri_uv, tri_mat, mat_paths
        self.parts = None   # closed solid parts, set by `_load` after thickening


def resolve_asset(spec: str, target_size: float) -> str:
    """An asset-set asset string to something `Usd.Stage.Open` accepts.

    Delegates to the generator's own resolver so `objaverse://` etc. mean here
    exactly what they mean in a config, and triggers the same download-and-
    convert path the scene generator relies on.
    """
    import re

    from scene_generator import _expand_scheme

    uid = re.match(r"^objaverse://([0-9a-fA-F]{32})$", spec.strip())
    if uid:
        import objaverse_assets
        objaverse_assets.ensure(uid.group(1), target_size_m=target_size)
    return _expand_scheme(spec) or spec


def _triangulate(counts, indices):
    """Fan-triangulate USD face arrays; returns triangles and their source face."""
    tris, owner, base = [], [], 0
    for f, c in enumerate(counts):
        for j in range(1, c - 1):
            tris.append((indices[base], indices[base + j], indices[base + j + 1]))
            owner.append((base, base + j, base + j + 1))
        base += c
    return np.array(tris, dtype=np.int64), np.array(owner, dtype=np.int64)


def load_source(stage, root_path: str, asset: str, target_size: float,
                up_axis: str = "z", quiet=False) -> Source:
    """Reference *asset* under *root_path* (invisible) and extract its triangles.

    Referencing rather than copying is what preserves appearance: the asset's
    `Material` prims land on the stage with their shader networks and texture
    paths intact, and the chunks simply bind to them.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    src = UsdGeom.Xform.Define(stage, root_path)
    src.GetPrim().GetReferences().AddReference(asset)

    if up_axis.lower() == "y":
        # Y-up asset onto a Z-up stage.
        src.AddRotateXOp().Set(90.0)

    verts, faces, uvs, mats, mat_paths = [], [], [], [], []
    mat_index = {}
    for prim in Usd.PrimRange(src.GetPrim()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue

        xf = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()))
        p = np.array(pts, dtype=np.float64)
        p = p @ xf[:3, :3] + xf[3, :3]

        tris, corner = _triangulate(np.array(counts), np.array(idx))
        if not len(tris):
            continue

        st = _read_st(prim, len(p), np.array(idx))
        # `corner` indexes face-varying slots; for vertex/constant `st` the
        # reader already expanded to one value per slot, so this is uniform.
        tri_uv = st[corner] if st is not None else np.zeros((len(tris), 3, 2))

        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        key = str(bound.GetPath()) if bound else ""
        if key not in mat_index:
            mat_index[key] = len(mat_paths)
            mat_paths.append(Sdf.Path(key) if key else None)

        faces.append(tris + sum(len(v) for v in verts))
        verts.append(p)
        uvs.append(tri_uv)
        mats.append(np.full(len(tris), mat_index[key]))

    if not verts:
        raise RuntimeError(f"no meshes found in {asset}")

    # Measured off the extracted geometry rather than a UsdGeom.BBoxCache:
    # the cache reported an empty range for a reference added moments earlier
    # and every asset silently came through at scale 1.0. The vertices are
    # already in hand and cannot disagree with what actually gets fractured.
    points = np.vstack(verts)
    scale = 1.0
    plan = np.ptp(points, axis=0)[:2].max()
    if target_size > 0 and plan > 0:
        scale = target_size / float(plan)
        points = points * scale

    UsdGeom.Imageable(src).MakeInvisible()
    if not quiet:
        print(f"[fracture] {asset}: {len(verts)} meshes, "
              f"{sum(len(f) for f in faces)} tris, {len(mat_paths)} materials, "
              f"scale x{scale:.4g}", flush=True)
    return Source(points, np.vstack(faces), np.vstack(uvs),
                  np.concatenate(mats), mat_paths)


def _read_st(prim, n_points, indices):
    """The mesh's texture coordinates, expanded to one per face-varying slot."""
    from pxr import UsdGeom

    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        if "TexCoord2f" not in str(pv.GetTypeName()):
            continue
        vals = pv.Get()
        if not vals:
            continue
        vals = np.array(vals, dtype=np.float64)
        interp = pv.GetInterpolation()
        pv_idx = pv.GetIndices() if pv.IsIndexed() else None
        if pv_idx:
            vals = vals[np.array(pv_idx)]
        if interp == UsdGeom.Tokens.faceVarying:
            return vals
        if interp == UsdGeom.Tokens.vertex or interp == UsdGeom.Tokens.varying:
            return vals[indices] if len(vals) == n_points else None
    return None


def cylinder_source(radius=2.0, height=6.0, sections=64) -> Source:
    """The default subject: a plain closed cylinder, no textures."""
    m = trimesh.creation.cylinder(radius=radius, height=height,
                                  sections=sections)
    m.apply_translation((0.0, 0.0, height * 0.5))
    return Source(np.array(m.vertices), np.array(m.faces),
                  np.zeros((len(m.faces), 3, 2)),
                  np.zeros(len(m.faces), dtype=np.int64), [None])


# ---------------------------------------------------------------------------
# USD authoring
# ---------------------------------------------------------------------------


def author_chunks(stage, root_path, chunks, mat_paths, density=2400.0,
                  interior_texture=INTERIOR_TEXTURE):
    """One rigid-body Mesh prim per chunk, split into per-material subsets."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    UsdGeom.Xform.Define(stage, root_path)
    if interior_texture:
        inner_mat = _textured_material(
            stage, root_path + "/Interior", interior_texture,
            interior_texture.replace("BaseColor", "Roughness"))
    else:
        inner_mat = _flat_material(stage, root_path + "/Interior",
                                   (0.93, 0.93, 0.93))
    default_mat = _flat_material(stage, root_path + "/Concrete", (0.62, 0.60, 0.57))

    for i, c in enumerate(chunks):
        path = f"{root_path}/chunk_{i:03d}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        faces = c["faces"]
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in c["points"]])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr(
            [Gf.Vec3f(*c["points"].min(0)), Gf.Vec3f(*c["points"].max(0))])
        UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(*c["centroid"]))

        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([Gf.Vec2f(*v) for v in c["uv"].reshape(-1, 2)])

        _bind_subsets(stage, mesh, c["face_mat"], mat_paths, inner_mat,
                      default_mat)

        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        # Chunks of a non-convex object are themselves non-convex, so the
        # collider cannot be a single hull the way it could for the cylinder.
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()) \
            .CreateApproximationAttr().Set(UsdPhysics.Tokens.convexDecomposition)
        body = UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
        body.CreateKinematicEnabledAttr(True)   # held — see `set_kinematic`
        UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateDensityAttr(density)


def _bind_subsets(stage, mesh, face_mat, mat_paths, inner_mat, default_mat):
    """Bind cut faces to the interior material and the rest to their source."""
    from pxr import UsdGeom, UsdShade

    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    for value in np.unique(face_mat):
        idx = np.nonzero(face_mat == value)[0]
        if value < 0:
            target = inner_mat
            name = "interior"
        else:
            src = mat_paths[value] if value < len(mat_paths) else None
            target = UsdShade.Material.Get(stage, src) if src else None
            if not (target and target.GetPrim().IsValid()):
                target = default_mat
            name = f"mat_{value:02d}"
        subset = UsdGeom.Subset.CreateGeomSubset(
            mesh, name, UsdGeom.Tokens.face, idx.tolist(),
            familyName="materialBind")
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(target)


def add_lighting(stage, path="/World/Lights"):
    """A dome plus a key light.

    `World` brings no lights of its own, so an unlit stage renders the object
    as a black silhouette against the ground grid — which reads as broken
    geometry rather than as missing lighting.
    """
    from pxr import Gf, UsdGeom, UsdLux

    UsdGeom.Xform.Define(stage, path)
    dome = UsdLux.DomeLight.Define(stage, path + "/Dome")
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))

    key = UsdLux.DistantLight.Define(stage, path + "/Key")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.5)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))
    return dome


def _textured_material(stage, path, colour_url, rough_url=""):
    """UsdPreviewSurface reading `st`, for the freshly-cut interior faces."""
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")

    reader = UsdShade.Shader.Define(stage, path + "/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv = reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def sample(name, url, out_type, channel):
        tex = UsdShade.Shader.Define(stage, f"{path}/{name}")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(url)
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv)
        for wrap in ("wrapS", "wrapT"):
            tex.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("repeat")
        return tex.CreateOutput(channel, out_type)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f) \
        .ConnectToSource(sample("diffuse", colour_url,
                                Sdf.ValueTypeNames.Float3, "rgb"))
    if rough_url:
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float) \
            .ConnectToSource(sample("rough", rough_url,
                                    Sdf.ValueTypeNames.Float, "r"))
    else:
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def _flat_material(stage, path, color):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def set_kinematic(stage, root_path, n, on: bool):
    """Flip every chunk between kinematic (held) and dynamic (falling).

    PhysX rebuilds the body on the transition, which is why the caller steps
    once before assigning velocities.
    """
    from pxr import UsdPhysics

    for i in range(n):
        prim = stage.GetPrimAtPath(f"{root_path}/chunk_{i:03d}")
        if prim:
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(on)


def blast_velocities(positions, centre, strength, scale, rng):
    """Radial linear velocity with inverse-square falloff, plus tumble."""
    d = positions - centre
    dist = np.linalg.norm(d, axis=1, keepdims=True)
    u = d / np.maximum(dist, 1e-6)
    lin = u * strength / (1.0 + (dist / scale) ** 2)
    lin[:, 2] += 0.35 * strength / (1.0 + (dist[:, 0] / scale) ** 2)
    ang = rng.normal(0.0, 8.0, positions.shape)
    return np.hstack([lin, ang])


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--asset", default="",
                   help="objaverse://<uid>, omniverse://…, airstack://…, or a "
                        "path. Omit for the default cylinder.")
    p.add_argument("--target-size", type=float, default=8.0,
                   help="metres on the largest plan dimension (0 = as authored)")
    p.add_argument("--up-axis", choices=("z", "y"), default="z",
                   help="up axis of the source asset")
    p.add_argument("--thickness", type=float, default=0.0,
                   help="metres to extrude open shells inward when solidifying "
                        "(0 = 1.5%% of the object's largest extent)")
    p.add_argument("--interior-texture", default=INTERIOR_TEXTURE,
                   help="texture for freshly-cut faces ('' = plain white)")
    p.add_argument("--interior-tile", type=float, default=0.5,
                   help="metres per texture repeat on cut faces")
    p.add_argument("--detail-area", type=float, default=0.25,
                   help="m^2 below which a sheet counts as small detail")
    p.add_argument("--detail-thickness", type=float, default=0.005,
                   help="metres to extrude small detail (0 = drop it)")
    p.add_argument("--fill", action="store_true",
                   help="fill sealed interior voids, so chunks are solid blocks "
                        "rather than slab fragments")
    p.add_argument("--force-voxel", action="store_true",
                   help="solidify by voxelising even when the mesh is already "
                        "closed. Only useful for seeing what tier 3 costs: run "
                        "a watertight asset with and without it.")
    p.add_argument("--voxel-res", type=int, default=192,
                   help="grid resolution when an open shell must be solidified. "
                        "Also the face-count knob: marching cubes scales with "
                        "it, and every chunk and collider inherits that.")
    p.add_argument("--radius", type=float, default=2.0, help="default cylinder")
    p.add_argument("--height", type=float, default=6.0, help="default cylinder")
    p.add_argument("--chunks", type=int, default=80)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--relax", type=int, default=2)
    p.add_argument("--gap", type=float, default=0.004,
                   help="metres of clearance between neighbouring chunks")
    p.add_argument("--lift", type=float, default=0.02,
                   help="metres of clearance above the ground while held")
    p.add_argument("--strength", type=float, default=18.0,
                   help="peak radial speed at the blast centre, m/s")
    p.add_argument("--dump", default="",
                   help="write the fractured chunks to this GLB and exit, "
                        "without starting Isaac Sim")
    p.add_argument("--headless", action="store_true",
                   help="no window; blasts once and exits (for smoke-testing)")
    p.add_argument("--shot", default="",
                   help="with --headless, write <shot>_held.png and "
                        "<shot>_blast.png from the viewport")
    return p.parse_args(argv)


def _capture(path, frames=60):
    """One rendered viewport frame, once the RTX render has settled."""
    import omni.kit.app
    import omni.kit.viewport.utility as vu

    vp = vu.get_active_viewport()
    for _ in range(frames):
        omni.kit.app.get_app().update()
    vu.capture_viewport_to_file(vp, file_path=path)
    for _ in range(frames):
        omni.kit.app.get_app().update()
    print(f"[fracture] wrote {path}", flush=True)


def _load(stage, args, quiet=False):
    """The source asset, placed with its base at `--lift`."""
    if args.asset:
        asset = resolve_asset(args.asset, args.target_size)
        source = load_source(stage, "/World/_source", asset, args.target_size,
                             args.up_axis, quiet)
        # Order matters: the appearance reference is read from the PRISTINE
        # asset above, because thickening adds faces without extending the `st`
        # primvar (measured: 2328 UV values against 7660 face-vertex slots
        # afterwards). Keeping the two separate means the cut solid can gain
        # geometry while the texture source stays exact.
        # Thicken only what actually needs it. An asset that already bounds a
        # volume is left alone — extruding it would hollow out a solid that was
        # never open (the test cone went 23.7 -> 46.2 m^3 that way).
        surface = trimesh.Trimesh(source.vertices, source.faces, process=False)
        if not args.force_voxel and close_directly(surface) is None:
            thickness = args.thickness or 0.015 * float(
                np.ptp(source.vertices, axis=0).max())
            source.parts = thicken(stage, "/World/_source", thickness,
                                   args.detail_area, args.detail_thickness,
                                   quiet)
            if args.fill and source.parts:
                source.parts = fill_cavities(source.parts, quiet)
    else:
        source = cylinder_source(args.radius, args.height)
    return source


def _dump(args):
    """Geometry-only path: no Kit, no physics — just prove the cut."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    source = _load(stage, args)
    chunks = build_chunks(source, args.chunks, args.seed, args.gap,
                          args.relax, args.voxel_res, args.lift,
                          force_voxel=args.force_voxel,
                          interior_tile=args.interior_tile)

    scene = trimesh.Scene()
    rng = np.random.default_rng(args.seed)
    for i, c in enumerate(chunks):
        m = trimesh.Trimesh(c["points"] + c["centroid"], c["faces"],
                            process=False)
        m.visual.face_colors = np.tile(
            np.append((rng.random(3) * 155 + 100).astype(np.uint8), 255),
            (len(m.faces), 1))
        scene.add_geometry(m, node_name=f"chunk_{i:03d}")
    scene.export(args.dump)

    outer = sum(int((c["face_mat"] >= 0).sum()) for c in chunks)
    total = sum(len(c["face_mat"]) for c in chunks)
    print(f"[fracture] wrote {args.dump} — {len(chunks)} chunks, "
          f"{outer}/{total} faces textured from the source", flush=True)
    return 0


def main(argv=None):
    args = parse_args(argv)
    if args.dump:
        return _dump(args)

    from isaacsim import SimulationApp
    simulation_app = SimulationApp(launch_config={"headless": args.headless})

    import carb
    import omni.appwindow
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.viewports import set_camera_view

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_lighting(world.stage)

    source = _load(world.stage, args)
    chunks = build_chunks(source, args.chunks, args.seed, args.gap,
                          args.relax, args.voxel_res, args.lift,
                          force_voxel=args.force_voxel,
                          interior_tile=args.interior_tile)
    if not chunks:
        print("[fracture] nothing to fracture", flush=True)
        simulation_app.close()
        return 1
    author_chunks(world.stage, "/World/fractured", chunks, source.mat_paths,
                  interior_texture=args.interior_texture)

    view = RigidPrim("/World/fractured/chunk_.*", name="chunks")
    world.scene.add(view)
    world.reset()

    rest = np.array([c["centroid"] for c in chunks])
    _, rest_q = view.get_world_poses()
    span = float(np.ptp(source.vertices, axis=0).max())
    centre = rest.mean(axis=0)
    rng = np.random.default_rng(args.seed + 1)
    state = {"armed": True}

    def re_form():
        view.set_velocities(np.zeros((len(chunks), 6)))
        view.set_world_poses(rest, rest_q)
        set_kinematic(world.stage, "/World/fractured", len(chunks), True)
        state["armed"] = True
        print("[fracture] re-formed", flush=True)

    def detonate():
        if not state["armed"]:
            return
        state["armed"] = False
        pos, _ = view.get_world_poses()
        set_kinematic(world.stage, "/World/fractured", len(chunks), False)
        # PhysX recreates each body as dynamic on this step; velocities written
        # before it has done so land on bodies about to be replaced.
        world.step(render=False)
        view.set_velocities(blast_velocities(
            np.asarray(pos), centre, args.strength, span * 0.25, rng))
        print("[fracture] blast", flush=True)

    set_camera_view([span * 1.8, -span * 1.8, span], centre.tolist())

    if args.headless:
        for _ in range(120):
            world.step(render=False)
        held, _ = view.get_world_poses()
        drift = np.abs(np.asarray(held) - rest).max()
        if args.shot:
            _capture(f"{args.shot}_held.png")
        detonate()
        for _ in range(120):
            world.step(render=False)
        if args.shot:
            _capture(f"{args.shot}_blast.png")
        pos, _ = view.get_world_poses()
        spread = np.linalg.norm(np.asarray(pos)[:, :2], axis=1).max()
        re_form()
        for _ in range(120):
            world.step(render=False)
        back, _ = view.get_world_poses()
        err = np.abs(np.asarray(back) - rest).max()
        print(f"[fracture] headless ok — drift while held {drift * 1e3:.2f} mm, "
              f"radial spread after blast {spread:.1f} m (object spans "
              f"{span:.1f} m), re-form error {err * 1e3:.2f} mm", flush=True)
        simulation_app.close()
        return 0

    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    _input = carb.input.acquire_input_interface()

    def on_key(event, *_a):
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        if event.input == carb.input.KeyboardInput.SPACE:
            detonate()
        elif event.input == carb.input.KeyboardInput.R:
            re_form()
        elif event.input == carb.input.KeyboardInput.ESCAPE:
            state["quit"] = True
        return True

    # Held for the process lifetime; dropping it unsubscribes.
    sub = _input.subscribe_to_keyboard_events(keyboard, on_key)  # noqa: F841

    print("[fracture] SPACE = blast, R = re-form, ESC = quit", flush=True)
    while simulation_app.is_running() and not state.get("quit"):
        world.step(render=True)

    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
