"""solids — closed volumes out of open art.

Fracturing needs something that bounds a volume, and assets overwhelmingly do
not: they are sheets with no thickness and no interior. The tiers here get from
one to the other, cheapest first, and all but the last leave the outer surface
exactly as authored. See the module docstring of ``earthquake`` for how a
damage model drives them."""

from __future__ import annotations

import numpy as np
import trimesh

from .source import _mesh_soup

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


def tier_name(tier: str) -> str:
    """The bare tier name from a `close_directly`/`solidify` tier string.

    The multi-solid tier carries a diagnostic suffix meant for a print
    statement (`"multi-solid, 5.0% of area dropped as open scrap"`), not for
    comparison — this is the one place that knows the format, so a caller
    that only wants the short name (`"multi-solid"`) does not have to know or
    re-derive how to strip it.
    """
    return tier.split(",", 1)[0]


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
    return [solid], "voxelised"
