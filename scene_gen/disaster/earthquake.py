"""earthquake — shake one building down.

Four stages, driven by a single ``severity`` in [0, 1] and by the structural
material. Everything is seeded, so the same arguments give the same ruin.

1. **Failure regions.** A ground-biased background from severity itself, plus a
   handful of local blobs. The background matters: with only blobs, most of a
   building is far from all of them and severity 0.6 detached nothing at all.
   ``soft_story`` adds a full storey-height band at the base — the one
   structural configuration worth special-casing, because an open ground floor
   has nothing to resist the shear with.

2. **Cracks propagate.** Two rules. Anything above a failure fails too, which
   is a running maximum up each column of the damage grid, decayed per metre so
   a small failure does not take out ten storeys; severity buys reach. And
   cracks favour weak points, which are the openings — a real earthquake crack
   starts at a window or door corner, and those are already in the mesh as its
   boundary loops, so nothing needs annotating.

3. **Settle.** Chunks over the material's threshold detach; the rest hold the
   building up. The caller runs PhysX — see ``tools/earthquake_damage.py``.

4. **Detail and rubble.** Handled by ``authoring``.

MATERIALS
---------
``MATERIALS`` is the table that becomes a tag in the asset-pack YAML, so it is
kept in one place and keyed by name. It sets how finely a structure shatters,
how readily a piece lets go, and what its rubble looks like. Several materials
are assigned in HEIGHT BANDS in the order given, so ``("brick", "wood")`` is a
masonry ground floor under a timber upper rather than a speckle.
"""

from __future__ import annotations

import numpy as np
import trimesh
from scipy.spatial import cKDTree

from . import fracture, solids
from .source import load_source, resolve_asset


# ---------------------------------------------------------------------------
# Materials — the future asset-pack tag, in one table
# ---------------------------------------------------------------------------


class Material:
    """How one structural material breaks, and what it leaves behind.

    `shatter` scales the number of Voronoi seeds, so it sets fragment size.
    `detach` is the damage above which a chunk lets go — brittle materials fail
    suddenly at low demand, ductile ones deform and hang on. `block` is the
    rubble unit as (size in metres, aspect ratio): a brick fragment is a lump,
    a timber is a plank, a steel member is a beam.
    """

    def __init__(self, name, colour, shatter, detach, block):
        self.name = name
        self.colour = np.array(colour, dtype=np.float64)
        self.shatter = shatter
        self.detach = detach
        self.block = block


MATERIALS = {
    # brittle, low shear capacity, fails into many small pieces — the classic
    # unreinforced-masonry earthquake ruin
    "brick":    Material("brick",    (0.52, 0.24, 0.18), 2.2, 0.30, (0.18, 1.0)),
    # spalls into chunky fragments, holds longer than masonry
    "concrete": Material("concrete", (0.62, 0.61, 0.58), 1.0, 0.45, (0.34, 1.3)),
    # racks and splinters along its length rather than crumbling
    "wood":     Material("wood",     (0.56, 0.40, 0.24), 0.7, 0.55, (0.28, 3.5)),
    # ductile: bends, connections go before members do, very few fragments
    "steel":    Material("steel",    (0.44, 0.46, 0.50), 0.35, 0.75, (0.22, 6.0)),
}


# ---------------------------------------------------------------------------
# Stage 1 and 2 — where it fails, and how that spreads
# ---------------------------------------------------------------------------


class DamageField:
    """Scalar damage in [0, 1] over the building, on a coarse grid.

    Coarse on purpose. This decides which region of a building failed, and a
    grid fine enough to resolve a window frame would be answering a question
    nobody asked — the fracture itself carries the detail.
    """

    def __init__(self, lo, hi, severity, soft_story, rng, res=44):
        self.lo = np.asarray(lo, dtype=np.float64)
        self.hi = np.asarray(hi, dtype=np.float64)
        self.size = np.maximum(self.hi - self.lo, 1e-6)
        self.res = res
        grid = np.zeros((res, res, res))

        height = float(self.size[2])
        span = float(np.linalg.norm(self.size))
        axes = [np.linspace(self.lo[k], self.hi[k], res) for k in range(3)]
        gx, gy, gz = np.meshgrid(*axes, indexing="ij")
        pts = np.stack([gx, gy, gz], axis=-1)

        # --- what the shaking does everywhere ------------------------------
        # Severity has to bite on its own, before any region is placed. With
        # only a handful of localised blobs, most of a building is far from all
        # of them and severity 0.6 detached nothing at all; the blobs are
        # variation on top of a background, not the whole story. Ground-biased,
        # because that is where an earthquake feeds the load in.
        t = np.clip((gz - self.lo[2]) / max(height, 1e-9), 0.0, 1.0)
        grid = np.maximum(grid, 0.9 * severity * (1.0 - t) ** 1.5)

        # --- failure regions, biased to the ground -------------------------
        n = int(round(1 + 7 * severity))
        self.regions = []
        for _ in range(n):
            # u**2.6 piles the draws near 0, i.e. near the base. A few land
            # higher up, which is what gives the intermediate failures.
            u = rng.random() ** 2.6
            centre = np.array([
                rng.uniform(self.lo[0], self.hi[0]),
                rng.uniform(self.lo[1], self.hi[1]),
                self.lo[2] + u * height,
            ])
            # Radius is scaled to the building HEIGHT, not its diagonal. On a
            # low wide house the diagonal is ~11 m, so a "0.3 x span" blob was
            # 3.5 m across and five of them summed to cover the whole model —
            # severity 0.6 flattened it completely. Height is the dimension an
            # earthquake actually stratifies damage along.
            radius = (0.15 + 0.25 * severity) * height * rng.uniform(0.7, 1.3)
            strength = (0.45 + 0.55 * severity) * rng.uniform(0.8, 1.0)
            self.regions.append((centre, radius, strength))
            d2 = ((pts - centre) ** 2).sum(axis=-1)
            # MAXIMUM, not sum: overlapping regions describe the same failure
            # seen twice, and adding them drives the field to 1 everywhere.
            grid = np.maximum(grid, strength * np.exp(-d2 / max(radius ** 2, 1e-9)))

        # --- soft storey ---------------------------------------------------
        # The one configuration worth special-casing. An open ground floor has
        # no wall to take the shear, so the whole storey racks and goes at once
        # — not a blob but a full-footprint band.
        self.storey = min(3.2, max(0.15 * height, 0.6))
        if soft_story:
            band = gz <= self.lo[2] + self.storey
            # Scaled by severity like everything else. A flat 0.85 meant a
            # soft storey flattened the building at severity 0.2 as readily as
            # at 1.0, which made the dial meaningless whenever the flag was set.
            grid = np.where(band, np.maximum(grid, 0.45 + 0.55 * severity), grid)

        grid = np.clip(grid, 0.0, 1.0)

        # --- propagate upward ----------------------------------------------
        # "When a region low down fails, the structure above it fails too."
        # A running maximum up each column says exactly that. The decay keeps
        # it honest: one failed bay does not level everything over it, but at
        # high severity the decay is small and collapse reaches the roof.
        dz = height / max(res - 1, 1)
        decay = (0.45 - 0.40 * severity) * dz
        for k in range(1, res):
            grid[:, :, k] = np.maximum(grid[:, :, k], grid[:, :, k - 1] - decay)
        self.grid = np.clip(grid, 0.0, 1.0)

    def at(self, points):
        """Damage at world points, nearest-cell."""
        p = np.atleast_2d(np.asarray(points, dtype=np.float64))
        idx = ((p - self.lo) / self.size * (self.res - 1)).round().astype(int)
        idx = np.clip(idx, 0, self.res - 1)
        return self.grid[idx[:, 0], idx[:, 1], idx[:, 2]]


def opening_tree(vertices, faces):
    """KD-tree over the free edges of the source surface — i.e. its openings.

    Window and door reveals are boundary loops in the mesh, so the weak points
    an earthquake crack actually starts from are already in the geometry and
    need no annotation. Returns None if the surface has no free edges.
    """
    surf = trimesh.Trimesh(vertices, faces, process=False)
    lone = trimesh.grouping.group_rows(surf.edges_sorted, require_count=1)
    if not len(lone):
        return None
    e = surf.edges_sorted[lone]
    return cKDTree(0.5 * (surf.vertices[e[:, 0]] + surf.vertices[e[:, 1]]))


def _world_box(chunk):
    p = chunk["points"] + chunk["centroid"]
    return np.array([p.min(axis=0), p.max(axis=0)])


def _weld_groups(chunks, idx, tol):
    """Group surviving chunks that still touch, by union-find on their boxes."""
    boxes = [_world_box(chunks[i]) for i in idx]
    parent = list(range(len(idx)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a in range(len(idx)):
        for b in range(a + 1, len(idx)):
            if (np.all(boxes[a][0] - tol <= boxes[b][1])
                    and np.all(boxes[b][0] - tol <= boxes[a][1])):
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[rb] = ra
    out = {}
    for k in range(len(idx)):
        out.setdefault(find(k), []).append(idx[k])
    return list(out.values())


def _fuse(chunks, idx):
    """One chunk dict from several, in a shared frame."""
    world = [chunks[i]["points"] + chunks[i]["centroid"] for i in idx]
    pts = np.vstack(world)
    centroid = pts.mean(axis=0)
    faces, off = [], 0
    for k, i in enumerate(idx):
        faces.append(chunks[i]["faces"] + off)
        off += len(world[k])
    return {
        "points": pts - centroid,
        "faces": np.vstack(faces),
        "uv": np.vstack([chunks[i]["uv"] for i in idx]),
        "face_mat": np.concatenate([chunks[i]["face_mat"] for i in idx]),
        "centroid": centroid,
    }


class Ruin:
    """What an earthquake left: fragments, and which of them let go."""

    def __init__(self, chunks, damage, materials, falls, field, source, lo, hi):
        self.chunks = chunks
        self.damage = damage
        self.materials = materials
        self.falls = falls
        self.field = field
        self.source = source
        self.lo, self.hi = lo, hi

    @property
    def span(self):
        return float((self.hi - self.lo).max())


def shake(stage, asset, *, severity=0.5, materials=("concrete",),
          soft_story=False, seed=0, target_size=8.0, up_axis="z",
          thickness=0.05, chunks=90, lift=0.02, quiet=False, scale=0.0):
    """Load *asset*, decide where it fails, and cut it accordingly.

    Returns a :class:`Ruin`. Nothing is stepped and nothing is authored — the
    caller owns the stage and the physics.
    """
    severity = float(np.clip(severity, 0.0, 1.0))
    mats = [MATERIALS[m] for m in materials]
    rng = np.random.default_rng(seed)

    src = load_source(stage, "/World/_source", resolve_asset(asset, target_size),
                      target_size, up_axis, quiet, scale=scale)
    surface = trimesh.Trimesh(src.vertices, src.faces, process=False)
    # `close_directly` copies, welds, fills holes and splits the mesh — the
    # most expensive step in this module — so it is called exactly once and
    # its result kept, not re-derived. `src.parts` is set unconditionally
    # (not only on the thicken path) so `build_chunks` never has to redo this
    # work itself for the common already-closed case.
    direct = solids.close_directly(surface)
    src.parts = direct[0] if direct is not None else solids.thicken(
        stage, "/World/_source", thickness,
        # `detail_area` is raised well above the fracture default: a ruin does
        # not need every shutter as its own rigid body, and each one is
        # another boolean against every cell it touches.
        detail_area=0.6, quiet=quiet)
    parts = src.parts

    # `build_chunks` re-centres and lifts the parts before it fractures, and
    # the damage field has to live in that same frame, so the shift is
    # recomputed here exactly as it is there.
    allb = np.array([p.bounds for p in parts])
    plo, phi = allb[:, 0].min(axis=0), allb[:, 1].max(axis=0)
    shift = np.array([-(plo + phi)[0] * 0.5, -(plo + phi)[1] * 0.5,
                      lift - plo[2]])
    lo, hi = plo + shift, phi + shift

    field = DamageField(lo, hi, severity, soft_story, rng)
    weak = opening_tree(src.vertices + shift, src.faces)

    def seed_fn(parts_now, n, seed_rng):
        """Crowd the Voronoi seeds into the damage, and onto the openings."""
        cand = fracture.seed_points(parts_now, n * 8, seed_rng)
        # Squared, over a low floor: the floor still puts a few seeds through
        # sound structure so it carries visible cracks, but the contrast is
        # sharp enough that an undamaged region comes out as a couple of large
        # blocks while a failed one shatters.
        w = 0.02 + field.at(cand) ** 2.0
        if weak is not None:
            d, _ = weak.query(cand)
            w = w * (1.0 + 1.6 * np.exp(-(d / 0.6) ** 2))
        w = np.maximum(w, 1e-9)
        pick = seed_rng.choice(len(cand), size=min(n, len(cand)),
                               replace=False, p=w / w.sum())
        return cand[pick]

    shatter = float(np.mean([m.shatter for m in mats]))
    n_cells = max(8, int(chunks * shatter * (0.35 + severity)))
    pieces = fracture.build_chunks(src, n_cells, seed, 0.004, 2, lift=lift,
                                   quiet=quiet, seed_fn=seed_fn)
    if not pieces:
        return Ruin([], np.zeros(0), [], np.zeros(0, bool), field, src, lo, hi)

    centroids = np.array([c["centroid"] for c in pieces])
    damage = field.at(centroids)

    edges = np.linspace(lo[2], hi[2], len(mats) + 1)
    band = np.clip(np.searchsorted(edges, centroids[:, 2], side="right") - 1,
                   0, len(mats) - 1)
    chunk_mats = [mats[b] for b in band]
    detach = damage > np.array([m.detach for m in chunk_mats])

    # EVERY body is dynamic; gravity decides what stays up. The alternative —
    # pinning the survivors as kinematic — left them hanging in mid-air once
    # the storey under them collapsed, held up by nothing.
    #
    # But releasing the survivors one by one is just as wrong: an undamaged
    # wall is monolithic, not a stack of loose blocks, and a Voronoi cut that
    # is only meant to show a crack would let it slump into a pile. So the
    # survivors that still touch are FUSED into single rigid bodies. An intact
    # upper storey then rides down as one piece when its supports go, which is
    # what a pancake collapse actually looks like.
    fused, fused_damage, fused_mats, fused_detach = [], [], [], []
    for i in np.nonzero(detach)[0]:
        fused.append(pieces[i])
        fused_damage.append(damage[i])
        fused_mats.append(chunk_mats[i])
        fused_detach.append(True)
    survivors = np.nonzero(~detach)[0]
    groups = _weld_groups(pieces, survivors, 3.0 * 0.004) if len(survivors) else []
    for g in groups:
        fused.append(_fuse(pieces, g))
        fused_damage.append(float(np.mean([damage[i] for i in g])))
        fused_mats.append(chunk_mats[g[0]])
        fused_detach.append(False)

    n_frag = len(pieces)
    pieces = fused
    damage = np.array(fused_damage)
    chunk_mats = fused_mats
    falls = np.array(fused_detach, dtype=bool)

    if not quiet:
        print(f"[quake] severity {severity:.2f} | {'+'.join(materials)}"
              f"{' | soft storey' if soft_story else ''} | "
              f"{len(field.regions)} failure regions", flush=True)
        # Percentage over the ORIGINAL fragment count, not the fused list —
        # after fusing, "118 of 119 bodies" reads as 99% when 118 of 172
        # fragments (69%) actually came down.
        print(f"[quake] {n_frag} fragments: {int(falls.sum())} detached "
              f"({100 * falls.sum() / max(n_frag, 1):.0f}%), survivors fused "
              f"into {len(groups)} bodies -> {len(pieces)} rigid bodies",
              flush=True)
    return Ruin(pieces, damage, chunk_mats, falls, field, src, lo, hi)
