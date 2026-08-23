"""survey — classify a mesh's solidity, without fracturing anything.

Three questions, each answered independently, because they are the three a
caller of `disaster.solids` actually needs before deciding what to do with an
asset:

1. **CLOSED — does it already bound a volume?** Reuses `solids.close_directly`
   directly, so the tier reported here is exactly the path `earthquake.shake`
   would take: ``watertight`` / ``hole-filled`` / ``multi-solid`` need no
   resampling; ``open-shell`` is what falls through to `solids.thicken`.

2. **COMPONENTS — one connected piece, or many?** A mesh that LOOKS like one
   object in a viewer is very often hundreds of disjoint pieces once split —
   the Objaverse house used to develop `earthquake.py` is 1821 of them.
   Component count on its own says nothing about closure (see 1), which is
   why it is reported separately rather than folded into the tier.

3. **HOLLOW — for a piece that IS closed, is it a thin shell or does it have
   real interior volume?** A car body is watertight and a few millimetres of
   metal; a stone block is watertight and solid to the core. Both are
   "closed", and treating them the same when deciding how to fracture would
   be wrong — cutting a hollow shell into cubes hollows out something that was
   never there. Measured by casting rays inward from the surface: the median
   distance to the first opposite-facing hit is the wall thickness, and
   comparing that to the object's own span says which case it is.
"""

from __future__ import annotations

import numpy as np
import trimesh

from . import solids

#: wall thickness, as a fraction of the object's largest extent, below which
#: a closed mesh counts as a hollow shell rather than a solid interior.
#: Calibrated by feel, not measurement — a car shell lands two orders of
#: magnitude below this, a stone block two above it, and there is a wide gap
#: between real examples for the threshold to sit in.
HOLLOW_RATIO = 0.05


def wall_thickness_ratio(mesh: trimesh.Trimesh, n_samples: int = 300, rng=None):
    """Median wall thickness / the mesh's own span, or None if untestable.

    Samples points on the surface, casts a ray inward along the surface
    normal, and measures the distance to the first backface it meets — which
    is the near wall's thickness if the mesh is a shell, or a long way (the
    far side of the object) if it is genuinely solid. Only meaningful on a
    mesh that already bounds a volume; the caller is responsible for that.
    """
    if not (mesh.is_volume and mesh.volume > 1e-9) or len(mesh.faces) == 0:
        return None
    span = float(mesh.extents.max())
    if span <= 0:
        return None

    rng = rng or np.random.default_rng(0)
    count = min(n_samples, max(len(mesh.faces) * 3, 8))
    pts, face_idx = trimesh.sample.sample_surface(mesh, count)
    normals = mesh.face_normals[face_idx]
    eps = span * 1e-4
    origins = pts - normals * eps        # start just inside the surface

    locations, i_ray, _ = mesh.ray.intersects_location(
        origins, -normals, multiple_hits=False)
    if not len(i_ray):
        return None
    dist = np.linalg.norm(locations - origins[i_ray], axis=1)
    return float(np.median(dist)) / span


def classify_mesh(vertices, faces, hollow_ratio=HOLLOW_RATIO, rng=None):
    """The three-axis report for one triangle soup.

    Returns a dict: ``tier`` (watertight / hole-filled / multi-solid /
    open-shell), ``n_components``, ``n_closed``, ``closed_area_frac`` (share
    of surface area already in a closed component — 1.0 for a clean
    watertight mesh, 0.0 for a fully open shell), ``wall_thickness_ratio`` and
    ``hollow`` (None when nothing closed exists to test), plus ``span``,
    ``area`` and ``faces`` for context.
    """
    surf = trimesh.Trimesh(vertices, faces, process=False)
    area_total = float(surf.area)
    span = float(surf.extents.max()) if len(surf.vertices) else 0.0

    result = solids.close_directly(surf)
    tier = solids.tier_name(result[1]) if result else "open-shell"

    welded = surf.copy()
    welded.merge_vertices()
    # Explicit FACE-INDEX groups, not `.split()`. `.split()` duplicated faces
    # at single-vertex (non-edge) contact points between parts on a real
    # asset — 514 faces came back from a 500-face mesh — which is silently
    # self-consistent in `solids.close_directly` (it compares the split's own
    # total to itself) but breaks here, where `closed_area` is compared
    # against the ORIGINAL mesh's area and came out at 105%. `submesh` on an
    # explicit index list cannot duplicate anything.
    groups = trimesh.graph.connected_components(
        welded.face_adjacency, nodes=np.arange(len(welded.faces)), min_len=1)
    # `repair=False`, and not optionally: `submesh()` defaults to
    # `repair=True` ("try to make submeshes watertight") and silently caps
    # open boundary loops with fan triangles — 500 faces came back as 514.
    # That is exactly the question this function exists to answer honestly;
    # letting `submesh` pre-answer it for me is the bug, not a convenience.
    comps = [welded.submesh([idx], append=False, repair=False)[0]
            for idx in groups]
    closed = [c for c in comps if c.is_volume and c.volume > 1e-9]
    closed_area = sum(c.area for c in closed)

    thickness_ratio = hollow = None
    if closed:
        biggest = max(closed, key=lambda c: c.volume)
        thickness_ratio = wall_thickness_ratio(biggest, rng=rng)
        if thickness_ratio is not None:
            hollow = thickness_ratio < hollow_ratio

    return {
        "tier": tier,
        "n_components": len(comps),
        "n_closed": len(closed),
        "closed_area_frac": closed_area / area_total if area_total > 0 else 0.0,
        "wall_thickness_ratio": thickness_ratio,
        "hollow": hollow,
        "span": span,
        "area": area_total,
        "faces": len(faces),
    }
