#!/usr/bin/env python3
"""earthquake_damage.py — shake one building down.

    cd AirStack
    UV_ENV_FILE=$PWD/.env.host uv run python \
        scene_gen/tools/earthquake_damage.py \
        --asset objaverse://6644de89c2f0449db3de934744162b63 \
        --material brick --severity 0.6 --soft-story --rubble

Loads one asset, decides where it fails, cracks it along those failures, drops
the failed parts with PhysX, and leaves a settled ruin. Everything is driven by
`--severity` between 0 and 1 and by `--material`, and everything is seeded, so
the same arguments give the same ruin.

The fracture machinery is `fracture_blast.py` — this file supplies only the
earthquake: WHERE the cracks go and WHAT falls. It hooks in through that
module's `seed_fn`, because where the Voronoi seeds are placed IS the fracture
pattern.

THE FOUR STAGES
---------------
1. **Failure regions.** A handful of blobs, biased toward the ground, because
   that is where an earthquake puts the load in. `--soft-story` adds a full
   storey-height band at the base, which is the one structural configuration
   worth special-casing: an open ground floor has nothing to resist the shear
   with and takes all the drift.

2. **Cracks propagate.** Two rules, both from the brief:
   - Anything above a failure fails too. Implemented as a running maximum up
     each column of the damage grid, with a per-metre decay so a small failure
     does not automatically take out ten storeys above it. Severity buys reach:
     at 1.0 the decay is small and the failure runs to the roof.
   - Cracks favour weak points. Openings are the weak points that matter — a
     real earthquake crack starts at a window or door corner, where the wall
     has the least section to carry shear. Those show up in the mesh as the
     boundary loops of the source surface, so no annotation is needed.

3. **Settle.** Chunks whose damage clears the material's threshold become
   dynamic and fall; the rest stay kinematic and hold the building up. PhysX
   runs for `--settle` seconds and everything is frozen again, so the result is
   a static scene ready to be searched.

4. **Detail and rubble.** Cut faces get the material colour plus per-face
   noise, standing in for the grime-and-fray texture that is not on Nucleus
   yet. `--rubble` scatters blocks whose count follows how much actually fell.

MATERIALS
---------
`--material` takes one or more of wood / concrete / brick / steel. They set how
finely the structure shatters, how readily a piece lets go, and what the rubble
looks like — brittle brick into many small fragments that detach early, ductile
steel into a few large ones that mostly hold. Several materials are assigned in
HEIGHT BANDS in the order given, so `--material brick wood` is a masonry ground
floor under a timber upper, which is a real building rather than a speckle.

Colours and block shapes stand in for the Nucleus materials; this table is the
thing that becomes a tag in the asset-pack YAML, so it is kept in one place and
keyed by name.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import trimesh
from scipy.spatial import cKDTree

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fracture_blast as fx                                     # noqa: E402


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


# ---------------------------------------------------------------------------
# Authoring
# ---------------------------------------------------------------------------


def author_ruin(stage, root, chunks, mat_paths, damage, mats, dirt, rng,
                density=2000.0):
    """Chunks as rigid bodies, cut faces coloured by material plus noise.

    Cut faces carry `primvars:displayColor` and are deliberately left with no
    material bound, so the renderer falls back to that colour — which is how
    the per-face noise shows at all. Exterior faces keep the source materials
    over the top of it.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    UsdGeom.Xform.Define(stage, root)
    default_mat = fx._flat_material(stage, root + "/Fallback", (0.6, 0.6, 0.6))

    for i, c in enumerate(chunks):
        path = f"{root}/chunk_{i:04d}"
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

        # Stage 4: the grime layer. Damage darkens, noise breaks it up.
        #
        # The noise is mostly on BRIGHTNESS, with only a trace of colour.
        # Perturbing R, G and B independently moves the hue rather than the
        # shade, and on a saturated colour like brick that is violent: at one
        # sigma of 0.05 the fragments came out magenta and olive. Grime varies
        # how light a surface is, not what colour it is.
        base = mats[i].colour * (1.0 - 0.35 * damage[i])
        shade = 1.0 + rng.normal(0.0, 2.0 * dirt, (len(faces), 1))
        tint = rng.normal(0.0, 0.2 * dirt, (len(faces), 3))
        cols = np.clip(base[None, :] * shade + tint, 0.0, 1.0)
        dc = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.uniform)
        dc.Set([Gf.Vec3f(*v) for v in cols])

        # Only the surviving original faces get a material; cut faces are left
        # bare so displayColor is what renders on them.
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        for value in np.unique(c["face_mat"]):
            if value < 0:
                continue
            src = mat_paths[value] if value < len(mat_paths) else None
            target = UsdShade.Material.Get(stage, src) if src else None
            if not (target and target.GetPrim().IsValid()):
                target = default_mat
            idx = np.nonzero(c["face_mat"] == value)[0]
            subset = UsdGeom.Subset.CreateGeomSubset(
                mesh, f"mat_{value:02d}", UsdGeom.Tokens.face, idx.tolist(),
                familyName="materialBind")
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(target)

        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()) \
            .CreateApproximationAttr().Set(UsdPhysics.Tokens.convexDecomposition)
        UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim()) \
            .CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateDensityAttr(density)


def author_rubble(stage, root, field, mats, severity, fallen, lo, hi, rng):
    """Solid-colour blocks around the base, more where more came down.

    Count follows the volume that actually detached rather than severity alone,
    so a building that held together does not spill a debris field it never
    produced. Placement is weighted by the damage projected down to the ground,
    which puts the pile under the part that collapsed.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    if fallen <= 0.0:
        return 0
    UsdGeom.Xform.Define(stage, root)

    # `fallen` is the fraction of fragments that came down, so the debris
    # field tracks what actually collapsed rather than the severity dial.
    n = int(np.clip(round(fallen * 150.0 * (0.3 + severity)), 0, 400))
    ground = field.grid.max(axis=2)                      # damage seen from above
    flat = ground.ravel()
    if flat.sum() <= 0:
        return 0
    cells = rng.choice(len(flat), size=n, p=flat / flat.sum())
    gy, gx = np.divmod(cells, field.res)
    step = (hi - lo)[:2] / max(field.res - 1, 1)

    count = 0
    for k in range(n):
        mat = mats[rng.integers(len(mats))]
        size, aspect = mat.block
        size *= rng.uniform(0.6, 1.5)
        dims = np.array([size * aspect, size, size * rng.uniform(0.5, 1.0)])
        # Spread outward from the footprint: debris lands beside a building,
        # not inside its footprint.
        centre = lo[:2] + np.array([gy[k], gx[k]]) * step
        centre = centre + rng.normal(0.0, 0.35 + 0.9 * severity, 2)
        box = trimesh.creation.box(dims)
        box.apply_transform(trimesh.transformations.random_rotation_matrix(
            rng.random(3)))
        drop = box.bounds[1][2] - box.bounds[0][2]
        pos = np.array([centre[0], centre[1],
                        lo[2] + drop * 0.5 + rng.uniform(0.0, 1.2 * severity)])

        path = f"{root}/rubble_{k:04d}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in box.vertices])
        mesh.CreateFaceVertexCountsAttr([3] * len(box.faces))
        mesh.CreateFaceVertexIndicesAttr(box.faces.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(*box.bounds[0]), Gf.Vec3f(*box.bounds[1])])
        UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(*pos))
        col = np.clip(mat.colour * (1.0 + rng.normal(0.0, 0.12)), 0.0, 1.0)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.constant).Set([Gf.Vec3f(*col)])

        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()) \
            .CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
        UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
        UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateDensityAttr(1800.0)
        count += 1
    return count


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--asset", required=True,
                   help="objaverse://<uid>, omniverse://…, airstack://…, path")
    p.add_argument("--material", nargs="+", default=["concrete"],
                   choices=sorted(MATERIALS),
                   help="structural material(s); several are assigned in "
                        "height bands, lowest first")
    p.add_argument("--severity", type=float, default=0.5,
                   help="0 = untouched, 1 = complete collapse")
    p.add_argument("--soft-story", action="store_true",
                   help="open ground floor: the whole base storey fails")
    p.add_argument("--rubble", action="store_true",
                   help="scatter debris blocks around the base")
    p.add_argument("--chunks", type=int, default=90,
                   help="baseline fragment count before material and severity")
    p.add_argument("--dirt", type=float, default=0.07,
                   help="per-face grime variation on cut faces (mostly shade)")
    p.add_argument("--settle", type=float, default=4.0,
                   help="seconds of PhysX before the ruin is frozen")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--target-size", type=float, default=8.0)
    p.add_argument("--up-axis", choices=("z", "y"), default="z")
    p.add_argument("--thickness", type=float, default=0.05,
                   help="metres of shell extrusion for open (sheet) geometry")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--shot", default="",
                   help="with --headless, write <shot>_before/_after.png")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    args.severity = float(np.clip(args.severity, 0.0, 1.0))
    mats = [MATERIALS[m] for m in args.material]
    rng = np.random.default_rng(args.seed)

    from isaacsim import SimulationApp
    simulation_app = SimulationApp(launch_config={"headless": args.headless})

    import omni.kit.app
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import UsdPhysics

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    fx.add_lighting(world.stage)

    # --- load and solidify, reusing the fracture tool's front end ----------
    asset = fx.resolve_asset(args.asset, args.target_size)
    source = fx.load_source(world.stage, "/World/_source", asset,
                            args.target_size, args.up_axis)
    surface = trimesh.Trimesh(source.vertices, source.faces, process=False)
    if fx.close_directly(surface) is None:
        # `detail_area` is raised well above the fracture tool's default: a
        # ruin does not need every shutter as its own rigid body, and each one
        # is another boolean against every cell it touches.
        source.parts = fx.thicken(world.stage, "/World/_source",
                                  args.thickness, detail_area=0.6)

    parts = source.parts or fx.close_directly(surface)[0]
    lift = 0.02

    # `build_chunks` re-centres and lifts the parts before it fractures. The
    # damage field has to live in that same frame, so the shift is recomputed
    # here exactly as it is there.
    allb = np.array([p.bounds for p in parts])
    plo, phi = allb[:, 0].min(axis=0), allb[:, 1].max(axis=0)
    shift = np.array([-(plo + phi)[0] * 0.5, -(plo + phi)[1] * 0.5,
                      lift - plo[2]])
    lo, hi = plo + shift, phi + shift

    field = DamageField(lo, hi, args.severity, args.soft_story, rng)
    weak = opening_tree(source.vertices + shift, source.faces)
    print(f"[quake] severity {args.severity:.2f} | {'+'.join(args.material)}"
          f"{' | soft storey' if args.soft_story else ''} | "
          f"{len(field.regions)} failure regions", flush=True)

    def seed_fn(parts_now, n, seed_rng):
        """Crowd the Voronoi seeds into the damage, and onto the openings."""
        cand = fx.seed_points(parts_now, n * 8, seed_rng)
        w = 0.05 + field.at(cand) ** 1.5
        if weak is not None:
            d, _ = weak.query(cand)
            w = w * (1.0 + 1.6 * np.exp(-(d / 0.6) ** 2))
        w = np.maximum(w, 1e-9)
        pick = seed_rng.choice(len(cand), size=min(n, len(cand)),
                               replace=False, p=w / w.sum())
        return cand[pick]

    shatter = float(np.mean([m.shatter for m in mats]))
    n_chunks = max(8, int(args.chunks * shatter * (0.35 + args.severity)))
    chunks = fx.build_chunks(source, n_chunks, args.seed, 0.004, 2,
                             lift=lift, seed_fn=seed_fn)
    if not chunks:
        print("[quake] nothing to damage", flush=True)
        simulation_app.close()
        return 1

    centroids = np.array([c["centroid"] for c in chunks])
    damage = field.at(centroids)

    # Several materials stack in height bands, lowest first.
    edges = np.linspace(lo[2], hi[2], len(mats) + 1)
    band = np.clip(np.searchsorted(edges, centroids[:, 2], side="right") - 1,
                   0, len(mats) - 1)
    chunk_mats = [mats[b] for b in band]

    thresh = np.array([m.detach for m in chunk_mats])
    falls = damage > thresh
    print(f"[quake] {len(chunks)} fragments, {int(falls.sum())} detach "
          f"({100 * falls.mean():.0f}%)", flush=True)

    author_ruin(world.stage, "/World/ruin", chunks, source.mat_paths,
                damage, chunk_mats, args.dirt, rng)
    n_rubble = 0
    if args.rubble:
        n_rubble = author_rubble(world.stage, "/World/rubble", field, mats,
                                 args.severity, float(falls.mean()), lo, hi, rng)
        print(f"[quake] {n_rubble} rubble blocks", flush=True)

    view = RigidPrim("/World/ruin/chunk_.*", name="ruin")
    world.scene.add(view)
    world.reset()

    span = float((hi - lo).max())
    set_camera_view([span * 1.8, -span * 1.8, span * 0.9],
                    [0.0, 0.0, span * 0.3])

    def shot(tag):
        if not (args.headless and args.shot):
            return
        import omni.kit.viewport.utility as vu
        for _ in range(60):
            omni.kit.app.get_app().update()
        vu.capture_viewport_to_file(vu.get_active_viewport(),
                                    file_path=f"{args.shot}_{tag}.png")
        for _ in range(40):
            omni.kit.app.get_app().update()
        print(f"[quake] wrote {args.shot}_{tag}.png", flush=True)

    shot("before")

    # --- stage 3: let the failed parts go ---------------------------------
    for i in np.nonzero(falls)[0]:
        prim = world.stage.GetPrimAtPath(f"/World/ruin/chunk_{i:04d}")
        if prim:
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(False)
    steps = int(args.settle * 60)
    for _ in range(steps):
        world.step(render=not args.headless)

    # Freeze, so what comes out is a static scene to fly a drone through.
    for i in range(len(chunks)):
        prim = world.stage.GetPrimAtPath(f"/World/ruin/chunk_{i:04d}")
        if prim:
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(True)
    for k in range(n_rubble):
        prim = world.stage.GetPrimAtPath(f"/World/rubble/rubble_{k:04d}")
        if prim:
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(True)
    world.step(render=False)
    print("[quake] settled and frozen", flush=True)

    shot("after")
    if args.headless:
        simulation_app.close()
        return 0

    while simulation_app.is_running():
        world.step(render=True)
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
