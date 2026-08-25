"""bake — Stage A: build every (type, level) once, and export it.

    Stage A is EXHAUSTIVE, layout-independent, and paid once.

Needs Isaac Sim: fracture is CPU/trimesh and settle is PhysX, so this module is
driven by a launch script rather than run on the host. `archetypes/plan.py`
computes what it will do and costs nothing — run that first.

WHAT THIS REPLACED
------------------
`bake_archetypes_launch_script.py`, which did the same thing for ONE disaster
(wildfire) over two hardcoded type lists (`mh.STYLES`, a dict of six tree URLs)
and two hardcoded level tuples. Everything it knew is now read from the config
and the ladder, so a new disaster costs a `LADDERS` entry rather than a new
launch script.

THE THREE WAYS TO BUILD A CLEAN INSTANCE
-----------------------------------------
SPEC step A.3.a says "build one clean instance of that type". That means three
different things depending on where the type came from, which is exactly where
the codebase's two surviving damage stacks both find a home:

    modular    assemble from the kit (`detail.modular_house`), then damage with
               `disaster.damage_flow` — it knows the kit's wall/roof/gable
               structure and produces wreckage matching the live suburb.
    library    reference the USD whole, then damage with `disaster.mesh_damage`
               — the only asset-generic operator set (subdivide, solidify,
               delete_faces, value_noise), which is what an arbitrary Nucleus
               or Objaverse building needs.
    vegetation reference the USD, then `disaster.vegetation.burn_tree`, which
               works on the instancer rather than the mesh.

ONE CELL AT A TIME: BUILD, SETTLE, EXPORT, UNLOAD
-------------------------------------------------
The original script laid every cell out on a grid and ran PhysX ONCE over all
of them, to pay the simulation start-up once. With ~1,200-cell rubble that is
the wrong trade, measured (2026-08-25, `--config urban_quake_tiny --used-only`,
16 cells): the resident geometry ran the GPU out of memory from cell 6 onward
(26,888 Vulkan OOM errors), and the single settle over 5,064 bodies then took
975 s, left 4,825 of them still moving at the step ceiling and DROPPED NOTHING
(drop median -0.02 m). Every baked tower was a cracked plate standing at full
height, and always had been.

So each cell is settled and exported as soon as it is built, then removed from
the stage. A cell is ~900 bodies and settles in seconds; the simulation
start-up it pays per cell is smaller than the settle it saves. Grid positions
are kept so nothing else about a cell changes.
"""

from __future__ import annotations

import math
import os
import random
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from archetypes import library as lib                           # noqa: E402
from archetypes import plan as P                                # noqa: E402
from disaster import kinds                                      # noqa: E402
from disaster import levels as L                                # noqa: E402

#: Metres between grid cells. Must exceed the largest asset's footprint plus
#: its debris throw, or two archetypes settle into each other and each exports
#: a piece of its neighbour.
GRID_M = 40.0

#: Ceiling on PhysX steps per cell's settle. `settle.run` stops as soon as
#: the pile is at rest, so this only binds on a collapse still moving: a
#: piece off a 70 m tower needs ~4 s just to reach the ground.
SETTLE_STEPS = 900


def prepare_stage(stage):
    """A ground plane to settle onto, and enough light to look at the result.

    Lives here rather than in an entry point because the ground's Z is what
    every archetype settles against: a launch script and a CLI that disagreed
    about it would bake two subtly different libraries from the same config.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux

    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))

    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/arch_ground"))
    e = 800.0
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                            Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.25, 0.15)])
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome")).CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2200.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))
    return stage


def _centroid_z(stage, path) -> float:
    """World z of the mean vertex of the mesh at *path* (metres)."""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return 0.0
    xf = UsdGeom.XformCache()
    acc, n = np.zeros(3), 0
    for gp in Usd.PrimRange(prim):
        if not gp.IsA(UsdGeom.Mesh):
            continue
        pts = gp.GetAttribute("points").Get()
        if not pts:
            continue
        c = np.asarray(pts, dtype=float).mean(axis=0)
        w = xf.GetLocalToWorldTransform(gp).Transform(Gf.Vec3d(*c))
        acc += np.array(w) * len(pts)
        n += len(pts)
    return float(acc[2] / n) if n else 0.0


def _cell_xy(idx: int, ncol: int, y0: float = 0.0) -> tuple:
    return (idx % ncol) * GRID_M, y0 + (idx // ncol) * GRID_M


def _grid_columns(n: int) -> int:
    return max(1, int(math.ceil(math.sqrt(max(n, 1)))))


class Baker:
    """Runs Stage A onto a live Isaac Sim stage.

    Split from the launch script so the sequencing is testable by reading it
    and so a second entry point (a batch bake over several disasters) does not
    have to copy it.
    """

    def __init__(self, stage, config: dict, disaster: str, out_dir: str,
                 seed: int = 7, parent: str = "/World/stage/generated"):
        self.stage = stage
        self.config = config
        self.disaster = str(disaster or "none").lower()
        # The type's own behaviour — which damage script wrecks an archetype,
        # and whether a felled tree comes out charred. `self.disaster` stays a
        # string because it is also a directory name and a manifest field.
        self.model = kinds.get(self.disaster)
        self.out_dir = out_dir
        self.seed = int(seed)
        self.parent = parent
        self.cells = []          # [(Item, level, x, y, prim_paths, extra)]
        self.loose = []
        self.static = []
        self.records = []
        self._missing = 0

    # -- build ------------------------------------------------------------
    def build(self, items: list, ssf: float) -> int:
        """Lay every (type, level) out on the grid and damage it in place."""
        from pxr import UsdGeom, Sdf

        combos = [(it, lv) for it in items for lv in it.levels]
        ncol = _grid_columns(len(combos))
        print(f"[stage-a] {len(combos)} cells on a {ncol}-column grid")

        for idx, (it, level) in enumerate(combos):
            x, y = _cell_xy(idx, ncol)
            cell = f"{self.parent}/a_{it.kind[:3]}_{it.type}_{level}"
            UsdGeom.Scope.Define(self.stage, Sdf.Path(cell))
            # Per-cell progress. A 250-archetype bake runs for hours; without
            # this there is no way to tell a slow cell from a hung one, and no
            # way to name the cell that killed the process when one does.
            t_cell = time.time()
            print(f"[stage-a] {idx + 1}/{len(combos)} {it.type}_{level} "
                  f"({it.build}/{it.kind})", flush=True)
            n_loose, n_static = len(self.loose), len(self.static)
            try:
                paths, extra = self._build_one(it, level, x, y, cell, ssf)
            except Exception as exc:                            # noqa: BLE001
                # One bad asset must not cost the other 200 archetypes.
                print(f"[stage-a] SKIP {it.type}_{level}: "
                      f"{type(exc).__name__}: {exc}")
                self._unload(cell)
                continue
            self.cells.append((it, level, x, y, paths, extra))
            # This cell's own loose pieces and statics — see the module
            # docstring for why the settle is per cell.
            stats = self._settle_cell(paths, self.loose[n_loose:],
                                      self.static[n_static:])
            # CONVERGED OR NOT EXPORTED. Fragments are merged into static
            # meshes on export, which takes away the scene's own settle as a
            # second chance at a bad pose — so a pose is only baked when the
            # solver actually finished with it: nothing still moving, nothing
            # through the floor. A frozen mid-air collapse is rejected the
            # way an untextured archetype is.
            if stats and not stats["converged"]:
                print(f"[stage-a] REJECTED {it.type}_{level}: settle did not "
                      f"converge ({stats['still_moving']} still moving, "
                      f"{stats['through_floor']} through the floor, "
                      f"{stats['steps_used']}/{stats['steps']} steps)")
                self._missing += 1
            else:
                self._export_cell(it, level, x, y, paths, extra, stats)
            self._unload(cell)
            dt = time.time() - t_cell
            if dt > 5.0:
                print(f"[stage-a]     {dt:.0f}s", flush=True)
        return len(self.cells)

    def _unload(self, cell):
        """Take a finished cell off the stage so it stops costing memory."""
        from pxr import Sdf
        prim = self.stage.GetPrimAtPath(Sdf.Path(cell))
        if prim and prim.IsValid():
            prim.SetActive(False)
            self.stage.RemovePrim(Sdf.Path(cell))
        self._pump(2)

    def _build_one(self, item, level, x, y, cell, ssf):
        if item.build == "modular":
            return self._build_modular(item, level, x, y, cell, ssf)
        if item.kind == L.VEGETATION:
            return self._build_vegetation(item, level, x, y, cell, ssf)
        return self._build_library(item, level, x, y, cell, ssf)

    def _rng(self, item, level):
        """Per-cell RNG. Seeded on (type, level) so re-baking one archetype
        reproduces it exactly, and so two cells never share a stream.

        `md.stable_seed`, not `hash()` — Python salts str hashing per process,
        so the sentence above was false in exactly the way that is hardest to
        notice: every bake looked right and no two were the same."""
        from disaster import mesh_damage as md

        return random.Random(
            self.seed + md.stable_seed(item.type, level, mod=100_000))

    def _build_modular(self, item, level, x, y, cell, ssf):
        """Kit house: assemble, then break with `damage_flow`."""
        import scene_generator as sg
        from detail import modular_house as mh
        from disaster import damage, damage_flow

        pls = mh.build_building(item.source, x, y, 0.0,
                                random.Random(self.seed), category="house")
        pal = mh.STYLES[item.source].get("palette")
        if pal:
            for q in pls:
                q["palette"] = pal
        sg.apply_placements(self.stage, pls, cell, ssf)
        mh.apply_palette(self.stage, pls, cell)
        paths = [q["prim_path"] for q in pls]

        rung = self._rung(item.kind, level)
        if level == "pristine":
            return paths, []
        rng = self._rng(item, level)
        if rung is not None and rung.finish is None and level == "scorched":
            damage.soot_materials(self.stage, pls, cell, rng, strength=(3, 5))
            return paths, []
        import numpy as np
        mats = damage.char_materials(self.stage, cell)
        frags = damage_flow.damage_building(
            self.stage, cell, pls, f"{item.type}_{level}", level,
            (rung.finish if rung else None), mats, rng,
            np.random.default_rng(self.seed))
        damage.soot_materials(self.stage, pls, cell, rng, strength=(4, 6))
        self.loose.extend(frags)
        return paths, list(frags)

    def _build_library(self, item, level, x, y, cell, ssf):
        """Arbitrary building USD: reference it, then the disaster's pipeline.

WHICH damage runs is the disaster's own decision, not this
        function's: `Disaster.damage_archetype` routes to the type's script
        where it has one (`mesh_damage.DAMAGE_SCRIPTS` — `disaster.quake` for
        an earthquake, `disaster.tornado` for a tornado) and to the
        asset-generic operators where it does not. This stage stays
        type-agnostic and never names a disaster.

        `solid=False`, unlike the code this replaced. That call passed
        `solid=True` unconditionally with a docstring explaining why Stage A
        should thicken MORE eagerly than the live path — but `solid=True` is
        the asset-pack's "this art already has material in its walls" flag,
        and `damage_building` reads it as "skip solidify". So the argument was
        inverted against its own reasoning and every library archetype baked
        so far is the paper shell it was written to prevent.
        """
        from disaster import mesh_damage as md

        prim_path = f"{cell}/asset"
        # AT THE SCENE'S SCALE. The Nucleus packs are centimetre-authored, so
        # referencing at 1.0 built archetypes 8 km across — which Stage B then
        # placed as buildings a hundred times bigger than their block.
        self._reference(prim_path, item.source, x, y, scale=item.scale,
                        axis_up=item.axis_up)
        if level == "pristine":
            return [prim_path], []

        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return [prim_path], []
        seed = self.seed + md.stable_seed(level)

        got = self.model.damage_archetype(
            self.stage, prim, level, seed=seed, config=self.config,
            intensity=self._intensity(item.kind, level))
        frags = list(got.get("loose", ()))
        self.loose.extend(frags)
        return [prim_path] + list(got.get("paths", ())), []

    def _build_vegetation(self, item, level, x, y, cell, ssf):
        """Tree: reference it, then burn/break it on the instancer."""
        from disaster import vegetation as veg

        prim_path = f"{cell}/tree"
        # The pack's scale, not a hardcoded 0.01 — that constant was right for
        # the AEC vegetation the original bake script had baked in and wrong
        # for anything else.
        self._reference(prim_path, item.source, x, y, scale=item.scale,
                        axis_up=item.axis_up)
        if level == "pristine":
            return [prim_path], []

        res = veg.burn_tree(self.stage, prim_path, level, self.parent,
                            f"{cell}/debris", self._rng(item, level),
                            debris_scale=0.5, ground_z=0.0, verbose=False,
                            # Only a fire chars a tree. Everything else uses
                            # the same felling geometry with no soot on it.
                            fire=self.model.chars_vegetation)
        extra = (list(res.get("statics", [])) + list(res.get("loose", []))
                 + list((res.get("info") or {}).get("made", [])))
        self.loose.extend(res.get("loose", ()))
        self.static.extend(res.get("statics", ()))
        return [prim_path], extra

    def _reference(self, prim_path, usd, x, y, scale=1.0, axis_up="Z"):
        """Reference *usd* at the cell, at the size and orientation the SCENE
        would have given it. An archetype is a stand-in for the asset in situ;
        one built at a different scale is not a stand-in for anything.
        """
        import scene_generator as sg
        from pxr import Gf, Sdf, UsdGeom

        prim = self.stage.DefinePrim(Sdf.Path(prim_path))
        prim.GetReferences().AddReference(
            sg._join_asset_root(usd, str(self.config.get("asset_root", ""))))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        # Y-up assets need the same 90-degree roll `apply_placements` gives
        # them, or the archetype is baked lying on its side.
        if str(axis_up).upper() == "Y":
            xf.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0))
        s = float(scale)
        if s != 1.0:
            xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))
        self._pump(2)

    def _rung(self, kind, level):
        for r in L.ladder(self.disaster, kind):
            if r.name == level:
                return r
        return None

    def _intensity(self, kind, level) -> float:
        """The damage intensity to render *level* at: the midpoint of its band.

        A rung's `at` is where it BEGINS, so damaging at `at` renders every
        archetype at the gentlest damage its name permits — a `burned_out`
        house that looks merely scorched. The midpoint between this rung's
        threshold and the next is what the name actually means.
        """
        rungs = L.ladder(self.disaster, kind)
        names = [r.name for r in rungs]
        if level in names:
            i = names.index(level)
            lo = rungs[i].at
            hi = rungs[i + 1].at if i + 1 < len(rungs) else 1.0
            return (lo + hi) / 2.0
        # A variant (`fallen`, `stump`) sits at its parent rung's top.
        for i, r in enumerate(rungs):
            if level in r.variants:
                hi = rungs[i + 1].at if i + 1 < len(rungs) else 1.0
                return (r.at + hi) / 2.0
        return 1.0

    def _pump(self, n=1):
        try:
            import omni.kit.app
            app = omni.kit.app.get_app()
            for _ in range(n):
                app.update()
        except Exception:                                       # noqa: BLE001
            pass

    # -- settle -----------------------------------------------------------
    def _settle_cell(self, paths, loose, static):
        """PhysX over ONE cell: its loose pieces against its own standing
        geometry. See the module docstring for why not the whole grid.
        Returns the convergence stats, or None when there was nothing to
        settle (a pristine cell converges trivially)."""
        from disaster import settle as S
        if not loose:
            return None
        statics = list(static)
        for p in paths:
            pr = self.stage.GetPrimAtPath(p)
            if pr and pr.IsValid() and pr.IsActive():
                statics.append(p)
        self._pump(5)
        print(f"[stage-a]     settling {len(loose)} loose fragment(s)",
              flush=True)
        info = S.run(self.stage, list(loose), statics, steps=SETTLE_STEPS,
                     kick=0.15, rng=random.Random(self.seed), bake_result=True)
        self._pump(5)
        # Through the floor is judged at the piece's CENTROID: a fragment is
        # authored with no xformOp, so its prim origin is the building's and
        # follows the piece down — a 40 m drop reads as 40 m underground.
        lost = sum(1 for p in loose if _centroid_z(self.stage, p) < -2.0)
        stats = {"bodies": int(info.get("rigid", len(loose))),
                 "still_moving": int(info.get("still_moving", 0)),
                 "through_floor": int(lost),
                 "steps_used": int(info.get("steps_used", 0)),
                 "steps": int(SETTLE_STEPS),
                 "drop_median_m": round(float(info.get("drop_median", 0.0)), 2),
                 "spread_max_m": round(float(info.get("spread_max", 0.0)), 1)}
        # A HANDFUL OF CREEPING BODIES IS NOT A FAILED SETTLE. Requiring zero
        # rejected a cell whose pile had visibly come to rest because 176 of
        # 359 fragments were still moving a few mm/s at the step ceiling —
        # interpenetrating Voronoi cells never quite stop shoving each other.
        # What actually ruins a baked wreck is geometry through the floor, or
        # a pile that never fell at all, so those stay absolute.
        moving_ok = max(4, int(0.05 * stats["bodies"]))
        stats["converged"] = (stats["still_moving"] <= moving_ok
                              and stats["through_floor"] == 0)
        return stats

    def settle(self):
        """Kept for the old build -> settle -> export sequence; every cell
        has already been settled as it was built."""
        return None

    # -- export -----------------------------------------------------------
    def export(self) -> list:
        """The records of every cell exported so far — each cell is written
        as soon as it is settled (`_export_cell`)."""
        return self.records

    def _export_cell(self, item, level, x, y, paths, extra,
                     settle=None) -> bool:
        """One self-contained USD for a cell, re-centred to the origin."""
        from disaster import bake as B

        out = lib.disaster_dir(_SCENE_GEN, self.disaster, self.out_dir)
        os.makedirs(out, exist_ok=True)
        if True:
            dst = os.path.join(out, lib.archetype_name(item.type, level)
                               + ".usd")
            try:
                if not B.export_object(self.stage, None, list(paths) + list(extra),
                                       dst, recenter=(x, y, 0.0)):
                    print(f"[stage-a] nothing to export for "
                          f"{os.path.basename(dst)}")
                    return False
                meshes, _ok, unbound = B.validate(dst)
                # Bound is not textured — see `bake.unresolved_textures`.
                # A library of black boxes is worse than no library, so an
                # archetype whose maps resolve to nothing is not recorded.
                untex = B.unresolved_textures(dst)
            except Exception as exc:                            # noqa: BLE001
                print(f"[stage-a] export FAILED {os.path.basename(dst)}: "
                      f"{type(exc).__name__}: {exc}")
                return False
            if untex:
                print(f"[stage-a] REJECTED {os.path.basename(dst)}: "
                      f"{len(untex)} texture(s) resolve to nothing, e.g. "
                      f"{untex[0]}")
                self._missing += len(untex)
                return False
            self._missing += unbound
            self.records.append({
                "type": item.type, "level": level, "kind": item.kind,
                "build": item.build, "source": str(item.source),
                # RELATIVE TO THE MANIFEST, not to `scene_gen`. Anchoring to
                # scene_gen looked fine for the default output root and
                # produced `../../../../../tmp/...` the moment `--out` pointed
                # outside the tree — which Stage B then pasted into an
                # `airstack://` URL. A library is self-describing: the
                # manifest sits beside its USDs and that is the only anchor
                # that survives being moved or copied to Nucleus.
                "usd": os.path.basename(dst),
                "meshes": meshes, "bound_missing": unbound,
                # How the settle ended, so a library says whether its wrecks
                # are piles (see the convergence gate in `build`).
                "settle": settle or {"bodies": 0, "converged": True},
            })
        return True

    def write_manifest(self) -> str:
        out = lib.disaster_dir(_SCENE_GEN, self.disaster, self.out_dir)
        # MERGED into what is there. A `--used-only` or `--only` bake is a
        # partial one by design; replacing the manifest with its records
        # silently un-baked every other type in the library (measured: a
        # tiny bake left the showcase with 2 of its 10 building types).
        return lib.merge_manifest(
            os.path.join(out, lib.MANIFEST_NAME), self.records,
            {"disaster": self.disaster, "seed": self.seed,
             "asset_pack": self.config.get("asset_pack"),
             "grid_m": GRID_M})


def run(stage, config: dict, disaster: str, out_dir: str = "",
        seed: int = 7, parent: str = "/World/stage/generated",
        ssf: float = 1.0, only=None) -> dict:
    """Bake one disaster's archetype library. Returns a summary dict.

    *only* restricts the bake to a set of ``(type, kind)`` pairs. A full bake
    runs for hours (measured: ~40 s per library archetype), so being able to
    re-bake the handful that failed — or try one before committing to all of
    them — is the difference between iterating and starting over.
    """
    dtype = str(disaster or (config.get("disaster") or {}).get("type")
                or "none").lower()
    items = P.build_plan(config, dtype)
    if only:
        want = set(only)
        items = [i for i in items if (i.type, i.kind) in want]
    # The plan is NOT printed here: every caller resolves and prints it first,
    # so that it can be reviewed (or --dry-run'd) before Kit boots.

    # Only the MODULAR path needs vtk_fracture's boolean backends. Calling this
    # unconditionally made a library-only bake die on an environment that has
    # no `pip` to install them with, having done no work — and the failure
    # names packages that bake would never have used.
    if any(i.build == "modular" for i in items):
        from disaster import vtk_fracture as fracture
        fracture.ensure_deps()
    t0 = time.time()
    baker = Baker(stage, config, dtype, out_dir, seed, parent)
    baker.build(items, ssf)
    baker.settle()
    recs = baker.export()
    manifest = baker.write_manifest()
    dt = time.time() - t0

    want = sum(len(i.levels) for i in items)
    out_dir = os.path.dirname(manifest)
    size_mb = sum(os.path.getsize(os.path.join(out_dir, r["usd"]))
                  for r in recs
                  if os.path.exists(os.path.join(out_dir, r["usd"]))) / 1e6
    print("\n" + "=" * 72)
    print(f"STAGE A — archetype library for '{dtype}'")
    print(f"  {len(recs)}/{want} archetypes  ->  {os.path.dirname(manifest)}")
    print(f"  {size_mb:.0f} MB, {dt:.0f} s, "
          f"{getattr(baker, '_missing', 0)} unresolved material(s)")
    if len(recs) < want:
        print(f"  WARNING: {want - len(recs)} archetype(s) did not export; "
              "Stage B will fall back down the ladder for those.")
    print("=" * 72 + "\n")
    return {"disaster": dtype, "baked": len(recs), "wanted": want,
            "manifest": manifest, "seconds": dt}
