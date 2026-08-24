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

ONE SETTLE FOR THE WHOLE GRID
-----------------------------
Every cell is laid out on a spaced grid and PhysX runs ONCE over all of them,
then each is exported re-centred to the origin. Settling per cell would pay the
simulation start-up cost hundreds of times; this is the single most important
thing the original script got right and it is preserved verbatim.
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
from disaster import levels as L                                # noqa: E402

#: Metres between grid cells. Must exceed the largest asset's footprint plus
#: its debris throw, or two archetypes settle into each other and each exports
#: a piece of its neighbour.
GRID_M = 40.0

#: PhysX steps for the one settle pass. From the original bake script.
SETTLE_STEPS = 420


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
        self.out_dir = out_dir
        self.seed = int(seed)
        self.parent = parent
        self.cells = []          # [(Item, level, x, y, prim_paths, extra)]
        self.loose = []
        self.static = []
        self.records = []

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
            try:
                paths, extra = self._build_one(it, level, x, y, cell, ssf)
            except Exception as exc:                            # noqa: BLE001
                # One bad asset must not cost the other 200 archetypes.
                print(f"[stage-a] SKIP {it.type}_{level}: "
                      f"{type(exc).__name__}: {exc}")
                continue
            self.cells.append((it, level, x, y, paths, extra))
            dt = time.time() - t_cell
            if dt > 5.0:
                print(f"[stage-a]     {dt:.0f}s", flush=True)
        return len(self.cells)

    def _build_one(self, item, level, x, y, cell, ssf):
        if item.build == "modular":
            return self._build_modular(item, level, x, y, cell, ssf)
        if item.kind == L.VEGETATION:
            return self._build_vegetation(item, level, x, y, cell, ssf)
        return self._build_library(item, level, x, y, cell, ssf)

    def _rng(self, item, level):
        """Per-cell RNG. Seeded on (type, level) so re-baking one archetype
        reproduces it exactly, and so two cells never share a stream."""
        return random.Random(self.seed + (abs(hash((item.type, level)))
                                          % 100_000))

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

        ONE SCRIPT PER DISASTER, ALL OF THEM ON THE `mesh_damage` API. For an
        earthquake that script is `disaster.quake`, and it is called by RUNG
        rather than by intensity: a rung is a set of mechanisms over regions of
        the plan (`quake.RUNG_PLAN`), so the archetype gets the composed
        failure — one wing sheared off, the ground floor gone under another,
        the rest cracked — instead of one mode at a midpoint intensity.
        Everything else still goes through `mesh_damage.damage_building`
        directly until it has a script of its own.

        `solid=False`, unlike the code this replaced. That call passed
        `solid=True` unconditionally with a docstring explaining why Stage A
        should thicken MORE eagerly than the live path — but `solid=True` is
        the asset-pack's "this art already has material in its walls" flag,
        and `damage_building` reads it as "skip solidify". So the argument was
        inverted against its own reasoning and every library archetype baked
        so far is the paper shell it was written to prevent.
        """
        from disaster import mesh_damage as md
        from disaster import quake

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
        seed = self.seed + abs(hash(level)) % 9973

        if self.disaster == "earthquake":
            # `settle_it=False`: Stage A settles the WHOLE grid in one PhysX
            # pass at the end (`Baker.settle`), so a per-cell settle here would
            # run the solver once per archetype and then again over everything.
            got = quake.at_level(self.stage, prim, level, seed=seed,
                                 settle_it=False)
        else:
            dis = (self.config.get("disaster") or {})
            thick = ((dis.get("mesh_damage") or {}).get("thickness") or {})
            # Intensity is the MIDPOINT of the rung's band, not its threshold:
            # the threshold is where the rung starts, so using it would render
            # every archetype at the gentlest damage its name allows.
            got = md.damage_building(
                self.stage, prim, self.disaster,
                self._intensity(item.kind, level), seed=seed,
                wall_m=float(thick.get("wall_m", 0.5)),
                heading_deg=md._heading_of(dis))
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
                            fire=(self.disaster == "fire"))
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
    def settle(self):
        """One PhysX pass over the whole grid. See the module docstring."""
        from disaster import settle as S

        for _it, _lv, _x, _y, paths, _extra in self.cells:
            for p in paths:
                pr = self.stage.GetPrimAtPath(p)
                if pr and pr.IsValid() and pr.IsActive():
                    self.static.append(p)
        self._pump(20)
        if self.loose:
            print(f"[stage-a] settling {len(self.loose)} loose fragment(s)")
            S.run(self.stage, self.loose, self.static, steps=SETTLE_STEPS,
                  kick=0.15, rng=random.Random(self.seed), bake_result=True)
        self._pump(10)

    # -- export -----------------------------------------------------------
    def export(self) -> list:
        """Write one self-contained USD per cell, re-centred to the origin."""
        from disaster import bake as B

        out = lib.disaster_dir(_SCENE_GEN, self.disaster, self.out_dir)
        os.makedirs(out, exist_ok=True)
        missing = 0
        for item, level, x, y, paths, extra in self.cells:
            dst = os.path.join(out, lib.archetype_name(item.type, level)
                               + ".usd")
            try:
                if not B.export_object(self.stage, None, list(paths) + list(extra),
                                       dst, recenter=(x, y, 0.0)):
                    continue
                meshes, _ok, unbound = B.validate(dst)
            except Exception as exc:                            # noqa: BLE001
                print(f"[stage-a] export FAILED {os.path.basename(dst)}: "
                      f"{type(exc).__name__}: {exc}")
                continue
            missing += unbound
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
            })
        self._missing = missing
        return self.records

    def write_manifest(self) -> str:
        out = lib.disaster_dir(_SCENE_GEN, self.disaster, self.out_dir)
        return lib.write_manifest(
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
