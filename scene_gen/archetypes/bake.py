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

BUILD ONE AT A TIME, SETTLE IN BATCHES, EXPORT, UNLOAD
------------------------------------------------------
The original script laid every cell out on a grid and ran PhysX ONCE over all
of them, to pay the simulation start-up once. Measured (2026-08-25,
`--config urban_quake_tiny --used-only`, 16 cells) that failed twice over: the
resident geometry ran the GPU out of memory from cell 6 onward (26,888 Vulkan
OOM errors), and the single settle over 5,064 bodies took 975 s, left 4,825 of
them still moving at the step ceiling and DROPPED NOTHING (drop median
-0.02 m). Every baked tower was a cracked plate standing at full height.

The answer then was one cell at a time. Neither cause was really about batch
size, though, and both have since been addressed:

  * the OOM was VULKAN — the renderer holding every fractured building
    resident, not PhysX holding bodies. `_hide_for_settle` takes the batch out
    of the renderer while it solves, which physics does not read.
  * the settle stopped on a single max over every body in the scene, so one
    piece still tumbling anywhere pinned the whole batch to the ceiling.
    `settle.run(groups=...)` now tracks and judges each pile separately.

So cells are built one at a time and settled in bounded batches
(`SETTLE_BATCH_CELLS`, `SETTLE_BATCH_BODIES`), then exported and removed from
the stage. A batch costs the SLOWEST pile rather than the sum of them and pays
the simulation start-up once for the group; the budget is what keeps a
1,195-body tower from being grouped with anything. `SCENE_SETTLE_BATCH=1`
restores the one-at-a-time behaviour exactly. Grid positions are unchanged —
`GRID_M` already guarantees two cells cannot reach each other, which is what
makes settling them together the same physics as settling them apart.
"""

from __future__ import annotations

import datetime
import math
import json
import os
import random
import sys
import time
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from archetypes import library as lib                           # noqa: E402
from archetypes import plan as P                                # noqa: E402
from archetypes import preview as PV                            # noqa: E402
from archetypes import version as V                             # noqa: E402
from disaster import kinds                                      # noqa: E402
from disaster import mesh_damage as _md                        # noqa: E402
from disaster import levels as L                                # noqa: E402

#: Metres between grid cells. Must exceed the largest asset's footprint plus
#: its debris throw, or two archetypes settle into each other and each exports
#: a piece of its neighbour.
GRID_M = 40.0

#: Half-extent of the ground `prepare_stage` lays down, metres. A batch may not
#: spread wider than this or its outermost cells fall off the edge of the world.
GROUND_HALF_M = 800.0

#: Ceiling on PhysX steps per cell's settle. `settle.run` stops as soon as
#: the pile is at rest, so this only binds on a collapse still moving: a
#: piece off a 70 m tower needs ~4 s just to reach the ground.
#: RAISED FROM 900 ON 2026-08-25: at 900 (15 s of sim) a 557-fragment
#: soft-storey collapse had every body still moving when the budget ran out,
#: and `settle.run` stops early the moment the pile is quiet, so a higher
#: ceiling costs nothing on the cells that were already converging.
#: Overridable so a stubborn cell can be retried at a higher ceiling without
#: editing this file: `SCENE_SETTLE_STEPS=6000`. Kept an env knob rather than a
#: raised default because the two rejections seen so far -- Amar_Tower and
#: SM_Building_13 -- look like depenetration feeding energy back in (bodies
#: reported above the 30 m/s velocity cap), which more steps would not fix.
SETTLE_STEPS = int(os.environ.get("SCENE_SETTLE_STEPS", "1800") or 1800)

#: Seconds of simulated time per solver step. Cost is per STEP, so a coarser
#: step buys the same collapse for proportionally fewer of them. Measured on
#: the 82 m tower: 1/20 settles ~4x faster than the 1/60 default and converges
#: just as well (0 bodies still moving either way). Below about 1/10 fast
#: fragments start passing THROUGH the ground between solves, so this is the
#: conservative end of what the sweep showed was safe.
SETTLE_DT = 1.0 / 20.0

#: How big a debris mound's COLLIDER is against the mound you can see.
PILE_COLLIDER = 0.72

#: Stop the bake when free space on the OUTPUT filesystem falls below this.
#:
#: A damaged archetype is ~100 MB (measured over the 2026-08-26 earthquake
#: library: cracked 160, partial_collapse 102, pancaked 87, soft_storey 68,
#: pristine 3), so a 75-type urban library is ~30 GB. That is more than this
#: box usually has spare, which makes an unattended overnight bake a way to
#: fill the root filesystem. Stopping early is cheap — a partial library is
#: usable and Stage B falls back down the ladder — and a full disk is not.
DISK_FLOOR_GB = float(os.environ.get("BAKE_DISK_FLOOR_GB", 12.0))

#: How many cells to settle in ONE PhysX pass, and the body budget that
#: overrides it.
#:
#: WHY BATCHING IS BACK. The first attempt laid every cell on the grid and ran
#: PhysX once over all of them; it OOMed from cell 6 of 16 and its single
#: settle over 5,064 bodies ran 975 s, left 4,825 bodies moving and dropped
#: nothing. Two causes, and neither was "batching is wrong":
#:
#:   * the OOM was 26,888 VULKAN errors — the renderer holding every fractured
#:     building resident, not the solver holding bodies. `_hide_for_settle`
#:     takes the batch out of the renderer for the duration.
#:   * the stopping rule was a single max over every body in the scene, so one
#:     piece still tumbling anywhere held the whole batch to the ceiling.
#:     `settle.run(groups=...)` now judges each pile on its own.
#:
#: With those fixed a batch costs the SLOWEST pile rather than the SUM of them.
#: Measured on 15 cells of `house_02,house_03,old_brick_shop`: settle 83.6 s
#: one at a time, 67.8 s at eight, 43.5 s with all fifteen in one pass (1.9x),
#: byte-identical output at 330.4 MB and every cell converged. The saving is
#: the VARIANCE — a batch runs for its slowest pile, so it pays most where the
#: piles differ, and nothing at all where they are all the same length.
#:
#: EIGHT is conservative on purpose. The library measures at median 135 bodies
#: per archetype (p90 416, max 1195), so eight cells is ~1,100 and lands just
#: under the body budget below; twelve cells / 386 bodies was verified fine and
#: there is headroom above that. An unattended overnight bake pays for an OOM
#: in hours, which is why the default is not the largest number that worked.
#:
#: `SCENE_SETTLE_BATCH=1` restores the exact one-cell-at-a-time behaviour.
SETTLE_BATCH_CELLS = max(1, int(os.environ.get("SCENE_SETTLE_BATCH", "8") or 1))

#: Flush the batch once it holds this many loose fragments, whatever the cell
#: count. A single `office_tower_cracked` is 1,195 bodies and lands in a batch
#: of its own.
SETTLE_BATCH_BODIES = max(1, int(
    os.environ.get("SCENE_SETTLE_BATCH_BODIES", "1200") or 1))

#: Per-cell trace, one JSON object per line, appended as each cell finishes.
#:
#: SEPARATE FROM THE MANIFEST, and append-only, because the two answer
#: different questions. The manifest describes the library that EXISTS and is
#: rewritten whole; the trace is the record of what the bake DID, including the
#: cells that failed to build, blew their settle, or came out untextured —
#: which is most of what you want when the question is "why is this slow" or
#: "which assets can this pipeline not handle yet". Being append-only, it also
#: survives the bake being killed, which a manifest written at the end does not.
TRACE_NAME = "bake_trace.jsonl"

#: Photograph every rung as it is baked (`SCENE_ARCH_PREVIEW=0` to stop).
#:
#: ON BY DEFAULT, which is the whole point: an opt-in review is one nobody
#: runs, and the bake is the only moment the wreck is already on a lit stage
#: with its debris settled around it. MEASURED at 0.3 s a cell (`t_preview_s`
#: on the record, 2026-08-29 smoke bake) against a 34 s median — two captures
#: at 24 converged frames each, on a renderer already warm from the cell
#: before. It is the only record of what a library looks like that survives
#: the library being too big to open.
PREVIEW = os.environ.get("SCENE_ARCH_PREVIEW", "1").strip() not in ("0", "")


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
    e = GROUND_HALF_M
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


def _cell_xy(idx: int, ncol: int, y0: float = 0.0, spacing: float = None) -> tuple:
    """Grid position for cell *idx*. *spacing* defaults to `GRID_M`.

    IT IS A PARAMETER BECAUSE 40 m DOES NOT FIT HALF THIS PACK. `GRID_M` has
    to exceed the asset's footprint or two cells settle into each other, and 45
    of the 89 assets in `urban_v2` are 32 m or wider — `SM_MERGED_BP_MBuilding02`
    is 96 m. That was harmless while cells were built and unloaded one at a
    time; batching makes several resident at once, so the batch picks a spacing
    that fits what is in it (`Baker._spacing_for`).

    Absolute position carries no meaning downstream — `_export_cell` re-centres
    every cell on its own contents — so a wider spacing costs nothing but
    ground, and `prepare_stage` lays down 1600 m of it.
    """
    step = float(GRID_M if spacing is None else spacing)
    return (idx % ncol) * step, y0 + (idx // ncol) * step


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
        #: cell -> the arguments `_shed` needs, once its wreck has settled.
        self.pending_shed = {}
        self.records = []
        #: Anything the cell being built wants written onto its manifest
        #: record. Reset per cell in `build`; merged in `_export_cell`.
        self.cell_record: dict = {}
        self._missing = 0
        #: Source-asset measurements, per type. The same USD is referenced
        #: once per rung and reading every points array of a 300k-point tower
        #: is not free, so it is measured on the first rung and reused.
        self._src_cache: dict = {}
        #: Which assets the pack declares already solid, and what each is made
        #: of — both are walks of the whole `usds:` tree, so they are done once
        #: here rather than per cell.
        self._solid = self._declared_solid()
        #: Cell spacing the CURRENT batch was laid out at, and how many cells
        #: have been placed in it. Reset on every flush — see `_spacing_for`.
        self._batch_spacing = float(GRID_M)
        self._batch_i = 0
        self._started_at = datetime.datetime.now().astimezone().isoformat(
            timespec="seconds")
        #: type slug -> {config: count}, from `set_census`. What lets a
        #: library say which of its archetypes a real scene depends on and
        #: which are speculative stock.
        self._used_by: dict = {}
        self._census_meta: dict = {}

    def set_census(self, doc: dict) -> int:
        """Mark the types *doc* saw placed, so records carry `used_by`.

        Additive across censuses: one library serves several scenes, and a
        second census must not un-mark what the first found.
        """
        assets = (doc or {}).get("assets") or []
        if not assets:
            return 0
        name = str(doc.get("config") or "scene")
        for a in assets:
            self._used_by.setdefault(str(a.get("type")), {})[name] = \
                int(a.get("count") or 0)
        self._census_meta[name] = {
            "seed": doc.get("seed"), "region_m": doc.get("region_m"),
            "asset_pack": doc.get("asset_pack"),
            "recorded_at": doc.get("recorded_at"),
            "types": len(assets),
        }
        return len(assets)

    # -- build ------------------------------------------------------------
    def build(self, items: list, ssf: float, skip_existing: bool = False) -> int:
        """Lay every (type, level) out on the grid and damage it in place.

        *skip_existing* leaves alone any cell whose USD is already on disk and
        already in the manifest. A bake has no other resume: changing one knob
        and restarting re-cuts all 86 cells from scratch, which is 90 minutes,
        and most of it is work that the knob does not affect — `pristine` is
        the source asset re-exported and comes out byte-identical every time.
        Baking the 500 m library, that cost four passes over the same cells.
        """
        from pxr import UsdGeom, Sdf

        combos = [(it, lv) for it in items for lv in it.levels]
        if skip_existing:
            have = self._baked_records()
            keep = []
            for it, lv in combos:
                name = lib.archetype_name(it.type, lv) + ".usd"
                if name in have and os.path.exists(
                        os.path.join(lib.disaster_dir(_SCENE_GEN, self.disaster,
                                                      self.out_dir), name)):
                    # CARRY THE SKIPPED CELL'S RECORD FORWARD. `merge_manifest`
                    # keeps what is already in the manifest, so a cell the
                    # manifest knows needs nothing here — but a cell known only
                    # to the TRACE (baked by a run that was killed before it
                    # wrote a manifest) would be skipped AND left out, and the
                    # archetype would sit on disk invisible to Stage B and to
                    # every gallery. Measured: `office_tower` baked two rungs,
                    # 1.5 GB, and appeared in no manifest at all.
                    if have[name] is not None:
                        self.records.append(have[name])
                    continue
                keep.append((it, lv))
            if len(keep) < len(combos):
                print(f"[stage-a] resume: {len(combos) - len(keep)} cell(s) "
                      f"already baked, {len(keep)} to do", flush=True)
            combos = keep
        ncol = _grid_columns(len(combos))
        print(f"[stage-a] {len(combos)} cells on a {ncol}-column grid"
              + (f", settling up to {SETTLE_BATCH_CELLS} at a time"
                 if SETTLE_BATCH_CELLS > 1 else ""))
        pending = []

        for idx, (it, level) in enumerate(combos):
            free_gb = self._free_gb()
            if free_gb < DISK_FLOOR_GB:
                print(f"[stage-a] STOPPING EARLY: {free_gb:.1f} GB free, "
                      f"floor is {DISK_FLOOR_GB:.0f} GB. "
                      f"{len(combos) - idx} cell(s) not baked. What is on "
                      f"disk is complete and in the manifest.", flush=True)
                # WHAT IS BUILT STILL GETS SETTLED. Breaking with a full queue
                # would throw away cells that are already paid for.
                self._flush(pending)
                pending = []
                break
            # THE BATCH IS LAID OUT AT A SPACING THAT FITS IT. A cell needing
            # more room than the cells already queued were placed with cannot
            # simply be put further out — the ones already there would still be
            # too close to each other's neighbour — so the batch is flushed and
            # the next one starts at the wider spacing.
            need = self._spacing_for(it)
            if pending and need > self._batch_spacing:
                self._flush(pending)
                pending = []
            if not pending:
                self._batch_spacing = need
                self._batch_i = 0
            x, y = _cell_xy(self._batch_i, ncol, spacing=self._batch_spacing)
            self._batch_i += 1
            cell = f"{self.parent}/a_{it.kind[:3]}_{it.type}_{level}"
            UsdGeom.Scope.Define(self.stage, Sdf.Path(cell))
            # Per-cell progress. A 250-archetype bake runs for hours; without
            # this there is no way to tell a slow cell from a hung one, and no
            # way to name the cell that killed the process when one does.
            t_cell = time.time()
            print(f"[stage-a] {idx + 1}/{len(combos)} {it.type}_{level} "
                  f"({it.build}/{it.kind})", flush=True)
            n_loose, n_static = len(self.loose), len(self.static)
            self.cell_record = {}
            try:
                paths, extra = self._build_one(it, level, x, y, cell, ssf)
            except Exception as exc:                            # noqa: BLE001
                # One bad asset must not cost the other 200 archetypes — but
                # PRINT THE TRACEBACK. The message alone is not enough to act
                # on: "object of type 'bool' has no len()" was every AEC
                # brownstone failing every damaged rung, 32 archetypes, and
                # nothing in the log said which of the pipeline's several
                # thousand lines produced it.
                tb = traceback.format_exc()
                print(f"[stage-a] SKIP {it.type}_{level}: "
                      f"{type(exc).__name__}: {exc}\n{tb}", flush=True)
                self._unload(cell)
                self._trace(it, level, "build_failed", time.time() - t_cell,
                            note=f"{type(exc).__name__}: {exc}",
                            traceback_=tb)
                continue
            t_built = time.time()
            self.cells.append((it, level, x, y, paths, extra))
            # ON THE RECORD, so the manifest reads as a table of what each
            # asset cost. The split is the useful part: a cell that spends
            # 20 s building and 400 s settling is a different problem from one
            # that spends 400 s in the fracture.
            self.cell_record["t_build_s"] = round(t_built - t_cell, 1)
            # QUEUED, NOT SETTLED YET. Its loose pieces and its own standing
            # geometry ride along so the batch can address them per cell.
            pending.append({
                "item": it, "level": level, "x": x, "y": y, "cell": cell,
                "paths": paths, "extra": extra,
                "loose": self.loose[n_loose:], "static": self.static[n_static:],
                "t_cell": t_cell, "record": dict(self.cell_record),
            })
            if self._batch_full(pending):
                self._flush(pending)
                pending = []
        self._flush(pending)
        return len(self.cells)

    # -- batching ---------------------------------------------------------
    def _spacing_for(self, item) -> float:
        """Metres this type needs between cells: footprint plus debris throw.

        Read from the source measurements `_source_stats` caches on the first
        rung of a type. That is always `pristine`, which contributes no bodies
        to a settle, so by the time a cell can actually collide with a
        neighbour the size is known — the unmeasured case is only the modular
        kit, whose houses are well inside `GRID_M`.
        """
        src = self._src_cache.get(item.type)
        if not src:
            return float(GRID_M)
        span = max(float(src.get("src_x_m") or 0.0),
                   float(src.get("src_y_m") or 0.0))
        # 1.6x leaves room for what the wreck throws; `debris.shed` scatters
        # to roughly the building's own footprint again.
        return max(float(GRID_M), span * 1.6)

    def _batch_full(self, pending) -> bool:
        """Is the queue ready to settle? Cells first, bodies as the override.

        The third cap is the GROUND: `prepare_stage` lays 1600 m of plane, and
        a batch spread wider than that would drop its far cells through the
        edge of the world.
        """
        if len(pending) >= SETTLE_BATCH_CELLS:
            return True
        ncol = _grid_columns(max(len(pending), 1))
        if (ncol + 1) * self._batch_spacing > GROUND_HALF_M:
            return True
        return sum(len(p["loose"]) for p in pending) >= SETTLE_BATCH_BODIES

    def _flush(self, pending) -> None:
        """Settle every queued cell in one pass, then judge and export each.

        A batch is one PhysX solve over piles that are `GRID_M` apart and so
        cannot touch each other; `settle.run(groups=...)` returns a verdict per
        pile, which is what keeps the convergence gate a per-archetype decision
        rather than a per-batch one.
        """
        if not pending:
            return
        t0 = time.time()
        stats_by = self._settle_batch(pending)
        self._batch_i = 0
        # AMORTISED, so the per-cell numbers still sum to the wall clock. A
        # batch is one solve; charging all of it to each of its cells would
        # make the report claim several times the time that actually passed.
        share = round((time.time() - t0) / len(pending), 1)

        for pd in pending:
            it, level, cell = pd["item"], pd["level"], pd["cell"]
            stats = stats_by.get(cell)
            self.cell_record = dict(pd["record"])
            self.cell_record["t_settle_s"] = share
            if len(pending) > 1:
                self.cell_record["settle_batch_n"] = len(pending)
            t_exp = time.time()
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
                outcome, rec = "rejected_settle", None
            elif self._export_cell(it, level, pd["x"], pd["y"], pd["paths"],
                                   pd["extra"], stats):
                outcome, rec = "baked", self.records[-1]
                rec["t_export_s"] = round(time.time() - t_exp, 1)
                t_pv = time.time()
                shots = self._preview_cell(it, level, pending, pd)
                if shots:
                    rec["preview"] = shots
                    rec["t_preview_s"] = round(time.time() - t_pv, 1)
            else:
                outcome, rec = "rejected_export", None
            self._unload(cell)
            dt = (self.cell_record["t_build_s"] + share
                  + float((rec or {}).get("t_export_s", 0.0)))
            if rec is not None:
                rec["seconds"] = round(dt, 1)
                # PER CELL, not once at the end. A library whose manifest
                # appears only when the last cell lands cannot be resumed,
                # cannot be read by Stage B, and cannot be rendered while it
                # fills — and an unattended bake is stopped by the disk or the
                # morning far more often than it runs to the end.
                self.write_manifest()
            self._trace(it, level, outcome, dt, record=rec, settle=stats,
                        cell=dict(self.cell_record))
            if dt > 5.0:
                print(f"[stage-a]     {dt:.0f}s", flush=True)

    # -- instrumentation --------------------------------------------------
    def _out(self) -> str:
        return lib.disaster_dir(_SCENE_GEN, self.disaster, self.out_dir)

    def _free_gb(self) -> float:
        """Free space on the filesystem the library is being written to."""
        try:
            out = self._out()
            os.makedirs(out, exist_ok=True)
            st = os.statvfs(out)
            return st.f_bavail * st.f_frsize / 1e9
        except Exception:                                       # noqa: BLE001
            return float("inf")

    def _trace(self, item, level, outcome, seconds, record=None, settle=None,
               cell=None, note="", traceback_=""):
        """Append one line to the trace. Never raises — see `TRACE_NAME`."""
        row = {
            "type": item.type, "level": level, "kind": item.kind,
            "build": item.build, "source": str(item.source),
            "outcome": outcome, "seconds": round(float(seconds), 1),
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "free_gb": round(self._free_gb(), 1),
        }
        for extra in (cell or {}), (record or {}):
            for k, v in extra.items():
                if k not in ("type", "level", "kind", "build", "source",
                             "seconds", "settle"):
                    row[k] = v
        if settle:
            row["settle"] = settle
        if note:
            row["note"] = note
        if traceback_:
            row["traceback"] = traceback_
        try:
            out = self._out()
            os.makedirs(out, exist_ok=True)
            with open(os.path.join(out, TRACE_NAME), "a") as fh:
                fh.write(json.dumps(row, sort_keys=True) + "\n")
        except Exception:                                       # noqa: BLE001
            pass

    def _declared_solid(self) -> set:
        """`scene_generator.solid_assets`, or empty if the config lacks a pack."""
        try:
            from scene_generator import solid_assets
            return set(solid_assets(self.config))
        except Exception:                                       # noqa: BLE001
            return set()

    def _source_stats(self, item, prim_path) -> dict:
        """Measure the INTACT asset: what the bake was handed, before it cuts.

        Meshes and points are the cost proxy — fracture time tracks them far
        more closely than it tracks the building's size — and `hollow` is the
        knob that decides whether `solidify` runs at all, which is the single
        biggest lever on both the time and the output size. All three belong
        beside the outcome, or a table of bake times explains nothing.
        """
        got = self._src_cache.get(item.type)
        if got is None:
            from pxr import Usd, UsdGeom
            meshes = points = 0
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                for p in Usd.PrimRange(prim):
                    if not p.IsA(UsdGeom.Mesh):
                        continue
                    meshes += 1
                    pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
                    points += len(pts) if pts else 0
            (sx, sy), sz = self._envelope(prim_path)
            got = {"src_meshes": meshes, "src_points": points,
                   "src_x_m": round(sx, 1), "src_y_m": round(sy, 1),
                   "src_z_m": round(sz, 1),
                   # DECLARED, not detected — see `solid_assets`. `hollow` is
                   # the negation because that is the question a reader of the
                   # table asks ("is this a paper shell?").
                   "hollow": str(item.source) not in self._solid}
            self._src_cache[item.type] = got
        self.cell_record.update(got)
        return got

    def _existing_records(self) -> list:
        """Records from the manifest already on disk, or empty."""
        path = os.path.join(
            lib.disaster_dir(_SCENE_GEN, self.disaster, self.out_dir),
            "manifest.json")
        try:
            data = json.load(open(path))
        except Exception:                                       # noqa: BLE001
            return []
        return data.get("archetypes", data) if isinstance(data, dict) else data

    def _baked_records(self) -> dict:
        """``{filename: manifest record or None}`` for every archetype known good.

        THE TRACE IS WHY THIS IS NOT JUST THE MANIFEST. An overnight bake is
        cut short by something more often than it finishes, and the manifest
        is only durable because `_export_cell` now writes it per cell; a
        library baked before that, or killed mid-cell, has none. The trace is
        appended as each cell lands and so always describes the run that died,
        which makes it the thing `--skip-existing` can actually resume from.
        Both are cross-checked against the file, since neither survives someone
        deleting a USD to reclaim space.
        """
        # None means "the manifest already has it", so nothing to re-add.
        out = {r.get("usd"): None for r in self._existing_records()
               if r.get("usd")}
        path = os.path.join(self._out(), TRACE_NAME)
        try:
            for line in open(path):
                row = json.loads(line)
                if row.get("outcome") != "baked" or not row.get("usd"):
                    continue
                if row["usd"] not in out:
                    out[row["usd"]] = self._record_from_trace(row)
        except Exception:                                       # noqa: BLE001
            pass
        return out

    @staticmethod
    def _record_from_trace(row: dict) -> dict:
        """A manifest record rebuilt from a trace line.

        The trace is a superset of a record — `_trace` copies the record onto
        it — so this is a projection, not a reconstruction. Only the keys
        `library.Library` and Stage B read are kept, plus the measurements a
        report wants; the trace's own bookkeeping (`outcome`, `free_gb`,
        `finished_at`) does not belong in a manifest.
        """
        keep = ("type", "level", "kind", "build", "source", "usd", "meshes",
                "bound_missing", "usd_mb", "seconds", "t_build_s",
                "t_settle_s", "t_export_s", "debris_r_m", "material",
                "frag_cells", "frag_loose", "frag_anchored", "thickened",
                "src_meshes", "src_points", "src_x_m", "src_y_m", "src_z_m",
                "hollow", "mechanisms")
        rec = {k: row[k] for k in keep if k in row}
        rec["settle"] = row.get("settle") or {"bodies": 0, "converged": True}
        return rec

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
        # NO SHED DEBRIS HERE, unlike `_build_library`. `damage_flow` returns a
        # fragment list rather than a report, so `debris.fallen` has nothing to
        # read — and a kit house breaks into its own wall panels, which is
        # already a debris field of the right material. Wire it up when
        # `damage_flow` grows a report worth reading.
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
        # BEFORE THE PRISTINE RETURN, so every rung of a type carries the same
        # source measurements and the `pristine` row can be read as the
        # baseline the other four are compared against.
        src = self._source_stats(item, prim_path)
        envelope = (src["src_x_m"], src["src_y_m"]), src["src_z_m"]
        if level == "pristine":
            return [prim_path], []

        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return [prim_path], []
        seed = self.seed + md.stable_seed(level)
        got = self.model.damage_archetype(
            self.stage, prim, level, seed=seed, config=self.config,
            intensity=self._intensity(item.kind, level),
            # WHICH ASSET this archetype is, so Stage A can look up what it is
            # built of. Without it every archetype was cut as the locale's one
            # material — masonry for a downtown block of concrete, steel and
            # glass towers.
            source=str(item.source))
        frags = list(got.get("loose", ()))
        self.loose.extend(frags)
        # WHAT THE DAMAGE ACTUALLY DID, on the record. `cells` is the Voronoi
        # cut count, `loose` how many of those came free, `thickened` how many
        # meshes solidify had to give walls to; `material` is the lookup
        # `damage_archetype` already printed. Together they are why one cell
        # took 40 s and its neighbour 600.
        self.cell_record.update({
            "material": str(got.get("material") or ""),
            "frag_cells": int(got.get("cells") or 0),
            "frag_loose": len(frags),
            "frag_anchored": max(0, len(got.get("paths") or ()) - len(frags)),
            "thickened": int(got.get("thickened") or 0),
            "mechanisms": list(got.get("mechanisms") or ()),
        })
        # WHAT IT SHED, BAKED IN — but NOT YET. The rubble a wreck leaves is
        # part of the wreck and belongs in the cell, so a scene referencing
        # this archetype gets it for free. What changed is WHEN: debris used to
        # be shed here, from the intact building's envelope, and settled in the
        # same pass as the fragments.
        #
        # It cannot be placed until the wreck has fallen. `debris.plan` lines
        # the scatter along the building's outline, and once a tower pancakes
        # that outline is nowhere near where the rubble ended up — a 50x40 m
        # plan settles to 42x36 m and the ring of debris sits outside the pile
        # entirely. So the inputs are parked here and `_settle_batch` sheds
        # after its first phase, against the SETTLED footprint.
        self.pending_shed[cell] = (item, level, envelope, x, y, cell,
                                   got.get("material") or "", got)
        return ([prim_path] + list(got.get("paths", ())), [])

    def _envelope(self, prim_path):
        """World AABB of the intact asset: ``((sx, sy), sz)``, metres.

        Measured BEFORE the damage runs. Afterwards the bounds are the bounds
        of the thrown pieces, which on a collapse is most of the cell.
        """
        from pxr import Usd, UsdGeom

        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return (0.0, 0.0), 0.0
        r = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        ).ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            return (0.0, 0.0), 0.0
        sz = r.GetSize()
        return (float(sz[0]), float(sz[1])), float(sz[2])

    def _shed(self, item, level, envelope, x, y, cell, kind, report):
        """Debris for one cell. Records its reach on the manifest record.

        NO REGION AND NO EXCLUSIONS, unlike the live path: a cell is not a
        place, it is a stand-in for the asset wherever it ends up standing. The
        bound that matters here is the export, and `_export_cell` re-centres
        the whole cell on its own contents.
        """
        from disaster import debris as D

        (sx, sy), sz = envelope
        if sz <= 0.0:
            return {"placements": [], "paths": [], "statics": [],
                    "radius_m": 0.0}
        fp, ctr = D.settled_footprint(
            self.stage, report.get("paths") or [], ((sx, sy), (x, y)))
        out = D.shed(self.stage, self.config, kind=kind,
                     # SIZE x RUNG — see `debris.budget_m3`. The footprint is
                     # the SETTLED one, so the scatter lines the rubble rather
                     # than the plan the building used to occupy.
                     rung=level, centre=ctr,
                     footprint_m=fp, height_m=sz,
                     # INSIDE the cell, so `_unload` takes it off the stage
                     # with everything else the cell built. The same place
                     # `_build_vegetation` puts a felled tree's debris.
                     parent_path=f"{cell}/debris",
                     rng=self._rng(item, level))
        # THE REACH IS RECORDED BY THE CALLER, not here. `_shed` now runs
        # during `_settle_batch`, long after `self.cell_record` was snapshotted
        # into the pending entry — writing it here would put it on a record
        # that has already been copied and is about to be overwritten by that
        # copy. See the caller, which lands it on `pd["record"]`.
        return out

    def _build_vegetation(self, item, level, x, y, cell, ssf):
        """Tree: reference it, then burn/break it on the instancer."""
        from disaster import vegetation as veg

        prim_path = f"{cell}/tree"
        # The pack's scale, not a hardcoded 0.01 — that constant was right for
        # the AEC vegetation the original bake script had baked in and wrong
        # for anything else.
        self._reference(prim_path, item.source, x, y, scale=item.scale,
                        axis_up=item.axis_up)
        self._source_stats(item, prim_path)
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
    def _settle_batch(self, pending) -> dict:
        """PhysX over a whole batch at once. ``{cell path: stats or None}``.

        The cells are `GRID_M` apart and cannot reach each other, so one solve
        over all of them is the same physics as one solve each — what changes
        is that the batch costs the SLOWEST pile instead of the sum of them,
        and that PhysX pays its start-up once instead of once per archetype.

        `groups=` is what makes that safe to judge: without it the stopping
        rule is a single max over every body in the scene, which is how the
        first attempt at this left 4,825 of 5,064 bodies moving and dropped
        nothing.
        """
        from disaster import settle as S

        loose_all, static_all, groups = [], [], {}
        for pd in pending:
            if not pd["loose"]:
                continue                       # a pristine cell settles trivially
            groups[pd["cell"]] = list(pd["loose"])
            loose_all.extend(pd["loose"])
            static_all.extend(pd["static"])
            # The cell's own standing geometry is what its rubble lands on.
            for path in pd["paths"]:
                prim = self.stage.GetPrimAtPath(path)
                if prim and prim.IsValid() and prim.IsActive():
                    static_all.append(path)
        if not loose_all:
            return {}

        hidden = self._hide_for_settle(pending) if len(groups) > 1 else []
        self._pump(5)
        if len(groups) > 1:
            print(f"[stage-a]     settling {len(loose_all)} loose fragment(s) "
                  f"across {len(groups)} cell(s)", flush=True)
        else:
            print(f"[stage-a]     settling {len(loose_all)} loose fragment(s)",
                  flush=True)
        info = S.run(self.stage, loose_all, static_all, steps=SETTLE_STEPS,
                     kick=0.15, rng=random.Random(self.seed), bake_result=True,
                     groups=groups, dt=SETTLE_DT)
        self._pump(5)

        # --- PHASE TWO: the debris, onto the wreck that just landed ---------
        # Everything above is the building coming down. Only now is there a
        # settled footprint to scatter along, and only now are the fragments
        # baked and therefore static scenery for the pieces to land on.
        from disaster import debris as D

        deb_loose, deb_static, deb_groups = [], [], {}
        for pd in pending:
            args = self.pending_shed.pop(pd["cell"], None)
            if args is None:
                continue
            shed = self._shed(*args)
            pd["extra"] = [q["prim_path"] for q in shed["placements"]]
            # SO A PLACED ARCHETYPE STILL KNOWS ITS GROUND IS COVERED. Once the
            # debris is inside the USD it is no longer a placement of its own,
            # and `targets` samples casualties against the placement list —
            # which is how the first debris-aware run put a victim inside a
            # rubble pile.
            if shed["radius_m"] > 0.0:
                pd["record"]["debris_r_m"] = round(shed["radius_m"], 2)
            if shed["paths"]:
                deb_groups[pd["cell"]] = list(shed["paths"])
                deb_loose.extend(shed["paths"])
            # THE MOUNDS GET THE SHRUNKEN COLLIDER, so a piece landing on one
            # beds into it instead of perching on its silhouette. The visible
            # mound carries no collision at all — see `debris.pile_colliders`.
            deb_static.extend(D.pile_colliders(self.stage, shed["statics"],
                                               PILE_COLLIDER))
            deb_static.extend(pd["paths"])
        if deb_loose:
            print(f"[stage-a]     settling {len(deb_loose)} debris piece(s) "
                  f"across {len(deb_groups)} cell(s)", flush=True)
            S.run(self.stage, deb_loose, deb_static, steps=SETTLE_STEPS,
                  kick=0.15, rng=random.Random(self.seed + 1),
                  bake_result=True, groups=deb_groups, dt=SETTLE_DT)
            self._pump(5)
        self._show_after_settle(hidden)

        per = info.get("groups") or {}
        # DROP WHAT SANK, THEN JUDGE. See `_drop_sunk`.
        sunk = {key: self._drop_sunk(paths) for key, paths in groups.items()}
        out = {}
        for key, paths in groups.items():
            keep = [p for p in paths if p not in sunk[key]]
            # `_stats_for` recounts `through_floor` over what is LEFT, so the
            # dropped pieces simply are not there any more. `still_moving` is
            # deliberately NOT adjusted: the settle's count stands as measured,
            # and it already passes the `max(4, 5%)` tolerance in every case
            # this was rejecting (it was always 1). Fudging it down by the
            # number dropped would mask a cell that genuinely never settled.
            st = self._stats_for(per.get(key) or info, keep)
            if sunk[key]:
                st["sunk_dropped"] = len(sunk[key])
            out[key] = st
        return out

    def _drop_sunk(self, loose) -> set:
        """Remove fragments that ended up under the floor. Returns their paths.

        WHY REMOVE RATHER THAN REJECT. The gate holds `through_floor` to zero
        absolutely, on the reasoning that geometry through the floor ruins a
        baked wreck. That is true of a collapse that fell through the world;
        it is not true of ONE piece out of hundreds. Measured on this pack,
        2026-08-30: four rejected cells, every one of them `1 still moving,
        1 through the floor` against 144-455 bodies — 0.2-0.7% of the wreck
        costing the whole archetype, which Stage B then replaces with a rung
        further down the ladder, so a partial collapse renders as merely
        cracked. 13% of structure cells went that way.

        The offender is a fragment that tunnels through the ground plane
        early (`SETTLE_DT` of 1/20 is, per QUAKE_STATE, the conservative end
        of where that starts) and then free-falls for the whole step budget —
        which is why it shows up as the still-moving body too.

        A fragment below the floor is INVISIBLE and contributes nothing but
        file size, so deleting it loses nothing a scene could see and keeps
        the several hundred pieces that settled correctly. What the gate is
        really for — a pile that never fell, or a wreck that left through the
        floor wholesale — still trips it, because dropping thirty pieces of a
        forty-piece wreck leaves a cell with nothing in it and `export_object`
        reports it as empty.
        """
        from pxr import Sdf

        gone = set()
        for p in loose or ():
            if _centroid_z(self.stage, p) >= -2.0:
                continue
            gone.add(p)
            prim = self.stage.GetPrimAtPath(Sdf.Path(str(p)))
            if prim and prim.IsValid():
                prim.SetActive(False)
                self.stage.RemovePrim(Sdf.Path(str(p)))
        if gone:
            print(f"[stage-a]     dropped {len(gone)} fragment(s) that sank "
                  f"below the floor", flush=True)
        return gone

    def _stats_for(self, got: dict, loose) -> dict:
        """One cell's verdict, in the shape the manifest and the gate expect."""
        # Through the floor is judged at the piece's CENTROID: a fragment is
        # authored with no xformOp, so its prim origin is the building's and
        # follows the piece down — a 40 m drop reads as 40 m underground.
        lost = sum(1 for p in loose if _centroid_z(self.stage, p) < -2.0)
        stats = {"bodies": int(got.get("bodies", len(loose))),
                 "still_moving": int(got.get("still_moving", 0)),
                 "through_floor": int(lost),
                 "steps_used": int(got.get("steps_used", 0)),
                 "steps": int(SETTLE_STEPS),
                 "drop_median_m": round(float(got.get("drop_median", 0.0)), 2),
                 "spread_max_m": round(float(got.get("spread_max", 0.0)), 1)}
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

    def _hide_for_settle(self, pending) -> list:
        """Take the batch out of the RENDERER while it settles.

        The first batched settle died on 26,888 Vulkan out-of-memory errors,
        not on anything PhysX ran out of: headless Kit still runs a render
        pipeline, and several fractured buildings resident at once is a great
        deal of geometry for it to hold. Physics does not read visibility, so
        hiding the cells changes what the solver does not at all.

        Returns what to put back, including whether the attribute was authored
        at all, so restoring leaves the stage exactly as it was found.
        """
        from pxr import UsdGeom

        saved = []
        for pd in pending:
            prim = self.stage.GetPrimAtPath(pd["cell"])
            if not prim or not prim.IsValid():
                continue
            attr = UsdGeom.Imageable(prim).GetVisibilityAttr()
            saved.append((prim, attr.Get() if attr and attr.HasAuthoredValue()
                          else None))
            UsdGeom.Imageable(prim).MakeInvisible()
        return saved

    def _show_after_settle(self, saved) -> None:
        """Undo `_hide_for_settle` before anything measures or exports."""
        from pxr import UsdGeom

        for prim, was in saved:
            if not prim or not prim.IsValid():
                continue
            attr = UsdGeom.Imageable(prim).GetVisibilityAttr()
            if was is None:
                attr.Clear()
            else:
                attr.Set(was)

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
                # THE NUMBER THAT PRICES A LIBRARY. A damaged archetype runs
                # to a few hundred MB and a scene references dozens, so this
                # is what decides whether a pack fits on the disk at all.
                "usd_mb": round(os.path.getsize(dst) / 1e6, 1),
                # How the settle ended, so a library says whether its wrecks
                # are piles (see the convergence gate in `build`).
                "settle": settle or {"bodies": 0, "converged": True},
                # WHEN, AND BY WHAT. An archetype is a cooked artifact and
                # carries no trace of the code that cut it, so a library
                # baked before a fix and one baked after are the same shape
                # on disk and behave differently in a scene. See
                # `archetypes/version.py`.
                **V.stamp(_SCENE_GEN),
                # WHICH CLIPPER CUT THIS. The urban_v3 library deliberately
                # holds both: everything through record 74 was cut by the
                # numpy clipper and is KEPT (user direction, 2026-08-30 — it
                # looks right and a re-bake is hours), everything after by
                # VTK. Without this the mixture is invisible and the only
                # signal is a fingerprint that says "stale", which is the
                # wrong reading.
                "fracture_backend": _md.active_backend(),
                # Which scenes place this type, and how many of each. Empty
                # means "no census has seen it", NOT "unused" — a library
                # baked without one says nothing either way.
                **({"used_by": self._used_by[item.type]}
                   if item.type in self._used_by else {}),
                **self.cell_record,
            })
        return True

    def _preview_cell(self, item, level, pending, pd) -> dict:
        """Photograph the settled cell, and say where the pictures went.

        Between the export and the unload, which is the only window: the
        export is what decides the cell is worth keeping, and the unload is
        what takes the geometry off the stage.

        THE REST OF THE BATCH IS HIDDEN FOR THE DURATION. Cells are `GRID_M`
        apart so they cannot touch, but a camera framed to fit a 40 m building
        stands ~120 m back and the neighbours are well inside that frame — a
        preview of one archetype with three others behind it is not a preview
        of anything. The same visibility save/restore the settle uses, applied
        to everything EXCEPT the subject.
        """
        if not PREVIEW:
            return {}
        out = self._out()
        stem = os.path.join(PV.preview_dir(out),
                            lib.archetype_name(item.type, level))
        others = [q for q in pending if q["cell"] != pd["cell"]]
        hidden = self._hide_for_settle(others) if others else []
        try:
            got = PV.capture_cell(self.stage,
                                  list(pd["paths"]) + list(pd["extra"]), stem)
        finally:
            self._show_after_settle(hidden)
        # RELATIVE TO THE MANIFEST, for the reason `usd` is: a library is
        # self-describing and moves as a directory.
        return {k: os.path.relpath(v, out) for k, v in got.items() if v}

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
             "grid_m": GRID_M,
             # The pipeline THIS bake ran, at the top level as well as on
             # every record. The records say what each archetype is; this
             # says what the library was last touched by, which is the
             # question `version.audit` answers in one read.
             "pipeline_version": V.PIPELINE_VERSION,
             "pipeline_fingerprint": V.source_fingerprint(_SCENE_GEN),
             "last_bake_at": self._started_at,
             # Which scenes are known to place these types, and how many of
             # each — see `archetypes/census.py`. Carried on the manifest
             # rather than worked out on demand because the answer needs a
             # scene built under Kit with Nucleus reachable, which a reader
             # of the library generally is not.
             "census": self._census_meta})


def _trim_to_cells(items, cells):
    """Restrict each item's ladder to *cells*, a set of ``(type, level)``.

    `run` REBUILDS the plan from the config and then filters it by `only`,
    which carries `(type, kind)` and knows nothing about levels. So a level
    restriction applied earlier -- in `bake_cli._select`, where `--dry-run`
    reads it -- was silently discarded by the time anything was cut: asking for
    8 specific rungs across 6 types re-cut all 6 ladders, 30 cells, and the
    dry run cheerfully reported 8. Measured 2026-08-30 from the gallery
    picker's Rebake button.
    """
    if not cells:
        return items
    out = []
    for it in items:
        keep = [lv for lv in it.levels if (it.type, lv) in cells]
        if not keep:
            continue
        it.levels = keep
        out.append(it)
    return out


def run(stage, config: dict, disaster: str, out_dir: str = "",
        seed: int = 7, parent: str = "/World/stage/generated",
        ssf: float = 1.0, only=None, skip_existing: bool = False,
        census: dict = None, cells=None) -> dict:
    """Bake one disaster's archetype library. Returns a summary dict.

    *only* restricts the bake to a set of ``(type, kind)`` pairs. A full bake
    runs for hours (measured: ~40 s per library archetype), so being able to
    re-bake the handful that failed — or try one before committing to all of
    them — is the difference between iterating and starting over.

    *census* is a document from `archetypes.census`: the types a real scene
    was measured placing. It marks `used_by` on every record it recognises, so
    the library says which of its archetypes a scene depends on.
    """
    dtype = str(disaster or (config.get("disaster") or {}).get("type")
                or "none").lower()
    items = P.build_plan(config, dtype)
    if only:
        # ORDER-PRESERVING when the caller passed a sequence. The plan's own
        # order is the pack's, which says nothing about what a cell costs;
        # `tools/bake_order.py` sorts by measured volume so that a bake cut
        # short has finished the most assets it could, and that intent is lost
        # if this reduces the selection to a set.
        want = list(dict.fromkeys(only))
        rank = {k: i for i, k in enumerate(want)}
        items = [i for i in items if (i.type, i.kind) in rank]
        items.sort(key=lambda i: rank[(i.type, i.kind)])
    items = _trim_to_cells(items, cells)
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
    if census:
        n = baker.set_census(census)
        print(f"[stage-a] census: {n} type(s) marked used by "
              f"'{census.get('config', 'scene')}'")
    baker.build(items, ssf, skip_existing=skip_existing)
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
