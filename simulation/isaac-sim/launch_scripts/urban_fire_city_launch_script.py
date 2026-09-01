#!/usr/bin/env python
"""
urban_fire_city — the 500 m downtown, built intact, with its BURNT buildings
swapped in from the per-building fire bakes and the flames and smoke put back
on top. Work item #7 of `scene_gen/_plans/urban_fire_city_plan.md`.

    ISAAC_SIM_HEADLESS=true SCENE_CONFIG=downtown_fire_500 \\
    FC_MANIFEST=/isaac-sim/AirStack/scene_gen/_plans/fire_city_4417.json \\
    FC_BAKES=/isaac-sim/.cache/fire_bakes/city_4417 \\
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_city \\
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \\
    /isaac-sim/python.sh \\
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py \\
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window

`downtown_quake_launch_script.py` for the city half (Pegasus environment ->
`generate_scene_on_stage` -> post-process), `fire_assembly_launch_script.py`
for the fire half (`place_fire` on a bake, the fire-facing review cameras,
`vram_mb` at every stage) — the shared half of the latter now lives in
`scene_gen/disaster/fire_assembly_lib.py`, because THIS file cannot import
that launcher: it builds a `SimulationApp` at import and a second Kit app in
one process is a segfault inside the first second (measured, and documented
in `downtown_quake_launch_script.py`).

THE TRANSFORM TRAP (plan sec 4a) — the one thing to get right here
------------------------------------------------------------------
`quake.assemble`'s idiom is "keep the placement's transform, swap the
reference" (`quake.py:~676`: `refs.ClearReferences(); refs.AddReference(...)`).
That is WRONG for a fire bake, and it is wrong silently.
`scene_generator.apply_placements` authors, for the INTACT asset:

    translate = (x_m - rotated_centroid_offset) * ssf
    rotateXYZ = (roll, pitch, yaw)        # roll 90 deg for a Y-up asset
    scale     = p["scale"]                # 0.01 for a GAC asset

— every part of which is about that asset's own units, up-axis and pivot. A
BAKE is none of those things: `gac_fire.place_source` has already applied the
pack scale and re-centred it, the kit path builds at (0,0,0), and the export
is metres, Z-up, base at z = 0. Inheriting the cell's transform would shrink
it 100x and lay it on its side.

So this launcher HIDES the intact prim and authors a FRESH holder:

    /World/fire/<stem>          translate (x, y, z) * ssf
                                rotateXYZ (0, 0, yaw_deg)      <- the
                                    placement's FINAL yaw, which already
                                    includes the per-asset yaw-offset
                                scale     1  (x ssf: a bake is in metres)
    /World/fire/<stem>/bake     the reference

with the reference onto a CHILD, never onto the Xform that carries the
transform — the same rule `gac_fire.place_source`, `kit_bake.load_kit` and
the row assembler all follow, so a referenced asset's own transform can never
compose with (or clobber) the one authored here.

Proved offline, on a real bake, by `scene_gen/tools/fc_transform_probe.py`:
the composed centre is `R(yaw) . own_centre + (x, y)` to 0.0000 m, the base z
is unchanged, and at yaw 90 the footprint's W and D exchange to 0.0000 m.
That probe also found the one thing the plan got wrong: the plan asks for the
composed BBOX centre to land on the cell, and it does not — a burnt bake's
bbox includes the DEBRIS it dropped into the street (measured: +1.64 m of
spall east of a building whose fire is on its east elevation), while its
`masses["main"]` centre is (0.000, 0.000). The SHELL is what is plan-centred
and the shell is what has to land on the cell, so nothing is re-centred here.

THE EMITTER BUDGET (plan sec 4b) — the bug that reports success
---------------------------------------------------------------
Flow's block pool is finite and shared. Past it, Flow logs "Out of GPU memory
allocating resource 'flow'" / "Maximum Flow blocks ... in use" and every
emitter after the first few gets NO VOXELS — a city that renders with no
smoke while every count in the log looks right. A row of 6 buildings could
ignore this; 12-20 damaged buildings at the row's own per-building budget is
150-350 emitters, so this launcher spends a GLOBAL budget
(`FA_EMITTER_BUDGET`, default 800 as of 2026-08-31 — see that knob's own
comment for the arithmetic behind the number):

  * buildings are RANKED by state — flame (F2/F3, actively burning) first,
    then smoulder (F4/F5/F6), then the F1 wisps and anything residual;
  * every accepted building gets one opening's worth first, in rank order,
    so the worst buildings are never the ones dropped;
  * the remainder is handed out in rounds, again in rank order, up to
    `FA_EMITTERS` openings each;
  * a building that cannot fit even its minimum carries GEOMETRY AND BAKED
    SOOT ONLY, and says so — the plan's "the rest carry soot and geometry
    only". Its damage is still there; only its live fire is not.

FIRST FIT BY RANK, not strict rank order: once a building is refused the
allocator keeps going down the list, so a cheap F1 wisp can still take the
change a 14-emitter F4 could not fit into. That spends the budget instead of
stranding it, and it is why the printed table can show a smouldering building
dropped beside a wisp that was kept — read the `note` column, which carries
the exact figure that did not fit.

The predicted count is printed per building beside the ACTUAL count
`place_fire` returns, and the Kit log is grepped for the two Flow OOM strings
after the captures. Do not trust a capture until that grep is clean.

A TALL BUILDING HAS AN OPENING FLOOR. `place_fire` widens the budget for a
tall building on purpose (`max_open = max(max_emitters, min(16, n_st // 2))`
from 12 storeys up — "the taller buildings need more fire/flames, looks weird
otherwise"). That floor cannot be allocated below, so the allocator uses the
same formula to price a building and, if even the floor does not fit, drops
the whole building rather than pretending a smaller allocation would help.

Env:
    SCENE_CONFIG   preset (default `downtown_fire_500`). NOTE: that preset
                   carries `disaster-type: fire`, which `compile_disaster.
                   DISASTERS` has no entry for — it is a spec vocabulary for
                   the spread solver, not a compiled field — so it is
                   compiled with `disaster-type` overridden to `none`, the
                   same override `tools/fire_city_dry_run.py` uses, and the
                   raw preset is read again for the wind.
    FC_MANIFEST    `_plans/fire_city_<seed>.json` — `urban_fire_city.
                   damaged_manifest` records. Default: the newest
                   `scene_gen/_plans/fire_city_*.json`.
    FC_BAKES       bake directory or comma list of `.usd`
                   (default `/isaac-sim/.cache/fire_bakes/city_<seed>/`,
                   exactly where `tools/fire_city_bake.sh` puts them and
                   prints as its `FA_BAKES` line)
    FC_INTACT_ONLY 1 builds the city and STOPS — the "measure the intact
                   city first" step of plan sec 4c, the only unknown in the
                   VRAM projection. No manifest is needed for it. In this
                   mode the launcher ALSO writes the placements dump (see
                   FC_DUMP) — the whole reason FC_INTACT_ONLY exists as a
                   separate step is that it is the one place this pipeline
                   has both the REAL Nucleus-measured footprints and no
                   manifest yet to get wrong.
    FC_DUMP        path to write the city-placements dump to, in
                   FC_INTACT_ONLY mode only (default `scene_gen/_plans/
                   city_placements_<preset>_<seed>.json`). Every house
                   placement's cell/usd/x_m/y_m/z_m/yaw_deg/scale/category
                   plus its MEASURED W/D/H (a fresh `SizeResolver`, same
                   class `generate_scene_on_stage` used, so the numbers are
                   real Nucleus footprints, not the dry run's host-side
                   GAC/DTC substitutes) and the typology block map
                   (`config["_city_layout"]["_typology_of"]`) — everything
                   `tools/fire_city_dry_run.py --placements-json` needs to
                   solve a fire on the EXACT layout Kit just built, instead
                   of reconstructing one host-side that can pack
                   differently and describe a city that does not exist.
    FC_ENV         `default` (the historical behavior: `PegasusInterface().
                   load_environment(ENV_URL)`, the same "Default
                   Environment" every other launcher in this repo loads),
                   `none` (skip `load_environment` entirely), or a URL to
                   load instead. `load_environment(ENV_URL)` measured at
                   ~9.5 GiB of the ~12.0 GiB "empty stage" baseline this
                   launcher prints — see this file's own note above
                   `FireCityApp.__init__` for what it actually buys (spoiler:
                   `_remove_env_clutter` deactivates its GroundPlane/
                   Environment children a few lines later, and the city
                   authors its own ground and sky regardless).
    FC_HIDE        `invisible` (default, `MakeInvisible`) or `deactivate`
                   (`SetActive(False)`). An invisible prim still composes and
                   still costs memory; a deactivated one does not, but its
                   subtree can no longer be inspected in the viewport.
    FC_SKY         `sunset` (default -- BYTE-IDENTICAL to every prior fire
                   city render: `add_sky(resolve_sky(config), ...)` off the
                   preset's own `sky_intensity`/`sky_exposure`, dome-only, no
                   sun) or `mid_day` (see `sky_presets.py` -- high overhead
                   sun, neutral-cool white, blue-sky dome; opt in here only
                   after the bench (`fire_people_bench_launch_script.py`,
                   `PB_SKY`) has been reviewed), or an explicit HDRI/stage
                   path or URL (mid_day's numbers, dome textured instead).
    FA_FLOW        1 (default) authors the Flow stack and the emitters
    FA_SMOKE       1 (default) — 0 gives flames only, no plumes
    FA_CELL_M      Flow density cell size, m (default 0.55 as of the THIRD
                   2026-08-31 review, up from 0.3/0.45 — coarsened on
                   purpose to keep the block-pool need down at the higher
                   emitter counts size-scaling/clusters/residual pockets
                   ask for; see `CELL_M`'s own comment for the arithmetic)
    FA_MAX_BLOCKS  the Flow block POOL (default 12288, unchanged since
                   round 2 — already proven clean at 560 emitters/0.45 m
                   cells; see `MAX_BLOCKS`'s own comment for why the SAME
                   pool still covers the new, higher emitter count once
                   `FA_CELL_M` coarsens). `rtx/flow/maxBlocks` is a carb
                   setting, not the USD attribute.
    FA_EMITTERS    per-building opening budget before the global budget
                   trims it (default 30 as of the THIRD review, up from
                   10/6 — see `MAX_EMITTERS`'s own comment: this is now the
                   size-scaling CEILING, `fal.FLAME_WINDOWS_MAX`, not a flat
                   per-building count)
    FA_EMITTER_BUDGET  the GLOBAL emitter cap (default 800 as of the THIRD
                   review, up from 560/200 — see `EMITTER_BUDGET`'s own
                   comment)
    FA_SIDE_SMOKE_FLAME  window/opening smoke sources an ACTIVELY-FLAMING
                   building draws from its own burnt-OUT compartments
                   (default 5; was the fixed `uf.SMOKE_EXTRA_MAX`=3)
    FA_SIDE_SMOKE_MAX  window/opening smoke sources a SMOULDERING/burnt-out
                   building draws from its SMOULDER events first, then its
                   "out" events to fill the rest (default 6; was the fixed
                   `spl.SMOULDER_EVENTS_MAX`=3, smoulder-only)
    FA_ROOF_INTACT_MAX  roof-plume sources on a building whose roof has NOT
                   collapsed (`fal.roof_has_collapsed`) — default 1, down
                   from the original fixed 2, so side smoke dominates an
                   intact roof per the 2026-08-31 review
    FA_ROOF_COLLAPSED_MAX  roof-plume sources once the roof HAS collapsed
                   (F5c/F6, or the sidecar's `top_z`/`deck_z` gap has
                   closed) — default 2, unchanged: a real hole earns them
    FA_STREET_DUMP  placements dump `street_side_ranks` scores a building's
                   `sides` against, to bias the round-robin toward whichever
                   one is more likely a real street (default: this run's own
                   `FC_DUMP`, so passing that once covers both). Missing or
                   unparseable degrades to no bias, not a failed build.
    FA_STREET_BIAS  extra weight (`_side_weights`) the CHOSEN side's own
                   round-robin turn gets, default 2 — see `fire_assembly_
                   lib._round_robin`'s own docstring. WHICH side is chosen
                   is a per-building weighted RANDOM draw
                   (`fal.choose_street_side`, seeded off the bake's own
                   stem — 2026-08-31 second review: "I want it randomized"),
                   not the argmax.
    FA_FLAME_MIN_CLUSTERS  minimum distinct `(side, storey)` flame clusters
                   a "flame"-state building's own live events top up to,
                   from dimmer "out"-event accents, when its own data alone
                   has fewer (default 3; 0 disables — 2026-08-31 second
                   review, "more actual places of fire on each building")
    FA_FLAME_EXTRA_MAX  cap on how many extra cluster-top-up picks that
                   adds (default 3)
    FA_FLAME_SIZE_SCALE  1 (default) drives the flame-opening ceiling off
                   `fal.flame_window_target(n_storeys)` instead of the flat/
                   height-floor formula (2026-08-31 third review, "if the
                   smaller building can have 5-7 fire windows, the bigger
                   one should have more")
    FA_SMOKE_SIZE_SCALE  1 (default) does the same for the two side-smoke
                   caps, via `fal.smoke_window_target`
    FA_SMOKE_WINDOW_JETS  1 (default) picks side smoke at the OPENING level
                   (several window-sheet sources, one per opening) instead
                   of one per EVENT at its own middle opening — 2026-08-31
                   second review, "they should be coming out of windows
                   similar to the fire"
    FA_RESIDUAL_FLAME_FRAC  fraction of a same-height FLAME-state building's
                   own `flame_window_target` that a "smoulder"(F4)/
                   "residual"(F5/F5c/F6) building gets as SCATTERED single-
                   window flame pockets (default 0.4; 0 restores the
                   original F4-only top-up and no F5+ flame at all) —
                   2026-08-31 third review, the headline case: a 28-storey
                   F5 tower with 75 baked events authored zero flame before
                   this
    FA_SCALE       emission scale multiplier (default 1.0)
    FA_SEED        rng seed for the per-emitter jitter (default 7)
    FC_SCORCH_VEG  1 (default) darkens street trees / greenery within reach
                   of a damaged building (F2 up — see `fire_assembly_lib.
                   veg_scorch_radius_m`) to a flat charred material. 0 skips
                   the whole pass; nothing about a tree changes, it stays
                   whatever the city generator authored (green).
    FC_FIRE_APRON  1 (default) scatters a small ground-debris apron along
                   the VENTING sides of every F1-F5 (non-collapse) burning
                   building — see `fire_assembly_lib.fire_apron_pass`. F5c/
                   F6 already drop a collapse heap and are not touched by
                   this pass. 0 skips it.
    FA_APRON_SCALE density multiplier on top of `fire_assembly_lib.
                   APRON_DENSITY` (default 1.0 — already the doubled
                   2026-08-31-round-2 table; "the minor debris ... needs to
                   increase"). One merged Mesh prim per qualifying building
                   regardless of lump count, so this is cheap to raise
                   further — see `tools/fire_street_debris_dry_run.py` for
                   the per-building cost against the real 39-manifest.
    FC_HIDE_PROPS  1 (default) hides a damaged building's COMPANION ROOF/WALL
                   props (`gac_props`' `roof_house`/`roof_tank`/`roof_mast`/
                   `roof_prop`/`wall_run`) along with its intact shell. They
                   are SIBLINGS of the building placement, not children, so
                   without this they keep standing at the elevation of a roof
                   that is no longer drawn — 39 of them on this city's 20
                   damaged buildings. Matched by `gac_props._place`'s own
                   `of` tag; see `prop_tag`.
    FC_UNINSTANCE_GPRIM_ROOTS
                   1 (default) un-instances any placement whose composed root
                   prim is a GPRIM. With `instance_placements: true` such an
                   asset's materials, GeomSubsets and (for the renderer) its
                   geometry all end up in the prototype while the drawn prim
                   stays outside it — the asset reads untextured grey, or
                   disappears. 161 of this city's 1469 placements: ALL 58
                   streetlights, 50 of the 62 benches, 16 cars, 12 traffic
                   lights, the fountains, and 10 buildings. Measured with
                   `tools/fc_instance_material_probe.py`; see
                   `_uninstance_gprim_roots`.
    FC_PEOPLE      1 (default) runs the people pass
    FC_PEOPLE_JSON the REVIEWED records file `tools/fire_people_dry_run.py`
                   wrote. Its records are under the **`people`** key (not
                   `records`); `FireCityApp.people_records` accepts either,
                   or a bare list, and raises a named error otherwise.
    FC_PEOPLE_MAX_DIST_M
                   120 (default) — "only keep humans that are in the
                   disaster". A record further than this from the nearest
                   BURNING BUILDING'S FOOTPRINT (not its centre) is dropped
                   with a count. 0 disables. Measured on
                   `_plans/fire_people_final.json`: all 89 records are within
                   59.5 m, so the default drops none — for THOSE it is a
                   guard, not the mechanism. It also drives
                   `cull_background_people`, which HIDES the city generator's
                   OWN pedestrians (128 of them on this plate,
                   `category == "human"` from `scene_generator` and
                   `detail/parks.py`) outside the same radius — that is where
                   the rule actually bites.
    SNAP_DIR       viewport captures, MUST be under
                   /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN      1 keeps the app up after the captures
    ISAAC_SIM_HEADLESS  true for a headless run

Banner: `URBAN FIRE CITY DONE`.
"""

import glob as _glob
import json
import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default. Treat empty as
    absent."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


def _flag(name, default="0"):
    return _env(name, default).lower() in ("1", "true", "yes")


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
# FRACTIONAL CUTOUT OPACITY — `extra_args`, NOT `carb.settings`, and BOTH
# forms are required (the startup flag does not survive stage composition;
# the carb form alone is too late for startup). The bakes carry
# `soot_plume`'s baked atlases, `urban_fire._glass_pane`'s smoke deposits and
# the void tone, all fractional-cutout, which RTX discards unless this is on
# — the staining then renders as a hard binary stamp.
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import carb                                                    # noqa: E402
import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, UsdGeom                               # noqa: E402
from omni.isaac.core.world import World                        # noqa: E402
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS   # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import (  # noqa: E402
    PegasusInterface)

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import (add_sky, get_stage_meters_per_unit,     # noqa: E402
                        scale_stage_prim, settle_rigid_props)
from scene_generator import resolve_sky, _make_resolver         # noqa: E402
from sky_presets import apply_sky_preset                        # noqa: E402
from generate_scene import generate_scene_on_stage              # noqa: E402
from compile_disaster import load_scene_config, resolve_config_path  # noqa: E402
from disaster import fire as fx                                 # noqa: E402
from disaster import fire_assembly_lib as fal                   # noqa: E402
from disaster import fire_bake as fb                            # noqa: E402
from disaster import soot_plume as spl                          # noqa: E402
from disaster import urban_fire as uf                           # noqa: E402
from disaster import urban_fire_city as ufc                     # noqa: E402



def _load_by_path(name, path):
    """Import a module from a FILE, without putting its directory on
    `sys.path` — the idiom this repo already uses for `utils/snapshots.py`.
    `scene_gen/tools` is not a package, and prepending it to `sys.path` would
    put every helper script in there ahead of the standard library for the
    rest of the process."""
    import importlib.util as _ilu
    spec = _ilu.spec_from_file_location(name, path)
    mod = _ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# `fire_city_manifest` is the ONE place a manifest record becomes an entry
# string and a cache stem — `fire_city_bake.sh` classifies with the same
# module — so a bake this launcher looks for and a bake that driver produced
# can never disagree about the name.
fcm = _load_by_path("fire_city_manifest",
                    os.path.join(_SCENE_GEN_DIR, "tools",
                                 "fire_city_manifest.py"))

ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
PARENT = "/World/stage/generated"
FIRE_ROOT = "/World/fire"
_ENV_CLUTTER = {"GroundPlane", "Environment"}

#: the two strings Flow logs when its block pool is exhausted. It keeps
#: reporting success while every emitter past the first few gets no voxels,
#: so this is the only way to know a capture is worth looking at.
FLOW_OOM_NEEDLES = ("Out of GPU memory allocating resource 'flow'",
                    "Maximum Flow blocks")
KIT_LOG_GLOBS = ("/isaac-sim/kit/logs/Kit/Isaac-Sim Python/*/*.log",
                 "/isaac-sim/kit/logs/Kit/*/*/*.log")

SCENE_CONFIG = _env("SCENE_CONFIG", "downtown_fire_500")
MANIFEST = _env("FC_MANIFEST", "")
BAKES = _env("FC_BAKES", "")
INTACT_ONLY = _flag("FC_INTACT_ONLY", "0")
DUMP_PATH = _env("FC_DUMP", "")
FC_ENV = _env("FC_ENV", "default")
HIDE_MODE = _env("FC_HIDE", "invisible").lower()
# "sunset" is the CURRENT look, kept as the default so the city only changes
# once "mid_day" is chosen explicitly -- see this file's own docstring and
# sky_presets.py for the full diagnosis/design.
SKY = _env("FC_SKY", "sunset")
FLOW = _env("FA_FLOW", "1") not in ("0", "false", "no")
SMOKE = _env("FA_SMOKE", "1") not in ("0", "false", "no")
# FA_SMOKE_SCALE multiplies the smoke-only emission scale and the interior/
# roof plume seat radii ("increase amount of smoke", user 2026-08-31);
# budget-neutral — emitter COUNTS are untouched, estimator parity holds.
SMOKE_SCALE = float(_env("FA_SMOKE_SCALE", "1.0"))
# 0.55 (was 0.3, was 0.45 by explicit override on both live relaunches):
# 2026-08-31 THIRD review coarsens the cell on purpose. `scene_gen/tools/
# fire_flow_dry_run.py`, run against the real 39-record `fire_city_500m_39
# .json` manifest, projects ~800 emitters at `FA_EMITTER_BUDGET=800` (the
# natural, unbudgeted total is ~976) -- at the 0.45 m cell BOTH live
# relaunches actually used, that needs ~16,600 blocks, well past the
# 12288-block pool already proven clean. Voxel count goes as `1/cell**3`,
# so 0.45 -> 0.55 cuts the requirement to ~9,100 blocks -- back under the
# SAME 12288 pool round 2 already validated at 560 emitters, with margin,
# rather than pushing the pool itself into untested territory. The
# coordinator's own instruction: "raise FA_MAX_BLOCKS proportionally, and
# coarsen FA_CELL_M slightly (0.45 -> up to 0.55) if the arithmetic needs
# it... do not exceed ~15.3 GB projected total" -- the arithmetic needs it.
CELL_M = float(_env("FA_CELL_M", "0.55"))
# 12288 (was 6144): unchanged from round 2 (already proven clean at 560
# emitters / 0.45 m cells) -- at 0.55 m cells and ~800-900 emitters the
# projected need is ~9,100-10,200 blocks (see `CELL_M`'s own comment and
# `fire_flow_dry_run.py`'s own projection), comfortably under this pool
# without raising it into a range nothing has actually measured. Still an
# ESTIMATE, not a substitute for the Kit-log OOM grep after an actual
# relaunch (see `grep_kit_log`/`FLOW_OOM_NEEDLES`, and the skill's own
# "Flow fails SILENTLY" warning -- a scene can under-report a full block
# pool while looking completely normal in this launcher's own banner).
MAX_BLOCKS = int(_env("FA_MAX_BLOCKS", "12288"))
# 30 (was 10, was 6): with `FA_FLAME_SIZE_SCALE=1` this is `allocate_
# emitters`' `cap` — the CEILING the allocator's own per-round `step`
# climbs to (see that function's own docstring) — and must reach `fal.
# FLAME_WINDOWS_MAX` (30) or a tall tower's own target is never fully
# funded even with unlimited budget. A short building's `min(step, target)`
# plateaus at its OWN (much smaller) target long before `step` gets this
# high, so this does not by itself cost every building 30 openings.
MAX_EMITTERS = int(_env("FA_EMITTERS", "30"))
# 800 (was 560, was 200): THIRD review — cluster diversity, size-scaled
# allocation and residual flame pockets all raise per-building demand; the
# dry run's own natural (unbudgeted) total across the real 39-building
# manifest is ~976, and 800 is comfortably below both that and the 15.3 GB
# VRAM cap the coordinator set (~15.1 GB projected at 800, per `fire_flow_
# dry_run.py`'s own two-point-calibrated model) — and the allocator's own
# rank-first spending (see `allocate_emitters`) means nothing is dropped
# WHOLE at this budget; the worst buildings (incl. the headline 28-storey
# residual tower) are funded to their FULL target regardless, only the
# lowest-ranked buildings' growth is trimmed.
EMITTER_BUDGET = int(_env("FA_EMITTER_BUDGET", "800"))
# SIDE SMOKE AT OPENING HEADS (2026-08-31 review: "let's have it from the
# sides as well... F4 buildings especially should pour smoke from
# openings"). `None` would fall back to the ORIGINAL fixed budgets inside
# `place_fire` (`uf.SMOKE_EXTRA_MAX`/`spl.SMOULDER_EVENTS_MAX`, 3 each); this
# launcher always passes a value, so its own default IS the effective one.
SIDE_SMOKE_FLAME_MAX = int(_env("FA_SIDE_SMOKE_FLAME", "5"))
SIDE_SMOKE_NONFLAME_MAX = int(_env("FA_SIDE_SMOKE_MAX", "6"))
# ROOF VS SIDE (same review: "unless the roof is collapsed there should be
# more smoke coming from sides than top"). `fal.roof_has_collapsed` decides
# which cap applies per building — see that function's own docstring for
# what it can and cannot tell from a kit vs. a GAC sidecar.
# 0 (was 1), 2026-08-31: "there's also still smoke on the roofs of
# non-collapsed buildings that look weird" — an intact roof now gets NO
# plume at all; its smoke story is entirely window jets + interior seats
# (which the vertical-bias redistribution below strengthens). A COLLAPSED
# roof keeps its plumes unchanged — smoke rising from a broken deck is
# correct, not a leftover look. The knob stays live for an A/B.
ROOF_CAP_INTACT = int(_env("FA_ROOF_INTACT_MAX", "0"))
ROOF_CAP_COLLAPSED = int(_env("FA_ROOF_COLLAPSED_MAX", "2"))
# STREET-FACING BIAS (same review: "I want more of the actual 'fire'
# elements on buildings, especially on street facing sides so it's
# visible"). `FA_STREET_DUMP` is the placements dump `street_side_ranks`
# scores against (default: `FC_DUMP`, so a run that is already passing that
# knob for its own sake gets the bias for free); empty/missing/unparseable
# degrades to `{}` (`fal.load_dump_positions`), which is "no bias" — the
# ORIGINAL unweighted round-robin — not a failed build. `FA_STREET_BIAS` is
# the extra weight (`_side_weights`) the ranked side's own round gets.
STREET_DUMP_PATH = _env("FA_STREET_DUMP", "") or _env("FC_DUMP", "")
STREET_BIAS_WEIGHT = int(_env("FA_STREET_BIAS", "2"))
# MULTIPLE FLAME CLUSTERS (2026-08-31, second review: "more actual places of
# fire on each building"). `0` for either disables the top-up entirely
# (`fal.place_fire`'s `flame_min_clusters=None` path).
FLAME_MIN_CLUSTERS = int(_env("FA_FLAME_MIN_CLUSTERS", "3"))
FLAME_EXTRA_MAX = int(_env("FA_FLAME_EXTRA_MAX", "3"))
# SIZE-SCALED ALLOCATION (2026-08-31, third review, item 4: "if the smaller
# building can have 5-7 fire windows, the bigger one should have more").
# `fal.flame_window_target`/`smoke_window_target` replace the flat/height-
# floor formulas with a straight line in `n_storeys` — see those functions'
# own docstrings for the exact clamp.
FLAME_SIZE_SCALE = _flag("FA_FLAME_SIZE_SCALE", "1")
SMOKE_SIZE_SCALE = _flag("FA_SMOKE_SIZE_SCALE", "1")
# WINDOW-JET SMOKE GEOMETRY (2026-08-31, second review: "I see more smoke
# but not really side smoke, they should be coming out of windows similar
# to the fire"). `1` picks smoke at the OPENING level (several sources, one
# per window) instead of one per EVENT at its own middle opening.
SMOKE_WINDOW_JETS = _flag("FA_SMOKE_WINDOW_JETS", "1")
# SMOKE COMPLEMENTS THE FLAME VERTICALLY (2026-08-31, "have more smoke on
# lower floors so it looks like those have been burnt out if you're not
# putting fire there"): ON by default — a side-smoke pick that used to rank
# candidates highest-storey-first now ranks not-yet-lit-and-lower-storey
# first instead (`fal.place_fire`'s own docstring has the full mechanism),
# with a guarantee that at least one pick still lands near the flame itself.
SMOKE_VERTICAL_BIAS = _flag("FA_SMOKE_VERTICAL_BIAS", "1")
# NO FIRE AT THE EXTREME TOP UNLESS THE WINDOWS ARE REAL (2026-08-31, "avoid
# fires at the extreme top of buildings... unless we're 100% sure about
# windows on the top floor"): `fal.place_fire` runs this filter
# UNCONDITIONALLY (it only ever touches a SYNTHETIC opening — see
# `fal.drop_top_storey_synthetic`), so there is no knob to gate it here; the
# per-building probe table has its own column.
# RESIDUAL FLAME POCKETS (2026-08-31, third review, item 5 — the headline
# case: a 28-storey F5 "residual" tower with 75 baked events authored ZERO
# flame). `0` restores today's behaviour exactly (F4 keeps its ORIGINAL
# top-up, F5/F5c/F6 gets none) — see `fal.place_fire`'s own docstring for
# what a positive fraction does.
RESIDUAL_FLAME_FRAC = float(_env("FA_RESIDUAL_FLAME_FRAC", "0.4"))
# CONTACT SNAP (2026-08-31, "some of the fires seem to be floating outside
# the building... make it go inside the building by some amount") — a
# building with NO real glazing anywhere (`gac_fire.window_rects` never fills
# `planes[side]`) gets its synthetic openings framed off the MASS BBOX face
# instead of the real wall, which can sit metres off it (a parapet, a
# setback, an L-notch the bbox spans). `fal.place_fire`'s `geom_root` opts
# into a per-building contact test — one `vtkStaticCellLocator` over the
# bake's own composed geometry, reused for every opening — that snaps a
# floating opening onto the real wall (inset `FA_SNAP_INSET_M` further in)
# or drops it outright if nothing real is within reach. ON by default: this
# is a correctness fix, not a look knob — `FC_CONTACT_SNAP=0` restores the
# pre-fix placement exactly (`geom_root=None`, `fal.place_fire`'s original
# behaviour) for an A/B or if `vtk` turns out unavailable in some image.
CONTACT_SNAP = _flag("FC_CONTACT_SNAP", "1")
SNAP_TOL_M = float(_env("FA_SNAP_TOL_M", str(fal.SNAP_TOL_M)))
SNAP_INSET_M = float(_env("FA_SNAP_INSET_M", str(fal.SNAP_INSET_M)))
SNAP_REACH_IN_M = float(_env("FA_SNAP_REACH_IN_M", str(fal.SNAP_REACH_IN_M)))
SNAP_REACH_OUT_M = float(_env("FA_SNAP_REACH_OUT_M",
                              str(fal.SNAP_REACH_OUT_M)))
SCALE = float(_env("FA_SCALE", "1.0"))
SEED = int(_env("FA_SEED", "7"))
SCORCH_VEG = _flag("FC_SCORCH_VEG", "1")
FIRE_APRON = _flag("FC_FIRE_APRON", "1")
# LIVE DENSITY KNOB (2026-08-31, second round: "the minor debris ... needs to
# increase"). `fal.APRON_DENSITY`/`APRON_MAX_PER_SIDE` were raised for the
# shipped default; this multiplies ON TOP of that table for further tuning
# without a code edit — 1.0 is the (already denser) new default look.
APRON_SCALE = float(_env("FA_APRON_SCALE", "1.0"))
# THE PEOPLE PASS RUNS LAST ("after everything is baked in", user
# 2026-08-31): FC_PEOPLE_JSON is the REVIEWED records file the fire_people
# dry run wrote (its PNG is the 2-D gate) — the launcher never re-solves
# placement, it authors what was approved.
PEOPLE = _flag("FC_PEOPLE", "1")
PEOPLE_JSON = _env("FC_PEOPLE_JSON", "")
# "Only keep humans that are in the disaster" (user, 2026-08-31). Every
# record already names the burning building it was solved against
# (`building_i`), so this is a GUARD, not the mechanism — see `place_people`.
PEOPLE_MAX_DIST_M = float(_env("FC_PEOPLE_MAX_DIST_M", "120"))
# 1 (default) hides a damaged building's COMPANION ROOF/WALL PROPS along with
# its intact shell — see `compose_bakes`.
HIDE_PROPS = _flag("FC_HIDE_PROPS", "1")
# 1 (default) un-instances any placement whose composed root prim is a GPRIM
# — see `_uninstance_gprim_roots`.
UNINSTANCE_GPRIM_ROOTS = _flag("FC_UNINSTANCE_GPRIM_ROOTS", "1")
SNAP_DIR = _env("SNAP_DIR", "")
KEEP_OPEN = _flag("KEEP_OPEN", "0")


def vram_mb(tag):
    """`fire_assembly_lib.vram_mb`, printing under this launcher's prefix."""
    return fal.vram_mb(tag, prefix="fc")


# ---------------------------------------------------------------------------
# The city (copied from `downtown_quake_launch_script`, which copied it from
# `scene_launch_script` — NOT imported, for the second-SimulationApp reason)
# ---------------------------------------------------------------------------
def _remove_env_clutter(stage):
    n = 0
    for root_path in ("/", "/World", "/World/stage"):
        root = (stage.GetPseudoRoot() if root_path == "/"
                else stage.GetPrimAtPath(root_path))
        if not root or not root.IsValid():
            continue
        for child in root.GetChildren():
            if child.GetName() not in _ENV_CLUTTER or not child.IsActive():
                continue
            if child.SetActive(False):
                n += 1
            else:
                UsdGeom.Imageable(child).MakeInvisible()
    print("[fc] env clutter: {0} prim(s) deactivated".format(n))


def _disable_sky_sun(stage):
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
    print("[fc] sky sun: {0} DistantLight(s) disabled".format(n))


def _wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            if [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]:
                return True
        time.sleep(0.1)
    return False


def preset_fire_block(name):
    """`epicenter` / `heading_deg` / `wind_mps` / `duration_s` /
    `start_offset_frac` read off the RAW preset yaml.

    They cannot come through `load_scene_config`: `compile_disaster.DISASTERS`
    has no `"fire"` entry (only `"wildfire"`, a continuous-fuel-bed ellipse
    over vegetation — the wrong model for discrete buildings separated by
    streets), so `compile_spec` would RAISE on `disaster-type: fire`. The
    preset says so in its own header; the dry run reads them the same way.
    Only `heading_deg` is used here — it aims the downwind "wave" capture —
    but all five are printed, because a capture that does not look like the
    wind in the manifest is a manifest/preset mismatch, not a camera bug.
    """
    import yaml
    out = {"epicenter": [0.0, 0.0], "heading_deg": 45.0, "wind_mps": 4.0,
           "duration_s": None, "start_offset_frac": None}
    try:
        with open(resolve_config_path(name)) as fh:
            raw = yaml.safe_load(fh) or {}
    except Exception as exc:
        print("[fc] could not read the raw preset for the wind ({0}) — "
              "defaulting to heading {1} deg".format(exc, out["heading_deg"]))
        return out
    for k in list(out):
        if raw.get(k) is not None:
            out[k] = raw[k]
    return out


# ---------------------------------------------------------------------------
# FC_DUMP — the placements Kit actually built, for the dry run to solve on
# instead of a host-side reconstruction that can pack differently (2026-08-30
# incident: the dry run's patched SizeResolver substitutes GAC/DTC footprints
# because Nucleus is not mirrored locally, so its packer makes different
# placement decisions than Kit's real one — the manifest it produced named
# cells that did not exist in the city this launcher actually built).
# ---------------------------------------------------------------------------
def default_dump_path(preset, seed):
    return os.path.join(_SCENE_GEN_DIR, "_plans",
                        "city_placements_{0}_{1}.json".format(preset, seed))


def _typology_rects(layout):
    """`[{"rect": [x0, y0, x1, y1], "name": name}, ...]` — JSON has no tuple
    keys, so `layout["_typology_of"]` (`{(x0,y0,x1,y1): name}`, written by
    `districts.rezone_blocks` and stashed onto `config["_city_layout"]` by
    `generate_scene_on_stage`) is serialised as a list, in the same
    iteration order `urban_fire_city.typology_at` itself walks."""
    return [{"rect": [float(v) for v in rect], "name": name}
            for rect, name in (layout or {}).get("_typology_of", {}).items()]


def dump_city_placements(path, preset, seed, config, placements, layout):
    """Write the city-placements dump `tools/fire_city_dry_run.py
    --placements-json` consumes — see the block comment above and this
    module's own docstring (FC_DUMP) for why the manifest must be solved on
    the SAME layout Kit built, not a host-side reconstruction.

    Only `category == "house"` placements are written: `urban_fire_city.
    burnable()`'s gate 1 refuses everything else outright, and a real
    downtown has one to two orders of magnitude more non-house placements
    (trees, benches, cars, streetlights, ...) that would bloat this file
    for no downstream use.

    EVERY HOUSE RECORD CARRIES ITS OWN `i` — its index in the FULL
    `placements` list (every category, not just houses) THIS PROCESS just
    built. `urban_fire_city_launch_script.resolve_cell`'s most-trusted route
    (route 1: match the record's `i` against the placement at that index,
    verified by usd + distance) needs `i` to still be the right index when
    the FULL launcher rebuilds the same (deterministic, same preset/seed)
    city later — a manifest built from indices renumbered relative to a
    house-only list would silently point route 1 at the wrong placement
    every time a non-house placement came before it. `n_placements_total`
    (the length of the full list) travels alongside so the dry run can
    rebuild a same-length list with non-house placeholders at every index
    it never saw, preserving exactly this alignment.

    W/D/H are MEASURED HERE, with a FRESH `SizeResolver` built the same way
    `generate_scene_on_stage`'s own internal one is
    (`scene_generator._make_resolver`) — same asset_scale / fallback_sizes /
    measure_usds, same `"house"` category, same per-placement scale/axis_up
    — so re-measuring in THIS process (which, unlike the host-side dry run,
    has live Nucleus access) reproduces the exact numbers `apply_placements`
    used for packing, rather than the dry run's patched GAC/DTC cache.
    """
    resolver = _make_resolver(config)
    houses = []
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        usd = p.get("usd")
        fp = resolver.get(usd, "house", scale=float(p.get("scale", 1.0)),
                          axis_up=p.get("axis_up", "Z"))
        houses.append({
            "i": i, "cell": p.get("prim_path"), "usd": usd,
            "x_m": float(p.get("x_m", 0.0)), "y_m": float(p.get("y_m", 0.0)),
            "z_m": float(p.get("z_m", 0.0)),
            "yaw_deg": float(p.get("yaw_deg", 0.0)),
            "scale": float(p.get("scale", 1.0)),
            "category": p.get("category"),
            "axis_up": p.get("axis_up", "Z"),
            "W": float(fp["sx"]), "D": float(fp["sy"]), "H": float(fp["sz"]),
        })
    doc = {
        "schema": "fire_city_placements_dump.v1",
        "preset": preset, "seed": int(seed),
        "region_m": [float(v) for v in
                    config.get("layout", {}).get("region_m", [0.0, 0.0])],
        "n_placements_total": len(placements),
        "placements": houses,
        "typology": {"blocks": _typology_rects(layout)},
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=1)
    print("[fc] wrote city placements dump ({0} house placement(s) of {1} "
          "total, {2} typology block(s)) -> {3}".format(
              len(houses), len(placements),
              len(doc["typology"]["blocks"]), path))
    return path


# ---------------------------------------------------------------------------
# The manifest and its bakes
# ---------------------------------------------------------------------------
def find_manifest(spec):
    """`FC_MANIFEST`, or the newest `_plans/fire_city_*.json`."""
    if spec:
        return spec
    pats = sorted(_glob.glob(os.path.join(_SCENE_GEN_DIR, "_plans",
                                          "fire_city_*.json")),
                  key=lambda p: os.path.getmtime(p), reverse=True)
    return pats[0] if pats else ""


def bake_paths(records, seed, spec):
    """One `(record, usd, json, stem)` per manifest record, in manifest order.

    `stem` is `fire_bake.out_stem` of the record's entry — via
    `fire_city_manifest.build_entry_and_stem`, the same call
    `fire_city_bake.sh` classifies with, so a bake this launcher cannot find
    is genuinely not baked rather than merely named differently. `usd` is
    `""` when nothing on disk matches.
    """
    if spec:
        items = [q.strip() for q in str(spec).split(",") if q.strip()]
    else:
        items = [os.path.join(fb.DEFAULT_OUT_DIR, "city_{0}".format(seed))]
    dirs = [q for q in items if os.path.isdir(q)]
    by_stem = {}
    for q in items:
        if os.path.isdir(q):
            for u in sorted(_glob.glob(os.path.join(q, "*.usd"))):
                by_stem.setdefault(
                    os.path.splitext(os.path.basename(u))[0], u)
        elif q.lower().endswith(".usd"):
            by_stem.setdefault(os.path.splitext(os.path.basename(q))[0], q)
    print("[fc] bakes: {0} file(s) visible in {1}".format(
        len(by_stem), ", ".join(dirs) or spec or "(nothing)"))
    out = []
    for i, rec in enumerate(records):
        try:
            _entry, stem = fcm.build_entry_and_stem(rec, i)
        except Exception as exc:
            print("[fc] *** record {0} is not a valid bake entry: {1}"
                  .format(i, exc))
            out.append((rec, "", "", None))
            continue
        usd = by_stem.get(stem, "")
        js = (os.path.splitext(usd)[0] + ".json") if usd else ""
        out.append((rec, usd, js if (js and os.path.exists(js)) else "", stem))
    return out


def resolve_cell(stage, placements, rec):
    """`(prim_path, how)` for the INTACT city prim this bake replaces.

    THREE ROUTES, MOST TRUSTWORTHY FIRST, because `cell` is a path the DRY
    RUN computed in a different process: `apply_placements` names a prim
    `<parent>/<category>_<group>_<i>` where `group` is the first-appearance
    order of its USD, so ANY change to the placement list — a different
    generator revision, a re-seeded pass — renumbers every cell after it, and
    a stale `cell` would hide the wrong building silently.

      1. the record's `i` (its index into the same placement list this
         process just rebuilt), accepted only if the placement at `i` has the
         SAME usd and is within 0.5 m of the record's own (x, y);
      2. the record's `cell` path, if it is a valid prim on this stage;
      3. the nearest `house` placement with the same usd within 2 m.

    `(None, reason)` when all three fail — the caller then refuses to place
    the bake, because a burnt shell composed inside an intact one is worse
    than a missing fire: it z-fights, and it reads as a different bug.
    """
    i = rec.get("i")
    rx, ry = float(rec.get("x", 0.0)), float(rec.get("y", 0.0))
    if isinstance(i, int) and 0 <= i < len(placements):
        p = placements[i]
        if p.get("usd") == rec.get("usd"):
            d = math.hypot(float(p.get("x_m", 0.0)) - rx,
                           float(p.get("y_m", 0.0)) - ry)
            if d <= 0.5 and p.get("prim_path"):
                return p["prim_path"], "index i={0} (d={1:.2f} m)".format(i, d)
    cell = rec.get("cell")
    if cell:
        prim = stage.GetPrimAtPath(Sdf.Path(cell))
        if prim and prim.IsValid():
            return cell, "manifest cell path"
    best, best_d = None, 2.0
    for p in placements:
        if p.get("category") != "house" or p.get("usd") != rec.get("usd"):
            continue
        d = math.hypot(float(p.get("x_m", 0.0)) - rx,
                       float(p.get("y_m", 0.0)) - ry)
        if d < best_d and p.get("prim_path"):
            best, best_d = p["prim_path"], d
    if best:
        return best, "nearest same-asset placement (d={0:.2f} m)".format(best_d)
    return None, ("no placement matches i={0!r}, cell={1!r}, or a same-asset "
                  "house within 2 m of ({2:.1f}, {3:.1f}) — the layout this "
                  "launcher built is NOT the one the manifest was solved on"
                  .format(i, cell, rx, ry))


def hide_intact(stage, path):
    """`MakeInvisible` (default) or `SetActive(False)` on the intact prim.

    `UsdGeom.Imageable.MakeInvisible()` RETURNS NOTHING (it is void in C++),
    so `bool(...)` on it is always False and a naive caller reports every
    hide as a failure — or, worse, trusts a hide that did not take. The
    visibility attribute is read back instead.
    """
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return False
    if HIDE_MODE.startswith("deact"):
        return bool(prim.SetActive(False))
    img = UsdGeom.Imageable(prim)
    if not img:
        # a typeless holder whose reference composed nothing typed
        return bool(prim.SetActive(False))
    img.MakeInvisible()
    return img.GetVisibilityAttr().Get() == UsdGeom.Tokens.invisible


# ---------------------------------------------------------------------------
# THE COMPANION PROPS (user, 2026-08-31: "/World/stage/generated/
# roof_house_94_1354/LOD0 this roof house is floating with no building near
# it" + "Lots of floating debris and roof props")
# ---------------------------------------------------------------------------
def prop_tag(usd, x_m, y_m):
    """`detail/gac_props._place`'s own `of` tag, `"{basename}@{x:.1f},{y:.1f}"`.

    `gac_props.dress()` authors the rooftop kit (`roof_house` stair/lift
    bulkheads, `roof_tank`, `roof_mast`, `roof_prop`, `wall_run`) as SEPARATE
    placements appended to the city's placement list — NOT as children of the
    building they stand on. So hiding a damaged building's intact prim leaves
    its props standing, unsupported, at the elevation of a roof that is no
    longer drawn. The bake carries its OWN settled roof plant (see any F5c
    sidecar's "roof plant: bulkhead on W, 1 row(s) of condensers on a pad"),
    so the city's props for that building are pure duplication anyway.

    MATCHED BY IDENTITY, NEVER PROXIMITY — `gac_props.roof_plant_of`'s own
    rule, and its docstring says why: "Matching by nearest-position instead
    fails on a real street, where the neighbour is closer than the far side of
    the same building." `gac_props` is not imported here (this launcher does
    not import the detail package) and the tag format is four characters of
    contract; `scene_gen/tools/fc_prop_orphan_probe.py` replays `dress()`
    offline and asserts this launcher's tag against `gac_props`' own.

    THE TAG IS BUILT FROM THE PLACEMENT, NOT THE MANIFEST RECORD. `dress()`
    formats the BUILDING PLACEMENT's `x_m`/`y_m` at one decimal; a record's
    own `x`/`y` can differ from the placement by up to the 0.5 m
    `resolve_cell` tolerates, which is enough to round differently and match
    nothing.
    """
    base = str(usd).rsplit("/", 1)[-1]
    name = base.rsplit(".", 1)[0] if "." in base else base
    return "%s@%.1f,%.1f" % (name, float(x_m), float(y_m))


def _uninstance_gprim_roots(stage, placements):
    """Un-instance every placement whose composed root prim is a GPRIM.

    THE BUG (2026-08-31, the 500 m fire city). `downtown_fire_500.yaml` turned
    on `instance_placements: true` to survive the composition OOM, and
    `scene_generator.apply_placements` then calls `SetInstanceable(True)` on
    every placement in a category no prune rule blocks — `house` included.

    USD instancing puts a prim's DESCENDANTS in the prototype and leaves the
    instance prim itself outside it. For the ordinary asset (an `Xform` root
    with meshes under it) that is exactly right. For an asset whose REFERENCED
    ROOT PRIM IS ITSELF A MESH — the Unreal/Muyang exports this repo already
    special-cases in `apply_placements` ("some assets have a Mesh as their
    root prim") — the prototype ends up holding only the GeomSubsets and the
    Looks, with NO Mesh in it, and Hydra draws the prototype. The building
    renders NOTHING while everything anchored to it (its roof props, its
    neighbours' shadows of it) stays. Measured on this city
    (`tools/fc_prop_orphan_probe.py` + a root-prim census of all 43 building
    assets): 10 of 75 buildings are Mesh-rooted — BG_Building_C and the eight
    `Building_Type{A,D}_{A,B}` low blocks that make up most of the NE lowrise
    quarter — and EXACTLY ONE of the 124 roof props stands on one of them:
    index 1354, the `roof_house` the user named.

    IT IS NOT ONLY THE BUILDINGS, and the second symptom looks different.
    Same city, same user, same day: "Some props like the street lights also
    look like they have no texture. What happened to it? same with some
    benches." These packs author EVERY MATERIAL AS A CHILD OF THE ROOT MESH
    (`/SM_lightpost_light_post_b/Section0/UnrealMaterial`, ...), and the
    per-section bindings live on `GeomSubset`s that are children of it too —
    so instancing separates the drawn points (which stay on the instance
    prim) from the subsets and materials that texture them (which go to the
    prototype). Whether a given asset then reads GREY or ABSENT is a Hydra
    population detail; both are this one cause and both are fixed here.

    MEASURED, by composing every asset in the run TWICE — plain and
    instanceable — and diffing the computed bound material of every mesh
    (`tools/fc_instance_material_probe.py`, run over
    `scene_gen/_scene_assets.tsv`, the provenance stamp of the failing run
    itself). 14 of the 110 distinct assets are gprim-rooted, and they account
    for **161 of the 1469 placements**:

        58  streetlight    SM_lightpost_light_post_b.usd   (ALL of them)
        50  bench          SM_bench_wood_a.usd             (the other 12 are
                           AEC ParkBench01, Xform-rooted and fine — which is
                           exactly the user's "SOME benches")
        16  car            Car_01_0.usd
        12  traffic_light  SM_light_streetlight_complete.usd
         7  planter        PlanterLarge_A.usd
         8  park_feature   SM_prop_fountain_full + 3 water layers
        10  house          BG_Building_C, Building_Type{A,D}_{A,B}

    The other 96 assets come back `ok` — every bind kept — so the pass is a
    scalpel, not a blanket un-instancing.

    THE RIGHT HOME FOR THIS IS `scene_generator.apply_placements`, which
    should never `SetInstanceable` a gprim-rooted prim; every other launcher
    on `instance_placements: true` still ships these 161. This is the
    launcher-side repair until that lands.

    Cheap and safe: an instance prim keeps its own composed type, so the test
    is local, and un-instancing 161 single-mesh props costs nothing against
    the 1308 placements instancing is actually there for.
    `FC_UNINSTANCE_GPRIM_ROOTS=0` disables it.
    """
    if not UNINSTANCE_GPRIM_ROOTS:
        return []
    fixed = []
    for p in placements:
        path = p.get("prim_path")
        if not path:
            continue
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim or not prim.IsValid() or not prim.IsInstance():
            continue
        if not prim.IsA(UsdGeom.Gprim):
            continue
        prim.SetInstanceable(False)
        fixed.append(p)
    if fixed:
        by_cat = {}
        for p in fixed:
            by_cat[p.get("category")] = by_cat.get(p.get("category"), 0) + 1
        print("[fc] un-instanced {0} placement(s) whose ROOT PRIM IS A MESH — "
              "an instanced gprim root leaves the prototype empty and the "
              "asset renders NOTHING ({1})".format(
                  len(fixed), ", ".join("{0}={1}".format(k, v)
                                        for k, v in sorted(
                                            by_cat.items(),
                                            key=lambda kv: str(kv[0])))))
        for p in fixed[:12]:
            print("[fc]    {0}  {1}".format(
                p.get("prim_path"), str(p.get("usd", "")).rsplit("/", 1)[-1]))
        if len(fixed) > 12:
            print("[fc]    ... and {0} more".format(len(fixed) - 12))
    else:
        print("[fc] un-instance pass: no placement has a gprim root "
              "(nothing to repair)")
    return fixed


def place_holder(stage, stem, x, y, z, yaw_deg, ssf):
    """THE FRESH HOLDER — see this module's docstring. Returns its path."""
    holder = "{0}/{1}".format(FIRE_ROOT, stem)
    xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x) * ssf, float(y) * ssf,
                                     float(z) * ssf))
    # THE PLACEMENT'S FINAL YAW, and only the yaw: the bake is already Z-up
    # and already in metres, so the roll/pitch `apply_placements` authors for
    # a Y-up intact asset must NOT be carried over.
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    # scale 1 IN METRES -> `ssf` in stage units (both are 1.0 on this stage;
    # authored as ssf so a stage with a different metersPerUnit is right too).
    xf.AddScaleOp().Set(Gf.Vec3f(float(ssf), float(ssf), float(ssf)))
    # REFERENCE ONTO A CHILD, not onto the Xform that carries the transform.
    kid = stage.DefinePrim(Sdf.Path(holder + "/bake"))
    return holder, kid


# ---------------------------------------------------------------------------
# The global emitter budget
# ---------------------------------------------------------------------------
#: rank 0 burns hardest and is served first.
STATE_RANK = {"flame": 0, "smoulder": 1}


def _live(ev):
    return all(not (o.get("e") or {}).get("dead") for o in (ev.get("ops") or []))


def fire_state(doc, events):
    """`(state, wisp_only)` exactly as `place_fire` decides it — including
    the F1 case, where the level has no ACTIVE state but `soot_plume` gave it
    one `smoulder` event and it becomes a wisp."""
    state = ((doc or {}).get("fire") or {}).get("state")
    if state:
        return state, False
    if any(ev.get("state") == "smoulder" and ev.get("ops")
           for ev in (events or [])):
        return "smoulder", True
    return None, False


def emitter_estimate(doc, events, max_emitters, smoke,
                     side_smoke_flame_max=None, side_smoke_nonflame_max=None,
                     roof_cap_intact=None, roof_cap_collapsed=None,
                     flame_min_clusters=None, flame_extra_max=None,
                     flame_size_scaling=False, smoke_size_scaling=False,
                     smoke_window_jets=False, residual_flame_frac=0.0):
    """Predicted emitter count for `place_fire(..., max_emitters=...)`.

    Mirrors `fire_assembly_lib.place_fire`'s branch structure exactly,
    counting instead of authoring, so the allocator prices a building the
    way it will actually cost. An UPPER BOUND: `_flame_sources` can author
    fewer than `per_opening` prims if a source fails to create, which is why
    the predicted and the actual counts are both printed per building.

    `side_smoke_flame_max`/`side_smoke_nonflame_max`/`roof_cap_intact`/
    `roof_cap_collapsed` — see `place_fire`'s own docstring for what each
    caps; `None` (every call before 2026-08-31) reproduces the ORIGINAL
    fixed budgets exactly (`uf.SMOKE_EXTRA_MAX`/`spl.SMOULDER_EVENTS_MAX`/2/
    2), so an existing caller that never passes them sees no change.
    `roof_has_collapsed`/`flame_window_target`/`smoke_window_target` (need
    `fal`, not otherwise used by this function) are only consulted once a
    caller actually opts into the knob that needs them — a bare
    `emitter_estimate(doc, events, n, smoke)` call, exactly as tested, never
    needs `fal` in scope at all.

    `flame_min_clusters`/`flame_extra_max`/`flame_size_scaling`/`smoke_
    size_scaling`/`smoke_window_jets`/`residual_flame_frac` — see `place_
    fire`'s own docstring; all default to their OFF value (`None`/`False`/
    `0.0`), reproducing the exact pre-2026-08-31 counts.
    """
    f = (doc or {}).get("fire") or {}
    state, wisp = fire_state(doc, events)
    zero = {"flame": 0, "smoke": 0, "interior": 0, "roof": 0, "openings": 0,
            "total": 0}
    if not state:
        return dict(zero)
    evs = [ev for ev in (events or []) if _live(ev) and ev.get("ops")]
    is_flame = state == "flame"
    n_st = int(f.get("n_storeys") or 0)
    if flame_size_scaling:
        max_open = min(max_emitters, fal.flame_window_target(n_st))
    else:
        max_open = (max(max_emitters, min(16, n_st // 2)) if n_st >= 12
                    else max_emitters)
    n_flame = n_open = 0
    lit_groups = set()
    for ev in [e for e in evs if e["state"] == "flame"]:
        for _op in ev["ops"]:
            if n_open >= max_open:
                break
            n_flame += uf.FLAME_PER_OPENING
            lit_groups.add((ev.get("side"), ev.get("storey")))
            n_open += 1
    residual_on = bool(residual_flame_frac) and float(residual_flame_frac) > 0.0
    if state == "smoulder" and not wisp and not residual_on:
        for ev in [e for e in evs if e["state"] == "smoulder"]:
            for _op in ev["ops"]:
                if n_open >= max(2, max_open // 2):
                    break
                n_flame += max(1, uf.FLAME_PER_OPENING - 1)
                n_open += 1
    # MULTIPLE FLAME CLUSTERS — mirrors `place_fire`'s own extra-cluster
    # top-up exactly (counting, not authoring).
    if is_flame and flame_min_clusters is not None:
        extra_max = fal.FLAME_EXTRA_MAX if flame_extra_max is None \
            else int(flame_extra_max)
        extra_budget = min(max(0, max_open - n_open), max(0, extra_max))
        if len(lit_groups) < int(flame_min_clusters) and extra_budget > 0:
            n_extra = 0
            for ev in [e for e in evs if e["state"] == "out"]:
                for _op in ev["ops"]:
                    if n_extra >= extra_budget:
                        break
                    n_flame += max(1, uf.FLAME_PER_OPENING - 1)
                    n_open += 1
                    n_extra += 1
                if n_extra >= extra_budget:
                    break
    # RESIDUAL FLAME POCKETS — mirrors `place_fire`'s `_scattered_selection_
    # order` (AT MOST ONE OPENING PER EVENT), so this counts EVENTS, not
    # total ops, unlike every other counting loop here.
    if not is_flame and not wisp and residual_on:
        residual_target = max(
            fal.RESIDUAL_FLAME_MIN,
            int(round(float(residual_flame_frac)
                      * fal.flame_window_target(n_st))))
        n_sm_pockets = len([e for e in evs if e["state"] == "smoulder"])
        n_residual_sm = min(n_sm_pockets, residual_target)
        n_residual_out = 0
        if n_residual_sm < residual_target:
            n_out_pockets = len([e for e in evs if e["state"] == "out"])
            n_residual_out = min(n_out_pockets, residual_target - n_residual_sm)
        n_residual = n_residual_sm + n_residual_out
        n_flame += n_residual
        n_open += n_residual
    if not smoke:
        return {"flame": n_flame, "smoke": 0, "interior": 0, "roof": 0,
                "openings": n_open, "total": n_flame}
    if wisp:
        n = len([e for e in evs if e["state"] == "smoulder"][:2])
        return {"flame": 0, "smoke": n, "interior": 0, "roof": 0,
                "openings": 0, "total": n}
    side_flame_cap = (fal.smoke_window_target(n_st) if smoke_size_scaling
                      else (uf.SMOKE_EXTRA_MAX if side_smoke_flame_max is None
                            else max(0, int(side_smoke_flame_max))))
    side_nonflame_cap = (fal.smoke_window_target(n_st) if smoke_size_scaling
                         else (spl.SMOULDER_EVENTS_MAX
                               if side_smoke_nonflame_max is None
                               else max(0, int(side_smoke_nonflame_max))))
    out_fill_enabled = (side_smoke_nonflame_max is not None
                       or smoke_size_scaling)
    if is_flame:
        if smoke_window_jets:
            # OPENING-LEVEL: counts every op across candidate events, capped
            # — can exceed the event-level count below when a few events
            # each carry several openings, which is the whole point (more
            # window jets from the SAME compartments, not more compartments).
            n_smoke = 0
            for ev in [e for e in evs if e["state"] == "out"]:
                for _op in ev["ops"]:
                    if n_smoke >= side_flame_cap:
                        break
                    n_smoke += 1
                if n_smoke >= side_flame_cap:
                    break
        else:
            n_smoke = len([e for e in evs
                          if e["state"] == "out"][:side_flame_cap])
    else:
        # SMOULDER FIRST, "OUT" TO FILL THE REST — must mirror `place_fire`'s
        # own two-pass pick exactly, INCLUDING the same gate: the "out" fill
        # only happens once a caller explicitly customises
        # `side_smoke_nonflame_max` OR turns on `smoke_size_scaling` — never
        # for a bare call, which is what keeps this estimator honest for a
        # caller that never opts in. Event ids are unique across the WHOLE
        # events list (`fire_bake.events_to_json`'s `enumerate`), so a
        # smoulder id and an "out" id never collide and the fill-in never has
        # to exclude anything already picked.
        if smoke_window_jets:
            n_smoke = 0
            for ev in [e for e in evs if e["state"] == "smoulder"]:
                for _op in ev["ops"]:
                    if n_smoke >= side_nonflame_cap:
                        break
                    n_smoke += 1
                if n_smoke >= side_nonflame_cap:
                    break
            if out_fill_enabled and n_smoke < side_nonflame_cap:
                for ev in [e for e in evs if e["state"] == "out"]:
                    for _op in ev["ops"]:
                        if n_smoke >= side_nonflame_cap:
                            break
                        n_smoke += 1
                    if n_smoke >= side_nonflame_cap:
                        break
        else:
            n_sm = min(len([e for e in evs if e["state"] == "smoulder"]),
                      side_nonflame_cap)
            n_out = 0
            if out_fill_enabled:
                n_out = min(len([e for e in evs if e["state"] == "out"]),
                           max(0, side_nonflame_cap - n_sm))
            n_smoke = n_sm + n_out
    seats = (doc or {}).get("seats") or {}
    # LAST RESORT: mirrors `place_fire`'s own fallback exactly — a building
    # with zero flame AND zero side smoke still gets its interior seats, so
    # the estimator prices what a starved-events building (e.g. `gac_
    # SM_Building_29_F3_o7_ENW_s808` in `city_138`) will actually author
    # instead of predicting 0 for a building that is not, in fact, empty.
    n_int = (len(seats.get("interior") or [])
            if (not is_flame or (n_flame == 0 and n_smoke == 0)) else 0)
    n_roof = 0
    if f.get("roof"):
        roof_cap = 2
        if roof_cap_intact is not None or roof_cap_collapsed is not None:
            collapsed = fal.roof_has_collapsed(doc or {})
            roof_cap = (roof_cap_collapsed if collapsed else roof_cap_intact)
            roof_cap = 2 if roof_cap is None else max(0, int(roof_cap))
        n_roof = min(len(seats.get("roof") or []), roof_cap)
    return {"flame": n_flame, "smoke": n_smoke, "interior": n_int,
            "roof": n_roof, "openings": n_open,
            "total": n_flame + n_smoke + n_int + n_roof}


def allocate_emitters(rows, budget, cap, smoke, side_smoke_flame_max=None,
                      side_smoke_nonflame_max=None, roof_cap_intact=None,
                      roof_cap_collapsed=None, flame_min_clusters=None,
                      flame_extra_max=None, flame_size_scaling=False,
                      smoke_size_scaling=False, smoke_window_jets=False,
                      residual_flame_frac=0.0):
    """Spend `budget` emitters across `rows`, worst building first.

    Sets `row["alloc"]` (the `max_emitters` to pass `place_fire`, or `None`
    for "geometry and baked soot only") and `row["est"]`. Returns the
    predicted total. See this module's docstring for the rule; the ranking is
    state first (flame > smoulder > wisp/residual), then height, then
    manifest order, so the decision is deterministic.

    `side_smoke_*`/`roof_cap_*`/`flame_*`/`smoke_*`/`residual_flame_frac`
    are forwarded to every `emitter_estimate` call below, unchanged — see
    that function's own docstring. WHEN `flame_size_scaling` IS ON, `cap`
    (the ceiling `step` climbs to) should be `fal.FLAME_WINDOWS_MAX`, not
    the flat `FA_EMITTERS` value — see `emitter_estimate`'s own `min(step,
    flame_window_target(n_storeys))`: a short building's count PLATEAUS
    once `step` exceeds its own target (the `delta <= 0` branch below marks
    it "free" and stops charging it), which is what lets the freed budget
    keep a tall building's `step` climbing across the rest of the range.
    """
    live = [r for r in rows if r.get("doc")]
    for r in rows:
        if not r.get("doc"):
            r["alloc"] = None
            r["est"] = {"total": 0}
            r["drop_reason"] = "no sidecar — geometry only, no emitters"
    for r in live:
        st, wisp = fire_state(r["doc"], r["events"])
        r["state"] = ("wisp" if wisp else st) or "none"
        r["rank"] = 2 if (wisp or not st) else STATE_RANK.get(st, 2)
        r["n_st"] = int((r["doc"].get("fire") or {}).get("n_storeys") or 0)
        r["alloc"] = None
        # zeros, NOT `emitter_estimate(..., 0, ...)`: a tall building's
        # opening FLOOR is non-zero even at max_emitters=0, and printing it
        # against a building that was dropped would claim emitters that were
        # never authored.
        r["est"] = {"flame": 0, "smoke": 0, "interior": 0, "roof": 0,
                    "openings": 0, "total": 0}
    order = sorted(live, key=lambda r: (r["rank"], -r["n_st"], r["i"]))
    spent = 0
    # 1) one opening each, in rank order — the floor a tall building cannot
    #    go below is priced here too, so a building that does not fit is
    #    dropped whole rather than quietly given a fire nobody can see.
    for r in order:
        est = emitter_estimate(r["doc"], r["events"], 1, smoke,
                               side_smoke_flame_max, side_smoke_nonflame_max,
                               roof_cap_intact, roof_cap_collapsed,
                               flame_min_clusters, flame_extra_max,
                               flame_size_scaling, smoke_size_scaling,
                               smoke_window_jets, residual_flame_frac)
        if est["total"] == 0:
            r["alloc"], r["est"] = 1, est          # nothing to spend anyway
            continue
        if spent + est["total"] <= budget:
            r["alloc"], r["est"] = 1, est
            spent += est["total"]
        else:
            r["drop_reason"] = ("global emitter budget spent ({0}/{1}) before "
                                "this building's minimum {2}"
                                .format(spent, budget, est["total"]))
    # 2) top up, one opening at a time, again in rank order
    for step in range(2, max(2, cap) + 1):
        for r in order:
            if r["alloc"] is None:
                continue
            est = emitter_estimate(r["doc"], r["events"], step, smoke,
                                   side_smoke_flame_max,
                                   side_smoke_nonflame_max,
                                   roof_cap_intact, roof_cap_collapsed,
                                   flame_min_clusters, flame_extra_max,
                                   flame_size_scaling, smoke_size_scaling,
                                   smoke_window_jets, residual_flame_frac)
            delta = est["total"] - r["est"]["total"]
            if delta <= 0:
                r["alloc"] = step                  # free: no more openings
                continue
            if spent + delta <= budget:
                r["alloc"], r["est"] = step, est
                spent += delta
    return spent


# ---------------------------------------------------------------------------
# The Kit log — Flow fails SILENTLY
# ---------------------------------------------------------------------------
def grep_kit_log(tail_bytes=8 * 1024 * 1024):
    """`(path, {needle: count})` for the newest Kit log, or `(None, {})`.

    Flow keeps reporting success when its block pool is exhausted, so a
    capture is only worth looking at once this comes back clean. Only the
    tail is read — a Kit log runs to hundreds of MB.
    """
    cands = []
    for pat in KIT_LOG_GLOBS:
        for p in _glob.glob(pat):
            try:
                cands.append((os.path.getmtime(p), p))
            except OSError:
                pass
    if not cands:
        return None, {}
    cands.sort(reverse=True)
    path = cands[0][1]
    hits = {n: 0 for n in FLOW_OOM_NEEDLES}
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as fh:
            if size > tail_bytes:
                fh.seek(size - tail_bytes)
            blob = fh.read().decode("utf-8", "replace")
        for n in FLOW_OOM_NEEDLES:
            hits[n] = blob.count(n)
    except Exception as exc:
        print("[fc] could not read the Kit log {0}: {1}".format(path, exc))
    return path, hits


# ---------------------------------------------------------------------------
class FireCityApp:
    def __init__(self):
        self.t0 = time.time()
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()
        self.vram = {}
        self.rows = []
        self.n_hidden = 0

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        env_mode = FC_ENV.lower()
        loaded_env_url = None
        if env_mode in ("none", "0", "off", "skip"):
            print("[fc] FC_ENV=none — skipping PegasusInterface()."
                  "load_environment(); the city authors its own ground "
                  "(scene_generator.apply_ground_planes, under PARENT + "
                  "'/ground') and sky (add_sky), so nothing this launcher "
                  "runs downstream should need the Pegasus environment")
        else:
            loaded_env_url = ENV_URL if env_mode in ("", "default") else FC_ENV
            pg.load_environment(loaded_env_url)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if loaded_env_url is not None and not _wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")
        self.stage = stage
        _remove_env_clutter(stage)
        if stage.GetPrimAtPath("/World/stage").IsValid():
            scale_stage_prim(stage, "/World/stage", 1.0)
            for _ in range(10):
                omni.kit.app.get_app().update()
        _, self.ssf = get_stage_meters_per_unit(stage)
        if abs(self.ssf - 1.0) > 1e-9:
            print("[fc] NOTE: this stage is {0} units/m — a bake is authored "
                  "in METRES, so the holders carry scale {0} rather than the "
                  "plan's literal 1.0".format(self.ssf))
        if loaded_env_url is None:
            env_tag = "empty stage (no Pegasus env, FC_ENV=none)"
        elif loaded_env_url == ENV_URL:
            env_tag = "empty stage (Pegasus env loaded)"
        else:
            env_tag = "empty stage (Pegasus env {0} loaded)".format(loaded_env_url)
        self.vram["empty"] = vram_mb(env_tag)

        # -- 1) the city, exactly as `scene_launch_script` builds it --------
        # `disaster-type: fire` is spec vocabulary for the spread solver, not
        # a `compile_disaster.DISASTERS` entry, so it is compiled as `none`.
        # Legitimate because `build_city`/`generate_scene_on_stage` never read
        # `config["disaster"]` for placement — the damage comes from the
        # BAKES, not from a compiled field.
        print("[fc] compiling {0} with disaster-type overridden to 'none' "
              "(the fire is the manifest, not a compiled field)"
              .format(SCENE_CONFIG))
        config = load_scene_config(SCENE_CONFIG,
                                   spec_overrides={"disaster-type": "none"})
        self.config = config
        self.wind = preset_fire_block(SCENE_CONFIG)
        print("[fc] wind: heading {0} deg (blowing TOWARD), {1} m/s; "
              "epicenter {2}; duration {3} s x start_offset {4}".format(
                  self.wind["heading_deg"], self.wind["wind_mps"],
                  self.wind["epicenter"], self.wind["duration_s"],
                  self.wind["start_offset_frac"]))
        t_layout = time.time()
        self.placements = generate_scene_on_stage(
            stage, config, parent_path=PARENT, scene_scale_factor=self.ssf)
        # BEFORE the first update: an instanced gprim root renders nothing
        # (see `_uninstance_gprim_roots`), and the repair has to be in place
        # before anything downstream measures or photographs the city.
        self.uninstanced = _uninstance_gprim_roots(stage, self.placements)
        for _ in range(10):
            omni.kit.app.get_app().update()
        settle_rigid_props(
            stage,
            [p["prim_path"] for p in self.placements
             if p.get("settle") and p.get("prim_path")],
            ground_path=PARENT + "/ground")
        if SKY in ("", "sunset"):
            # UNCHANGED -- byte-identical to every fire-city render before
            # FC_SKY existed. See sky_presets.py's module docstring for why
            # this dome-only path is what "sunset" means HERE specifically
            # (no sun at all, not a literal low-sun look like the bench's).
            add_sky(stage, resolve_sky(config),
                    intensity=float(config.get("sky_intensity", 3500.0)),
                    exposure=float(config.get("sky_exposure", -3.0)))
            _disable_sky_sun(stage)
        else:
            # Still veto a borrowed stage's own sun (none of today's presets
            # set `sky:`, but a future one might) before authoring ours, so
            # there is never a second, unaccounted-for DistantLight.
            _disable_sky_sun(stage)
            resolved_sky = apply_sky_preset(stage, SKY, prefix="fc")
            print("[fc] sky: FC_SKY={0} -> preset '{1}'".format(
                SKY, resolved_sky))
        for _ in range(10):
            omni.kit.app.get_app().update()
        self.t_layout = time.time() - t_layout
        self.region = [float(v) for v in config["layout"]["region_m"]]
        self.n_houses = sum(1 for p in self.placements
                            if p.get("category") == "house")
        print("[fc] city: {0:.0f} x {1:.0f} m, {2} placement(s), {3} "
              "building(s), layout in {4:.0f} s".format(
                  self.region[0], self.region[-1], len(self.placements),
                  self.n_houses, self.t_layout))
        if INTACT_ONLY:
            # THE FIX FOR THE 2026-08-30 MANIFEST/CITY MISMATCH: dump the
            # layout Kit just built (real Nucleus footprints, real packing
            # decisions) so `tools/fire_city_dry_run.py --placements-json`
            # solves the fire on THIS city, not a host-side reconstruction
            # that can pack differently and describe a city that never
            # existed. See FC_DUMP in this module's own docstring.
            dump_path = DUMP_PATH or default_dump_path(
                SCENE_CONFIG, config.get("seed", 0))
            dump_city_placements(dump_path, SCENE_CONFIG, config.get("seed", 0),
                                 config, self.placements,
                                 config.get("_city_layout") or {})
        self.vram["intact"] = vram_mb(
            "city built INTACT ({0} buildings)".format(self.n_houses))

    # -- 2/3) the manifest, the bakes, the swap ----------------------------
    def load_fire(self):
        path = find_manifest(MANIFEST)
        if not path or not os.path.exists(path):
            raise RuntimeError(
                "FC_MANIFEST={0!r} not found and no scene_gen/_plans/"
                "fire_city_*.json to fall back on — run tools/"
                "fire_city_dry_run.py first (plan work item #4)".format(MANIFEST))
        top_seed, records = fcm.load_manifest(path)
        seed = fcm.resolve_city_seed(path, top_seed)
        print("[fc] manifest {0}: {1} record(s), city seed {2}".format(
            path, len(records), seed))
        self.manifest_path, self.city_seed = path, seed

        # THE DISTRICT RULE, re-asserted at assembly time. The dry run proved
        # it; a hand-edited manifest can still break it, and a burning tower
        # is the one thing this scene is defined by NOT having.
        tally = {}
        offenders = []
        for i, rec in enumerate(records):
            t = rec.get("typology")
            tally[t] = tally.get(t, 0) + 1
            if t in ufc.NO_FIRE_TYPOLOGIES:
                offenders.append((i, t, rec.get("asset") or rec.get("style")))
        print("[fc] damaged by typology: {0}".format(
            ", ".join("{0}={1}".format(k, v) for k, v in sorted(
                tally.items(), key=lambda kv: str(kv[0])))))
        if offenders:
            print("[fc] *** DISTRICT RULE VIOLATED: {0} record(s) sit in a "
                  "no-fire district {1} — {2}".format(
                      len(offenders), ufc.NO_FIRE_TYPOLOGIES, offenders))

        rows = []
        missing = []
        for i, (rec, usd, js, stem) in enumerate(
                bake_paths(records, self.city_seed, BAKES)):
            if not usd:
                missing.append((i, stem, rec.get("asset") or rec.get("style"),
                                rec.get("level")))
                continue
            doc = masses = events = None
            if js:
                try:
                    doc, masses, events = fb.load_for_assembly(js)
                except Exception as exc:
                    print("[fc] sidecar {0} unreadable ({1}) — geometry only"
                          .format(js, exc))
            rows.append({"i": i, "rec": rec, "usd": usd, "json": js,
                         "stem": stem, "doc": doc or {},
                         "masses": masses or {}, "events": events or []})
        if missing:
            print("\n[fc] " + "!" * 70)
            print("[fc] {0} of {1} manifest record(s) HAVE NO BAKE — those "
                  "buildings stay INTACT in this scene:".format(
                      len(missing), len(records)))
            for i, stem, name, level in missing:
                print("[fc]    record {0:<3} {1:<24} {2:<5} -> no {3}.usd"
                      .format(i, name, level, stem))
            print("[fc]   run  scene_gen/tools/fire_city_bake.sh {0}"
                  .format(self.manifest_path))
            print("[fc] " + "!" * 70 + "\n")
        self.rows = rows
        self.missing = missing
        return rows

    def hide_companion_props(self, cell):
        """Hide the roof/wall props that belong to the building at `cell`.

        See `prop_tag` for the rule and why it is identity on the `of` tag
        rather than a radius. Returns the list of prop placements hidden.
        """
        if not HIDE_PROPS:
            return []
        bld = self._placement_by_path.get(cell)
        if bld is None:
            return []
        tag = prop_tag(bld.get("usd", ""), bld.get("x_m", 0.0),
                       bld.get("y_m", 0.0))
        out = []
        for p in self._props_by_tag.get(tag, ()):
            path = p.get("prim_path")
            if path and hide_intact(self.stage, path):
                out.append(p)
            elif path:
                print("[fc] WARNING: could not hide companion prop {0}"
                      .format(path))
        return out

    def compose_bakes(self):
        """Hide each intact cell and reference its bake under a fresh holder."""
        stage = self.stage
        UsdGeom.Xform.Define(stage, Sdf.Path(FIRE_ROOT))
        # THE COMPANION-PROP INDEX (see `prop_tag`). Built once: `of` is the
        # only explicit building->prop link in a placement list, and it is
        # written by `gac_props._place` on every prop it authors.
        self._placement_by_path = {p["prim_path"]: p for p in self.placements
                                   if p.get("prim_path")}
        self._props_by_tag = {}
        for p in self.placements:
            t = p.get("of")
            if t:
                self._props_by_tag.setdefault(t, []).append(p)
        print("[fc] companion props: {0} placement(s) carry an `of` tag "
              "across {1} building(s) (FC_HIDE_PROPS={2})".format(
                  sum(len(v) for v in self._props_by_tag.values()),
                  len(self._props_by_tag), int(HIDE_PROPS)))
        self.hidden_props = []
        self._hidden_cells = []
        placed, refused = [], []
        n_cell_matched = 0
        for r in self.rows:
            rec = r["rec"]
            cell, how = resolve_cell(stage, self.placements, rec)
            if not cell:
                print("[fc] *** record {0} ({1}): {2}".format(
                    r["i"], r["stem"], how))
                print("[fc]     refusing to place the bake — a burnt shell "
                      "inside an intact one z-fights and reads as a "
                      "different bug entirely")
                r["skip"] = "cell not found"
                refused.append(r)
                continue
            n_cell_matched += 1
            x = float(rec.get("x", 0.0))
            y = float(rec.get("y", 0.0))
            z = float(rec.get("z", 0.0))
            yaw = float(rec.get("yaw_deg", 0.0))
            # the bake's own record of the cell it replaces, if the bake
            # driver wrote one — a disagreement means the manifest and the
            # bake are from different solves.
            city = (r["doc"] or {}).get("city") or {}
            if city:
                dx = math.hypot(float(city.get("x", x)) - x,
                                float(city.get("y", y)) - y)
                dyaw = abs(float(city.get("yaw_deg", yaw)) - yaw)
                if dx > 0.5 or dyaw > 1.0:
                    print("[fc] WARNING {0}: the bake's own sidecar says it "
                          "belongs at ({1:.1f}, {2:.1f}) yaw {3:.0f}, the "
                          "manifest says ({4:.1f}, {5:.1f}) yaw {6:.0f} — "
                          "different solves?".format(
                              r["stem"], city.get("x"), city.get("y"),
                              city.get("yaw_deg", 0.0), x, y, yaw))
            if hide_intact(stage, cell):
                self.n_hidden += 1
                self._hidden_cells.append(cell)
            else:
                print("[fc] WARNING: could not hide {0}".format(cell))
            # ... AND ITS COMPANION PROPS, which are siblings of the cell,
            # not children of it, so hiding the cell leaves them hovering
            # over a roof that is no longer drawn.
            r["hidden_props"] = self.hide_companion_props(cell)
            self.hidden_props += r["hidden_props"]
            if r["hidden_props"]:
                by_cat = {}
                for q in r["hidden_props"]:
                    by_cat[q.get("category")] = by_cat.get(q.get("category"),
                                                           0) + 1
                print("[fc]     + {0} companion prop(s) hidden with it ({1})"
                      .format(len(r["hidden_props"]),
                              ", ".join("{0}={1}".format(k, v) for k, v
                                        in sorted(by_cat.items(),
                                                  key=lambda kv: str(kv[0])))))
            holder, kid = place_holder(stage, r["stem"], x, y, z, yaw, self.ssf)
            if not kid.GetReferences().AddReference(r["usd"]):
                print("[fc] *** FAILED to reference {0}".format(r["usd"]))
                r["skip"] = "reference failed"
                refused.append(r)
                continue
            stage.Load(Sdf.Path(holder + "/bake"))
            r["prim"], r["cell"], r["cell_how"] = holder, cell, how
            r["x"], r["y"], r["yaw"] = x, y, yaw
            r["bbox"] = fal.bbox(stage, holder)
            b = r["bbox"]
            print("[fc] d{0:<2} ({1:+7.1f},{2:+7.1f}) yaw {3:+4.0f}  {4:<30} "
                  "{5:<4} {6}  [cell: {7}]".format(
                      r["i"], x, y, yaw, r["stem"],
                      (r["doc"].get("level") or "?"),
                      ("bbox {0:.0f}x{1:.0f}x{2:.0f} m".format(
                          b[3] - b[0], b[4] - b[1], b[5] - b[2])
                       if b else "*** EMPTY BBOX ***"), how))
            if r["doc"].get("src_kept"):
                print("[fc]     NOTE: this bake still carries its merged "
                      "source subtree (src_kept) — it composes and costs "
                      "memory")
            placed.append(r)
            for _ in range(2):
                omni.kit.app.get_app().update()
        for _ in range(10):
            omni.kit.app.get_app().update()
        self.placed, self.refused = placed, refused
        print("[fc] {0} bake(s) composed, {1} refused, {2} intact prim(s) "
              "hidden ({3}), {4} companion prop(s) hidden with them".format(
                  len(placed), len(refused), self.n_hidden, HIDE_MODE,
                  len(self.hidden_props)))
        # THE ORPHAN CHECK. Anything still `of`-tagged to a hidden building
        # after this pass is a prop the rule missed, and it will be standing
        # in mid-air. Zero is the only acceptable number.
        hidden_tags = set()
        for cell in self._hidden_cells:
            bld = self._placement_by_path.get(cell)
            if bld:
                hidden_tags.add(prop_tag(bld.get("usd", ""),
                                         bld.get("x_m", 0.0),
                                         bld.get("y_m", 0.0)))
        done = {p.get("prim_path") for p in self.hidden_props}
        left = [p for t in hidden_tags for p in self._props_by_tag.get(t, ())
                if p.get("prim_path") not in done]
        if left:
            print("[fc] *** {0} prop(s) still tagged to a HIDDEN building — "
                  "they are floating: {1}".format(
                      len(left), ", ".join(str(p.get("prim_path"))
                                           for p in left[:8])))
        # THE REGRESSION CHECK FOR THE 2026-08-30 INCIDENT: with a manifest
        # derived from `FC_DUMP`/`--placements-json`, Kit-to-Kit determinism
        # (same preset/seed -> same layout) means every record's cell should
        # resolve — a mismatch here means the manifest was NOT solved on
        # this city.
        print("[fc] manifest/city match: {0}/{1} record(s) found their "
              "placement cell in the layout this launcher just built"
              .format(n_cell_matched, len(self.rows)))
        self.vram["bakes"] = vram_mb(
            "{0} bake(s) composed, no Flow".format(len(placed)))

    # -- 3b) scorched vegetation near a burning building (2026-08-31) ------
    def scorch_vegetation(self):
        """Darken every street tree / greenery placement within reach of a
        damaged building — see `fire_assembly_lib.scorch_vegetation_pass`
        for the radius rule and the material reuse. Runs on `self.placed`
        (composed bakes only), independent of the Flow emitter budget: a
        building whose fire did not get live flames because the budget ran
        out still did real damage and still scorches what is next to it.
        """
        if not SCORCH_VEG or not self.placed:
            self.veg_scorch = {"trees": 0, "leaf_binds": 0,
                               "trunk_binds": 0, "fuels_total": 0,
                               "scorched": 0, "targets": {}}
            return
        stats = fal.scorch_vegetation_pass(self.stage, self.placements,
                                           self.placed, FIRE_ROOT)
        self.veg_scorch = stats
        print("[fc] scorched vegetation: {0}/{1} fuel placement(s) within "
              "reach of a burning building ({2} tree(s) touched, {3} leaf "
              "bind(s), {4} trunk bind(s))".format(
                  stats["scorched"], stats["fuels_total"], stats["trees"],
                  stats["leaf_binds"], stats["trunk_binds"]))
        for path, t in sorted(stats["targets"].items(),
                              key=lambda kv: kv[1]["dist_m"])[:8]:
            print("[fc]   d{0:<3} {1:<5} {2:5.1f} m of {3:5.1f} m radius  "
                  "{4}".format(t["i"], t["level"], t["dist_m"],
                              t["radius_m"], path))
        self.vram["scorch_veg"] = vram_mb(
            "vegetation scorched ({0} tree(s))".format(stats["trees"]))

    # -- 3c) fire-side debris apron, non-collapse burning buildings --------
    def fire_debris_apron(self):
        """Small ground-debris apron along the venting sides of every F1-F5
        (non-collapse) burning building — see `fire_assembly_lib.
        fire_apron_pass`. One merged Mesh per qualifying building, so the
        prim cost is bounded regardless of how many lumps it carries.
        """
        if not FIRE_APRON or not self.placed:
            self.aprons = []
            return
        rows = fal.fire_apron_pass(self.stage, FIRE_ROOT, self.placed,
                                   seed=SEED, scale=APRON_SCALE)
        self.aprons = rows
        n_lumps = sum(r["n"] for r in rows)
        n_buildings = sum(1 for r in rows if r.get("prim"))
        print("[fc] fire-side debris apron: {0} lump(s) on {1} building(s), "
              "{2} mesh prim(s) ({3} building(s) not gated in: collapse or "
              "unburnt level)".format(n_lumps, n_buildings, n_buildings,
                                      len(rows) - n_buildings))
        for r in rows:
            if not r.get("prim"):
                continue
            print("[fc]   d{0:<3} {1:<30} {2:<4} {3:>3} lump(s) on side(s) "
                  "{4}".format(r["i"], r["stem"], r["level"], r["n"],
                              "/".join(r["sides"])))
        self.vram["apron"] = vram_mb(
            "fire debris apron ({0} lump(s), {1} mesh(es))".format(
                n_lumps, n_buildings))

    # -- 4) the fire, under a GLOBAL emitter budget -------------------------
    @staticmethod
    def people_records(doc):
        """The record list out of whatever shape the people JSON is in.

        THE 2026-08-31 CRASH. `tools/fire_people_dry_run.py` writes
        ``{"meta", "people", "census", "refused", "dropped", "degraded"}`` —
        the records are under **`people`**, and there is no `records` key at
        all. This launcher handed the whole dict to
        `fire_people.to_placements`, whose fallback is ``list(
        plan_or_records)``; on a dict that yields its KEYS, so `_convertible`
        got the string ``"meta"`` and raised

            AttributeError: 'str' object has no attribute 'get'

        which propagated out of `run()` and killed the launch AFTER the whole
        city, all 20 bakes and all 247 emitters were up — no people, and no
        captures either, because `capture()` and `banner()` never ran. The
        pane's own record of it: "URBAN FIRE CITY FAILED: 'str' object has no
        attribute 'get'" at 50.4 s of app uptime.

        `people` first, then `records`, then a bare list. Anything else is a
        loud, named error rather than a shape guess.
        """
        if isinstance(doc, dict):
            for key in ("people", "records"):
                v = doc.get(key)
                if isinstance(v, list):
                    return v, key
            raise TypeError(
                "people JSON is a dict with keys {0} — expected a `people` or "
                "`records` list (tools/fire_people_dry_run.py writes "
                "`people`)".format(sorted(doc)))
        if isinstance(doc, list):
            return doc, "(bare list)"
        raise TypeError("people JSON is a {0}; expected a list of records or "
                        "a dict carrying one".format(type(doc).__name__))

    def people_in_the_fire(self, recs):
        """"Only keep humans that are in the disaster" (user, 2026-08-31).

        `(kept, dropped)`, filtering on the distance to the nearest BURNING
        BUILDING'S FOOTPRINT — not its centre. The centre is the wrong datum
        on this scene by a wide margin: the four onlookers watching
        `SM_Building_31` stand 122 m from its centre and 59 m from its wall,
        because that building is 149 x 64 m in plan. Measured over
        `_plans/fire_people_final.json` x `_plans/fire_city_500m.json`: every
        one of the 89 records is within **59.5 m** of a burning footprint
        (median 13.8 m), so the default 120 m drops nothing here.

        THIS IS A GUARD, NOT THE MECHANISM. The records already come from a
        solve that seeded every figure against a specific burning building
        (`building_i`, and all 89 of this file's point at a manifest record),
        so there are no background pedestrians to remove — the filter exists
        so a hand-edited or re-solved records file cannot quietly scatter
        onlookers across a city that is only burning in one corner.
        `FC_PEOPLE_MAX_DIST_M=0` disables it.
        """
        if PEOPLE_MAX_DIST_M <= 0.0:
            return list(recs), []
        burning = []
        for r in (getattr(self, "placed", None) or []):
            rec = r.get("rec") or {}
            burning.append((float(rec.get("x", 0.0)), float(rec.get("y", 0.0)),
                            float(rec.get("W", 0.0)), float(rec.get("D", 0.0)),
                            math.radians(float(rec.get("yaw_deg", 0.0)))))
        if not burning:
            print("[fc] people: nothing is burning on this stage — the "
                  "distance filter has no datum, keeping all {0} record(s)"
                  .format(len(recs)))
            return list(recs), []
        kept, dropped = [], []
        for rec in recs:
            px, py = float(rec.get("x", 0.0)), float(rec.get("y", 0.0))
            best = 1e18
            for cx, cy, W, D, a in burning:
                dx, dy = px - cx, py - cy
                ca, sa = math.cos(a), math.sin(a)
                u = ca * dx + sa * dy
                v = -sa * dx + ca * dy
                d = math.hypot(max(0.0, abs(u) - W / 2.0),
                               max(0.0, abs(v) - D / 2.0))
                if d < best:
                    best = d
            (kept if best <= PEOPLE_MAX_DIST_M else dropped).append(
                (rec, best))
        if dropped:
            print("[fc] people: {0} record(s) DROPPED — further than {1:.0f} m "
                  "(FC_PEOPLE_MAX_DIST_M) from any burning building's "
                  "footprint".format(len(dropped), PEOPLE_MAX_DIST_M))
            for rec, d in sorted(dropped, key=lambda kv: -kv[1])[:8]:
                print("[fc]    id {0} {1:<14} {2:.0f} m".format(
                    rec.get("id"), rec.get("cls"), d))
        far = max([d for _, d in kept], default=0.0)
        print("[fc] people: {0}/{1} record(s) within {2:.0f} m of a burning "
              "footprint (farthest kept {3:.1f} m)".format(
                  len(kept), len(recs), PEOPLE_MAX_DIST_M, far))
        return [rec for rec, _ in kept], [rec for rec, _ in dropped]

    def cull_background_people(self):
        """"Only keep humans that are in the disaster" — applied to the CITY's
        OWN pedestrians, which is where the rule actually bites.

        The 89 approved `fire_people` records were never the problem: every
        one of them was solved against a specific burning building and all 89
        sit within 59.5 m of a burning footprint. But `generate_scene_on_stage`
        ALSO plants its own background population — `scene_generator`'s
        sidewalk/crossing pedestrians and `detail/parks.py`'s park figures,
        `category == "human"` — scattered across the whole plate with no idea
        there is a fire on it. This run has **128 of them** against 75
        buildings (`scene_gen/_scene_assets.tsv`), most of them blocks away
        from anything burning, and they are what makes the scene read as a
        normal Tuesday with a fire in the corner.

        They are HIDDEN, not deleted: `FC_HIDE` already exists for exactly
        this ("an invisible prim still composes and still costs memory; a
        deactivated one does not"), the same treatment the intact shells get,
        and it keeps the placement list and every index into it intact.

        Same datum as `people_in_the_fire`: distance to the nearest burning
        building's FOOTPRINT, `FC_PEOPLE_MAX_DIST_M` (0 disables).
        """
        if PEOPLE_MAX_DIST_M <= 0.0 or not getattr(self, "placed", None):
            self.culled_people = []
            return
        burning = []
        for r in self.placed:
            rec = r.get("rec") or {}
            burning.append((float(rec.get("x", 0.0)), float(rec.get("y", 0.0)),
                            float(rec.get("W", 0.0)), float(rec.get("D", 0.0)),
                            math.radians(float(rec.get("yaw_deg", 0.0)))))
        kept, culled = 0, []
        for p in self.placements:
            if p.get("category") != "human" or not p.get("prim_path"):
                continue
            px, py = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
            best = 1e18
            for cx, cy, W, D, a in burning:
                dx, dy = px - cx, py - cy
                ca, sa = math.cos(a), math.sin(a)
                u = ca * dx + sa * dy
                v = -sa * dx + ca * dy
                best = min(best, math.hypot(max(0.0, abs(u) - W / 2.0),
                                            max(0.0, abs(v) - D / 2.0)))
            if best <= PEOPLE_MAX_DIST_M:
                kept += 1
                continue
            if hide_intact(self.stage, p["prim_path"]):
                culled.append((p, best))
        self.culled_people = [q for q, _d in culled]
        n = kept + len(culled)
        print("[fc] background pedestrians: {0} of the city's own {1} human "
              "placement(s) hidden — further than {2:.0f} m "
              "(FC_PEOPLE_MAX_DIST_M) from any burning footprint; {3} kept "
              "in the fire".format(len(culled), n, PEOPLE_MAX_DIST_M, kept))
        if culled:
            far = max(d for _q, d in culled)
            print("[fc]   farthest was {0:.0f} m out; the approved "
                  "fire_people records are authored separately and are NOT "
                  "touched by this pass".format(far))

    def place_people(self):
        """Author the approved people records onto the assembled city.

        `fire_people.to_placements` converts the dry run's records into
        `apply_placements` dicts (pose binding, prone rolls, pose-z drops);
        NOT instanced — `_bind_human_pose` edits inside each rig. Runs after
        `put_the_fire_back` so figures land on the final scene.

        NOTHING IN HERE MAY ABORT THE LAUNCH. The 2026-08-31 crash (see
        `people_records`) took the captures down with it, which is the one
        deliverable of a run; the whole pass is inside a try/except that
        prints and returns. Progress is printed BEFORE the conversion and
        again before authoring, so a pass that stops making progress is
        distinguishable in the pane from one that never started.
        """
        if not PEOPLE:
            print("[fc] people: FC_PEOPLE=0 — skipped")
            return
        if not PEOPLE_JSON or not os.path.isfile(PEOPLE_JSON):
            print("[fc] people: FC_PEOPLE_JSON missing ({0!r}) — skipped; "
                  "run tools/fire_people_dry_run.py and pass its records "
                  "JSON".format(PEOPLE_JSON))
            return
        t0 = time.time()
        try:
            import json as _json
            import scene_generator as _sg
            from disaster import fire_people as fpl
            doc = _json.load(open(PEOPLE_JSON))
            recs, key = self.people_records(doc)
            print("[fc] people: {0} record(s) read from {1} (key {2!r})"
                  .format(len(recs), PEOPLE_JSON, key))
            recs, _out = self.people_in_the_fire(recs)
            by = {}
            for r in recs:
                by[r.get("cls", "?")] = by.get(r.get("cls", "?"), 0) + 1
            placements, skipped = fpl.to_placements(recs)
            print("[fc] people: converting -> {0} placement(s), {1} skipped "
                  "({2}); authoring under {3}/people".format(
                      len(placements), sum(len(v) for v in skipped.values()),
                      ", ".join("{0}={1}".format(k, len(v))
                                for k, v in sorted(skipped.items())) or "none",
                      PARENT))
            # ONE CALL, NOT CHUNKS. `apply_placements` names a prim
            # `<parent>/<category>_<group>_<i>` with `i` the index INTO THE
            # LIST IT WAS GIVEN, and both `i` and `group` restart at 0 on
            # every call — so authoring in chunks under one parent makes
            # chunk 2's `human_0_0` land on chunk 1's prim and add a SECOND
            # reference to it. The progress this pass needs instead is the
            # asset count printed below: the slow step is the cold Nucleus
            # fetch of each distinct RenderPeople rig, which happens once per
            # asset, not once per figure.
            assets = sorted({p["usd"] for p in placements})
            print("[fc] people: {0} figure(s) over {1} distinct rig(s) — a "
                  "cold Nucleus fetch of these is the slow step:".format(
                      len(placements), len(assets)))
            for u in assets:
                print("[fc]    {0} x{1}".format(
                    u.rsplit("/", 1)[-1],
                    sum(1 for p in placements if p["usd"] == u)))
            _sg.apply_placements(self.stage, placements, PARENT + "/people",
                                 self.ssf)
            omni.kit.app.get_app().update()
            n_live = 0
            for k in range(len(placements)):
                pp = placements[k].get("prim_path")
                if pp and self.stage.GetPrimAtPath(Sdf.Path(pp)).IsValid():
                    n_live += 1
            print("[fc] people: {0}/{1} prim(s) valid on the stage under "
                  "{2}/people".format(n_live, len(placements), PARENT))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[fc] *** people pass FAILED ({0}) — the city, the bakes "
                  "and the fire are all up; continuing to the captures "
                  "WITHOUT people rather than aborting the run".format(exc))
            return
        print("[fc] people: {0} authored in {1:.0f} s ({2})".format(
            len(placements), time.time() - t0,
            ", ".join("{0} {1}".format(k, v) for k, v in sorted(by.items()))))
        vram_mb("people placed")

    def put_the_fire_back(self):
        if not (FLOW and self.placed):
            self.fires = []
            return
        stage = self.stage
        fx.setup_flow_stack(stage, density_cell_size_m=CELL_M,
                            max_blocks=MAX_BLOCKS, scene_scale_factor=self.ssf)
        flow_root = fx.FLOW_ROOT
        print("[fc] flow stack up at {0} ({1} m cells, {2} block pool)".format(
            flow_root, CELL_M, MAX_BLOCKS))

        # STREET-FACING BIAS — computed ONCE, from the dump, before the loop
        # below (not per building): `fal.load_dump_positions` is a flat file
        # read, cheap to call once and reuse, expensive to reopen 32 times
        # for no reason. Missing/unparseable degrades to `{}` — every
        # `street_side_ranks` call then also returns `{}` (see its own
        # docstring) and `place_fire` falls back to its ORIGINAL unweighted
        # round-robin, so a run with no dump path never fails over this.
        street_positions = (fal.load_dump_positions(STREET_DUMP_PATH)
                            if STREET_DUMP_PATH else {})
        # kept for capture(): the same footprints double as the oblique
        # review camera's occluder list (`fal.clear_oblique`)
        self.street_positions = street_positions
        print("[fc] street-facing bias: {0} from {1!r}".format(
            "{0} building position(s) loaded".format(len(street_positions))
            if street_positions else "OFF (no/unreadable dump)",
            STREET_DUMP_PATH))

        predicted = allocate_emitters(
            self.placed, EMITTER_BUDGET, MAX_EMITTERS, SMOKE,
            SIDE_SMOKE_FLAME_MAX, SIDE_SMOKE_NONFLAME_MAX,
            ROOF_CAP_INTACT, ROOF_CAP_COLLAPSED,
            FLAME_MIN_CLUSTERS if FLAME_MIN_CLUSTERS > 0 else None,
            FLAME_EXTRA_MAX, FLAME_SIZE_SCALE, SMOKE_SIZE_SCALE,
            SMOKE_WINDOW_JETS, RESIDUAL_FLAME_FRAC)
        print("\n[fc] EMITTER BUDGET {0} (FA_EMITTER_BUDGET), {1} damaged "
              "building(s), per-building cap {2} opening(s) (FA_EMITTERS), "
              "side smoke {3}/{4} (flame/non-flame), roof cap {5}/{6} "
              "(intact/collapsed), size-scaling flame={7} smoke={8}, "
              "window-jets={9}, residual-flame-frac={10}, min-clusters={11}"
              .format(EMITTER_BUDGET, len(self.placed), MAX_EMITTERS,
                      SIDE_SMOKE_FLAME_MAX, SIDE_SMOKE_NONFLAME_MAX,
                      ROOF_CAP_INTACT, ROOF_CAP_COLLAPSED,
                      int(FLAME_SIZE_SCALE), int(SMOKE_SIZE_SCALE),
                      int(SMOKE_WINDOW_JETS), RESIDUAL_FLAME_FRAC,
                      FLAME_MIN_CLUSTERS))
        print("[fc]   {0:<3} {1:<30} {2:<5} {3:<9} {4:>3} {5:>5} {6:>6}  {7}"
              .format("#", "stem", "level", "state", "st", "alloc", "pred",
                      "note"))
        for r in sorted(self.placed,
                        key=lambda q: (q.get("rank", 9), -q.get("n_st", 0),
                                       q["i"])):
            print("[fc]   {0:<3} {1:<30} {2:<5} {3:<9} {4:>3} {5:>5} {6:>6}  "
                  "{7}".format(
                      r["i"], r["stem"], r["doc"].get("level") or "?",
                      r.get("state", "?"), r.get("n_st", 0),
                      "-" if r.get("alloc") is None else r["alloc"],
                      r.get("est", {}).get("total", 0),
                      r.get("drop_reason", "")))
        print("[fc]   predicted {0} emitter(s) of {1} budgeted; {2} building("
              "s) carry geometry and baked soot only".format(
                  predicted, EMITTER_BUDGET,
                  sum(1 for r in self.placed if r.get("alloc") is None)))
        if predicted > EMITTER_BUDGET:
            print("[fc]   *** the allocator could not stay inside the budget "
                  "— check FA_EMITTERS and FA_MAX_BLOCKS")

        fires = []
        for r in self.placed:
            if r.get("alloc") is None or not r.get("doc"):
                continue
            # THE CELL TRANSFORM, ONTO THE FIRE DATA. An emitter is authored
            # under `<flow_root>/emitters`, which inherits NEITHER the
            # holder's rotation nor its translation, so the wall FRAMES, the
            # MASS centres and the recorded SEATS are all moved here instead
            # — `fire_bake.place`, the general (rotating) form of the row
            # launcher's `translate`.
            #
            # SEATS ARE MOVED BY `place`, SO `place_fire` GETS dx=dy=0.
            # `_sphere_source` adds its own `dx/dy` to each seat; passing the
            # cell again there would translate every plume TWICE.
            fb.place(r["masses"], r["events"], r["doc"].get("seats"),
                     r["x"], r["y"], r["yaw"])
            top_z = r["bbox"][5] if r.get("bbox") else r["doc"].get("top_z")
            sides = tuple((r["doc"].get("fire") or {}).get("sides") or ())
            # KEY BY THE RECORD'S OWN GLOBAL DUMP INDEX, not the row's
            # enumerate position: `street_positions` is keyed 214-297 (the
            # dump's `i`), while `r["i"]` here is 0..N-1 — the wrong key
            # returns `{}` and the street bias silently no-ops for every
            # building while the launch banner still reads correct.
            street_rank = fal.street_side_ranks(street_positions,
                                                r["rec"].get("i"), sides)
            # RANDOMISED, NOT ARGMAX (2026-08-31, second review: "street
            # facing fires all seem to be in the top left. I want it
            # randomized"). Seeded off the bake's own STEM — stable no
            # matter which row this run happens to iterate it as — so two
            # buildings sharing the identical `street_rank` pattern (common:
            # `street_side_score`'s 200 m "nothing nearby" sentinel repeats
            # across a whole district) still draw independently. See
            # `fal.choose_street_side`'s own docstring for the measured
            # cause and `_side_weights`' own docstring for why this function
            # never recomputes the choice itself.
            street_bias_side = fal.choose_street_side(
                street_rank, "{0}-{1}-street".format(SEED, r["stem"]))
            res = fal.place_fire(
                self.stage, flow_root, r["doc"], r["masses"], r["events"],
                "c{0}".format(r["i"]), random.Random(SEED + 31 * r["i"]),
                top_z, 0.0, 0.0, scale=SCALE, max_emitters=r["alloc"],
                smoke=SMOKE, smoke_scale=SMOKE_SCALE,
                street_bias_side=street_bias_side,
                street_bias_weight=STREET_BIAS_WEIGHT,
                side_smoke_flame_max=SIDE_SMOKE_FLAME_MAX,
                side_smoke_nonflame_max=SIDE_SMOKE_NONFLAME_MAX,
                roof_cap_intact=ROOF_CAP_INTACT,
                roof_cap_collapsed=ROOF_CAP_COLLAPSED,
                flame_min_clusters=(FLAME_MIN_CLUSTERS
                                    if FLAME_MIN_CLUSTERS > 0 else None),
                flame_extra_max=FLAME_EXTRA_MAX,
                flame_size_scaling=FLAME_SIZE_SCALE,
                smoke_size_scaling=SMOKE_SIZE_SCALE,
                smoke_window_jets=SMOKE_WINDOW_JETS,
                residual_flame_frac=RESIDUAL_FLAME_FRAC,
                smoke_vertical_bias=SMOKE_VERTICAL_BIAS,
                # CONTACT SNAP — `bake_geometry_root` resolves to
                # `<holder>/bake/bake`, where the referenced bake's own
                # geometry actually composed (`compose_bakes` referenced it
                # onto `<holder>/bake`; the bake's own default prim is
                # `/World`, its content root `/World/bake`). `r["prim"]` is
                # that holder path, set by `compose_bakes` for exactly this
                # use. `None` (CONTACT_SNAP off) reproduces the pre-fix
                # placement exactly.
                geom_root=(fal.bake_geometry_root(r["prim"], r["doc"])
                          if CONTACT_SNAP else None),
                snap_tol_m=SNAP_TOL_M, snap_inset_m=SNAP_INSET_M,
                snap_reach_in_m=SNAP_REACH_IN_M,
                snap_reach_out_m=SNAP_REACH_OUT_M)
            res["i"] = r["i"]
            res["total"] = (res["flame"] + res["smoke"] + res["interior"]
                            + res["roof"])
            fires.append(res)
            print("[fc] d{0:<2} fire: {1} flame over {2} opening(s), {3} "
                  "side smoke, {4} interior, {5} roof = {6} (predicted {7}, "
                  "state={8}, roof_collapsed={9}, street_rank={10}, "
                  "boosted_side={11})".format(
                      r["i"], res["flame"], res.get("openings", 0),
                      res["smoke"], res["interior"], res["roof"],
                      res["total"], r.get("est", {}).get("total", 0),
                      res.get("state"), res.get("roof_collapsed"),
                      street_rank or "n/a", street_bias_side or "none"))
            if res.get("note"):
                print("[fc]     " + res["note"])
            snap = res.get("snap") or {}
            if snap.get("tested"):
                print("[fc]     contact snap: {0} tested, {1} ok, {2} "
                      "snapped, {3} dropped, worst pre-fix offset {4:.2f} m"
                      .format(snap.get("tested", 0), snap.get("ok", 0),
                              snap.get("snapped", 0), snap.get("dropped", 0),
                              snap.get("worst_offset_m", 0.0)))
            elif CONTACT_SNAP and not snap.get("locator"):
                print("[fc]     contact snap: no locator built for {0} — "
                      "vtk unavailable, or the composed bake carried no "
                      "usable geometry".format(r["stem"]))
            top = res.get("synthetic_top") or {}
            if top.get("dropped"):
                print("[fc]     top-storey synthetic: {0}/{1} synthetic "
                      "opening(s) above storey {2} dropped".format(
                          top.get("dropped", 0), top.get("tested", 0),
                          top.get("max_allowed")))
        self.fires = fires
        total = sum(f["total"] for f in fires)
        print("[fc] {0} Flow emitter(s) in all (budget {1}, predicted {2})"
              .format(total, EMITTER_BUDGET, predicted))
        if CONTACT_SNAP:
            snap_tot = {"tested": 0, "ok": 0, "snapped": 0, "dropped": 0}
            snap_worst = 0.0
            n_locator = 0
            for f in fires:
                s = f.get("snap") or {}
                for k in ("tested", "ok", "snapped", "dropped"):
                    snap_tot[k] += s.get(k, 0)
                snap_worst = max(snap_worst, s.get("worst_offset_m", 0.0))
                n_locator += int(bool(s.get("locator")))
            print("[fc] CONTACT SNAP (FC_CONTACT_SNAP=1): {0} building(s) "
                  "with a locator, {1} opening(s) tested, {2} already "
                  "touching, {3} snapped onto real geometry, {4} dropped "
                  "(no geometry within reach), worst pre-fix offset "
                  "{5:.2f} m (tol {6:.2f} m, inset {7:.2f} m)".format(
                      n_locator, snap_tot["tested"], snap_tot["ok"],
                      snap_tot["snapped"], snap_tot["dropped"], snap_worst,
                      SNAP_TOL_M, SNAP_INSET_M))
        top_tot = {"tested": 0, "dropped": 0}
        for f in fires:
            t = f.get("synthetic_top") or {}
            for k in top_tot:
                top_tot[k] += t.get(k, 0)
        print("[fc] TOP-STOREY SYNTHETIC FILTER: {0} synthetic opening(s) "
              "tested citywide, {1} dropped for sitting above the excluded "
              "top storeys (kit and real-glazing GAC openings are never "
              "tested)".format(top_tot["tested"], top_tot["dropped"]))
        if total > EMITTER_BUDGET:
            print("[fc] *** OVER BUDGET: {0} > {1}. Flow's block pool is "
                  "finite; past it every emitter after the first few gets no "
                  "voxels and the city renders with NO SMOKE while these "
                  "counts still look right. Lower FA_EMITTER_BUDGET or raise "
                  "FA_MAX_BLOCKS (currently {2})".format(
                      total, EMITTER_BUDGET, MAX_BLOCKS))
        if _flag("FC_FIRE_PHASED", "0"):
            # TWO-PHASE VRAM MEASUREMENT (user, 2026-08-31: "have them
            # deactivated first to measure vram then enable them and
            # activate and test VRAM"). Flow allocates its block pool on
            # UPDATE while emitters are ACTIVE — so deactivate the flow
            # root BEFORE the first post-authoring update, measure the
            # city-without-fire cost, then activate and measure the fire
            # cost as a clean delta.
            fr = self.stage.GetPrimAtPath(flow_root)
            if fr and fr.IsValid():
                fr.SetActive(False)
            for _ in range(30):
                omni.kit.app.get_app().update()
            vram_mb("fires staged but DEACTIVATED")
            if fr and fr.IsValid():
                fr.SetActive(True)
            for _ in range(30):                 # Flow allocates its pool now
                omni.kit.app.get_app().update()
            vram_mb("fires ACTIVATED")
        else:
            for _ in range(30):                 # let Flow allocate its pool
                omni.kit.app.get_app().update()
        self.vram["flow"] = vram_mb("Flow up ({0} emitter(s))".format(total))

        # RE-ASSERT FRACTIONAL CUTOUT OPACITY, NOW THAT THE STAGE IS COMPOSED
        # — the second of the two required forms (see `KIT_ARGS`). The
        # startup flag does not survive composition; the carb form alone is
        # too late for startup. Every launcher in this pipeline does both.
        try:
            _s = carb.settings.get_settings()
            for _k in ("/rtx/raytracing/fractionalCutoutOpacity",
                       "/rtx/pathtracing/fractionalCutoutOpacity"):
                _s.set_bool(_k, True)
            print("[fc] fractionalCutoutOpacity re-asserted post-composition "
                  "(raytracing={0}, pathtracing={1})".format(
                      _s.get("/rtx/raytracing/fractionalCutoutOpacity"),
                      _s.get("/rtx/pathtracing/fractionalCutoutOpacity")))
        except Exception as exc:
            print("[fc] WARNING: could not re-assert fractionalCutoutOpacity "
                  "({0}); the soot and the glass deposits will render as "
                  "hard cutouts".format(exc))
        for _ in range(4):
            omni.kit.app.get_app().update()

    # -- 5) the captures ---------------------------------------------------
    def _snaps(self):
        import importlib.util as _ilu
        sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
        spec = _ilu.spec_from_file_location("snapshots", sp)
        mod = _ilu.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    def capture(self):
        if not SNAP_DIR:
            return
        try:
            snaps = self._snaps()
            os.makedirs(SNAP_DIR, exist_ok=True)
            app = omni.kit.app.get_app()
            for _ in range(60):
                app.update()
            # FLOW NEEDS TIME BEFORE IT IS PHOTOGRAPHED — the emitters inject
            # fuel per step, so a capture at t=0 is of an empty grid.
            if FLOW and getattr(self, "fires", None):
                self.timeline.play()
                for _ in range(300):
                    app.update()
            span = max(self.region)
            snaps.overview(self.stage, (0.0, 0.0), span * 1.05,
                           os.path.join(SNAP_DIR, "city_top.png"), self.ssf)
            if INTACT_ONLY:
                print("[fc] snapshots (intact only) -> {0}".format(SNAP_DIR))
                return
            # THE WAVE — the deliverable. One LOW oblique from behind the
            # origin building, looking DOWNWIND along the line the fire
            # travelled, so the whole spread is in one frame instead of
            # sixteen building portraits. `heading_deg` is the direction the
            # wind blows TOWARD (`urban_fire_spread._wind_factor`'s own
            # `wind_dir` convention), so the camera stands UPWIND of the
            # origin and looks the other way.
            org = self.origin_row()
            if org is not None:
                h = math.radians(float(self.wind["heading_deg"]))
                ux, uy = math.cos(h), math.sin(h)
                b = org.get("bbox")
                H = (b[5] - b[2]) if b else 30.0
                back = max(70.0, 1.8 * H)
                fwd = max(150.0, 0.55 * span)
                ex, ey = org["x"] - back * ux, org["y"] - back * uy
                tx, ty = org["x"] + fwd * ux, org["y"] + fwd * uy
                ez, tz = max(14.0, 0.45 * H), 0.30 * H
                # raise the eye until the sightline clears every building
                # between it and the target (v4's wave was nose-to-wall
                # against a neighbour's facade)
                wobs = list((getattr(self, "street_positions", None)
                             or {}).values())
                if wobs:
                    dd = math.hypot(ex - tx, ey - ty)
                    azd = math.degrees(math.atan2(ey - ty, ex - tx))
                    ez2 = fal.raise_over_sightline(tx, ty, tz, dd, azd,
                                                   ez, wobs)
                    if ez2 > ez:
                        print("[fc] wave eye raised {0:.0f} -> {1:.0f} m "
                              "to clear the sightline".format(ez, ez2))
                        ez = ez2
                eye = (ex * self.ssf, ey * self.ssf, ez * self.ssf)
                tgt = (tx * self.ssf, ty * self.ssf, tz * self.ssf)
                snaps.place_camera(self.stage, eye, tgt)
                snaps.snapshot(os.path.join(SNAP_DIR, "wave_downwind.png"))
                print("[fc] the wave: from ({0:+.0f}, {1:+.0f}) at {2:.0f} m "
                      "looking {3:.0f} deg downwind past {4}".format(
                          org["x"] - back * ux, org["y"] - back * uy,
                          eye[2] / max(self.ssf, 1e-9),
                          self.wind["heading_deg"], org["stem"]))
            # the fire-facing per-building review pair, framed by the same
            # arithmetic the row launcher uses (`fal.fire_view_params`)
            for r in self.placed:
                b = r.get("bbox")
                if not b:
                    continue
                name = "d{0}_{1}_{2}".format(
                    r["i"], r["doc"].get("name") or r["stem"],
                    r["doc"].get("level", ""))
                vp = fal.fire_view_params(r["doc"], r["masses"], b)
                # push the oblique out of any neighbour it would sit inside
                # (3 of 32 city_v3 obliques were shot from inside a wall)
                obs = [o for i2, o in
                       (getattr(self, "street_positions", None) or {}).items()
                       if i2 != r["rec"].get("i")]
                if obs:
                    vp, moved = fal.clear_oblique(vp, r["x"], r["y"], obs)
                    if moved:
                        print("[fc] obl camera for {0} pushed {1} step(s) "
                              "clear of a neighbour (dist {2:.0f} m, "
                              "h {3:.0f} m)".format(name, moved,
                                                    vp["obl_dist"],
                                                    vp["obl_h"]))
                snaps.views_around(self.stage, {name: (r["x"], r["y"])},
                                   SNAP_DIR, self.ssf, top_h=vp["top_h"],
                                   obl_dist=vp["obl_dist"],
                                   obl_h=vp["obl_h"],
                                   azimuth_deg=vp["azimuth_deg"],
                                   aim_h=vp["aim_h"])
            print("[fc] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[fc] snapshots FAILED: {0}".format(exc))

    def origin_row(self):
        """The building the fire started in: the earliest `t_ignite_s` among
        the placed bakes (the spread solve's own origin has `t_ignite_s` 0),
        falling back to manifest order."""
        cands = [r for r in self.placed if r.get("bbox")]
        if not cands:
            return None
        return sorted(cands, key=lambda r: (
            float(r["rec"].get("t_ignite_s") if r["rec"].get("t_ignite_s")
                  is not None else 1e18), r["i"]))[0]

    # -- 6) the budget and the log ------------------------------------------
    def report(self):
        self.vram["end"] = vram_mb("after captures")
        n_dmg = max(1, len(getattr(self, "placed", []) or []))
        base = self.vram.get("empty")
        intact = self.vram.get("intact")
        bakes = self.vram.get("bakes")
        flow_v = self.vram.get("flow")
        end = self.vram.get("end")
        try:
            if base is not None and intact is not None:
                city_cost = intact - base
                area = self.region[0] * self.region[-1]
                per_km2 = city_cost * (1.0e6 / max(1.0, area))
                bake_cost = (bakes - intact) if bakes is not None else None
                per_b = (bake_cost / n_dmg) if bake_cost is not None else None
                flow_cost = ((flow_v - bakes) if (flow_v is not None
                                                  and bakes is not None)
                             else None)
                print("[fc] VRAM BUDGET: baseline {0:.0f} MiB | INTACT CITY "
                      "{1:.0f} MiB for {2} building(s) on {3:.2f} km2 "
                      "(= {4:.0f} MiB/km2) | bakes {5} for {6} damaged = {7} "
                      "| Flow {8} | end {9:.0f} MiB".format(
                          base, city_cost, self.n_houses, area / 1.0e6,
                          per_km2,
                          "{0:.0f} MiB".format(bake_cost)
                          if bake_cost is not None else "n/a", n_dmg,
                          "{0:.0f} MiB/building".format(per_b)
                          if per_b is not None else "n/a",
                          "{0:.0f} MiB".format(flow_cost)
                          if flow_cost is not None else "off",
                          end if end is not None else -1))
                print("[fc]   NOTE: the intact-city figure INCLUDES the {0} "
                      "building(s) later hidden ({1}) — an invisible prim "
                      "still composes; FC_HIDE=deactivate to drop them"
                      .format(self.n_hidden, HIDE_MODE))
                if per_b is not None:
                    for n_city, gpu, cap in ((16, "5090", 32768),
                                             (16, "RTX PRO 5000", 49152),
                                             (40, "5090", 32768),
                                             (40, "RTX PRO 5000", 49152)):
                        proj = (base + per_km2 + per_b * n_city
                                + (flow_cost or 0.0))
                        print("[fc]   projection: 1 km2 of this city + {0} "
                              "damaged building(s) + this Flow -> {1:.0f} MiB "
                              "on a {2} ({3:.0%} of {4} MiB)".format(
                                  n_city, proj, gpu, proj / cap, cap))
        except Exception as exc:
            print("[fc] VRAM budget summary failed: {0}".format(exc))

        # THE KIT LOG. Flow fails silently — see the module docstring.
        path, hits = grep_kit_log()
        if path is None:
            print("[fc] Kit log not found under {0} — the Flow OOM check did "
                  "NOT run; do not trust a capture that shows no smoke"
                  .format(" / ".join(KIT_LOG_GLOBS)))
        elif any(hits.values()):
            print("\n[fc] " + "!" * 70)
            print("[fc] FLOW STARVED — {0}".format(
                ", ".join("{0!r} x{1}".format(k, v)
                          for k, v in hits.items() if v)))
            print("[fc]   log: {0}".format(path))
            print("[fc]   Flow's block pool ran out. Emitters past the first "
                  "few got NO VOXELS: the city renders with no smoke while "
                  "every count above looks right. RAISE FA_MAX_BLOCKS "
                  "(currently {0}) or LOWER FA_EMITTER_BUDGET (currently "
                  "{1}) — do not go looking at the emitters.".format(
                      MAX_BLOCKS, EMITTER_BUDGET))
            print("[fc] " + "!" * 70 + "\n")
        else:
            print("[fc] Flow OOM check CLEAN ({0})".format(path))

    def run(self):
        if INTACT_ONLY:
            print("\n[fc] FC_INTACT_ONLY=1 — the city is built and nothing "
                  "will be damaged. This is plan sec 4c's 'measure the "
                  "intact city FIRST': it is the only unknown in the VRAM "
                  "projection.")
            self.placed, self.refused, self.rows = [], [], []
            self.missing, self.fires = [], []
            self.n_hidden = 0
            self.capture()
            self.banner()
            return
        self.load_fire()
        self.compose_bakes()
        self.scorch_vegetation()
        self.fire_debris_apron()
        self.put_the_fire_back()
        self.cull_background_people()
        self.place_people()
        self.capture()
        self.banner()

    def banner(self):
        self.report()
        print("\n" + "=" * 78)
        print("URBAN FIRE CITY   {0}   {1:.0f} x {2:.0f} m   {3} building(s), "
              "{4} damaged".format(SCENE_CONFIG, self.region[0],
                                   self.region[-1], self.n_houses,
                                   len(getattr(self, "placed", []) or [])))
        if not INTACT_ONLY:
            print("  manifest: {0}  (city seed {1})".format(
                getattr(self, "manifest_path", "?"),
                getattr(self, "city_seed", "?")))
            for r in getattr(self, "placed", []) or []:
                b = r.get("bbox")
                print("  ({0:+7.1f},{1:+7.1f}) yaw {2:+4.0f}  {3:<30} {4:<5} "
                      "{5:>6.1f} MB  {6} event(s)  {7}".format(
                          r["x"], r["y"], r["yaw"], r["stem"],
                          r["doc"].get("level") or "?",
                          os.path.getsize(r["usd"]) / 1e6, len(r["events"]),
                          "H {0:.0f} m".format(b[5] - b[2]) if b
                          else "NO GEOMETRY"))
            if getattr(self, "missing", None):
                print("  {0} record(s) NOT BAKED (still intact): {1}".format(
                    len(self.missing),
                    ", ".join(str(m[1]) for m in self.missing)))
            if getattr(self, "refused", None):
                print("  {0} bake(s) refused: {1}".format(
                    len(self.refused),
                    ", ".join("{0} ({1})".format(r["stem"], r.get("skip"))
                              for r in self.refused)))
            veg = getattr(self, "veg_scorch", None)
            if veg is not None:
                print("  scorched vegetation: {0} tree(s) darkened of {1} "
                      "fuel placement(s) checked (FC_SCORCH_VEG={2})".format(
                          veg.get("trees", 0), veg.get("fuels_total", 0),
                          int(SCORCH_VEG)))
            print("  background pedestrians hidden (outside the fire): {0}"
                  .format(len(getattr(self, "culled_people", []) or [])))
            print("  companion props hidden with their damaged building: {0} "
                  "(FC_HIDE_PROPS={1});  gprim-root placements un-instanced: "
                  "{2} (FC_UNINSTANCE_GPRIM_ROOTS={3})".format(
                      len(getattr(self, "hidden_props", []) or []),
                      int(HIDE_PROPS),
                      len(getattr(self, "uninstanced", []) or []),
                      int(UNINSTANCE_GPRIM_ROOTS)))
            aprons = getattr(self, "aprons", None)
            if aprons is not None:
                n_b = sum(1 for r in aprons if r.get("prim"))
                n_l = sum(r.get("n", 0) for r in aprons)
                print("  fire debris apron: {0} lump(s) on {1} building(s) "
                      "(FC_FIRE_APRON={2})".format(n_l, n_b, int(FIRE_APRON)))
        print("  built in {0:.0f} s".format(time.time() - self.t0))
        print("=" * 78 + "\n")
        print("URBAN FIRE CITY DONE")


def main():
    FireCityApp().run()


if __name__ == "__main__":
    try:
        main()
    except Exception as _exc:
        import traceback
        traceback.print_exc()
        print("URBAN FIRE CITY FAILED: {0}".format(_exc))
    if KEEP_OPEN or not _HEADLESS:
        _app = omni.kit.app.get_app()
        omni.timeline.get_timeline_interface().play()
        while simulation_app.is_running():
            _app.update()
        omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
