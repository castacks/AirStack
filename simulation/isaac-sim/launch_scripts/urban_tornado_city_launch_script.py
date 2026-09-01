#!/usr/bin/env python
"""
urban_tornado_city — LIVE ASSEMBLY, v1: the 500 m downtown, built intact,
with a tornado track's damage authored DIRECTLY onto REAL merged (GAC /
downtowncity) buildings in the SAME Kit process that built the city — no
bake, no manifest JSON, no separate dry-run pass. `scene_gen/_plans/
urban_tornado_plan.md` §6 work item #3, stream W's own file (the round-2
"build-urban-tornado-scenes" brief).

    ISAAC_SIM_HEADLESS=true SCENE_CONFIG=downtown_tornado_bench_500 \\
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/tornado_city \\
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \\
    /isaac-sim/python.sh \\
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/urban_tornado_city_launch_script.py \\
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window

Modelled STRUCTURALLY on `urban_fire_city_launch_script.py` (its `_env`/
`_flag`, `dump_city_placements`, `place_holder`, `hide_intact`, `vram_mb`,
the `_snaps()`/capture machinery, `banner`, `main`) but a SIMPLER pipeline
end to end: this launcher decides the damage ITSELF, live, in the SAME Kit
process that built the intact city. There is no separate dry-run tool, no
manifest JSON, no bake directory to search, and therefore none of
`resolve_cell`'s three-route reconciliation (that machinery exists because
the fire manifest is solved by a DIFFERENT process at a different time;
here `self.placements[i]["prim_path"]` is trustworthy by construction —
same process, same list, same index — so this file hides a building by its
placement's own `prim_path` directly).

WHY THIS FILE CANNOT IMPORT THE FIRE LAUNCHER (or vice versa): a
`SimulationApp` is constructed at import in BOTH files, and a second Kit
app in one process is a segfault inside the first second (measured,
documented in `urban_fire_city_launch_script.py`'s own docstring). Anything
shared between the two lives in a plain-python sibling module instead
(`disaster.tornado_city`, `disaster.urban_fire_city` for `typology_at`,
`disaster.fire_assembly_lib` for its two generic `vram_mb`/`bbox` helpers)
— this file copies the launcher-local helpers it needs (`_env`/`_flag`,
`hide_intact`, `place_holder`, `_uninstance_gprim_roots`, the env-clutter/
sky helpers) rather than import them, the same way `urban_fire_city_
launch_script.py` itself does relative to `downtown_quake_launch_script.py`.

THE LAYOUT-TIME DISASTER-FIELD TRAP — discovered writing this file, not in
the round-1 offline plan, because NOTHING before this launcher ever called
`generate_scene_on_stage` on a `disaster-type: tornado` URBAN preset (the
city dry run only ever reads `disaster.tornado` off the compiled config for
`tornado.resolve_cfg`; it never touches the layout generator). Unlike fire
(`disaster-type: fire` is not a `compile_disaster.DISASTERS` entry at all),
`disaster-type: tornado` DOES compile for real, and `compile_tornado`'s
output is not just the `tornado` track sub-block: it is also a full set of
top-level `disaster.*` fraction knobs (`damaged_fraction`,
`destroyed_fraction`, `trees_toppled_fraction`, `streetlights_toppled_
fraction`, `traffic_lights_toppled_fraction`, `trash_cans_toppled_
fraction`, `cars_toppled_fraction`/`cars_strewn`, `humans_prone_fraction`/
`humans_strewn`, plus a `debris.path_pieces_per_100m2`/`path_piles_per_
100m2` ground-litter density) THAT `scene_generator.build_city` READS
DIRECTLY AT LAYOUT TIME — this is the suburb tornado launcher's own damage
mechanism (tree/streetlight/car/human toppling scattered by the compiled
field), designed for a preset with no separate "urban ladder" at all.

Compiling `downtown_tornado_500` normally therefore does NOT hand this
launcher an intact city: `urban_gac.yaml`'s building pools carry no
`damaged`/`destroyed` sub-keys (that shape only exists for suburb single-
family houses), so `scene_generator.build_city`'s fate-weighted picker
falls back to its OWN documented stand-in — "No pre-damaged pool: fall back
to tilting + sinking an intact building" — and EVERY building whose slot
rolls a `damaged_fraction` hit (scaled by the compiled field, which is wide
open along this preset's whole corridor) gets a GENERIC 18 deg roll/pitch
tilt and a 0.3-0.9 m sink authored straight into its INTACT placement. That
is a quake-style lean+sink, and it directly violates this round's own
constraint (`_plans/urban_tornado_plan.md` §0.1 / the build-urban-tornado-
scenes skill: "no completely demolished skyscrapers... it never pancakes,
never loses a storey, never leans or sinks") — worse on a building this
launcher ALSO decides to slice, which would then be sliced out from under
an already-tilted intact mesh in an uncontrolled, unreviewed way. The
`humans_prone_fraction`/`humans_strewn` knobs are just as live and are
flatly out of scope (a whole separate, carefully-reviewed people pipeline
exists for this — the `place-people-in-tornado-scenes` skill).

FIXED by `_neutralise_layout_disaster()`, called on the COMPILED config
right after `load_scene_config`, BEFORE `generate_scene_on_stage`: zeroes
`damaged_fraction`/`destroyed_fraction`/`humans_prone_fraction`/
`humans_strewn` UNCONDITIONALLY (never safe for this pipeline — building
fate is this launcher's own job and humans are out of scope entirely), and
(default; `UT_LAYOUT_STREET=1` opts back in) the street-furniture toppling
knobs and the path-debris density too, so v1 ships with an UNAMBIGUOUSLY
intact base city: every piece of visible damage in this scene comes from
this launcher's own reviewed building ladder, nothing from an unreviewed
compile side-channel nobody has looked at on a downtown preset before.
`disaster.tornado` (`severity`/`epicenter`/`heading_deg`/
`curvature_deg_per_km`) is never touched — it is a SEPARATE sub-dict the
zeroing loop does not walk into, so `tornado.resolve_cfg`/`intensity_field`/
`wind_at` see the exact track the preset compiled.

THE TRANSFORM TRAP — the one thing to get right per building, identical in
kind to the fire launcher's own (see that file's docstring for the full
derivation; restated here because a LIVE slice has the exact same failure
mode as a bake). A live-sliced building's pieces are authored in the CELL's
own local frame — metres, Z-up, base at the cell origin — by
`gac_storey_slice.slice_to_kit`, exactly the frame a bake exports in. The
INTACT placement's own transform (`scene_generator.apply_placements`:
translate by the asset's own centroid-corrected offset, rotateXYZ with a
90 deg roll for a Y-up asset, scale = the pack's own asset scale, e.g. 0.01
for GAC) is about THAT asset's units/up-axis/pivot and must never be
inherited by the sliced pieces — it would shrink them 100x and lay them on
their side. So exactly like the fire launcher: hide the intact prim and
author a FRESH holder

    /World/tornado/<stem>          translate (x, y, z) * ssf
                                    rotateXYZ (0, 0, yaw_deg)   <- the
                                        placement's FINAL yaw only, no
                                        roll/pitch: the slice is already
                                        Z-up and already in metres
                                    scale     ssf   (a bake/slice is metres)
    /World/tornado/<stem>/cell     an Xform (NEVER `Scope.Define` — a Scope
                                        carries no transform op order, so
                                        the slicer's pieces would compose
                                        at the STAGE origin, not the
                                        holder's)

and reference/build EVERYTHING for that building — `gac_fire.place_source`,
`gac_storey_slice.slice_to_kit`'s pieces, `tornado_urban_usd.wreck_urban`'s
debris and displaced pieces — onto that CHILD `cell`, never onto the holder
Xform itself.

THE WIND-FRAME ROTATION — the other place this differs from every offline
tool that touches `tornado_urban`. `tools/tornado_urban_probe.py` and
`tools/tornado_city_dry_run.py` both slice and plan at yaw 0 (a bench, or a
dry run that never authors a rotated holder), so `wind["bearing_deg"]`
(world convention, math angle, from `tornado.wind_at`) IS the cell-frame
bearing `tornado_urban.side_weights`/`_windward_side` expect: the two
frames are the same when yaw is 0. HERE THEY ARE NOT. A building's holder
carries the placement's own `yaw_deg` (whatever `apply_placements` drew for
it), and `slice_to_kit`'s pieces — and therefore every `_side` (`S`/`N`/
`E`/`W`) the planner keys on — are written in the CELL's own UNROTATED
local frame (`quake_sliced._SIDE_NORMAL`: S faces -y, N +y, E +x, W -x,
ALWAYS, regardless of the holder's yaw). So the wind bearing handed to
`tornado_urban.plan_damage` (via `wreck_urban`) must be rotated INTO the
cell's own frame before the call:

    wind_local = dict(wind)
    wind_local["bearing_deg"] = (wind["bearing_deg"] - yaw_deg) % 360.0

A building yawed 90 deg whose real-world windward face is its WORLD-SOUTH
wall has a cell-frame windward face of "world-south rotated back 90 deg" —
its LOCAL-WEST wall (`S` in cell-frame terms). Get this wrong and every
building's damage lands on the wrong elevation, deterministically, in a way
no single-building probe would ever catch (the probe never rotates its
holder).

ROUND 2 WIRING (`_plans/urban_tornado_plan.md` §7, stream W2) -- three
things round 2 built and this file now calls: `kind == "kit"` buildings
(`bld_<style>_DG0` archetypes) are no longer left intact -- `disaster.
tornado_kit.wreck_kit` runs the SAME planner/apply machinery `tornado_urban_
usd.wreck_urban` runs for a sliced GAC/dtc building, on a kit build instead
(see `apply_damage`'s kit pass, right after the gac/dtc pass); the corridor's
GROUND EVIDENCE (a scattered masonry-city debris field plus a translucent
dust/scour stain, `disaster.tornado_urban_ground.scatter_corridor`/`build`/
`stain_overlay`) is authored after every per-building pass, before captures
(`TornadoCityApp.ground_evidence`); and the oblique captures were widened
(`obl_dist`/`obl_h`/`aim_h`) plus one new city-wide overview oblique added,
because round 1's tight, low oblique (dist 60, h 60, aim 12) sat INSIDE the
tower canyon and rendered the inside of a facade rather than the corridor --
see `capture()`'s own comment for the exact numbers.

WHAT v1 LEAVES INTACT (`_plans/urban_tornado_plan.md` §6, known gaps #2/#3
of the round-1 skill -- #2 (kit buildings) is CLOSED by round 2, see above):
  * every `route == "slice"` building with no `kind in ("gac", "dtc",
    "kit")` (AEC brownstones, `standalone/buildings/...` — `fire_bake.
    KINDS` has no entry for them, the same reason the fire pipeline cannot
    bake them, and `tornado_kit` only adapts a KIT-shaped `urban_building`
    style, never a whole-asset AEC merge);
  * NO bake, NO settle-on-damage, NO fracture, NO Flow, NO people, NO
    companion-roof-prop hiding (a damaged building's own roof plant/AC/
    fire-escape props — placed by the layout generator as SEPARATE
    placements, `urban_fire_city.prop_tag`'s own problem — are left
    standing; the fire launcher's `hide_companion_props` machinery was not
    ported here, flagged as a real gap in the notes, not silently
    dropped), NO street-level corridor pass beyond what `UT_LAYOUT_STREET`
    optionally leaves on, NO debris relocation out of a neighbour's
    footprint (the ground-evidence field rejects building FOOTPRINTS via
    `tornado_urban_ground._footprint_test`, but a per-building damage
    ledger's own shed debris is not re-checked against ITS NEIGHBOURS'
    footprints), NO crop window (this preset already IS the 500 m scene).
    Every removed piece is authored STATIC — `tornado_urban_usd.apply_plan`
    deactivates/displaces/voids glass and drops debris meshes; nothing here
    or downstream simulates any of it.

Env:
    SCENE_CONFIG    preset (default `downtown_tornado_bench_500`, round 2's
                    brownstone/low-rise/mid-rise bench with skyscrapers
                    routed OUT of the corridor — `_plans/urban_tornado_
                    plan.md` §7 R1/R2). Compiled DIRECTLY — no
                    `disaster-type` override dance: unlike
                    `disaster-type: fire`, `compile_disaster.DISASTERS` HAS
                    a `"tornado"` entry (`compile_tornado`), so
                    `load_scene_config(SCENE_CONFIG)` alone returns a
                    config whose `disaster.tornado` block already IS the
                    resolved track and whose layout is the ordinary city.
                    See `_neutralise_layout_disaster()` above/below for what
                    is scrubbed out of the REST of `disaster.*` before the
                    layout is actually built.
    UT_SEED         overrides the preset's own `seed` for every draw THIS
                    LAUNCHER makes downstream of the layout (the intensity
                    field's edge noise, the per-building level jitter, the
                    per-building plan/apply rng) — default: the preset's
                    own `seed`. The LAYOUT itself is always built from the
                    preset's `seed` (`generate_scene_on_stage` reads
                    `config["seed"]` directly and this launcher never
                    overrides that), so a different `UT_SEED` re-rolls only
                    which buildings get hit and how hard, never which
                    buildings exist or where.
    UT_MAX_BUILDINGS  int, 0 = unlimited (default). Caps the DAMAGED
                    `gac`/`dtc` list, ranked by intensity DESCENDING, so a
                    cap keeps the corridor's core and drops its weakest
                    edge first — never a random subset.
    UT_LAYOUT_STREET  0 (default) neutralises `trees_toppled_fraction` /
                    `streetlights_toppled_fraction` / `traffic_lights_
                    toppled_fraction` / `traffic_lights_leaning_fraction` /
                    `trash_cans_toppled_fraction` / `cars_toppled_fraction`
                    / `cars_strewn` / the path-debris density
                    (`disaster.debris.path_pieces_per_100m2` / `path_piles_
                    per_100m2`) ALONGSIDE the always-neutralised building/
                    human knobs, so v1 ships with an unambiguously intact
                    base city. `1` leaves this generic street-furniture/
                    ground-litter mechanism ON (it does not touch buildings
                    and does not conflict with this launcher's own ladder —
                    it is a judgement call under time pressure, not a known
                    bug, and the skill text explicitly allows a downtown
                    scene to keep "debris and a light dust/glass wash" on
                    its asphalt). `damaged_fraction`/`destroyed_fraction`/
                    `humans_prone_fraction`/`humans_strewn` are ALWAYS
                    zeroed regardless of this flag — see the module
                    docstring's "THE LAYOUT-TIME DISASTER-FIELD TRAP".
    TORNADO_MAX_H_M  passthrough, read directly by `disaster.tornado_city`
                    (not by this file) — see that module's own env var
                    (default 232.0, the fire pipeline's own cap).
    SNAP_DIR        viewport captures, MUST be under
                    /isaac-sim/.nvidia-omniverse/logs/ — the host-mounted
                    log tree; a path outside it produces PNGs the host can
                    never see (`run-isaac-sim-launcher` skill).
    UT_ENV          `default` (`PegasusInterface().load_environment(...)`,
                    like every other launcher) or `none` — the same knob
                    `urban_fire_city_launch_script.FC_ENV` offers, same
                    reasoning (the city authors its own ground/sky
                    regardless).
    UT_HIDE         `invisible` (default, `MakeInvisible`) or `deactivate`
                    (`SetActive(False)`) — `hide_intact`'s own choice,
                    identical to `urban_fire_city_launch_script.FC_HIDE`.
    UT_SKY          `sunset` (default, byte-identical dome-only look) or
                    `mid_day`, or an explicit HDRI/stage path — `urban_
                    fire_city_launch_script.FC_SKY`'s own knob, same
                    `sky_presets.py`.
    UT_UNINSTANCE_GPRIM_ROOTS  1 (default) repairs the same
                    instanced-gprim-root bug `urban_fire_city_launch_
                    script._uninstance_gprim_roots` fixes — this preset
                    also sets `instance_placements: true`, so the same
                    "streetlights/benches/some houses render nothing"
                    failure applies here unchanged. `0` disables it.
    UT_GROUND       1 (default) builds the corridor's ground evidence
                    after the damage loop — `disaster.tornado_urban_ground.
                    scatter_corridor`/`build` (a merged-mesh masonry-city
                    debris field, footprint-rejected against every house
                    placement) and `stain_overlay` (a translucent dust/
                    scour band, the fire scar's own band-mesh machinery
                    over an urban asphalt surface). `0` skips BOTH layers —
                    there is no finer split, R4a/R4b are one module's two
                    halves. See `TornadoCityApp.ground_evidence`.
    UT_GROUND_PER100  float, passthrough to `scatter_corridor`'s own
                    `per_100m2` density knob (default when unset: that
                    function's own default, 0.8 fragments per 100 sq m per
                    lattice cell, scaled by local intensity**1.4).
    UT_STREET       1 (default) runs THE STREET PASS (round 3, R9 --
                    `disaster.tornado_street.plan_street` + `TornadoCityApp.
                    street_pass`): the corridor's own streetlights/traffic
                    lights/signs/bus stops felled (in correlated runs),
                    benches/bins/cones/etc. carried away or tossed, street
                    trees tipped per `tornado.tree_level_for_intensity`,
                    parked cars pushed/rolled/thrown via `tornado.car_pose`
                    plus a core "thrown 20-60 m" promotion. `0` skips it
                    entirely and leaves every corridor prop exactly as the
                    intact layout placed it. REPLACES `UT_LAYOUT_STREET`'s
                    generic mechanism -- see that flag's own docstring.
    UT_MIN_TIPPED   int, default 2 -- the suburb's `TOR_MIN_TIPPED` review-
                    floor pattern (`tornado_street.plan_street`'s own
                    `min_tipped=`): if fewer than this many in-track cars
                    end up toppled after the ordinary draw, the highest-
                    intensity untouched candidates are forced over via
                    `car_pose(force="tip")` -- the DRAW only is skipped,
                    never the pose model -- until the floor is met or
                    candidates run out. The banner reports how many were
                    forced. `0` leaves the model exactly as measured; NEVER
                    set this to anything but 0 on a scene that is being
                    measured (this launcher's own default is 2, not 0 --
                    a deliberate deviation for review scenes, see the
                    `UT_MIN_TIPPED` constant's own comment above `main()`).
    KEEP_OPEN       1 keeps the app up after the captures.
    ISAAC_SIM_HEADLESS  true for a headless run.

Banner: `URBAN TORNADO CITY DONE` / `URBAN TORNADO CITY FAILED`.
"""

import json
import math
import os
import random
import sys
import time

import numpy as np

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default. Treat empty
    as absent. (`urban_fire_city_launch_script._env`, copied verbatim — see
    this file's own docstring for why this file cannot import that one.)"""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


def _flag(name, default="0"):
    return _env(name, default).lower() in ("1", "true", "yes")


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
# FRACTIONAL CUTOUT OPACITY — the same two required forms the fire launcher
# documents (the startup flag does not survive stage composition; the carb
# form alone is too late for startup). This launcher's own glass-void tone
# and debris materials are all opaque, so this is HARMLESS parity with the
# fire launcher rather than a hard requirement here — kept so a future
# capability (soot, fractional glass) does not silently need a second flag
# day.
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

# GUARDED, so this module can also be loaded as a library by a caller that
# already has its own `SimulationApp` running (the same convention `urban_
# fire_city_launch_script.py` documents for itself) — `_load_by_path` sets
# `__name__` to whatever string it is given, never `"__main__"`. Direct
# invocation (the pod workflow this file's whole docstring is written
# against) is unaffected: `__name__ == "__main__"` there, always.
if __name__ == "__main__":
    simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                                  "extra_args": KIT_ARGS})
else:
    simulation_app = None

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
# NOTE: omni.flowusd is NOT enabled — v1 authors no Flow (no live fire/smoke
# analogue for a tornado; nothing in this pipeline needs it).

import carb                                                    # noqa: E402
import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, UsdGeom                                # noqa: E402
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
from scene_generator import resolve_sky, _make_resolver          # noqa: E402
from sky_presets import apply_sky_preset                        # noqa: E402
from generate_scene import generate_scene_on_stage               # noqa: E402
from compile_disaster import load_scene_config                  # noqa: E402
from detail import gac_storey_slice as gss                      # noqa: E402
from disaster import fire_assembly_lib as fal                   # noqa: E402
from disaster import fracture                                   # noqa: E402
from disaster import gac_fire as gcf                             # noqa: E402
from disaster import quake_sliced as qs                         # noqa: E402
from disaster import tornado as tn                              # noqa: E402
from disaster import tornado_city as tc                         # noqa: E402
from disaster import tornado_kit as tk                           # noqa: E402
from disaster import tornado_street as tst                       # noqa: E402
from disaster import tornado_urban_ground as tug                 # noqa: E402
from disaster import tornado_urban_usd as tuu                   # noqa: E402
from disaster import urban_fire_city as ufc                     # noqa: E402


ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
PARENT = "/World/stage/generated"
TORNADO_ROOT = "/World/tornado"
_ENV_CLUTTER = {"GroundPlane", "Environment"}
#: FAMILY string `gac_storey_slice.slice_to_kit` wants, keyed by measured
#: construction type — `tools/tornado_urban_probe.py`'s own table, verbatim.
FAMILY_OF_BTYPE = {"urm": "01", "rc": "02", "rc_glass": "05"}

SCENE_CONFIG = _env("SCENE_CONFIG", "downtown_tornado_bench_500")
UT_SEED = _env("UT_SEED", "")
MAX_BUILDINGS = int(_env("UT_MAX_BUILDINGS", "0") or "0")
LAYOUT_STREET = _flag("UT_LAYOUT_STREET", "0")
SNAP_DIR = _env("SNAP_DIR", "")
KEEP_OPEN = _flag("KEEP_OPEN", "0")
UT_ENV = _env("UT_ENV", "default")
HIDE_MODE = _env("UT_HIDE", "invisible").lower()
SKY = _env("UT_SKY", "sunset")
UNINSTANCE_GPRIM_ROOTS = _flag("UT_UNINSTANCE_GPRIM_ROOTS", "1")
# ROUND 2 (`_plans/urban_tornado_plan.md` §7) -- the ground-evidence layer
# (`disaster.tornado_urban_ground.scatter_corridor`/`build`/`stain_overlay`,
# see `TornadoCityApp.ground_evidence`). `UT_GROUND=0` skips BOTH the debris
# field and the stain overlay -- there is no finer on/off split, the two
# layers are always built together (R4a/R4b are one module's own two halves,
# not independent knobs). `UT_GROUND_PER100`, if given, overrides `scatter_
# corridor`'s own `per_100m2` density default (0.8); left `None` (the knob's
# own default applies) when the env var is absent/empty.
UT_GROUND = _flag("UT_GROUND", "1")
_ut_ground_per100_raw = _env("UT_GROUND_PER100", "")
UT_GROUND_PER100 = float(_ut_ground_per100_raw) if _ut_ground_per100_raw \
    else None
# ROUND 3 (`_plans/urban_tornado_plan.md` §8, R9, stream SP) -- THE STREET
# PASS (`disaster.tornado_street.plan_street`, see `TornadoCityApp.
# street_pass`). `UT_STREET=0` skips it entirely -- the corridor's own
# props (streetlights, signals, signs, bus stops, benches, bins, street
# trees, parked cars) are then left exactly as the intact-city layout put
# them, same as `UT_GROUND=0`'s own all-or-nothing split. This pass
# REPLACES `UT_LAYOUT_STREET`'s generic layout-time street mechanism (see
# the module docstring's "THE LAYOUT-TIME DISASTER-FIELD TRAP" and
# `_neutralise_layout_disaster`) -- it does not sit beside it, so
# `UT_LAYOUT_STREET` stays at its own default (0) regardless of this flag.
UT_STREET = _flag("UT_STREET", "1")
# `UT_MIN_TIPPED` -- the suburb's `TOR_MIN_TIPPED` review-floor pattern
# (see that launcher's own block comment), ported as `tornado_street.
# plan_street`'s `min_tipped=`. DEFAULT 2, NOT 0 -- a deliberate deviation
# from the suburb file's own "never non-zero by default" convention,
# per the 2026-09-01 mid-round car directive ("do cars get overturned/
# thrown? Cause you can do that"): this launcher's primary use today is
# building review scenes, and `car_pose`'s own tip share on a corridor
# that mostly sits at i 0.2-0.4 makes "zero visibly rolled cars" the
# ordinary outcome of a deterministic seed, exactly the situation that
# knob exists to fix. A MEASURED run (`_plans/urban_tornado_W3SP_notes.
# md`) must set `UT_MIN_TIPPED=0` explicitly -- `plan_street`'s own
# `min_tipped=` default is 0, unchanged, so the pure model is never forced
# unless this env var (or an explicit kwarg) says so. Forced cars are
# marked `"forced": True` and reported separately in the banner; they are
# never also promoted to "thrown" (see `tornado_street`'s own docstring).
UT_MIN_TIPPED = int(_env("UT_MIN_TIPPED", "2") or "0")


def vram_mb(tag):
    """`fire_assembly_lib.vram_mb`, printing under this launcher's own
    prefix — the ONE thing this file imports from the fire pipeline's own
    module, because it is generic (an `nvidia-smi` read) and has nothing
    fire-specific in it."""
    return fal.vram_mb(tag, prefix="ut")


# ---------------------------------------------------------------------------
# The city (copied from `urban_fire_city_launch_script.py`, which copied it
# from `downtown_quake_launch_script.py` / `scene_launch_script.py` — NOT
# imported, for the second-SimulationApp reason the module docstring gives)
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
    print("[ut] env clutter: {0} prim(s) deactivated".format(n))


def _disable_sky_sun(stage):
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
    print("[ut] sky sun: {0} DistantLight(s) disabled".format(n))


def _wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            if [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]:
                return True
        time.sleep(0.1)
    return False


# ---------------------------------------------------------------------------
# THE LAYOUT-TIME DISASTER-FIELD TRAP — see the module docstring's own
# section by this name for the full derivation.
# ---------------------------------------------------------------------------
#: never safe to leave on for this pipeline: building fate is THIS
#: launcher's own job (a generic tilt+sink stand-in directly violates the
#: round's "never leans or sinks" rule) and humans are out of scope
#: entirely (a separate, reviewed pipeline owns people).
_ALWAYS_ZERO_FRACTIONS = ("damaged_fraction", "destroyed_fraction",
                          "humans_prone_fraction")
_ALWAYS_ZERO_RANGES = ("humans_strewn",)
#: gated behind `UT_LAYOUT_STREET` — generic street-furniture toppling and
#: ground-litter density. Does not touch buildings, does not conflict with
#: this launcher's own ladder; off by default in v1 purely so every pixel
#: of damage in the scene is attributable to a reviewed pass.
_STREET_ZERO_FRACTIONS = ("trees_toppled_fraction",
                          "streetlights_toppled_fraction",
                          "traffic_lights_toppled_fraction",
                          "traffic_lights_leaning_fraction",
                          "trash_cans_toppled_fraction",
                          "cars_toppled_fraction")
_STREET_ZERO_RANGES = ("cars_strewn",)
_STREET_ZERO_DEBRIS_KEYS = ("path_pieces_per_100m2", "path_piles_per_100m2")


def _neutralise_layout_disaster(config):
    """Zero, IN MEMORY, the `config["disaster"]` fraction/range knobs
    `scene_generator.build_city` reads GENERICALLY for house fate, street
    furniture and humans — see the module docstring's "THE LAYOUT-TIME
    DISASTER-FIELD TRAP" for what each one does and why it is dangerous
    left on. Leaves `config["disaster"]["tornado"]` (the compiled track)
    and `config["disaster"]["field"]` (now inert for placement purposes,
    since every fraction consumer is zeroed) untouched — this function
    never walks into either.

    Call AFTER `load_scene_config`, BEFORE `generate_scene_on_stage`: the
    same dict object is threaded all the way down to `scene_generator.
    build_city` (`generate_scene.generate_scene_on_stage` passes `config`
    straight through, no copy), so mutating it here is sufficient — no
    `spec_overrides` recompile needed, unlike fire's `disaster-type:
    "none"` override (which would also wipe `disaster.tornado` itself).
    """
    dis = config.get("disaster")
    if not isinstance(dis, dict):
        return
    zeroed = []
    for key in _ALWAYS_ZERO_FRACTIONS:
        if dis.get(key):
            dis[key] = 0.0
            zeroed.append(key)
    for key in _ALWAYS_ZERO_RANGES:
        if dis.get(key):
            dis[key] = [0, 0]
            zeroed.append(key)
    if not LAYOUT_STREET:
        for key in _STREET_ZERO_FRACTIONS:
            if dis.get(key):
                dis[key] = 0.0
                zeroed.append(key)
        for key in _STREET_ZERO_RANGES:
            if dis.get(key):
                dis[key] = [0, 0]
                zeroed.append(key)
        debris = dis.get("debris")
        if isinstance(debris, dict):
            for key in _STREET_ZERO_DEBRIS_KEYS:
                if debris.get(key):
                    debris[key] = 0.0
                    zeroed.append("debris." + key)
    print("[ut] neutralised {0} layout-time disaster knob(s) so the intact-"
          "city step is ACTUALLY intact (UT_LAYOUT_STREET={1}): {2}".format(
              len(zeroed), int(LAYOUT_STREET), ", ".join(zeroed) or "(none)"))


# ---------------------------------------------------------------------------
# instanced-gprim-root repair — `urban_fire_city_launch_script.
# _uninstance_gprim_roots`, copied verbatim (this preset also sets
# `instance_placements: true`, so the same bug applies unchanged: a
# placement whose composed ROOT PRIM IS A MESH (streetlights, some
# benches/cars, 10 of the 500 m fire city's own building assets) renders
# NOTHING when instanced, because its prototype ends up holding only the
# GeomSubsets/Looks with no Mesh in it).
# ---------------------------------------------------------------------------
def _uninstance_gprim_roots(stage, placements):
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
        print("[ut] un-instanced {0} placement(s) whose root prim is a mesh "
              "({1})".format(len(fixed), ", ".join(
                  "{0}={1}".format(k, v) for k, v in sorted(
                      by_cat.items(), key=lambda kv: str(kv[0])))))
    else:
        print("[ut] un-instance pass: no placement has a gprim root "
              "(nothing to repair)")
    return fixed


def hide_intact(stage, path):
    """`MakeInvisible` (default) or `SetActive(False)` on the intact prim —
    `urban_fire_city_launch_script.hide_intact`, copied verbatim (same
    `MakeInvisible` returns-nothing gotcha: the visibility attribute is read
    back rather than trusting the call's own return value)."""
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return False
    if HIDE_MODE.startswith("deact"):
        return bool(prim.SetActive(False))
    img = UsdGeom.Imageable(prim)
    if not img:
        return bool(prim.SetActive(False))
    img.MakeInvisible()
    return img.GetVisibilityAttr().Get() == UsdGeom.Tokens.invisible


def _safe_token(s):
    """A USD-prim-legal token from an arbitrary string — the same one-line
    idiom `tornado_urban_usd._safe_name` uses for a material hint, inlined
    here rather than imported (that helper is private to its module)."""
    s = str(s or "unknown")
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in s)
    return out or "unknown"


def place_holder(stage, stem, x, y, z, yaw_deg, ssf):
    """THE FRESH HOLDER — see the module docstring's "THE TRANSFORM TRAP".
    Returns `(holder_path, cell_path)`; `cell` is an Xform (never
    `Scope.Define`), the CHILD everything for this building is built onto."""
    holder = "{0}/{1}".format(TORNADO_ROOT, _safe_token(stem))
    xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x) * ssf, float(y) * ssf,
                                     float(z) * ssf))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(float(ssf), float(ssf), float(ssf)))
    cell = holder + "/cell"
    UsdGeom.Xform.Define(stage, Sdf.Path(cell))
    return holder, cell


# ---------------------------------------------------------------------------
# THE STREET PASS (round 3, R9, stream SP) -- application logic EXTRACTED
# into `disaster.tornado_street` on 2026-09-01 (`_normalize_yaw_op`,
# `_apply_tree_level`, `_apply_action`, `apply_street`) so a non-launcher
# caller (stream B's bench) can wire `tornado_street.plan_street`'s own
# output onto a stage without importing this file (a launcher constructs
# a `SimulationApp` at import time, so it cannot be imported anywhere
# else -- see this module's own docstring). `TornadoCityApp.street_pass`
# (below) is now a thin wrapper: it resolves env vars into `plan_street`'s
# arguments, calls `plan_street` then `tornado_street.apply_street`, and
# does launcher-only bookkeeping (VRAM capture) with what comes back. See
# `disaster/tornado_street.py`'s own "APPLY" section for the moved code
# and the full rationale.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# FC_DUMP-equivalent — the placements Kit actually built, in the SAME
# `fire_city_placements_dump.v1` shape `tools/fire_city_dry_run.py`/
# `tools/tornado_city_dry_run.py` already read, so any tool built against
# that schema works unchanged against this launcher's own dump. Reused/
# copied from `urban_fire_city_launch_script.dump_city_placements` — see
# that function's own docstring for the full "why re-measure here" case;
# trimmed of the FC_CROP_WINDOW `_fc_cropped` skip (this launcher has no
# crop feature, v1).
# ---------------------------------------------------------------------------
def default_dump_path(preset, seed):
    return os.path.join(_SCENE_GEN_DIR, "_plans",
                        "city_placements_{0}_{1}.json".format(preset, seed))


def _typology_rects(layout):
    """`[{"rect": [x0, y0, x1, y1], "name": name}, ...]` — JSON has no tuple
    keys, so `layout["_typology_of"]` is serialised as a list."""
    return [{"rect": [float(v) for v in rect], "name": name}
            for rect, name in (layout or {}).get("_typology_of", {}).items()]


def dump_city_placements(path, preset, seed, config, placements, layout,
                         resolver):
    """Write the city-placements dump — see this module's own block comment
    above. `resolver` is `_make_resolver(config)`, passed in rather than
    rebuilt here so the SAME resolver instance also drives the per-building
    damage-selection loop (`TornadoCityApp.select_damage`), one measurement
    pass, not two.

    Only `category == "house"` placements are written (the same "one to two
    orders of magnitude of non-house placements would bloat this file for
    no downstream use" reasoning the fire launcher's own docstring gives).
    Every house record carries its own `i` — its index in the FULL
    `placements` list this process just built — so a future tool solving on
    this dump can `resolve_cell`-style match a record back to a live
    rebuild the same way the fire pipeline does.
    """
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
    print("[ut] wrote city placements dump ({0} house placement(s) of {1} "
          "total, {2} typology block(s)) -> {3}".format(
              len(houses), len(placements),
              len(doc["typology"]["blocks"]), path))
    return doc


# ---------------------------------------------------------------------------
# The app
# ---------------------------------------------------------------------------
class TornadoCityApp:
    def __init__(self):
        self.t0 = time.time()
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()
        self.vram = {}
        self.n_hidden = 0

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        env_mode = UT_ENV.lower()
        loaded_env_url = None
        if env_mode in ("none", "0", "off", "skip"):
            print("[ut] UT_ENV=none — skipping PegasusInterface()."
                  "load_environment(); the city authors its own ground/sky")
        else:
            loaded_env_url = ENV_URL if env_mode in ("", "default") else UT_ENV
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
        self.vram["empty"] = vram_mb("empty stage")

        # -- 1) the city, compiled DIRECTLY — disaster-type: tornado IS a
        # compiled type (unlike fire's fake five top-level keys), so this
        # ONE call already returns both the ordinary city layout and the
        # resolved track. See the module docstring's "THE LAYOUT-TIME
        # DISASTER-FIELD TRAP" for why the result is neutralised below
        # before it is built.
        print("[ut] compiling {0} (disaster-type: tornado compiles "
              "directly — no override dance)".format(SCENE_CONFIG))
        config = load_scene_config(SCENE_CONFIG)
        _neutralise_layout_disaster(config)
        self.config = config
        self.seed = int(UT_SEED) if UT_SEED else int(config.get("seed", 0))

        t_layout = time.time()
        self.placements = generate_scene_on_stage(
            stage, config, parent_path=PARENT, scene_scale_factor=self.ssf)
        self.uninstanced = _uninstance_gprim_roots(stage, self.placements)
        for _ in range(10):
            omni.kit.app.get_app().update()
        settle_rigid_props(
            stage,
            [p["prim_path"] for p in self.placements
             if p.get("settle") and p.get("prim_path")],
            ground_path=PARENT + "/ground")
        if SKY in ("", "sunset"):
            add_sky(stage, resolve_sky(config),
                    intensity=float(config.get("sky_intensity", 3500.0)),
                    exposure=float(config.get("sky_exposure", -3.0)))
            _disable_sky_sun(stage)
        else:
            _disable_sky_sun(stage)
            resolved_sky = apply_sky_preset(stage, SKY, prefix="ut")
            print("[ut] sky: UT_SKY={0} -> preset '{1}'".format(
                SKY, resolved_sky))
        for _ in range(10):
            omni.kit.app.get_app().update()
        self.t_layout = time.time() - t_layout
        self.region = [float(v) for v in config["layout"]["region_m"]]
        self.n_houses = sum(1 for p in self.placements
                            if p.get("category") == "house")
        print("[ut] city: {0:.0f} x {1:.0f} m, {2} placement(s), {3} "
              "building(s), layout in {4:.0f} s".format(
                  self.region[0], self.region[-1], len(self.placements),
                  self.n_houses, self.t_layout))
        self.vram["intact"] = vram_mb(
            "city built INTACT ({0} buildings)".format(self.n_houses))

        # -- 2) dump the placements Kit actually built --------------------
        resolver = _make_resolver(config)
        self.dump_path = default_dump_path(SCENE_CONFIG, self.seed)
        self.dump_doc = dump_city_placements(
            self.dump_path, SCENE_CONFIG, self.seed, config,
            self.placements, config.get("_city_layout") or {}, resolver)

        # -- 3) the track ---------------------------------------------------
        self.tcfg = tn.resolve_cfg(config)
        w, h = self.region
        self.plate = (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)
        self.inten = tn.intensity_field(
            self.tcfg, self.plate, np.random.default_rng(self.seed + 23))
        print("[ut] track: width {0:.1f} m, peak {1:.3f}, core_frac {2:.3f}, "
              "origin {3}, heading {4:.1f} deg, curvature {5:.1f} deg/km"
              .format(self.tcfg["width_m"], self.tcfg["peak"],
                      self.tcfg["core_frac"], self.tcfg["origin_m"],
                      self.tcfg["heading_deg"],
                      self.tcfg.get("curvature_deg_per_km", 0.0)))

    # -- 4) gate + level-draw every house placement, EXACTLY the dry run's
    # own per-building convention (`tools/tornado_city_dry_run.py`), so a
    # dry-run manifest solved on this same dump/seed and this live scene
    # agree building-for-building. ------------------------------------------
    def select_damage(self):
        layout = self.config.get("_city_layout") or {}
        records, refused = [], []
        n_t0 = 0
        for h in self.dump_doc["placements"]:
            i = int(h["i"])
            p = self.placements[i]
            usd = h["usd"]
            x, y, z = float(h["x_m"]), float(h["y_m"]), float(h["z_m"])
            yaw = float(h["yaw_deg"])
            W, D, H = float(h["W"]), float(h["D"]), float(h["H"])

            i_val = float(self.inten(x, y))
            wind = tn.wind_at(self.tcfg, x, y)

            ok, reason, route, kind, name, bakeable = tc.damageable(
                p, {usd: (W, D, H)})

            # THE DRY RUN'S OWN PER-BUILDING SEED CONVENTION, verbatim, so a
            # dry-run manifest solved on this dump draws the identical level
            # this launcher does.
            level_rng = random.Random(self.seed * 1000003 + i)
            level = tc.level_for_intensity(i_val, level_rng)

            if not ok:
                refused.append({"i": i, "cell": h["cell"], "usd": usd,
                                "name": name, "H": H, "x": x, "y": y,
                                "intensity": round(i_val, 5), "level": level,
                                "reason": reason})
                continue
            if level == "T0":
                n_t0 += 1
                continue

            typology = ufc.typology_at(layout, x, y)
            btype = tc.btype_for(usd, H)
            height_class = tc.height_class_for(H, typology)
            rec = tc.record(i, h["cell"], usd, kind, name, x, y, yaw, W, D,
                            H, btype, height_class, i_val, level, wind,
                            route, bakeable, self.seed)
            # `tc.record` carries no `z` field (the earthquake/fire
            # manifests never needed one) — this launcher's `place_holder`
            # does, so it is added here rather than widening the shared
            # schema for one caller.
            rec["z"] = z
            rec["typology"] = typology
            records.append(rec)

        self.records = records
        self.refused = refused
        self.n_t0 = n_t0
        n_house = len(self.dump_doc["placements"])
        print("[ut] gate+level: {0} house(s) -> {1} T1+ record(s), {2} T0, "
              "{3} refused".format(n_house, len(records), n_t0,
                                   len(refused)))
        return records

    # -- 5) apply the ladder to every gac/dtc record -------------------------
    def apply_damage(self):
        stage = self.stage
        UsdGeom.Xform.Define(stage, Sdf.Path(TORNADO_ROOT))

        damaged_all = sorted(
            (r for r in self.records if r["kind"] in ("gac", "dtc")),
            key=lambda r: -r["intensity"])
        # ROUND 2 (R3): `kind == "kit"` is no longer left intact — every
        # T1+ kit record is ATTEMPTED below, in its own pass right after
        # this one (`tornado_kit.wreck_kit`). Kept as its own list (not
        # merged into `damaged_all`) because it runs a completely different
        # call sequence (no `gac_fire.place_source`/`gac_storey_slice.
        # slice_to_kit` — see that pass's own comment) and is not subject
        # to `UT_MAX_BUILDINGS` (that knob's own docstring says "gac/dtc
        # list" — kit buildings are the cheaper, smaller stock this round
        # exists to SHOW damaged, not the one to budget-drop first).
        kit_records = [r for r in self.records if r["kind"] == "kit"]
        slice_intact = [r for r in self.records if r["kind"] == "slice"]
        # ROUND 3 (R11): industrial sheds get their own pass (5c below) —
        # `tornado_collapse.plan_industrial`/`apply_industrial`, no piece
        # grid at all, so neither the gac/dtc nor the kit call sequence
        # applies.
        industrial_records = [r for r in self.records
                              if r["kind"] == "industrial"]

        dropped_by_budget = []
        if MAX_BUILDINGS > 0 and len(damaged_all) > MAX_BUILDINGS:
            dropped_by_budget = damaged_all[MAX_BUILDINGS:]
            damaged = damaged_all[:MAX_BUILDINGS]
            print("[ut] UT_MAX_BUILDINGS={0}: keeping the {0} highest-"
                  "intensity gac/dtc record(s) of {1}, {2} weaker one(s) "
                  "left intact".format(MAX_BUILDINGS, len(damaged_all),
                                       len(dropped_by_budget)))
        else:
            damaged = damaged_all

        fracture.ensure_vtk(verbose=True)

        placed, failed = [], []
        total_pieces = 0
        t_apply0 = time.time()
        for n, rec in enumerate(damaged):
            i, name, kind = rec["i"], rec["name"], rec["kind"]
            x, y, z, yaw = rec["x"], rec["y"], rec["z"], rec["yaw"]
            level, height_class = rec["level"], rec["height_class"]
            stem = "{0}_{1}_{2}".format(kind, name, i)
            cell_prim_path = rec["cell"]

            if hide_intact(stage, cell_prim_path):
                self.n_hidden += 1
            else:
                print("[ut] WARNING: could not hide {0}".format(
                    cell_prim_path))

            holder, cell = place_holder(stage, stem, x, y, z, yaw, self.ssf)

            t_b0 = time.time()
            try:
                pack = gcf.PACKS[kind]
                url = gcf.asset_url(name, kind=kind)
                scale = gcf.asset_scale(url, pack["scale"], verbose=False)
                src = gcf.place_source(stage, cell, url, scale)
                if not src:
                    raise RuntimeError(
                        "place_source composed nothing for {0}".format(name))

                btype_guess = qs.construction_type(name)
                family = FAMILY_OF_BTYPE.get(btype_guess, "01")
                style = "ut_{0}_{1}".format(_safe_token(name).lower(), i)
                force_regular = name in (pack.get("force_regular_grid") or ())
                pls, grid, measured = gss.slice_to_kit(
                    stage, src, cell, style, region=None, family=family,
                    verbose=False, force_regular=force_regular)

                tuu.annotate_glazing(stage, pls)

                # THE WIND-FRAME ROTATION — see the module docstring.
                wind_local = dict(rec["wind"])
                wind_local["bearing_deg"] = (
                    float(rec["wind"]["bearing_deg"]) - float(yaw)) % 360.0

                # A single per-building seed drives BOTH rng streams handed
                # to `wreck_urban` — the SAME `seed * 1000003 + i` convention
                # `select_damage`'s own level draw uses, reused here rather
                # than invented separately (`random.Random` and `np.random.
                # default_rng` are independent generator algorithms even
                # fed the same integer, and this repo's own probe seeds both
                # of ITS rng objects off one `SEED` the same way).
                rec_seed = self.seed * 1000003 + i
                ctx = tuu.wreck_urban(
                    stage, cell, pls, style, level, random.Random(rec_seed),
                    np.random.default_rng(rec_seed), {}, "ut", wind_local,
                    btype=None, height_class=height_class,
                    intensity=rec["intensity"], usd=name, verbose=False)

                # THE MERGED SOURCE STAYS COMPOSED — do NOT deactivate it.
                # The brief's original instruction ("deactivate <cell>/src")
                # was WRONG and was caught in lead review before the first
                # launch: every sliced piece's GeomSubsets bind materials
                # that live INSIDE the source's subtree
                # (`<cell>/src/asset/LOD0/Section*/UnrealMaterial` — measured
                # in the round-1 glass probe), and `SetActive(False)` prunes
                # the subtree from composition, which unbinds them all and
                # renders every piece fallback-white. `slice_to_kit` already
                # calls `UsdGeom.Imageable(src).MakeInvisible()` on it
                # (gac_storey_slice.py, end of the slice), which hides the
                # merged original while keeping its materials composed and
                # resolvable — exactly what an in-session assembly needs.
                # A future BAKE breaks the dependency for good with
                # `fire_bake.rehome_for_export`; this launcher never exports.

                plan = ctx.get("plan") or {}
                counts = ctx.get("counts") or {}
                n_pieces = len(pls)
                total_pieces += n_pieces
                dt = time.time() - t_b0
                rec_out = dict(rec, n_pieces=n_pieces, slice_s=round(dt, 2),
                              counts=counts, stats=plan.get("stats") or {},
                              holder=holder, cell=cell, style=style)
                placed.append(rec_out)
                v = vram_mb("d{0} {1} {2} {3} done".format(i, name, kind,
                                                           level))
                print("[ut] d{0:<3} {1:<24} {2:<3} {3:<5} btype={4:<8} "
                      "hc={5:<8} H {6:6.1f} m  {7:4} pieces  {8:5.2f}s  "
                      "removed={9} glass={10} displaced={11} debris={12}  "
                      "vram={13}".format(
                          i, name, kind, level, rec.get("btype"),
                          height_class, rec["H"], n_pieces, dt,
                          counts.get("n_removed", 0),
                          counts.get("n_glass", 0),
                          counts.get("n_displaced", 0),
                          counts.get("n_fragments", 0),
                          "{0:.0f} MiB".format(v) if v is not None else "?"))
            except Exception as exc:                              # noqa: BLE001
                import traceback
                traceback.print_exc()
                print("[ut] *** FAILED building {0} ({1}, {2}): {3}".format(
                    name, kind, level, exc))
                failed.append({"rec": rec, "error": str(exc)})
            for _ in range(2):
                omni.kit.app.get_app().update()

        self.placed = placed
        self.failed = failed
        self.total_pieces = total_pieces
        self.damaged_all = damaged_all
        self.dropped_by_budget = dropped_by_budget
        self.slice_intact = slice_intact
        self.t_apply = time.time() - t_apply0
        print("[ut] applied damage to {0}/{1} building(s) ({2} failed) in "
              "{3:.0f} s, {4} total piece(s)".format(
                  len(placed), len(damaged), len(failed), self.t_apply,
                  total_pieces))
        self.vram["damaged"] = vram_mb(
            "{0} building(s) damaged live, no Flow".format(len(placed)))

        # -- 5b) KIT-BUILDING DAMAGE (round 2, R3) — same holder/hide
        # convention as the gac/dtc pass above, `disaster.tornado_kit.
        # wreck_kit` in place of `gac_fire.place_source` + `gac_storey_
        # slice.slice_to_kit` + `tornado_urban_usd.wreck_urban`. A SEPARATE
        # pass (not folded into the loop above) because a kit record's
        # `name` IS the style (`tornado_city.damageable` -> `urban_fire_
        # city.bake_kind`'s own "kit" branch, verified against `bake_kind`'s
        # docstring: `('kit', style) -> ("kit", style)`), so there is no
        # `gcf.PACKS[kind]`/`gcf.asset_url`/`gac_storey_slice` step at all —
        # `wreck_kit` owns the ENTIRE build-and-damage sequence itself
        # (`kit_substitute.build_kit` -> `annotate_glazing` -> `describe`/
        # `adapt` -> `plan_damage` -> `apply_plan`, see that function's own
        # docstring).
        #
        # Kept OUT of `self.placed`/`self.failed` accumulation until the
        # very end (`self.failed = self.failed + kit_failed` below) so the
        # banner's existing, UNCHANGED print statements (`len(self.placed)`
        # labelled "damaged (gac/dtc)") stay literally true — `self.placed` is
        # gac/dtc only, `self.kit_placed` is the new, separate kit list the
        # report() table concatenates in.
        kit_placed, kit_failed = [], []
        kit_total_pieces = 0
        t_kit0 = time.time()
        for n, rec in enumerate(kit_records):
            i, style, kind = rec["i"], rec["name"], rec["kind"]
            x, y, z, yaw = rec["x"], rec["y"], rec["z"], rec["yaw"]
            level, height_class = rec["level"], rec["height_class"]
            stem = "{0}_{1}_{2}".format(kind, style, i)
            cell_prim_path = rec["cell"]

            if hide_intact(stage, cell_prim_path):
                self.n_hidden += 1
            else:
                print("[ut] WARNING: could not hide {0}".format(
                    cell_prim_path))

            holder, cell = place_holder(stage, stem, x, y, z, yaw, self.ssf)

            t_b0 = time.time()
            try:
                # THE WIND-FRAME ROTATION — identical to the gac/dtc pass
                # (see the module docstring): `tornado_kit.adapt` stamps
                # `_side` in the SAME cell-local S/E/N/W convention a real
                # slice uses (`quake_flow.classify`'s own side test, not a
                # world-frame one), so a kit holder's own yaw needs the
                # same rotation into the cell's local frame.
                wind_local = dict(rec["wind"])
                wind_local["bearing_deg"] = (
                    float(rec["wind"]["bearing_deg"]) - float(yaw)) % 360.0

                # SAME per-record seed convention as the gac/dtc pass —
                # `select_damage`'s own `seed * 1000003 + i`. `wreck_kit`'s
                # `seed=` kwarg is a THIRD, separate draw (`kit_substitute.
                # build_kit`'s own `random.Random(seed)`, `urban_building.
                # build_building`'s placement jitter) — distinct from the
                # `rng`/`nrng` pair it also takes for the damage planner,
                # by that function's own signature. `% 1000` only keeps it
                # a short, readable int in the per-building log line.
                #
                # `ssf=1.0`, NOT `self.ssf` — VERIFIED against `kit_
                # substitute.build_kit`'s own body (`sg.apply_placements
                # (stage, pls, cell + "/parts", ssf)`): `cell` is `place_
                # holder`'s CHILD Xform and already inherits the holder's
                # own `scale = self.ssf` (the transform trap, see the
                # module docstring), so passing `self.ssf` here a second
                # time would compose `self.ssf ** 2` onto the kit's own
                # pieces — shrinking (or growing) every one of them by the
                # stage's own unit ratio a second time. `ssf=1.0` matches
                # `tools/tornado_kit_probe.py`'s own call (a cell with NO
                # holder above it, at stage scale 1 already) and keeps
                # `cell`'s children in the SAME real-metres frame the
                # gac/dtc pass's sliced pieces already use.
                rec_seed = self.seed * 1000003 + i
                ctx = tk.wreck_kit(
                    stage, cell, style, level, random.Random(rec_seed),
                    np.random.default_rng(rec_seed), {}, "ut", wind_local,
                    height_class=height_class, intensity=rec["intensity"],
                    ssf=1.0, seed=rec_seed % 1000, verbose=False)

                plan = ctx.get("plan") or {}
                counts = ctx.get("counts") or {}
                n_pieces = int((ctx.get("kit") or {}).get("n_pieces", 0))
                kit_total_pieces += n_pieces
                dt = time.time() - t_b0
                rec_out = dict(rec, n_pieces=n_pieces, slice_s=round(dt, 2),
                              counts=counts, stats=plan.get("stats") or {},
                              holder=holder, cell=cell, style=style)
                kit_placed.append(rec_out)
                v = vram_mb("d{0} {1} {2} {3} done".format(i, style, kind,
                                                           level))
                print("[ut] k{0:<3} {1:<24} {2:<3} {3:<5} btype={4:<8} "
                      "hc={5:<8} H {6:6.1f} m  {7:4} pieces  {8:5.2f}s  "
                      "removed={9} glass={10}(+{11} knocked out) "
                      "displaced={12} debris={13}  vram={14}".format(
                          i, style, kind, level, rec.get("btype"),
                          height_class, rec["H"], n_pieces, dt,
                          counts.get("n_removed", 0),
                          counts.get("n_glass", 0),
                          counts.get("n_glass_removed", 0),
                          counts.get("n_displaced", 0),
                          counts.get("n_fragments", 0),
                          "{0:.0f} MiB".format(v) if v is not None else "?"))
            except Exception as exc:                              # noqa: BLE001
                import traceback
                traceback.print_exc()
                print("[ut] *** FAILED kit building {0} ({1}, {2}): {3}"
                      .format(style, kind, level, exc))
                kit_failed.append({"rec": rec, "error": str(exc)})
            for _ in range(2):
                omni.kit.app.get_app().update()

        self.kit_records = kit_records
        self.kit_placed = kit_placed
        self.kit_failed = kit_failed
        # `self.kit_intact` — kept as a distinct field (the banner's own,
        # UNCHANGED print statement reads `len(self.kit_intact)`) but its
        # MEANING moves with R3: no kit record is deliberately left intact
        # any more, so this is now "kit records this pass attempted but
        # could not damage" (multi-mass `tornado_kit._refuse_if_unsupported`
        # styles, or any other exception) rather than "no damage path
        # existed at all". Those buildings are hidden with nothing placed
        # in their stead (the SAME accepted hide-first-then-attempt
        # ordering the gac/dtc pass above already uses) — real failures,
        # not silently-fine ones, which is why they ALSO land in `self.
        # failed` (the banner's existing "N building(s) FAILED to apply"
        # line already reports them by name/kind).
        self.kit_intact = kit_failed
        self.failed = self.failed + kit_failed
        self.total_pieces += kit_total_pieces
        print("[ut] kit damage: applied to {0}/{1} building(s) ({2} failed) "
              "in {3:.0f} s, {4} total piece(s)".format(
                  len(kit_placed), len(kit_records), len(kit_failed),
                  time.time() - t_kit0, kit_total_pieces))
        self.vram["kit_damaged"] = vram_mb(
            "{0} kit building(s) damaged live".format(len(kit_placed)))

        # -- 5c) INDUSTRIAL COLLAPSE (round 3, R11, stream C3's hook —
        # applied by the lead from `_plans/urban_tornado_C3_notes.md`'s
        # snippet, with ONE correction: the snippet passed `rec["wind"]`
        # in the WORLD frame, but `plan_industrial` runs in the cell's own
        # local frame under a yawed holder, so the bearing rotates by -yaw
        # exactly as the gac/dtc and kit passes above do).
        industrial_placed, industrial_failed = [], []
        t_ind0 = time.time()
        if industrial_records:
            try:
                from disaster import tornado_collapse as tcol
            except Exception as exc:                            # noqa: BLE001
                tcol = None
                print("[ut] SKIP industrial pass: disaster.tornado_collapse "
                      "not importable ({0}) — {1} record(s) left intact"
                      .format(exc, len(industrial_records)))
        for rec in (industrial_records if industrial_records else ()):
            if tcol is None:
                break
            i, name = rec["i"], rec["name"]
            x, y, yaw = rec["x"], rec["y"], rec["yaw"]
            grade = tcol.grade_for_intensity(rec["intensity"])
            if grade is None:
                print("[ut] industrial {0}: i {1:.2f} below the partial "
                      "grade — left intact".format(name, rec["intensity"]))
                continue
            stem = "industrial_{0}_{1}".format(_safe_token(name), i)
            if hide_intact(stage, rec["cell"]):
                self.n_hidden += 1
            holder, cell = place_holder(stage, stem, x, y,
                                        rec.get("z", 0.0) or 0.0, yaw,
                                        self.ssf)
            t_b0 = time.time()
            try:
                scale = gcf.asset_scale(rec["usd"], 1.0, verbose=False)
                src = gcf.place_source(stage, cell, rec["usd"], scale)
                if not src:
                    raise RuntimeError("place_source composed nothing")
                wind_local = dict(rec["wind"])
                wind_local["bearing_deg"] = (
                    float(rec["wind"]["bearing_deg"]) - float(yaw)) % 360.0
                rng = random.Random(self.seed * 1000003 + i)
                plan = tcol.plan_industrial(
                    rec["W"], rec["D"], rec["H"], 0.0, 0.0, 0.0, grade,
                    wind_local, rng)
                ctx = {"stage": stage, "parent": cell, "mats": {},
                       "verbose": False}
                counts = tcol.apply_industrial(stage, ctx, plan,
                                               holder=holder)
                industrial_placed.append(dict(rec, grade=grade,
                                              counts=counts, holder=holder,
                                              cell=cell))
                print("[ut] d{0} {1:<24s} industrial {2:<7s} "
                      "{3:5.1f}x{4:.1f}x{5:.1f} m  {6:.1f}s  {7}".format(
                          i, name, grade, rec["W"], rec["D"], rec["H"],
                          time.time() - t_b0,
                          {k: v for k, v in (counts or {}).items()
                           if isinstance(v, int)}))
            except Exception as exc:                            # noqa: BLE001
                import traceback
                traceback.print_exc()
                print("[ut] *** FAILED industrial {0} ({1}): {2}".format(
                    name, grade, exc))
                industrial_failed.append({"rec": rec, "error": str(exc)})
            for _ in range(2):
                omni.kit.app.get_app().update()
        self.industrial_records = industrial_records
        self.industrial_placed = industrial_placed
        self.industrial_failed = industrial_failed
        self.failed = self.failed + industrial_failed
        if industrial_records:
            print("[ut] industrial collapse: applied to {0}/{1} shed(s) "
                  "({2} failed) in {3:.0f} s".format(
                      len(industrial_placed), len(industrial_records),
                      len(industrial_failed), time.time() - t_ind0))
            self.vram["industrial"] = vram_mb(
                "{0} industrial collapse(s)".format(len(industrial_placed)))
        return placed + kit_placed + industrial_placed

    # -- 5d) THE STREET PASS (round 3, R9, stream SP) — `_plans/
    # urban_tornado_plan.md` §8: "The corridor's own props stood untouched
    # through two launches (streetlights plumb, signals up, awnings on,
    # trees green, cars parked) — the strongest 'nothing happened here'
    # signal left." `disaster.tornado_street.plan_street` decides (pure,
    # no `pxr`) and `disaster.tornado_street.apply_street` APPLIES it —
    # BOTH extracted 2026-09-01 so a non-launcher caller (stream B's
    # bench) can wire this pass without importing a file that constructs
    # a `SimulationApp` at import time. This method is now THIN: resolve
    # env vars into the callables `plan_street` wants, call `plan_street`
    # then `apply_street`, keep what the launcher itself still needs
    # (`self.street_actions`/`self.street_counts`, VRAM capture) — no
    # application LOGIC lives here any more, see `tornado_street.py`'s
    # own "APPLY" section for that. Runs AFTER every per-building damage
    # pass (so a felled pole's own local wind bearing reads the SAME
    # resolved track those passes did) and BEFORE `ground_evidence` (so
    # the corridor's own props are already down before the ambient
    # debris field is scattered — a felled streetlight should not end up
    # buried under a rubble berm authored on top of it). `UT_STREET=0`
    # skips this method entirely; `UT_LAYOUT_STREET` (module docstring)
    # stays at its own default (0) regardless — this pass REPLACES that
    # generic layout-time mechanism, it does not run alongside it.
    # ---------------------------------------------------------------------
    def street_pass(self):
        self.street_actions = []
        self.street_counts = {}
        if not UT_STREET:
            print("[ut] street pass: SKIPPED (UT_STREET=0)")
            return
        t0 = time.time()

        def wind_at_fn(x, y):
            return tn.wind_at(self.tcfg, x, y)

        # A FRESH resolver rather than threading `self`'s own — `__init__`
        # builds one LOCALLY (for `dump_city_placements` alone) and does
        # not keep it, and this stage is the only other place in the file
        # that wants one, for exactly one thing: a car's own measured
        # length (the mass proxy `car_pose` scales its rates by). Cheap —
        # `SizeResolver` caches per (path, scale, axis) and every car usd
        # here is a repeat of a handful of pool entries.
        resolver = _make_resolver(self.config)

        def car_length_fn(p):
            try:
                fp = resolver.get(str(p.get("usd", "")), "car",
                                  scale=float(p.get("scale", 1.0) or 1.0),
                                  axis_up=p.get("axis_up", "Z"))
                return max(float(fp.get("sx", 4.5)), float(fp.get("sy", 2.0)))
            except Exception:                                   # noqa: BLE001
                return tn.CAR_REF_LEN_M

        # No per-asset yaw-offset lookup is wired here (that needs a
        # loaded `suburb_scene.AssetPools`, which this launcher has no
        # other use for) — a car's own placement `yaw_deg` is used as its
        # long-axis heading directly. Approximate for an asset whose art
        # carries a real yaw-offset; harmless for `car_pose`'s own
        # resting-heading cosmetics, which is the only thing it feeds.
        def car_heading_fn(p):
            return float(p.get("yaw_deg", 0.0))

        # The SAME house-placements list `ground_evidence`'s own footprint
        # rejection reads (`self.dump_doc["placements"]`) — already the
        # `x_m`/`y_m`/`W`/`D`/`yaw_deg` shape `tornado_street._footprint_
        # test` (imported from `tornado_urban_ground`) wants, covering
        # every house regardless of its own damage outcome, which is
        # exactly right for "never land a thrown car inside a building".
        buildings = self.dump_doc["placements"]

        rng = random.Random(self.seed * 1000003 + 900001)
        actions = tst.plan_street(
            self.placements, self.inten, wind_at_fn, rng,
            buildings=buildings, bounds=self.plate,
            throw_m=float(self.tcfg.get("throw_m", 24.0)),
            min_tipped=UT_MIN_TIPPED, car_length_fn=car_length_fn,
            car_heading_fn=car_heading_fn)
        self.street_actions = actions

        # `apply_street` does the application AND prints its own
        # `[tornado_street]`-prefixed per-category table + thrown/forced/
        # correlated counts (the SAME information this method used to
        # print itself under `[ut]`) — nothing left here but the env-
        # driven knobs and the launcher's own VRAM capture.
        self.street_counts = tst.apply_street(
            self.stage, actions, min_tipped=UT_MIN_TIPPED, verbose=True)

        dt = time.time() - t0
        print("[ut] street pass: {0:.1f}s".format(dt))
        self.vram["street"] = vram_mb(
            "street pass ({0} action(s))".format(len(actions)))

    # -- 5c) GROUND EVIDENCE (round 2, R4) — the corridor's own debris
    # field + surface stain, `disaster.tornado_urban_ground`. Runs AFTER
    # every per-building pass (so the footprint rejection sees every house
    # placement whether it ended up damaged, kit-damaged, or left intact —
    # the field does not care WHICH, only "is there a building here") and
    # BEFORE captures (so the corridor reads from altitude in every
    # snapshot). `UT_GROUND=0` skips both layers.
    # ---------------------------------------------------------------------
    def ground_evidence(self):
        self.ground_fragments = []
        self.ground_debris_meshes = []
        self.ground_stain_paths = []
        if not UT_GROUND:
            print("[ut] ground evidence: SKIPPED (UT_GROUND=0)")
            return
        stage = self.stage
        # The SAME house placements `select_damage`'s own gate loop reads
        # (`self.dump_doc["placements"]`, `dump_city_placements`'s own
        # `x_m`/`y_m`/`yaw_deg`/`W`/`D`/`H` keys) — `tornado_urban_ground.
        # _footprint_test` accepts either that fallback shape or the short
        # `x`/`y`/`yaw` one, so no reshaping is needed. Every house is
        # passed regardless of its damage OUTCOME this round (T0, refused,
        # gac/dtc-damaged, kit-damaged, or left intact) — the footprint
        # test only cares that a building's own OBB is still there to keep
        # ground debris out of.
        house_placements = self.dump_doc["placements"]
        t0 = time.time()

        # -- 5c-i) R4a: the scattered debris field --------------------------
        scatter_kwargs = {}
        if UT_GROUND_PER100 is not None:
            scatter_kwargs["per_100m2"] = UT_GROUND_PER100
        frags = tug.scatter_corridor(
            self.plate, self.inten, self.tcfg,
            random.Random(self.seed * 31 + 7), house_placements,
            **scatter_kwargs)
        self.ground_fragments = frags

        GROUND_ROOT = "/World/tornado_ground"
        ground_ctx = {"parent": GROUND_ROOT, "mats": {}, "verbose": True}
        self.ground_debris_meshes = tug.build(
            stage, GROUND_ROOT, frags, ground_ctx, ground_z=0.0)

        # -- 5c-ii) R4b: the surface stain -----------------------------------
        # `rng` HERE MUST BE A NUMPY GENERATOR (`stain_overlay`'s own
        # docstring — `scour_coverage` -> `_island_field` calls `rng.
        # normal(...)`, which `random.Random` does not have). A DIFFERENT
        # seed offset than `self.inten`'s own `np.random.default_rng(
        # self.seed + 23)` (built in `__init__`, not kept as an attribute) —
        # deliberately not `self.seed + 23` again: that would replay the
        # EXACT SAME draw sequence intensity_field's own edge noise already
        # consumed, correlating the stain's island pattern with the track's
        # own edge wobble instead of drawing an independent one.
        self.ground_stain_paths = tug.stain_overlay(
            stage, GROUND_ROOT, self.plate, self.tcfg,
            np.random.default_rng(self.seed + 97), self.inten, ssf=self.ssf)

        by_cls = {}
        for f in frags:
            by_cls[f["class"]] = by_cls.get(f["class"], 0) + 1
        print("[ut] ground evidence: {0} fragment(s) ({1}) -> {2} mesh(es), "
              "stain overlay {3} band(s), {4:.1f}s".format(
                  len(frags),
                  ", ".join("{0}={1}".format(k, by_cls[k])
                           for k in sorted(by_cls)),
                  len(self.ground_debris_meshes),
                  len(self.ground_stain_paths), time.time() - t0))
        self.vram["ground"] = vram_mb(
            "ground evidence ({0} fragment(s))".format(len(frags)))

    # -- 6) captures -----------------------------------------------------
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

            w, h = self.region
            span = max(w, h)
            snaps.overview(self.stage, (0.0, 0.0), span * 1.05,
                           os.path.join(SNAP_DIR, "city_top.png"), self.ssf)

            # THREE POINTS ALONG THE CENTRELINE plus one per flank at
            # +-0.6 x half-width — `_plans/urban_tornado_W_notes.md`'s own
            # rehearsal quotes the exact numbers this preset compiles to.
            from_track = tn.from_track(self.tcfg)
            half_w = float(self.tcfg["width_m"]) / 2.0
            heading = float(self.tcfg["heading_deg"])
            # camera sits UPWIND (opposite the direction of travel) and
            # looks DOWN-TRACK — the fire launcher's own "the wave" camera
            # convention, `urban_fire_city_launch_script.capture`'s
            # `heading_deg` -> upwind-eye derivation, reused here.
            obl_azimuth = (heading + 180.0) % 360.0

            points = {}
            for a in (-150.0, 0.0, 150.0):
                x, y = from_track(a, 0.0)
                points["track_{0}".format(int(round(a)))] = (x, y)
            for side, cross in (("left", 0.6 * half_w),
                               ("right", -0.6 * half_w)):
                x, y = from_track(0.0, cross)
                points["track_{0}".format(side)] = (x, y)

            # ROUND 2 CAMERA FIX (`_plans/urban_tornado_plan.md` §7, the
            # round-1 photo lesson): round 1's oblique (obl_dist 60 m,
            # obl_h 60 m, aim_h 12 m) sat INSIDE the tower canyon — a
            # camera 60 m up and 60 m back from a point between mid/high-
            # rise buildings looks straight into the FACADE of the nearest
            # one rather than clearing the roofline to see the corridor.
            # Widened well past any one block's own height/setback so the
            # camera clears the roofline before it looks back down-track.
            snaps.views_around(self.stage, points, SNAP_DIR, self.ssf,
                              top_h=90.0, obl_dist=150.0, obl_h=120.0,
                              azimuth_deg=obl_azimuth, aim_h=15.0)
            print("[ut] snapshots ({0} point(s)) -> {1}".format(
                len(points), SNAP_DIR))

            # CITY OVERVIEW OBLIQUE — one wide establishing shot, well
            # outside and above the whole plate (dist ~420 m, h ~300 m —
            # comfortably past the 500 m corridor's own tallest low/mid-
            # rise stock, `TORNADO_MAX_H_M`'s tower/highrise cap sits at
            # 232 m so 300 m of eye height clears it too), aimed at the
            # plate centre along the SAME upwind azimuth the track
            # obliques use so the whole corridor lines up in one frame.
            # `views_around` has no single-shot oblique entry point (it
            # always pairs a top-down with an oblique per NAMED point), so
            # this is authored directly with `place_camera`/`snapshot`.
            cx0, cy0 = 0.0, 0.0
            a = math.radians(obl_azimuth)
            ox = cx0 + 420.0 * math.cos(a)
            oy = cy0 + 420.0 * math.sin(a)
            snaps.place_camera(
                self.stage,
                (ox * self.ssf, oy * self.ssf, 300.0 * self.ssf),
                (cx0 * self.ssf, cy0 * self.ssf, 0.0))
            snaps.snapshot(os.path.join(SNAP_DIR, "city_obl.png"))
            print("[ut] city overview oblique -> {0}".format(
                os.path.join(SNAP_DIR, "city_obl.png")))
        except Exception as exc:                                  # noqa: BLE001
            import traceback
            traceback.print_exc()
            print("[ut] snapshots FAILED: {0}".format(exc))

    # -- 7) report + banner -------------------------------------------------
    def _refusal_bucket(self, reason):
        r = str(reason or "")
        if "blacklisted" in r:
            return "refused-blacklist"
        if "tornado-height cap" in r:
            return "refused-cap"
        return "refused-other"

    def report(self):
        by_level = {}
        for r in self.records:
            by_level.setdefault(r["level"], 0)
            by_level[r["level"]] += 1
        applied_by_level = {}
        for r in self.placed:
            applied_by_level.setdefault(r["level"], 0)
            applied_by_level[r["level"]] += 1

        refusal_buckets = {}
        for r in self.refused:
            b = self._refusal_bucket(r.get("reason"))
            refusal_buckets.setdefault(b, 0)
            refusal_buckets[b] += 1

        lines = []
        lines.append("# Urban tornado LIVE city — {0}, seed {1}".format(
            SCENE_CONFIG, self.seed))
        lines.append("")
        lines.append("Track: width {0:.1f} m, peak {1:.3f}, origin {2}, "
                     "heading {3:.1f} deg, curvature {4:.1f} deg/km".format(
                         self.tcfg["width_m"], self.tcfg["peak"],
                         self.tcfg["origin_m"], self.tcfg["heading_deg"],
                         self.tcfg.get("curvature_deg_per_km", 0.0)))
        lines.append("")
        lines.append("| level | T1+ records (all kinds) | applied "
                     "(gac/dtc) |")
        lines.append("|---|---|---|")
        for lvl in ("T1", "T2", "T3", "T4"):
            lines.append("| {0} | {1} | {2} |".format(
                lvl, by_level.get(lvl, 0), applied_by_level.get(lvl, 0)))
        lines.append("")
        lines.append("damaged-applied: {0}/{1} gac/dtc record(s) "
                     "({2} failed, {3} dropped by UT_MAX_BUILDINGS)".format(
                         len(self.placed), len(self.damaged_all),
                         len(self.failed) - len(self.kit_failed),
                         len(self.dropped_by_budget)))
        # ROUND 2 (R3): kit records moved from the "intact" bucket below
        # into their OWN "damaged" bucket here — `tornado_kit.wreck_kit`
        # now applies to every T1+ kit record (see `apply_damage`'s kit
        # pass), so "kit-no-path" (the round-1 reading: no damage path
        # existed at all) no longer describes what happens to them.
        lines.append("kit-damaged (bld_<style>_DG0, tornado_kit.wreck_kit): "
                     "{0}/{1} record(s) applied ({2} failed)".format(
                         len(self.kit_placed), len(self.kit_records),
                         len(self.kit_failed)))
        lines.append("industrial-collapsed (tornado_collapse, R11): "
                     "{0}/{1} shed(s) applied ({2} failed)".format(
                         len(getattr(self, "industrial_placed", []) or []),
                         len(getattr(self, "industrial_records", []) or []),
                         len(getattr(self, "industrial_failed", []) or [])))
        lines.append("")
        lines.append("intact-in-corridor (T1+ but left untouched this "
                     "round):")
        lines.append("  slice-no-path: {0}".format(len(self.slice_intact)))
        lines.append("  budget-dropped (gac/dtc, UT_MAX_BUILDINGS): {0}"
                     .format(len(self.dropped_by_budget)))
        lines.append("")
        lines.append("refused (gate 1-4 in tornado_city.damageable):")
        for b, n in sorted(refusal_buckets.items()):
            lines.append("  {0}: {1}".format(b, n))
        lines.append("")
        lines.append("T0 (below the corridor / jittered below cut): {0}"
                     .format(self.n_t0))
        lines.append("")
        # ROUND 2 (R4) — the corridor's ground evidence
        # (`disaster.tornado_urban_ground`, `TornadoCityApp.
        # ground_evidence`): a scattered masonry-city debris field
        # (fragment counts by STOCK class) plus a translucent dust/scour
        # surface stain, both rejected out of every house's own footprint.
        lines.append("ground evidence (UT_GROUND={0}):".format(
            int(UT_GROUND)))
        if UT_GROUND:
            by_cls = {}
            for f in self.ground_fragments:
                by_cls[f["class"]] = by_cls.get(f["class"], 0) + 1
            lines.append("  fragments: {0} total across {1} merged mesh(es)"
                         .format(len(self.ground_fragments),
                                 len(self.ground_debris_meshes)))
            for cls in sorted(by_cls):
                lines.append("    {0}: {1}".format(cls, by_cls[cls]))
            lines.append("  stain overlay: {0} ({1} band(s))".format(
                "yes" if self.ground_stain_paths else "no",
                len(self.ground_stain_paths)))
        else:
            lines.append("  skipped (UT_GROUND=0)")
        lines.append("")
        lines.append("| i | name | kind | level | hc | pieces | s | "
                     "removed | glass | debris |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for r in self.placed + self.kit_placed:
            c = r.get("counts") or {}
            lines.append("| {0} | {1} | {2} | {3} | {4} | {5} | {6:.2f} | "
                         "{7} | {8} | {9} |".format(
                             r["i"], r["name"], r["kind"], r["level"],
                             r["height_class"], r.get("n_pieces", 0),
                             r.get("slice_s", 0.0),
                             c.get("n_removed", 0), c.get("n_glass", 0),
                             c.get("n_fragments", 0)))
        text = "\n".join(lines) + "\n"
        out_path = os.path.join(
            _SCENE_GEN_DIR, "_plans",
            "tornado_city_live_{0}_report.md".format(self.seed))
        with open(out_path, "w") as fh:
            fh.write(text)
        print("[ut] report -> {0}".format(out_path))
        self.report_path = out_path
        return text

    def banner(self):
        self.vram["end"] = vram_mb("after captures")
        self.report()
        print("\n" + "=" * 78)
        # ROUND 2 (lead): the previous revision "preserved" this line
        # byte-for-byte while `self.kit_intact`'s meaning changed to "kit
        # FAILURES" — a banner that printed failures under an "intact"
        # label. The label follows the data now: kit buildings are DAMAGED
        # live this round, and a kit failure is called a failure.
        print("URBAN TORNADO CITY   {0}   {1:.0f} x {2:.0f} m   {3} "
              "building(s), {4} damaged (gac/dtc) + {5} damaged (kit), "
              "{6} kit-FAILED, {7} slice-intact, {8} refused, {9} T0".format(
                  SCENE_CONFIG, self.region[0], self.region[-1],
                  self.n_houses, len(self.placed),
                  len(getattr(self, "kit_placed", []) or []),
                  len(self.kit_intact), len(self.slice_intact),
                  len(self.refused), self.n_t0))
        print("  track: width {0:.1f} m peak {1:.3f} origin {2} heading "
              "{3:.1f} deg curvature {4:.1f} deg/km".format(
                  self.tcfg["width_m"], self.tcfg["peak"],
                  self.tcfg["origin_m"], self.tcfg["heading_deg"],
                  self.tcfg.get("curvature_deg_per_km", 0.0)))
        print("  dump: {0}".format(self.dump_path))
        print("  report: {0}".format(getattr(self, "report_path", "?")))
        if self.failed:
            print("  {0} building(s) FAILED to apply: {1}".format(
                len(self.failed),
                ", ".join("{0}({1})".format(f["rec"]["name"],
                                            f["rec"]["kind"])
                          for f in self.failed[:8])))
        print("  intact prim(s) hidden: {0} (UT_HIDE={1})".format(
            self.n_hidden, HIDE_MODE))
        print("  built in {0:.0f} s".format(time.time() - self.t0))
        print("=" * 78 + "\n")
        print("URBAN TORNADO CITY DONE")

    def run(self):
        self.select_damage()
        self.apply_damage()
        self.street_pass()
        self.ground_evidence()
        self.capture()
        self.banner()


def main():
    TornadoCityApp().run()


if __name__ == "__main__":
    try:
        main()
    except Exception as _exc:                                     # noqa: BLE001
        import traceback
        traceback.print_exc()
        print("URBAN TORNADO CITY FAILED: {0}".format(_exc))
    if KEEP_OPEN or not _HEADLESS:
        _app = omni.kit.app.get_app()
        omni.timeline.get_timeline_interface().play()
        while simulation_app.is_running():
            _app.update()
        omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
