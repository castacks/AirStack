#!/usr/bin/env python
"""
fire_people_bench — 3 UNIQUE damaged buildings, empty plate, real people.

    ISAAC_SIM_HEADLESS=false PB_FLOW=1 KEEP_OPEN=1 \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_people_bench \
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
    /isaac-sim/python.sh \
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/fire_people_bench_launch_script.py \
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

WHY THIS EXISTS
---------------
The user, on the live 39-building `city_138` scene: *"The people placement is
wrong. They are floating, etc."* and *"Create an empty scene with 3
buildings. 1 roof collapsed, 1 with only external fire showing, 1 with
partial collapse. Place humans in their positions for all 3 buildings. We
can work from them there before scaling up. Make sure the 3 buildings are
unique."* This is that scene — no city generator, no dump dependency beyond
the tiny 3-record dump/manifest pair this bench ships its own copy of
(`scene_gen/_plans/people_bench/`), so it brings up in well under a minute
and a reviewer can walk each building's people in isolation before the fix
is trusted at 39-building scale.

Modelled on `fire_assembly_launch_script.py` (the row bench) with two
departures, both because this bench's buildings carry a REAL, non-zero yaw
(270/90/0 deg) inherited from where they actually stood in `city_138`,
while the row bench places every column at yaw 0:

  1. Buildings are referenced at the mini-dump's own recorded `(x, y,
     yaw_deg)` (`place_holder`, `urban_fire_city_launch_script`'s idiom:
     translate then `AddRotateXYZOp`), not at a computed column position —
     so the geometry lands exactly where `fire_people` already solved the
     people against.
  2. Fire is re-placed with `fire_bake.place` (the ROTATING transform,
     `urban_fire_city_launch_script`'s own choice for exactly this reason),
     not `fire_bake.translate` (translate-only, correct only at yaw 0).

THE THREE PICKS (justified from their own bake sidecars — see
`scene_gen/_plans/people_bench/PICKS.md`... no such file; the justification
lives here and in the handoff message, not a second document):

  - **roof_collapsed** — `gac_SM_Building_26_F5_o3_NW_s684` (GAC 18-storey
    tower, F5). Sidecar notes: *"roof: burnt through over ~40% of the
    deck ... fire collapse: top 1 storey(s) down from storey 18 ... heap on
    the storey below at z=65.4"* — a real, measured roof loss, not an
    assumption; `fire.deck_z` is a real MEASURED value (69.2 m), not the
    estimate.
  - **intact_shell** — `kit_apartment_tall_F3_o4_SNW_s758` (kit midrise
    apartment archetype, F3). Sidecar notes: *"roof: fire never reached it,
    roof intact"* — the cleanest possible "external fire only" pick on this
    bake set; the two F3 kit/GAC alternatives the brief named both carry
    SOME roof burn-through even at F3 (`kit_commercial_mid`: "burnt through
    over ~20% of the deck"), which this one does not.
  - **partial_collapse** — `kit_office_F5c_o3_SW_s820` (kit lowrise office
    archetype, F5c). Sidecar notes: *"partial collapse (corner, corner SW):
    S/W lost from storey 3 up (2 storey(s), 17.3 m of each wall at the
    top)"* — an actual corner loss with a torn edge, not a full collapse.

Three different `city.typology` (tower / midrise / lowrise), three
different `kind` (gac sliced whole-asset / kit archetype x2 but different
archetypes with different construction — `office`'s "0 column(s) charred, 30
column(s) steel" vs `apartment_tall`'s "0 column(s) steel" — steel-frame
office vs a masonry/concrete-frame residential block), so "make the 3
buildings unique" is satisfied on construction type as well as damage state.

THE MINI MANIFEST/DUMP/PEOPLE TRIO
-----------------------------------
`scene_gen/_plans/people_bench/{bench_dump,bench_manifest,bench_people}
.json` are NOT hand-authored positions — they are the REAL `fire_city_500m
_39.json`/`fc_dump_500.json` records for `i=273/257/259` (the three picks'
own manifest indices), copied verbatim (same W/D/H/yaw/level/sides/seed/
n_storeys — everything the fire was actually solved and baked against) and
repositioned to `(-90, 0)`, `(0, 0)`, `(90, 0)` — 90 m apart, matching the
brief's "~80-100 m apart" — with block rects added to `bench_dump.json`'s
`typology.blocks` so sidewalks form around each one.
`scene_gen/disaster/fire_people.plan_people` was then run for REAL against
these + the real `city_138` sidecars (`tools/fire_people_dry_run.py
--dump .../bench_dump.json --manifest .../bench_manifest.json --sidecar-dir
.../city_138`) — every class the city uses came out non-degraded: evacuee,
onlooker, at_car, window, roof, roof_victim, casualty_apron, roof_debris,
all PASS on the 19-rule gate.

WHAT FED THIS BENCH FORWARD, AND WHAT IS THEREFORE ALREADY FIXED IN IT
------------------------------------------------------------------------
Two real bugs in `disaster/fire_people.py` were found and fixed building
THIS bench (`tools/people_float_audit.py`, host-side, no Isaac):

  1. `deck_z()`'s "estimated" fallback (a kit bake never measures a real
     roof deck) used to trust the MANIFEST's `H` even when a sidecar was
     right there — and a manifest record whose cell was re-skinned to a
     different archetype after the fire was solved (`i=261/264/269` in the
     real city) carries the OLD building's `H`, up to 4.1 m taller than
     what actually got baked. Fixed to prefer the sidecar's own measured
     `top_z` whenever a sidecar exists at all.
  2. `load_sidecars()` keyed a re-baked cell's sidecar by the BARE cell/tag,
     and 10 cells in `city_138` (including `i=257`, this bench's own
     `intact_shell` pick) carry TWO sidecars — alternate severities from
     iteration — sharing one cell/tag, so a bare-key lookup silently
     returned whichever file `os.listdir` sorted last, regardless of which
     level the manifest record actually was. Fixed with a `(cell, level)`
     tuple key tried first.
  Both are pinned by new tests in `scene_gen/tests/test_fire_people.py`
  (`test_58`/`test_59`) and verified end to end with
  `tools/fire_people_rerun.sh` against the real 39-manifest (19/19 PASS,
  0 flags from `people_float_audit.py`). The 39-building `fire_people_final
  .json` this bench's sibling scene uses was RE-SOLVED with both fixes —
  it already reflects them; this bench exercises the same fixed code path
  from a clean, tiny manifest instead of re-deriving anything.

Env:
    PB_BENCH_DIR   dir with bench_dump.json / bench_manifest.json /
                   bench_people.json (default
                   scene_gen/_plans/people_bench)
    PB_BAKES_DIR   dir the 3 bake stems below are resolved against
                   (default /isaac-sim/.cache/fire_bakes/city_138)
    PB_FLOW        1 (default) authors Flow + re-places the fire
    PB_CELL_M      Flow density cell size, metres (default 0.15)
    PB_MAX_BLOCKS  Flow block pool (default 8192 — 3 buildings, generous)
    PB_EMITTERS    flame openings per building (default 9)
    PB_SMOKE       1 (default) side/interior/roof smoke on top of flame
    PB_SMOKE_SCALE emission density multiplier (default 1.0)
    PB_SEED        rng seed for the per-emitter jitter (default 7)
    PB_SKY         lighting preset applied ON TOP of `build_ground_and_
                   light`'s own key light: "mid_day" (default -- new,
                   see sky_presets.py), "sunset" (the old/current bench
                   look, reproduced exactly, for an A/B), or an explicit
                   HDRI/.hdr/.exr/stage-USD path or URL (mid_day's sun/dome
                   numbers, dome textured with that instead)
    SNAP_DIR       viewport captures, MUST be under
                   /isaac-sim/.nvidia-omniverse/logs/
    PB_CLOSEUPS    1 (default) shoots one close-in portrait per person
                   record, named by class, index and POSE
                   (`<cls>_<NN>_<pose>_closeup_id<id>.png`, e.g.
                   `roof_victim_02_wave_help_closeup_id17.png`) under
                   `SNAP_DIR/people_closeups/` — the pre-review pass so the
                   lead can judge poses from captures before the user looks
    PB_CLOSEUP_DIST_M   metres from the figure the closeup camera stands
                   (default 5.0, i.e. inside the requested 4-6 m band)
    PB_CLOSEUP_LIMIT    cap on closeups per class, 0 = every record
                   (default 0 — the bench trio's ~20-30 people is cheap to
                   shoot in full)
    KEEP_OPEN      1 keeps the app up after the captures

Banner: `PEOPLE BENCH DONE`.
"""

import json
import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import omni.kit.app                                             # noqa: E402
import omni.timeline                                            # noqa: E402
import omni.usd                                                 # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom                           # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import scene_generator as sg                                    # noqa: E402
from scene_prep import get_stage_meters_per_unit                # noqa: E402
from sky_presets import apply_sky_preset                        # noqa: E402
from disaster import fire as fx                                 # noqa: E402
from disaster import fire_assembly_lib as fal                   # noqa: E402
from disaster import fire_bake as fb                            # noqa: E402
from disaster import fire_people as fpl                         # noqa: E402

WORLD = "/World"
BENCH = "/World/bench"

# ---------------------------------------------------------------------------
# CURATED METADATA for the original three picks — `i` is the real
# fire_city_500m_39.json / fc_dump_500.json index the bake was solved
# against, and this table's job is ONLY to supply a human-readable `tag`/
# `why` for the buildings someone hand-picked and wrote a sentence about.
#
# BUG, 2026-09-01, v6 RENDER: this table used to be BOTH the metadata AND
# the LIST OF BUILDINGS TO COMPOSE — the main loop iterated it directly,
# never the manifest. A fourth building appended to `bench_manifest.json`
# (the window-showcase pick) had no entry here, so it was never referenced,
# never composed, and the people solve — which reads the manifest, not
# this table — placed six `lean_window` figures and a roof group at its
# cell anyway: six people floating in empty sky, logged as "3 bake(s)
# composed" with nobody printing that a fourth manifest record existed and
# was silently skipped. THE LOOP NOW ITERATES `manifest["records"]`
# (`_bench_rows`, below) — every record gets composed, this table supplies
# `tag`/`why` when it has an entry for that `i` and a derived fallback
# otherwise (`_bake_stem_for`/`_derive_tag_why`). This dict is no longer
# capable of silently omitting a building: worst case a manifest record's
# `tag`/`why` are generic instead of hand-written, never absent.
# ---------------------------------------------------------------------------
BENCH_BUILDING_INFO = {
    273: {"stem": "gac_SM_Building_26_F5_o3_NW_s684",
         "tag": "roof_collapsed",
         "why": "roof burnt through ~40%, top storey collapsed onto the one "
                "below (deck_z MEASURED, not estimated)"},
    257: {"stem": "kit_apartment_tall_F3_o4_SNW_s758",
         "tag": "intact_shell",
         "why": "sidecar: 'roof: fire never reached it, roof intact' — "
                "external fire only"},
    259: {"stem": "kit_office_F5c_o3_SW_s820",
         "tag": "partial_collapse",
         "why": "sidecar: partial collapse, SW corner, 2 storeys of wall lost"},
}


def _stems_by_seed_level(bakes_dir):
    """`{(seed, level): bake stem}` for every `.json` sidecar under
    `bakes_dir` — lets the launcher find ANY manifest record's own bake
    without it being named in `BENCH_BUILDING_INFO`.

    NOT BY `cell` — MEASURED, 2026-09-01, while chasing the v6 "floating
    building" bug's fix: a bake sidecar's own `"cell"` field is its
    BAKE-INTERNAL prim path from the single-building bake stage (e.g.
    `/World/bake/g23`), which has nothing to do with the CITY manifest's
    `cell` (`/World/stage/generated/house_20_274`) — the two are different
    coordinate spaces that happen to share a JSON key name. A `cell`-keyed
    index therefore NEVER matches a real manifest record and silently
    resolves nothing (confirmed: it returned `stem=None` for the very
    building this fix exists for). `seed` is what the bake filename itself
    encodes (`..._s715.json`) AND what the sidecar's own top-level `seed`
    field carries AND what the manifest record's own `seed` field carries
    — the one join key present in all three places. Paired with `level`
    for the same reason `fire_people.load_sidecars` pairs its own tuple
    key with level: two bakes of the same building at different severities
    can share a seed.
    """
    out = {}
    if not bakes_dir or not os.path.isdir(bakes_dir):
        return out
    for name in os.listdir(bakes_dir):
        if not name.endswith(".json"):
            continue
        try:
            with open(os.path.join(bakes_dir, name)) as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            continue
        seed, level = doc.get("seed"), doc.get("level")
        if seed is None or level is None:
            continue
        key = (int(seed), str(level))
        if key not in out:
            out[key] = name[:-len(".json")]
    return out


def missing_composed_buildings(composed_is, people_recs):
    """Sorted list of building `i` values the PEOPLE SOLVE references that
    never actually composed — the check that makes "people floating in
    empty sky" (2026-09-01 v6 render) impossible to ship again silently.

    Pure set arithmetic, no `pxr`/`omni` — deliberately dependency-free so
    it can be unit-tested host-side (`tests/test_bench_building_check.py`)
    without an Isaac environment, and so THIS launcher can call it with
    nothing more than the two plain lists it already has in hand
    (`composed_is` from the `rows` this script's own composition loop
    built; `people_recs` the JSON `plan_people` wrote). A record with no
    `building_i` at all (a class that carries no building reference) is
    not a miss — only a NAMED building that never composed is.
    """
    composed = set(composed_is)
    referenced = {r["building_i"] for r in people_recs
                 if r.get("building_i") is not None}
    return sorted(referenced - composed)


def _bench_rows(manifest, bakes_dir):
    """One row per MANIFEST RECORD (not per `BENCH_BUILDING_INFO` entry) —
    `(i, stem, tag, why)`, `stem` resolved by `i` against the curated table
    first, then by the record's own `(seed, level)` against every sidecar
    actually on disk (`_stems_by_seed_level`). A record neither table can
    resolve gets `stem=None` and is reported, not skipped — the caller
    decides what an unresolvable bake means."""
    by_seed_level = _stems_by_seed_level(bakes_dir)
    rows = []
    for rec in manifest.get("records") or []:
        i = rec.get("i")
        info = BENCH_BUILDING_INFO.get(i)
        stem = info["stem"] if info else (
            by_seed_level.get((int(rec["seed"]), str(rec.get("level"))))
            if rec.get("seed") is not None else None)
        if info:
            tag, why = info["tag"], info["why"]
        else:
            # DERIVED FALLBACK — the record's own `_bench_tag` (set by
            # whoever appended it, e.g. "window_showcase") if it carries
            # one, else a generic but still-unique label. Never blank:
            # an unlabelled row in the console log is what let a missing
            # building go unnoticed in the first place.
            tag = str(rec.get("_bench_tag") or "building_i{0}".format(i))
            why = ("no curated description — level {0}, H {1:.0f} m, "
                   "added to the manifest without a BENCH_BUILDING_INFO "
                   "entry".format(rec.get("level"), float(rec.get("H", 0.0))))
        rows.append({"i": i, "stem": stem, "tag": tag, "why": why})
    return rows

BENCH_DIR = _env("PB_BENCH_DIR",
                 os.path.join(_SCENE_GEN_DIR, "_plans", "people_bench"))
BAKES_DIR = _env("PB_BAKES_DIR", "/isaac-sim/.cache/fire_bakes/city_138")
DUMP_PATH = os.path.join(BENCH_DIR, "bench_dump.json")
MANIFEST_PATH = os.path.join(BENCH_DIR, "bench_manifest.json")
PEOPLE_PATH = os.path.join(BENCH_DIR, "bench_people.json")

FLOW = _env("PB_FLOW", "1") not in ("0", "false", "no")
CELL_M = float(_env("PB_CELL_M", "0.15"))
MAX_BLOCKS = int(_env("PB_MAX_BLOCKS", "8192"))
MAX_EMITTERS = int(_env("PB_EMITTERS", "9"))
SMOKE = _env("PB_SMOKE", "1") not in ("0", "false", "no")
SMOKE_SCALE = float(_env("PB_SMOKE_SCALE", "1.0"))
SEED = int(_env("PB_SEED", "7"))
SKY = _env("PB_SKY", "mid_day")
SNAP_DIR = _env("SNAP_DIR", "")
PEOPLE_CLOSEUPS = _env("PB_CLOSEUPS", "1") not in ("0", "false", "no")
CLOSEUP_DIST_M = float(_env("PB_CLOSEUP_DIST_M", "5.0"))
# 0 = every record. The bench trio runs ~20-30 people total, small enough to
# shoot every one; the cap exists for a caller pointed at a bigger manifest.
CLOSEUP_LIMIT = int(_env("PB_CLOSEUP_LIMIT", "0"))

vram_mb = fal.vram_mb
build_ground_and_light = fal.build_ground_and_light


def place_holder(stage, stem, x, y, z, yaw_deg, ssf):
    """`urban_fire_city_launch_script.place_holder`, reproduced: a fresh
    Xform carrying translate + yaw-only rotate + scale, with the reference
    on a CHILD prim so it can never compose with the holder's own ops."""
    holder = "{0}/{1}".format(BENCH, stem)
    xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x) * ssf, float(y) * ssf,
                                     float(z) * ssf))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(float(ssf), float(ssf), float(ssf)))
    kid = stage.DefinePrim(Sdf.Path(holder + "/bake"))
    return holder, kid


def load_json(path):
    with open(path) as fh:
        return json.load(fh)


# CHEST HEIGHT ABOVE `z`, BY SHAPE OF FIGURE. `z` is `fire_people`'s own
# support-surface convention ("z IS THE SUPPORT SURFACE ON EVERY CLASS,
# without exception" — the debris top for a burial figure, the sill for a
# window leaner, the deck for a roof figure, the ground for a stander): the
# right aim height above it is completely different for a standing figure
# (~1.1-1.3 m to the sternum) and a prone one (~0.2-0.35 m — a lying body's
# own crest, `scene_generator._BODY_RISE`'s own numbers, doubled for slop).
#
# KEYED OFF THE RECORD'S OWN `prone` FLAG, NOT A CLASS WHITELIST.
# `casualty_apron` and `roof_debris` are prone on every record, which is what
# a `_PRONE_CLASSES` tuple used to test — but `interior_trapped` (added
# 2026-08-31, section 5d) is BOTH: ~half its records are the conscious
# variant standing/leaning at the opening (`prone: False`) and the rest are
# passed out on the slab (`prone: True`), decided per-record by
# `interior_trapped_conscious_share`. A class-keyed lookup silently aimed
# every passed-out figure's closeup at 1.15 m over an empty slab — well over
# a body lying on it — while every standing one was fine. `prone` is on
# every record fire_people writes (`_pass_*`, without exception), so testing
# it directly is both more correct and shorter than maintaining a second
# list in sync with fire_people.py's own class table.
_STAND_AIM_M = 1.15
_PRONE_AIM_M = 0.30

# WINDOW CLOSEUP FRAMING, 2026-09-01 bench-v4 feedback: street level, ~12 m
# out, wide enough (35 mm vs the default 50 mm) to keep the facade/sill in
# frame alongside the figure so recession/occlusion is actually auditable.
_WINDOW_CLOSEUP_DIST_M = 12.0
_WINDOW_CLOSEUP_EYE_M = 1.6            # a pedestrian's eye height at grade
_WINDOW_CLOSEUP_FOCAL_MM = 35.0

# BURIAL CLOSEUP FRAMING, 2026-09-01 bench-v5 feedback: the level ground-
# height shot put the camera eye inside the covering debris roughly half
# the time (a burial record's `yaw_deg` is a random lie-down direction, not
# a facing side to stand clear of). Climbing to 35 deg elevation at 7 m —
# the middle of the coordinator's own "30-45 deg ... 6-8 m" — clears
# anything resting at the figure's own height on any bearing.
_BURIAL_CLOSEUP_ELEV_DEG = 35.0
_BURIAL_CLOSEUP_DIST_M = 7.0


def _people_closeups(stage, snaps, recs, out_dir, ssf, dist_m=5.0,
                     limit_per_class=0, frames=40, focal_mm=50.0):
    """One close-in portrait per person record — the pre-review pass so the
    lead can judge a pose from a capture before the user looks at the bench.

    Reuses `snapshots.place_camera`/`snapshot` directly (the same primitives
    `views_around` is built from) rather than a new camera mechanism: eye and
    target are both `dist_m` from the figure, at aim height, so the shot is a
    level portrait rather than a look-down.

    STANDS WHERE THE FIGURE ITSELF IS LOOKING. `yaw_deg` on every class in
    this planner is the FACING bearing the figure ends up looking along (see
    `fire_people.py`'s own account of `HUMAN_YAW_OFFSET_DEG`) — a window
    leaner looks out over the street, a roof figure looks out over the
    parapet — so a camera placed `dist_m` along that same bearing stands
    where the street/parapet side is and looks back at the figure's face,
    rather than at the back of its head or into the wall behind it.
    """
    counts = {}
    n = 0
    for r in recs:
        cls = str(r.get("cls", "unknown"))
        if limit_per_class and counts.get(cls, 0) >= limit_per_class:
            continue
        counts[cls] = counts.get(cls, 0) + 1
        try:
            x, y, z = float(r["x"]), float(r["y"]), float(r["z"])
        except (KeyError, TypeError, ValueError):
            continue
        yaw = math.radians(float(r.get("yaw_deg", 0.0)))
        aim = _PRONE_AIM_M if r.get("prone") else _STAND_AIM_M
        tz = z + aim
        if cls == "window":
            # STREET-LEVEL FRAMING, 2026-09-01 bench-v4 feedback: "the per-
            # figure closeup camera frames the figure so tightly that
            # recession/occlusion is invisible ... change window closeups
            # to: camera at street level ~12 m out from the facade, aimed
            # ~20 deg up at the figure, so the sill/spandrel occlusion is
            # visible in context." A tight portrait (the default below)
            # crops out the wall the figure is set back behind, which is
            # exactly the context an occlusion/recession audit needs — the
            # WALL has to be in frame, not just the figure. Camera sits at
            # a real pedestrian eye height at the building's own base
            # (`z=0` locally — the bench plate's own ground datum) rather
            # than at the figure's own elevation, `_WINDOW_CLOSEUP_DIST_M`
            # out along the SAME facing bearing the figure looks along
            # (over the street, per this function's own account), aimed AT
            # the figure — for a window a few storeys up this naturally
            # lands well over 20 deg above the horizontal; for a low one it
            # is close to the requested 20.
            ex = x + _WINDOW_CLOSEUP_DIST_M * math.cos(yaw)
            ey = y + _WINDOW_CLOSEUP_DIST_M * math.sin(yaw)
            ez = _WINDOW_CLOSEUP_EYE_M
            snaps.place_camera(stage, (ex * ssf, ey * ssf, ez * ssf),
                               (x * ssf, y * ssf, tz * ssf),
                               focal_mm=_WINDOW_CLOSEUP_FOCAL_MM)
        elif cls in ("casualty_apron", "roof_debris"):
            # OBLIQUE OVERHEAD FRAMING, 2026-09-01 bench-v5 feedback: "it now
            # sits INSIDE the heap (casualty_apron_03's capture is a wall of
            # rubble texture)." A level, ground-height shot (the default
            # branch below) plants the camera eye at the figure's own near-
            # ground z ALONG A RANDOM BEARING (`yaw_deg` on a burial record
            # is the body's random lie-down yaw, not a facing direction with
            # any "outward" side — `_burial_record`'s own `rng.uniform(0,
            # 360)`), so roughly half the time that bearing points straight
            # into the covering pieces or the collapse windrow the figure is
            # deliberately sunk into. Climbing to `_BURIAL_CLOSEUP_ELEV_DEG`
            # above the horizontal at `_BURIAL_CLOSEUP_DIST_M` clears
            # anything resting AT the figure's own height regardless of
            # which way that random yaw points — the same fix an oblique
            # drone shot is for a body under boards. Bearing is still the
            # record's own yaw (no reason to throw away a real number), only
            # the ELEVATION changes.
            ce = math.radians(_BURIAL_CLOSEUP_ELEV_DEG)
            horiz = _BURIAL_CLOSEUP_DIST_M * math.cos(ce)
            rise = _BURIAL_CLOSEUP_DIST_M * math.sin(ce)
            ex = x + horiz * math.cos(yaw)
            ey = y + horiz * math.sin(yaw)
            ez = tz + rise
            snaps.place_camera(stage, (ex * ssf, ey * ssf, ez * ssf),
                               (x * ssf, y * ssf, tz * ssf),
                               focal_mm=focal_mm)
        else:
            ex = x + dist_m * math.cos(yaw)
            ey = y + dist_m * math.sin(yaw)
            snaps.place_camera(stage, (ex * ssf, ey * ssf, tz * ssf),
                               (x * ssf, y * ssf, tz * ssf),
                               focal_mm=focal_mm)
        # POSE IN THE NAME, not just class + index. The whole point of this
        # relaunch is pose review after today's 5-item pass, so a reviewer
        # should not have to open bench_people.json to know what a given
        # closeup is meant to show. A posed static (no skeleton, no `pose`
        # key — see `fire_people.CLASSES`'s own account of `POSED_HUMANS`)
        # falls back to "static" rather than dropping the segment.
        pose = str(r.get("pose") or "static")
        name = "{0}_{1:02d}_{2}_closeup_id{3}".format(
            cls, counts[cls], pose, r.get("id", "?"))
        snaps.snapshot(os.path.join(out_dir, name + ".png"), frames)
        n += 1
    return n


def main():
    t0 = time.time()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path(WORLD))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path(BENCH))
    _, ssf = get_stage_meters_per_unit(stage)

    for p, what in ((DUMP_PATH, "dump"), (MANIFEST_PATH, "manifest"),
                    (PEOPLE_PATH, "people")):
        if not os.path.isfile(p):
            raise RuntimeError(
                "{0} missing at {1} — build the bench trio first "
                "(see this script's own docstring)".format(what, p))
    manifest = load_json(MANIFEST_PATH)
    man_by_i = {r["i"]: r for r in manifest["records"]}

    vram = {"empty": vram_mb("empty stage", "pb")}
    span = 300.0
    build_ground_and_light(stage, span, "pb")
    # `build_ground_and_light` authors its own sunset-y key light (see its
    # docstring: a low 25 deg raking sun, picked for scorch-mark
    # readability). This overrides those same two prims (`/World/domeLight`,
    # `/World/keyLight`) with PB_SKY's preset -- "mid_day" by default, per
    # the user's own review of this bench ("more of a mid day vibe"). PB_SKY
    # =sunset reproduces the line above's numbers exactly, for an A/B.
    resolved_sky = apply_sky_preset(stage, SKY, dome_path="/World/domeLight",
                                    sun_path="/World/keyLight", prefix="pb")
    print("[pb] sky: PB_SKY={0} -> preset '{1}'".format(SKY, resolved_sky))

    flow_root = None
    if FLOW:
        fx.setup_flow_stack(stage, density_cell_size_m=CELL_M,
                            max_blocks=MAX_BLOCKS, scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
        print("[pb] flow stack up at {0} ({1} m cells, {2} block pool)"
              .format(flow_root, CELL_M, MAX_BLOCKS))

    # -- 1) reference EVERY MANIFEST RECORD's bake at its REAL mini-manifest
    #       position — `_bench_rows`, NOT a hardcoded list (2026-09-01 fix;
    #       see `BENCH_BUILDING_INFO`'s own account of the v6 bug this
    #       replaces: a 4th manifest record with no entry in that table
    #       used to be silently invisible to this loop while the people
    #       solve placed figures at its cell anyway).
    rows = []
    for b in _bench_rows(manifest, BAKES_DIR):
        i = b["i"]
        rec = man_by_i.get(i)
        if rec is None:
            print("[pb] *** i={0} ({1}) not in {2} — SKIPPED".format(
                i, b["stem"], MANIFEST_PATH))
            continue
        if not b["stem"]:
            print("[pb] *** i={0} (tag {1!r}): no bake stem resolved "
                  "(not in BENCH_BUILDING_INFO and no sidecar under {2} "
                  "names cell {3!r}) — SKIPPED".format(
                      i, b["tag"], BAKES_DIR, rec.get("cell")))
            continue
        usd = os.path.join(BAKES_DIR, b["stem"] + ".usd")
        js = os.path.join(BAKES_DIR, b["stem"] + ".json")
        if not os.path.isfile(usd):
            print("[pb] *** bake .usd not found: {0} — SKIPPED".format(usd))
            continue
        doc = masses = events = None
        if os.path.isfile(js):
            try:
                doc, masses, events = fb.load_for_assembly(js)
            except Exception as exc:
                print("[pb] sidecar {0} unreadable ({1}) — geometry only"
                      .format(js, exc))
        x, y = float(rec["x"]), float(rec["y"])
        yaw = float(rec.get("yaw_deg", 0.0))
        holder, kid = place_holder(stage, b["tag"], x, y, 0.0, yaw, ssf)
        if not kid.GetReferences().AddReference(usd):
            print("[pb] *** FAILED to reference {0}".format(usd))
            continue
        stage.Load(Sdf.Path(holder + "/bake"))
        for _ in range(2):
            omni.kit.app.get_app().update()
        box = fal.bbox(stage, holder)
        row = {"stem": b["stem"], "tag": b["tag"], "i": i, "why": b["why"],
              "usd": usd, "doc": doc or {}, "masses": masses or {},
              "events": events or [], "x": x, "y": y, "yaw": yaw,
              "prim": holder, "bbox": box}
        rows.append(row)
        print("[pb] {0:<16} i={1:<4} ({2:+7.1f},{3:+7.1f}) yaw {4:+4.0f}  "
              "{5:<38} {6:<4} {7}".format(
                  b["tag"], i, x, y, yaw, b["stem"],
                  row["doc"].get("level") or "?",
                  "bbox {0:.0f}x{1:.0f}x{2:.0f} m".format(
                      box[3] - box[0], box[4] - box[1], box[5] - box[2])
                  if box else "*** EMPTY BBOX ***"))
        print("[pb]     why: {0}".format(b["why"]))
    for _ in range(10):
        omni.kit.app.get_app().update()
    vram["geometry"] = vram_mb("{0} bake(s) composed, no Flow".format(
        len(rows)), "pb")

    # -- 2) put the fire back, ROTATED (these buildings are not at yaw 0) --
    fires = []
    if FLOW and flow_root:
        for r in rows:
            if not r.get("doc") or not r.get("prim"):
                continue
            # THE ROTATING TRANSFORM. `fire_bake.place` moves wall frames,
            # mass centres AND the recorded seats by (x, y, yaw) in one call
            # — the city launcher's own choice for exactly this reason (a
            # non-zero yaw); `place_fire` gets dx=dy=0 because `place`
            # already did the move (its own `_sphere_source` would double
            # it otherwise).
            fb.place(r["masses"], r["events"], r["doc"].get("seats"),
                     r["x"], r["y"], r["yaw"])
            top_z = r["bbox"][5] if r.get("bbox") else r["doc"].get("top_z")
            res = fal.place_fire(
                stage, flow_root, r["doc"], r["masses"], r["events"],
                r["tag"], random.Random(SEED + 31 * r["i"]), top_z,
                0.0, 0.0, scale=1.0, max_emitters=MAX_EMITTERS,
                smoke=SMOKE, smoke_scale=SMOKE_SCALE,
                smoke_window_jets=True, flame_size_scaling=True,
                smoke_size_scaling=True)
            res["i"] = r["i"]
            fires.append(res)
            print("[pb] {0:<16} fire: {1} flame source(s) over {2} "
                  "opening(s), {3} smoke, {4} interior, {5} roof "
                  "(state={6})".format(
                      r["tag"], res["flame"], res.get("openings", 0),
                      res["smoke"], res["interior"], res["roof"],
                      res.get("state")))
            if res.get("note"):
                print("[pb]     " + res["note"])
        total = sum(f["flame"] + f["smoke"] + f["interior"] + f["roof"]
                    for f in fires)
        print("[pb] {0} Flow emitter(s) in all".format(total))
    for _ in range(10):
        omni.kit.app.get_app().update()

    try:
        for _ in range(30):
            omni.kit.app.get_app().update()
        vram["flow"] = vram_mb("Flow up", "pb")
        import carb
        _s = carb.settings.get_settings()
        for _k in ("/rtx/raytracing/fractionalCutoutOpacity",
                   "/rtx/pathtracing/fractionalCutoutOpacity"):
            _s.set_bool(_k, True)
        print("[pb] fractionalCutoutOpacity re-asserted post-composition")
    except Exception as _exc:
        print("[pb] WARNING: could not re-assert fractionalCutoutOpacity "
              "({0})".format(_exc))

    # -- 3) the people — the whole point of this bench ---------------------
    people_doc = load_json(PEOPLE_PATH)
    recs = people_doc.get("people") or people_doc.get("records") or []

    # THE CHECK THAT MAKES "PEOPLE FLOATING IN EMPTY SKY" IMPOSSIBLE AGAIN —
    # 2026-09-01, v6 render: a manifest record the composition loop above
    # silently skipped (no `BENCH_BUILDING_INFO` entry, back when that
    # table WAS the loop) still had six `lean_window` figures and a roof
    # group placed at its cell by the people solve, which reads the
    # manifest independently. ABORT LOUDLY here — BEFORE a single person is
    # authored — if the people solve references any building `i` that is
    # not in `rows` (the buildings that actually composed), regardless of
    # WHY that building failed to compose. This does not depend on fix 1
    # (the manifest-driven loop) staying correct forever; it is the
    # independent second check the coordinator asked for.
    composed_is = [row["i"] for row in rows]
    missing = missing_composed_buildings(composed_is, recs)
    if missing:
        n_orphaned = sum(1 for r in recs if r.get("building_i") in missing)
        raise RuntimeError(
            "[pb] ABORT: the people solve ({0}) references building i={1} "
            "which never composed (composed: {2}) — {3} people record(s) "
            "would be authored floating with nothing under them. Fix the "
            "manifest/bake pairing (see _bench_rows / BENCH_BUILDING_INFO) "
            "and re-run; this check exists so that failure mode can never "
            "ship silently again.".format(
                PEOPLE_PATH, missing, sorted(composed_is), n_orphaned))
    print("[pb] building cross-check: all {0} building(s) the people solve "
          "references ({1}) are among the {2} composed ({3}) — OK".format(
              len({r.get("building_i") for r in recs
                  if r.get("building_i") is not None}),
              sorted({r.get("building_i") for r in recs
                     if r.get("building_i") is not None}),
              len(composed_is), sorted(composed_is)))

    # SAME CALL THE LIVE 39-BUILDING LAUNCHER MAKES (no `ctx=`) — this bench
    # exists to review exactly that authoring path, not a hypothetical
    # better one. See `_placement_no_ctx`'s own docstring for the male-rig
    # seated correction it now applies without a ctx.
    placements, skipped = fpl.to_placements(recs)
    print("[pb] people: {0} record(s) -> {1} placement(s), {2} skipped "
          "({3})".format(
              len(recs), len(placements), sum(len(v) for v in skipped.values()),
              ", ".join("{0}={1}".format(k, len(v))
                        for k, v in sorted(skipped.items())) or "none"))
    by_cls = {}
    for r in recs:
        by_cls[r.get("cls", "?")] = by_cls.get(r.get("cls", "?"), 0) + 1
    print("[pb] people classes: {0}".format(
        ", ".join("{0}={1}".format(k, v) for k, v in sorted(by_cls.items()))))
    sg.apply_placements(stage, placements, BENCH + "/people", ssf)
    omni.kit.app.get_app().update()
    n_live = sum(1 for p in placements
                if p.get("prim_path")
                and stage.GetPrimAtPath(Sdf.Path(p["prim_path"])).IsValid())
    print("[pb] people: {0}/{1} prim(s) valid on the stage under "
          "{2}/people".format(n_live, len(placements), BENCH))

    # -- 3b) THE COVERING DEBRIS OVER A BURIAL FIGURE -----------------------
    # FIX, 2026-08-31 bench-v2 REJECTION: "I don't see any partially under
    # debris like tornado did." `fire_people._cover_burial` DOES author real
    # box specs (`plan.covering`, `write_records`'s own `covering` key) and
    # the gate (`burial_cover_is_authored`) DOES pass — the data was correct
    # and nothing ever put it on the stage. This bench launcher never called
    # `planks.build` at all. `covering` entries are already full `planks`-
    # spec dicts (`x/y/z/len/wide/t/yaw/pitch/roll/class`, `tornado_people.
    # _cover_piece`'s own shape) — the same call the corridor/apron debris
    # elsewhere in this pipeline makes, just with a rubble-appropriate
    # material instead of `planks.wood_material`.
    covering = people_doc.get("covering") or []
    if covering:
        from disaster import planks as pk
        # REMAP `len`/`wide` -> `l`/`w`. `tornado_people._cover_piece`'s own
        # spec shape (`x/y/z/len/wide/t/yaw/pitch/roll/class`) is NOT what
        # `planks._box`/`planks.build` read (`l`/`w`, not `len`/`wide`) — the
        # tornado launcher itself never calls `planks.build` on a raw
        # `_cover_piece` dict without this exact translation first
        # (`tornado_people_preview_launch_script._to_plank_spec`, `suburb_
        # tornado_launch_script`'s own `"l": d["len"], "w": d["wide"]`).
        # Missing this is a `KeyError: 'l'` at authoring time, not a subtle
        # bug — caught here before it ever ran once.
        cov_specs = [dict(d, l=d["len"], w=d["wide"]) for d in covering]
        char_mats = fal.apron_debris_materials(stage, BENCH)
        cov_mats = {"rubble_slab": char_mats["char"],
                   "brick_chunk": char_mats["scorch"],
                   "char_beam": char_mats["char"]}
        made_paths = pk.build(stage, BENCH + "/people_debris", cov_specs,
                              cov_mats, ssf)
        omni.kit.app.get_app().update()
        # DIAGNOSTIC, bench-v3 REJECTION ("untextured grey boxes ...
        # touching nothing"): print exactly which material each authored
        # mesh actually resolved to and whether the binding relationship
        # landed on the prim, so a repeat of "grey, unbound" is visible in
        # the console rather than only in a screenshot. `_CHAR_RGB`/
        # `_SCORCH_RGB` (damage.py) are both near-black — a genuinely BOUND
        # piece cannot read as neutral grey; unbound reads as `planks._box`'s
        # own displayColor fallback (0.66, 0.58, 0.48), a light tan.
        from pxr import UsdShade
        for mp in made_paths or []:
            prim = stage.GetPrimAtPath(Sdf.Path(mp))
            bound = None
            if prim and prim.IsValid():
                rel = UsdShade.MaterialBindingAPI(prim).GetDirectBinding()
                bmat = rel.GetMaterial()
                bound = bmat.GetPath() if bmat else None
            print("[pb]    covering mesh {0} -> material {1}".format(
                mp, bound))
        print("[pb] burial covering: {0} piece(s), {1} mesh(es) -> "
              "{2}/people_debris (rubble_slab/char_beam -> {3}, "
              "brick_chunk -> {4})".format(
                  len(covering), len(made_paths or []), BENCH,
                  char_mats["char"].GetPath(), char_mats["scorch"].GetPath()))
    else:
        print("[pb] burial covering: none in this solve "
              "(author_burial_cover off, or no burial figures placed)")

    print("\n" + "=" * 78)
    print("PEOPLE BENCH  {0} building(s), {1} people record(s)".format(
        len(rows), len(recs)))
    for r in rows:
        b = r.get("bbox")
        print("  {0:<16} i={1:<4} {2:<38} {3:<4} {4}".format(
            r["tag"], r["i"], r["stem"], r["doc"].get("level") or "?",
            "H {0:.0f} m".format(b[5] - b[2]) if b else "NO GEOMETRY"))
    print("  built in {0:.0f} s".format(time.time() - t0))
    print("=" * 78 + "\n")

    # -- 4) captures: overview + one review pair per building ---------------
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec2 = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec2)
            _spec2.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            if FLOW and fires:
                timeline.play()
                for _ in range(300):
                    omni.kit.app.get_app().update()
            _snaps.overview(stage, (0.0, 0.0), span,
                            os.path.join(SNAP_DIR, "bench_top.png"), ssf)
            # OBSTACLES ARE JUST THE OTHER 2 BUILDINGS — `fal.load_dump_
            # positions` reads the same shape `fal.clear_oblique` wants
            # ({"x","y","W","D","yaw"}) straight off `bench_dump.json`,
            # exactly the way the city launcher builds its own obstacle
            # list from the full FC dump.
            positions = fal.load_dump_positions(DUMP_PATH)
            for r in rows:
                b = r.get("bbox")
                if not b:
                    continue
                name = "{0}_{1}".format(r["tag"], r["stem"])
                vp = fal.fire_view_params(r["doc"], r["masses"], b)
                obs = [o for i2, o in positions.items() if i2 != r["i"]]
                if obs:
                    vp, moved = fal.clear_oblique(vp, r["x"], r["y"], obs)
                    if moved:
                        print("[pb] obl camera for {0} pushed {1} step(s) "
                              "clear of a neighbour".format(name, moved))
                _snaps.views_around(stage, {name: (r["x"], r["y"])},
                                    SNAP_DIR, ssf, top_h=vp["top_h"],
                                    obl_dist=vp["obl_dist"],
                                    obl_h=vp["obl_h"],
                                    azimuth_deg=vp["azimuth_deg"],
                                    aim_h=vp["aim_h"])
            print("[pb] snapshots -> {0}".format(SNAP_DIR))
            if PEOPLE_CLOSEUPS:
                closeup_dir = os.path.join(SNAP_DIR, "people_closeups")
                n_shot = _people_closeups(
                    stage, _snaps, recs, closeup_dir, ssf,
                    dist_m=CLOSEUP_DIST_M, limit_per_class=CLOSEUP_LIMIT)
                print("[pb] people closeups: {0} shot(s) -> {1}".format(
                    n_shot, closeup_dir))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[pb] snapshots FAILED: {0}".format(exc))

    vram["end"] = vram_mb("after captures", "pb")
    print("PEOPLE BENCH DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception as _exc:
        import traceback
        traceback.print_exc()
        print("PEOPLE BENCH FAILED: {0}".format(_exc))
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    if keep:
        _app = omni.kit.app.get_app()
        omni.timeline.get_timeline_interface().play()
        while simulation_app.is_running():
            _app.update()
        omni.timeline.get_timeline_interface().stop()
