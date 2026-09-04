#!/usr/bin/env python
"""Freeze ONE urban-fire-CITY dataset cell: geometry + GT_people.json +
GT_hints.json + build_stats.json (+ the flattened, portable .usd itself).

    FREEZE_OUT=/isaac-sim/AirStack/final_disaster_dataset/Fire/Urban/level_1/1 \\
    SCENE_CONFIG=downtown_fire_500 \\
    FC_MANIFEST=/isaac-sim/AirStack/scene_gen/_plans/fire_city_4.json \\
    FC_BAKES=/isaac-sim/.cache/fire_bakes/city_4 \\
    FC_DUMP=/isaac-sim/AirStack/scene_gen/_plans/city_placements_downtown_fire_500_4.json \\
    PEOPLE_VARIANT=1 FREEZE_EXPORT=1 FREEZE_SNAPS=1 \\
    PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \\
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/freeze_urban_fire_city_launch_script.py \\
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

WHAT THIS IS, AND WHAT IT IS NOT
---------------------------------
`freeze_dataset_launch_script.py` is the SUBURBAN dataset launcher: it
compiles ONE `scene_api.build_scene` config and freezes the result. The
urban-fire CITY does not go through `build_scene` at all — it is assembled by
`FireCityApp` (`urban_fire_city_launch_script.py`): an intact downtown with
per-building fire bakes swapped in and NVIDIA Flow flame/smoke put back on
top, driven off THREE artefacts that already exist on disk rather than one
compiled config:

    FC_DUMP       the city LAYOUT dump (`dump_city_placements`, schema
                  `fire_city_placements_dump.v1`) — every house placement's
                  REAL Nucleus-measured W/D/H plus the typology block map,
                  written by a prior `FC_INTACT_ONLY=1` pass over the same
                  SCENE_CONFIG/seed. This IS the "1 km(-class) layout dump"
                  the assignment names.
    FC_MANIFEST   the fire MANIFEST (`urban_fire_city.damaged_manifest`,
                  written by `tools/fire_city_dry_run.py`) — which buildings
                  ignite, from where, to what level, on what schedule.
    FC_BAKES      the shared per-building BAKE directory
                  (`tools/fire_city_bake.sh`'s output: one `.usd` + one
                  `.json` sidecar per manifest record). SHARED across every
                  people variant and every level of this cell that reuses the
                  same manifest — baking is the expensive step and this
                  launcher never repeats it.

This file's OWN job is everything the suburban freeze launcher already does,
adapted to that shape:

  * the PEOPLE_VARIANT axis — offsets ONLY `disaster.fire_people.
    plan_people`'s own seed. The dump, the manifest and every bake are
    IDENTICAL across variants 0..5; only the survivor plan moves, exactly the
    suburban contract ("PEOPLE_VARIANT=k offsets people.seed and NOTHING
    else" — freeze-disaster-dataset skill).
  * GT_hints.json — `disaster.gt_hints.build(..., disaster="urban_fire")`.
    `gt_hints.py` already carries `EXTRA_CLASSES["urban_fire"]` and
    `_TREE_CLASS["urban_fire"]`; nothing in that FIXED-vocabulary file needed
    to change. The `info` dict it wants (shaped like `scene_api.build_scene`'s
    own `info_out`) is assembled here from `FireCityApp`'s own state — see
    `_build_gt_info` below.
  * GT_people.json — `disaster.fire_people.write_records`, the SAME envelope
    (`meta` + `people` + `census` + ...) `tools/fire_people_dry_run.py
    --json` already writes. Not a new format, per the freeze skill's own
    rule ("it is the existing json under the dataset's name").
  * build_stats.json, snaps/ (a documented hook — see FREEZE_SNAPS_HOOK
    below — a dedicated capture agent owns the photo pass; this file writes a
    serviceable default so the pipeline is never blocked on that agent), and
    the freeze itself (`disaster.freeze`, completely UNCHANGED — it flattens
    and verifies whatever stage is composed, and does not care how the stage
    got built).

WHY `FireCityApp` IS LOADED AS A LIBRARY, NOT IMPORTED NORMALLY
-----------------------------------------------------------------
`urban_fire_city_launch_script.py` builds a `SimulationApp` at module scope.
A second one in the same process is a segfault inside the first second — the
exact constraint that already keeps `urban_fire_city_launch_script.py` from
importing `fire_assembly_launch_script.py` (see that file's own docstring).
So: THIS script builds the one and only `SimulationApp` for the process
first, then loads `urban_fire_city_launch_script.py` by FILE PATH
(`_load_by_path`, the same idiom that file already uses to load
`fire_city_manifest.py`) under a name that is never `"__main__"`. That file's
own `SimulationApp(...)` line is now guarded
(`if __name__ == "__main__":` — see the 2026-09-01 patch on top of it), so
loading it this way is a no-op for Kit bring-up and just gives us the
`FireCityApp` class and its helper functions. Every `FC_*`/`FA_*` knob that
class reads is a module-level `_env(...)` read AT IMPORT TIME, so every env
var this script wants honoured (`FC_MANIFEST`, `FC_DUMP`, `FC_PEOPLE_JSON`,
...) is set BEFORE `_load_by_path` runs, never passed as an argument.
Direct invocation of `urban_fire_city_launch_script.py` itself is completely
unaffected by any of this — `__name__ == "__main__"` there, always, so its
own `SimulationApp(...)` line still runs exactly as before.

THE FIRE/SMOKE-IN-THE-FROZEN-EXPORT DECISION — read before changing it
-------------------------------------------------------------------------
**Flow prims are KEPT, unmodified, exactly like the wildfire cells.**
`freeze-disaster-dataset`'s own "Open questions on the export" section:
"Flow prims. The wildfire scenes carry NVIDIA Flow emitters for the flame
and smoke. They export as ordinary prims but render only with `omni.flowusd`
enabled and mean nothing to a non-Isaac consumer. Kept: they are tiny."
`freeze-dataset-state`'s punch list repeats the same verdict. `disaster.
freeze.export_scene` does not special-case Flow at all: `export_as_stage_
async` flattens a `FlowEmitterBox`/`FlowSimulate`/`FlowRender` prim like any
other prim (small attribute sets — no baked volumetric grid is EVER written
to USD; the volume is GPU-simulated at runtime from the emitter geometry),
and `make_portable`/`verify` never look for them. This file does exactly the
same thing: no stripping, no baking to an emissive material, no special
handling whatsoever. `FREEZE_STRIP_FLOW=1` deactivates `/World/flow`
(`disaster.fire.FLOW_ROOT`) before the flatten, purely as an opt-in A/B
knob to measure the alternative — OFF by default, matching the wildfire
convention exactly.

**The one thing genuinely different at CITY scale, flagged rather than
silently assumed**: a suburban wildfire cell carries a handful of Flow
emitters; `urban_fire_city_launch_script.py`'s own `FA_EMITTER_BUDGET`
defaults to 800 emitters across one cell. "Tiny" was measured on the
suburban shape, never on this one — see this launcher's own report for the
open risk. If a frozen cell's `.usd` turns out unexpectedly large, the Flow
subtree (`/World/flow`) is the first thing to measure in isolation
(`Sdf.Layer.Traverse` under that path, or `FREEZE_STRIP_FLOW=1` on a second
export to diff the size) before assuming anything else is the cause.

Env knobs, beyond everything `urban_fire_city_launch_script.py` itself takes
(SCENE_CONFIG, FC_MANIFEST, FC_BAKES, FC_DUMP, FA_*, FC_HIDE, FC_SKY, ...
— all of them pass straight through, this file does not shadow or rename
any of them):

    FREEZE_OUT          output directory. REQUIRED.
    FREEZE_DISASTER     hint ladder to use. Default "urban_fire".
    FREEZE_NAME         basename of the frozen usd. Default: derived from
                         FREEZE_OUT as <disaster>_<locale>_lvl<n>_<k>, same
                         rule the suburban launcher uses.
    FREEZE_EXPORT        1 to flatten + verify + write the frozen .usd.
    FREEZE_COLLECT       1 to also localise textures/MDL (off by default,
                         same reason as the suburban launcher — the collect
                         step stalls on a kit-flattened layer; see
                         `disaster.freeze`'s own module docstring).
    FREEZE_EXIT          1 to close the app after the export, for batch loops
                         over people variants / levels.
    FREEZE_SNAPS         1 (default) to photograph the plate for review,
                         written to <FREEZE_OUT>/snaps/ (NOT the
                         `.nvidia-omniverse/logs` tree `SNAP_DIR` writes to —
                         that is FireCityApp's OWN bench diagnostics, and is
                         left off here by never setting SNAP_DIR).
                         WHATEVER IS AT `SNAP_DIR` IS ALSO MERGED into
                         <FREEZE_OUT>/snaps/, regardless of this flag — see
                         `sync_snap_dir_into_cell`. If a caller explicitly
                         sets `SNAP_DIR` before invoking this launcher (e.g.
                         to run FireCityApp's own richer built-in capture
                         pass, or a separate capture agent's own suite), the
                         2026-09 L1K1 incident is what happens if this merge
                         is skipped: a full baseline_captures set lands
                         under `.nvidia-omniverse/logs/...` and the
                         contract's own `<cell>/snaps/` ships with only
                         whichever pass wrote there directly.
    FREEZE_SNAPS_HOOK   path to a python file exposing
                         `capture(app, snap_dir, ssf)` — loaded and called
                         INSTEAD of the default pass when given. THE HOOK A
                         DEDICATED CAPTURE AGENT OWNS. The default pass
                         (`_default_capture`, below) is a serviceable
                         overview + 3x3 tile walk + one close pair per
                         damaged building + one per distinct people class —
                         good enough to sanity-check a build, not a
                         substitute for a considered per-archetype review
                         pass the way the suburban cells got.
    FREEZE_STRIP_FLOW   1 deactivates `/World/flow` before the flatten — an
                         A/B knob for the fire/smoke decision above. Default
                         0 (kept, matching the wildfire convention).
    FREEZE_WAIVE_VEGETATION  1 (default 0) caps how many instances
                         `make_portable`'s targeted de-instancing will
                         de-instance for one offending prototype
                         (`FREEZE_WAIVE_ABOVE_INSTANCES`, default 500) —
                         above it, the prototype stays instanced and its
                         paths are WAIVED (still build-local in the shipped
                         file, but confirmed to exist on the Nucleus mirror
                         at the same relative location; recorded in
                         freeze_report.json's `waived_mirror_paths`, never
                         silently dropped). Off by default: the primary
                         path de-instances every offender unconditionally.
                         Turn this on only if RSS/VRAM from de-instancing
                         thousands of tree instances proves too heavy on a
                         real run — see freeze.make_portable's own
                         docstring.
    FREEZE_WAIVE_ABOVE_INSTANCES  the threshold above, default 500.
    FREEZE_WAIVE_MIRRORED  1 (default 0) — the run-9 ENDGAME safety valve,
                         gate-side rather than de-instancing-side. For every
                         `build_local` path still standing after
                         `FREEZE_WAIVE_VEGETATION` (if on) and the
                         de-instancing fixpoint itself have both had a
                         chance to fix it, `_enforce_portable` stats its
                         Nucleus mirror twin (same verified machinery the
                         rewrite route uses) and waives ONLY the ones with
                         a confirmed twin — a twin-less path still fails
                         the gate. Exists because a real pod run (L2K1,
                         2026-09-01) showed the fixpoint converging on the
                         live stage (`stage.GetPrototypes()` reporting 0
                         offenders) while the COLD flattened file still
                         carried the same build-local bindings as before —
                         an unresolved scan-vs-flatten divergence, see the
                         dated note in `freeze.make_portable`'s own
                         docstring. Off by default; turn on only as the
                         last-resort ship path when the primary
                         (de-instancing) mechanism alone does not clear the
                         gate.

    PEOPLE_VARIANT      integer k. `fire_people` seed = FP_SEED_BASE +
                         1000 * k. Default 0 (FP_SEED_BASE itself — the
                         already-`fire_people_dry_run.py`-reviewed default).
    FP_SEED_BASE        base fire_people seed before the variant offset.
                         Default 7 (fire_people_dry_run.py's own default,
                         chosen so PEOPLE_VARIANT=0 reproduces whatever cast
                         a 2-D dry-run review already signed off on).
    FREEZE_PEOPLE_JSON  path to an ALREADY-SOLVED, already-reviewed
                         `fire_people` records JSON (the shape
                         `tools/fire_people_dry_run.py --json` writes) — used
                         VERBATIM as GT_people.json instead of re-solving,
                         but ONLY when PEOPLE_VARIANT is 0 (unset). Passing
                         both a nonzero PEOPLE_VARIANT and this override is a
                         contradiction (which seed cast did the reviewer
                         actually look at?) and this launcher refuses it
                         loudly rather than silently picking one.
    FC_SIDECAR_DIR      bake `.json` sidecar directory for the people solve
                         (`fire_people.load_sidecars`) — improves the roof/
                         window/interior-trapped passes' geometry fidelity.
                         Default: FC_BAKES (the sidecars sit right next to
                         the bakes; `fire_city_bake.sh`'s own convention).
"""

import glob
import json
import math
import os
import re
import shutil
import sys
import time


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING — see
    every other launch script in this repo for the same helper and the same
    reason: `os.environ.get(name, default)` never reaches its default when
    the key is PRESENT-but-empty."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


def _flag(name, default="0"):
    return _env(name, default).lower() in ("1", "true", "yes")


_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

SCENE_CONFIG = _env("SCENE_CONFIG", "downtown_fire_500")
OUT = _env("FREEZE_OUT")
if not OUT:
    raise SystemExit("freeze_urban_fire_city: FREEZE_OUT is required")
os.makedirs(OUT, exist_ok=True)
DISASTER = _env("FREEZE_DISASTER", "urban_fire")
NAME = _env("FREEZE_NAME")
EXPORT = _flag("FREEZE_EXPORT", "0")
COLLECT = _flag("FREEZE_COLLECT", "0")
SNAPS = _flag("FREEZE_SNAPS", "1")
SNAPS_HOOK = _env("FREEZE_SNAPS_HOOK", "")
STRIP_FLOW = _flag("FREEZE_STRIP_FLOW", "0")
# 2026-09-01, run 8: the PRIMARY path is to de-instance every offending
# prototype unconditionally (freeze.make_portable's default,
# waive_above_instances=None). FREEZE_WAIVE_VEGETATION=1 is the SAFETY
# VALVE — a prototype with more than WAIVE_ABOVE_INSTANCES current
# instances is left instanced and its offending paths WAIVED (recorded in
# freeze_report.json's waived_mirror_paths, never silently dropped) rather
# than de-instanced. Named for the expected case (thousands of AEC tree
# instances), not enforced by asset type -- see freeze.make_portable's own
# docstring for why a count threshold, not a string/category match, is
# what actually decides it.
WAIVE_VEGETATION = _flag("FREEZE_WAIVE_VEGETATION", "0")
WAIVE_ABOVE_INSTANCES = int(_env("FREEZE_WAIVE_ABOVE_INSTANCES", "500"))
# 2026-09-01, run 9 ENDGAME: gate-side fallback for whatever the
# de-instancing fixpoint itself could not close in time (see the dated
# "OPEN QUESTION, UNRESOLVED" note in freeze.make_portable's own
# docstring) — a build_local path with a STAT-VERIFIED Nucleus twin is
# waived at the _enforce_portable gate, never a twin-less one. Off by
# default; the primary mechanism is de-instancing, this is the last-resort
# ship path.
WAIVE_MIRRORED = _flag("FREEZE_WAIVE_MIRRORED", "0")
VARIANT = int(_env("PEOPLE_VARIANT", "0"))
FP_SEED_BASE = int(_env("FP_SEED_BASE", "7"))
FP_SEED = FP_SEED_BASE + 1000 * VARIANT
PEOPLE_JSON_OVERRIDE = _env("FREEZE_PEOPLE_JSON", "")

if PEOPLE_JSON_OVERRIDE and VARIANT != 0:
    raise SystemExit(
        "freeze_urban_fire_city: both FREEZE_PEOPLE_JSON={0!r} and "
        "PEOPLE_VARIANT={1} were given -- these disagree about which "
        "survivor cast to ship (the override is one specific reviewed cast; "
        "a nonzero variant asks for a DIFFERENT, re-solved one). Pass "
        "PEOPLE_VARIANT=0 (or leave it unset) to use the override, or drop "
        "the override to let this variant solve its own cast."
        .format(PEOPLE_JSON_OVERRIDE, VARIANT))


def _default_name(out_dir):
    """`<disaster>_<locale>_lvl<n>_<k>` from the dataset path — identical
    rule to `freeze_dataset_launch_script._default_name`; the directory
    contract (`.../<Disaster>/<Locale>/level_<n>/<k>/`) is the same for
    every disaster."""
    parts = [q for q in os.path.abspath(out_dir).split(os.sep) if q]
    if len(parts) >= 4 and parts[-2].startswith("level_"):
        dis, loc = parts[-4].lower(), parts[-3].lower()
        return "{0}_{1}_lvl{2}_{3}".format(
            dis, loc, parts[-2].split("_", 1)[1], parts[-1])
    return "scene"


def _find_manifest(spec):
    """`FC_MANIFEST`, or the newest `_plans/fire_city_*.json` — the SAME
    rule `urban_fire_city_launch_script.find_manifest` uses, duplicated in
    miniature (not imported) because that function lives in a file this
    process cannot import until Kit is already running, and this needs to
    run BEFORE Kit starts (see the module docstring's "loaded as a library"
    section — the people solve, which needs this path, has to happen first
    so its result can be handed to `FireCityApp` via an env var)."""
    if spec:
        return spec
    pats = sorted(glob.glob(os.path.join(_SCENE_GEN_DIR, "_plans",
                                         "fire_city_*.json")),
                 key=lambda p: os.path.getmtime(p), reverse=True)
    return pats[0] if pats else ""


def _resolve_city_seed(manifest_path, manifest_doc):
    """`fire_city_manifest.resolve_city_seed`'s exact rule, duplicated for
    the same pre-Kit-import reason as `_find_manifest`."""
    top_seed = manifest_doc.get("seed")
    if top_seed is not None and str(top_seed).strip():
        return str(top_seed)
    stem = os.path.splitext(os.path.basename(manifest_path))[0]
    m = re.search(r"(\d+)$", stem)
    return m.group(1) if m else stem


def _preset_heading_deg(preset, default=45.0):
    """`heading_deg` off the raw preset yaml — `tools/fire_people_dry_run.
    preset_heading_deg`'s exact rule (never through `compile_spec`:
    `compile_disaster.DISASTERS` has no `"fire"` entry and would raise),
    duplicated rather than imported for the same pre-Kit reason."""
    path = os.path.join(_SCENE_GEN_DIR, "config", "presets",
                        "{0}.yaml".format(preset))
    if not os.path.isfile(path):
        return default
    try:
        import yaml
        with open(path) as fh:
            raw = yaml.safe_load(fh) or {}
        if raw.get("heading_deg") is not None:
            return float(raw["heading_deg"])
    except Exception:
        pass
    return default


# ---------------------------------------------------------------------------
# THE PEOPLE SOLVE — BEFORE KIT STARTS.
#
# `disaster.fire_people` is a PURE PLANNER (its own module docstring: "No
# stage, no Isaac, no pxr, no Nucleus"), so this can run cheaply on the host
# before paying for a SimulationApp — and it MUST run first, because the
# result (GT_people.json) is handed to `FireCityApp` through the
# `FC_PEOPLE_JSON` env var, which that class's module-level code reads at
# IMPORT time.
# ---------------------------------------------------------------------------
MANIFEST_PATH = _find_manifest(_env("FC_MANIFEST", ""))
if not MANIFEST_PATH or not os.path.isfile(MANIFEST_PATH):
    raise SystemExit(
        "freeze_urban_fire_city: FC_MANIFEST={0!r} not found and no "
        "scene_gen/_plans/fire_city_*.json to fall back on -- run "
        "tools/fire_city_dry_run.py first".format(_env("FC_MANIFEST", "")))
os.environ["FC_MANIFEST"] = MANIFEST_PATH   # pin it: the people solve and
                                             # FireCityApp MUST use the same
                                             # file, not two different
                                             # "newest" resolutions taken a
                                             # few seconds apart.

from disaster import fire_people as fp        # noqa: E402  (pure, no Kit)

_manifest_doc = fp.load_manifest(MANIFEST_PATH)
CITY_SEED = _resolve_city_seed(MANIFEST_PATH, _manifest_doc)


def _resolve_dump_path(manifest_doc, preset, city_seed):
    """`FC_DUMP`, else the manifest's OWN `placements_dump.path` (written by
    `tools/fire_city_dry_run.py` alongside `placements_dump.sha256` whenever
    it solved against a real `--placements-json` dump -- MEASURED against
    three real manifests in `scene_gen/_plans/`: this pointer's sha256
    matches the file on disk, and its filename does NOT follow the guessed
    `city_placements_<preset>_<seed>.json` pattern at all when the manifest's
    own top-level `seed` is a bake-cache-naming seed rather than the
    compiled preset's own layout seed -- e.g. `fire_city_500m_39.json`
    carries `seed: 138` (the `city_138/` BAKE directory this manifest's
    bakes live under) while its `placements_dump.path` is `fc_dump_500.json`,
    built from `downtown_fire_500.yaml`'s own `seed: 4`. Trusting a guessed
    filename over the manifest's own recorded pointer would have silently
    picked the WRONG dump (or none at all) for every manifest of this shape.
    Falls back to the guessed default path only when the manifest predates
    this field (`fire_city_4.json`'s own shape has no `placements_dump` key
    at all)."""
    env = _env("FC_DUMP", "")
    if env:
        return env
    pd = manifest_doc.get("placements_dump") or {}
    path = pd.get("path") if isinstance(pd, dict) else None
    if path:
        sha = pd.get("sha256")
        if sha and os.path.isfile(path):
            import hashlib
            got = hashlib.sha256(open(path, "rb").read()).hexdigest()
            if got != sha:
                print("[freeze] *** WARNING: {0}'s own placements_dump "
                      "sha256 does not match the file on disk ({1}) -- the "
                      "dump may have been rebuilt since this manifest was "
                      "solved".format(MANIFEST_PATH, path))
        return path
    return os.path.join(_SCENE_GEN_DIR, "_plans",
                        "city_placements_{0}_{1}.json".format(
                            preset, city_seed))


DUMP_PATH = _resolve_dump_path(_manifest_doc, SCENE_CONFIG, CITY_SEED)
if not DUMP_PATH or not os.path.isfile(DUMP_PATH):
    raise SystemExit(
        "freeze_urban_fire_city: FC_DUMP={0!r} not found -- run a "
        "FC_INTACT_ONLY=1 pass over SCENE_CONFIG={1!r} first (it writes the "
        "layout dump `fire_people`/GT_hints both need), or pass FC_DUMP "
        "explicitly".format(DUMP_PATH, SCENE_CONFIG))
os.environ["FC_DUMP"] = DUMP_PATH

BAKES_SPEC = _env("FC_BAKES", "")
SIDECAR_DIR = _env("FC_SIDECAR_DIR", "") or BAKES_SPEC
_sidecars = (fp.load_sidecars(SIDECAR_DIR)
            if SIDECAR_DIR and os.path.isdir(SIDECAR_DIR) else {})

GT_PEOPLE_PATH = os.path.join(OUT, "GT_people.json")
_heading_deg = _preset_heading_deg(SCENE_CONFIG)

if PEOPLE_JSON_OVERRIDE:
    if not os.path.isfile(PEOPLE_JSON_OVERRIDE):
        raise SystemExit("freeze_urban_fire_city: FREEZE_PEOPLE_JSON={0!r} "
                         "not found".format(PEOPLE_JSON_OVERRIDE))
    shutil.copyfile(PEOPLE_JSON_OVERRIDE, GT_PEOPLE_PATH)
    print("[freeze] people: FREEZE_PEOPLE_JSON override, copied verbatim -> "
          "{0}".format(GT_PEOPLE_PATH))
else:
    _dump = fp.load_dump(DUMP_PATH)
    _plan = fp.plan_people(_dump, _manifest_doc, seed=FP_SEED, layout=None,
                           sidecars=_sidecars, heading_deg=_heading_deg)
    fp.write_records(GT_PEOPLE_PATH, _plan)
    _locations = len({(r["cls"], r["group"]) for r in _plan.records})
    print("[freeze] people variant {0}: fire_people seed {1} "
          "(FP_SEED_BASE {2} + 1000x{0}) -> {3} record(s) at {4} "
          "location(s) -> {5}".format(
              VARIANT, FP_SEED, FP_SEED_BASE, len(_plan.records),
              _locations, GT_PEOPLE_PATH))
    if _plan.meta.get("n_burning", 0) <= 0:
        print("[freeze] *** WARNING: fire_people solved ZERO burning "
              "buildings against this manifest/dump pair -- GT_people.json "
              "will be an empty cast. Check FC_MANIFEST/FC_DUMP actually "
              "describe the same city.")

os.environ["FC_PEOPLE_JSON"] = GT_PEOPLE_PATH
# The city launcher's own diagnostic capture writes under
# `.nvidia-omniverse/logs/<SNAP_DIR>`, a different tree than the dataset
# contract's `<FREEZE_OUT>/snaps/`. Leaving SNAP_DIR unset makes
# `FireCityApp.capture()` a documented no-op (`if not SNAP_DIR: return`);
# THIS file does its own capture pass into `<FREEZE_OUT>/snaps/` below.
os.environ.setdefault("SNAP_DIR", "")
os.environ["FC_INTACT_ONLY"] = "0"   # this launcher always wants the full,
                                      # damaged assembly -- never the
                                      # layout-dump-only mode.


# ---------------------------------------------------------------------------
# KIT STARTS HERE.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp                            # noqa: E402

_extra_args = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
               "--/rtx/pathtracing/fractionalCutoutOpacity=true",
               # Static export has no moving geometry.  Avoid building the
               # motion BVH, which is pure VRAM overhead during a freeze.
               "--/renderer/raytracingMotion/enabled=false",
               "--/renderer/raytracingMotion/enableHydraEngineMasking=false",
               "--/renderer/raytracingMotion/enableInstanceInPointInstancer=false"]
_multigpu_count = int(_env("FREEZE_MULTIGPU_COUNT", "0") or 0)
if _multigpu_count > 1:
    _extra_args.extend(["--/renderer/multiGpu/enabled=true",
                        "--/renderer/multiGpu/autoEnable=true",
                        "--/renderer/multiGpu/maxGpuCount={0}".format(
                            _multigpu_count)])
    print("[freeze] distributing Hydra/RTX across {0} GPUs".format(
        _multigpu_count), flush=True)

simulation_app = SimulationApp(launch_config={
    "headless": _env("FREEZE_HEADLESS", "false").lower() in ("1", "true", "yes"),
    # SAME two flags `urban_fire_city_launch_script.py` and
    # `freeze_dataset_launch_script.py` both pass -- the bakes carry
    # fractional-cutout soot/glass materials and RTX forces them opaque
    # without this (see build-urban-fire-scenes' bug 4).
    "extra_args": _extra_args,
})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import carb                                                    # noqa: E402
import omni.kit.app                                            # noqa: E402
import omni.usd                                                # noqa: E402

# The freeze path writes USD and sidecars and captures no viewport when
# FREEZE_SNAPS=0.  Prevent the default viewport from continually asking Hydra
# for frames while thousands of city prims are being composed.
if _env("FREEZE_DISABLE_VIEWPORT_UPDATES", "1").lower() in (
        "1", "true", "yes"):
    carb.settings.get_settings().set("/app/viewport/updatesEnabled", False)
    carb.settings.get_settings().set(
        "/persistent/app/viewport/updatesEnabled", False)
    print("[freeze] viewport updates disabled", flush=True)

from disaster import gt_hints                                  # noqa: E402


def _load_by_path(name, path):
    """Import a module from a FILE without putting its directory on
    `sys.path` -- the idiom `urban_fire_city_launch_script.py` already uses
    for `fire_city_manifest.py`, reused here to load THAT file itself. Safe
    to call now (and only now): its own `SimulationApp(...)` line is guarded
    behind `if __name__ == "__main__":`, and `name` here is never
    `"__main__"`, so loading it builds no second Kit app."""
    import importlib.util as _ilu
    spec = _ilu.spec_from_file_location(name, path)
    mod = _ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_UFC_PATH = os.path.join(_ISAAC_SIM_DIR, "launch_scripts",
                         "urban_fire_city_launch_script.py")
ufcl = _load_by_path("urban_fire_city_launch_script", _UFC_PATH)


# ---------------------------------------------------------------------------
# GT_hints.json -- assembling `scene_api.build_scene`-shaped `info` out of
# `FireCityApp`'s own state. Nothing in `disaster/gt_hints.py` changed for
# this: `EXTRA_CLASSES["urban_fire"]` and `_TREE_CLASS["urban_fire"]` were
# already there (both alias the wildfire tables), so this is entirely glue.
# ---------------------------------------------------------------------------
def _build_gt_info(app):
    placements = app.placements or []

    # ---- houses: every burnt cell from `app.placed`, everything else
    # (never in the manifest, never baked (`app.missing`), or refused
    # composition (`app.refused`)) stays "pristine" -- which is exactly what
    # is actually visible on the stage in each case: `app.missing`/
    # `app.refused` buildings were never hidden, so they render as their
    # ordinary intact kit selves.
    burnt_by_cell = {}
    for r in (app.placed or []):
        cell = r.get("cell")
        if cell:
            burnt_by_cell[cell] = r

    house_objects = []
    for p in placements:
        if p.get("category") != "house":
            continue
        cell = p.get("prim_path")
        r = burnt_by_cell.get(cell)
        if r is not None:
            rec = r.get("rec") or {}
            level = rec.get("level") or (r.get("doc") or {}).get(
                "fire", {}).get("level") or "?"
            style = rec.get("style") or rec.get("asset") or os.path.basename(
                str(p.get("usd") or ""))
            house_objects.append({
                "level": level,
                # measure the BAKE holder (shell + any collapse debris it
                # authored), not the hidden intact cell underneath it.
                "prim_path": r.get("prim") or cell,
                "style": style,
                "yaw_deg": float(r.get("yaw", p.get("yaw_deg", 0.0))),
                "row": False,
                "burn_age_s": rec.get("age_s"),
            })
        else:
            house_objects.append({
                "level": "pristine",
                "prim_path": cell,
                "style": os.path.basename(str(p.get("usd") or "")),
                "yaw_deg": float(p.get("yaw_deg", 0.0)),
                "row": False,
                "burn_age_s": None,
            })

    # ---- trees: `fire.select_fuels` is the SAME "what can burn" query the
    # scorch-vegetation pass itself uses, so this walks the identical
    # population; `veg_scorch["targets"]` (keyed by prim_path) names which
    # of them were actually darkened.
    scorched_paths = set((app.veg_scorch or {}).get("targets", {}) or {})
    tree_objects = []
    try:
        for x_m, y_m, path in ufcl.fx.select_fuels(placements):
            tree_objects.append({
                "level": "scorched" if path in scorched_paths else "pristine",
                "prim_path": path, "species": "?", "yaw_deg": 0.0,
                "burn_age_s": None,
            })
    except Exception as exc:                                   # noqa: BLE001
        print("[freeze] gt_hints: tree pass failed ({0}) -- shipping 0 "
              "tree records rather than guessing".format(exc))

    # ---- vehicles: by asset stem, same as every other disaster. Urban fire
    # never tips a car (`gt_hints.is_toppled` compares against `art_roll`,
    # and a city placement carries no roll/pitch beyond the art correction,
    # so nothing here reads Toppled -- correct, nothing DOES tip a car in a
    # structure fire).
    cars = []
    for p in placements:
        if p.get("category") != "car":
            continue
        cars.append({
            "prim_path": p.get("prim_path"), "usd": p.get("usd"),
            "roll_deg": p.get("roll_deg"), "pitch_deg": p.get("pitch_deg"),
            "axis_up": p.get("axis_up", "Z"),
            "yaw_deg": float(p.get("yaw_deg", 0.0)),
            "heading_deg": None, "role": None, "occupied": False,
        })

    return {
        "parent": ufcl.PARENT,
        "house_objects": house_objects,
        "tree_objects": tree_objects,
        "binfo": {"cars": cars, "pools": [], "park": {}, "clusters": []},
        "cars": [],
        "blockers": [],       # urban fire authors no suburban-style road
                              # blockers; see the "known gaps" note below.
        "records": [],        # people are counted in GT_people.json instead
                              # (see the module docstring) -- gt_hints.build
                              # never reads this for urban_fire's own extra
                              # classes.
        "config": app.config,
    }


def _apron_debris_records(app, ssf):
    """One `Debris` record per building that got a fire-side ground apron
    (`FireCityApp.aprons` -- `disaster.fire_assembly_lib.fire_apron_pass`,
    ONE merged Mesh prim per qualifying building). Matches `gt_hints`'s own
    rule for the wildfire road-blockage litter: one record per FIELD, never
    per lump. A collapse heap (F5c/F6) is deliberately NOT hinted separately
    here, for the same reason a wildfire `rubble` house's thousands of loose
    fragments are not: the building's own `Damaged building` record already
    covers it (`gt_hints.py`'s own "what is NOT in the hints" section) --
    doing otherwise for urban fire alone would be inventing a rule the rest
    of the dataset does not follow.
    """
    bc = gt_hints._bbox_cache()
    out = []
    for r in (app.aprons or ()):
        path = r.get("prim")
        if not path:
            continue
        box = gt_hints._world_box(bc, app.stage, path, ssf)
        if box is None:
            continue
        rec = gt_hints._rec("Debris", box, pieces=int(r.get("n", 0)),
                            source="fire_apron", building_i=r.get("i"),
                            sides="/".join(r.get("sides") or ()),
                            prim_path=path)
        out.append(rec)
    return out


def _build_stats(app, t0):
    return {
        "scene_config": SCENE_CONFIG,
        "disaster": DISASTER,
        "locale": "urban",
        "people_variant": VARIANT,
        "fire_people_seed": FP_SEED,
        "manifest_path": MANIFEST_PATH,
        "dump_path": DUMP_PATH,
        "bakes": BAKES_SPEC or None,
        "city_seed": CITY_SEED,
        "region_m": app.region,
        "n_placements": len(app.placements or []),
        "n_houses": app.n_houses,
        "n_damaged": len(app.placed or []),
        "n_refused": len(app.refused or []),
        "n_missing_bake": len(app.missing or []),
        "wind": app.wind,
        "veg_scorch": {k: v for k, v in (app.veg_scorch or {}).items()
                       if k != "targets"},
        "n_trees_scorched": (app.veg_scorch or {}).get("scorched", 0),
        "aprons": {"n_buildings": sum(1 for r in (app.aprons or ())
                                      if r.get("prim")),
                  "n_lumps": sum(r.get("n", 0) for r in (app.aprons or ()))},
        "background_people_culled": len(app.culled_people or []),
        "companion_props_hidden": len(app.hidden_props or []),
        "gprim_roots_uninstanced": len(app.uninstanced or []),
        "flow_prims_kept": not STRIP_FLOW,
        "elapsed_s": time.time() - t0,
    }


# ---------------------------------------------------------------------------
# snaps/ -- default pass, overridable via FREEZE_SNAPS_HOOK. See the module
# docstring: a dedicated capture agent owns the real photo pass, this is a
# serviceable default so the pipeline is never blocked on it.
# ---------------------------------------------------------------------------
def _default_capture(app, snap_dir, ssf):
    import importlib.util as _ilu
    spec = _ilu.spec_from_file_location(
        "snapshots", os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py"))
    snaps = _ilu.module_from_spec(spec)
    spec.loader.exec_module(snaps)

    reg = app.region or [500.0, 500.0]
    span = max(float(reg[0]), float(reg[-1]))
    snaps.overview(app.stage, (0.0, 0.0), span * 1.05,
                   os.path.join(snap_dir, "overview.png"), ssf)

    # a 3x3 walk at ~1/3 span -- the wildfire launcher's own rule: a 1 km
    # overview puts a building at a few pixels, it cannot be judged alone.
    step = span / 3.0
    pts = {}
    for iy in range(3):
        for ix in range(3):
            pts["tile_%d%d" % (iy, ix)] = (
                -span / 2.0 + step * (ix + 0.5),
                -span / 2.0 + step * (iy + 0.5))
    # one close pair per damaged building -- the reviewable set.
    for r in (app.placed or [])[:24]:
        pts["fire_{0}_{1}".format(r.get("i"), r.get("stem"))] = (
            r.get("x", 0.0), r.get("y", 0.0))
    # one member of each distinct people class actually placed.
    try:
        doc = json.load(open(GT_PEOPLE_PATH))
        seen_cls = set()
        for rec in (doc.get("people") or []):
            cls = rec.get("cls")
            if cls in seen_cls:
                continue
            seen_cls.add(cls)
            pts["people_" + str(cls)] = (rec.get("x", 0.0), rec.get("y", 0.0))
    except Exception as exc:                                   # noqa: BLE001
        print("[freeze] snapshots: could not read GT_people.json for "
              "points of interest ({0})".format(exc))
    snaps.views_around(app.stage, pts, snap_dir, ssf)
    print("[freeze] snapshots -> {0} ({1} subject(s))".format(
        snap_dir, len(pts)))


def capture_snaps(app, out_dir):
    snap_dir = os.path.join(out_dir, "snaps")
    os.makedirs(snap_dir, exist_ok=True)
    if SNAPS_HOOK:
        if not os.path.isfile(SNAPS_HOOK):
            print("[freeze] *** FREEZE_SNAPS_HOOK={0!r} not found -- "
                  "falling back to the default capture pass".format(
                      SNAPS_HOOK))
        else:
            hook = _load_by_path("freeze_snaps_hook", SNAPS_HOOK)
            hook.capture(app, snap_dir, app.ssf)
            return
    _default_capture(app, snap_dir, app.ssf)


def sync_snap_dir_into_cell(out_dir):
    """Merge a SEPARATE capture pass's own output into `<out_dir>/snaps/`
    per the dataset contract.

    2026-09, the first real city freeze (L1K1): the FULL baseline_captures
    suite (people/districts/buildings/overviews) landed at `SNAP_DIR`
    (`/isaac-sim/.nvidia-omniverse/logs/BASE_L1_K1/`) — a caller had set
    `SNAP_DIR` explicitly BEFORE this launcher ran, which this module's own
    `os.environ.setdefault("SNAP_DIR", "")` does not override (`setdefault`
    only fills an ABSENT key), so `FireCityApp.capture()` ran its own,
    richer capture pass there — while THIS file's `capture_snaps` wrote a
    separate, smaller "legacy" pass straight into `<out_dir>/snaps/`. Two
    photo sets, two locations, and only the smaller one landed where the
    contract puts them.

    This does not try to guess which tool produced `SNAP_DIR`'s content or
    prevent a caller from setting it — it just makes sure the CONTRACT
    location ends up complete: whatever is sitting at `SNAP_DIR` when this
    runs gets copied (not moved — the original stays where a diagnostic
    tool may still expect it) into `<out_dir>/snaps/`, on top of (never
    replacing) whatever `capture_snaps` already wrote there. Plain
    `shutil.copytree(..., dirs_exist_ok=True)` rather than an `rsync`
    subprocess, so this has no dependency on an external binary being
    present in the minimal Kit python environment — same end result the
    coordinator's own "rsync post-capture is fine" asked for.

    A no-op, quietly, when `SNAP_DIR` was never set, does not exist, or is
    itself `<out_dir>/snaps` (nothing to merge into itself).
    """
    snap_dir_env = (os.environ.get("SNAP_DIR") or "").strip()
    if not snap_dir_env:
        return
    if not os.path.isdir(snap_dir_env):
        print("[freeze] snaps: SNAP_DIR={0!r} is set but not a directory -- "
              "nothing to merge".format(snap_dir_env))
        return
    dest = os.path.join(out_dir, "snaps")
    if os.path.abspath(snap_dir_env) == os.path.abspath(dest):
        return
    n_before = sum(len(files) for _r, _d, files in os.walk(dest)) \
        if os.path.isdir(dest) else 0
    n_source = sum(len(files) for _r, _d, files in os.walk(snap_dir_env))
    import shutil as _shutil
    _shutil.copytree(snap_dir_env, dest, dirs_exist_ok=True)
    n_after = sum(len(files) for _r, _d, files in os.walk(dest))
    print("[freeze] snaps: merged {0} file(s) from SNAP_DIR={1} into {2} "
          "({3} file(s) already there from this launcher's own capture "
          "pass -> {4} total)".format(
              n_source, snap_dir_env, dest, n_before, n_after))


def main():
    t0 = time.time()
    app = ufcl.FireCityApp()
    app.run()

    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(10):
        omni.kit.app.get_app().update()

    print("[freeze] building GT_hints.json ...")
    info = _build_gt_info(app)
    recs = gt_hints.build(app.stage, info, app.ssf, disaster=DISASTER)
    recs += _apron_debris_records(app, app.ssf)
    for i, r in enumerate(recs):
        r["id"] = i
    reg = app.region or [500.0, 500.0]
    HINTS_JSON = os.path.join(OUT, "GT_hints.json")
    gt_hints.write(HINTS_JSON, recs, meta={
        "scene_config": SCENE_CONFIG, "disaster": DISASTER,
        "people_variant": VARIANT, "fire_people_seed": FP_SEED,
        "manifest_path": MANIFEST_PATH, "dump_path": DUMP_PATH,
        "city_seed": CITY_SEED,
        "region_m": [-float(reg[0]) / 2.0, -float(reg[-1]) / 2.0,
                    float(reg[0]) / 2.0, float(reg[-1]) / 2.0],
        "units": "metres, world frame, plate centred on the origin",
    })

    stats = _build_stats(app, t0)
    with open(os.path.join(OUT, "build_stats.json"), "w") as fh:
        json.dump(stats, fh, indent=1)

    print("\n" + "=" * 72)
    print("FROZEN URBAN FIRE CITY  {0}  ({1:.0f} x {2:.0f} m)".format(
        SCENE_CONFIG, reg[0], reg[-1]))
    print("  out          {0}".format(OUT))
    print("  buildings    {0} total, {1} damaged, {2} refused, {3} missing "
          "bake".format(stats["n_houses"], stats["n_damaged"],
                        stats["n_refused"], stats["n_missing_bake"]))
    print("  GT_people    {0}".format(GT_PEOPLE_PATH))
    print("  GT_hints     {0}".format(HINTS_JSON))
    for k, v in gt_hints.summarise(recs).items():
        print("    {0:<20} {1:>6}".format(k, v))
    print("=" * 72 + "\n")

    if SNAPS:
        try:
            capture_snaps(app, OUT)
        except Exception as exc:                                # noqa: BLE001
            print("[freeze] snapshots FAILED: {0}".format(exc))
    # ALWAYS attempted, independent of FREEZE_SNAPS: SNAP_DIR is a SEPARATE
    # knob (FireCityApp's own, read at import time) a caller may have set
    # to get its richer built-in capture pass -- see this function's own
    # docstring for the 2026-09 incident where that landed outside the
    # contract's `<cell>/snaps/` entirely.
    try:
        sync_snap_dir_into_cell(OUT)
    except Exception as exc:                                    # noqa: BLE001
        print("[freeze] snaps: SNAP_DIR merge FAILED: {0}".format(exc))

    if EXPORT:
        if STRIP_FLOW:
            flow = app.stage.GetPrimAtPath(ufcl.fx.FLOW_ROOT)
            if flow and flow.IsValid():
                flow.SetActive(False)
                print("[freeze] FREEZE_STRIP_FLOW=1 -- deactivated {0} "
                      "before the flatten (A/B knob; wildfire convention is "
                      "to KEEP Flow prims -- see this file's own "
                      "docstring)".format(ufcl.fx.FLOW_ROOT))
        from disaster import freeze
        name = NAME or _default_name(OUT)
        try:
            finfo = freeze.export_scene(
                OUT, name, collect=COLLECT,
                waive_above_instances=(WAIVE_ABOVE_INSTANCES
                                       if WAIVE_VEGETATION else None),
                waive_mirrored=WAIVE_MIRRORED)
            freeze.report(finfo)
            with open(os.path.join(OUT, "freeze_report.json"), "w") as fh:
                json.dump(finfo, fh, indent=1)
        except freeze.PortabilityError as exc:
            # THE PORTABILITY GATE FIRED -- nothing ships (the .usd stays on
            # disk but is never uploaded/referenced downstream from here),
            # but the DIAGNOSIS should not be lost with it. 2026-09-01
            # finding: a bare `except Exception:` here left freeze_report.json
            # either unwritten or written by some other path with
            # `portable_ok: None` -- `exc.info` IS the real, complete
            # `verify()` result (build_local, build_local_bindings,
            # sky_lights, cross_scope_bindings, portable_ok=False, ...) that
            # `_enforce_portable` already computed before raising; write it
            # verbatim so a human/tool reading this cell's freeze_report.json
            # sees exactly WHY it failed, never a blank report.
            print("[freeze] EXPORT FAILED (portability gate): {0}".format(exc))
            try:
                with open(os.path.join(OUT, "freeze_report.json"), "w") as fh:
                    json.dump(exc.info, fh, indent=1)
                print("[freeze] freeze_report.json written WITH THE FAILURE "
                      "(portable_ok={0}) -- see build_local/sky_lights/"
                      "cross_scope_bindings above for why".format(
                          exc.info.get("portable_ok")))
            except Exception as exc2:                            # noqa: BLE001
                print("[freeze] *** could not even write the failure "
                      "report: {0}".format(exc2))
        except Exception as exc:                                # noqa: BLE001
            import traceback
            print("[freeze] EXPORT FAILED: {0}".format(exc))
            traceback.print_exc()

    # Headless default is EXIT: a headless freeze that falls into the
    # keep-open loop below is an invisible 30 GB Kit process spinning Flow
    # forever — two of them squatted on the pod for hours (2026-09-01) and
    # segfaulted every subsequent launch at startup.
    _headless = _env("FREEZE_HEADLESS", "false").lower() in ("1", "true", "yes")
    if _flag("FREEZE_EXIT", "1" if _headless else "0"):
        print("[freeze] FREEZE_EXIT set -- closing after the export")
        simulation_app.close()
        return

    # NOTE: no bare `import omni.timeline` here — a function-local import of
    # any `omni.*` shadows the module-level `omni` for the WHOLE of main()
    # (python scoping), which made line ~694's `omni.kit.app` an
    # UnboundLocalError a thousand seconds into the first pod freeze run.
    import importlib
    timeline = importlib.import_module("omni.timeline").get_timeline_interface()
    timeline.play()
    while simulation_app.is_running():
        omni.kit.app.get_app().update()
    simulation_app.close()


if __name__ == "__main__":
    main()
