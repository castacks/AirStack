# Urban fire scene export — bug catalogue for a test suite

Written 2026-09-03, for an agent building a regression test suite before the
urban-fire corridor pipeline (`.agents/skills/build-urban-fire-city/SKILL.md`,
`scene_gen/tools/make_cell_plan.sh` + `scene_gen/tools/urban_fire_cell.sh`) is
restarted. Scope: bugs actually hit while exporting Fire/Urban cells with the
CURRENT (2026-09-02 corridor rewrite) pipeline, in the order they surfaced.
Older, already-fixed bugs from earlier pipeline generations have their own
catalogues in `.agents/skills/build-wildfire-scenes`, `build-urban-fire-scenes`,
`fix-floating-debris`, and `reassemble-buildings-in-frame` — not repeated here
unless directly relevant to the current pipeline.

Each entry: symptom, root cause, fix status, and a concrete test suggestion.

---

## 1. `fire_corridor.unbakeable()` false-positive on `bake_kind()`'s own refusal text

**Status: FIXED, has a regression test.**

`scene_gen/disaster/fire_corridor.py`'s `_BAKEABLE_HINTS = ("no bake", "fire_b",
"bake")` is a substring match meant to tell a temporary "no bake exists yet"
refusal (worth including in the corridor and baking) apart from a PERMANENT
refusal (unregistered pack, height cap, blacklist — must be excluded from the
corridor before placement search runs). `urban_fire_city.bake_kind()`'s own
`('slice', None)` permanent-refusal message for an unregistered pack reads:

> "...fire_bake.KINDS=(...) has no bake kind for it... must be REFUSED, not
> silently dropped..."

Its own substring `"no bake"` false-matched `_BAKEABLE_HINTS`, so `unbakeable()`
returned `False` — temporary — for a refusal that names itself as permanent in
its own text. Measured 2026-09-02: 44 of 61 L1 corridor records landed as
`needs_bake: true, kind: null` this way, instead of being excluded via
`permanent_bad` before the corridor placement search ran. Most of a "burning"
corridor shipped undamaged while every downstream count still looked correct.

**Fix**: `_PERMANENT_HINTS = ("must be refused", "no bake kind for it")`,
checked BEFORE `_BAKEABLE_HINTS`, in `fire_corridor.py:87,119-124`.

**Existing test**: `scene_gen/tests/test_fire_corridor_unbakeable.py` — 5 cases,
covers the exact false-positive string, a genuine temporary refusal staying
temporary, a pack-blacklist refusal, a height-cap refusal, and `None`.

**For the new suite**: keep this test; it's a good template for the next two
entries, which have no test yet.

---

## 2. AEC asset paths get eagerly resolved to local absolute paths at generation time

**Status: ROOT-CAUSED, NOT FIXED. Deferred — no in-flight corridor currently selects an AEC building.**

`scene_gen/scene_generator.py`'s `_join_asset_root` / `_parse_usd_entry` /
`_normalize_usd_list` eagerly resolve `airstack://` scheme paths to absolute
LOCAL filesystem paths at generation time — i.e. at whatever path they resolve
to on the machine that ran `make_cell_plan.sh` (the HOST). `bake_kind()`'s pack
routing does a literal `u.startswith(spec["dir"])` prefix check against
`PACKS["aec"]["dir"]`'s raw `"airstack://..."` string. On the POD, the
already-absolutized path no longer starts with that raw scheme string, so an
AEC building's `bake_kind()` lookup fails to route it as `aec` at all. GAC/DTC
use real `omniverse://` URLs, which are the same on every machine, so they are
unaffected — this is AEC-specific.

**Not fixed** because none of the L1/L2/L3 corridors currently in flight
select an AEC (brownstone) building, so it hasn't actually broken a bake yet.
It will as soon as a corridor's placement search picks one.

**For the new suite**: a unit test that calls `_join_asset_root`/
`_parse_usd_entry` on an `airstack://...` URL with a HOST-side asset-root
config and asserts the returned string is still `airstack://...` (unresolved),
not an absolute filesystem path. A second test: run `bake_kind()` against a
fixture AEC placement record built from a HOST-resolved dump and confirm it
still routes to `aec`, not `None` — this is the actual failure mode and is
otherwise invisible until a corridor happens to pick an AEC building.

---

## 3. `urban_fire_cell.sh` Stages 5/7 invoke `python.sh` by a relative path that only resolves inside the AirStack checkout

**Status: FIXED on pod -52's local copy. NOT YET PORTED to the git repo — `scene_gen/tools/urban_fire_cell.sh` in this repo (as of `13987fab`) still has the broken invocation at lines 149, 190, 233.**

`urban_fire_cell.sh` Stage 5 (assemble) and Stage 7 (freeze) ran:

```bash
cd $REPO && ./python.sh simulation/isaac-sim/launch_scripts/<script>.py --no-window
```

`python.sh` internally computes its own directory via
`$(dirname "${BASH_SOURCE[0]}")` to find `setup_python_env.sh`. That only
works if `python.sh` lives inside `$REPO` — it doesn't: on this pod it's at
`/isaac-sim/python.sh`, one directory above `$REPO=/isaac-sim/AirStack`. A
symlink into `$REPO` does not fix it either: a symlinked `$0` still resolves
`BASH_SOURCE[0]`'s directory to the symlink's own location, not the real
script's.

**Fix**: invoke it the way `fire_bake.sh`/`fire_city_bake.sh` already do
(`fire_bake.sh:269`, `fire_city_bake.sh:378`):

```bash
cd /isaac-sim && /isaac-sim/python.sh $REPO/simulation/isaac-sim/launch_scripts/<script>.py --no-window
```

Applied and verified (`bash -n` + line diff, only the 4 intended lines
changed) on pod -52's `urban_fire_cell.sh`, backed up as
`urban_fire_cell.sh.pre_pythonsh_fix.bak`. **This fix needs to be ported into
the git repo before any fresh checkout (a new pod, a re-clone) will have it.**

**For the new suite**: a lint-style test that greps `urban_fire_cell.sh` (and
any future stage scripts) for `./python.sh` invoked after a `cd "$REPO"` and
fails the build if found — the correct pattern is always
`cd /isaac-sim && /isaac-sim/python.sh <absolute path under $REPO>`. Cheap,
catches this whole class without needing a pod.

---

## 4. Stage 3b's dump-vs-Kit verification gate is silently skippable

**Status: this is a PROCESS gap, not exactly a code bug — but it is what let #5 below go undetected for an entire bake+assemble cycle.**

`urban_fire_cell.sh` Stage 3b (`urban_fire_cell.sh:132-155`) runs
`verify_dump_matches_kit.py` to prove the offline `fc_dump_1km_l<N>.json` (a
CPU reimplementation of the city layout) agrees with what Kit actually builds,
BEFORE any bake or assembly work happens against it. It's the cheap, early
version of the same check assembly does downstream (`grep 'manifest/city
match'`). But it only runs if you invoke the WRAPPER SCRIPT end-to-end — hand-
invoking Stage 5 (assemble) directly, e.g. while iterating on a fix to some
other stage, skips Stage 3b (and its stamp file `.dump_verified_l<N>_s<seed>`
+ `kit_dump_l<N>.json`) with **no error at all**. Assembly just runs against
whatever dump happens to be on disk.

This is exactly what happened 2026-09-03: while fixing bug #3 above, the
assemble stage was invoked directly to test the fix, bypassing Stage 3b
entirely. The resulting `manifest/city match: 2/42` reading was initially
suspected to be a real desync, but the first-pass evidence was inconclusive
BECAUSE the gate that would have caught it earlier never ran. (It turned out
to be a real desync anyway — see #5 — but the process gap is independently
worth closing.)

**For the new suite**: `urban_fire_city_launch_script.py` (the assemble
launcher) should refuse to run in assemble mode (`FC_INTACT_ONLY=0`) unless a
fresh `kit_dump_l<N>.json` + stamp exists for the current `(preset, seed)` —
making the gate load-bearing at the launcher level, not just a wrapper-script
convention that a hand invocation can bypass. Test: invoke the assemble
launcher directly (no wrapper, no stamp present) and assert it fails loudly
rather than composing a manifest against an unverified dump.

---

## 5. Host/pod layout desync from offline dimensions — repaired and gated

**Status: root cause confirmed; canonical Kit-dimension repair implemented.**

With Stage 3b actually run properly (`PYTHONHASHSEED=0` and
`SG_INSTANCE_PLACEMENTS=1` set identically on both sides, matching the
existing "Determinism" section of the build-urban-fire-city skill), the
offline dump and a fresh Kit rebuild of the SAME `(preset=downtown_urban_fire_1000_l1,
seed=3)` disagree massively:

```
offline : fc_dump_1km_l1.json    443 houses,   2274 placements total, seed=3
kit     : kit_dump_l1.json       471 houses,   7576 placements total, seed=3

matched indices  : 443
moved > 0.50 m   : 443   (ALL of them — many over 1000 m apart)
different model  : 418 of 443
```

Every matched index moved, most by over a kilometre; 418 of 443 carry a
different building model at the same index; total placement count differs by
3.3x. This is NOT the previously-documented hash-seed-order desync (that bug
is already fixed and both sides had identical env vars here) — it is either a
genuinely different bug, or a HOST/POD version mismatch: `fc_dump_1km_l1.json`
may have been generated against a different preset/config/asset-set revision
than what's currently compiled into the pod's Kit build for
`downtown_urban_fire_1000_l1`.

Root cause not yet found. Investigation was paused (background agent stopped
by the user) before determining which side — host dump generation or pod
config/assets — is actually stale.

**For the new suite, in priority order**:
1. A fast, cheap check (no Kit, no GPU) that hashes or fingerprints the
   `downtown_urban_fire_1000_l<N>` preset/config files used by
   `make_cell_plan.sh` and the pod's checked-out copy of the same files —
   catches a stale-checkout version mismatch immediately, before ever
   generating a dump.
2. Make `verify_dump_matches_kit.py`'s check (bug #4) mandatory and run it as
   part of CI/deploy for this pipeline whenever `fc_dump_1km_l<N>.json` is
   regenerated, not just once manually per level.
3. Once the actual root cause is found, add a targeted regression test for it
specifically (this entry should be updated with that once known).

### 2026-09-03 diagnostic and shutdown correction

The apparent post-success hang was independent of the layout mismatch.
Stage 3b passed `--no-window` but did not set `ISAAC_SIM_HEADLESS=true`.
`--no-window` controls Kit window creation; it does not set the launcher's
`_HEADLESS` flag.  Consequently the launcher printed `URBAN FIRE CITY DONE`
and then deliberately entered its interactive update loop.  The wrapper now
sets the headless environment explicitly, and the launcher itself excludes
`FC_INTACT_ONLY` from that loop unless `KEEP_OPEN=1` was explicitly requested.

`verify_dump_matches_kit.py` now also reports dimensions grouped by USD
basename.  Comparing dimensions only at a shared placement index becomes
meaningless as soon as packing has diverged (the two indices then name
different models); the grouped report reveals whether a checked-in offline
footprint differs from Kit's live Nucleus measurement and caused the first
packing divergence.  Run the verifier once more on the already-written pair
before regenerating either file; its `model size conflicts` section is the
next root-cause discriminator.

The diagnostic found 66 conflicts across 122 models: apparent W/D
transposes, square placeholder footprints, and a repeatable ~4.26 m height
undercount on `bld_*_DG0`. The W/D signal included a dump-contract bug:
offline serialized world-oriented axes while Kit serialized resolver-local
axes. Both writers now declare `dimensions_space: world_xy` and serialize the
same representation; the verifier normalizes old and new dumps before its
model comparison.

Packing no longer tries to repair another basename scrape. A real Kit dump is
converted by `tools/kit_dump_to_dimension_catalog.py` into
`config/harvested/urban_building_dimensions.json`, keyed by full USD URL,
effective scale, and source up-axis. `tools/plan_png.py` gives that exact
catalog priority over legacy basename measurements. If Stage 3b finds a
mismatch, both cell drivers now merge the Kit measurements, regenerate that
level's dump, manifest, worklist and review sheets, then rerun parity. Baking
remains blocked unless the regenerated layout matches Kit exactly.

---

## Related, not a code bug: `nas_dest` pointed at NAS bases that never existed

Found and fixed while uploading mission RESULTS (not scene exports) to
airlab-storage: `hurricane_suburban_8robot.yaml`, `raven_suburban_8robot.yaml`,
`urban_fire_8robot.yaml`, and generator `make_rerun_mission.py` all pointed
`nas_dest` at a NAS subdirectory (`coa-sei_hurricane`, `coa-sei_raven`,
`coa-sei_urban`, `coa-sei_rerun`) that was never created — every other mission
uploads to the shared base `/volume2/coa-sei/<mission name>/`. All four fixed
2026-09-03. Not part of the scene-export pipeline and not a candidate for this
test suite, but noted here in case a mission-level test suite is built
separately — the mission's own end-of-run upload silently fails and leaves
the pod alive burning GPU hours with no error surfaced short of an interactive
check.

---

## Open, not a confirmed bug: the "empty blocks" investigation

Paused 2026-09-02 at the user's explicit instruction ("stop investigating...
I will figure this out with the other agent... I will tell you when you
should work on recreating them") before reaching a conclusion. Do not treat
either "empty blocks are a real defect" or "empty blocks are expected" as
settled — a test suite item here would need to wait for that investigation to
resume and conclude. See `.agents/skills/urban-fire-city-layout-gaps` (now
superseded by the corridor rewrite, which changed the underlying selection
model enough that the original measurement may not carry over) and memory
`urban-fire-empty-blocks-paused`.
