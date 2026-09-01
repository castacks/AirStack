# Runbook: freezing Hurricane / Suburban into `final_disaster_dataset/`

Investigation-only deliverable (2026-09-01). **Nothing in this file has been
run.** Every command below is offline (`python3`, `grep`, `ls`) except the
ones explicitly marked as future Isaac Sim / Nucleus commands, which are
written for someone to run later, not by this pass. Read
[SKILL.md](SKILL.md) (the matrix, the directory contract, the self-contained-
export design) and
[freeze-dataset-state/SKILL.md](../freeze-dataset-state/SKILL.md) (what is
actually on disk, the Nucleus upload recipe) first — this file does not repeat
either.

**Zero files were edited to produce this runbook.**
`simulation/isaac-sim/launch_scripts/freeze_dataset_launch_script.py` needs no
change — see §2 for why. Every other file this runbook talks about
(`suburb_hurricane_launch_script.py`, `scene_gen/disaster/*.py`,
`scene_gen/config/presets/*`) is out of scope by the task's own rule and is
being edited by someone else concurrently; this file only specifies what they
still need to do.

---

## 0. The one-paragraph verdict

**The archetype bake gap this runbook was commissioned to close does not
exist in the code as of 2026-09-01.** The premise handed to this
investigation — that the hurricane house ladder is 8 levels
(`pristine, shingles_lost, cover_lost, deck_panels_lost, roof_stripped,
roof_collapsed, partial_collapse, leveled`) and that the middle three have
never been baked — describes a design `disaster/hurricane.py` **itself
documents as superseded** on 2026-08-31 ("STREAM S") and re-confirmed
2026-09-01. `suburb_hurricane_launch_script.py` does not call the 8-level
function at all; it calls `hurricane.tornado_level_for_intensity`, which
returns one of the **tornado's own six levels**
(`pristine/roof_stripped/roof_collapsed/partial_collapse/leveled/swept`), and
those are **100% baked** already, in `archetypes_tornado/`, which is exactly
where `HOUSE_ARCH_DIR` points by default. See §1 for the evidence. **The real
blocker is not a bake — it is that nothing exports a hurricane cell at all**
(§2), and that the 1 km, 3-level presets the dataset contract needs do not
exist yet (§2b).

If this verdict is wrong by the time this is read — i.e. someone has since
reinstated the 8-level ladder — §1's verification script will say so
immediately; run it before trusting anything else in this file.

---

## 1. THE BAKE — verify, do not run

### 1a. What the code actually asks the archetype libraries for

- **Houses.** `suburb_hurricane_launch_script.py:661`:
  `level = hu.tornado_level_for_intensity(it, drng, vuln=vuln)` (plus
  `"swept"`, assigned separately from the surge water state, never from
  wind — see the file's own comment at line 653-660). The archetype key is
  `house_<style>_<level>.usd`, read from `HOUSE_ARCH_DIR`
  (`suburb_hurricane_launch_script.py:211-213`), which **defaults to
  `scene_gen/assets/archetypes_tornado`**, not `archetypes_hurricane`. This is
  deliberate and documented ("HOUSES = TORNADO, VERBATIM" in the module
  docstring, point 3) — the hurricane's own 8-level ladder
  (`hurricane.HOUSE_LEVELS` / `_HOUSE_CUTS` / `house_level_for_intensity`,
  `disaster/hurricane.py:518-875`) is explicitly **"LEFT IN PLACE, unused by
  the launcher"** (`disaster/hurricane.py:889`) — kept only so
  `test_hurricane_house_cuts.py` and the standalone tools that still call it
  (`tools/hurricane_house_cut_search.py`, `tools/hurricane_debris_fences.py`,
  `tools/tornado_png.py`) keep working, not because anything in the assembled
  scene reads it. `scene_gen/tests/test_hurricane_tornado_parity_launcher.py`
  **pins this as a regression test**:

      assert "hu.house_level_for_intensity(it, drng, vuln=vuln)" not in src
      assert "hu.tornado_level_for_intensity(it, drng, vuln=vuln)" in src

  reading `suburb_hurricane_launch_script.py`'s own source. Ran offline,
  2026-09-01: **14/14 pass** (`test_hurricane_tornado_parity_launcher.py` +
  `test_hurricane_tornado_parity_levels.py`, the latter replaying the fitted
  level distributions against two real built scenes' recorded
  `(intensity, vulnerability)` populations).

- **Trees.** `hu.tree_level_for_intensity` (`disaster/hurricane.py:1201`)
  returns one of `hurricane.TREE_LEVELS` — `pristine, defoliated, limbed,
  leaning, fallen, snapped` — read from `ARCH_DIR`, default
  `scene_gen/assets/archetypes_hurricane`. `NO_UPROOT = ("Black_Oak",
  "Douglas_Fir")` (`disaster/hurricane.py:814`) means those two species can
  never resolve to `fallen` (a drawn `fallen` is promoted to `snapped`) — by
  design (wide shallow root plate for the oak, StEER photo evidence of
  mid-trunk snap for the fir), so their missing `_fallen.usd` file is correct,
  not a gap.

### 1b. What is actually baked, measured directly against 1a

    house_<style>_<level>.usd in archetypes_tornado/, style x level:
      8 styles (cottage, l_bungalow, l_family, ranch, terrace, two_storey,
                villa, wide_house)
      x 6 levels (pristine, roof_stripped, roof_collapsed, partial_collapse,
                  leveled, swept)
      = 48 expected, 48 present, 0 missing        (ls, 2026-09-01)

    tree_<species>_<level>.usd in archetypes_hurricane/, species x level:
      6 species x 6 levels, minus NO_UPROOT's "fallen":
      4 species (American_Beech, Common_Apple, Largetooth_Aspen, Shumard_Oak)
        x 6 levels = 24
      2 species (Black_Oak, Douglas_Fir) x 5 levels (no fallen) = 10
      = 34 expected, 34 present, 0 missing        (ls, 2026-09-01)

    house_*.usd in archetypes_hurricane/ (the 8-level ladder's own would-be
    output, from bake_hurricane_archetypes_launch_script.py):
      0 present — consistent with 1a: nothing asks HOUSE_ARCH_DIR=
      archetypes_hurricane for a house today.

**No bake is required to build any of the 15 cells with the pipeline as it
exists today.**

### 1c. The verification script (run this before believing §1b, or after any
future change to `hurricane.py` / the launcher / the archetype directories)

```bash
python3 - <<'PY'
import os, re

REPO = "/home/krrishjain/SEI-COA/disaster-dataset"
SG = os.path.join(REPO, "scene_gen")

launcher = open(os.path.join(
    REPO, "simulation/isaac-sim/launch_scripts",
    "suburb_hurricane_launch_script.py")).read()

# 1) which house-level function does the launcher actually call?
uses_8level = "hu.house_level_for_intensity(" in launcher
uses_tornado6 = "hu.tornado_level_for_intensity(" in launcher
print("launcher calls house_level_for_intensity (8-level ladder):", uses_8level)
print("launcher calls tornado_level_for_intensity (6-level ladder):",
      uses_tornado6)
if uses_8level:
    print("!!! PREMISE FLIPPED: the 8-level ladder is back in the launcher. "
          "Re-derive the required bake set from hurricane.HOUSE_LEVELS "
          "(8 rungs) x the style list in ss.modular_catalogue(config), and "
          "point HOUSE_ARCH_DIR at wherever those got baked -- almost "
          "certainly NOT archetypes_tornado any more.")

m = re.search(r'HOUSE_ARCH_DIR\s*=\s*_env\(\s*\n?\s*"HOUSE_ARCH_DIR",\s*\n?'
              r'\s*os\.path\.join\(_SCENE_GEN_DIR,\s*"assets",\s*"([^"]+)"\)',
              launcher)
house_dir = os.path.join(SG, "assets", m.group(1) if m else "archetypes_tornado")
print("HOUSE_ARCH_DIR default resolves to:", house_dir)

STYLES = ("cottage", "l_bungalow", "l_family", "ranch", "terrace",
          "two_storey", "villa", "wide_house")
if uses_tornado6:
    HOUSE_LEVELS = ("pristine", "roof_stripped", "roof_collapsed",
                     "partial_collapse", "leveled", "swept")
else:
    HOUSE_LEVELS = ("pristine", "shingles_lost", "cover_lost",
                     "deck_panels_lost", "roof_stripped", "roof_collapsed",
                     "partial_collapse", "leveled")

have = set(os.listdir(house_dir)) if os.path.isdir(house_dir) else set()
missing = [f"house_{s}_{lv}.usd" for s in STYLES for lv in HOUSE_LEVELS
           if f"house_{s}_{lv}.usd" not in have]
print(f"house archetypes: {len(STYLES) * len(HOUSE_LEVELS) - len(missing)}/"
      f"{len(STYLES) * len(HOUSE_LEVELS)} present in {house_dir}")
for f in missing:
    print("  MISSING", f)

TREE_SPECIES = ("Black_Oak", "Shumard_Oak", "Douglas_Fir", "Largetooth_Aspen",
                 "Common_Apple", "American_Beech")
TREE_LEVELS = ("pristine", "defoliated", "limbed", "leaning", "fallen",
               "snapped")
NO_UPROOT = ("Black_Oak", "Douglas_Fir")
tree_dir = os.path.join(SG, "assets", "archetypes_hurricane")
have_t = set(os.listdir(tree_dir)) if os.path.isdir(tree_dir) else set()
expect = [f"tree_{sp}_{lv}.usd" for sp in TREE_SPECIES for lv in TREE_LEVELS
          if not (lv == "fallen" and sp in NO_UPROOT)]
missing_t = [f for f in expect if f not in have_t]
print(f"tree archetypes: {len(expect) - len(missing_t)}/{len(expect)} "
      f"present in {tree_dir}")
for f in missing_t:
    print("  MISSING", f)
PY
```

Expected output as of 2026-09-01: `house archetypes: 48/48 present`,
`tree archetypes: 34/34 present`, both "MISSING" lists empty,
`uses_tornado6: True`, `uses_8level: False`.

### 1d. If the design ever reverts to the 8-level ladder

Not recommended to pre-bake against speculation, but the path exists and is
documented so nobody has to rediscover it:
`bake_hurricane_archetypes_launch_script.py` already knows how to bake the
missing three rungs — its own header comment gives
`ARCH_LEVELS=shingles_lost,cover_lost,deck_panels_lost,roof_stripped` and
mentions `hurricane_house_pose_bake.py --adopt-tornado`. That baker, and
whether to run it, is a decision for whoever owns
`suburb_hurricane_launch_script.py` and `disaster/hurricane.py` — not for this
runbook, which documents the pipeline as it stands.

---

## 2. THE BUILD + EXPORT

### 2a. Freeze cannot drive hurricane today, and should not be made to

`freeze_dataset_launch_script.py` has exactly one way to build a scene:

```python
st = build_scene(stage, scene_cfg, ssf, arch_dir=ARCH_DIR, seed=SEED,
                 burn_frac=BURN_FRAC, elapsed=ELAPSED, poles=False,
                 parent_path=PARENT, people_json=PEOPLE_JSON,
                 info_out=info, spec_overrides=overrides)
```

`build_scene` is `scene_gen/scene_api.py:383`, and it is **not
disaster-agnostic** despite the launcher's `_disaster_kind()` helper
recognising `"hurricane"`/`"flood"` as keys (that helper only picks which
`GT_hints.json` class ladder to *label with* — it does not change what gets
*built*). `build_scene`'s own module docstring says what it is: "everything
`suburb_assemble_launch_script.py` used to do inline" — the **wildfire**
launcher. Every damage decision inside it is `age(x, y)` off a wildfire
arrival-time field (`scene_api.py:502-508`); there is no wind field, no surge
water, no tornado-derived house ladder, no `hurricane_people` call anywhere in
the function. Pointing `SCENE_CONFIG` at a hurricane preset would build a
plain, undamaged suburb with a fire-shaped hole cut in nothing, because
`has_disaster` (`scene_api.py:479`) is gated on `config["disaster"]["fire"]`
being present, which a hurricane preset never sets.

The entire hurricane pipeline — the wind/gust field, `draw_vulnerability`,
the surge water pass, the washaway house-drift/collapse pass, the car-float
pass, `hurricane_people`, the overcast sky — is **2,249 lines inline in
`suburb_hurricane_launch_script.py`'s own `main()`**, not a reusable function
`freeze_dataset_launch_script.py` could call instead of `build_scene`.
Extracting that into something with `build_scene`'s signature would mean
rewriting `suburb_hurricane_launch_script.py` (explicitly not mine to touch,
and being edited concurrently) and very likely touching `scene_gen/disaster/*`
too. That is out of scope for this pass and arguably wrong regardless: the
task's own fallback clause anticipated exactly this shape of answer.

**Conclusion: `freeze_dataset_launch_script.py` is left unmodified.** It is
not broken for hurricane — it is simply the wrong tool, architecturally, and
no "minimal" patch to it fixes that.

### 2b. What actually has to happen: export support inside
`suburb_hurricane_launch_script.py`

This is a precise spec for whoever owns that file next, **not code** — every
piece named below already has a working, tested counterpart in
`freeze_dataset_launch_script.py` and/or `scene_api.build_scene` to copy the
*shape* of, not the disaster logic.

**Confirmed by `grep -in freeze suburb_hurricane_launch_script.py` (0 hits,
2026-09-01): none of this exists yet.**

1. **`FREEZE_OUT` (required) and directory contract.** The launcher currently
   writes `GT_hurricane.json`, `PEOPLE_JSON` and its `snaps/` under
   `SNAP_DIR` or `ARCH_DIR`/`HOUSE_ARCH_DIR` (`suburb_hurricane_launch_script.py:
   235-236, 1989`). None of that is the dataset contract's
   `<Disaster>/<Locale>/level_<n>/<k>/` shape. Needs a `FREEZE_OUT` env var
   (same contract as `freeze_dataset_launch_script.py`'s) that both
   `PEOPLE_JSON` and the new `GT_hints.json`/`build_stats.json`/`snaps/`
   resolve under.

2. **`GT_hints.json` — closer than it looks.** `disaster/gt_hints.py` is
   **already hurricane-aware**: `EXTRA_CLASSES["hurricane"] = ("Pool",
   "Parking Lot", "Defoliated Tree", "Destroyed building")` and
   `_HOUSE_DESTROYED["hurricane"] = ("partial_collapse", "leveled", "swept")`
   are live code (`disaster/gt_hints.py:78, 151-153`), not a proposal — this
   supersedes the SKILL.md matrix's stale "Flooded Road / Standing Water /
   Debris Raft / Washover Fan" plan, which was never implemented; the team
   went with tree/building tiering instead. `gt_hints.build(stage, info, ssf,
   disaster="hurricane")` (`disaster/gt_hints.py:277`) expects `info` shaped
   like `scene_api.build_scene`'s `info_out` — and the launcher's own local
   variables are **already most of the way there**:

   | `gt_hints.build` wants | `suburb_hurricane_launch_script.py` already has |
   |---|---|
   | `info["parent"]` | `PARENT` |
   | `info["binfo"]` | `binfo` (same `generate_suburb_on_stage(..., info_out=binfo, assembly=True)` call `scene_api.build_scene` makes — pools/park/clusters/house_instances/tree_instances are disaster-agnostic) |
   | `info["house_objects"]` | `_h_recs` — has `prim_path`, `style`, `level`, `yaw_deg`, `row`; `gt_hints` reads exactly those plus an optional `burn_age_s` it will silently omit |
   | `info["tree_objects"]` | `_t_recs` — has `prim_path`, `species`, `level`, `yaw_deg`; same fit |
   | `info["cars"]` + `binfo["cars"]` | **missing.** `binfo["cars"]` is always empty (scene_api.py's own comment: "the suburb generator never actually fills it"), and the hurricane launcher currently only walks the stage for `car_*` prims when `DO_WASHAWAY` is on. Needs an unconditional stage walk building `{prim_path, usd, roll_deg, pitch_deg, axis_up, yaw_deg, heading_deg, occupied}` per car — the washaway car-pass block (`suburb_hurricane_launch_script.py` ~line 955 on) already does most of this walk and can be reused/generalised |
   | `info["blockers"]` | **does not exist as a concept in this launcher.** The wildfire "fallen tree across the road" / "toppled streetlight" blockage model has no hurricane equivalent built — `Fallen Tree`/road-blockage `Debris` records will simply be empty for hurricane cells unless someone adds one. **Flag this as an open question, not a blocker**: it degrades the hint file (fewer Debris/Fallen Tree records), it does not break the build |

3. **`GT_people.json`.** Already produced —
   `hpp.write_records(PEOPLE_JSON, p_recs, meta={...})`
   (`suburb_hurricane_launch_script.py:1969`) — just needs `PEOPLE_JSON` to
   resolve under `FREEZE_OUT` (item 1) instead of its current default.

4. **`PEOPLE_VARIANT` — does not exist yet, and is needed for all 15 cells.**
   The people RNG is currently `random.Random(SEED + 191)`
   (`suburb_hurricane_launch_script.py:1895`), tied to the **same** `SEED`
   that drives the wind field (`SEED + 23`), house/water draws (`SEED + 5`),
   tree draws (`SEED + 9`), and the car pass (`SEED + 77`). The dataset
   contract requires `PEOPLE_VARIANT` to touch **only** the people draw
   (`freeze_dataset_launch_script.py`'s own docstring: "leaving layout, fire
   and damage bit-identical"). The fix is the same shape
   `freeze_dataset_launch_script.py:279` already uses:
   `random.Random(SEED + 191 + 1000 * VARIANT)` — never touch the other five
   offsets. This is new work; nothing in the file does it today.

5. **`FREEZE_EXPORT` -> `disaster.freeze.export_scene`.** `disaster/freeze.py`
   is **generic to any live stage** — `export_scene(out_dir, name,
   collect=False)` (`disaster/freeze.py:1475`) reads
   `omni.usd.get_context().get_stage()` directly; it has no dependency on
   `scene_api.build_scene`'s `info` shape at all. It needs no
   hurricane-specific change. Add, after the existing "8) GROUND TRUTH" block
   (`suburb_hurricane_launch_script.py:1988-2003`), the same pattern
   `freeze_dataset_launch_script.py:353-380` uses: call
   `freeze.export_scene(OUT, name, collect=...)` guarded by `catch
   freeze.PortabilityError` and a generic `except Exception`, write
   `freeze_report.json`. `DEACTIVATE_DEFAULT`
   (`disaster/freeze.py:496-517`) already lists both `/World/GroundPlane` and
   `/World/Environment` paths this launcher also deactivates by hand at the
   top of `main()` — no new prim paths needed there **unless** the hurricane
   people pass authors its own locator-pole scope (grep found none: no
   `_people_poles`/`build_people_poles` call anywhere in
   `hurricane_people.py` or the launcher — hurricane survivors currently ship
   with **no deactivated answer-key pole scope at all**, unlike every other
   disaster. Worth a decision, not assumed either way here).

6. **`build_stats.json`.** Not strictly in the directory contract, but
   written by every other disaster's freeze path and read by tooling
   (`freeze-dataset-state/SKILL.md`). Would need assembling from the
   launcher's own local tallies (`htally`, `ttally`, `wtally`, `era_tally`,
   `region`, `hcfg`, `scfg`) the same way
   `freeze_dataset_launch_script.py:338-341` assembles it from `build_scene`'s
   return dict.

7. **`FREEZE_NAME` / default naming.** `freeze_dataset_launch_script.py:140-152`
   (`_default_name`) derives `<disaster>_<locale>_lvl<n>_<k>` from the
   `FREEZE_OUT` path's last four components. The same derivation works
   unchanged if `FREEZE_OUT` follows the same directory contract — it is pure
   path string logic, no dependency on `scene_api`. Until it exists, §3's
   per-cell commands below pass an explicit basename rather than assume it.

None of items 1-7 touches `scene_gen/disaster/*` beyond what already exists
(`gt_hints.py`, `freeze.py`, `hurricane_people.py` are all read-only from the
launcher's point of view here) — everything is new code **inside**
`suburb_hurricane_launch_script.py`'s `main()`, where every local variable it
needs (`binfo`, `_h_recs`, `_t_recs`, `p_recs`, `region`, `SEED`, `PARENT`) is
already in scope at the point the ground-truth block runs today. This is a
localized addition, not a rewrite — which is exactly why it belongs there and
not in a `scene_api`-style rewrite.

### 2c. The 1 km / level-1 preset gap (separate from 2a/2b, also blocking)

`scene_gen/config/presets/` currently has `suburb_hurricane_500_l2.yaml` and
`suburb_hurricane_500_l3.yaml` **only** — confirmed by directory listing,
2026-09-01. There is:

- **no `suburb_hurricane_1000_l{1,2,3}.yaml`** — the dataset contract is 1 km
  x 1 km (SKILL.md's own matrix), and both existing hurricane presets are
  500 x 500 m plate mini-versions;
- **no level-1 preset at any size** — only the L2 ("brown", 55 m/s) and L3
  ("you can see into buildings", 70 m/s) rungs of the skill's proposed
  3-level ladder exist; the L1 ("green scene with litter, screen cages down",
  ~38 m/s) preset has not been written.

`suburb_tornado_1000_l1.yaml` / `_l2` / `_l3` are the template to scale from —
they exist and their own header comment documents the exact recipe: scale
`region_m` (`[500,500] -> [1000,1000]`) and every position-based knob by the
same factor, but leave physically-scaled knobs (a track's `width_m`) alone.
**One number in `suburb_hurricane_500_l2.yaml` will NOT scale by the region
factor and needs re-solving by hand**: `shore_offset_m` (`-260.0`, currently
under `overrides.disaster.hurricane`) is plate-relative — "measured from the
world origin along `shore_bearing_deg`" — and the preset's own comment already
warns "A 1 km cut of this scene MUST re-solve it or the shoreline leaves the
plate" (`suburb_hurricane_500_l2.yaml:156-158`). The same file documents the
measurement method used to solve it the first time (a 625-point grid check
against `slope_pct` / `surge_m`) — re-run that at 1 km, do not guess a scaled
number.

This is `scene_gen/config/presets/*` work, explicitly out of scope for this
runbook to write. §5's pre-flight checklist has the check command.

---

## 3. THE 15 CELLS

**This section is prospective.** It cannot be run until §2b (export wiring)
and §2c (1 km presets, including a level-1 preset that does not exist yet)
both land. It is written now so the commands are ready the moment they do,
using the naming convention every other disaster in the matrix already
follows (`suburb_wildfire_1000`, `suburb_tornado_1000_l{1,2,3}`) and assuming
§2b's `FREEZE_OUT`/`PEOPLE_VARIANT`/`FREEZE_EXPORT` knobs land with the exact
names `freeze_dataset_launch_script.py` already uses, for consistency across
the dataset's own tooling (`dataset_upload.py`, `frozen_annotations.py`, the
`FROZEN_SCENE` launcher path) — if whoever wires §2b picks different names,
substitute them here.

Per the contract: lowercase in the filename, capitalised in the path
(SKILL.md line 74).

```bash
for LVL in 1 2 3; do
  for K in 1 2 3 4 5; do
    OUT="/isaac-sim/AirStack/final_disaster_dataset/Hurricane/Suburban/level_${LVL}/${K}"
    NAME="hurricane_suburban_lvl${LVL}_${K}"
    docker exec isaac-sim tmux send-keys -t isaac C-c
    docker exec isaac-sim tmux clear-history -t isaac
    docker exec isaac-sim tmux send-keys -t isaac "clear; \
      SCENE_CONFIG=suburb_hurricane_1000_l${LVL} \
      ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_hurricane \
      HOUSE_ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
      HUR_SEED=11 PEOPLE_VARIANT=${K} \
      HUR_HEADLESS=1 \
      FREEZE_OUT=${OUT} FREEZE_NAME=${NAME} FREEZE_EXPORT=1 \
      PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" /isaac-sim/python.sh \
      /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/suburb_hurricane_launch_script.py \
      --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts" ENTER
    # WAIT for "[hurricane] SCENE_DONE" (the launcher's own completion banner,
    # suburb_hurricane_launch_script.py:2230) in the pane / piped log / Kit
    # log before sending the next line -- see run-isaac-sim-launcher SKILL.md
    # section 2 for how to read any of the three. The launcher already exits
    # on its own after SCENE_DONE (HUR_KEEP_OPEN defaults off), so the loop
    # does not need a FREEZE_EXIT-equivalent knob the way
    # freeze_dataset_launch_script.py does.
  done
done
```

Fifteen iterations: `level_1..3` x `1..5`. `HUR_SEED=11` is held fixed across
all five people variants of a level (same convention as
`freeze_dataset_launch_script.py`'s `MINI_SEED` for wildfire) — only
`PEOPLE_VARIANT` changes within a level, which is what keeps the geometry
bit-identical across the five per §2b item 4. Each level uses its own preset
file (`suburb_hurricane_1000_l1/l2/l3`), which is what makes a level a
different **place**, not the same place hit harder — same rule the wildfire
and tornado ladders already follow (SKILL.md's "the level axis is a layout
axis, not only an intensity one").

**Do not run this loop unattended before checking §5's pre-flight list once**
— in particular the "does this launcher print `SCENE_DONE`" and "does
`freeze_report.json` show `portable_ok: true`" checks. A silent portability
failure on cell 1 of 15 (four hours before anyone looks) is a bad way to find
out `make_portable` did not waive something a hurricane-only asset path
needed.

---

## 4. THE NUCLEUS PUSH

`scene_gen/tools/dataset_upload.py` already exists, is disaster-agnostic (it
walks whatever directory tree is under `--local`/`--only`, with no knowledge
of "Fire" vs "Hurricane" baked into it), and is the tool `freeze-dataset-
state/SKILL.md`'s own inline `omni.client` uploader was packaged into
(2026-09-01). Its own docstring already anticipates exactly this use —
"mirroring the exact same contract, one cell at a time as urban-fire cells are
frozen" applies identically to hurricane cells once they exist. Target root,
confirmed live 2026-08-29 and re-listed 2026-09-01:

    omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/final_disaster_dataset

Dry run first, one cell, before trusting the loop over all 15:

```bash
docker exec isaac-sim bash -c '
  EXT=/isaac-sim/kit/extscore/omni.client.lib
  LIB=$(ls -d /isaac-sim/extscache/omni.client-*/bin | head -1)
  LD_LIBRARY_PATH="/isaac-sim/kit:$LIB:$LD_LIBRARY_PATH" \
  PYTHONPATH="$EXT" /isaac-sim/kit/python/bin/python3 -u \
  /isaac-sim/AirStack/scene_gen/tools/dataset_upload.py \
  --only Hurricane/Suburban/level_1/1 --dry-run'
```

Then for real, whole subtree, once every cell is frozen:

```bash
docker exec isaac-sim bash -c '
  EXT=/isaac-sim/kit/extscore/omni.client.lib
  LIB=$(ls -d /isaac-sim/extscache/omni.client-*/bin | head -1)
  LD_LIBRARY_PATH="/isaac-sim/kit:$LIB:$LD_LIBRARY_PATH" \
  PYTHONPATH="$EXT" /isaac-sim/kit/python/bin/python3 -u \
  /isaac-sim/AirStack/scene_gen/tools/dataset_upload.py \
  --only Hurricane/Suburban'
```

`--only Hurricane/Suburban` restricts the walk to the new subtree so this does
not re-stat the whole existing multi-hundred-cell dataset (the size-skip dedup
makes a full walk safe, just slower — `dataset_upload.py`'s own docstring).
`snaps/` is excluded by default (`--include-snaps` to override, matching
existing precedent — 571 MB/cell nothing at run time reads). `Materials/` will
be empty on Nucleus too, same as every other cell today, because `FREEZE_
COLLECT` defaults off (§2b item 5 does not change that default) — this
matches the existing Fire/Tornado cells' state exactly and is not a hurricane-
specific gap; see `freeze-dataset-state/SKILL.md`'s "collect step is OFF"
section for why (`Usd_CrateFile::_UnpackValue` on Kit's poisoned `assetInfo`).

The tool verifies its own upload by default (re-lists and size-compares —
drop `--no-verify` only to skip that). No separate verification step is
needed.

---

## 5. PRE-FLIGHT CHECKLIST

Run every one of these, in order, before the first `docker exec ... tmux
send-keys` in §3. Each command is offline or a Nucleus listing — none needs
Isaac Sim running.

| # | must be true | command |
|---|---|---|
| 1 | §1's bake verdict still holds (or is re-derived if not) | the script in §1c |
| 2 | the 1 km, 3-level hurricane presets exist | `ls scene_gen/config/presets/ \| grep hurricane` — expect `suburb_hurricane_1000_l1.yaml`, `_l2.yaml`, `_l3.yaml` (not the current `_500_l2`/`_500_l3` only) |
| 3 | those presets compile cleanly (host has no `pxr`, so stub it — the pattern `suburb_hurricane_500_l2.yaml`'s own header comment already gives) | `cd scene_gen && python3 -c "import sys, types;\n[sys.modules.setdefault(m, types.ModuleType(m)) for m in ('pxr','pxr.Gf','pxr.Sdf','pxr.Usd','pxr.UsdGeom','pxr.UsdShade','pxr.UsdSkel','pxr.Vt')];\n[setattr(sys.modules['pxr'], n, types.SimpleNamespace()) for n in ('Gf','Sdf','Usd','UsdGeom','UsdShade','UsdSkel','Vt')];\nfrom compile_disaster import load_scene_config; import json;\nprint(json.dumps(load_scene_config('suburb_hurricane_1000_l1')['disaster']['hurricane'], indent=2))"` — repeat for `_l2`/`_l3`; must print `site_gust_mps` matching the intended level (~38/55/70) and `surge_m` matching (~0.9/2.0/2.8) |
| 4 | `suburb_hurricane_launch_script.py` has gained the §2b export wiring | `grep -c FREEZE_OUT simulation/isaac-sim/launch_scripts/suburb_hurricane_launch_script.py` — 0 today; must be nonzero before §3 can run at all |
| 5 | the hurricane parity/damage tests are still green (per-file, not the whole `-k hurricane` batch — batching many `pxr`-heavy test modules together produces environmental cross-test failures unrelated to any real bug, measured 2026-09-01: `test_hurricane_asphalt.py`'s single real assertion failure only surfaces cleanly run alone) | `cd scene_gen && python3 -m pytest tests/test_hurricane_tornado_parity_launcher.py tests/test_hurricane_tornado_parity_levels.py -q` — expect `14 passed` |
| 6 | the archetype directories resolve inside the container the same way they do on this host | `docker exec isaac-sim bash -c 'ls /isaac-sim/AirStack/scene_gen/assets/archetypes_tornado/house_*.usd | wc -l'` — expect 48; same for `archetypes_hurricane/tree_*.usd` -> 34 |
| 7 | `FINAL_DATASET_DIR` mount is live and has room | `docker exec isaac-sim df -h /isaac-sim/final_disaster_dataset` — 15 cells at ~265 MB (the measured Fire/Suburban size) is ~4 GB; confirm headroom before starting, not after cell 12 |
| 8 | `disaster/freeze.py` still imports clean and `DEACTIVATE_DEFAULT` / `ASSET_MIRROR` still match what §2b item 5 assumes (it is being edited concurrently — re-check immediately before §3, not once at the start of this investigation). **There is no offline pytest coverage for `export_scene` / `verify` / `make_portable` / `_enforce_portable` at all** — confirmed 2026-09-01: `grep -rl "export_scene\|make_portable\|_enforce_portable\|PortabilityError" scene_gen/tests/*.py` returns nothing, and `pytest -k freeze` only collects two unrelated hits (a `TestFreeze` class in `test_bake_reseat.py`, a `test_settle_rest.py` name coincidence). This module's correctness has only ever been exercised live, against a wildfire stage — a first hurricane export is genuinely first-of-its-kind for this code, not a well-trodden path with a green suite behind it | `python3 -c "import ast; ast.parse(open('scene_gen/disaster/freeze.py').read())"` (syntax only) + `grep -n "^DEACTIVATE_DEFAULT\|^ASSET_LOCAL_PREFIX\|^ASSET_MIRROR" scene_gen/disaster/freeze.py` (diff against this runbook's §2b/§6 quotes of them) |
| 9 | the Nucleus target root is reachable and still empty of a `Hurricane/` subtree (so the first upload is additive, not a silent overwrite of someone else's in-flight cells) | the `--dry-run` command in §4, `--only Hurricane` |
| 10 | nobody else is mid-build on the same `isaac-sim` container | `docker exec isaac-sim tmux capture-pane -p -J -t isaac -S -200 \| tail -20` — read before sending anything, per `run-isaac-sim-launcher` SKILL.md's own opening warning about `tmux send-keys` stepping on a live run |

---

## 6. What this pass could NOT determine offline

- **`disaster/freeze.py` has zero offline pytest coverage** — no test file
  exercises `export_scene`, `verify`, `make_portable`, `_enforce_portable` or
  `PortabilityError` (confirmed by grep, §5 row 8). Every wildfire cell that
  has ever been frozen was the proving ground for this module; a hurricane
  cell will be the first stage it has ever seen that was not built by
  `scene_api.build_scene`. Nothing about `export_scene`'s own code reads as
  wildfire-specific (§2b item 5), but "reads as generic" and "verified
  generic" are different claims and this pass could only make the first one.
- **Whether `freeze.export_scene`'s portability gate (`make_portable` /
  `_enforce_portable`) passes clean on a hurricane stage.** `ASSET_LOCAL_
  PREFIX`/`ASSET_MIRROR`/`LOCAL_MIRROR_ROOTS` (`disaster/freeze.py:519-546`)
  are generic to `scene_gen/assets/`, which covers `archetypes_tornado/`,
  `archetypes_hurricane/`, `aec/` (the green species USDs) and `people/`
  (the RenderPeople rigs `hurricane_people.py` uses) by the same blanket rule
  already verified live against Nucleus for the `aec/` subtree
  (`disaster/freeze.py:522-526`). This is a strong reason to expect it works
  unchanged, but "the same rule covers it" is not the same as "it has been
  run" — nobody has exported a hurricane stage through `freeze.export_scene`
  yet, and this pass was barred from doing so (no Isaac Sim).
- **Whether `gt_hints.build`'s bbox/prototype measurement passes cleanly on
  the hurricane-specific prims** — the washaway-displaced houses in
  particular author their own extra `xformOp:translate`/`rotate` on top of
  the wrapper Xform (`suburb_hurricane_launch_script.py`'s own comment at
  lines 704-715 on why `h_{i}` is a bare Xform for exactly this reason).
  `gt_hints._world_box` uses `ComputeWorldBound`, which should compose that
  correctly, but it has not been checked against a live washaway'd house.
- **The exact size and export time of a frozen hurricane cell.** Fire/
  Suburban/level_1/1 measured 265 MB / 447k prims (`freeze-dataset-state/
  SKILL.md`); a hurricane cell adds a static water surface, mud/seed-line
  geometry, and per-house washaway transforms on top of a comparable house/
  tree count, so it will likely be larger, but this pass had no live scene to
  measure.
- **Whether the missing `info["blockers"]` (§2b item 2) meaningfully thins
  the `GT_hints.json` `Debris`/`Fallen Tree` classes for a hurricane cell**,
  since the hurricane's own land-debris field (`# LAND DEBRIS`,
  `washaway.land_debris_specs` / `planks.scatter_from_wreck`) is a different,
  larger mechanism than the wildfire/tornado road-blockage model `gt_hints.py`
  was written against — it may need its own aggregation rule in `gt_hints.py`
  rather than being shoehorned into the `blockers` list, which is a design
  call for whoever owns that file, not something to guess at here.
