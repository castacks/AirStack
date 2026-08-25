---
name: launch-generated-scene-with-drones
description: Fly PX4 drones in a scene that is BUILT in-process by scene_gen instead of loaded from a finished USD. Covers `scene_gen/scene_api.build_scene` (the one entry point, its signature and its load-bearing internal order), the `SCENE_CONFIG`-vs-`ENV_URL` switch in `example_multi_drone_scene_import.py`, the archetype bake on Nucleus, and the six traps that make a generated scene different from a loaded one — STAGE_SCALE, the borrowed sky, fractionalCutoutOpacity, the Pegasus base env, the overhead framing and the spawn contract. Read before wiring a generated scene into a mission or touching either launcher.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Launch a GENERATED scene with drones

## When to use

- Flying `suburb_wildfire` (or any `scene_gen` preset) with PX4 drones, live or
  through an OSMO mission.
- Editing `scene_gen/scene_api.py`, `suburb_assemble_launch_script.py`, or the
  generated branch of `example_multi_drone_scene_import.py`.
- Writing a mission that sets `SCENE_CONFIG` instead of `ENV_URL`.
- Debugging "the plat is 100x too small", "the scene is black", "car glass is
  opaque", "every house is missing", "the map camera shows a 225 m crop".

Companions, not repeats: [run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md)
for how to start a launcher and read what it printed;
[build-wildfire-scenes](../build-wildfire-scenes/SKILL.md) for the burn model,
the archetype bake and the Flow catalogue;
[place-people-in-scenes](../place-people-in-scenes/SKILL.md) for the survivor
planner whose output `build_scene` authors.

---

## 1. Two scene sources, one launcher

`example_multi_drone_scene_import.py` is the drone launcher for everything —
GPS origins, PX4 spawn, ZED stereo, RTX lidar, the overhead orthographic
camera, colliders, the run loop. It gets its world from one of two places, and
**the only thing that selects between them is whether `SCENE_CONFIG` is set**:

| env | source | what happens |
|---|---|---|
| `SCENE_CONFIG` unset | `ENV_URL` | `pg.load_environment(ENV_URL)` — a finished USD off Nucleus, then `reference_root_prims_under_world`, `scale_stage_prim(STAGE_SCALE)`, colliders on `/World/stage`. Unchanged, and 42 missions depend on it. |
| `SCENE_CONFIG=<preset>` | `scene_gen/scene_api.build_scene` | Pegasus' **Default Environment** is loaded only to give the World a PhysicsScene, `/World/GroundPlane` and `/World/Environment` are deactivated, and the plat is authored into the stage. No load, no scale, no sky borrow. |

Everything path-specific in the launcher sits behind `if _GENERATED:`.
`_GENERATED` is computed before `SimulationApp(...)` because the startup config
itself differs (see trap 3).

`suburb_assemble_launch_script.py` is the same scene with no drones — a thin
GUI launcher over the same `build_scene` call. If the two ever disagree about
what the plat looks like, one of them is not calling the API.

---

## 2. The API

```python
from scene_api import build_scene

stats = build_scene(stage, scene_config, scene_scale_factor, *,
                    arch_dir=None, seed=11, burn_frac=0.45, elapsed=None,
                    poles=False, parent_path="/World/stage/generated",
                    people_json=None, sky_fn=None, info_out=None) -> dict
```

| arg | meaning |
|---|---|
| `stage` | the composed stage. Must already exist — get it from `omni.usd.get_context().get_stage()` after `load_environment`. |
| `scene_config` | preset NAME (`"suburb_wildfire"`) or an already-loaded config dict. |
| `scene_scale_factor` | stage units per metre, from `scene_prep.get_stage_meters_per_unit(stage)`. Everything is authored in metres x this. |
| `arch_dir` | the damage bake. `None` -> `default_arch_dir()`: the local `scene_gen/assets/archetypes` if it exists, else `$AIRSTACK_ASSET_ROOT/scene_gen/assets/archetypes`. Accepts an `omniverse://` URL (section 5). |
| `seed` | one seed drives layout, damage levels, vegetation outcome and the survivor plan. |
| `burn_frac` | share of houses inside the burn. |
| `elapsed` | seconds of fire; overrides `burn_frac` outright. `None`/`0` derives it. |
| `poles` | author the magenta survivor + cyan row-home locator markers. |
| `parent_path` | where the plat is authored. Both launchers use the default. |
| `people_json` | survivor ground truth. Written with plain `open()`, so it must be a FILESYSTEM path even when `arch_dir` is a URL. |
| `sky_fn` | `fn(stage, sky_path)`. `None` -> lazily imports `scene_prep.add_sky`, which needs `simulation/isaac-sim/utils` on `sys.path`. |
| `info_out` | optional dict, filled with `binfo`, `placements`, `records`, `blockers`, `cars`, `config`, `arch`, `parent` — the raw internals `SNAP_DIR` and any post-pass needs. |

Returns a flat stats dict: `seconds`, `region`, `elapsed_s`, `span_s`,
`burn_frac`, `seed`, `arch_dir`, `houses`/`houses_missing`/`house_tally`,
`trees`/`trees_missing`/`tree_tally`, `bands`, `flow`, `glades`,
`fences_consumed`, `fences_charred`, `props_scorched`, `park_surfaces`,
`roads_damaged`, `people`/`people_alive`/`people_tally`/`people_json`, `cars`,
`blockers`, `poles`. Both launchers' banners are formatted straight off it —
add a number to the dict, not a `print` inside the API.

Also exported: `default_arch_dir()`, `load_archetypes(arch_dir)`,
`build_people_poles`, `build_row_poles`, `LOCAL_ARCH_DIR`, `PARENT_DEFAULT`.

**The module imports standalone.** Module level is stdlib + numpy + pxr, the
same bar the rest of `scene_gen` holds; `omni.client` (URL archetype listing)
and `scene_prep.add_sky` are imported inside the functions that need them. To
prove it after an edit — no GPU, no pane, safe against a container someone else
is driving (the base recipe is section 6 of
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md); `numpy` and
`yaml` are the two extra prebundles `scene_gen` needs):

```bash
docker exec isaac-sim bash -c '
  U=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
  R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-*/ | head -1)
  C=/isaac-sim/kit/extscore/omni.client.lib
  NP=$(ls -d /isaac-sim/extscache/omni.kit.pip_archive-*/pip_prebundle | head -1)
  YM=/isaac-sim/exts/omni.pip.compute/pip_prebundle
  LD_LIBRARY_PATH="/isaac-sim/kit:${U}bin:${R}lib:$C/bin" \
  PYTHONPATH="$U:$C:$NP:$YM:/isaac-sim/AirStack/scene_gen" \
  PXR_PLUGINPATH_NAME="${R}usd/omni_usd_resolver/resources" \
  PYTHONDONTWRITEBYTECODE=1 \
  /isaac-sim/kit/python/bin/python3 -c "
import inspect, sys, scene_api
print(inspect.signature(scene_api.build_scene))
print(sorted(m for m in sys.modules if m.split(\".\")[0] in (\"omni\",\"carb\",\"isaacsim\")))
"'
# -> the signature, then []   in ~1 s
```

That `[]` is the whole point: if a Kit module shows up there, someone put an
app-side import at module level and `scene_gen` stopped being host-runnable.

---

## 3. THE ORDER INSIDE `build_scene` IS LOAD-BEARING

    plan survivors -> author the queue's CARS -> burnable/scorch pass
      -> ground scar + Damaged_Asphalt re-bind -> glades -> the PEOPLE

Not stylistic. Two of those edges are the reason the function exists at all
rather than a set of helpers a caller sequences:

- **The evacuation queue's cars go in BEFORE the scorch pass.** They are cars
  and they belong to the fire, so `damage.soot_materials` has to see them in
  `placements` like every other car on the plat. Author them after and you get
  a gridlocked queue of showroom-clean vehicles in the middle of a burn scar.
- **The people go in AFTER it.** `"human"` deliberately matches nothing in
  `BURNABLE`, and holding them back until 4c is the belt-and-braces version of
  the same rule: a survivor is not scorched.

The ground scar and the `Damaged_Asphalt` re-bind sit between the two because
both walk `<parent>/ground`'s children and the scorch pass has already
re-bound the park surfaces there. The glade pass must run after the trees are
referenced (it deactivates tree prims) and before the people (it is clearing
ground for them to stand on).

The step numbering (`1)`, `2)`, `3)`, `4)`, `4a)`, `4b)`, `4b-2)`, `4c)`, `5)`)
is deliberately non-monotonic and matches what
`suburb_assemble_launch_script.py` carried inline before the extraction, so the
API body still diffs cleanly against git history. Do not renumber it.

---

## 4. Env knobs

Read by the drone launcher only when `SCENE_CONFIG` is set, and by
`suburb_assemble_launch_script.py` always:

| env | default | does |
|---|---|---|
| `SCENE_CONFIG` | — (drones) / `suburb_wildfire` (assemble) | preset name. **Setting it is what selects the generated path.** |
| `MINI_SEED` | `11` | the one seed. |
| `MINI_BURN_FRAC` | `0.45` | share of houses inside the burn. |
| `MINI_ELAPSED` | `0` | seconds of fire; overrides `MINI_BURN_FRAC`. |
| `ARCH_DIR` | local bake, else `$AIRSTACK_ASSET_ROOT/scene_gen/assets/archetypes` | the damage bake; local path or URL. |
| `PEOPLE_JSON` | `scene_gen/assets/archetypes/humans_<seed>.json` | survivor ground truth. Filesystem only. |
| `PEOPLE_POLES` | off | `1`/`true`/`yes` authors the locator markers. |
| `AIRSTACK_ASSET_ROOT` | repo | repoints `airstack://`. **Read by `scene_generator` at IMPORT time** — it must be in the environment before the process starts; nothing in either launcher writes it. |
| `SUBURB_COLLIDERS` | `ground` | drone launcher only: `off` \| `ground` \| `all`. |
| `SNAP_DIR` | — | assemble launcher only: viewport captures (see run-isaac-sim-launcher §5). |

Drone-side, generated path only:

| env | default | does |
|---|---|---|
| `NUM_ROBOTS` | `1` | fleet size; also what the mission runner sizes containers by. |
| `SPAWN_CONFIGS` | — | JSON list of dicts needing only `x_m`/`y_m`. Wins over `SPAWN_POLY`. |
| `SPAWN_POLY` | — | JSON rect (2 opposite corners) or convex polygon; `NUM_ROBOTS` spawns randomized inside it. |
| `SPAWN_HEIGHT_M` | `0.5` | z above the floor. |
| `SPAWN_SEED`, `SPAWN_MIN_DIST_M`, `SPAWN_MARGIN_M` | — / `3.0` / `0.0` | `SPAWN_POLY` knobs. |
| `OVERHEAD_ALTITUDE_M`, `OVERHEAD_COVERAGE_M`, `OVERHEAD_CENTER_X_M`, `OVERHEAD_CENTER_Y_M`, `OVERHEAD_PX_PER_METER` | `400` / `1600` / `0` / `0` / `1.28` | map camera framing. |
| `ISAAC_SIM_HEADLESS`, `ISAAC_SIM_LIVESTREAM`, `ISAAC_SIM_LIVESTREAM_UDP_PORT` | `false` / `false` / `49099` | app startup. `entrypoint.sh` defaults `ISAAC_SIM_LIVESTREAM=true` on OSMO. |

**`SPAWN_CONFIGS` / `SPAWN_POLY` are read on the GENERATED path only.** The
ENV_URL branch's fleet is the three hard-coded configs at ±3 m of the origin,
and that is the compatibility bar this refactor was held to. When the
multi-raven launcher (which parses them on both paths, and also reads `ENV_URL`
from the environment) merges into this branch, delete the gate — it exists to
keep the ENV_URL path byte-identical, not because the fleet contract is
scene-specific.

---

## 5. Archetypes, and why `os.listdir` is not enough

The plat is assembled BY REFERENCE to `house_<style>_<level>.usd` /
`tree_<species>_<level>.usd` baked by `bake_archetypes_launch_script.py`. That
bake is **untracked**: `scene_gen/assets/archetypes/` does not exist in a fresh
clone, so on an OSMO pod `os.listdir(ARCH_DIR)` raises before a single prim is
authored.

`scene_api.load_archetypes` therefore takes a URL:

- `omni.client.list(base + "/")` is the direct analogue of `os.listdir` and is
  tried first.
- `archetypes.json` (the bake manifest) is the fallback for a server that
  refuses a listing. Its `usd` fields carry the **bake machine's** absolute
  paths, so only the basename is usable and it is re-joined onto the base.
- Only DISCOVERY is filesystem-bound. `GetReferences().AddReference` takes an
  `omniverse://` URL happily, so nothing downstream cares.

`default_arch_dir()` falls back to `$AIRSTACK_ASSET_ROOT/scene_gen/assets/archetypes`
only when the local bake is genuinely absent, so a workstation run is
unchanged and a pod needs no second variable beyond `AIRSTACK_ASSET_ROOT`.

**Zero archetypes raises.** It is not an error anywhere downstream — it builds
a plat with roads and no houses, which reads as a bad scene rather than as a
broken path, and that has cost real debugging time. Same for a `arch_dir` that
does not exist.

---

## 6. The traps

**1. NO `STAGE_SCALE` / `scale_stage_prim` on a generated scene.** A loaded
Nucleus stage is authored in centimetres and gets `scale_stage_prim(stage,
"/World/stage", 0.01)`. The generated plat is authored in metres x
`scene_scale_factor` and is already correct; scaling `/World/stage` by 0.01
shrinks a 1600 x 1200 m suburb to 16 x 12 m. The symptom is a drone standing
on a doll's house, not an error.

**2. No `reference_root_prims_under_world(ENV_URL)`.** There is no source USD
to borrow a sky, sun or environment light from — `ENV_URL` on this path is
Pegasus' flat default env, and borrowing its root prims puts the ground plane
back. The sky comes from the config: `add_sky(stage, resolve_sky(config))`,
which `build_scene` does for you. `scene_generator.resolve_sky` only RESOLVES
the path (that module stays sim-agnostic); the HDRI-vs-borrowed-stage dispatch
is in `scene_prep.add_sky`, which is why the API imports it lazily.

**3. `fractionalCutoutOpacity` needs BOTH forms.** The renderer forces cutout
opacity to 1.0 unless asked otherwise, which makes car glass opaque and hides
every occupant `disaster.people` put inside a vehicle — the whole gridlock and
cul-de-sac scenario becomes invisible. It has to be a `SimulationApp`
`extra_args` entry (`--/rtx/raytracing/...` **and** `--/rtx/pathtracing/...`)
*and* re-asserted through `carb.settings.get_settings().set_bool(...)` after
the stage is composed. The startup form alone does not survive composition;
the carb form alone is too late for startup. Both launchers do both.

**4. Pegasus' "Default Environment" is still loaded — and then switched off.**
`PegasusInterface` needs a `World` with a valid `PhysicsScene` to hang the plat
off, and `load_environment` is how it gets one. Immediately after
`wait_for_stage`, `/World/GroundPlane` and `/World/Environment` are
deactivated, because the generated plat brings its own ground and the config
brings its own sky. Skip the load and PX4 has nothing to spawn against; skip
the deactivation and you fly over a grey infinite plane with double lighting.

**5. The overhead camera must frame the PLAT, not the spawn crop.** The
ENV_URL default is 225 m per side at 10 px/m — sized for a 300 m stage and
useless as a mission map for a 1600 x 1200 m region. The generated default is
1600 m at 1.28 px/m, which is exactly 2048 px: `add_overhead_camera_publisher`
caps at `max_resolution=2048` and silently clamps anything larger, so asking
for 2 px/m gets you a stretched 2048 px image, not a sharper one.

**6. Dome-light and vehicle-prop branches key off `ENV_URL` substrings.** On
the multi-raven launcher those are `"downtown_edited" in ENV_URL`,
`"ModernCity" in ENV_URL` and friends. They are meaningless on a generated
scene — `ENV_URL` there names Pegasus' default env, not the world — so a
generated path must never fall into them.

**Also worth knowing:**

- **`set_gps_origins` goes FIRST**, before the ~450 s scene build. Robot
  containers block on those files and the origins are known from
  `DRONE_CONFIGS` alone; making them wait out the assembly is pure dead time on
  the readiness gate. It is the first statement in `PegasusApp.__init__` for
  exactly this reason.
- **Colliders default to `ground`, not `all`.** The plat is ~10^5 prims and
  `add_colliders` is a Python recursion that prints per mesh, so a full pass is
  minutes of log spam — and mostly futile: houses and trees are referenced
  `SetInstanceable(True)` (what keeps 9k trees inside VRAM) and `GetChildren()`
  does not descend into an instance, so they get no collider either way.
  `<parent>/ground` is the one surface a drone needs to take off from.
- **`pymavlink` uses `select.select()`, which dies on fd >= 1024.** DDS over
  TCP (fastdds `LARGE_DATA`) opens a connection per remote participant and
  pushes this process's fd numbers past that; every MAVLink read then throws
  `filedescriptor out of range in select()`. The generated path monkeypatches
  `mavutil.mavfile.select` to use `poll()` (no fd-number limit) BEFORE Pegasus
  builds its connections.
- **`objaverse://` has no Nucleus copy** and `scene_gen/assets/objaverse/` is
  untracked, so ~41 small props render as placeholder prisms on a pod.
  `scene_generator` reports it and carries on. Not fatal, not fixed here.
- **The build costs ~450 s** on `suburb_wildfire` (236 s of it the layout; 504
  houses + 9,465 trees referenced, 0 missing, 10.4 GB GPU) BEFORE PX4 starts.
  A mission's `ready.timeout_s` must clear that — the 600 s default times out
  mid-build. `conavgpt_wildfire_1robot.yaml` uses 1800 s.

---

## 7. Wiring a mission

The generated path is selected by env, so a mission is an ordinary mission:

```yaml
env:
  ISAAC_SIM_SCRIPT_NAME: example_multi_drone_scene_import.py
  SCENE_CONFIG: suburb_wildfire            # no ENV_URL — this is the switch
  AIRSTACK_ASSET_ROOT: "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA"
  SPAWN_CONFIGS: '[{"x_m": 302.4, "y_m": -144.3, "orient": [0, 0, 0.9342, 0.3568]}]'
  SPAWN_HEIGHT_M: "0.5"
  PEOPLE_POLES: "1"
ready:
  timeout_s: 1800
```

`osmo/missions/conavgpt_wildfire_1robot.yaml` is the worked example. Two things
it gets right that are easy to miss:

- **The default spawn is derived, not eyeballed.** One drone on a 10.7 m local
  road at the SE flank: 166 s of arrival-time margin outside the severity-0.6
  front at `MINI_BURN_FRAC=0.45` (so it starts in clear air, not smoke), 381 m
  from the burn centre, yawed to face it —
  `[0, 0, sin(69.05°), cos(69.05°)] = [0, 0, 0.9342, 0.3568]`. Changing
  `MINI_SEED`, `MINI_BURN_FRAC` or the preset's epicenter invalidates the
  margin figure. The same numbers are the launcher's built-in default when no
  `SPAWN_CONFIGS` is given.
- **The search area is sized to CONTAIN EVERY GT PERSON**, not to the burn
  ellipse. The ellipse left one `at_home` survivor outside the polygon, and a
  target the drone is forbidden to fly to is an unreachable recall point that
  looks like a planner failure. Check a new polygon against `PEOPLE_JSON`
  before trusting a recall number.

---

## 8. What this path does NOT give you

- **`GT_ANNOTATIONS` is near-useless here.** Bounding boxes only cover prims
  carrying SEMANTIC LABELS, and nothing in the procedural plat authors one
  except the Flow fire emitters (`disaster.fire`, `semantic_class "fire"`).
  Houses, trees, cars and survivors are unlabelled. The scene's real ground
  truth is `PEOPLE_JSON`, written by the people pass.
- **No physics settle.** The assembly is O(reference) and there is no drop, so
  the burnable vocabulary is consumed-or-scorched and nothing tips over. See
  [build-wildfire-scenes](../build-wildfire-scenes/SKILL.md) for why that is
  the scaling path and what it costs.
- **No incremental rebuild.** Changing a preset means a full relaunch (~450 s).
  Iterate the SCENE on `suburb_assemble_launch_script.py`, which skips PX4
  entirely, and only move to the drone launcher once the plat is right.
