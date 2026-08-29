# Build and Fly Your Own Scene

In this tutorial you'll create a simulation environment of your own from
scratch — no Unreal Engine, no special server access — and fly the AirStack
drone in it. You'll author a small obstacle stage in the Isaac Sim editor,
register it in the **scene catalog** under a shortname, and launch it with
`airstack up --sim isaac --scene`, watching AirStack turn a graphics-only
stage into a physics-ready scene automatically. Plan for **30–45 minutes**.

Prerequisites: you've completed [Get AirStack Flying](index.md) — cloned the
repo, built or pulled the images, and flown the default Isaac Sim scene once.

## 1. Open the Isaac Sim editor

AirStack ships a GUI-only compose service ([Isaac Sim Container Workflows](../simulation/isaac_sim/container_workflows.md))
that opens the full Isaac Sim editor with no Pegasus launch script and no
drones — exactly what you want for authoring:

```bash
airstack up --profile isaac-sim-gui isaac-sim-gui
```

**Check:** the Isaac Sim editor window appears on your display with an empty
stage. If it doesn't, run `airstack logs isaac-sim-gui` — the usual fix is
`xhost +local:docker` (see
[troubleshooting](../simulation/isaac_sim/container_workflows.md#troubleshooting)).

## 2. Author a stage and save it into the repo

Build a minimal obstacle course — the menus below are standard Isaac Sim GUI
actions, not AirStack-specific:

1. Add a floor: **Create > Physics > Ground Plane**.
2. Add obstacles: **Create > Mesh > Cube**, then use the translate/scale
   gizmos to spread three or four cubes a couple of meters apart, leaving the
   world origin clear (the drone spawns there). Each cube is 1 m per side by
   default — a handy scale reference.
3. Save with **File > Save As** to
   `/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/my_first_scene.stage.usd`.

The repo is mounted read-write at `/isaac-sim/AirStack` inside the container,
so the file lands in your checkout. The `*.stage.usd` suffix is the
[naming convention](../simulation/isaac_sim/index.md#usd-file-naming-conventions)
for "graphics only, no physics, no robots" — physics comes later, for free.

**Check:** on the host, `ls -lh simulation/isaac-sim/assets/scenes/` lists
`my_first_scene.stage.usd`. Then `airstack down` to stop the editor.

## 3. Register the scene in the catalog

The [scene catalog](../simulation/scenes.md) (`simulation/scenes.yaml`) maps
simulator-agnostic shortnames to whatever each simulator understands — an
Isaac entry may be a Pegasus catalog key or any `omniverse://` / `https://`
URL / `*.usd` path. Add an entry pointing at your stage's **container path**
(Isaac Sim authors new stages in meters, so `stage_scale` stays `1.0`):

```yaml
scenes:
  # ... existing entries ...
  my-first-scene:
    isaac:
      ref: /isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/my_first_scene.stage.usd
      stage_scale: 1.0   # scale applied to /World/stage; 0.01 for cm-authored stages
```

**Check:** the resolver turns the shortname into launch config:

```bash
python3 simulation/resolve_scene.py --sim isaac --scene my-first-scene
# ISAAC_SIM_SCENE=/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/my_first_scene.stage.usd
# ISAAC_SIM_STAGE_SCALE=1.0
```

## 4. Fly it

```bash
airstack up --sim isaac --scene my-first-scene
airstack ready    # waits: containers → sim /clock → nodes → PX4 ready
```

At launch, the launch script's shared base class
([`pegasus_app.PegasusApp`](../simulation/isaac_sim/spawning_drones.md))
prepares your stage automatically: it scales `/World/stage` by `stage_scale`,
applies collision geometry to every mesh (`add_colliders` — without this,
drones fall through the floor), and adds a dome light (`add_dome_light`): your
graphics-only stage becomes a physics-ready scene, no USD edits by hand.

**Check:** the drone spawns resting on your ground plane among the cubes.
Press `Takeoff` and then `Navigate` in Foxglove exactly as in
[Get AirStack Flying](index.md#move-robot) — the drone lifts off. If the
spawn point (world origin) is inside an obstacle, move the cube and relaunch.

## 5. Bake a self-contained `*.scene.usd`

The prep from step 4 happens in memory on every launch. To bake it into a
shareable, self-contained scene package, copy
`simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`
to `my_scene_bake.py` alongside it, and add one kwarg to `PegasusApp(...)`:

```python
save_scene_to="/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/my_first_scene_baked",
```

Launch once with your script selected:

```bash
ISAAC_SIM_SCRIPT_NAME=my_scene_bake.py airstack up --sim isaac --scene my-first-scene
```

`PegasusApp` exports the prepared stage (colliders and dome light included)
and collects every referenced asset into that directory via
[`save_scene_as_contained_usd`](../simulation/isaac_sim/spawning_drones.md#scene-prep-helpers).

**Check:** `ls simulation/isaac-sim/assets/scenes/my_first_scene_baked/`
shows a root `prepared_scene.usd` (plus `SubUSDs/`). Rename it to
`my_first_scene.scene.usd` per the physics-enabled naming convention — it now
loads standalone, no prep needed.

## Congratulations

You built an environment, gave it a shortname every teammate (and every
simulator) can use, and flew the full autonomy stack in it — the same
`--scene` flow that serves the shared catalog scenes. Where to go next:

- [Export Stages from Unreal](../simulation/isaac_sim/export_stages_from_unreal.md) —
  photoreal stages from Fab Marketplace environments; the source of the
  [catalog's](../simulation/scenes.md#scene-catalog) guest-readable AirLab
  stages (loading those requires access to the AirLab Nucleus server)
- [Spawning Drones](../simulation/isaac_sim/spawning_drones.md) — custom
  launch scripts: multiple drones, explicit spawn poses, GPS origins, hooks
- [Simulation Scenes](../simulation/scenes.md) — the full catalog reference
  and how `--scene` resolution works
