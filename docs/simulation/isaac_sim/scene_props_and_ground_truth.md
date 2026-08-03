# Adding Vehicles & Ground-Truth Bounding Boxes

Some Nucleus scenes ship with traffic, some don't. `downtown_edited_v3_818.usd` is one that doesn't — its `prop_car_pillar*` prims are parking-garage pillars, not cars. This page covers borrowing vehicles from another stage into whichever scene you're flying in, and getting ground-truth bounding boxes for them.

Two utility modules do the work:

| Module | Purpose |
|--------|---------|
| [`simulation/isaac-sim/utils/scene_props.py`](../../../simulation/isaac-sim/utils/scene_props.py) | Reference props out of a donor stage, place/scale/label them |
| [`simulation/isaac-sim/utils/gt_annotations.py`](../../../simulation/isaac-sim/utils/gt_annotations.py) | Write an annotation dataset to disk, or publish live boxes on ROS 2 |

Both are wired into [`example_multi_drone_scene_import.py`](../../../simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py) and driven entirely by environment variables, so no code edit is needed for the common cases.

## Quick start

Out of the box the launch script places a **hand-surveyed layout of 7 cars and 4 trucks** on the roads of `downtown_edited_v3_818`. Edit `_DEFAULT_VEHICLE_PLACEMENTS` in [`example_multi_drone_scene_import.py`](../../../simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py) to move them, override per-run with `VEHICLE_PLACEMENTS`, or set `NUM_VEHICLES=0` for no traffic.

Ground snapping defaults to **off** for surveyed layouts (their z is already correct) and **on** for random scatter (which has no way to know the road height). `VEHICLE_SNAP_GROUND` overrides either way.

Six vehicles scattered over a 60 m × 60 m area instead, with a labelled RGB + bounding-box dataset written at 2 Hz:

```bash
NUM_VEHICLES=6 \
VEHICLE_POLY='[[30,30],[-30,-30]]' \
VEHICLE_KINDS=car,truck \
GT_ANNOTATIONS=files \
airstack up isaac-sim
```

The dataset lands in `analysis_runs/gt_annotations/` (gitignored), one folder per robot camera.

## How the vehicles get there

The vehicles in `ModernCityDowntown.stage.usd` are plain USD prims:

```
/Root/BP_MCar01                          Xform, positioned in the donor scene
    └── SM_MCar01_Body                   → SM_MCar01_Body.usd
          ├── SM_MCar01_FL_Door …        → SM_MCar01_*_Door.usd
          └── SM_MCar01_FL_WHL  …        → SM_MCar01_Wheel.usd
```

A **prim-level reference** pulls that whole subtree — body, doors, wheels — into the live stage:

```python
asset = stage.DefinePrim("/World/vehicles/car_0/asset", "Xform")
asset.GetReferences().AddReference(DONOR_URL, "/Root/BP_MCar01")
```

Nothing is copied and nothing on Nucleus is modified. `add_vehicles()` wraps this with the three things that otherwise bite you:

- **The donor's transform rides in on the reference.** `BP_MCar01` sits at (-64.9, -60.1) in ModernCity, so a naive reference lands your car there. Each vehicle is parented under a wrapper Xform whose op order we own, and the donor's ops are cleared on the referenced child.
- **Units differ per stage.** Rather than hard-coding centimetres-vs-metres per donor, the asset's bounding box is measured after referencing and scaled to a target real-world length (`length_m` in the catalog). Placements are always plain world metres.
- **Props are parented at `/World/vehicles`**, a sibling of `/World/stage`, so they never inherit the `STAGE_SCALE` the environment prim carries.

Vehicles also get `CollisionAPI` (so drones can hit them) and any `RigidBodyAPI` in the donor asset disabled (so the construction truck doesn't fall over when the timeline plays).

### The catalog

`scene_props.VEHICLE_CATALOG` maps a short kind to a donor prim:

| Kind | Donor prim | Class | Length |
|------|-----------|-------|--------|
| `car`, `car2` … `car6` | `/Root/BP_MCar01` … `/Root/BP_MCar6` | `car` | 4.6 m |
| `truck`, `truck2` | `/Root/PA_ConstructionTruck01FullyRigged_PhysicsAsset*` | `truck` | 7.5 m |

All six `BP_MCar*` prims reference the same meshes, so they are the same sedan at different donor positions — the variety comes from where *you* put them, not from the mesh.

To find borrowable props in an unfamiliar stage:

```python
from utils.scene_props import list_source_prims
list_source_prims("omniverse://.../SomeOther.stage.usd")   # defaults to vehicle-ish names
list_source_prims(url, pattern=r"bench|streetlight|dumpster")
```

### Placement

`VEHICLE_POLY` scatter reuses the same rejection sampler as drone spawns (`spawn_utils.generate_spawn_configs`), so vehicles land inside the polygon, at least `VEHICLE_MIN_DIST_M` apart, with random headings.

For exact control, give placements explicitly in world metres:

```bash
VEHICLE_PLACEMENTS='[{"kind":"car","x_m":12,"y_m":-4,"yaw_deg":90},
                     {"kind":"truck","x_m":18,"y_m":-4.5,"yaw_deg":90},
                     {"kind":"car3","x_m":25,"y_m":3,"yaw_deg":-90}]'
```

Finding coordinates that land on a road rather than inside a building: the overhead map camera (see [2D World Map in Foxglove](overhead_camera.md)) publishes an aerial of the scene with its coverage and centre, so you can read XY straight off the textured ground in Foxglove's 3D panel. Alternatively open the stage in the Isaac Sim GUI and read the transform of any prim you click.

`z_m` is where the vehicle's *underside* goes, not its origin — so a car surveyed at z = 0.001 rests its wheels a millimetre above the road rather than sinking its origin into it.

If you don't know the road height at a given XY, `VEHICLE_SNAP_GROUND=true` raycasts each vehicle straight down 60 frames after playback starts and drops it onto whatever surface is below. PhysX only knows the scene's colliders once the timeline is running, which is why the snap happens in the run loop rather than at stage setup. It is off by default for surveyed layouts, since it would only move coordinates you already measured.

### Environment variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `NUM_VEHICLES` | unset → the 1 car + 1 truck default | Vehicles to scatter; set explicitly to `0` for no traffic |
| `VEHICLE_POLY` | falls back to `SPAWN_POLY`, then ±30 m | Scatter area, world metres; 2 corners (rect) or ≥3 (polygon) |
| `VEHICLE_PLACEMENTS` | — | Explicit JSON placements; takes priority over `NUM_VEHICLES` |
| `VEHICLE_KINDS` | `car,truck` | Catalog keys, cycled over the placements |
| `VEHICLE_SEED` | fresh each run | Reproducible scatter |
| `VEHICLE_MIN_DIST_M` | `8.0` | Minimum spacing between vehicles |
| `VEHICLE_LIBRARY_URL` | ModernCityDowntown | Donor stage |
| `VEHICLE_GROUND_Z_M` | `0.0` | Ground height used before/without the snap |
| `VEHICLE_SNAP_GROUND` | off for surveyed layouts, on for scatter | Raycast each vehicle onto the surface below it |

## Ground-truth bounding boxes

Boxes come from Isaac's synthetic-data pipeline, which only annotates prims carrying **semantic labels**. `add_vehicles()` labels everything it spawns (`car`, `truck`), so vehicles are annotated automatically. For scenery that was already in the scene:

```python
from utils.scene_props import label_prims_matching
label_prims_matching(stage, r"^BP_MBuilding", "building")
```

A label on an ancestor Xform covers its whole subtree — one box per vehicle, not one per door.

### Files (dataset)

`GT_ANNOTATIONS=files` attaches a Replicator `BasicWriter` to a render product per drone camera:

```
analysis_runs/gt_annotations/
├── robot_1/
│   ├── rgb_0000.png
│   ├── bounding_box_2d_tight_0000.npy
│   ├── bounding_box_2d_tight_labels_0000.json
│   ├── bounding_box_3d_0000.npy
│   └── camera_params_0000.json
└── robot_2/…
```

Capture is **not** on-play: at 60 render FPS a writer would try to serialise every frame for every drone. Instead the sim loop calls `maybe_capture()` and the recorder steps the Replicator orchestrator at `GT_CAPTURE_HZ` with `pause_timeline=False`, so physics is never advanced or paused by the capture.

| Variable | Default | Meaning |
|----------|---------|---------|
| `GT_ANNOTATIONS` | `off` | `off` / `files` / `ros` / `both` |
| `GT_OUTPUT_DIR` | `/isaac-sim/AirStack/analysis_runs/gt_annotations` | Dataset root (falls back to `/tmp` if unwritable) |
| `GT_CAPTURE_HZ` | `2.0` | Captured frames per wall-clock second |
| `GT_WIDTH` / `GT_HEIGHT` | `1280` / `720` | Annotation render resolution |
| `GT_SEMANTIC_SEG` | `false` | Also emit semantic segmentation masks |

### ROS 2 (live)

`GT_ANNOTATIONS=ros` builds a standalone OmniGraph per drone — its own `ROS2Context` on that robot's domain, a render product, and a `ROS2CameraHelper` per box type:

| Topic | Type |
|-------|------|
| `/robot_N/sensors/gt/bbox_2d_tight` | `vision_msgs/Detection2DArray` |
| `/robot_N/sensors/gt/bbox_3d` | `vision_msgs/Detection3DArray` |
| `/robot_N/sensors/gt/bbox_*_labels` | `std_msgs/String` (semantic id → label map) |

Subscribe to the `_labels` topic too — without it the detections carry numeric class ids you can't resolve.

Each mode adds a render product per camera on top of the ZED RGB/depth streams the drone already renders, so expect a frame-rate cost; `both` pays it twice.

## Using the modules directly

Outside the launch-script env-var path:

```python
from utils.scene_prep import get_stage_meters_per_unit
from utils.scene_props import add_vehicles, snap_prims_to_ground
from utils.gt_annotations import GTAnnotationRecorder

mpu, s = get_stage_meters_per_unit(stage)

paths = add_vehicles(
    stage,
    [{"kind": "car",   "x_m": 12.0, "y_m": -4.0, "yaw_deg": 90.0},
     {"kind": "truck", "x_m": 18.0, "y_m": -4.5, "yaw_deg": 90.0}],
    scene_scale_factor=s,
)

recorder = GTAnnotationRecorder(
    [("robot_1", "/World/drone1/base_link/ZEDCamera/base_link/ZED_X/camera_left")],
    output_dir="/isaac-sim/AirStack/analysis_runs/gt_annotations",
    capture_hz=2.0,
)

# ... in the sim loop, after the timeline is playing:
snap_prims_to_ground(stage, paths, scene_scale_factor=s)   # once, ~60 frames in
recorder.maybe_capture()                                   # every iteration
# ... at shutdown:
recorder.close()
```

## Troubleshooting

| Symptom | Cause |
|---------|-------|
| `WARN <name>: empty bounding box` | The donor prim path didn't resolve. Check it with `list_source_prims(donor_url)`. |
| Vehicles are untextured/grey | The donor prim's material bindings point outside the referenced subtree. Reference the mesh USD directly instead (`source_prim=None`, `source_url=".../Meshes/SM_MCar01_Body.usd"`) or borrow a higher ancestor that includes the `Looks` scope. |
| Vehicles float or sink | The raycast found a rooftop or nothing at all. Set `VEHICLE_SNAP_GROUND=false` and give explicit `z_m` values. |
| Boxes are empty in every frame | Nothing in view carries a semantic label. Vehicles added via `add_vehicles` are labelled; anything else needs `label_prims_matching`. |
| Dataset folder is empty | `GT_ANNOTATIONS` wasn't set to `files`/`both`, or the sim was killed before `close()` flushed. |
