# Isaac Sim

The primary simulator we support is [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html). 
We chose Isaac Sim as the best balance between photorealism and physics simulation.

Isaac Sim is built on [NVIDIA Omniverse](https://developer.nvidia.com/omniverse), which provides a physically-based rendering engine and accurate rigid-body physics. This combination allows us to create realistic scenes that behave and look close to the real world.

<video controls muted loop playsinline preload="metadata" style="max-width: 100%;">
  <source src="../../assets/media/isaac_sim_demo.mp4" type="video/mp4">
</video>
*Three drones running the full AirStack autonomy stack in Isaac Sim (`airstack up --sim isaac --robots 3 --scene full-warehouse`): parallel `TakeoffTask` actions followed by concurrent Circle and Figure-8 `FixedTrajectoryTask` patterns, viewed from the built-in follow camera (`ISAAC_SIM_FOLLOW_CAM`).*

## Why Isaac Sim

### ROS 2 Integration
Isaac Sim has native support for ROS 2 via the Isaac ROS bridge, enabling seamless communication of sensor topics, transforms, and commands.
Robots simulated in Isaac Sim can publish camera images, LiDAR scans, odometry, and TF data directly to ROS 2 nodes, while subscribing to control topics for motion commands or trajectory execution.
This makes it easy to test the same ROS 2 stack in simulation and later deploy it to real robots with minimal changes.

### Sensor Availability and Realism
Isaac Sim provides a wide range of high-fidelity, GPU-accelerated virtual sensors, including:

- RGB, depth, and segmentation cameras

- Stereo and fisheye cameras

- LiDARs and Radars (see [Pegasus scene setup](pegasus_scene_setup.md#rtx-lidar-near-range) for RTX OmniLidar in AirStack and near-range filtering on the robot stack)

- IMUs, GPS, and odometry sensors

### Custom sensor scripting via OmniGraph or Python
These sensors produce data with realistic noise models and latency, allowing perception pipelines to be validated under realistic conditions.

### Scalability and Reusability
Through USD (Universal Scene Description), Isaac Sim supports modular world composition and scalable scene assembly.
Props, environments, and robots can be managed independently, versioned, and reused across projects.

AirStack leverages this architecture to promote the creation of reusable simulation components—for example, a robot defined in one project can be dropped into another scene without modification.
USD’s layer and reference system also makes it straightforward to build complex environments from smaller, composable files, reducing duplication and simplifying collaboration between teams.

This modularity enables rapid iteration, large-scale simulation generation, and consistent environment definitions across training, testing, and deployment workflows.

Together, these features make Isaac Sim a comprehensive platform for robotics simulation, bridging the gap between visual realism, physical accuracy, and ROS 2-based autonomy stacks.

## Optional bounded physics diagnostics

The existing `rrm_office_visual_eval.py` launcher selects the Office marker fixture
when launched with `ISAAC_SIM_SCENE=Office` and scale 1. With `ISAAC_SIM_TRUTH_DIR`
set, it also writes `office-marker-observation.json`: actual active prim identities,
USD world translations, stage units, observer episode, engine time and receipt time.
Output updates at most once per simulated second for up to 600 samples. Check its
timestamp/sample advancement; the final file becomes stale after observation stops.
This is read-only stage evidence, not camera-frame labels, map alignment, navigation
waypoints, physical feasibility or execution authority. Use a new diagnostic directory
per startup so old observations cannot be mistaken for a new scene epoch.

For fresh camera assessment, the same launcher also enables
`office_camera_teacher.py`. Write `office-camera-request.json` in that directory
with `{"capture_id":"check_01","enabled":true}`. A new ID exclusively creates
`office-camera-check_01/`; one native writer on the existing drone-left render
product retains RGB, semantic pixels, ReferenceTime, the bridge's
`IsaacReadSimulationTime` and camera parameters from the same callback. It stops
after three frames or 15 wall seconds and detaches during
the post-render hook, not inside a physics callback. Disable the request to stop;
stopped IDs cannot resume. `office-camera-status.json` reports lifecycle/errors;
check freshness and actual output, not status alone. Idle capture is detached.
Marker prim identities/poses are separately read during the writer callback, not
historical rendered transforms. Native ReferenceTime and ROS image stamps are
distinct clocks in the observed profile: do not equate or rebase them. Use the
same-render bridge clock plus exact ROS stamp/RGB correspondence, not a fitted
offset. Read-only vehicle physical input is separately retained at writer-callback
phase to diagnose mounting versus estimated-pose error; it is not historical render
state or a map calibration. Native segmentation pairing is assessment-only; map
binding and authority remain separate checks. Load changes through a grounded restart.

Camera diagnostics retain the composed USD attachment chain, physics joint targets
and vehicle body-frame correction. Pegasus reads authored `/body` rotation before
Robot initialization; for an already-simulated OmniGraph vehicle it reads the selected
source USD asset, not a live body tilt. The stereo optical-link translations in the
Pegasus Iris URDF match the pinned Isaac5.1 ZED_X asset. Passing this mount check does
not establish the separate USD-world↔ROS-map origin/estimated-pose binding.

### AirStack-owned rigid-body recorder

`ISAAC_SIM_TRUTH_DIR` enables the recorder in
`simulation/isaac-sim/launch_scripts/physical_truth.py`. Set it to an existing
directory visible inside Isaac. The source belongs to AirStack, so it does not
depend on the unavailable Pegasus diagnostic commit described below. Load it
through a grounded simulator restart; source edits do not update running objects.

Create `<directory>/capture.json` with a fresh ID:

```json
{"capture_id": "grounded_check_01", "enabled": true}
```

The recorder creates `<capture_id>.jsonl` exclusively and polls the request once
per wall second. It samples after world steps at nominal 10 Hz wall time, only
while playing, and caps each capture at 10,000 records, 64 MiB or 300 wall seconds.
Records from all vehicles share these caps. Disable the request to stop; a stopped
ID cannot be resumed, and existing files are never overwritten. Invalid requests,
nonfinite data and request/data I/O failures disable capture without changing control
or physics. `status.json` reports counts, stop/error reason, receipt/wall timestamps
and maximum sampling duration after state transitions. Check its freshness: status
storage failure produces a warning and can leave an old file; it cannot certify
current capture state. Observer initialization and cleanup failures are contained.
That duration excludes request/status-file I/O; synchronous storage can stall.
Measure actual cadence and overhead while grounded before collecting flight evidence.

Schema `airstack-physical-truth/v1` records direct Dynamic Control rigid-body pose,
linear/angular velocity, world simulation time, monotonic receipt and wall time.
The legacy `sensor_state_position_m`/quaternion fields retain Pegasus `vehicle.state`,
the physical-state input to simulated sensors, not their noisy measurements or a
PX4 estimate. Agreement with direct pose checks capture paths only; callback phase
can differ. The earlier "sensor belief" docstring was a mislabel, now corrected. Rigid-body
quaternions include the asset body's rotation, whereas sensor orientation may be
corrected. Record the scene's units/frame conventions before comparing poses.
This recorder has no contact or rotor-force stream and makes no collision or
delivered-actuation claim. It reads state and never issues a control command.

With the current AirStack observer source, records also contain `physics_clock`
and `backend_clocks`. A separate engine physics callback accumulates the actual
callback count, duration sum, min/max and sum of `int(dt * 1e6)`. The callback has
no file I/O or backend writes. After `world.step`, the recorder reads those totals
and each available PX4 backend's HIL microsecond counter, source-file hash, process
instance identity and gate flags. The latter are **post-step snapshots**, not a
trace of the branches taken by every backend update. Source inspection relates
the engine callback argument to backend forwarding; the backend call is not wrapped.

Observer identities and totals reset when the World object changes, and an old
callback is inert after replacement even if removal fails. Registration retries
are contained; attachment/error status is explicit. Reject clock resets, identity
changes, missing or invalid observations before comparing interval deltas. A
callback error remains visible rather than becoming valid timing evidence.
The optional environment setting enables callback accumulation even while no
named capture is active; without it, no observer callback is registered. These
fields need a grounded restart to load. Measure their callback and sampling
cost/cadence while grounded; maximum sampling duration still excludes request and
status I/O. Counter agreement alone does not establish source/binary equivalence,
acquisition-time epoch mapping, transport latency or the cause of a prior flight.


The fraction-preserving PX4 backend candidate adds `utime_remainder_us` to these
snapshots (null with older backend sources). Its integer HIL counter carries the
sub-microsecond remainder between admitted updates instead of truncating every
step. The remainder is per instance and shares the existing counter lifetime;
stop/start/reinitialize and the existing no-op reset preserve both. This correction
does not change heartbeat/IMU gates, lockstep, rates or the timestamp origin.
Compare interval counter delta **plus remainder delta** against callback-duration
sum; integer counter deltas alone retain up to one microsecond of quantization.
Keep backend/observer identity and source hash fixed within an interval. The patch
is in the Pegasus child checkout, so a clean parent checkout alone cannot supply
it until a child commit and parent pin are published. Grounded qualification does
not establish sensor acquisition registration or flight robustness.


### Earlier Pegasus instrumentation

The earlier `ISAAC_SIM_STATE_LOG` contract below requires Pegasus commit
`627ece9128d66d99bd53753092d964e2630a9fb4`, referenced by AirStack `e633a658`.
In the fresh 2026-10-05 workspace, that object was absent and the configured
Pegasus remote refused fetching it. Available checkout `8c7a664` lacks this
instrumentation. The contract below is historical until that source is recovered;
setting its environment variable alone does not establish recording.

`ISAAC_SIM_STATE_LOG` opts Pegasus vehicles into state/contact diagnostics. The
legacy base JSONL file records at most 10,000 samples at nominal 10Hz. Contact
reporting is optional: an inactive subscription or empty report is not proof of
no collision. Collider AABBs are broad-phase bounds, not exact contact geometry.

For a fresh named capture, place a JSON request at
`<ISAAC_SIM_STATE_LOG>.capture.json` inside the simulator filesystem:

```json
{"capture_id": "diagnostic_attempt_01", "enabled": true}
```

The ID must contain 1–64 ASCII letters, digits, underscores or hyphens. The file
is polled once per wall second during state sampling. Each vehicle writes a
separate `<base>.capture-<vehicle-hash>-<capture-id>.jsonl` using exclusive creation:
existing captures are never truncated. Each capture stops at 10,000 samples,
64MiB, or 300 wall seconds, whichever is reached first. Use a **new** ID for every
attempt, including after STOP/pause, errors or a simulator restart. Setting
`enabled` to false or removing the request stops capture at the next poll. Files
remain on disk; manage their retention separately. Request/output I/O and JSON
failures are contained, and legacy-file failure does not disable named captures.

Records distinguish independent rigid-body pose, corrected sensor orientation,
latest HIL receipt/source/backend timestamps, and the previous vehicle update's
modeled rotor forces/rolling torque. The force snapshot includes its own timing
and associated HIL snapshot. `modeled_force_before_apply_api` describes a model
output, **not measured delivered rotor force**. Contacts and snapshots may belong
to different callback phases; compare their timestamps, not just row placement.
Logging is synchronous; output caps do not guarantee bounded I/O latency or zero
simulation overhead. Qualify cadence and overhead while grounded before flight.

Python source edits do not reload existing simulator objects. Load instrumentation
through a planned simulator restart only while grounded/disarmed with no active
mission; restore the intended scene and restart the robot stack after the new
Isaac clock epoch, then verify fresh readiness. A capture request never commands
flight or resets the simulator.

## USD File Naming Conventions
AirStack uses the following file naming conventions:

**Purely 3D graphics**

- `*.prop.usd` ⟵ simply a 3D model with materials, typically encompassing just a single object. Used for individual assets or objects (as mentioned earlier), representing reusable props.

- `*.stage.usd` ⟵ an environment composed of many props, but with no physics, no simulation, no robots. simply scene graphics

**Simulation-ready**

- `*.robot.usd` ⟵ a prop representing a robot plus ROS2 topic and TF publishers, physics, etc.

- `*.scene.usd` ⟵ an environment PLUS physics, simulation, or robots

### Opt-in loop timing for grounded pause diagnosis

With `ISAAC_SIM_TRUTH_DIR` enabled, physical capture records now include
`loop_timing` (`airstack-loop-timing/v1`). Wall-monotonic and current-thread CPU
boundaries measure follow-camera work, observer binding, the unchanged
`world.step(render=True)` call, physical capture, world rebinding, or fallback
`app.update()`. Gaps between loops are measured separately. Engine exceptions
still propagate; observer failures disable only timing observation.

Each 10 Hz physical record distinguishes its current partial loop from the prior
completed loop. Completed slow loops or between-loop gaps above 0.1 s enter a
32-entry history, with cumulative counts and eviction metadata. Slow entries are
streamed after successful writes without consuming the current partial loop.
History delivery is capture-global; per-vehicle history is not guaranteed for
multi-vehicle captures. Physics-observer IDs at boundaries prevent interpreting
callback counts across a world change. A separate bounded physics-callback gap
history reports callback receipt intervals, not ROS publication or acquisition.

Timing brackets around `world_step` include rendering, physics and bridge work.
Current-thread CPU does not measure all engine workers, GPU time or OS scheduling.
These diagnostics localize a broad phase and require grounded GUI checks; they do
not qualify flight robustness or instrumentation overhead. The RRM read-only
control capture also retains raw `/clock` messages as `sim_clock`, separately from
physical callback timing and control receipt ages.


Optional loop timing also retains separate operation spans for metadata lookup,
request polling, body reads, clock and backend metadata, loop snapshot copying,
record encoding/write/flush, and status encoding/write/replace. These spans do not
add outer phases or change engine/control execution. Each loop keeps at most 32
completed spans, with explicit count/drop metadata; cumulative maxima cover
successfully measured completed spans. A span measurement failure disables only
subphase timing, reports its error, and lets the original operation run. Original
operation exceptions retain the existing capture/status containment behavior.

The current physical record snapshots an active `loop_snapshot` span. Its own
encoding/write/status tail is incomplete. `latest_sampling_completed` retains one
latest completed loop containing successfully flushed physical records, separately
from the immediate prior engine loop. Capture/observer identity and successful-write
delivery cursors prevent an old capture or failed containing write from consuming
this evidence. Nonsampling loops do not replace it. Capacity one is qualified for
the single-vehicle stream; arbitrary multi-vehicle queues are not guaranteed.

`sampling_record` identifies capture and first/last physical record sequences.
Final status reports retention availability, errors and the last written versus
last delivered loop. It distinguishes a pending written loop from a known completed
pending loop, without writing an extra final-tail record. Consumers must check
retention state and status freshness. The final successful record normally remains
undelivered because it needs a later containing record.

`max_subphase_records` carries the full completed span setting each cumulative wall
maximum: loop/observer identity, wall timestamps, current-thread CPU and physics
boundary metadata. Strictly larger maxima replace records; ties retain the first.
These maxima apply through the snapshot, including startup and settling; timestamps
permit attribution to comparison windows. Selected brackets exclude observer
bookkeeping and do not measure total observer overhead. Low current-thread CPU
alone cannot identify I/O or scheduling.
