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
