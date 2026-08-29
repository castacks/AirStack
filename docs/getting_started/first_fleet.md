# Your First Fleet

A ~25 minute lesson. By the end you will have declared a two-robot fleet in
one YAML file, validated it, launched both drones in Isaac Sim, watched them
side by side in Foxglove, and commanded each one independently.

**Prerequisite:** you finished the
[Modular AirStack Walkthrough](modular_airstack.md) — you've flown
`full_default` and know that a fleet file owns *who exists* while the stack
owns *how each robot flies*.

## 1. Write the fleet file

Copy the reference single-robot fleet and grow it to two:

```bash
cp config/fleets/sim_one_default.yaml config/fleets/my_fleet.yaml
```

Edit `config/fleets/my_fleet.yaml` to exactly this:

```yaml
# Fleet: my_fleet — two quad_default robots flying the full_default stack.
defaults:
  vehicle: quad_default
  stack: stacks/full_default

robots:
  robot_1:
    spawn: [-2, 0, 0.07]
  robot_2:
    spawn: [2, 0, 0.07]

sim:
  scene: default

network:
  domain_policy: auto
  gossip_domain: 99
```

**Check:** the file has exactly two entries under `robots:` — both inherit
`defaults:` (same vehicle, same brain: a **homogeneous** fleet), differing
only in spawn position, 4 m apart along X.

## 2. Validate it

```bash
airstack fleet list
python3 tools/fleet/resolve_fleet.py config/fleets/my_fleet.yaml --table
```

**Check:** `fleet list` shows a `my_fleet` row (`ROBOTS 2`, `homogeneous`),
and the resolver prints this table — note `DOMAIN`: robot N → ROS domain N,
its own DDS partition ([how identity resolves](../robot/docker/robot_identity.md)):

```text
ROBOT    DOMAIN  VEHICLE       STACK                ENTRY  HOSTS  SPAWN
robot_1  1       quad_default  stacks/full_default  stack  -      [-2, 0, 0.07]
robot_2  2       quad_default  stacks/full_default  stack  -      [2, 0, 0.07]
```

## 3. Launch the fleet

```bash
airstack up --fleet my_fleet --sim isaac
airstack ready
```

One flag does everything: validates the file, derives `NUM_ROBOTS=2`, stamps
two robot containers, and swaps in the generic fleet spawner
(`fleet_spawn.py`), which reads spawn positions and the scene from your YAML.

**Check:** `airstack status` lists both `airstack-robot-desktop-1` and
`airstack-robot-desktop-2`, and the Isaac Sim viewport shows two drones on
the ground 4 m apart. `airstack ready` reports both flight-ready.

## 4. Observe both in Foxglove

The GCS container renders its layout to match the fleet: Foxglove opens
already showing **AirStack default (2 robots)** — the single-robot template
replicated per robot, no manual import
([how seeding works](../gcs/foxglove.md)).

**Check:** the 3D panel shows both drone meshes in one shared frame, and the
tab strip has a **robot 1** and a **robot 2** tab, each with that robot's own
camera and depth feeds.

## 5. Command each robot

Each per-robot tab contains its **own Robot Tasks panel**, pre-targeted by
the **Robot:** field at the top (`robot_1` in the robot 1 tab, `robot_2` in
robot 2's) — that field is what addresses the goal, sent as a ROS 2 action
onto `/robot_N/tasks/...` and relayed into that robot's DDS domain.

In the **robot 1** tab, open **Takeoff**, keep the defaults
(`target_altitude_m` 10.0, `velocity_m_s` 1.0), click **Send** — then switch
to the **robot 2** tab and do the same. (The Robot: field is editable text,
so any panel can retarget any robot by name.)

**Check:** both drones climb in the 3D panel and settle in a hover; each
panel streams feedback (`3.2 / 10.0 m`) only for its own robot.

## 6. Land and shut down

Land each robot from its tab's **Land** task, then run `airstack down`.

**Check:** `airstack status` shows no running AirStack containers.

## Congratulations

You declared a deployment as one readable file, validated it before spending
a GPU-second, and flew two independently-commanded robots — identity and
placement in the fleet, topology in the stack. Next, one line each:

- Three quads, three different brains:
  `airstack up --fleet sim_three_mixed --sim isaac`
  ([the reference heterogeneous fleet](../development/fleets.md))
- Heterogeneous fleets need generated per-robot services:
  `airstack fleet generate <name>` ([Fleets guide](../development/fleets.md))
- Put a robot's global layer on a ground host — split stacks and `hosts:`:
  [split stacks and bridge.yaml](../development/stacks.md#split-stacks-and-bridgeyaml)
