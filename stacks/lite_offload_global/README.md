# `lite_offload_global` — split stack: lite vehicle, offboard global planning

The first **split stack** (RFC #380 §2): a lite vehicle half plus an offboard
global-planning half, with an explicit [`bridge.yaml`](bridge.yaml) listing
everything that crosses the machine boundary. This is the stack-shaped
successor of the legacy `AUTONOMY_ROLE=onboard` / `offboard` pair.

## Anatomy

| File | Runs where | What |
|------|-----------|------|
| `launch/onboard.launch.xml` | the vehicle | [`lite_default`](../lite_default/README.md)'s topology (interface, sensors, perception, flat Local layer, behavior — no global, no logging) + the DDS router configured **from `bridge.yaml`** + gossip |
| `launch/offboard.launch.xml` | the ground host (conventionally the GCS machine, domain 0) | the global layer only: `vdb_mapping` + `random_walk` (the `global_bringup` include, as in `full_default`) |
| `bridge.yaml` | — | **THE boundary document**: every topic/service/action crossing between the halves — name, type, direction, QoS. Feeds DDS-router config generation; readable in source. |

A split is a stack *shape*, not special machinery: same four-file anatomy,
plus one entry file per host role and the bridge list
(`tests/meta/test_stack_layout_contract.py` requires `bridge.yaml` for any
stack with two or more entry points).

## The bridge

`bridge.yaml` is authoritative. Generate the DDS-router config from it (the
onboard entry loads the generated file):

```bash
python3 tools/gen_dds_router.py stacks/lite_offload_global/bridge.yaml
# writes .airstack/generated/dds_router.lite_offload_global.yaml
```

What crosses (seeded from the legacy `onboard_local_offboard_global`
DDS-router allowlist, then curated — the full rationale is in `bridge.yaml`'s
header comments):

- **onboard → offboard:** filtered lidar cloud (`sensors/ouster/point_cloud`)
  and odometry (`odometry_conversion/odometry`) — the global layer's inputs —
  plus the operator camera streams, stereo point cloud, and GPS fix.
- **offboard → onboard:** `global_plan` (the split's whole point), the
  `robot_command` / `set_takeoff_landing_command` services, and
  `tasks/navigate` goals (the offboard `random_walk` is a NavigateTask client
  of the onboard `droan_gl` server). `tasks/exploration` crosses the other
  way (its server moves offboard with `random_walk`).

**What must never cross (doctor hard gate — RFC #379 §4 / #380 §2):**
`control_setpoint` and the trajectory group (`trajectory_override`,
`trajectory_segment_to_add`, `set_trajectory_mode`, `tracking_point`,
`look_ahead` — the `trajectory_controller/*` group). Command authority stays
onboard so link loss leaves the vehicle able to failsafe: **`global_plan`
crosses; trajectory commands don't.** `gen_dds_router.py --check` (run inside
`airstack doctor`) exits 1 naming any violation. Note the legacy allowlist
bridged the `set_trajectory_mode` service to the GCS; this stack deliberately
does not.

## How to run

```bash
# vehicle (or the robot container in sim):
airstack up --stack lite_offload_global:onboard --sim isaac --robots 1

# ground host (offboard container / GCS machine, domain 0):
airstack up --stack lite_offload_global:offboard
```

Generate the router config first (`gen_dds_router.py` command above) — the
onboard entry fails fast if `.airstack/generated/dds_router.lite_offload_global.yaml`
is missing. Check the composition any time with `airstack doctor` and, against
a running system, `airstack doctor --live --stack lite_offload_global`.

## Known limits

- One `wiring.md` per stack, split or not: nodes grouped by host, bridge edges
  drawn as boundary crossings. Not committed yet — bootstrap via the wiring
  snapshot run (both entry points up) or `airstack doctor --snapshot` on a
  real bring-up (committed with an `unverified-in-CI` provenance line).
- Host placement is by convention (offboard = GCS machine, domain 0) until the
  fleet configuration layer (`hosts:` maps, RFC #380 §2–3) lands.
- `modules.repos` pins no external modules yet; `docker-compose.yaml` is a
  stub (trunk compose profiles provide the services).
