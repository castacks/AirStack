# AirStack Fleets

A **fleet file** (`config/fleets/*.yaml`) declares a whole deployment in one
readable document: who exists, which body each robot flies (vehicle), which
brain it runs (stack), and which ground hosts run each split stack's offboard
half. Fleets exist to separate *who exists* from *how each robot flies* —
the stack owns topology, the fleet owns identity and placement.
The same file drives simulation (spawn positions, scene) and hardware
(identity, placement) — sim vs. real is a deployment mode of one artifact.

**Everything here is opt-in.** No `--fleet` flag and no `FLEET_CONFIG_FILE`
env var ⇒ the replica-based multi-robot configuration
(`NUM_ROBOTS` + `robot_name_map` + `ISAAC_SIM_SCRIPT_NAME`) runs instead —
it remains the default and fully supported.

## The hierarchy

```text
platform class  →  vehicle type   →  vehicle unit      →  robot instance  →  fleet
(code: px4_     (data: config/    (calibration:        (one entry in      (config/
 multirotor)     vehicles/<n>/)    config/local/         a fleet file)      fleets/<f>.yaml)
                                   calibration/<SN>/)
```

- **Platform class** — code: interface, controller, safety behaviors.
  `px4_multirotor` is the only platform today; platform *modules* are
  future work.
- **Vehicle type** — data: `config/vehicles/<name>/vehicle.yaml` (URDF,
  sensor suite with each sensor's real driver + sim representation declared
  together, sim asset). See
  [config/vehicles/README.md](../../config/vehicles/README.md).
- **Vehicle unit** — one serial number's calibration, in the gitignored
  `config/local/calibration/<serial>/` overlay; a fleet entry binds it with
  `unit:` (exported as `CALIBRATION_DIR`).
- **Robot instance** — the composition *vehicle × stack × unit*, one entry in
  a fleet file.
- **Fleet** — all robots + named ground hosts + network policy + sim scene.

## File tour

`config/fleets/sim_one_default.yaml` — today's default checkout as a fleet
(one `quad_default` on `full_default`; resolves identically to the
replica-based default — a contract test pins the parity):

```yaml
defaults: {vehicle: quad_default, stack: stacks/full_default}
robots:
  robot_1: {spawn: [0, 0, 0.07]}
sim: {scene: default}
network: {domain_policy: auto, gossip_domain: 99}
```

`config/fleets/sim_three_mixed.yaml` — the reference **heterogeneous** fleet:

```yaml
defaults: {vehicle: quad_default, stack: stacks/full_default}
robots:
  robot_1: {spawn: [-2, 0, 0.07]}                    # full_default (defaults)
  robot_2: {stack: stacks/lite_default, spawn: [0, 0, 0.07]}
  robot_3:
    stack: stacks/lite_offload_global                # a SPLIT stack
    hosts: {offboard: gcs}                           # placement of its offboard half
    spawn: [2, 0, 0.07]
ground:
  gcs: {}                                            # named ground host
sim: {scene: default}
network: {domain_policy: auto, gossip_domain: 99, gcs_domain: 0}
```

Per-robot keys: `vehicle`, `stack`, `unit`, `spawn` (sim-only, `[x, y, z]` m),
`hosts` (split placement), `overrides` (leaf values only — accepted by the
schema, launch-time application is future work). `defaults:` keeps
homogeneous fleets terse. A robot needing different *topology* points at a
different stack folder — that division keeps fleet files skimmable.

A fleet's `stack:` values resolve first as checkout paths (`stacks/...`),
then as `<alias>/<stack>` against external stack repos declared in
`airstack.yaml` and fetched by `airstack sync` into gitignored
`stacks/.external/<alias>/`.

## Running a fleet

```bash
airstack fleet list                              # what exists, robots, shape
airstack up --fleet sim_one_default --sim isaac  # homogeneous: replicas
airstack up --fleet sim_three_mixed --sim isaac  # heterogeneous: generated services
airstack ready
```

`--fleet <name>` (or an explicit `FLEET_CONFIG_FILE` env var, which the test
harness uses):

1. **validates** the fleet file (named errors: unknown stack/vehicle, a
   `hosts:` role with no matching entry point, a host naming no `ground:`
   entry, a split stack used without `hosts:`, bad `spawn`, unknown keys);
2. exports **`FLEET_CONFIG_FILE`** (the container path —
   `config/` is bind-mounted read-only at `/root/AirStack/config`);
3. derives **`NUM_ROBOTS`** from the robot count (leaf-value precedence: an
   explicitly set env `NUM_ROBOTS` still wins, with an override banner);
4. on Isaac, switches an untouched-default `ISAAC_SIM_SCRIPT_NAME` to the
   generic **fleet spawner** (`fleet_spawn.py`) — an explicit env value wins;
5. for **heterogeneous** fleets, regenerates
   `.airstack/generated/docker-compose.fleet.yaml` and includes it, swapping
   the `desktop` profile for the generated services' `fleet` profile.

The effective-config dump gains a `FLEET_CONFIG_FILE=` line and a resolved
robot table (robot, domain, vehicle, stack, entry, hosts, spawn) — only when
a fleet is selected.

## How a container resolves its identity (opt-in mechanics)

`robot/docker/.bashrc` branches on `FLEET_CONFIG_FILE`:

- **Set** → `tools/fleet/resolve_fleet.py` resolves the container's whole
  fleet entry and exports `ROBOT_NAME`, `ROS_DOMAIN_ID`,
  `AIRSTACK_STACK_DIR`, `AIRSTACK_STACK_ENTRY`, `URDF_FILE`, `VEHICLE`,
  `CALIBRATION_DIR`. Identity comes from the container name / hostname
  (exact robot key, else the trailing replica index — the same convention as
  the `robot_name_map` resolver). Pre-set env still wins per variable: an
  explicit `ROBOT_NAME` skips resolution entirely (heterogeneous-fleet
  services set it explicitly), and non-empty `ROS_DOMAIN_ID` /
  `AIRSTACK_STACK_DIR` / `URDF_FILE` keep their values. Resolution failure
  warns and falls back to the `robot_name_map` resolver.
- **Unset/empty** → the `robot_name_map` resolver runs, exactly as with no
  fleet.

`network.domain_policy: auto` (the only implemented policy) assigns robot N
(1-based file order) → domain N — the same rule the `robot_name_map`
resolver applies for `robot_1..robot_N` fleets.

## Homogeneous vs. heterogeneous

`deploy.replicas` can only stamp **identical** containers, so:

| Fleet shape | Mechanism |
|---|---|
| Homogeneous (same vehicle + stack everywhere, no `hosts:`, no `ground:`) | `deploy.replicas` (`NUM_ROBOTS` derived); each replica resolves itself via `FLEET_CONFIG_FILE`. `airstack fleet generate` detects this and writes nothing. |
| Heterogeneous | `airstack fleet generate <fleet>` → `.airstack/generated/docker-compose.fleet.yaml`: one **self-contained** service per robot (extending nothing; explicit `ROBOT_NAME` / `ROS_DOMAIN_ID` / `AIRSTACK_STACK_DIR` / `AIRSTACK_STACK_ENTRY` / `FLEET_CONFIG_FILE` env) plus one per (ground host × offboard tenant), all under the `fleet` compose profile. `airstack up --fleet` regenerates and includes it automatically. |

## Split placement (`hosts:`)

A **split is a stack shape** ([stacks guide](stacks.md#split-stacks-and-bridgeyaml)):
multiple launch entry points plus a `bridge.yaml`. The fleet decides *where
each half runs*:

```yaml
robot_3:
  stack: stacks/lite_offload_global
  hosts: {offboard: gcs}
ground:
  gcs: {}
```

- `robot_3`'s container gets `AIRSTACK_STACK_ENTRY=onboard` (a robot with
  `hosts:` runs the onboard entry point).
- The ground host `gcs` gets a generated service (`gcs-robot_3`) running the
  **same stack** with `AIRSTACK_STACK_ENTRY=offboard`, `ROBOT_NAME=robot_3`
  (the tenant it serves), and `ROS_DOMAIN_ID` = the fleet's `gcs_domain`
  (default 0) — the same shape as the `robot-offboard` compose service.
- Every `hosts:` role must match an entry-point launch file of the robot's
  stack, and every named host must exist under `ground:` — both are named
  validation errors. Doctor's bridge hard-gate (no control-setpoint /
  trajectory-group names in any `bridge.yaml`) holds unchanged for every
  split stack a fleet places.
- There is no separate role variable: the entry point *is* the role.

## Simulation

- **Isaac Sim** — `simulation/isaac-sim/launch_scripts/fleet_spawn.py`
  replaces the hardcoded one-/multi-drone example scripts when a fleet is
  selected: spawn positions and per-vehicle sensor toggles (any `lidar*`
  sensor in the vehicle manifest enables the RTX lidar subgraph — the
  per-vehicle `ENABLE_LIDAR` equivalent) come from the fleet file; the scene
  comes from `sim.scene` (`default` = the Pegasus "Default Environment", a
  `SIMULATION_ENVIRONMENTS` key, or a `.usd` path). Vehicles are pass-through
  today: every `px4_multirotor` spawns the Pegasus Iris asset.
- **ms-airsim** — `generate_settings.py` already consumes `NUM_ROBOTS`, which
  the fleet derives; nothing more is needed (spawn spacing stays the
  simulator's `AIRSIM_SPAWN_SPACING` grid for now).
- **Test harness** — `airstack test ... --fleet <name>` passes
  `FLEET_CONFIG_FILE` + the derived `NUM_ROBOTS` through `airstack_env`
  (Isaac runs pin `fleet_spawn.py`). Without `--fleet`, `--num-robots`
  behaves exactly as before.

## Top level: `airstack.yaml` + `airstack sync`

`airstack.yaml` (checked in, hand-edited) answers "what does this checkout
run?": `release` (informational until registry-backed release sets land),
`fleet`, `sim`, `modules` (additions beyond the pins — `{path: ...}` or
`{repo: ..., version: <pin>}`), and `stacks` (external stack repo aliases).
`airstack sync` reads it: upserts module additions into `modules.repos`
(naming every deviation), runs the module sync, fetches external stack repos
into `stacks/.external/<alias>/` (pinned refs only), validates the declared
fleet, and records the result in
`.airstack/generated/effective_sources.yaml`.

Deliberately **not** done yet (future work): generating `.env` (it stays
hand-edited; `airstack.yaml` layers on top and never rewrites it), resolving
bare `{version: ...}` module pins against a registry, and deriving the
launch-time fleet default from `airstack.yaml` (select fleets explicitly with
`--fleet`).

## Environment variables under a fleet

| Env var | Under a fleet | Precedence |
|---|---|---|
| `NUM_ROBOTS` | **Derived** from the fleet's robot count | explicit env still wins (banner) |
| `ROBOT_NAME_MAP_CONFIG_FILE` | Absorbed: identity resolves from the fleet entry | the `robot_name_map` resolver remains the no-fleet default |
| `AUTONOMY_ROLE` | Not a launch input — the stack entry point *is* the role, derived from `hosts:` | no-fleet/no-stack default is `stacks/full_default`; a set `AUTONOMY_ROLE` is a preflight error |
| `URDF_FILE` | From the vehicle's `airframe.base_urdf` (pass-through form; xacro generation is future) | explicit env still wins |
| `ISAAC_SIM_SCRIPT_NAME` | **Derived**: the generic fleet spawner | explicit env still wins |
| `ROBOT_NAME` / `ROS_DOMAIN_ID` | Resolved per robot (`domain_policy: auto` = robot N → domain N) | pre-set env still wins |
| `VERSION` | Unaffected by fleets (`release:` is informational until the registry lands) | `.env` stays hand-edited |

## CLI reference

```bash
airstack fleet list                # fleets: robots, vehicles, stacks, shape
airstack fleet generate <fleet>    # per-robot compose for heterogeneous fleets
airstack up --fleet <name> [...]   # validate + export + (re)generate + up
airstack sync                      # airstack.yaml → modules, external stacks,
                                   # fleet validation, effective_sources.yaml
python3 tools/fleet/resolve_fleet.py config/fleets/<f>.yaml --table   # inspect
```

Contract tests: `tests/meta/test_fleet_contract.py` (resolver parity with the
`robot_name_map` resolver, generation determinism, split placement, the
bridge hard-gate, precedence, named schema errors).
