# Vehicle types (`config/vehicles/`) — RFC #380 §1

A **vehicle type** is a data-only description of a class of airframe: which
platform flies it, which URDF describes it, and which sensors it carries —
with each sensor's **real driver and sim representation declared in one
entry**, so a sim-generated wiring baseline diffs cleanly against a hardware
bring-up. One directory per type:

```
config/vehicles/<name>/
└── vehicle.yaml        # platform, airframe, sensor suite, sim_asset
```

In-tree types ship with trunk (`quad_default` is the reference); other
vehicles arrive as data modules (`type: data` in RFC #379's module mechanism).

## Type vs. unit

| | Vehicle **type** | Vehicle **unit** |
|---|---|---|
| What | A class of airframe (`quad_default`) | One physical serial number (`SN-0042`) |
| Content | URDF, sensor suite, tuning | Intrinsics/extrinsics **calibration** |
| Lives in | `config/vehicles/<name>/` — shared, committed | `config/local/calibration/<serial>/` — **gitignored, per machine** |
| Changes when | The design changes | The camera gets re-mounted / re-calibrated in the field |

A fleet entry binds the two: `vehicle: quad_default` selects the type,
`unit: SN-0042` binds `config/local/calibration/SN-0042/` (exported to the
container as `CALIBRATION_DIR`). Recalibrating a unit never edits the shared
package — see [`config/local/README.md`](../local/README.md).

## Fields (`vehicle.yaml`)

- `platform:` — the platform class (code layer). `px4_multirotor` is the only
  platform today; platform *modules* are RFC #380 Part 2 (future).
- `airframe.base_urdf:` — **pass-through form**: the existing hand-built URDF,
  package-relative (what `URDF_FILE` carries today). RFC #380 §1's generated
  form (base xacro + `mounts/` extrinsics → URDF) is future work; when it
  lands, `base_xacro:` replaces `base_urdf:` and monolithic URDFs retire.
- `sensors:` — list of `{type, id, frame, driver, sim}` entries. `id` names
  the `sensors/<id>/*` topic namespace (interface conventions); `driver` is
  the hardware-side module, `sim` the Isaac/Pegasus subgraph that stands in
  for it. The fleet spawner reads this list (e.g. any `lidar*` entry enables
  the RTX lidar subgraph — the `ENABLE_LIDAR` env var equivalent).
- `sim_asset:` — the sim asset the spawner instantiates.

Consumed by `tools/fleet/resolve_fleet.py` (URDF/vehicle exports per robot)
and `simulation/isaac-sim/launch_scripts/fleet_spawn.py` (spawn + sensor
toggles). See [docs/development/fleets.md](../../docs/development/fleets.md).
