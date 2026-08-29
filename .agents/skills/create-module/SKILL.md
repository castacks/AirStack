---
name: create-module
description: Create a new AirStack module repository by hand — author the thin module.yaml manifest, lay out colcon packages with canonical-default launch args (never remaps), add test_stack/ and CI, and validate with tools/validate_module.py. Use when packaging a capability (planner, estimator, sim extension, vehicle data) as a standalone module repo per RFC #379.
license: BSD-3-Clause-Clear
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Create an AirStack Module Repo

## When to Use

Creating a **new AirStack module repo** — a thin, standalone repository that
distributes an optional capability (a planner, state estimator, world model, Isaac
Sim extension, vehicle definition, …) without forking AirStack. Defined by
[RFC #379](https://github.com/castacks/AirStack/discussions/379) §2 (manifest) and
[RFC #385](https://github.com/castacks/AirStack/discussions/385) §2 (repo anatomy).

Not this skill: adding a trunk-resident package (use `add-ros2-package`), or wiring
an existing module into a bringup (use `integrate-module-into-layer`).

> **Status:** `airstack module create` scaffolding arrives in a later phase. Until
> then this skill documents the **by-hand procedure**; only the manifest schema and
> validator (`tools/validate_module.py`) exist today.

## Module Repo Anatomy (RFC #385 §2)

A module is a *thin* repo: ordinary colcon packages, a small manifest carrying deps
and identity only, and a `test_stack/` that is both its CI target and its living
install documentation.

```
my-module/
├── module.yaml                  # the thin manifest — deps, identity, tests; NO wiring
├── my_module/                   # ordinary colcon package(s)
│   ├── package.xml              # rosdep keys live here (dep tier 1)
│   ├── src/  config/  test/     # co-located unit tests, standard colcon convention
│   └── launch/
│       └── my_module.launch.xml # topic args DEFAULT to canonical names; never remaps
├── test_stack/                  # reference-stack copy with this module wired in
│   ├── modules.repos            # PINNED to tags/commits — never branches
│   ├── launch/stack.launch.xml  # the ONE place this module is wired
│   ├── docker-compose.yaml
│   ├── wiring.md                # generated in CI from the running graph
│   └── README.md
├── Dockerfile.module            # optional dep tier 2 — against ARG BASE_IMAGE only
├── .github/workflows/
│   └── ci.yml                   # ~10 lines: uses castacks/AirStack/.github/workflows/
│                                #   module-system-tests.yml@vX.Y.Z (later phase)
└── README.md
```

## The Manifest (`module.yaml`)

Schema + full field reference: [`common/module_schema/`](../../../common/module_schema/README.md).
Summary:

- **Required:** `name` (snake_case), `description`, `maintainer` (email),
  `license`, `type` (`isaac_extension` | `ros_package` | `data` | `platform`),
  `airstack_compat` (semver range vs trunk `.env` `VERSION`, e.g.
  `">=0.19.0 <0.21.0"` — never a branch name), `targets`
  (non-empty: `robot` | `gcs` | `isaac-sim` | `ms-airsim`).
- **Optional:** `deps` `{apt, pip}`, `dockerfile` (path ending
  `Dockerfile.module`), `overlay_image`, `compose`, `assets`
  (`[{url (https), sha256, dest}]` — no Git LFS), `docs`, `foxglove`, `hooks`
  (`host_setup`: idempotent, no sudo, writes only inside the module checkout),
  `tests` (`packages` + `marks` from the known mark set).
- **Deliberately absent: wiring.** No slot, role, or topic metadata — unknown keys
  are rejected. A module's interface is its launch file's declared args
  (`ros2 launch <pkg> <file> --show-args`).

Minimal working example: [`tests/fixtures/modules/hello_module/module.yaml`](../../../tests/fixtures/modules/hello_module/module.yaml).

## The Canonical-Defaults Launch Rule

The one interface convention that does the plug-and-play work (RFC #379 §2, §4):

- Expose **every topic endpoint as a launch arg**, and **default it to the
  canonical name** from the interface conventions spec (today:
  `docs/robot/autonomy/integration_checklist.md`).
- **NEVER put `<remap>` in a module launch file**, and never hardcode a topic in
  node code. All cross-module remaps live in the *stack's* entry launch file —
  the single-locus wiring rule.

```xml
<launch>
  <!-- every endpoint is an arg; the default IS the canonical name -->
  <arg name="odom_out" default="/$(env ROBOT_NAME)/odometry"
       description="Output odometry topic" />
  <node pkg="my_module" exec="my_node" name="my_node" output="screen">
    <param from="$(find-pkg-share my_module)/config/my_module.yaml" allow_substs="true" />
    <remap from="odometry" to="$(var odom_out)" />
  </node>
</launch>
```

(The `<remap>` *inside* the node block binds the node's internal name to the
declared arg — that is the mechanism, not a cross-module rewire. What is forbidden
is remapping other modules' topics or overriding canonical names in module launch
files: in a conventional stack, including the module must require **zero** remaps,
so only deviations appear in stack files.)

## Steps (by hand, until `airstack module create` lands)

1. Create the repo with the anatomy above; write `module.yaml` first.
2. Write the package(s) following `add-ros2-package` conventions (package.xml
   format 3, co-located `test/`).
3. Author the module launch file under the canonical-defaults rule.
4. Validate:

   ```bash
   python3 tools/validate_module.py path/to/my-module
   ```

   Exit 0 and `{"valid": true, "errors": []}` on stdout is the gate. Dir mode also
   checks that declared `dockerfile`/`compose`/`hooks`/`docs` paths exist and warns
   when `tests.packages` entries match no directory.
5. Copy a trunk reference stack into `test_stack/` and wire the module in its
   `stack.launch.xml` (reference stacks land in a later phase; until then model it
   on the bringup you tested against).
6. Add `ci.yml` calling the reusable `module-system-tests.yml` workflow (later
   phase) with pinned `airstack_ref` and the marks for your module category
   (RFC #379 §5: global planner → `waypoint_flight`,`autonomy`; state estimator →
   `liveliness`,`sensors`,`takeoff_hover_land`; world model/perception →
   `liveliness`,`sensors`; sim extension → `liveliness`).

## References

- Manifest schema + validator: [`common/module_schema/`](../../../common/module_schema/README.md)
- Fixture module: [`tests/fixtures/modules/hello_module/`](../../../tests/fixtures/modules/hello_module/)
- Contract tests: [`tests/meta/test_module_manifest_contract.py`](../../../tests/meta/test_module_manifest_contract.py)
- RFC #379 (design), RFC #385 (directory atlas)
- Related skills: [add-ros2-package](../add-ros2-package), [write-launch-file](../write-launch-file), [run-system-tests](../run-system-tests)
