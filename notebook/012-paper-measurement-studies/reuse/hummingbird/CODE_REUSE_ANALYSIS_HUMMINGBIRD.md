# Hummingbird: Code-Reuse Analysis vs. Base AirStack

**Question:** how much code did the Hummingbird wire-perching research extension have to write,
and how much of the AirStack base framework did it get to keep unchanged?

**Measured:** 2026-09-01, on branch `rough/workshop_demo`.

---

## 1. The divergence point is unambiguous

| | |
|---|---|
| Base (fork point) | `1c41f8c0` — *"OptiTrack (3/3): Isaac wrapper, mocap EV fusion in sim, and a Circle-trajectory e2e (#376)"*, 2026-08-17 |
| Base is | **exactly the tip of `upstream/develop`** (castacks/AirStack) |
| Extension head | `106dc726` on `rough/workshop_demo` |
| Divergence | **64 commits ahead, 0 behind** |

Because the fork point *is* `upstream/develop`'s tip, the entire delta below is attributable to
the Hummingbird research extension — there is no upstream drift mixed in.

---

## 2. Headline numbers

```
133 files changed, 20,979 insertions(+), 46 deletions(-)
```

| | Files | Lines added | Lines deleted |
|---|---:|---:|---:|
| **New files** (didn't exist upstream) | 104 | 20,442 | 0 |
| **Modified upstream files** | 28 | 537 | 35 |
| **Deleted upstream files** | 1 | 0 | 11 |
| **Total** | **133** | **20,979** | **46** |

Plus one binary asset: `simulation/isaac-sim/assets/robots/hummingbird_grasp.usd` (44.9 MB, not line-countable).

### Denominators

The raw repo line count is dominated by vendored/third-party blobs (269 k lines of `.obj` meshes,
plus `glad`, `stb_image`, bundled `dist/` JS, `xdot`). Both denominators are given:

| Tree | Text files | Raw text lines | Hand-written source lines* |
|---|---:|---:|---:|
| Base AirStack @ `1c41f8c0` | 1,593 | 459,929 | **159,967** |
| Hummingbird @ `106dc726` | 1,696 | 480,862 | **180,845** |

\* excludes 299,962 lines of vendored meshes/third-party headers/generated bundles. None of that
material was touched by this work.

---

## 3. The reuse ratios

| Metric | Value |
|---|---|
| **Lines we authored / final source LOC** | 20,979 / 180,845 = **11.6%** → **88.4% inherited from AirStack unchanged** |
| Same, against raw all-text LOC | 20,979 / 480,862 = 4.4% |
| **Upstream source lines we had to touch** | 572 / 159,967 = **0.36%** |
| **Upstream files we had to touch** | 29 / 1,593 = **1.8%** |
| Even within the 28 files we did edit (5,959 lines total) | only 572 lines = 9.6% of them changed |
| **Deletions across the whole extension** | **46 lines** — the work is essentially purely additive |
| Share of our own diff spent modifying the framework | 572 / 20,979 = **2.7%** |

> **The one-line summary:** adding wire-perching research to AirStack cost ~21 k new lines and
> required rewriting **572 lines (0.36%)** of the 160 k-line base. 97.3% of the diff went into
> new modules that plugged into existing extension points; 2.7% went into bending the framework.

---

## 4. What the 20,442 new lines actually are

| Category | Lines | Share of new code |
|---|---:|---:|
| Runtime code + config (sim models, ROS 2 nodes, env/launch) | 11,646 | 57% |
| Tests | 5,517 | 27% |
| Documentation / agent knowledge (`.md`) | 3,279 | 16% |

By file type:

| Ext | + | − |
|---|---:|---:|
| `.py` | 15,926 | 16 |
| `.md` | 3,330 | 0 |
| `.env` | 725 | 0 |
| `.yaml` | 450 | 7 |
| `.xml` | 279 | 4 |
| `.cfg` / `.action` / `.js` / `.cpp` / other | 269 | 19 |

So the **actual new runtime engineering is ~12.2 k lines** (11.6 k new + 0.5 k modifications) —
about **7.6% the size of the base framework's source.**

---

## 5. Where the new code went

| Area | Files | + | − |
|---|---:|---:|---:|
| `simulation/isaac-sim` (wind, powerline, gripper, vehicle, landing gear, bag replay) | 34 | 12,392 | 1 |
| `robot/global/planners` (`waypoint_planner`) | 19 | 3,055 | 0 |
| `.agents/` (skills + Hummingbird knowledge base) | 13 | 1,863 | 0 |
| `robot/sensors/teensy_serial` | 14 | 836 | 0 |
| `tests/` (system tests) | 6 | 610 | 3 |
| `robot/local/planners` (`perch_task`) | 10 | 591 | 0 |
| `hummingbird_docs/` | 5 | 451 | 0 |
| `overrides/` (`hummingbird-{real,sim,gcs}.env`) | 3 | 390 | 0 |
| root / top-level (`AGENTS.md`, `.env`, …) | 5 | 300 | 2 |
| `gcs/` | 5 | 202 | 29 |
| everything else (`common`, `robot/docker`, bringup, interface, perception) | 19 | 289 | 11 |

**Three entirely new ROS 2 packages** were added and integrated:
`waypoint_planner` (global), `perch_task` (local), `teensy_serial` (sensors).

---

## 6. The framework-modification cost, itemized

All 29 upstream files we touched, with churn vs. their original size:

| + | − | base size | File | Nature |
|---:|---:|---:|---|---|
| 143 | 14 | 475 | `gcs/ros_ws/src/action_relay/action_relay/relay_node.py` | real logic |
| 112 | 1 | 164 | `simulation/isaac-sim/docker/docker-compose.yaml` | config |
| 56 | 1 | 1,575 | `gcs/foxglove_extensions/robot-commands/dist/extension.js` | **generated bundle** |
| 49 | 0 | 478 | `AGENTS.md` | docs |
| 45 | 2 | 715 | `robot/ros_ws/src/interface/mavros_interface/src/mavros_interface.cpp` | real logic |
| 19 | 1 | 11 | `robot/.../sensors_bringup/launch/sensors.launch.xml` | integration glue |
| 16 | 0 | 45 | `robot/docker/robot-base-docker-compose.yaml` | config |
| 14 | 1 | 215 | `robot/.../local_bringup/launch/local.launch.xml` | integration glue |
| 12 | 3 | 23 | `tests/colcon_unit_test_packages.yaml` | test registration |
| 12 | 0 | 24 | `robot/.../global_bringup/launch/global.launch.xml` | integration glue |
| 10 | 2 | 60 | `robot/.../onboard_autonomy_all.launch.xml` | integration glue |
| 7 | 0 | 241 | `robot/docker/.bashrc` | config |
| 7 | 0 | 142 | `robot/.../interface_bringup/launch/interface.launch.py` | integration glue |
| 5 | 2 | 272 | `robot/.../natnet_ros2/launch/natnet_ros2.launch.py` | config |
| 4 | 3 | 101 | `robot/.../natnet_ros2/config/natnet_config.yaml` | config |
| 4 | 0 | 126 | `tests/harness/collection.py` | test harness |
| 3 | 0 | 427 | `robot/docker/Dockerfile.robot` | config |
| 3 | 0 | 9 | `simulation/isaac-sim/docker/px4-params/default.env` | config |
| 2 | 2 | 565 | `gcs/foxglove_extensions/airstack_default.json` | config |
| 2 | 0 | 95 | `docs/README.md` | docs |
| 2 | 2 | 56 | `.env` | config |
| 2 | 0 | 35 | `common/.../task_msgs/CMakeLists.txt` | msg registration |
| 2 | 0 | 18 | `robot/.../local_bringup/package.xml` | dep registration |
| 2 | 0 | 18 | `robot/.../global_bringup/package.xml` | dep registration |
| 1 | 0 | 20 | `robot/.../sensors_bringup/package.xml` | dep registration |
| 1 | 0 | 19 | `tests/pytest.ini` | test registration |
| 1 | 0 | 18 | `simulation/isaac-sim/docker/px4-params/external-vision.env` | config |
| 1 | 1 | 9 | `gcs/foxglove_extensions/robot-commands/package.json` | config |
| 0 | 11 | 10 | `gcs/bags/.gitignore` | housekeeping |

**Read of this table:** only **two** upstream files received genuine algorithmic changes
(`relay_node.py`, `mavros_interface.cpp` — 204 lines combined). Everything else is
config, dependency/test registration, docs, or one generated bundle.

Attaching three brand-new autonomy modules to the running stack cost roughly **55 lines of
launch-file glue** across four bringup files. That is the clearest single measurement of what the
layered bringup + remapping pattern buys.

---

## 7. Caveats

- Line counts are a proxy for effort, not complexity. The 45 lines in `mavros_interface.cpp` (yaw
  slewing) took far longer than 1,450 lines of new wind-field code.
- The 44.9 MB `hummingbird_grasp.usd` asset is excluded from all line counts.
- ~16% of new lines are Markdown (docs + `.agents/` knowledge base). Excluding those,
  authored share of final source drops from 11.6% to 9.9%.
- Not included in the totals: 3 uncommitted working-tree files (+159 / −34).
- "Hand-written source" excludes vendored meshes and third-party headers; the raw figures are
  given alongside so either denominator can be used.

---

## 8. Bottom line for the AirStack team

The base framework absorbed a substantially different research application — wire perching, with a
new gripper sim, wind/powerline physics, a serial sensor, a perch task executor, and a waypoint
planner — while **98.2% of its files and 99.64% of its source lines went untouched**, and with only
46 lines deleted repo-wide. The extension points that carried the load were: layer bringup launch
files with topic remapping, `overrides/*.env`, the PX4 param-slot composition, the task-executor
action pattern, and the Isaac Sim `utils/` module structure. Those are the parts worth protecting
in future refactors.
