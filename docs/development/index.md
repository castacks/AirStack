# How-to Guides

Task-oriented recipes for working on AirStack. Each guide assumes you've completed [Get AirStack Flying](../getting_started/index.md) and know the [Key Concepts](beginner/key_concepts.md); it gets you through one job without re-teaching the system.

## Guide groups

| Group | Covers |
|---|---|
| [Development Environment](beginner/development_environment.md) | Local setup, [VSCode & debugging](beginner/vscode/vscode_debug.md), [forking](beginner/fork_your_own_project.md), [remote dev on OSMO](../tutorials/airstack_on_osmo.md) |
| [Docker & Builds](beginner/airstack-cli/docker_usage.md) | Pulling/building images, per-container shells, [build profiles](intermediate/docker-build-profiles.md) |
| [Simulation](../simulation/scenes.md) | Scene catalog, [spawning drones](../simulation/isaac_sim/spawning_drones.md), [overhead camera](../simulation/isaac_sim/overhead_camera.md), [Unreal export](../simulation/isaac_sim/export_stages_from_unreal.md) |
| [GCS Operation](../gcs/waypoints_and_geofences.md) | Waypoints & geofences, [Foxglove](../gcs/foxglove.md) |
| [Robot & Field](../real_world/index.md) | Hardware install, HITL, [logging](../robot/logging/index.md), data offloading |
| [Modules & Stacks](../robot/autonomy/integration_checklist.md) | Integrating a module into a stack, [module CI](module_ci.md) |
| [Testing](intermediate/testing/index.md) | [Unit tests](intermediate/testing/unit_testing.md), [end-to-end benchmarks](intermediate/testing/end_to_end_testing.md) |
| [Contributing](intermediate/contributing.md) | Branching & releases, [docs](intermediate/documentation.md), [feature notebook](intermediate/feature_notebook.md), [extending the CLI](advanced/airstack-cli/extending.md) |

## Quick command reference

The full command and flag tables live in the [CLI Reference](beginner/airstack-cli/index.md). The ones you'll use constantly:

```bash
airstack up --sim isaac --robots 1     # launch sim + robot + GCS
airstack status                        # container status
airstack connect robot-desktop         # tmux into a container
airstack test -m unit -v               # run unit tests
airstack docs                          # serve these docs locally
airstack down                          # stop everything
```

Looking for *why* things are built this way? See **Concepts**: [Key Concepts](beginner/key_concepts.md), [System Architecture](../robot/autonomy/system_architecture.md), and the [Modular AirStack design pages](modules.md).
