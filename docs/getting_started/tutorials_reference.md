# What's Next?

You've flown the stack — here's where to go next, organized by what you're trying to do. The docs are arranged in four kinds: **Tutorials** (guided lessons), **How-to Guides** (task recipes), **Reference** (look-up tables and specs), and **Concepts** (how and why the system is built this way).

## Continue learning

The tutorial sequence, in order:

1. [Fly a Mission from the GCS](fly_a_mission.md) — take off, place waypoints, run a Navigate task, land, save your mission.
2. [Change a Parameter](change_a_parameter.md) — the edit → relaunch → observe development loop in ten minutes.
3. [Modular AirStack Walkthrough](modular_airstack.md) — fly a reference stack, read its wiring, add a module, make your own stack, launch a fleet.
4. [Write Your First Module](first_module.md) — scaffold a module, implement a node, wire it into your own stack, see it in the wiring.
5. [Your First Fleet](first_fleet.md) — a two-robot fleet file, both drones in Foxglove, tasks to each.
6. [Build and Fly Your Own Scene](build_your_own_scene.md) — author a stage, register it in the scene catalog, fly it.
7. [Deploy to Hardware](../real_world/deploying_to_hardware.md) — from bench-ready Jetson to first-flight checks.

## By goal

| I want to… | Go to | Kind |
|---|---|---|
| Set up my dev environment (local or remote) | [Development Environment Setup](../development/beginner/development_environment.md) · [Remote Development on OSMO](../tutorials/airstack_on_osmo.md) | How-to |
| Understand the big picture first | [Key Concepts](../development/beginner/key_concepts.md) · [System Architecture](../robot/autonomy/system_architecture.md) | Concepts |
| Build my own algorithm module | [Integration Checklist](../robot/autonomy/integration_checklist.md) · [Modules](../development/modules.md) | How-to / Concepts |
| Customize the simulation | [Scenes](../simulation/scenes.md) · [Spawning Drones](../simulation/isaac_sim/spawning_drones.md) | How-to |
| Operate the GCS | [Waypoints & Geofences](../gcs/waypoints_and_geofences.md) · [Foxglove Visualization](../gcs/foxglove.md) | How-to |
| Run more robots | [Fleets](../development/fleets.md) · [Robot Identity](../robot/docker/robot_identity.md) | Concepts |
| Deploy to real hardware | [Robot & Field overview](../real_world/index.md) | How-to |
| Look up a command, topic, or config value | [CLI Reference](../development/beginner/airstack-cli/index.md) · [Interface Conventions Spec](../robot/autonomy/interface_conventions.md) | Reference |
| Run or write tests | [Testing Overview](../development/intermediate/testing/index.md) | How-to |
| Contribute changes back | [Contributing Guide](../development/intermediate/contributing.md) | How-to |

## Suggested paths

- **Simulation user**: Get AirStack Flying → Scenes → Spawning Drones → Waypoints & Geofences.
- **Module developer**: Modular AirStack Walkthrough → Key Concepts → Integration Checklist → Interface Conventions Spec.
- **Robot deployer**: Key Concepts → Robot Identity → Robot & Field overview → Deployment Topologies.
- **Contributor**: Development Environment Setup → Testing Overview → Contributing Guide.
