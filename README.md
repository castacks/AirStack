# AirStack

<div align="center">
  <img src="docs/assets/logo_horizontal_color.png" alt="AirStack Logo" width="400"/>
</div>

AirStack is a comprehensive, modular autonomy stack for autonomous aerial
robotics, developed by the [AirLab](https://theairlab.org) at Carnegie Mellon
University's Robotics Institute. It provides an end-to-end system for
autonomous drone operations — a layered ROS 2 (Jazzy) autonomy stack,
high-fidelity simulation (NVIDIA Isaac Sim with the Pegasus extension;
Microsoft AirSim legacy), a Ground Control Station, multi-robot coordination,
and hardware deployment tools — all running in Docker.

[![License](https://img.shields.io/github/license/castacks/AirStack)](LICENSE)
[![Documentation](https://img.shields.io/badge/docs-mkdocs-blue)](https://docs.theairlab.org)

## Modular architecture

AirStack is organized around three concepts:
**modules** — thin external repos with a small `module.yaml`, pulled on demand
(`airstack module add <url> --version <pin>`) and discovered through the
[module registry](https://github.com/castacks/airstack-modules-index);
**stacks** — self-contained autonomy topology folders under [`stacks/`](stacks/)
with pinned `modules.repos` and a CI-observed `wiring.md`; and **fleets** —
files under [`config/fleets/`](config/fleets/) declaring who exists, which
vehicle, which stack, and which ground hosts run split-stack offboard halves.

## Quick start

```bash
./airstack.sh setup          # configure AirStack and add `airstack` to PATH
airstack install             # install Docker and dependencies
airstack up --sim isaac      # bring up sim + robot + GCS (default stack: full_default)
```

Then follow the [Getting Started guide](https://docs.theairlab.org/latest/docs/getting_started/)
and the [Modular AirStack Walkthrough](docs/getting_started/modular_airstack.md).

## Documentation

Full documentation lives at **<https://docs.theairlab.org>** (built from
[`docs/`](docs/) with MkDocs — `airstack docs` serves it locally).

## Repository map

- [`robot/`](robot/) — onboard ROS 2 autonomy stack (interface, sensors, perception, local, global, behavior)
- [`stacks/`](stacks/) — reference autonomy stacks (launch topology + pinned modules + wiring baselines)
- [`config/`](config/) — fleets and vehicle definitions
- [`simulation/`](simulation/) — Isaac Sim (Pegasus) and Microsoft AirSim (legacy)
- [`gcs/`](gcs/) — Ground Control Station
- [`common/`](common/) — shared ROS packages and the [`module_schema/`](common/module_schema/) for `module.yaml`
- [`tools/`](tools/) — repo tooling (docs catalog generator, fleet resolver, wiring/DDS generators)
- [`tests/`](tests/) — pytest system tests, integration tests, and contract tests
- [`docs/`](docs/) — MkDocs documentation source

## Community & license

Contributions are welcome — see the [Contributing guide](https://docs.theairlab.org/latest/docs/development/)
and open issues/discussions on GitHub. Contact the AirLab team via
[theairlab.org](https://theairlab.org). Licensed under the terms in
[LICENSE](LICENSE).
