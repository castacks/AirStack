# AirStack — WS2 automated test bench

This branch runs MonoNav, Kim/FCRN and PULP-DroNet v3 through a common
simulator, flight controller and evaluator. Choose where flights execute:

| Mode | Your computer | GPU execution | Start here |
|---|---|---|---|
| Local | Viewer, campaign, optional LLM, Docker/ROS/Isaac and selected model | Local NVIDIA GPU | [Local installation and run](tools/ws2_bench/README.md) |
| OSMO | Viewer, campaign, optional LLM and OSMO CLI | One remote GPU job per flight | [OSMO installation and run](tools/ws2_bench/osmo/README.md) |

Random/Search needs no LLM account. For Claude/Codex, each person signs in with
their own official CLI on the computer running the viewer. Credentials stay
there; they are never copied into task images or flight configurations.
See [agent authentication and configuration](tools/ws2_bench/CORRIDOR_AGENT.md).

Model setup: [MonoNav](https://github.com/engcang/MonoNav),
[Kim/FCRN](https://github.com/engcang/Collision-avoidance),
[PULP-DroNet](https://github.com/engcang/pulp-dronet).
With published OSMO images, colleagues do not need to build the models or
install a local GPU/Isaac/ROS environment.

For other simulation tasks, use [the corridor scenes on their own](tools/ws2_bench/README.md#use-only-the-corridor-scenes).
Export portable USD scenes and textures without installing or running the bench.

Use `tools/ws2_bench/scenarios/corridor_{mononav,kim,pulp}.yaml` for target
settings and `tools/ws2_bench/scenarios/corridor_suite.yaml` for fixed-suite
maps and conditions. OSMO resources/image digests belong in an ignored local
configuration, as shown in the OSMO guide. Runtime results, videos and weights
stay out of Git.

General AirStack documentation: [docs](docs/README.md).
