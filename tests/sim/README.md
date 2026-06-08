# Simulation-side unit tests

Tests for **simulation components** that are not part of the onboard ROS workspace
(for example an OptiTrack Motive / NatNet emulator, Isaac launch helpers, or
AirSim bridge utilities).

Mark fast, hermetic checks with `@pytest.mark.unit`. Tests that require a GPU,
full sim, or Docker belong in [`tests/system/`](../system/) instead.
Cross-component tests that need the robot container (but not a sim) belong in
[`tests/integration/`](../integration/) (mark: `integration`).

Suggested layout:

| Directory | Purpose |
|-----------|---------|
| `optitrack_natnet_emulator/` | NatNet emulator unit tests (proxy; mark: `unit`) |
