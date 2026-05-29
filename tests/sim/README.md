# Simulation-side unit tests

Tests for **simulation components** that are not part of the onboard ROS workspace
(for example an OptiTrack Motive / NatNet emulator, Isaac launch helpers, or
AirSim bridge utilities).

Mark fast, hermetic checks with `@pytest.mark.unit`. Tests that require a GPU,
full sim, or Docker belong in [`tests/system/`](../system/) instead.

Suggested layout:

| Directory | Purpose |
|-----------|---------|
| `motive_emulator/` | Motive / NatNet protocol emulation / parsing |
