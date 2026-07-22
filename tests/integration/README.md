# Integration tests (`tests/integration/`)

Cross-component tests (`@pytest.mark.integration`) that wire a few **real** components
together — the robot autonomy container plus a host-side component — **without** a
simulator or GPU. They sit between the hermetic unit tests and the full sim-based system
tests: heavier than a unit test (they need the `robot-desktop` image + a running
container), lighter than a system test (no sim license, no GPU).

## The `robot_autonomy_stack` fixture

Defined in [`../conftest.py`](../conftest.py). Module-scoped. It:

- reuses an already-running `robot-desktop` container when one is present (fast local
  iteration — left running afterward), otherwise runs `airstack up robot-desktop` with
  `AUTOLAUNCH=true NUM_ROBOTS=1 COMPOSE_PROFILES=desktop` and tears it down after the
  module (same behavior as the `build_packages` fixture);
- **skips** cleanly when the `robot-desktop` image isn't built locally.

It yields `{"container": <name>, "brought_up": bool}`.

## Collection order

Integration runs after `build_docker` / `build_packages` (it needs the image + a colcon
build) and before the sim tiers — see `_MODULE_ORDER` in `conftest.py`.

## Running

```bash
airstack test -m integration -v
```

## Adding an integration test

Create `tests/integration/<area>/test_*.py`, request the `robot_autonomy_stack` fixture,
and mark the module `pytestmark = pytest.mark.integration`. Keep it sim-free and
GPU-free — anything needing a simulator belongs in `tests/system/`.
