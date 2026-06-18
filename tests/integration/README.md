# Integration tests (`tests/integration/`)

The **integration** tier sits between **unit** and **system**:

| Tier | Lives in | Brings up | Hardware | Mark |
|------|----------|-----------|----------|------|
| unit | `<pkg>/test/` + proxies in `tests/robot/`, `tests/sim/`, `tests/gcs/` | nothing | none | `unit` |
| **integration** | **`tests/integration/<scenario>/`** | **robot container + a host-side component** | **Docker (no sim/GPU)** | **`integration`** |
| system | `tests/system/` | full sim + robot + GCS | Docker + GPU + sim license | `liveliness`, `sensors`, `takeoff_hover_land` |

An integration test wires a few **real** components together — for example the
robot autonomy stack plus a host-side NatNet server — without using a full simulator or GPU.

**Collection order** (see `pytest_collection_modifyitems` in [`../conftest.py`](../conftest.py)):

## The harness: `robot_autonomy_stack`

The shared bring-up fixture lives in the root [`../conftest.py`](../conftest.py)
(alongside `airstack_env`, matching the repo convention that fixtures are
defined there). Request it instead of hand-rolling `airstack up`:

- **Reuses** an already-running `robot-desktop` container (fast local iteration;
  left running afterward).
- Otherwise brings one up (`airstack up robot-desktop`, autonomy on, no sim)
  and tears it down after.

## Running

```bash
# Reuse a container you already started:
AUTOLAUNCH=false airstack up robot-desktop
pytest tests/integration/ -m integration -v

# One scenario by path:
pytest tests/integration/natnet/ -m integration -v

# Or let the harness bring the container up/down itself:
pytest tests/integration/ -m integration -v

# On CI / a PR, on-demand:
#   /pytest -m integration
```

## Adding a scenario

1. Create `tests/integration/<scenario>/test_*.py`.
2. Set `pytestmark = pytest.mark.integration`.
3. Request the `robot_autonomy_stack` fixture for the container.
4. Filter by path (`tests/integration/<scenario>/`) when you only want that scenario.

## Residents

| Scenario | What it verifies |
|----------|------------------|
| [`natnet/`](natnet/) | Host NatNet emulator → `natnet_ros2` → pose topic Hz |
