# GPS Degradation — Laptop ↔ VM Bridge

This file is the single source of truth for work split between the personal
laptop (code authoring) and the VM (Isaac Sim runtime testing). Update it
in both directions: when the laptop adds files, document them here; when the
VM discovers something (variable names, API behaviour, scene state), write it
back here.

---

## Current status (last updated: laptop)

### Done on laptop
- All 10 Python package files written to `simulation/isaac-sim/gps_degradation_staging/`
- Two bugs from the design review were fixed before writing (not carried forward from the spec):
  - **Issue A fixed** (`signal_model.py`): fully NLOS-blocked satellites are now dropped
    immediately before C/N0 is computed, instead of flowing through the filter with an
    incorrect (too-optimistic) signal level.
  - **Issue B fixed** (`visibility.py` + `config.py`): multipath extra path length is now
    capped at `multipath_max_extra_m = 300 m` (new config field). Previously uncapped,
    which could inject a 50 km pseudorange bias from a distant hillside hit.
- `state_machine.py`: `import math` moved to file top (was inside a method in the spec).
- Bridge file written with all VM tasks, verification checkpoints, and fill-in tables.

### Blocked — needs VM
- **Hook into `px4_mavlink_backend.py`** (the actual integration): cannot be written
  until variable names are read from the file on the VM. See Part 1.3 below.
- All verification checkpoints (Parts 5.1–5.6): require Isaac Sim to run.

### Next action
Go to the VM, work through Part 1 in order (prerequisites), fill in the table in
Part 1.3, and report the variable names back here. The hook can then be written
from the laptop in one short session.

---

**Goal**: Full GPS realism simulation — not just MACVIO fallback triggers.
The full pipeline (constellation → PhysX raycasting → DOP → pseudorange WLS
→ signal quality / fading → state machine → HIL_GPS hook) is in scope.

---

## Part 1 — VM prerequisites (run these first, in order)

### 1.1 Check Isaac Sim version

```bash
cat /isaac-sim/VERSION
# or:
cat /isaac-sim/apps/isaacsim.exp.base.kit | grep "version"
```

**Write the version back here**: `Isaac Sim version: ___________`

The PhysX raycasting API (`get_physx_scene_query_interface().raycast_closest`)
is stable across 4.x — no changes expected. If the version is 2023.x or older,
flag it; there may be minor import path differences.

---

### 1.2 Initialize the PegasusSimulator submodule

The submodule is registered in `.gitmodules` but not checked out. From the
AirStack repo root on the VM:

```bash
git submodule update --init simulation/isaac-sim/extensions/PegasusSimulator
```

Verify it populated:

```bash
ls simulation/isaac-sim/extensions/PegasusSimulator/extensions/
# Should show: pegasus.simulator/
```

**Do not run this on the laptop** — the submodule is only useful where
Isaac Sim is installed.

---

### 1.3 Locate px4_mavlink_backend.py and extract variable names

```bash
find simulation/isaac-sim/extensions/PegasusSimulator \
  -name "px4_mavlink_backend.py"
```

Open that file and find the block that calls `hil_gps_send` (or equivalent).
Look for any of these patterns:

```bash
grep -n "hil_gps_send\|HIL_GPS\|mavlink_msg_hil_gps" \
  <path to px4_mavlink_backend.py>
```

**Write back here — the actual variable names used just before that call:**

| Spec name | Actual variable name in the file | Encoding |
|---|---|---|
| `lat` | ___ | degrees × 1e7 (int32) |
| `lon` | ___ | degrees × 1e7 (int32) |
| `alt` | ___ | millimetres (int32) |
| `eph` | ___ | cm-int (uint16) |
| `epv` | ___ | cm-int (uint16) |
| `fix_type` | ___ | int |
| `satellites_visible` | ___ | int |
| drone world-pos | ___ | Isaac Sim (x,y,z) metres |

Also note: what is the per-step callback method called?
(`update()`, `_update()`, `apply()`, something else?)

**Write back**: per-step method name: `___________`

And: how is drone position accessed inside that method?
(e.g. `self._state.position`, `state["position"]`, `vehicle.get_world_pose()`)

**Write back**: world position accessor: `___________`

---

### 1.4 Install pymap3d into ISAACSIM_PYTHON

```bash
ISAACSIM_PYTHON -m pip install pymap3d --quiet
ISAACSIM_PYTHON -c "import pymap3d; print('pymap3d ok')"
```

If `ISAACSIM_PYTHON` is not on PATH, the typical location is:

```bash
/isaac-sim/python.sh -m pip install pymap3d --quiet
/isaac-sim/python.sh -c "import pymap3d; print('pymap3d ok')"
```

**Write back**: which command works: `___________`
**Write back**: pymap3d install succeeded: yes / no

---

### 1.5 Verify numpy (should always be present)

```bash
ISAACSIM_PYTHON -c "import numpy; print(numpy.__version__)"
```

---

## Part 2 — Verify collision meshes in the test scene

The PhysX raycaster only hits geometry with `PhysicsCollisionAPI` applied.
Run this inside ISAACSIM_PYTHON **after loading a scene**:

```python
from pxr import UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()
collision_prims = [
    str(p.GetPath())
    for p in stage.Traverse()
    if p.HasAPI(UsdPhysics.CollisionAPI)
]
print(f"{len(collision_prims)} prims with PhysicsCollisionAPI")
print("First 10:", collision_prims[:10])
```

**Write back**: how many prims have collision enabled: `___`
**Write back**: are any of those prims buildings/terrain (not just the drone): yes / no

### If buildings have no collision meshes

Apply them with this one-time scene-prep call (add to the scene launch script,
not the GPS code itself — run once at scene load, not per-step):

```python
import omni.kit.commands
from pxr import UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()
n_applied = 0
for prim in stage.Traverse():
    # Skip drone prims — only apply to static environment geometry
    prim_path = str(prim.GetPath())
    if any(skip in prim_path.lower() for skip in ["drone", "uav", "iris", "vehicle", "robot"]):
        continue
    if prim.GetTypeName() == "Mesh":
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
            n_applied += 1
print(f"Applied PhysicsCollisionAPI to {n_applied} mesh prims")
```

**Note**: This adds physics collision to all non-drone meshes, which also
affects the drone's collision avoidance and physics stepping cost. For large
detailed scenes with thousands of mesh prims, use `approximationShape="boundingBox"`
to keep it lightweight:

```python
from pxr import UsdPhysics, Gf

mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
mesh_collision.GetApproximationAttr().Set("boundingBox")
```

---

## Part 3 — Where the gps_degradation package lives

### Target path (after submodule init)

```
simulation/isaac-sim/extensions/PegasusSimulator/
  extensions/pegasus.simulator/
    pegasus/simulator/logic/
      gps_degradation/          ← CREATE THIS DIRECTORY
        __init__.py
        config.py
        constellation.py
        visibility.py
        dop.py
        pseudorange.py
        signal_model.py
        state_machine.py
        composer.py             ← not in current spec; may be added
        ros2_publisher.py
        model.py
```

The hook (`import` + init + GPS intercept block) goes into:

```
simulation/isaac-sim/extensions/PegasusSimulator/
  extensions/pegasus.simulator/
    pegasus/simulator/logic/
      backends/
        px4_mavlink_backend.py  ← MODIFY THIS FILE (do not replace)
```

### How files will arrive on the VM

The `gps_degradation/` package files are authored on the laptop and committed
to this repo at a staging path (TBD — see Part 4 below). Once the submodule
is initialized, copy them into the target path above.

The staging path used during development is:

```
simulation/isaac-sim/gps_degradation_staging/
```

(This directory will exist in the main AirStack repo, not inside the submodule,
so it can be committed and synced to the VM without needing submodule write access.)

---

## Part 4 — Staged files (written from laptop, ready to copy to VM)

Files listed here are complete and ready to be copied to the target path in
Part 3 once the submodule is initialized.

| File | Status | Notes |
|---|---|---|
| `config.py` | **done** | Added `multipath_max_extra_m = 300.0` field |
| `constellation.py` | **done** | Walker-delta(24:6:1), pymap3d for az/el |
| `visibility.py` | **done** | Multipath cap fix applied (Issue B) |
| `dop.py` | **done** | LOS-only geometry matrix |
| `pseudorange.py` | **done** | WLS with elevation-dependent weighting |
| `signal_model.py` | **done** | NLOS-blocked early-exit fix applied (Issue A) |
| `state_machine.py` | **done** | `import math` moved to top-level |
| `ros2_publisher.py` | **done** | |
| `model.py` | **done** | Hook usage shown in docstring |
| `__init__.py` | **done** | |
| hook into `px4_mavlink_backend.py` | blocked — needs variable names from VM (Part 1.3) | |

**Copy command (run on VM after Part 1.2 and once staging dir is populated):**

```bash
PEGASUS_LOGIC=simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic

cp -r simulation/isaac-sim/gps_degradation_staging/ \
      $PEGASUS_LOGIC/gps_degradation/

echo "Files copied:"
ls $PEGASUS_LOGIC/gps_degradation/
```

---

## Part 5 — Verification checkpoints (run on VM after copy)

These must pass in order. Do not skip ahead.

### 5.1 pymap3d import

```bash
ISAACSIM_PYTHON -c "
from pegasus.simulator.logic.gps_degradation import GpsDegradationModel
m = GpsDegradationModel(vehicle_id=0)
print('import ok')
"
```

Expected: `import ok`

---

### 5.2 Constellation smoke test

```bash
ISAACSIM_PYTHON -c "
from pegasus.simulator.logic.gps_degradation.constellation import ConstellationModel
from pegasus.simulator.logic.gps_degradation.config import GpsDegradationConfig
c = ConstellationModel(GpsDegradationConfig())
c.update(0.0)
sats = c.get_visible_satellites(40.4, -79.9, 100.0)
print(f'{len(sats)} satellites visible')
assert 4 <= len(sats) <= 24, f'unexpected count: {len(sats)}'
print('constellation ok')
"
```

Expected: 6–12 satellites visible, `constellation ok`

**Write back**: actual satellite count observed: `___`

---

### 5.3 DOP sanity check

```bash
ISAACSIM_PYTHON -c "
import math, numpy as np
from pegasus.simulator.logic.gps_degradation.dop import compute_dop
from pegasus.simulator.logic.gps_degradation.visibility import SatelliteVisibility
from pegasus.simulator.logic.gps_degradation.constellation import SatelliteView

def make_sv(az_deg, el_deg):
    az_r, el_r = math.radians(az_deg), math.radians(el_deg)
    e = math.cos(el_r)*math.sin(az_r)
    n = math.cos(el_r)*math.cos(az_r)
    u = math.sin(el_r)
    sv = SatelliteView(0, el_deg, az_deg, 0, 0, 0, np.array([e, n, u]))
    return SatelliteVisibility(sat=sv, is_los=True)

vis = [make_sv(az*60, 30) for az in range(6)]
vis.append(make_sv(0, 80))
h, v, p = compute_dop(vis)
print(f'HDOP={h:.2f} VDOP={v:.2f} PDOP={p:.2f}')
assert 0.5 < h < 3.0, f'HDOP {h} out of range'
print('DOP ok')
"
```

Expected: `HDOP` between 0.5 and 3.0, `DOP ok`

---

### 5.4 Full pipeline with mocked PhysX (no scene needed)

```bash
ISAACSIM_PYTHON -c "
from pegasus.simulator.logic.gps_degradation.model import GpsDegradationModel
from pegasus.simulator.logic.gps_degradation.config import GpsDegradationConfig
from unittest.mock import MagicMock, patch

cfg = GpsDegradationConfig()
cfg.gps_update_every_n_steps = 1

with patch('pegasus.simulator.logic.gps_degradation.visibility.get_physx_scene_query_interface') as mock_p:
    mock_iface = MagicMock()
    mock_iface.raycast_closest.return_value = {'hit': False}
    mock_p.return_value = mock_iface

    model = GpsDegradationModel(vehicle_id=0, cfg=cfg)
    out = model.step(0.0, 1, 40.4, -79.9, 100.0, (0.0, 0.0, 100.0))
    print(f'fix_type={out.fix_type} sats={out.satellites_visible} eph={out.eph_m:.2f}m state={out.state.name}')
    assert out.fix_type == 3, f'expected 3D fix, got {out.fix_type}'
    assert out.satellites_visible >= 6, f'expected >= 6 sats, got {out.satellites_visible}'
    assert out.eph_m < 3.0, f'expected eph < 3.0m, got {out.eph_m:.2f}'
    print('pipeline ok')
"
```

Expected: `fix_type=3`, `sats >= 6`, `eph < 3.0`, `pipeline ok`

**Write back**: actual output line: `___________`

---

### 5.5 Live PhysX raycast test (requires scene + simulation playing)

Load a scene with buildings. Start simulation (play button or lockstep). Then:

```python
# Run inside an Isaac Sim script / extension callback, not standalone
from pegasus.simulator.logic.gps_degradation.visibility import VisibilityEngine
from pegasus.simulator.logic.gps_degradation.constellation import ConstellationModel
from pegasus.simulator.logic.gps_degradation.config import GpsDegradationConfig

cfg = GpsDegradationConfig()
const = ConstellationModel(cfg)
const.update(0.0)
sats = const.get_visible_satellites(40.4, -79.9, 100.0)

vis_engine = VisibilityEngine(cfg)
# Place drone at a known position inside/near a building for the test
drone_world_pos = (5.0, 0.0, 10.0)   # adjust to your scene
results = vis_engine.classify(drone_world_pos, sats)

n_los  = sum(1 for r in results if r.is_los)
n_nlos = sum(1 for r in results if not r.is_los)
print(f"LOS={n_los}  NLOS={n_nlos}")
# In open sky: expect n_nlos near 0
# Near a building: expect some n_nlos > 0
```

**Write back**: LOS/NLOS counts near a building: `___________`
**Write back**: LOS/NLOS counts in open sky: `___________`

---

### 5.6 EKF2 acceptance check (full integration, drone flying)

After the hook is wired into `px4_mavlink_backend.py`, launch the simulation
and connect a MAVLink console:

```bash
# In PX4 NSH console (via MAVLink shell or QGroundControl):
listener vehicle_gps_position
# Look for: eph_m, epv_m, satellites_used, fix_type changing as drone moves

listener estimator_status
# In open-sky scene: gps_check_fail_flags should be 0
# In urban canyon: some bits may flip (hacc/vacc/nsats)
```

**Write back**: `gps_check_fail_flags` value in open sky: `___`
**Write back**: `gps_check_fail_flags` value near buildings: `___`
**Write back**: does MACVIO switch trigger when fix_type drops to 0: yes / no

---

## Part 6 — Known issues to watch for on VM

These are confirmed design issues found during laptop review. They don't block
testing but need to be validated or fixed on the VM.

**Issue A — NLOS-blocked satellites flow through signal_model.**
A satellite classified as `is_los=False, multipath_extra_m=0.0` (fully blocked)
gets a C/N0 calculated from its elevation angle alone and can survive
`apply_signal_quality`. It's filtered correctly inside `compute_dop` and
`compute_position_error`, so outputs are numerically correct — but it's
confusing and wasteful. Fix: in `apply_signal_quality`, skip (or immediately
drop) any satellite where `sv.is_los == False and sv.multipath_extra_m == 0.0`
before computing C/N0.

**Issue B — multipath_extra_m has no cap.**
`multipath_extra_m = hit_dist` where `hit_dist` can be up to `raycast_max_distance_m = 50,000 m`.
A satellite behind a distant hill would inject a 50 km pseudorange bias into
the WLS solve, producing an enormous position error. Fix: cap at a physically
meaningful maximum — real multipath delays rarely exceed ~300 m of extra path.

```python
MAX_MULTIPATH_M = 300.0
multipath_extra = min(hit_dist, MAX_MULTIPATH_M)
```

**Issue C — World frame assumption is unverified.**
`visibility.py` maps `enu_unit = (east, north, up)` directly to Isaac Sim world
`(X, Y, Z)`. `gps_utils.py` confirms `+X=East, +Y=North, +Z=Up` for AirStack's
convention, but this must be verified scene by scene. If a scene uses a
different origin rotation (some imported USD scenes default to Y-up), ray
directions will be wrong and buildings won't block correctly.

Quick check on VM:
```python
# In the running sim, find a known landmark (e.g. a building at +X direction)
# and verify it is actually to the East of the world origin.
# If East is +Y instead of +X, swap e and n in visibility.py's ray_dir.
```

**Issue D — Scenario type is static.**
`cfg.scenario = "urban"` applies urban shadow fading everywhere regardless of
drone position. Not a correctness issue for urban-only test scenes, but will
give overly pessimistic results in open-field scenes. Fix is to make scenario
a per-scene config key (already noted in spec section 7).

---

## Part 7 — Questions answered from VM (fill in as you go)

| Question | Answer | Date |
|---|---|---|
| Isaac Sim version | | |
| ISAACSIM_PYTHON command that works | | |
| pymap3d install succeeded | | |
| px4_mavlink_backend.py full path | | |
| `lat` variable name | | |
| `lon` variable name | | |
| `alt` variable name | | |
| `eph` variable name | | |
| `epv` variable name | | |
| `fix_type` variable name | | |
| `satellites_visible` variable name | | |
| per-step method name | | |
| world position accessor | | |
| collision mesh count in test scene | | |
| buildings have collision meshes | | |
| LOS/NLOS near building (counts) | | |
| LOS/NLOS open sky (counts) | | |
| EKF2 gps_check_fail_flags open sky | | |
| EKF2 gps_check_fail_flags urban | | |
| MACVIO switches on GPS DENIED | | |
