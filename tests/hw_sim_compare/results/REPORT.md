# Hardware vs Isaac velocity-ZOH replay — `drone1_20260812_030151`

No-delay baseline. Same 20 Hz `TwistStamped` stream as hardware. Not Takeoff/Navigate/Land.

## Plant (`starling2max.usd`)

| Quantity | Value |
|---|---|
| `physics:mass` on `/World/base_link/body` | **0.557 kg** (already set; not overwritten) |
| COM | (0.00102, 0.00016, 0.00741) m |
| diagonal inertia | (0.002943, 0.001857, 0.004649) kg·m² |
| rotors | `/World/base_link/rotor0`–`3` (Pegasus paths present) |

Spawn XY = hardware start (0.0242, 0.0353). Z raised to 0.08 m so the mesh is not in the Grid ground plane.

## Interface

Live topic: `/robot_1/interface/velocity_command` (`geometry_msgs/TwistStamped`, 1 subscriber).

Hardware bag namespace is `drone_1`; sim is `robot_1`.

`mavros_interface` originally published `FRAME_BODY_NED` (map-frame +vz takeoff sits on the floor). Fix: `FRAME_LOCAL_NED` with **ENU values as-is**. MAVROS converts ENU→NED. Do not convert in the plugin (that double-flips z).

PX4 params set live: `MPC_XY_VEL_P_ACC=1.8`, `MPC_Z_VEL_P_ACC=8.0`, `MPC_XY_VEL_MAX=0.6`, `MPC_XY_P=0.95`, `MPC_Z_P=5.0`.

## A. Isolated step `v = (0, −0.6, 0)` from hover

| | dead time | τ |
|---|---|---|
| **sim (this run)** | **~200 ms** (100–350 ms; 10 Hz vel samples) | **~151 ms** |
| hardware bag (5 steps) | ~120 ms (100–130) | ~555 ms (550–670) |

τ is **~3.7× faster** than hardware. Dead time is the same order (slightly longer). No-delay baseline — gains not retuned. Sim reaches −0.59 m/s in ~1.0 s; hardware never quite reaches 0.6 m/s in 1.7 s holds.

## B. Full ~98 s overlay (log time)

Recorded 8350 poses at **85 Hz**.

| | XY RMSE | Z RMSE | end pose |
|---|---|---|---|
| **this run** | **2.61 m** | **3.43 m** | sim (1.73, 0.09, 5.96) vs hw (−2.04, −3.01, −0.04) |

Qualitative mismatch (expected for no-delay / v=0 hover thrust):

- Streaming `v=0` after arm **lifts** the sim (hover thrust). Hardware stays on the ground until +vz at t≈7.3 s. First replay sample is already z≈1.3 m.
- Z then climbs to ~6 m (takeoff +vz applied on top of an already-airborne hover).
- XY oscillates in a ~2 m box (path length 27 m vs hardware 7.7 m) and does not settle on the (0,−2)→(−2,−3) polyline.

Artifacts: `overlay.png`, `step_overlay.png`, `velocity_zoh_full.npz`, `velocity_zoh_step.npz`, `report.json`.
