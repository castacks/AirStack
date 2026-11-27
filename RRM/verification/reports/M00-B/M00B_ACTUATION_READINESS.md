# M00-B actuation readiness — original M00-B03

**Diagnosis established; execution blocker remains.** Live telemetry reports `actuation_status=STARTUP_DEADLINE`, with60/60 sampled `actuation_ready=false` observations. The effective child parameter is already ROS PARAMETER_DOUBLE value1.0. This excludes an incorrect *current* thrust-scaling value as the explanation for the retained false readiness, but it does not prove successful earlier initialization.

## Origin and prerequisites

The PX4-only `Px4ActuationGuard` is instantiated by `MAVROSInterface` when `is_ardupilot=false`. The actual publisher endpoint is `robot_interface` in `/robot_1/interface`. Source: `px4_actuation_guard.hpp` (SHA256e522a5f6…), `mavros_interface.cpp` and README, all inventoried in the source manifest. Loaded plugin file SHA2565173ece556bed1d09911c17c690f3702d42d4bcb1b95c9322a8d8d01e45e667b and process/library paths were captured. This establishes current file/process association and diagnostic consistency, not a reproducible-build attestation of loaded memory.

Startup requires:

- An accepted normalized floating-point1.0 profile in the installed px4_config.yaml, and a valid timeout in(0,60]s; current configured timeout is10.0s and explicit config path is empty, selecting the installed package default.
- Received connected, disarmed MAVROS state and ON_GROUND extended state, each no older than0.5 steady receipt seconds. Armed or IN_AIR evidence before initialization latches unsafe startup.
- Parameter setter availability, one guarded SetParameters attempt at most, successful response, immediate correct GetParameters response and safe state still verified. Failure/deadline latches closed; later correct telemetry/readback cannot reset it.

After verified initialization, the published Boolean is `initialized && last_good_readback_age<=0.5s`; the wall timer runs every200ms. It is **not** continuous physical-safe-state verification. ARM, REQUEST_CONTROL and TAKEOFF independently require initialized state and a fresh bounded readback through admit(); LAND/DISARM are not blocked by this guard. CLI readiness additionally requires exact child double 1.0 and the positive Boolean. Direct MAVROS/PX4 clients are outside this interface's enforcement boundary; none was used to bypass it.

## Current evidence

| Observation | Retained evidence |
|---|---|
| Guard terminal reason | All60 sampled String values STARTUP_DEADLINE; all60 readiness samples false |
| Child parameter | GetParameters on /robot_1/interface/mavros/setpoint_raw: type3/double 1.0 |
| Guard configuration | GetParameters on robot_interface: timeout10.0s; PX4 profile; use_sim_time=true |
| Startup error | Robot interface PID20295 log at1795805000.934741647: Actuation NOT_READY: STARTUP_DEADLINE |
| First recorded FCU connection | MAVROS log at1795805020.210903826: Got HEARTBEAT, connected;19.2762 wall-log seconds after deadline |
| Process ordering | PX4 PID975 begins21.62s after interface PID20295 by /proc start ticks; shared kernel boot ID and100ticks/s verified |
| Current vehicle telemetry | Correct topic /robot_1/interface/mavros/state received9 samples; last connected=true, armed=false, AUTO.LOITER. Extended state47 samples; last ON_GROUND. These are FCU reports, not independent physical truth or uninterrupted freshness proof |
| Reference versus now | November17 report used another plugin hash and generic failure reason. Current runtime has diagnostic String telemetry and a distinct plugin hash; the old undeployed-diagnostics limitation cannot be carried forward as current fact |

Evidence: `live-readiness.json`, `startup-log-extract.json`, `robot-identity.json`, `isaac-identity.json`, `isaac-detail.json`, `readiness-analysis.json` under `RRM/evidence/M00-B/`.

The current false flag is expected **fail-closed behaviour after this startup deadline**, not expected healthy readiness for execution. Late FCU startup/connection strongly supports startup ordering as the cause of the missed initialization window. The explicit terminal reason resolves the former timeout-versus-unsafe ambiguity. Complete historical state/ground callback receipts and parameter-transaction history were not retained, so no broader timing/transport root cause or counterfactual safe startup is claimed.

## Readiness evidence and next action

Closure requires a separately authorized controlled startup/recovery procedure with fresh connected/disarmed/grounded evidence, valid installed profile, recorded successful one-time transaction/readback, sustained fresh positive readiness and applicable broader readiness checks. Bind image/source/plugin/controller/scene/epoch identities and retain the startup evidence. A green interface flag alone does not qualify F00 flight, protective stopping or independent controller authority.

No restart, profile/timeout change, manual SetParameters, forced flag, mode request, ARM, TAKEOFF or other motion command occurred. A future operational recovery may require an authorized robot-only restart after PX4 is available; this is a proposed procedure for review, not executed work or a request to waive startup checks. Any protected-code repair needs its own PD-approved change. WP01 remains unstarted.
