# MAVROS interface

Implements the AirStack robot interface over MAVROS. Commands and setpoints use
relative names under the launch namespace; no robot identity is hardcoded.

## PX4 actuation configuration and admission

Dynamic MAVROS plugin nodes can ignore process-wide ROS parameter files. The
interface therefore configures `mavros/setpoint_raw` directly at startup and
verifies its effective `thrust_scaling`, rather than trusting source YAML alone.
This guard applies to PX4, not ArduPilot.

Startup requires fresh (at most0.5s steady receipt age) connected, disarmed MAVROS
state and ON_GROUND extended state. An armed or IN_AIR observation before successful
initialization locks out repair until an interface restart. Missing evidence waits
at most `actuation_startup_timeout_s` (default10s, finite and0–60s). The guard makes
at most one SetParameters attempt, then checks an immediate GetParameters response
and fresh safe state. Any rejection, timeout or wrong readback stays NOT_READY;
an uncertain remote Set may still take effect but does not grant admission.

The profile is read from the installed `interface_bringup/config/px4_config.yaml`,
or the explicit `mavros_actuation_config` path. Only an unquoted floating-point
finite1.0 under `/**/setpoint_raw/ros__parameters/thrust_scaling` is accepted.
Missing files, integer/string values and other scaling profiles fail closed.

After startup, monitoring is read-only: no automatic parameter repairs during
operation. Every ARM, REQUEST_CONTROL and TAKEOFF call independently performs a
bounded fresh GetParameters check before forwarding; cached READY alone is never
admission. LAND and DISARM are not blocked by this gate. Parameter discovery and
response waits are250ms each; query-lock acquisition is at most100ms. MAVROS control,
arming, disarming and ArduPilot takeoff response waits are at most2s; these are
software waits, not physical stop-time guarantees.

`actuation_ready` (`std_msgs/Bool`) is published from a wall timer and means startup
was verified and the last good readback is at most0.5s old. `airstack ready` requires
both effective double1.0 and this positive report. A failed initializer cannot be
made ready merely by a one-off manual parameter set.

`actuation_status` (`std_msgs/String`) explains the guard's current diagnostic
state. It does not grant command admission or replace a fresh `actuation_ready`
sample and per-command readback.

| Reason | Meaning |
| --- | --- |
| `WAITING_FOR_SAFE_STARTUP` | Required safe startup observations are not yet sufficient. |
| `WAITING_FOR_PARAMETER_SERVICE` | Safe startup evidence was seen, but the child setter is unavailable. |
| `UNSAFE_BEFORE_INITIALIZATION` | Armed/in-air evidence latched the initializer closed. |
| `STARTUP_DEADLINE` | Safe initialization did not begin before the configured deadline. |
| `CONFIG_REJECTED` | Startup profile or timeout configuration was rejected. |
| `STARTUP_NOT_VERIFIED` | The one attempted configuration/readback/safe-state transaction did not verify. |
| `EXCEPTION` | Initialization or monitoring raised an exception; the guard stays uninitialized. |
| `READY` | The same readiness predicate used for the Boolean is true. |
| `READBACK_UNVERIFIED` | Initialization succeeded earlier, but current readback freshness/validity is insufficient. |
| `QUERY_BUSY` | Another guarded query owns the lock; this tick did not refresh its readback. |

These reasons distinguish observations from permission. A terminal startup reason
persists until restart; later safe telemetry or a correct parameter alone does not
repair it. Monitoring after initialization remains read-only. Existing startup
and command deadlines, one-attempt policy and admission decisions are unchanged.

This is a command-interface guard, not permission enforcement on direct MAVROS/PX4
clients. The GUI does not independently query it before launching a mission;
commands routed through this interface are guarded. It does not qualify thrust
baseline, scene physics, flight behavior or physical containment.

## Control authority

`has_control` requires received armed state plus PX4 OFFBOARD (or ArduPilot
GUIDED) mode. A disarmed vehicle retaining that mode is not reported as having
armed control authority. PX4 `arm()` still checks retained OFFBOARD mode directly
and requests AUTO.LOITER before rearming; changing the authority predicate does
not bypass that handoff. This flag alone is not a freshness/connection guarantee:
consumers must independently require current connected state and fresh telemetry.

The armed-aware predicate is a source-only candidate in the current remote
workflow; its running interface still uses the previous mode-only implementation.

## Verification

`test/isolated_actuation_admission.py` runs the actual plugin through
`robot_interface_node` against mocked MAVROS services in empty domain198. It covers
safe initialization, unavailable/rejected/delayed services, invalid YAML/readback,
unknown/armed/airborne startup, unsafe-to-grounded transitions, parameter regression,
nondefault namespace, open LAND/DISARM and clean process shutdown.
Additional transition assertions cover automatic disarm retaining OFFBOARD,
armed control versus LAND mode and LOITER-before-rearm service ordering.
Never run its synthetic publications on a live robot domain. It is registered as serial CTest
`isolated_actuation_admission` when BUILD_TESTING is enabled.
