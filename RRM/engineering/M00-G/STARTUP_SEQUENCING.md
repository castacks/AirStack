# Startup sequencing contract

Offline implementation only. No orchestration driver or runtime readiness gate is implemented. Compose `depends_on: service_started` records process ordering intent; it does not mean recorder durability, simulator readiness, MAVROS connection, safety or permission. All entrypoints remain unresolved and every runtime component is blocked.

| Stage | Required evidence before progression | Current implementation |
|---|---|---|
| Recorder | Independent supervisor and durable init/ready; bounded storage/drop/termination plan | Observational writer/auditor implemented and tested in memory; runtime collector, durable fsync and supervisor blocked |
| Simulator | Exact image/local assets/GPU initialization reviewed; app-complete separately observed | Real upstream standalone command referenced read-only; entrypoint/image/asset/GPU unresolved |
| Scene/vehicle | Scene begin/end, graph, first Play/backend/PX4 spawn captured in producer clock | Event schema available; native adapters/hooks not implemented; no implicit Play/arming |
| MAVROS | Confined companion UDP, PX4/SITL identity, fresh actual state/extended-state, parameter services | MAVROS-only XML binds explicit sim peer and blank GCS route; plugins/effective namespace/endpoints unverified |
| Guard P0 | Concurrent guard construction with same event observers; preserve constructor steady origin | Minimal C++ original-header host source exists, unbuilt; no P0 trial |
| Guard P1 | Separate finite fresh connected/disarmed/on-ground and required-service predicate gates construction only | Independent readiness observer/controller is BLOCKED; do not replace with depends_on or guard READY |
| Startup deadline | Original guard >= timeout decision only during actual tick execution; original query lock/service effects remain | Header unchanged; offline reference covers deadline/latch subset only |
| Recovery | Request -> seal incomplete evidence -> isolate leftover resources -> explicit approved new attempt | Recorder validates documentary ordering; no reset, shutdown or restart commands implemented |

All actual new callbacks must occur after guard construction. No imported observer state, timeout extension, manual parameter repair or latch reset. The existing24-start proposal/first P1-P0 fresh pair is preserved as historical planning, not enacted by this Compose package. Exact scheduling, finite outer wait and resources require a separately reviewed binding.

Application-complete is not graph/first Play/PX4 or MAVROS readiness. Native producer monotonic/realtime clocks, recorder receipt clocks, kernel/process boot and simulation time remain separate until calibrated. The historical n=1 19.276162179s deadline-error→MAVROS and74.228771713s app-complete→graph are retained as characterization; neither becomes a configured acceptance threshold. Every failed/aborted/incomplete attempt must survive collection; no hidden warm-up/replacement/start is authorized.
