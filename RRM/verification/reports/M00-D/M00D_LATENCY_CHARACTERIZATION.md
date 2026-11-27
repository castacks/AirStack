# M00-D latency characterization

**One historical startup episode; zero new startup trials.** Independent offline replay of the original retained producer logs confirms **19.276162179s from the guard deadline error to the first recorded MAVROS connected heartbeat**. This is not total PX4 startup duration, heartbeat transit latency, timer jitter or a physical response time. M00-C's measurement is preserved, with no prior evidence altered.

Source: `RRM/evidence/M00-C/startup-source-records.json` contains the original ROS log snapshots, selected Docker timestamped startup lines and same-boot process tick records. `RRM/evidence/M00-D/latency-measurements.csv` includes exact endpoints/JSON locators, domains, missing-event dispositions and interpretation limits for 19 quantities; `characterization-summary.json` records 12 measured intervals and 7 missing measurements.

| ID | Available interval | Seconds | Meaning / limit |
|---|---|---|---|
| L01 | Application Starting→app ready |0.408421426 | Docker log receipts of these application messages, not launch/container/scene readiness |
| L02 | app ready→Application Startup Complete |5.165411227 | Another application message phase |
| L03 | Application Startup Complete→vehicle graph created |74.228771713 | Broad uninstrumented application/scene/setup interval; cannot attribute all to scene loading |
| L04 | Graph created→PX4 starting log |8.511624795 | Graph evaluation/backend/spawn work plus logging; not pure PX4 executable initialization |
| L05 | PX4 starting→startup script returned |6.676805859 | Logged PX4 boot-script phase only |
| L06 | PX4 starting→simulator TCP connected |0.271513958 | Simulator path; not companion UDP/MAVROS or DDS |
| L07 | Simulator TCP connected→Pegasus heartbeat received |5.535654037 | Backend path log interval; no source-emission/packet-loss timestamps |
| L08 | Companion UDP initialized→MAVROS connection receipts |1.222467913 | Host Docker receipt differences across producer containers; buffering unknown, not wire/discovery latency |
| L09 | app ready→PX4 starting |87.905807735 | Shows app-ready does not mean actual vehicle/PX4 readiness; overlaps L02–L04, not an independent sample |
| L10 | PX4 starting→commander Ready for takeoff |10.377025568 | PX4 message sequence, not AirStack guard initialization success |
| L11 | Deadline error→MAVROS connected heartbeat |19.276162179 | Original producer epoch log arithmetic; not total PX4 startup duration |
| L12 | PX4 process birth after interface process birth |21.62 | Same-boot kernel ticks divided by 100 Hz; process-start separation, not initialization duration |

L01–L10 use the host Docker receipt clock; stream buffering, log formatting and scheduler delay are not bounded. L11 uses original ROS logging epoch timestamps1795805000.934741647 and1795805020.210903826 with Decimal subtraction. Realtime continuity was not independently calibrated. L12 uses matching boot IDs and100 ticks/s; 10 ms tick granularity per event, conservative delta quantization<20 ms. No mapping from kernel or steady clock to UTC is fabricated. Exact printed arithmetic is reproducible; accuracy is limited by acquisition semantics.

## Missing quantities and variability

| Missing quantity | Why unavailable / required instrumentation |
|---|---|
| L13 workflow initialization | No deployed workflow identity, submit/schedule/pull/init markers; I01 |
| L14 exclusive scene-load duration | No scene-load begin/end or asset/shader/I/O spans; I02/I06 |
| L15 actual PX4 spawn-to-ready | No spawner request/return/exec-ready semantic boundaries; I03 |
| L16 ROS2/DDS discovery latency | Router running and a later callback do not timestamp endpoint matching/source publish/guard receipt; I04/I05 |
| L17 deadline evaluation/jitter | Guard uses steady construction origin and wall timer; numeric origin, scheduled/actual callbacks and query-lock events missing; I05 |
| L18 initial latch transitions | Complete original safe-state/Set/Get/initialized/attempted/unsafe history absent; later false/STARTUP_DEADLINE observations only; I05 |
| L19 resource contention | No historical phase-aligned CPU/GPU/quota/I/O/asset/network trace; present load cannot reconstruct it; I01/I02/I06 |

No mean startup estimate, standard deviation, confidence interval, success rate or p95 is asserted from n=1. Repeated reads of the same log or dozens of readiness samples do not add independent startups. M00-B/M00-C captures came from the same running episode and remain historical; no new Boolean observation is implied by current Docker metadata.

Existing evidence supports a broad pre-PX4 delay plus an interface dependency-ordering mismatch and intended failed-initializer latch. It does not discriminate exclusive scene/asset/shader work versus resource contention, nor quantify DDS callback latency or full guard deadline behaviour. Simulator poll-timeout records identify an additional post-spawn phase, not a sufficient explanation for the earlier guard expiry. Missing instrumentation is identified rather than assigning a generic cause from warnings or event order.

The 10.0 s guard timeout, 0.5 s freshness and 200 ms timer remain implementation observations. Authoritative F00 startup timing allocation is unavailable. E4's readiness-true≥30 s run-entry condition is separate from guard-start timing; neither is changed. All latency quantities are characterization-only; no new acceptance threshold is created.
