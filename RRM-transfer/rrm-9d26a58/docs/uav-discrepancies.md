# UAV discrepancies and physical-model register

2026-11-15; `ore_proj`, base `07ff1dcbb` plus working changes. Preserve incomplete/failed attempts; these findings are not all confirmed root causes.

| ID | Finding | Evidence / disposition |
| --- | --- | --- |
| D01 | Startup actuation guard latched NOT_READY before safe simulator state arrived. | First takeoff blocked. Grounded robot restart initialized the guard. No forced arming. |
| D02 | Failed arming containment left disarmed PX4 in LAND, which was not armable. | Second takeoff blocked; normal grounded AUTO.LOITER restored commander preflight OK. |
| D03 | Low-rate state/health receipt can exceed a fixed wall freshness bound under slow simulation. | Survey-01 state age 2.0039 s; stop-01 health age 2.0276 s with otherwise fresh state. Reliable low-rate subscriptions and explicit 0.25-s health publication added; 2-wall-second freshness and 1-s loss threshold retained. |
| D04 | Planner consumed a 0.54-s-old commanded start despite 0.075-s receipt age. | Survey-02 rejected. Latest-only reference histories built; later five-leg survey completed. Internal queue backlog is plausible, not proven sole cause. |
| D05 | Contact evaluator initially allowed any UAV body against floor. | Provisional PASS superseded. Camera descendant reports violate strict body-only policy. Old reports lack separation; zero camera impulses do not prove impact or penetration. Old no-separation flights remain unqualified; two new separation-aware episodes pass their declared cases. |
| D06 | Camera collision removal does not match active descendant state. | Spawn code calls root removeCollider; five camera descendant collision meshes are enabled in the live stage. Passive settling and fresh flight evidence show positive-gap/zero-impulse proximity. Geometry unchanged; sensor-collider intent remains a configuration concern. |
| D07 | Assembled vehicle is substantially heavier than its authored body alone. | Live dynamic-control snapshot totals 2.834705 kg: body 1.5, lidar 1.0, camera 0.21609, rotors about 0.11861 kg. Do not use body-only hover estimates or transfer controller claims without this configuration. No mass was changed. |
| D08 | Two floor actor paths produce different contact records. | Environment Geometry provides supporting impulses and near-zero separation; GroundPlane gives positive 0.019–0.022 m separation with zero impulse in the passive probe. Contact-manifold presence is not equivalent to physical impact. |
| D09 | Requested trajectory speed is not an observed hard speed limit. | Requested vertical 0.5 m/s produced about 0.244 m/s ascent peak; circle requested 0.3 m/s reached about 0.422 m/s. No retrospective speed requirement imposed. |
| D10 | Reference freeze and action completion differ from physical outcome. | Circle reference-to-PhysX raw RMSE 0.172 m. Moving PAUSE produced 0.104 m maximum extra displacement; final endpoint displacement was only 0.017 m. Producer label corrected; raw trace retained. |
| D11 | First contact status history started 12.64 s late. | First contact-instrumented procedure completed but cannot establish complete contact qualification. Recording now starts before launch. |
| D12 | Bounded diagnostic recorders eventually stop. | PhysX recorder byte cap and contact header cap observed after relevant trials. Assess only fully covered intervals; no whole-file/whole-session completeness claim from final status alone. |
| D13 | Runtime parameter introspection initially hid a fallback default. | Helper get_param declared a typed unset parameter. New health period uses native default declaration and startup logging; live reports 0.25 s. |
| D14 | Host AirStack CLI dry-run is broken. | Missing _env_value and empty effective config; invalid launch not executed. Verified existing launchers and container identifiers used. |
| D15 | Broad package lint/style checks fail. | Functional/GPU planner tests pass 18; broad colcon output retains 98 failures/13 skips. Do not report a clean full package suite. |
| D16 | Evidence export remains local. | Raw sensor/pose/contact records and source changes are not a verified external backup or source push. GUI video was not recorded; onboard images are distinct evidence. |

| D17 | Client could exit while goal admission was pending. | First separation attempt blocked before arm. Explicit UUID/pending future now retained and late acceptance canceled before normal landing. Three isolated actual-probe contracts pass; pending goals false in both new flights. Backend gates unchanged. |
| D18 | Watchdog detects receipt silence, not stale source time. | Actual installed binary on DDS90 stayed healthy with continuous10-second-old odometry. Source freshness is NOT DEMONSTRATED; subsequently corrected by source-aware checks with isolated and bounded physical evidence. |
| D19 | PAUSE request does not establish successful intervention. | Rejecting mock service received one request in a3-second outage; no retry observed. Acknowledged/bounded retry behavior subsequently passes isolated service-fault tests and one physical source-stale PAUSE case; general fault recovery remains unqualified. |

Passive semantics diagnostics are complete for this fixture; bounded vertical and operator-stop cases have new full separation evidence. No colliders, masses, materials, solver settings, NVIDIA drivers, or planner collision/visibility gates were changed to make a test pass. The diagnostic initial-pitch case is an unarmed gravity-settling probe, not a flight qualification scenario.

The source/acknowledgement monitor changes add a wall health timer and explicit two-wall-second advancing-stamp deadline alongside the unchanged one-ROS-second source/receipt threshold. Its full package suite passes. A private stale-stamp relay into a dedicated installed monitor caused real automatic PAUSE and physical braking; the main estimator/controller continued receiving genuine state. Supervisor cancellation and LAND are separate from automatic intervention. Expired RPC bookkeeping removal cannot cancel remote execution, and other controller-mode writers remain outside exclusive monitor arbitration.

Automatic-stop timing starts at fault injection, not at the later controller acknowledgement. Initial assessment field names inherited from operator PAUSE were clarified in a separate final output; both outputs and original assessor source are retained. Final physical metrics:0.31397m/s at injection, max extra travel0.140849m, stable stop3.04970wallseconds including1s dwell. No contact-policy violations or unknown contacts in this episode. Live-stream snapshots are exact complete JSONL prefixes, hashed and checked against original prefixes; no stream-tail completeness claim beyond snapshot bounds.

The marker viewpoint model predicted area ratio1.5; observed ratio was1.40741 with physical displacement0.55496m versus planned0.63547m. The inverse-depth-squared ratio from measured depths was1.42368. Do not identify a unique cause or equate the difference of displacement magnitudes to a calibrated tracking error. Optional UAV test imports are guarded so absent pytest/NumPy/OpenCV does not break the dependency-light core runner; four missing-dependency module skips were verified separately.

## Restored-workspace development — 2026-11-17

| ID | Finding | Evidence / disposition |
| --- | --- | --- |
| D20 | Historical UAV raw evidence is absent from checked restored-host artifact locations. | Old compact results remain versioned; current Isaac process runs the default launch script. No historical raw-flight reproduction or current ground-state claim. |
| D21 | Restricted segmentation cannot detect every occlusion. | Fixed offline campaign retains partial connected occlusion as OBSERVED/SELECTED; split/full cover abstain. Software disposition evidence only. |
| D22 | Malformed viewpoint payloads could raise instead of abstaining. | Container/field validation now returns ABSTAIN/no goal. Nominal geometry and detector unchanged; focused47tests pass. |
| D23 | Watchdog PAUSE waited behind an outstanding operator resume/rewind response. | Isolated installed binary on DDS197 failed all three400ms priority checks. Candidate built separately sends PAUSE concurrently and requires post-operator-completion acknowledgement. No live deployment or physical-stop claim. |
| D24 | Operator timeout discarded the future and could leave readiness true despite a remotely pending mode change. | Isolated baseline retained; candidate keeps the single future, requests PAUSE and reports operator_request_unresolved until reconciliation and renewed PAUSE. A lost response remains uncertainty, not successful cancellation. |
| D25 | PAUSE did not prevent a late trajectory_override replacing the reference. | Actual isolated controller baseline moved the published reference0.40999952m under fixed synthetic odometry while output velocity stayed zero. Candidate drops overrides during PAUSE; zero-excursion regression passes. No physical-stop or exclusive-authority qualification. |
| D26 | Controller accepted undeclared mode values. | Baseline accepted999 and replaced PAUSE. Candidate rejects unknown values before mutation; actual service/output regression passes. Valid legacy mode writers remain separate authorities. |

The watchdog candidate's full eight-check package suite passes (aggregate18tests,
zero errors/failures,two skips). Existing source-clock/service-fault contracts remain
passing. The first candidate's operator-timeout test exposed a cached steady-time
comparison delaying a newly latched PAUSE until another timer tick; corrected and
failed output retained. Other controller-mode writers and remotely queued requests
still require controller-side authority/generation fencing for exclusive arbitration.
The candidate is source and isolated-test evidence, not a replacement for the live
monitor or a requalification of November physical traces.

| D27 | Total depth spread rejected the normal rendered marker cube. | Nominal front and underside span 0.402 m, with maximum adjacent step 0.092 m. The initial 0.1 m global veto is retained as an explicit optional profile; default now checks local continuity and connected valid support. Saved baseline and all 18 private-rendered inputs remain available. Development controls, no physical requalification. |
| D28 | Kit cleanup can mask an exception with exit status zero. | Initial private capture omitted later cases despite exit0; the next attempt saved an explicit HydraTexture cleanup AttributeError. Final path-based cleanup requires a complete six-case result and no error record in addition to terminal exit0. Live processes preserved. |
| D29 | Host GPU enumeration differs from the simulator container. | Host's single small-used-memory view did not describe active GPU0. Simulator sees four GPUs; GPU0 had approximately 44 GiB free at the process check. Resource observations must come from the process's container. |

| D30 | Marker receipt-age check preceded a potentially five-second action-server wait and used adjustable wall time. | Source now rechecks every channel against its observation window using monotonic time after server discovery. Clock-controlled expiry/ordering tests and a passive18-window audit pass. No new powered-flight qualification. |

| D31 | The local 0.1 m depth-step gate depends on resolution and view. | Frozen nine-view nominal-cube matrix passes8controls; low-camera240×150 produces one OBSERVED then two DEPTH_INCONSISTENT frames (max step0.1674m) and window ABSTAIN. The480×300/960×600 low views pass. Preserved raw/source evidence; no threshold retuning or unique renderer-cause claim. |

| D32 | Repeated PAUSE retries lacked a distinct deadline-escalation signal. | Source adds a finite positive steady-time acceptance budget, current escalation Bool and process/episode JSON snapshot. Actual DDS195 fake-server contract28/28 and package9/9CTest pass. Late acceptance retains miss history; no automatic recovery consumer, physical stop confirmation or controller-wide fencing is claimed. |

| D33 | Integer mode RPCs and untagged trajectory topics cannot reject delayed commands by stop generation. | New opt-in controller-local profile gates all three paths with process/generation/sequence. Actual isolated60-check contract and both package suites pass. Default stack unchanged; caller labels are unauthenticated and full admission remains open. |
| D34 | Tiny requested trajectory speeds are silently raised by the library timing floor. | A precursor control expecting overflow instead accepted1e-308m/s and replaced the path. Final fenced profile rejects speeds below the existing0.01m/s floor; boundary and separate true overflow checks pass. Failed trace/source preserved. |

| D35 | Empty trajectory buffers published NaN completion from0/0. | Actual combined monitor/controller precursor retained34nonfinite samples. Empty buffer now publishes0; final25-check combined contract and full controller package pass.0 denotes no path progress, not successful task completion. |
