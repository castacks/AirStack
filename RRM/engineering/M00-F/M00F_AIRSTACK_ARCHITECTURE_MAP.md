# AirStack architecture and evidence map

DESIGN ONLY — UNVALIDATED FOR DEPLOYMENT. No execution authorized.

Source IDs resolve in ../../evidence/M00-F/M00F_SOURCE_REGISTER.csv. OBSERVED means inspected checked-out bytes, not observed running behavior. DOCUMENTED means an official web statement; INFERRED means engineering deduction; UNKNOWN means missing evidence. Local parent is 6c63054b2d1ac0dee0c5840ed3d4c31083874177. Official /0.20/ pages were retrieved at the UTC times in the register; they expose no authenticated site-build commit. Their modern stack material agrees with selected local mechanisms, but the URL alone cannot establish release identity or frozen F00 equivalence.

| Layer | Classification | Mechanism / source |
|---|---|---|
| CLI | DOCUMENTED + OBSERVED | DOC1 describes CLI over Compose; CLI compose_cmd and launch resolution apply root .env plus overrides, profiles and generated module/fleet overlays. No CLI was sourced or executed. |
| Root composition | OBSERVED | ROOT includes Isaac, simple-sim, legacy AirSim, robot, GCS and docs component definitions. airstack_network is a bridge at 172.31.0.0/24 without internal:true. Inclusion does not select every profile. |
| Robot | OBSERVED | ROBOT desktop extends BASE with privileged:true, developer mounts, SSH publication and GPU request. Desktop command autolaunches desktop_bringup; default AUTOLAUNCH true can start more than diagnostics. |
| Topology dispatch | DOCUMENTED + OBSERVED | DOC2 / STACKDOC and DISPATCH: AIRSTACK_STACK_DIR and AIRSTACK_STACK_ENTRY select a namespaced launch entry after shared preamble and use_sim_time. FULL includes production autonomy. No legacy AUTONOMY_ROLE dispatch remains. |
| Extension model | DOCUMENTED + INFERRED | DOC2 supports custom stack copies with pinned modules and launch wiring. RRM design proposes a separate RRM-owned stack mounted at an explicit container path through the same dispatch contract; current CLI --stack resolves stacks/<name>, so an RRM-only path requires future reviewed mount/env adaptation. No generated overlay is assumed safe. |
| Simulator | DOCUMENTED + OBSERVED | DOC4, ISAAC, SCENE, APP: selected standalone script builds SimulationApp, environment and vehicle graph; first Play starts backend work. GUI-only mode is unsuitable for PX4 startup investigation. |
| PX4 / MAVROS | OBSERVED | BACKEND defaults PX4 autolaunch true and base simulator port4560 plus vehicle ID; INTERFACE computes companion UDP endpoints from domain and SIM_IP, unless FCU_URL overrides. MAVROS XML alone defaults to a serial FCU. This is a software SITL path with dangerous alternate entry points. |
| ROS2 / DDS | OBSERVED | DISPATCH establishes robot namespace and simulation time. Robot domain is identity-dependent; GCS/offboard domain0 and DDS routers can bridge domains. Namespace/domain separation is not network containment (DDS/SPLIT). |
| GCS | OBSERVED + INFERRED | GCS appears in both desktop profiles and includes monitoring/relay infrastructure. It is unnecessary for observing startup lifecycle and increases command/network exposure; remove from proposed diagnostic topology. |
| Effective runtime | UNKNOWN | Target, resolved overlays/env, immutable images, GPU policy, endpoint reachability, asset availability and failure survival have not been verified. Static source cannot prove deployed topology. |

Version drift: current dispatch/module/fleet design and reference stacks must be assessed at the pinned local commit, not treated as the frozen a6dad8c F00 configuration. Historical E1B runtime snapshots are dated evidence, not current runtime attestations. DOC3 describes desktop build choices; no build-network statement proves runtime isolation.
