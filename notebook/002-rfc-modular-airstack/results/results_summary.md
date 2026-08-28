# Results Summary — RFC #379/#380 Campaign

> Written 2026-08-21 (runs 2026-08-20 → 2026-08-21, local RTX 5090, Isaac Sim 5.1).
> Sections mirror `design_spec.md` §3. Raw artifacts in the sibling letter dirs.

## (a) Wiring snapshot tooling — PASS

**Setup:** `airstack test -m "liveliness or wiring" --sim isaacsim --num-robots 1`. **Run at:** 2026-08-20 20:51.

| Check | Result |
|---|---|
| liveliness + wiring, isaacsim | 10/10 passed (5m14s) |
| First observed graph | 83 nodes / 405 edges (later 402 after keepalive fix) |
| Drift gate on independent bring-up | identical:true (2 passed, 61s) |
| Determinism fixes | phantom `_CREATED_BY_BARE_DDS_APP_` + Isaac SDG render-pipeline nodes excluded |

## (b) Module manifest + overlay + CLI — PASS

**Setup:** unit suites + live round-trips. **Run at:** 2026-08-20 (P1 16:xx, P2 19:xx).

| Check | Result |
|---|---|
| Manifest contract tests | 24 passed |
| Overlay contract tests | 12 passed (hermetic sandbox) |
| hello_module add→list→remove round-trip | clean tree restored |
| Real-module verify (asm_dfm2) | fragment merged, 3 launch scripts symlinked, compose config valid |
| Pinning rule | url-without-version and branch refs refused |

## (c) Reusable module CI — PARTIAL (local-equivalent PASS; GitHub dispatch pending)

**Setup:** workflow authored + actionlint-clean; dispatch blocked until the workflow exists on the default branch and the orchestrator polls the asm_ repos. The workflow BODY was executed locally end-to-end (see h). **Run at:** 2026-08-20.

## (d) Docker layer composition — PASS

**Setup:** heavy_module fixture, real builds. **Run at:** 2026-08-20 21:4x.

| Check | Result |
|---|---|
| Contract tests (identity, determinism, tiers, conflicts) | 18 passed |
| Real 2-step chain build (apt cowsay + pip tabulate + tier-2 marker) | verified in-container |
| Zero-module identity | byte-identical tags proven |
| PEP 668 | pip layers use --break-system-packages per house pattern |

## (e) Stacks wrap (P5-E1) — PASS

**Run at:** 2026-08-20 22:0x–22:3x.

| Check | Result |
|---|---|
| Legacy drift-clean through dispatch change | 10 passed |
| `--stack full_default` graph vs legacy golden | **identical:true** (machine-proven equivalence) |
| full_droan_cpu / full_macvo baselines | 84/419 and 84/407 (macvo = first WORKING macvo topology; droan subscribes perception/macvo/disparity) |
| Stack THL 1-robot | 4/4 |
| 3-robot landing timeout | reproduces identically on legacy → pre-existing (fixed later, see h) |
| Single-locus lint | frozen allowlist (19 real files; plan estimated ~10) |

## (f) Flatten + split + doctor (P5-E2/E3) — PASS

**Run at:** 2026-08-20 23:xx – 2026-08-21 00:5x.

| Check | Result |
|---|---|
| E2 flatten (local layer) graph equality | all 3 stacks drift-clean |
| E3 flatten (perception/sensors/global/behavior) | legacy + 3 stacks drift-clean, ZERO re-blessing |
| lite_default baseline | 80 nodes / 388 edges, no global-layer leakage (asserted) |
| Split stack bridge gate | gen_dds_router --check rejects trajectory topics; legacy config actually bridged set_trajectory_mode — now barred |
| doctor compose-time / --live | exit 0 on checkout; live drift + stack-scoping validated on running fleet |
| Generic-arg collisions | dds_router/gossip args prefixed w/ execution-tested aliases (the class that bit asm_optitrack) |
| Unit suite after E3 | 207→238 passed across P6 |

## (g) Fleet + airstack.yaml (P6) — PASS

**Run at:** 2026-08-21 01:xx–02:5x (artifacts: `g-fleet-airstack-yaml/`).

| Check | Result |
|---|---|
| Fleet liveliness (sim_one_default) | 16/16 |
| Fleet THL parity | 4/4, PX4-ready 84s |
| Heterogeneous sim_three_mixed (full+lite+split) | flight-ready 100s, all 3 armable |
| Split live dataflow | offboard random_walk+vdb on gcs host (domain 0); /robot_3/global_plan bridged to domain 3 |
| Resolver parity vs legacy | exact (contract-tested) |
| No-fleet byte-identical contract | test-pinned |
| Debug epic | root cause = fleet services invisible to `airstack down` → orphans at 100% CPU (load 70) tanked ALL flight tests (legacy control isolated it); fixed + 5 hardening fixes |

## (h) Module extractions (M1–M3) — PASS (GitHub CI pending)

**Run at:** 2026-08-20 throughout (artifacts: `h-module-extractions/`).

| Module | Validation |
|---|---|
| asm_dfm2_disturbances | ext loads in Isaac 5.1; flight-ready 91s under 3 spherical + 1 conical force fields; **takeoff_hover_land 4/4 with disturbances active**; friction log = RFC trip report |
| asm_optitrack | history preserved (PRs #359/#374/#375/#376); **e2e 6/6**: pose_alive → EV fusion → px4_ready → takeoff → Circle on mocap EKF2 → landing; EULA-gated SDK hook; emulator moved with module per maintainer |
| asm_macvo | **robot-desktop 17.1GB → 6.06GB (−65%)**; composed image 17.3GB carries the 11.3GB torch layer for MACVO users only; full_macvo wiring on composed image drift-clean |
| Landing-timeout fix | evidence-based: passing runs at 45.1–45.2s vs 45s cap → margin widened; optitrack e2e landing then passed |

## (i) Docs + registry — PASS (catalog run in progress at time of writing)

| Check | Result |
|---|---|
| Registry repo | live; 6 entries schema-valid; compat-stamp receiver dry-run tested |
| Per-phase docs | modules.md, stacks.md, fleets.md, module_ci.md, interface_conventions.md v1.0.0; skills rewritten; .agents README index 8→22 rows |

## Overall Verdict

| Spec section | Verdict |
|---|---|
| P0–P4 machinery | **PASS** |
| P5 stacks/lint/wiring/doctor | **PASS** |
| P6 fleets/airstack.yaml | **PASS** |
| P7 registry + docs | **PASS** (catalog finishing) |
| M1/M2/M3 extractions | **PASS** locally; GitHub CI + v0.1.0 tags pending infra |

## Known limitations

- Module GitHub CI needs: phases merged (workflow on default branch) + orchestrator poll-list additions (other host) + org registry secrets. Tags v0.1.0 follow first green CI.
- Split-stack wiring.md baseline not yet captured (two-host capture; doctor --live validated the mechanism).
- Deferred by design: per-robot `overrides:` application, `effective_config.yaml` YAML form + `config freeze`, release-set registry resolution, URDF generation from xacro+mounts, nightly canary, external dispatch test bench, cross-embodiment (RFC #380 Part 2).
- Pre-existing issues found and FIXED: landing-timeout margin, launch-config global collisions, effective-config same-second collision. Found and DOCUMENTED: topic_keepalive type mismatch on macvo disparity (doctor --live candidate).
- Force-push of the rewritten branch history pending owner approval; branch is ~35 commits ahead locally.

## J — Full-stack audit execution (owner rulings 1–14, 2026-08-22)

All 14 rulings executed and validated. Domain commits 58a4bd2a..262de774.

| Verdict | Gate | Result |
|---|---|---|
| Deletions (rulings 1–3,5,6,8,11) | unit + grep sweep + clean colcon | robot 45/45, gcs 17/17 packages build; zero dangling references |
| Dep purge (7,14) | image build + in-container build | −152MB (6.06→5.91GB); droan_gl deps now declared |
| simple-sim adoption (10) | new simple_sim smoke mark | 4/4 passed — **found sim dead since Jazzy migration** (bashrc sourced humble); fixed |
| isaac-sim-gui doc (9), prebuilt demotion (12) | docs contracts | green |
| License/metadata (13) | new package-metadata contract | BSD-3-Clause-Clear repo-wide + 4 module repos; 38 package.xmls real maintainers/descriptions; airstack_msgs 1.0.0 |
| Wiring drift, all 5 stacks | wiring mark, isaac | full_default, full_droan_cpu, full_macvo, lite_default, lite_offload_global:onboard — all green |
| Unit suite | pytest -m unit | 400 passed, 7 skipped |

Cross-repo catches the gates surfaced (both fixed module-side + re-pinned):
- asm_macvo rode on trunk's rich/tqdm — declared in its Dockerfile.module (06f3a8c8)
- asm_macvo imported trunk-deleted sensor_interfaces srv it never used — dropped (269ffc0b)
- Harness --stack now accepts <name>[:<entry>] (split stacks were untestable via --stack)

Deferred candidates surfaced, awaiting ruling: rqt_behavior_tree (zero referrers post-deletions), ETH mav_state_machine/mav_system_msgs upstream "TBD" licenses, C++ MIT header blocks in robot_interface/mavros_interface, drone_australia_zoomed_*.png orphans.
