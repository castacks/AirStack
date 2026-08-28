# Agent report B — docs/development tree

(Verbatim classifier output, archived for the audit design doc.)

**Nav ground truth (mkdocs.yml lines 77-108):** Everything under `Development` sits in three buckets literally named "Beginner Tutorials", "Intermediate Tutorials", "Advanced Tutorials". Three audited files are NOT in the nav: `development_environment.md` (top-level), `airstack-cli/index.md` (top-level), and `intermediate/testing/testing_frameworks.md` (still linked from testing/index.md and unit_testing.md, so reachable but nav-invisible).

## Per-file classification

| File | Quadrant | Purity / notes | Audience |
|---|---|---|---|
| index.md | Hub (none) | Learning-path nav + reference command tables; contradicts nav taxonomy (calls CLI docs "Reference Documentation" while nav says "Beginner Tutorials") | newcomer |
| beginner/key_concepts.md | Explanation (high) | Hybrid: "The Development Loop" + "Configuration: Environment Variables" are How-to/Reference embedded | newcomer |
| beginner/airstack-cli/index.md | Reference (high) | Mostly clean; `up` flags table omits `--scene` (exists at airstack.sh:1156) | competent |
| beginner/airstack-cli/docker_usage.md | How-to (med) | Hybrid; STALE: "Automated Testing" section predates pytest harness (service is `robot-test`, not `autotest`); Isaac `runapp`/Streaming Client-era instructions likely dead; SSH example IP 172.18.0.6 wrong subnet (should be 172.31.0.0/24) | newcomer→competent |
| beginner/development_environment.md | How-to (med) | Hybrid (requirements=Reference); near-duplicate of top-level development_environment.md; this one is in nav and slightly better | newcomer |
| beginner/vscode/vscode_debug.md | How-to (high) | Clean; STALE: says `.devcontainer/devcontainer.json` + `.devcontainer/Dockerfile` but actual layout is per-container subdirs (robot/, gcs/, isaac-sim/), no Dockerfile; omits isaac-sim devcontainer | newcomer |
| beginner/fork_your_own_project.md | How-to (high) | Clean; modules.md "researcher workflow" is the modern continuation, neither links the other | newcomer |
| development_environment.md (ORPHAN) | How-to | Confirmed duplicate revision of beginner/ version; delete/redirect | newcomer |
| airstack-cli/index.md (ORPHAN) | Reference (high) | Stale ancestor of beginner CLI index; `./airstack.sh` era; omits images/module/stack/fleet/doctor/ready/test/sync/osmo commands; delete/redirect | newcomer |
| intermediate/contributing.md | How-to+Reference (med) | Branch enforcement + VERSION tables = Reference; recipes = How-to; `pip install mkdocs-material; mkdocs serve` vs `airstack docs` inconsistency | contributor |
| intermediate/documentation.md | How-to + heavy Reference | Repo doc standards (templates, checklists) | contributor |
| intermediate/feature_notebook.md | How-to (med) | Hybrid: intro=Explanation, layout=Reference; fresh, links skill | contributor |
| intermediate/frame_conventions.md | Reference (med) | 7-line stub, NO H1; link to scene_setup.md#frame-conventions likely dead/renamed | competent |
| intermediate/docker-build-profiles.md | Reference+How-to (med) | Well-managed pairing with skill; `airstack connect --command` flag unverified | maintainer |
| intermediate/testing/index.md | Reference hub (med) | STALE: marks table omits `wiring` and `waypoint_flight` (both in tests/pytest.ini) | competent |
| intermediate/testing/unit_testing.md | How-to (high) | Hybrid: design principles=Explanation, CI tables=Reference; "Current test coverage" table rots instantly | contributor |
| intermediate/testing/end_to_end_testing.md | Reference w/ embedded how-tos (med, ambiguous) | All four quadrants in one doc; troubleshooting row about `autonomy` mark not in pytest.ini is stale (it IS declared); hard-coded AirStation baseline numbers will rot | competent/maintainer |
| intermediate/testing/ci_cd.md | Explanation+Reference (high it's both) | Interleaves maintainer (architecture, security) and user ("Using CI well") audiences; marks table omits wiring/waypoint_flight; absolute blob/main GitHub links brittle | maintainer+competent |
| intermediate/testing/testing_frameworks.md (ORPHAN from nav) | Fossil (low) | VERY STALE: contradicts co-located test convention; `@pytest.mark.rostest` not registered; GUIDED (ArduPilot) mode in PX4 stack; `launch_robot_headless.yaml` doesn't exist; still linked from testing/index.md + unit_testing.md | unclear |
| advanced/ai_agent_guide.md | Reference (high) | STALE: `local/c_controls/trajectory_controller` — actual dir is `local/controls` (verified); topic example inconsistent with own standard-topics list; second copy of drift-prone tables | AI agents |
| advanced/airstack-cli/architecture.md | Explanation (high) | Clean; "Future Enhancements" stale (command grouping/aliases shipped); module list omits 8 real modules (dev.sh, doctor.sh, fleet.sh, module.sh, osmo.sh, ready.sh, stack.sh, sync.sh, _lib.sh) | maintainer |
| advanced/airstack-cli/extending.md | How-to (high) | Mostly clean; more current than architecture.md | maintainer |
| modules.md | Explanation+Reference (high it's both) | Fresh RFC #379 material; correct Diátaxis pairing with modular_airstack.md tutorial; manifest schema correctly delegated to common/module_schema/README.md | competent→maintainer |
| stacks.md | Explanation+Reference (high) | Fresh + authoritative; "Why stacks don't launch standalone" is textbook Explanation | competent→maintainer |
| fleets.md | Explanation+Reference (high) | Fresh; env-var precedence table = Reference | competent→maintainer |
| module_ci.md | How-to-flavored Reference (med) | Fresh; file lives at docs/development/ top level but nav'd under Intermediate→Testing while siblings (modules/stacks/fleets) are under Advanced — location/nav disagreement | maintainer (module authors) |

## Tutorial-label verdict per nav bucket

- "Beginner Tutorials": zero actual tutorials (Explanation/Reference/How-to only). The only genuine tutorial in the doc set — getting_started/modular_airstack.md — lives outside this tree.
- "Intermediate Tutorials": zero tutorials.
- "Advanced Tutorials": zero tutorials — entirely reference/explanation/how-to.

## Synthesis

1. **Orthogonal axes**: nav organized by difficulty ("Beginner/Intermediate/Advanced Tutorials") while content is organized by Diátaxis type — every label wrong; a rename to Concepts/Guides/Reference or true Diátaxis nav fixes most without moving content.
2. **Fossil stratum**: three orphaned files duplicating/contradicting maintained successors; testing_frameworks.md worst (still linked from live pages, teaches superseded layout with broken example).
3. **Drift-prone duplicated hot tables**: `airstack up` flags in 3 places (one missing --scene), pytest marks table in ≥4 (two missing wiring/waypoint_flight), reference-implementations table in 2 (one carrying dead c_controls path). Each needs a single canonical home.
4. **Modular-era pages (modules/stacks/fleets/module_ci)** excellent but fuse Explanation+Reference for two audiences in one scroll.
5. Hygiene: frame_conventions.md titleless 7-line stub; docker_usage.md pre-harness "Automated Testing" + dead Isaac streaming instructions; vscode_debug.md lagging .devcontainer layout; absolute blob/main links in ci_cd.md/stacks.md.

Verified against: airstack.sh, .airstack/modules/, tests/pytest.ini, robot/ros_ws/src/local/controls/.
