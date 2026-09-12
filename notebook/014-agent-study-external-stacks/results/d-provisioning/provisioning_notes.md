# (d) Provisioning — 2026-09-11, agent_study `e52f94d`+

| Job | Pod | Duration | Output |
|---|---|---|---|
| `agent-study-provision-as2-2` (Aerostack2 Harmonic variant) | 8 CPU / 32 Gi / 300 Gi | 30 min (build 1718 s) | `airstack/agent-study-as2:humble-gzharmonic-82d85abcd904` (1.8 GB); built from upstream `docker/humble_gzharmonic/dockerfile` with the base tag and project_gazebo clone pinned (diff recorded in the pod log) |
| `agent-study-provision-1` (UAS `make images`) | 16 CPU / 64 Gi | 348 min | 13 targets pushed as `airstack/agent-study-uas:<target>-cb78bf7f57d0` (1–12 GB each; `ros2_ros1_bridge` and `ros2_cbf` share a digest); the second half of the job rebuilt the AS2 variant redundantly (same digest) |

Per-trial provisioning inside trial pods (before the agent starts; recorded in `provision_time_s`):

| Arm | What | Time |
|---|---|---|
| E2 | pull `aerostack2/nightly-humble:82d85abc…` (Docker Hub) + Harmonic mirror | 117–169 s |
| E1 | pull 13 UAS images (~80 GB) + `vcs import --exact` of 49 component repos (SSH→HTTPS rewrite; `ntnu-arl/helicopter_bridge` private → recorded failure, rest imported) | 853–1078 s |

Platform findings (not study defects): UAS's `scripts/import_all_repos.sh --exact` aborts at `ws_sim` because one listed repo is private, so an outside user never gets `ws_vectornav`/`ws_vlm` without editing the script; Aerostack2's nightly image ships Gazebo Fortress (ign-gazebo 6.18) while its docs describe Harmonic support via a separate dockerfile whose base tag floats.
