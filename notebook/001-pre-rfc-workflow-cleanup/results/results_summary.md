# Results Summary: Pre-RFC Workflow Cleanup

> Written: 2026-08-20 · Branch `feature/pre-rfc-workflow-cleanup` (base `develop` @ `9e2e0e39`) · Machine: RTX 5090, Docker 29.7.2, images `v0.19.0-alpha.17`
>
> Commits under test: `87a22f4b` (launch-script dedup), `af9bb191` (CLI flags + preflight + ready), `090356bf` (tmux log teeing), `a5ff5723` (docs fixes)

## (a) Baseline — `results/a-baseline/`

Runs on clean `develop` before any change (2026-08-20 08:12–08:37 local):

| Suite | Command | Result | Wall time |
|---|---|---|---|
| unit | `airstack test -m unit` | **193 passed** | 20 s |
| liveliness + sensors (isaac) | `airstack test -m "liveliness or sensors" --sim isaacsim --num-robots 1 --stress-iterations 1` | **16 passed** | 14:15 |
| takeoff_hover_land (isaac) | `airstack test -m takeoff_hover_land --sim isaacsim --takeoff-velocities 1.0` | **8 passed** | 5:27 |

Raw artifacts: `a-baseline/2026-08-20_08-17-06/` (liveliness+sensors), `a-baseline/2026-08-20_08-31-23/` (takeoff), `baseline_console.log`, `baseline_system_console.log`.

**Incident during baseline:** first attempt failed fast — merged PR #384 had bumped `.env` `VERSION` to `alpha.17` while only `alpha.16` images were local. Live demonstration of the missing-images footgun this campaign's preflight check now surfaces in `airstack up` itself. Resolved with `airstack image-pull`.

## (b) Launch-script dedup parity — `results/b-dedup-parity/`

Same two system campaigns re-run on the branch (2026-08-20 08:55–09:26). **Verdict: PASS — full parity.**

| Suite | Baseline | Branch |
|---|---|---|
| liveliness + sensors | 16 passed | **16 passed** (15:01) |
| takeoff_hover_land | 8 passed | **8 passed** (5:28) |

`tests/parse_metrics.py` diffs: `metrics_diff_livsen.md`, `metrics_diff_takeoff.md`. All pass rates 100% → 100%. Flight-quality metrics (the behavior-parity signal for the rewritten Isaac launch scripts):

| Metric (rob#1, v1.0) | Baseline | Branch |
|---|---|---|
| takeoff altitude error | −0.151 m | −0.164 m |
| takeoff velocity RMSE | 0.227 m/s | 0.228 m/s |
| hover altitude mean error | 0.036 m | **0.021 m** |
| hover position stddev | 0.075 m | **0.061 m** |
| landing final altitude | 0.052 m | **−0.001 m** |
| PX4 ready duration | 90.1 s | 88.6 s |
| sim realtime factor | 1.276 | 1.301 |

Flagged deltas judged **not regressions**:

- `sim_ready_duration_s` 20 s → 77 s (liveliness): baseline outlier. The baseline's *own* sensors campaign measured the same gate at 79.6 s (branch: 73.5 s, −7.6%); my live `--wait` run measured 77 s. Three of four measurements cluster at 73–80 s.
- `airstack_up_duration_s` +0.15–0.2 s absolute (0.55 → 0.73 s): the new preflight (`docker compose config --images` + resolution) — intended cost.
- net_io / GPU-util increases during the stable poll: the branch runs stutter *less* (topic min-rates 14–24 Hz → 40+ Hz across the board), i.e. sustained full-rate sensor streaming. Quality improved; bytes followed.

Code delta: six scripts 1,415 lines → 536 lines + one shared 443-line `pegasus_app.py` (net −438). Deliberate fixes verified by construction: `ISAAC_SIM_HEADLESS`/`ISAAC_SIM_LIVESTREAM` uniform across all scripts (the sensors campaign runs headless — passing), `barebones` template no longer crashes (`py_compile` + import-order review), NatNet body-name env overrides now real.

## (c) CLI intent flags + preflight — `results/c-cli-flags/`

**Verdict: PASS.**

- `tests/meta/test_launch_intent_contract.py`: **14 passed, 1 skipped** (dump test skips on the runner's read-only mount) inside `airstack test -m unit`; full unit suite **207 passed** (193 baseline + 14 new).
- Dry-run matrix transcript: `c-cli-flags/dry_run_matrix.txt` — `--sim airsim` swaps profile+URDF; `--sim isaac --robots 3` auto-selects the multi script; `NUM_ROBOTS=3` + single-drone script is a hard preflight error naming the fix; `--env-file overrides/ms-airsim.env` now resolves through the guard (historical bypass closed); missing ms-airsim image correctly warned by name.
- Real bring-up without any `.env` edit: `airstack up --sim isaac --play --wait` (see (d)) — first time the repo can be launched sim-selected from flags alone.

## (d) Readiness gates — `results/d-readiness/`

**Verdict: PASS.** `up_wait_console.log`:

```
✓ robot containers running (0s)
✓ sim publishing /clock (77s)
✓ robot_1 (domain 1): autonomy nodes up (0s)
✓ robot_1: MAVROS connected to PX4 (12s)
✓ robot_1: PX4 EKF ready (armable) (2s)
[INFO] Stack is flight-ready (92s).
```

- Success path: flight-ready in **92 s**, well inside the harness budgets (600/300/300 s).
- Already-ready stack: `airstack ready --json` → all gates ok in 8 s, exit 0.
- Failure path: empty stack → gate 1 timeout, actionable message, exit 1; `--json` reports `"ready": false`.
- Caught during validation: `up --wait` without `--play` (with `.env`'s paused default) correctly blocks at the /clock gate — the gate message tells the user to press Play or use `--play`.

## (e) Log visibility — `results/e-log-visibility/`

**Verdict: PASS.** `docker_logs_excerpt.txt`: `docker logs airstack-robot-desktop-1` now shows live colcon build progress (`Starting >>> …` × 58 packages) and MAVROS output — 535 lines during bring-up; `docker logs isaac-sim` shows Kit extension startup through "app ready" — 576 lines. Before the change both were empty by construction (everything lived in unpiped tmux panes). `airstack connect` unchanged as the interactive path.

## (f) Docs correctness

**Follow-up (2026-08-20, commit `b1a3e4a2`):** added authoring documentation for the new structure — `spawning_drones.md` documents `PegasusApp` (import contract, kwargs, drone-config dict, hooks); `docker_usage.md` gains a "Launch flags and readiness" section; the `write-isaac-sim-scene` skill was re-taught from scratch (its old skeleton API was non-runnable), and five other skills' stale launch guidance fixed (`airstack stop`, bare `NUM_ROBOTS=N up`, NatNet name overrides). `mkdocs build` clean.

**Verdict: PASS.** `mkdocs build` clean (11.9 s; remaining INFO/WARN anchors pre-exist on develop). Greps: `ISAAC_SIM_SCENE` 0 hits in docs/ (was 6 across 4 files); `airstack stop`/`airstack build` 0 hits in AGENTS.md.

## Overall verdict

| Spec section | Verdict |
|---|---|
| 2.1 launch-script dedup | ✅ full system-test parity, −438 lines, 3 latent bugs fixed |
| 2.2 CLI intent flags | ✅ 14 contract tests + live bring-up |
| 2.3 preflight | ✅ guard bypass closed; footguns now named errors/warnings |
| 2.4 readiness | ✅ 92 s to flight-ready with staged progress; JSON mode |
| 2.5 log teeing | ✅ docker logs truthful for all services |
| 2.6 docs | ✅ 9 files corrected, build clean |

Known limitations / follow-ups:

- ms-airsim backend not system-tested this campaign (no local UE4 image; CLI derivation covered by contract tests + dry-run). Suggest one `liveliness --sim msairsim` run on CI (`/pytest -m liveliness --sim msairsim`).
- `example_multi_drone_scene_import.py` keeps its divergent ZED offset `[0.21, 0, 0.05]` (annotated in-code); resolving it is RFC #380 vehicle-config work.
- `ready` gates assume the PX4 stack; non-PX4 platforms are RFC #380 Part 2 territory.
