# Results Summary: `airstack up --scene <shortname>`

- **Date written:** 2026-08-24
- **Spec:** [../design_spec.md](../design_spec.md)

## (a) Resolver contract — PASS

**Setup:** 2026-08-24 ~20:45 EDT, host-side `python3 simulation/resolve_scene.py`
(artifacts: [a-resolver-contract/resolver_runs.txt](a-resolver-contract/resolver_runs.txt)).

| Case | Input | Result |
|---|---|---|
| Pegasus catalog key | `--sim isaac --scene warehouse` | `ISAAC_SIM_SCENE=Warehouse`, `ISAAC_SIM_STAGE_SCALE=1.0` |
| Nucleus URL + scale | `--sim isaac --scene construction-site` | `ISAAC_SIM_SCENE=omniverse://…/ConstructionSite.stage.usd`, `ISAAC_SIM_STAGE_SCALE=0.01` |
| MS AirSim key | `--sim airsim --scene blocks` | `MS_AIRSIM_SCENE=blocks` |
| Name collision (both sims) | `--scene default` on isaac / msairsim | `Default Environment` / `blocks` — per-sim resolution confirmed |
| Unknown scene | `--scene nonsense` | exit 1 + full availability table (29 scenes) |
| Wrong simulator | `--sim isaac --scene blocks` | exit 1 + "not available for Isaac (only: MS AirSim)" + table |

## (b) CLI dry-run matrix — PASS

**Setup:** 2026-08-24 ~20:45 EDT, `./airstack.sh up --dry-run …`
(artifacts: [b-cli-dry-run/dry_run_matrix.txt](b-cli-dry-run/dry_run_matrix.txt)).

| Case | Outcome |
|---|---|
| `--sim isaac --scene construction-site` | effective config gains `ISAAC_SIM_SCENE=omniverse://…` + `ISAAC_SIM_STAGE_SCALE=0.01`; scene shown in launch-config banner |
| `--sim airsim --scene blocks` | `MS_AIRSIM_SCENE=blocks` exported and shown |
| `--sim isaac --scene blocks` | exit 1, table printed |
| `--sim simple --scene warehouse` | exit 1: "simple-sim has no scene catalog" |
| `--sim airsim --scene forest` (not downloaded) | warn + "container will auto-fetch 'Forest'" (non-interactive path) |
| `--scene warehouse-forklifts` | value with spaces exports cleanly (`Warehouse with Forklifts`) |
| no `--scene` | effective config contains **zero** scene keys — byte-identical to pre-feature contract |

## (c) Stage-scale ground truth — PASS

**Setup:** 2026-08-24 ~20:40 EDT, `Sdf` layer metadata read via omni.client in the
isaac-sim image (artifacts: [c-stage-scale-ground-truth/units_check.txt](c-stage-scale-ground-truth/units_check.txt)).

| Nucleus stage | metersPerUnit | scenes.yaml stage_scale |
|---|---|---|
| AbandonedFactory.stage.usd | 1.0 | 1.0 |
| Warehouse_01_night.stage.usd | 0.01 | 0.01 |
| Warehouse_02_day.stage.usd | 0.01 | 0.01 |
| Map_ChemicalPlant_2.stage.usd | 1.0 | 1.0 |
| ConstructionSite.stage.usd | 0.01 | 0.01 |
| RetroNeighborhood.stage.usd | 0.01 | 0.01 |

Initial assumption (all 0.01) was wrong for 2 of 5 stages — values now match the layers.

## (d) Isaac smoke test — PASS

**Setup:** 2026-08-24 20:44–20:48 EDT, `./airstack.sh up --sim isaac --headless
--scene construction-site isaac-sim` on RTX 5090
(artifacts: [d-isaac-smoke-test/scene_load_log_excerpt.txt](d-isaac-smoke-test/scene_load_log_excerpt.txt)).

| Milestone | Evidence |
|---|---|
| Env-driven scene picked up | `[example_one] Scene: omniverse://…/ConstructionSite.stage.usd (stage_scale=0.01)` |
| Stage loaded from Public Nucleus | 682 colliders added to ConstructionSite meshes; MDL materials compiled from the Public URL |
| Drone spawned in the stage | `Prim '/World/base_link' created at [0.0, 0.0, 0.07]` + `PX4 node at /World/base_link/…/PX4Multirotor created` |
| No errors | 0 hits for Traceback / Stage failed / ERROR_ACCESS / ERROR_CONNECTION |

Container torn down after the check (`airstack down isaac-sim`).

## (e) MS AirSim pipeline test — PASS (with 2 bugs found and fixed)

**Setup:** 2026-08-24 20:57–21:08 EDT, `./airstack.sh up --sim airsim
--headless --scene <name> ms-airsim`
(artifacts: [e-msairsim-pipeline/pipeline_test_notes.txt](e-msairsim-pipeline/pipeline_test_notes.txt)).

| Case | Outcome |
|---|---|
| `--scene blocks` (present locally) | Entrypoint resolved `Blocks.sh`, UE4 booted headless, PX4 SITL reached "Ready for takeoff!", AirSim received lockstep heartbeat, bridge node published `/clock` (verified via `ros2 topic echo` on domain 1) |
| `--scene msbuild2018` (absent) | Host warn + in-container auto-fetch: Downloading → Extracting → Ready → UE booted from `/ms-airsim-env/MSBuild2018/`, PX4 "Ready for takeoff!" on the new scene |

**Bugs found by this test, both fixed:**

1. *Image-baked entrypoint*: the first non-blocks run executed the old
   entrypoint (baked via Dockerfile `COPY`) and silently launched Blocks —
   entrypoint changes require an ms-airsim image rebuild.
2. *Fictional fetch catalog*: `SoccerField.zip` 404s — the pre-existing
   `fetch_scene.sh` listed scenes the v1.8.1 release never shipped
   (forest, soccerfield, building99-as-`Building99.zip`) and pointed
   `airsimnh` at a nonexistent `Neighborhood.zip`. Catalog corrected to the
   real assets (+ `africasavannah`, `msbuild2018`); the entrypoint now
   glob-resolves the UE launcher after extraction.

## Overall verdict

All five sections PASS. Known limitations: the interactive MS AirSim
download prompt (TTY path) was not exercised (non-interactive path verified);
`africa-savannah` and the remaining large UE scenes were catalog-verified
(release assets exist) but not individually booted;
`barebones_pegasus_launch.py` intentionally keeps its hardcoded scene.
