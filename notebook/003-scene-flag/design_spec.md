# Design Spec: `airstack up --scene <shortname>`

- **Date started:** 2026-08-24
- **Last updated:** 2026-08-24
- **Overall status:** `DONE`

## 1. Problem context

Developers currently pick simulation scenes by hand-editing simulator-specific
knobs: for Isaac/Pegasus they edit `env_url=` inside a launch script (or write a
fleet file with `sim.scene`); for MS AirSim they set `MS_AIRSIM_BINARY_PATH` to
a UE binary path (default auto-fetches Blocks). There is no single,
simulator-agnostic way to say "run the construction site scene".

This session also just published five AirLab stages to the guest-accessible
Nucleus folder `omniverse://airlab-nucleus.andrew.cmu.edu:443/Public/AirStack/Stages/`
(AbandonedFactory, AbandonedWarehouse day/night, ChemicalPlant,
ConstructionSite, RetroNeighborhood) — audited: all intra-stage refs are
relative; only external deps are 4 public NVIDIA S3 dome-light USDs. These
should be selectable by shortname.

Decisions made in session:

- One flag: `airstack up --scene <shortname>`; the shortname is mapped to the
  **selected simulator's** scene reference automatically.
- Unknown scene (or scene not available for the selected sim) → error + a
  table of available scenes per simulator.
- Isaac mapping includes both the Pegasus `SIMULATION_ENVIRONMENTS` catalog
  keys and our own Nucleus `omniverse://` URLs.
- The name→scene mapping is a **new config file under `simulation/`**.
- Follows the existing intent-flag discipline in `airstack.sh`: flags derive
  and export leaf env vars only (RFC #380 §4).

## 2. Proposed implementation

### 2.1 Scene catalog — `simulation/scenes.yaml` — `DONE`

New config file, the single source of truth for shortnames:

```yaml
scenes:
  construction-site:
    isaac:
      ref: omniverse://airlab-nucleus.andrew.cmu.edu:443/Public/AirStack/Stages/ConstructionSite/ConstructionSite.stage.usd
      stage_scale: 0.01     # UE-exported Nucleus stages are authored in cm
  warehouse:
    isaac: Warehouse         # bare string = Pegasus SIMULATION_ENVIRONMENTS key (or URL)
  blocks:
    msairsim: blocks         # fetch_scene.sh catalog key
```

Contents: all 15 Pegasus catalog keys, our 6 Nucleus stage variants, all 8
MS AirSim fetchable scenes (`fetch_scene.sh` keys). A scene may declare both
sims when a genuine counterpart exists (initially none do).

### 2.2 Resolver — `simulation/resolve_scene.py` — `DONE`

Host-side python3 (+PyYAML, already a host dep for fleet tooling):

- `resolve_scene.py --sim isaac|msairsim --scene NAME` → prints
  `KEY=VALUE` export lines: Isaac → `ISAAC_SIM_SCENE=<ref>` +
  `ISAAC_SIM_STAGE_SCALE=<scale>`; msairsim → `MS_AIRSIM_SCENE=<key>`.
- Unknown/unavailable scene → exit 1, availability table on stderr.
- `--table` → print the table (also used by help/error paths).

### 2.3 CLI — `airstack.sh` `--scene` intent flag — `DONE`

- `parse_launch_intent`: accept `--scene NAME` / `--scene=NAME`.
- `apply_launch_intent`: after sim-profile resolution, derive the target sim
  from `AIRSTACK_INTENT_SIM` or resolved `COMPOSE_PROFILES`
  (isaac-sim→isaac, ms-airsim→msairsim, simple→error: no scene support).
  Run the resolver; eval+export its output; on failure the table has already
  been printed → return 1.
- `--scene` with `--fleet`: exported `ISAAC_SIM_SCENE` overrides the fleet's
  `sim.scene` (fleet_spawn logs the override).
- Help text for `up`; `print_launch_config` gains the three new keys
  conditionally (only when set), preserving the byte-identical no-flag config.

### 2.4 Env plumbing — `.env` + compose files — `DONE`

- `.env`: commented documentation for `ISAAC_SIM_SCENE` / `MS_AIRSIM_SCENE`
  (defaults stay unset → current behavior byte-identical).
- `simulation/isaac-sim/docker/docker-compose.yaml`: pass
  `ISAAC_SIM_SCENE` and `ISAAC_SIM_STAGE_SCALE` into the container.
- `simulation/ms-airsim/docker/docker-compose.yaml`: pass `MS_AIRSIM_SCENE`.

### 2.5 Isaac launch scripts — env-driven scene — `DONE`

- `pegasus_app.py`: new helper `resolve_scene_from_env(simulation_environments,
  default_key="Default Environment")` → `(env_url, stage_scale)`:
  unset → default; catalog key → mapped URL; URL/`.usd*` path → verbatim;
  otherwise raise with the catalog keys listed (mirrors `fleet_env_url`).
- `example_one_px4_pegasus_launch_script.py`,
  `example_multi_px4_pegasus_launch_script.py`: use the helper.
- `fleet_spawn.py` `fleet_env_url`: `ISAAC_SIM_SCENE` env (when set) wins over
  the fleet file's `sim.scene`, with a printed note.

### 2.6 MS AirSim scene selection — `DONE`

- `fetch_scene.sh`: add `--name <key>` mode that prints the canonical dir
  NAME (e.g. `blocks` → `Blocks`) without downloading.
- `docker/entrypoint.sh`: when `MS_AIRSIM_BINARY_PATH` is unset, use
  `MS_AIRSIM_SCENE` (default `blocks`) to fetch and derive
  `/ms-airsim-env/<NAME>/LinuxNoEditor/<NAME>.sh`. Explicit
  `MS_AIRSIM_BINARY_PATH` still wins (warn if both set).

## 3. Test plan

- **(a) Resolver contract** — run `resolve_scene.py` host-side: valid isaac
  scene (pegasus key + nucleus URL cases), valid msairsim scene, unknown scene
  (exit 1 + table), scene valid for other sim only (exit 1 + table), `--table`.
  Pass: correct KEY=VALUE lines / exit codes / table lists every catalog entry.
- **(b) CLI dry-run matrix** — `airstack up --dry-run` with: `--sim isaac
  --scene construction-site` (effective config shows ISAAC_SIM_SCENE +
  ISAAC_SIM_STAGE_SCALE), `--sim airsim --scene blocks` (MS_AIRSIM_SCENE),
  `--sim isaac --scene blocks` (fails, table), `--scene nonsense` (fails,
  table), no `--scene` (effective config byte-identical to before the change).
  Pass: all as stated.
- **(c) Stage-scale ground truth** — read `metersPerUnit` from each of the 5
  Nucleus stage root layers (omni.client download + Sdf) to confirm the
  per-scene `stage_scale` values in scenes.yaml. Pass: yaml matches the layers.
- **(e) MS AirSim pipeline test (GPU)** — `airstack up --sim airsim
  --headless --scene blocks ms-airsim`: entrypoint resolves the binary from
  MS_AIRSIM_SCENE, UE4 boots, PX4 SITL reaches "Ready for takeoff", bridge
  node publishes /clock. Then `--scene soccer-field` (absent locally):
  host-side warn + in-container auto-fetch downloads, extracts, and boots the
  right binary. Pass: all milestones, correct scene name throughout.
- **(d) Isaac smoke test (best-effort, GPU)** — `airstack up --sim isaac
  --headless --scene construction-site`; check the isaac container log loads
  the Nucleus URL and reaches the play gate. Pass: stage URL appears in the
  Pegasus load line, no scene-load error.
