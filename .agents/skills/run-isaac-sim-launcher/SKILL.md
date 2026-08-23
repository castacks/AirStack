---
name: run-isaac-sim-launcher
description: Launch, relaunch and READ THE OUTPUT of an Isaac Sim standalone launch script (scene_gen scenes, the damage / fire / ground benches) through `airstack up isaac-sim` + `docker exec isaac-sim tmux ...`. Use whenever you need to see a scene, iterate on a launcher, pass SCENE_CONFIG / env vars / Kit settings, or find out why a launch died. `docker logs isaac-sim` is EMPTY for this container — this skill is how you actually see what the script printed.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Run an Isaac Sim launch script and read its output

## When to use

- Starting any `simulation/isaac-sim/launch_scripts/*_launch_script.py`
  (`scene_launch_script.py`, `suburb_mini_wildfire_launch_script.py`, the
  `*_test_launch_script.py` benches, `burn_ground_preview_launch_script.py`).
- Re-running a launcher after editing code, a YAML preset, a material or a
  texture — WITHOUT recreating the container.
- Reading what the launcher printed: scene stats, `[settle]` drop/spread,
  `[soot]` counts, readiness banners, Python tracebacks.
- Passing `SCENE_CONFIG`, launcher env knobs (`GROUND_ELAPSED`, `KEEP_PHYSICS`,
  `MINI_ELAPSED`, ...) or Kit/carb settings (`--/rtx/...`) to a run.

This is the Isaac-specific companion to [use-airstack-cli](../use-airstack-cli/SKILL.md).
Everything there about `docker exec <c> bash -c "..."` (never `-it`) still holds.

## How the container actually runs the script

`simulation/isaac-sim/docker/docker-compose.yaml` starts the container as

    tmux new -d -s isaac
    tmux send-keys -t isaac 'PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \
        /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/${ISAAC_SIM_SCRIPT_NAME} \
        --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts' ENTER
    sleep infinity

Three consequences that drive everything below:

1. **The process lives in a tmux pane, so `docker logs` / `airstack logs isaac-sim`
   show NOTHING.** The pane scrollback is the log.
2. **`ISAAC_SIM_SCRIPT_NAME`, `SCENE_CONFIG`, `AUTOLAUNCH` are read at container
   CREATION.** Changing them later needs either a recreate (`down` + `up`) or —
   far better — sending a new command line to the pane yourself.
3. **The container's env carries `SCENE_CONFIG` from `.env` (`downtown` by default)**
   and a launcher reads `os.environ` before its own default. Always pass
   `SCENE_CONFIG=` explicitly on the line you send.

The repo is bind-mounted at `/isaac-sim/AirStack`, so host edits are live in the
container; no copy, no rebuild.

## 1. Start the container

```bash
# from the repo root. `airstack` is a shell FUNCTION from `airstack setup`;
# in an agent shell it is usually not defined — ./airstack.sh is the same thing.
ISAAC_SIM_SCRIPT_NAME=burn_ground_preview_launch_script.py \
SCENE_CONFIG=suburb_mini_wildfire \
./airstack.sh up isaac-sim
```

- `up` is detached (`compose up -d`) and returns in seconds. Isaac is NOT ready
  when it returns.
- Verify a NEW container was made: `docker ps --format '{{.Names}}\t{{.CreatedAt}}'`
  — the CreatedAt must be now. If a container named `isaac-sim` already exists,
  `up` prints "Services brought up successfully" and does nothing: no new env,
  no new script, a crashed process is not restarted. (`airstack stop` leaves the
  container behind; only `down` removes it.)
- Container name is literally `isaac-sim` (`container_name:` in the compose
  file), not `airstack-isaac-sim-1`.

## 2. Read the output — three places, use all three

### a. Pane scrollback (what the script printed, last ~2000 lines)

```bash
docker exec isaac-sim tmux resize-window -t isaac -x 200 -y 50   # once; otherwise lines wrap at 80 cols
docker exec isaac-sim tmux capture-pane -p -J -t isaac -S -3000 | grep -v '^$' | tail -40
```

`-J` joins wrapped lines, `-S -3000` reaches back through history. The default
`history-limit` is **2000 lines** and a full block build prints far more than
that, so the START of a long run (the `[compile_disaster] compiled high-level
spec ...` line that tells you which preset was used) scrolls out. Hence (b).

### b. Pipe the pane to a file on the HOST (the real log; start it BEFORE launching)

```bash
docker exec isaac-sim tmux pipe-pane -t isaac -o \
    'cat >> /isaac-sim/.nvidia-omniverse/logs/isaac_pane.log'
# ... that path is the compose mount of $HOME/docker/isaac-sim/logs, so on the host:
tail -f ~/docker/isaac-sim/logs/isaac_pane.log
grep -n 'compile_disaster\|Traceback\|\[settle\]\|\[soot\]' ~/docker/isaac-sim/logs/isaac_pane.log
```

`pipe-pane` captures only from the moment it is started — start it right after
`up`, or right before you send a relaunch line, and it survives C-c / relaunch
(it is attached to the pane, not the process). `-o` makes it a no-op if a pipe
is already open, so it is safe to repeat. The file is outside the repo, so
nothing lands in git.

### c. The Kit log — the COMPLETE record, including the script's prints

```bash
docker exec isaac-sim bash -c \
  'D="/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1"; F="$D/$(ls -t "$D" | head -1)";
   grep "py stdout" "$F" | tail -40'          # only what the launcher print()ed, timestamped
docker exec isaac-sim bash -c \
  'D="/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1"; F="$D/$(ls -t "$D" | head -1)";
   grep -n "\[Error\]\|Traceback\|py stderr" "$F" | tail -20'
```

One file per launch, no line cap, and every `print()` from the script is
mirrored into it as `[omni.kit.app._impl] [py stdout]: ...` once the app is up.
So this, not the pane, is where to grep for `[compile_disaster]`, `[settle]`,
`[soot]` on a long run. It is also the only place that explains a SILENT pane:
after `app ready` the renderer compiles ray-tracing pipelines (`*** Waiting for
RtPso async group async compilation: N seconds so far`), printing nothing to the
pane — measured 133 s to `Simulation App Startup Complete` on a cold cache
versus 14 s on the relaunch. Note the directory is
`kit/logs/.../Isaac-Sim Python`, not the host-mounted `.nvidia-omniverse/logs`
(that one holds the GUI app's logs).

Which to use: the Kit log for "what happened" after the fact; the piped file
for a host-side `tail -f` while it runs (strip colour codes with
`sed 's/\x1b\[[0-9;]*m//g'` if you grep it); the pane for a quick look.

## 3. Know when it is READY (and whether it ran what you meant)

Startup, in order, as seen in the pane:

    Starting kit application with the following args: [...]
    Passing the following args to the base kit application: [...]   <- your --/ flags show up here
    [ext: ...] startup                                               <- hundreds of these
    [19.049s] app ready                                              <- KIT is ready; the SCENE is not
    (silence: shader compile — see the Kit log)
    [133.121s] Simulation App Startup Complete                       <- extensions + renderer up; prints start after this
    [compile_disaster] compiled high-level spec ...                  <- CHECK THE PRESET NAME
    [suburb_scene] ... / [settle] ... / [soot] ...                   <- the launcher's own prints
    ==========================================================
    <LAUNCHER BANNER>                                                <- every bench ends with one
    ==========================================================

So: grep the piped log for the banner (or for `Traceback`) rather than watching
the GPU. A run that ends with the banner is up and waiting in
`while simulation_app.is_running(): app.update()`.

## 4. Relaunch WITHOUT recreating the container (the iteration loop)

```bash
# 1. stop the running launcher
docker exec isaac-sim tmux send-keys -t isaac C-c
# 2. wait until the shell prompt is back (a few seconds; Kit shuts down cleanly)
docker exec isaac-sim tmux capture-pane -p -J -t isaac | grep -v '^$' | tail -1
#    -> '[DOCKER Isaac]root@<id>:~#'
# 3. drop the old run's scrollback, so `capture-pane` cannot hand you stale output as this run
docker exec isaac-sim tmux clear-history -t isaac
# 4. send the new line — `clear;` wipes the visible screen too (for a human attached to the pane)
docker exec isaac-sim tmux send-keys -t isaac \
  'clear; SCENE_CONFIG=suburb_mini_wildfire GROUND_ELAPSED=0 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/burn_ground_preview_launch_script.py --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --/rtx/raytracing/fractionalCutoutOpacity=true' ENTER
```

- Keep the line in SINGLE quotes on the host side: `$ISAAC_SIM_PYTHONPATH` must
  reach the pane unexpanded, because it is defined by the container's
  `.bashrc`, not by your host shell.
- Anything you put before `python.sh` is an env var for that run only; anything
  after the script name is passed to the script / to Kit. Unknown `--/a/b=c`
  arguments are forwarded to Kit as carb settings (they show up in the
  `Passing the following args...` line), which is how to flip a render setting
  without touching code. In a script the equivalent is
  `SimulationApp(launch_config={..., "extra_args": ["--/rtx/...=true"]})`.
  **Do not rely on `carb.settings.get_settings().set_bool(...)` after the app
  is up for `/rtx/...` settings**: ~12 s into startup `omni.usd-abi` maps each
  of them onto a USD render-settings property and the renderer reads the
  property from then on, so a late `set_bool` is never copied across — the
  fractional-cutout overlay silently vanished the run that tried it.
- Why not `down` + `up`: `down` REMOVES the container, so anything pip-installed
  into it at runtime (`manifold3d`, `shapely` for fracture) is gone and
  reinstalls on the next run; a process restart is ~2 min, a recreate is
  longer; and the piped log keeps going across a relaunch.

## 5. Running Python inside the LIVE app

There is no headless exec-into-Kit path. What exists:

- **Script Editor** (Window -> Script Editor) — a human at the GUI pastes the
  reload snippet from the launcher's docstring (`scene_launch_script.py`) or
  `exec(open("/isaac-sim/AirStack/scene_gen/reload_scene.py").read())`. Good
  for regenerating the procedural city in place; all the benches enable
  `omni.kit.window.script_editor` for this.
- **Do not hot-swap textures** under a running app to dodge a relaunch — it
  crashed Isaac (`Aborted (core dumped)`). Relaunch via section 4 instead.

## 6. Stopping — mostly, don't

- If a person is looking at the GUI, the running container IS the deliverable.
  Do not `timeout` the launch, do not `stop`/`down` it, and ask before
  relaunching over a scene they are inspecting.
- When a recreate is genuinely needed (new `AUTOLAUNCH`, a changed compose
  file): `./airstack.sh down isaac-sim`, then `up`. Never `stop`.

## Pitfalls

| Symptom | Cause | Fix |
|---|---|---|
| `airstack logs isaac-sim` / `docker logs` print nothing | the script runs inside tmux | `capture-pane` or the piped file (section 2) |
| pane silent for minutes after `app ready` | RtPso shader compile | Kit log shows `Waiting for RtPso ...`; wait |
| "250 m scene" builds 83 houses / 3,284 trees | container env `SCENE_CONFIG=downtown` beat the launcher default | pass `SCENE_CONFIG=` on the relaunch line; read the `[compile_disaster]` line |
| lines chopped at 80 columns in `capture-pane` | pane is 80x24 with no client attached | `tmux resize-window -t isaac -x 200 -y 50`, use `-J` |
| the beginning of the run is gone from `capture-pane` | 2000-line history-limit | grep the Kit log for `py stdout`, or `pipe-pane` to a file before launching |
| `up` says success, nothing changed | container already existed (`stop` instead of `down`) | `down`, then `up`; check CreatedAt |
| relaunch line fails with an empty PYTHONPATH | host shell expanded `$ISAAC_SIM_PYTHONPATH` | single-quote the whole `send-keys` argument |
| old run's output mistaken for the new run | stale scrollback | `tmux clear-history -t isaac` after C-c, and prefix the relaunch with `clear;` (`clear` alone leaves tmux history intact) |
| `ImportError: manifold3d` after a recreate | `down` destroyed runtime pip installs | relaunch via tmux, not `down`; or let `fracture.ensure_deps()` reinstall |
| `airstack: command not found` | it is a shell function from `airstack setup` | `./airstack.sh` from the repo root |

## Cheatsheet

```bash
# start (new container) — from the repo root
ISAAC_SIM_SCRIPT_NAME=<script>.py SCENE_CONFIG=<preset> ./airstack.sh up isaac-sim
docker ps --format '{{.Names}}\t{{.Status}}\t{{.CreatedAt}}'

# log to a host file, widen the pane
docker exec isaac-sim tmux pipe-pane -t isaac -o 'cat >> /isaac-sim/.nvidia-omniverse/logs/isaac_pane.log'
docker exec isaac-sim tmux resize-window -t isaac -x 200 -y 50

# read
docker exec isaac-sim tmux capture-pane -p -J -t isaac -S -3000 | grep -v '^$' | tail -40
tail -f ~/docker/isaac-sim/logs/isaac_pane.log
docker exec isaac-sim bash -c 'D="/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1"; tail -30 "$D/$(ls -t "$D" | head -1)"'

# relaunch
docker exec isaac-sim tmux send-keys -t isaac C-c            # prompt is back in ~30 s
docker exec isaac-sim tmux clear-history -t isaac
docker exec isaac-sim tmux send-keys -t isaac 'clear; SCENE_CONFIG=<preset> <ENV=val ...> PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/<script>.py --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts <--/carb/setting=value ...>' ENTER

# is it alive / what is it doing
docker exec isaac-sim bash -c 'pgrep -af launch_script | head -2; nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader'
```

## References

- [`simulation/isaac-sim/docker/docker-compose.yaml`](../../../simulation/isaac-sim/docker/docker-compose.yaml) — the exact tmux line, env and mounts
- [`simulation/isaac-sim/docker/.bashrc`](../../../simulation/isaac-sim/docker/.bashrc) — defines `ISAAC_SIM_PYTHONPATH`
- [`scene_gen/reload_scene.py`](../../../scene_gen/reload_scene.py) — in-app reload (Script Editor)
- Related skills: [use-airstack-cli](../use-airstack-cli/SKILL.md),
  [build-wildfire-scenes](../build-wildfire-scenes/SKILL.md),
  [write-isaac-sim-scene](../write-isaac-sim-scene/SKILL.md),
  [test-in-simulation](../test-in-simulation/SKILL.md)
