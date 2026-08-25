---
name: run-isaac-sim-launcher
description: Launch, relaunch and READ THE OUTPUT of an Isaac Sim standalone launch script (scene_gen scenes, the damage / fire / ground benches) through `airstack up isaac-sim` + `docker exec isaac-sim tmux ...`. Use whenever you need to see a scene, iterate on a launcher, pass SCENE_CONFIG / env vars / Kit settings, or find out why a launch died. Also covers SEEING the scene — placing a review camera, writing viewport PNGs to a path the host can read (`utils/snapshots.py` + `SNAP_DIR`) — and talking to Nucleus (list / stat / upload with `omni.client`, standalone, no SimulationApp). `docker logs isaac-sim` is EMPTY for this container — this skill is how you actually see what the script printed.
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
- LOOKING at what it built: `SNAP_DIR=`, `utils/snapshots.py`, and getting the
  PNGs somewhere the host can read them (section 5).
- Listing, uploading or verifying anything on Nucleus — materials, stages,
  archetypes (section 6). That part needs no app and no pane, so it is safe
  while someone else is driving the container.

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
# 2b. IF THE SCRIPT SPAWNED DRONES, kill the orphaned PX4s. See below — this is
#     not optional, and skipping it costs a whole run.
docker exec isaac-sim bash -c 'pkill -9 -f px4_sitl_default/bin/px4; sleep 2'
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
### Ctrl-C ORPHANS PX4, and the next run fails in a way that blames the sim

Pegasus starts one `px4` process per drone as a CHILD of the Kit process, and
Ctrl-C on Kit does not take them with it. They keep running, keep holding their
instance directory and their uxrce/MAVLink ports, and keep answering MAVLink.

What that looks like on the next launch is not "a stale process":

- the pane sits on `[px4_mavlink_backend] Waiting for first hearbeat` forever
  and the scene never reaches its READY banner;
- the newly spawned `px4` shows up as `<defunct>` — it could not take the
  instance directory the orphan still holds;
- meanwhile **MAVROS in the robot container connects happily** and even prints
  the FCU version, because it found the ORPHAN. So `mavros/state` looks fine;
- but no `local_position/odom` ever arrives, so `TakeoffTask` is rejected with
  `state estimate timed out`, which reads like a takeoff-planner bug.

Check for them before blaming anything else:

```bash
docker exec isaac-sim bash -c 'pgrep -af px4_sitl_default/bin/px4'
```

Any PID older than the current Kit process is an orphan. `pkill -9 -f
px4_sitl_default/bin/px4` and relaunch. When in doubt with drones in the scene,
`down` + `up` the container — it is the only way that cannot leave one behind.

- Why not `down` + `up`: `down` REMOVES the container, so anything pip-installed
  into it at runtime (`manifold3d`, `shapely` for fracture) is gone and
  reinstalls on the next run; a process restart is ~2 min, a recreate is
  longer; and the piped log keeps going across a relaunch.

## 5. SEE the scene — cameras and snapshots

Sections 2 and 3 tell you the bench BUILT; only a picture tells you it built
the right thing. `simulation/isaac-sim/utils/snapshots.py` is the missing
sense: a camera prim the launcher aims, the active viewport retargeted to it,
and a PNG on the host. Every bench that has it is off by default and turns on
with one env var on the relaunch line (section 4) — this is the real invocation
of the last people-bench run, read back out of the piped pane log:

```bash
docker exec isaac-sim tmux send-keys -t isaac \
  'clear; SCENE_CONFIG=suburb_wildfire UNITS=A,C SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/people_ac PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/people_showcase_launch_script.py --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts' ENTER
```

The knob is `SNAP_DIR` in `people_showcase`, `car_occupants`,
`suburb_assemble`; `URBAN_SNAP_DIR` in the `urban_*` benches; `CAT_SNAP_DIR` in
`modern_city_catalogue`. Empty = no captures, and the run is otherwise
identical.

### SNAP_DIR must sit under the mounted log directory

    container   /isaac-sim/.nvidia-omniverse/logs/<name>
    host        ~/docker/isaac-sim/logs/<name>

That bind mount — the same one section 2b pipes the pane log into — is the ONLY
reason a capture is readable from outside the container. Point `SNAP_DIR` at
`/tmp/snaps` and the PNGs are written, announced in the log, and unreachable by
the agent that asked for them. Then just read the host path:

```bash
ls -l ~/docker/isaac-sim/logs/people_ac/          # A_obl.png A_top.png C_obl.png C_top.png row.png
grep -a '\[snapshots\] ->' ~/docker/isaac-sim/logs/isaac_pane.log | tail
```

### The API — `utils/snapshots.py`

*Signatures, defaults and rationale below are READ FROM SOURCE* —
`utils/snapshots.py` and the two benches that call it — *because the camera
calls need a live stage and this write-up did not start one.* What WAS checked
on disk: the container→host mapping, the 1280x720 frame, the root ownership and
the `row.png` framing failure, against the PNGs two separate runs left behind in
`~/docker/isaac-sim/logs/` (`people_ac/`, `cluster/`). To confirm the rest,
add a `SNAP_DIR=` to the next run you own and diff the PNG names against the
table.

Coordinates are STAGE units, not metres. Get the factor from
`scene_prep.get_stage_meters_per_unit(stage) -> (mpu, ssf)` and multiply: on a
cm stage `mpu=0.01`, `ssf=100`.

| call | what it does |
|---|---|
| `place_camera(stage, eye, target, focal_mm=18.0)` | defines/reuses `/World/ReviewCamera` (20.955 mm aperture, clip 0.5–20000), puts it at `eye` looking at `target` — both 3-tuples in stage units on a Z-up stage — and points the active viewport at it |
| `snapshot(path, frames=40)` | `hide_decorations()`, pump `frames` app updates so the ray-traced image converges, capture, pump 10 more; `makedirs` the parent for you |
| `views_around(stage, {name: (x, y)}, out_dir, ssf=1.0, top_h=60.0, obl_dist=45.0, obl_h=22.0, frames=40)` | `<name>_top.png` + `<name>_obl.png` per point. Takes METRES and applies `ssf` itself — the one entry point that does. The oblique looks from the south-west, which keeps the default sky's sun on the subject |
| `overview(stage, centre, span_m, out_path, ssf=1.0, frames=40)` | one plumb shot with the eye at `span_m * 0.95` |
| `hide_decorations()` | kills grid / outline / gizmos. `capture_viewport_to_file` grabs what the viewport DRAWS, so Kit's world reference grid lands in the PNG and reads as a defect in the scene — it was misdiagnosed twice, once on a bench with no overlay in it at all. Read the comment block above `_DECOR_SETTINGS` before writing your own capture |

Captures come out **1280x720** (measured on every PNG in
`~/docker/isaac-sim/logs/*`) and there is no resolution argument. More detail
means moving the camera, not asking for a bigger image.

### Choosing an eye and a target

At the default 18 mm on that 20.955 mm aperture the frame is **1.16 x the eye
height** wide and **0.65 x** tall (computed from the focal/aperture in
`place_camera` and the measured 1280x720; the `row.png` miss below is the
empirical check). So `h ≈ span / 1.16` frames `span` metres
across, and `overview(..., span_m, ...)` at `0.95 * span_m` covers `1.11 *
span_m` horizontally by `0.62 * span_m` vertically.

- **Top-down**: `eye=(x, y, h)`, `target=(x, y, 0)`. `_look_at` PINS the yaw to
  0 in this case so +X is right and +Y is up, the way a map reads — the
  `atan2(0, 0)` fallthrough quarter-turns the image, and a car pointing along
  +Y was once read off such a capture as pointing along +X and an asset's
  yaw-offset "fixed" against it.
- **Oblique**: `eye=(x-d, y-d, obl_h)` with `d = dist/√2`, `target=(x, y, ~1 m)`
  — aim at subject height, not at 0, or the subject sits at the top of the frame.
- **Close-ups scale off the measured object**, never a constant.
  `car_occupants` stands off at `length*1.25` at 30 mm because a fixed 5.2 m
  suits a coupe and puts the camera *inside* an 8.4 m van — the first van
  capture was a close-up of one door panel.
- **Frame the context, not just the subject.** `people_showcase` widened to
  `top_h=62 / obl_dist=52` deliberately: the burnt trees are what say
  "wildfire", and a shot tight enough to miss them is a shot of a car park.

### Import it BY FILE PATH, at the tail, inside a try

```python
if SNAP_DIR:
    try:
        import importlib.util as _ilu
        _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
        _spec = _ilu.spec_from_file_location("snapshots", _sp)
        _snaps = _ilu.module_from_spec(_spec)
        _spec.loader.exec_module(_snaps)
        _snaps.overview(stage, (20.0, 20.0), 300.0,
                        os.path.join(SNAP_DIR, "row.png"), ssf)
        _snaps.views_around(stage, {"C": (-20.0, 0.0)}, SNAP_DIR, ssf,
                            top_h=62.0, obl_dist=52.0, obl_h=26.0)
        print("[bench] snapshots -> {0}".format(SNAP_DIR))
    except Exception as exc:
        print("[bench] snapshots FAILED: {0}".format(exc))
```

That is the shape both benches use, lifted from
`people_showcase_launch_script.py`. Three deliberate choices, all worth copying:

1. **By path, not `import snapshots`** — the module's own docstring: Isaac's
   script runner does not reliably set `__file__`. The benches DO reach
   `scene_prep` / `scene_generator` through `sys.path.insert` at the top, so the
   by-path idiom is specifically about this late, optional import, which must
   not depend on a `sys.path` entry set 600 lines earlier still being intact.
2. **At the tail, after the banner** — `snapshots.py` does
   `import carb, omni.kit.app` and `from pxr import ...` at module level, and
   `omni.kit.app` / `pxr` only exist once `SimulationApp(...)` has built the Kit
   environment. That is the same reason every launcher's own `pxr` import sits
   BELOW its `SimulationApp(...)` call.
3. **Wrapped in `try`** — a failed capture must not take down a scene someone
   is looking at (section 8). The bench prints `snapshots FAILED: ...` and
   carries on into `while simulation_app.is_running()`.

`car_occupants` adds a fourth: flow needs TIME before it is photographed.
`timeline.play()` then 240 `app.update()`s, THEN the captures — the emitters
inject fuel per step, so a shot at t=0 is of an empty grid.

### What bites

- **A bench's `overview` frames the FULL station row even when `UNITS=` built a
  subset.** `people_showcase` hard-codes `overview(stage, (20, 20), 300.0, ...)`,
  which covers x ∈ [-146, 186]; with `UNITS=A,C` station A (x=-180) is 34 m off
  frame and `row.png` came back as an empty green field with the C pool a
  40-pixel rectangle near the middle. The per-unit `<UID>_top.png` /
  `<UID>_obl.png` are the useful ones for a subset run; `row.png` only means
  something on a full build.
- **The PNGs land root-owned.** The container runs as uid 0, so both the files
  (`root:root 644`) and the directory (`root:root 755`) are owned by root and a
  uid-1000 host shell cannot delete or overwrite them —
  `test -w ~/docker/isaac-sim/logs/people_ac` fails. Reading is fine. Clean up
  from inside: `docker exec isaac-sim rm -rf /isaac-sim/.nvidia-omniverse/logs/<name>`.
- **`[snapshots] -> path` prints before the file exists.**
  `capture_viewport_to_file` returns "a future-like object that can be awaited
  to ensure the capture completes" and `snapshot()` discards it, pumping 10
  frames instead. That is enough in a loop, but trust `ls -l` on the host over
  the print for the LAST capture before an exit.
- **Do not copy the inline camera code out of `urban_buildings`,
  `urban_showcase` or `modern_city_catalogue`.** Those carry a pre-`snapshots.py`
  `aim_camera` / `snapshot` pair that has neither `hide_decorations()` nor the
  plumb-view yaw pin — i.e. exactly the two bugs `snapshots.py` exists to fix.
  Import `utils/snapshots.py`.

## 6. Nucleus — list, upload and verify WITHOUT booting an app

`omni.client` runs standalone under the Kit python in this container — no
`SimulationApp`, no GPU, no pane. Measured **0.83 s** wall for `docker exec` +
connect + list of a real Nucleus folder, against the headless app boot that
`scene_gen/tools/nucleus_browse.py` and `nucleus_catalogue.py` still pay (their
docstrings say ~80 s; section 2c measured 133 s cold / 14 s warm for a full
launcher). Their shared premise — "`pxr`/`omni.client` only import after
SimulationApp has started" — is stale; the real blocker was a library path, see
below. Because it never touches the pane or the GPU, this is also the ONE thing
in this skill that is safe to run against a container someone else is using.

```bash
docker exec isaac-sim bash -c '
  EXT=/isaac-sim/kit/extscore/omni.client.lib
  LIB=$(ls -d /isaac-sim/extscache/omni.client-*/bin | head -1)
  LD_LIBRARY_PATH="/isaac-sim/kit:$LIB:$LD_LIBRARY_PATH" PYTHONPATH="$EXT" \
  /isaac-sim/kit/python/bin/python3 - <<PY
import omni.client as c
c.initialize()
r, entries = c.list("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/materials/megascans")
print(r, [(e.relative_path, e.flags) for e in entries])
c.shutdown()
PY'
```

Both env vars are load-bearing, and each fails differently:

| dropped | failure |
|---|---|
| `PYTHONPATH=/isaac-sim/kit/extscore/omni.client.lib` | `ModuleNotFoundError: No module named 'omni'` |
| `/isaac-sim/kit` on `LD_LIBRARY_PATH` | `ImportError: libcarb.so: cannot open shared object file: No such file or directory`, raised from `omni/client/impl/__init__.py` |

`libcarb.so` is at `/isaac-sim/kit/libcarb.so` and nowhere else on the default
path; that second error, not a real dependency on Kit, is what the "boot an app
first" folklore was actually describing. The `extscache` `bin` entry is belt
and braces — `_omniclient*.so` finds `libomniclient.so` by RPATH, so
`LD_LIBRARY_PATH=/isaac-sim/kit` alone lists and stats fine — but keep it: the
USD resolver below has no such RPATH.

Credentials need nothing: the image env carries `OMNI_USER=$omni-api-token`
and `OMNI_PASS`, and `initialize()` picks them up.

### API surface (all blocking wrappers over `*_with_callback`)

| call | returns / notes |
|---|---|
| `initialize()` / `shutdown()` | bracket the session |
| `list(url)` | `(Result, [ListEntry])`; entries have `.relative_path`, `.size`, `.flags` (`4` = folder, `515` = file — or test `flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN`) |
| `stat(url)` | `(Result, ListEntry)` — the existence/size check |
| `copy(src, dst, behavior=CopyBehavior.ERROR_IF_EXISTS)` | **the upload.** `src` is an absolute IN-CONTAINER path, `dst` an `omniverse://` URL. Handles a single file or a whole folder tree in one call, creating the destination folder |
| `create_folder(url)` | one level; unnecessary before a folder `copy` |
| `delete(url)` | file or whole tree |
| `move`, `read_file`, `write_file`, `get_server_info` | also present |

`copy` defaults to **ERROR_IF_EXISTS** — re-uploading a material you are
iterating on returns `Result.ERROR_ALREADY_EXISTS` and writes nothing. Pass
`behavior=c.CopyBehavior.OVERWRITE` when you mean to replace.

### Where things go

The repo is bind-mounted at `/isaac-sim/AirStack`, so a host path
`/home/<you>/SEI-COA/disaster-dataset/X` is `/isaac-sim/AirStack/X` inside —
that is the `copy` source, no `docker cp` needed.

The project asset root is

    omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA

which is what `airstack://` becomes when `AIRSTACK_ASSET_ROOT` is set to it, so
`airstack://scene_gen/assets/materials/megascans/Worn_Pavement.usda` lands at
`.../Projects/SEI-COA/scene_gen/assets/materials/megascans/Worn_Pavement.usda`.
`AIRSTACK_ASSET_ROOT` is NOT set in the container today (checked): by default
`scene_gen/scene_generator.py` resolves `airstack://` against the repo root,
i.e. the local bind mount. Uploading buys you (a) other machines and (b) a run
with `AIRSTACK_ASSET_ROOT=omniverse://...:443/Projects/SEI-COA` on the launch
line, which sources every asset off Nucleus instead.

### A megascans material is TWO uploads

    Worn_Pavement.usda      the OmniPBR wrapper  (~1.2–1.5 kB)
    Worn_Pavement/          T_<id>_2K_B.png, _N.png, _ORM.png  (~7–10 MB each)

because the wrapper names its maps RELATIVELY:

    asset inputs:diffuse_texture = @./Damaged_Asphalt/T_vizcebf_2K_B.png@

so the folder name must equal the wrapper stem exactly. Every material already
up there follows this (`Brick_Wall_Worn`, `Burnt_Forest_Floor`,
`Damaged_Asphalt`, `Road_Asphalt`, `Road_Line*`, `Road_Marking_Line`,
`Worn_Pavement`) — match it. Nucleus grows a `.thumbs/` of its own inside each
folder; that is server-side, not something you upload or copy back.

So the upload is two `copy` calls, one of them a folder:

```python
import omni.client as c
c.initialize()
SRC = "/isaac-sim/AirStack/scene_gen/assets/materials/megascans"      # the bind mount
DST = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA"
       "/scene_gen/assets/materials/megascans")
OW = c.CopyBehavior.OVERWRITE
print(c.copy(f"{SRC}/Worn_Pavement.usda", f"{DST}/Worn_Pavement.usda", behavior=OW))
print(c.copy(f"{SRC}/Worn_Pavement",      f"{DST}/Worn_Pavement",      behavior=OW))
c.shutdown()
```

*The `copy`/`create_folder`/`delete` semantics above — folder trees copied in
one call, the destination folder created for you, `ERROR_ALREADY_EXISTS`
without `OVERWRITE`, `delete` removing a whole tree — were verified live
against a throwaway `.../Projects/SEI-COA/_agent_probe` path, then deleted. The
material paths in this block are illustrative; the mechanism under them is not.*

### Verify the upload landed

`copy` returning `Result.OK` is necessary, not sufficient. Check size and then
open the stage:

```bash
docker exec isaac-sim bash -c '
  EXT=/isaac-sim/kit/extscore/omni.client.lib
  LD_LIBRARY_PATH="/isaac-sim/kit" PYTHONPATH="$EXT" /isaac-sim/kit/python/bin/python3 - <<PY
import omni.client as c
c.initialize()
B="omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/materials/megascans/"
print(c.stat(B+"Worn_Pavement.usda")[1].size)
print([ (e.relative_path, e.size) for e in c.list(B+"Worn_Pavement")[1] ])
c.shutdown()
PY'
```

against `stat -c%s` on the local originals. Then read the USD itself — also
standalone, also sub-second:

```bash
docker exec isaac-sim bash -c '
  U=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
  R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-*/ | head -1)
  C=/isaac-sim/kit/extscore/omni.client.lib
  LD_LIBRARY_PATH="/isaac-sim/kit:${U}bin:${R}lib:$C/bin" PYTHONPATH="$U:$C" \
  PXR_PLUGINPATH_NAME="${R}usd/omni_usd_resolver/resources" \
  /isaac-sim/kit/python/bin/python3 -c "
from pxr import Usd, UsdGeom
s = Usd.Stage.Open(\"omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/materials/megascans/Damaged_Asphalt.usda\")
print(UsdGeom.GetStageMetersPerUnit(s), [p.GetName() for p in s.Traverse()])
"'
# -> 0.01 ['Damaged_Asphalt', 'Shader']      in 0.8 s
```

`pxr` imports standalone too (`omni.usd.libs` in `extscache`), so the second
half of the `nucleus_catalogue.py` docstring is stale as well — but
`Usd.Stage.Open` on an `omniverse://` URL additionally needs
`PXR_PLUGINPATH_NAME` pointed at the OmniUsdResolver resources AND
`omni.client.lib/bin` on the library path, or the plugin load fails with
`libomniclient.so: cannot open shared object file` and USD reports only
`Failed to open layer @omniverse://...@`. Kit normally sets both up, which is
why booting an app "works".

`scene_gen/tools/nucleus_browse.py` (recursive listing) and
`nucleus_catalogue.py` (bbox / prim tree / walk, driven by a JSON job file)
still exist and are the right tools for a deep crawl — neither uploads, and
both pay the app boot.

## 7. Running Python inside the LIVE app

There is no headless exec-into-Kit path. What exists:

- **Script Editor** (Window -> Script Editor) — a human at the GUI pastes the
  reload snippet from the launcher's docstring (`scene_launch_script.py`) or
  `exec(open("/isaac-sim/AirStack/scene_gen/reload_scene.py").read())`. Good
  for regenerating the procedural city in place; all the benches enable
  `omni.kit.window.script_editor` for this.
- **Do not hot-swap textures** under a running app to dodge a relaunch — it
  crashed Isaac (`Aborted (core dumped)`). Relaunch via section 4 instead.

## 8. Stopping — mostly, don't

- If a person is looking at the GUI, the running container IS the deliverable.
  Do not `timeout` the launch, do not `stop`/`down` it, and ask before
  relaunching over a scene they are inspecting.
- When a recreate is genuinely needed (new `AUTOLAUNCH`, a changed compose
  file): `./airstack.sh down isaac-sim`, then `up`. Never `stop`.

## Two ways a CAPTURE lies to you

**A straight-down shot is rotated a quarter turn.** `snapshots._look_at` takes
its yaw from `atan2(dy, dx)`, and a plumb camera has `dx = dy = 0` — which
returns 0, so the yaw came out -90 and world +X ran UP the frame. Nothing
about the picture looks wrong. A car correctly pointing along +Y was read off
one of these and an asset's `yaw-offset` was "corrected" against it, which
then had to be reverted. The yaw is pinned to 0 for a plumb view now; the
lesson is to **measure a bearing from the SIDE**, where a head-on car cannot
be confused with a broadside one.

**Kit's viewport grid is IN the capture.** `capture_viewport_to_file` grabs
what the viewport draws, decorations included — a regular lattice over the
whole frame. It was diagnosed as a defect in the SCENE twice (once as a
burn-scar overlay printing a 3 m grid across the plat, and a fix was written
for it) before a bench whose ground is plain grass showed the same grid and
proved it was the viewport. `snapshots.hide_decorations()` clears grid,
selection outline and gizmos before every shot; it runs from `snapshot()`.

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
| snapshots taken but no PNG anywhere on the host | `SNAP_DIR` outside the mounted log tree | put it under `/isaac-sim/.nvidia-omniverse/logs/<name>` = `~/docker/isaac-sim/logs/<name>` |
| the overview / `row.png` is an empty field | the bench's `overview()` frames the FULL station row; `UNITS=` built a subset | read the per-unit `<UID>_top.png` / `_obl.png`; `row.png` only means something on a full build |
| cannot delete or overwrite a PNG under `~/docker/isaac-sim/logs/` | the container runs as uid 0, so files AND the directory are `root:root` | `docker exec isaac-sim rm -rf /isaac-sim/.nvidia-omniverse/logs/<name>` |
| the last capture is missing though `[snapshots] -> ...` printed | `capture_viewport_to_file` returns a future that `snapshot()` discards | trust `ls -l` on the host, not the print; keep the trailing `app.update()` pump |
| a top-down capture is quarter-turned, or Kit's grid is baked into the PNG | a launcher's own inline camera code (`urban_*`, `modern_city_catalogue`) predates `snapshots.py` | import `utils/snapshots.py`; do not copy the inline pair |
| `ModuleNotFoundError: No module named 'omni'` under `kit/python/bin/python3` | `PYTHONPATH=/isaac-sim/kit/extscore/omni.client.lib` missing | use the section 6 invocation verbatim |
| `ImportError: libcarb.so: cannot open shared object file` | `/isaac-sim/kit` missing from `LD_LIBRARY_PATH` | same; this error — not a real Kit dependency — is why the repo's Nucleus tools boot an app |
| `omni.client.copy` returns `Result.ERROR_ALREADY_EXISTS` | default `CopyBehavior.ERROR_IF_EXISTS` | `behavior=omni.client.CopyBehavior.OVERWRITE` when replacing |
| a Nucleus material renders untextured | the `<Name>/` texture folder was not uploaded next to `<Name>.usda`, or was renamed | the wrapper names its maps as `@./<Name>/T_*.png@` — folder stem must match |

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

# snapshots: add SNAP_DIR to the relaunch line, under the MOUNTED log dir, then read them on the host
#   ... SCENE_CONFIG=<preset> SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/<name> PYTHONPATH="$ISAAC_SIM_PYTHONPATH" ...
ls -l ~/docker/isaac-sim/logs/<name>/                     # root-owned 1280x720 PNGs
grep -a '\[snapshots\] ->' ~/docker/isaac-sim/logs/isaac_pane.log | tail
docker exec isaac-sim rm -rf /isaac-sim/.nvidia-omniverse/logs/<name>   # they are root-owned

# Nucleus WITHOUT a SimulationApp (~0.8 s): list / stat / upload / verify
docker exec isaac-sim bash -c '
  EXT=/isaac-sim/kit/extscore/omni.client.lib
  LIB=$(ls -d /isaac-sim/extscache/omni.client-*/bin | head -1)
  LD_LIBRARY_PATH="/isaac-sim/kit:$LIB:$LD_LIBRARY_PATH" PYTHONPATH="$EXT" \
  /isaac-sim/kit/python/bin/python3 -c "
import omni.client as c
c.initialize()
B=\"omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/materials/megascans/\"
r, es = c.list(B); print(r, [e.relative_path for e in es])
# c.copy(\"/isaac-sim/AirStack/scene_gen/assets/materials/megascans/X.usda\", B+\"X.usda\", behavior=c.CopyBehavior.OVERWRITE)
# c.copy(\"/isaac-sim/AirStack/scene_gen/assets/materials/megascans/X\",      B+\"X\",      behavior=c.CopyBehavior.OVERWRITE)
c.shutdown()"'
```

## References

- [`simulation/isaac-sim/docker/docker-compose.yaml`](../../../simulation/isaac-sim/docker/docker-compose.yaml) — the exact tmux line, env and mounts
- [`simulation/isaac-sim/docker/.bashrc`](../../../simulation/isaac-sim/docker/.bashrc) — defines `ISAAC_SIM_PYTHONPATH`
- [`scene_gen/reload_scene.py`](../../../scene_gen/reload_scene.py) — in-app reload (Script Editor)
- [`simulation/isaac-sim/utils/snapshots.py`](../../../simulation/isaac-sim/utils/snapshots.py) — the camera / capture API; its comment blocks hold the grid and plumb-yaw stories
- [`simulation/isaac-sim/launch_scripts/people_showcase_launch_script.py`](../../../simulation/isaac-sim/launch_scripts/people_showcase_launch_script.py) and [`car_occupants_launch_script.py`](../../../simulation/isaac-sim/launch_scripts/car_occupants_launch_script.py) — the tails show the by-path import and the framing choices
- [`scene_gen/scene_generator.py`](../../../scene_gen/scene_generator.py) — `airstack://` / `AIRSTACK_ASSET_ROOT` resolution
- [`scene_gen/tools/nucleus_browse.py`](../../../scene_gen/tools/nucleus_browse.py), [`nucleus_catalogue.py`](../../../scene_gen/tools/nucleus_catalogue.py) — recursive listing and USD measurement; read-only, and their "boot an app first" docstrings are stale (section 6)
- Related skills: [use-airstack-cli](../use-airstack-cli/SKILL.md),
  [build-wildfire-scenes](../build-wildfire-scenes/SKILL.md),
  [write-isaac-sim-scene](../write-isaac-sim-scene/SKILL.md),
  [test-in-simulation](../test-in-simulation/SKILL.md)
