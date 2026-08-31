---
name: build-scenes-on-osmo
description: Build and render scene_gen scenes on an OSMO dev pod when the local Isaac Sim is busy or absent — submitting airstack-dev.yaml at the scene-authoring resource shape, getting UNCOMMITTED working-tree files onto the pod without a git push, driving Isaac there, and pulling rendered frames back. Read before running `airstack osmo:up` for scene work, or when a pod render behaves differently from a local one.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build scenes on an OSMO dev pod

The local Isaac Sim is often busy with a mission or another agent. A 500 m
scene build fits an OSMO workspace pod comfortably, and the pod has two RTX
PRO 5000s. This is the loop, and the traps, as actually used to build the
hurricane suburban scenes on 2026-08-30.

**Prerequisite:** [use-airstack-cli](../use-airstack-cli/SKILL.md) and
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md). Everything the
second one says about tmux, `SNAP_DIR` and `docker logs` being empty still
holds — with one exception this file spells out (see "Do not drive the pane").

## The loop

    airstack osmo:up --pool <pool>
    scene_gen/tools/osmo_provision.sh <wf-id> &     # unattended: steps 2-6
    scene_gen/tools/osmo_isaac.sh exec '...'        # run in the sim container
    scene_gen/tools/osmo_pull.sh <snapdir> <local-dir>
    airstack osmo:down                              # give the GPU back

**Run `osmo_provision.sh` in the background and walk away.** It waits for
SCHEDULING to clear (which can take a long time — the pod needs a worker with
2 GPUs, 80Gi and 500Gi free, and it queues behind everyone else), waits for
the inner Docker stack, opens the tunnel, checks the pod's clone against your
HEAD, syncs the missing assets in priority order, syncs the uncommitted code,
and then VERIFIES all of it with counts rather than assuming.

That script exists because this wait is too long to babysit and too long for a
subagent: asked to "wait for the pod", one gave up after three minutes and
reported that it would wait for a notification instead. The wait is the job,
so it belongs in a script rather than in an agent's patience.

The individual steps, if you need them by hand:

    airstack osmo:ide --no-open             # ssh tunnel on localhost:2200
    scene_gen/tools/osmo_sync.sh <files>    # push an EXPLICIT file list

`osmo:up` auto-pins `AIRSTACK_BRANCH` to your current local branch and warns
if it is not pushed. **The pod clones from GitHub**, so it starts from your
branch's ORIGIN tip — not your working tree.

## The resource shape is the scene-authoring one, not the mission one

`osmo/workflows/airstack-dev.yaml` asks for **2 GPU / 12 cpu / 80Gi / 500Gi**.

A scene build is one Isaac process plus the `scene_gen` driver: no robot
containers, no rayfronts, no ROS graph. Cores past ~12 sit idle because the
settle is single-threaded PhysX. The 500Gi is the number that actually
matters — the inner image set alone exceeds 100Gi extracted.

**`exec_timeout` is 20h and should stay long.** It was 8h and
`airstack-dev-175` timed out mid-session with the pod still in use. A short
timeout does not free the GPU politely — `airstack osmo:down` does, in one
command. A timeout is a backstop against a forgotten pod, not a scheduler.

## Getting UNCOMMITTED work onto the pod

This is the whole reason `scene_gen/tools/osmo_sync.sh` exists.

The pod only ever sees COMMITTED, PUSHED code. That is normally right — but
when several agents share this working tree, committing to get one file onto
the pod drags in everyone else's in-flight edits and creates a branch that
matches nothing anyone is testing. `osmo_sync.sh` rsyncs a NAMED FILE LIST
over the ssh tunnel at ~18 MB/s. Nothing is committed, nothing is pushed.

    scene_gen/tools/osmo_sync.sh scene_gen/disaster/surge.py <more files>
    scene_gen/tools/osmo_sync.sh --dir scene_gen/assets/aec/tower/Assets/Vegetation

## THE POD'S CLONE IS MISSING GIGABYTES OF ASSETS THAT ARE IN YOUR TREE

The single most expensive surprise. Two separate causes:

1. **`.gitignore:130` excludes `scene_gen/**/*.usda`** — every material
   definition. There is an exception for `scene_gen/assets/materials/*.usda`
   but it does NOT cover `megascans/`, so `Soil_Mud.usda` and friends are
   absent. Only 16 files, 64K, and the scene cannot texture its ground
   without them.
2. **The AEC packs and objaverse props are untracked entirely** — ~4.4 GB.

Symptoms, none of which say "missing asset": `[UsdToMdl] References an asset
that can not be found`, MDL `comp error: C120 could not find module`, fences
rendering as 4 x 4 m grey placeholder boxes, trees absent.

**Do not change `.gitignore`.** ssh-copy them; they are a temp copy on an
ephemeral pod. The set that mattered for a suburban scene:

    find scene_gen -name "*.usda"                      # 16 files, 64K
    scene_gen/assets/aec/tower/Assets/Vegetation       # 754M
    scene_gen/assets/aec/brownstone/Assets/Vegetation  # 792M
    scene_gen/assets/aec/brownstone/Materials          # 1.5G
    scene_gen/assets/aec/brownstone/Props
    scene_gen/assets/aec/brownstone/Assets/Create_Brownstone02
    scene_gen/assets/aec/tower/Assets/Bollard_01
    scene_gen/assets/objaverse                         # 419M

Most assets resolve from Nucleus (`asset_root` is
`omniverse://airlab-nucleus.../Library/Stages/`) and are fine. Only
`airstack://`-prefixed paths point at the repo. Enumerate them with:

    grep -rhoE "airstack://[A-Za-z0-9_/.-]+" scene_gen/ --include=*.py --include=*.yaml \
      | sed 's|airstack://||' | sed 's|/[^/]*$||' | sort -u

## Three container facts

1. **NEVER HARDCODE THE SIM CONTAINER'S NAME — resolve it.** It is
   `isaac-sim-livestream` under `COMPOSE_PROFILES=...,isaac-sim-livestream`
   and plain `isaac-sim` under `...,isaac-sim`, and **which one you get is
   decided by the COMMITTED `.env`, not by the workflow yaml.** Any session
   can change that file, and one did between `airstack-dev-175` and `-176`:
   every `docker exec` then returned *"No such container:
   isaac-sim-livestream"*, an unattended bake chain failed on the very first
   step, and twenty minutes were spent on nothing.

   `scene_gen/tools/osmo_isaac.sh` now resolves it per invocation (exact match
   first, then any running container matching `isaac-sim`, with
   `OSMO_SIM_CTNR` overriding). Any new script must do the same:

       CT=$(ssh … 'docker ps --format "{{.Names}}" | grep -m1 isaac-sim')

   The same `.env` also drives `NUM_ROBOTS` — this pod came up with EIGHT
   robot-desktop containers because the committed value said so, not because
   `airstack-dev.yaml` asked for one.
2. `/root/AirStack` on the pod is bind-mounted to `/isaac-sim/AirStack` in the
   container, so a synced file is visible to Isaac immediately. Prove it with
   a probe file the first time; do not assume.
3. **`SNAP_DIR` must sit under `/isaac-sim/.nvidia-omniverse/logs/...`**,
   which is the pod's `/root/docker/isaac-sim/logs`. Anywhere else and the
   frames land where the host cannot reach them.

## Do not drive the tmux pane. Run detached.

`run-isaac-sim-launcher` teaches `tmux send-keys`, and on a local container it
is fine. On the pod it cost several wasted cycles: **keystrokes go to whatever
is CURRENTLY in the pane.** Send a launch line while a bake is still running
and it is swallowed; the text then appears at the prompt afterwards with no
newline, looking exactly as though it ran. Meanwhile `ls` shows no output and
you conclude the launcher is broken.

Run detached with its own log instead:

    docker exec -d isaac-sim-livestream bash -lc \
      "PYTHONUNBUFFERED=1 SCENE_CONFIG=... SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/run1 \
       /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/<launcher>.py \
       --ext-folder /isaac-sim/.local/share/ov/data/documents/Kit/shared/exts \
       > /isaac-sim/.nvidia-omniverse/logs/run1.log 2>&1"

**`PYTHONUNBUFFERED=1` is not optional.** Python buffers stdout when it is not
a tty, so a launcher's whole banner can vanish while the run succeeds — a bake
wrote all 72 archetypes and printed nothing at all.

To stop a run: `pkill -9 -f <launcher>`. Ctrl-C into the pane does not
reliably reach a detached process, and a finished launcher sitting in
`while simulation_app.is_running()` ignores it.

## PhysX GPU dynamics DOES NOT ENGAGE on this pod

Measured 2026-08-30. `settle.py:1011` prints `"GPU" if gpu else "CPU"` — the
flag it was ASKED for, not the backend it got. `nvidia-smi` shows 0%
utilisation while the solver runs at ~1.6-2.3 CPU cores. A 6,863-body house
bake the wildfire skill costs at ~6 minutes ran past 25 and was still going.

Two mitigations, both worth taking:

* cut the budget — `SETTLE_STEPS=300 SETTLE_QUIET_STEPS=250
  SETTLE_MAX_STEPS=500` finished in ~20 min AND still audited clean (0.0%
  airborne, nothing below grade), so the default 1800/900 was over-budgeted
  for this content anyway;
* better, **design the ladder so most of it needs no settle at all.** The
  hurricane's three roof-damage states are `SetActive(False)` on per-bay roof
  meshes and bake in 4 SECONDS.

## Capture: the viewport path segfaults, use a render product

`utils/snapshots.py` captures the active viewport. The livestream container is
described by its own compose comment as *"Headless: no X server, no display,
no GUI window"*, so there is no window behind the viewport and the capture
dies with SIGSEGV inside `save_aov_to_file` — **after** the scene builds and
the ground truth is written, so the run looks 95% successful.

Use `utils/snapshots_rp.py`: same API (`overview`, `views_around`,
`place_camera`), but an explicit camera + a Replicator render product + the
`rgb` annotator. No viewport. Launchers should try it first and fall back.

**And check the pixels.** `rep.orchestrator.step()` returning is not a
guarantee the frame is ready: with a second Isaac process sharing the card,
eleven of twelve captures came back as uniform black 7.5 kB PNGs while every
one reported success. `snapshots_rp.snapshot` now retries unless
`arr[..., :3].std() > 1.0`. Do not run two assemblies on one card.

## Reading a render that looks wrong

One diagnostic is worth more than the rest combined:

> **If a large lighting change produces a BYTE-IDENTICAL image, the camera
> cannot see the lights.**

That single fact would have gone straight to the real bug (every tree
referenced 100x oversized, camera inside the bark) instead of a round chasing
the sky and then the water. Compare file sizes before theorising.

## Pod lifetime

The pod is ephemeral. When it ends — cancelled or timed out — everything
written on it is gone. **Pull frames as they are produced, not at the end**,
and treat any baked archetype library as reproducible rather than precious.
Code lives in your working tree; only outputs are at risk.

    airstack osmo:down            # deliberate, polite, one command
