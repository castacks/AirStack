#!/usr/bin/env python3
"""
Spawn PX4 drones into a scene. The scene comes from ONE of THREE sources.

  ENV_URL (the default)  a finished USD pulled off Nucleus by
                         `pg.load_environment`. This is the historical path and
                         nothing about it changes.
  SCENE_CONFIG=<preset>  a GENERATED scene: `scene_gen/scene_api.build_scene`
                         authors the whole plat into the stage in-process —
                         layout, fire field, damage archetypes, ground scar,
                         survivors. Nothing is loaded and NOTHING IS RESCALED.
  FROZEN_SCENE=<path>    a FROZEN dataset cell: one of the `.usd` files under
                         `final_disaster_dataset/`, referenced in as it stands.
                         Same plat as the generated path — same frame, same
                         metres, same `/World/stage/generated` prim paths —
                         but ALREADY BUILT, so a run costs a file open instead
                         of a twenty-minute assembly, and every arm flies
                         bit-identical geometry.

`_GENERATED` still means "a scene_gen plat": authored in metres, never
rescaled, drone-flown, overhead camera framed on the whole plate. Both new
sources are that. `_BUILT` is the narrower switch — "assemble it in-process" —
and guards only the things that assembly needs (the archetype bake, the spec
overrides, the fire clock, the people plan). `_FROZEN` guards the reference.

See `.agents/skills/launch-generated-scene-with-drones/SKILL.md` for the
generated path's env knobs and traps, `.agents/skills/freeze-disaster-dataset`
for what a frozen cell is, and `.agents/skills/benchmark-disaster-dataset` for
how a mission drives either one.
"""

import os

import carb
from isaacsim import SimulationApp

SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "").strip()
# A frozen cell is a finished file, so it WINS over SCENE_CONFIG rather than
# being combined with it: a mission that sets both is asking for one scene and
# would otherwise get a procedural plat authored on top of a frozen one, which
# renders as two suburbs in the same square kilometre.
FROZEN_SCENE = os.environ.get("FROZEN_SCENE", "").strip()
_FROZEN = bool(FROZEN_SCENE)
_BUILT = bool(SCENE_CONFIG) and not _FROZEN
_GENERATED = _BUILT or _FROZEN
if _FROZEN and SCENE_CONFIG:
    print("[scene] FROZEN_SCENE is set, so SCENE_CONFIG={0!r} is IGNORED — the "
          "cell is loaded as frozen, not rebuilt".format(SCENE_CONFIG),
          flush=True)

# fractionalCutoutOpacity: this renderer forces cutout opacity to 1.0 unless
# asked otherwise, which makes car glass opaque and hides every occupant the
# people pass put inside a vehicle. Both raster and path-traced paths need it,
# and the flag has to be BOTH a startup arg and a carb setting re-asserted
# after the stage is composed (see PegasusApp._build_generated_scene).
_CUTOUT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

# MULTI-GPU RENDERING (opt-in). Isaac distributes MULTI-CAMERA rendering across
# GPUs — the drones' ZED render products spread over the visible cards, which is
# the multi-drone bottleneck (PhysX stays on the primary device and is not the
# limit for a handful of drones). Off by default, so single-GPU missions are
# byte-for-byte unchanged.
#   ISAAC_SIM_MULTIGPU=N     enable, cap the pool at N GPUs (maxGpuCount)
#   ISAAC_SIM_MULTIGPU=true  enable, auto-allocate over every visible GPU
# NEEDS the cards visible too: ISAAC_SIM_CUDA_DEVICES="0,1[,...]" (-> the
# container's CUDA_VISIBLE_DEVICES) and a workflow that requests >1 GPU
# (osmo/workflows/airstack-mission-2gpu.yaml). currentGpuCount is the readback
# that confirms it took (see the benchmark scripts).
_MULTIGPU = os.environ.get("ISAAC_SIM_MULTIGPU", "").strip().lower()
_MULTIGPU_ARGS = []
if _MULTIGPU and _MULTIGPU not in ("0", "1", "false", "off", "no"):
    _MULTIGPU_ARGS = ["--/renderer/multiGpu/enabled=true",
                      "--/renderer/multiGpu/autoEnable=true"]
    if _MULTIGPU.isdigit():
        _MULTIGPU_ARGS.append(f"--/renderer/multiGpu/maxGpuCount={int(_MULTIGPU)}")
    print(f"[isaac] multi-GPU rendering ON (ISAAC_SIM_MULTIGPU={_MULTIGPU}, "
          f"CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES', '<unset>')}): "
          f"{_MULTIGPU_ARGS}", flush=True)
else:
    # OFF MUST BE SAID OUT LOUD. Leaving it unset does NOT mean single-GPU:
    # SimulationApp's own default command line carries
    # `--/renderer/multiGpu/enabled=True`, so Kit tries to bind a render
    # context on EVERY device NVML reports.
    #
    # That is fatal when NVML and CUDA disagree, which is the normal state of
    # an OSMO pod. Measured on airstack-mission-1gpu-52 (2026-08-31): the
    # container reserves one GPU, but is handed four device nodes
    # (/dev/nvidia0..3, NVML count 4) while CUDA_VISIBLE_DEVICES permits one.
    # Kit then logs, six times, three seconds into startup:
    #
    #     [gpu.foundation.plugin] Skipping NVIDIA GPU due CUDA being in bad
    #     state: NVIDIA RTX PRO 5000 Blackwell
    #     [gpu.foundation.plugin] Please restart your system if CUDA is known
    #     to work in your system.
    #
    # and drops EVERY card, including the one that works — `cuInit(0)` returns
    # CUDA_SUCCESS in that same container. It then renders in software: both
    # GPUs at 0 % / 4 MiB / P8 for a whole run while Isaac burns 273 % CPU,
    # and the real-time factor collapses to ~0.083 — 600 sim-seconds took
    # ~2 hours, making a 24-iteration sweep ~48 h instead of a few.
    #
    # NVIDIA's own guidance for this symptom is to disable multi-GPU
    # rendering or pin to one device; setting NVIDIA_VISIBLE_DEVICES does NOT
    # help here, because the runtime injects all four device nodes regardless
    # (verified: NVML still reported 4 with NVIDIA_VISIBLE_DEVICES=0).
    _MULTIGPU_ARGS = ["--/renderer/multiGpu/enabled=false"]
    print("[isaac] multi-GPU rendering OFF (explicit; Kit's default is True, "
          "which makes it probe every NVML device and disable the GPU "
          "entirely when CUDA cannot open them). "
          f"CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES', '<unset>')}",
          flush=True)

# ACTIVE GPU (single-GPU pin, for the RENDERER specifically). NVIDIA_VISIBLE_
# DEVICES / CUDA_VISIBLE_DEVICES do NOT affect Vulkan (confirmed against
# Omniverse's own Linux troubleshooting doc: "Environment variables like
# CUDA_VISIBLE_DEVICES have no effect on Vulkan applications") — the renderer
# picks its device independently, via `--/renderer/activeGpu=<index>`, and
# that index is Kit's OWN gpu.foundation device table (this pod: it matches
# nvidia-smi/CUDA 1:1 — "Device N: CUDA device index: N" in the Kit log — but
# that is NOT guaranteed on every pod; check the log's `[gpu.foundation]`
# "Device N: ... CUDA device index: M" lines if this ever needs re-deriving).
#
# Root-caused 2026-09-02 on airstack-mission-1gpu-56: with NO activeGpu pin,
# renderer defaults to device 0 regardless of NVIDIA_VISIBLE_DEVICES/
# CUDA_VISIBLE_DEVICES — on a shared OSMO host where device 0 is a different,
# heavily-loaded tenant's card (98% util measured), the renderer's resource
# upload hangs (`ResourceLoader::endAndSubmit - Failed to wait for fence`,
# `Failed to upload font texture`) and Kit segfaults minutes into scene setup,
# at a time that varies with how contended device 0 happens to be (225s-411s
# measured across 3 reproductions) — easy to misread as a code/timing bug.
# Neither NVIDIA_VISIBLE_DEVICES nor CUDA_VISIBLE_DEVICES fixed it (former is
# ignored by this host's runtime for enumeration purposes; latter genuinely
# restricts CUDA but has zero effect on the Vulkan renderer that actually
# crashed). Set ISAAC_SIM_ACTIVE_GPU to this pod's OWN verified device index
# (via `nvidia-smi -L` at the pod's own shell to get the UUID, then
# `docker exec isaac-sim nvidia-smi -L` to find its index inside the
# container) whenever device 0 is not confirmed to be this pod's own card.
_ACTIVE_GPU = os.environ.get("ISAAC_SIM_ACTIVE_GPU", "").strip()
_ACTIVE_GPU_ARGS = []
if _ACTIVE_GPU:
    _ACTIVE_GPU_ARGS = [f"--/renderer/activeGpu={_ACTIVE_GPU}"]
    print(f"[isaac] renderer pinned to GPU index {_ACTIVE_GPU} "
          "(ISAAC_SIM_ACTIVE_GPU)", flush=True)

# Optional rendering-cost controls. All are intentionally opt-in: an unset
# environment reproduces the established benchmark renderer exactly.
_DLSS_NAMES = {"performance": 0, "balanced": 1, "quality": 2, "auto": 3}
_DLSS_MODE = os.environ.get("ISAAC_SIM_DLSS_MODE", "").strip().lower()
if _DLSS_MODE and _DLSS_MODE not in _DLSS_NAMES:
    raise ValueError("ISAAC_SIM_DLSS_MODE must be performance, balanced, quality, or auto")
_RTX_TUNING_ARGS = ([] if not _DLSS_MODE else
                    [f"--/rtx/post/dlss/execMode={_DLSS_NAMES[_DLSS_MODE]}"])
if os.environ.get("ISAAC_SIM_DISABLE_MOTION_BVH", "false").lower() == "true":
    _RTX_TUNING_ARGS += [
        "--/renderer/raytracingMotion/enabled=false",
        "--/renderer/raytracingMotion/enableHydraEngineMasking=false",
        "--/renderer/raytracingMotion/enableInstanceInPointInstancer=false",
    ]
_DISABLE_VIEWPORT_UPDATES = (
    os.environ.get("ISAAC_SIM_DISABLE_VIEWPORT_UPDATES", "false").lower() == "true")
if _RTX_TUNING_ARGS or _DISABLE_VIEWPORT_UPDATES:
    print(f"[isaac] optional RTX tuning: args={_RTX_TUNING_ARGS}, "
          f"disable_viewport_updates={_DISABLE_VIEWPORT_UPDATES}", flush=True)

_LIVESTREAM = _GENERATED and os.environ.get(
    "ISAAC_SIM_LIVESTREAM", "").lower() == "true"

# Start Isaac Sim's simulation environment (Must start this before importing omni modules)
if not _GENERATED:
    simulation_app = SimulationApp({"headless": False})
elif _LIVESTREAM:
    # Mirrors the NVIDIA reference config — headless with the UI kept VISIBLE,
    # so the Kit GUI is what renders into the WebRTC stream. See
    # example_one_px4_pegasus_launch_script.py for the full rationale.
    simulation_app = SimulationApp(launch_config={
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
        "headless": True,
        "hide_ui": False,
        "renderer": "RaytracedLighting",
        "display_options": 3286,
        "extra_args": _CUTOUT_ARGS + _MULTIGPU_ARGS + _ACTIVE_GPU_ARGS + _RTX_TUNING_ARGS,
    })
else:
    simulation_app = SimulationApp(launch_config={
        "headless": os.getenv("ISAAC_SIM_HEADLESS", "false").lower() == "true",
        "disable_viewport_updates": _DISABLE_VIEWPORT_UPDATES,
        "extra_args": _CUTOUT_ARGS + _MULTIGPU_ARGS + _ACTIVE_GPU_ARGS + _RTX_TUNING_ARGS,
    })

if _GENERATED:
    from isaacsim.core.utils.extensions import enable_extension

    if _LIVESTREAM:
        # Pin the UDP media port to the single port the isaac-sim-livestream
        # profile publishes; Kit 107 otherwise picks a dynamic one outside the
        # forwarded set and the viewport stays black.
        simulation_app.set_setting("/app/window/drawMouse", True)
        simulation_app.set_setting("/app/livestream/enabled", True)
        _LS_PORT = int(os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
        simulation_app.set_setting("/app/livestream/fixedHostPort", _LS_PORT)
        simulation_app.set_setting("/app/livestream/minHostPort", _LS_PORT)
        simulation_app.set_setting("/app/livestream/maxHostPort", _LS_PORT)
        enable_extension("omni.kit.livestream.webrtc")

    # Flow supplies the plumes over the road blockages. Enable before the scene
    # modules import, since `disaster.fire` touches Flow prim types.
    enable_extension("omni.flowusd")

# Set local Nucleus as asset root before importing Pegasus (which resolves it at import time)
# Unrelated to AIRSTACK_ASSET_ROOT: this one is NVIDIA's Isaac asset tree,
# which is where Pegasus' default environment comes from.
carb.settings.get_settings().set(
    "/persistent/isaac/asset_root/default",
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
)

if _GENERATED:
    # pymavlink waits on its socket with select.select(), which cannot handle
    # file descriptors >= 1024. DDS-over-TCP (fastdds LARGE_DATA) opens a TCP
    # connection per remote participant, pushing this process's fd numbers past
    # that, and every MAVLink read then throws "filedescriptor out of range in
    # select()". poll() has no fd-number limit. Must run before Pegasus creates
    # its MAVLink connections. Scoped to this path so the ENV_URL path stays
    # exactly what it was.
    import select as _select
    import time as _time
    from pymavlink import mavutil as _mavutil

    def _mavfile_select_poll(self, timeout):
        if self.fd is None:
            _time.sleep(min(timeout, 0.5))
            return True
        poller = _select.poll()
        poller.register(self.fd, _select.POLLIN)
        try:
            return len(poller.poll(timeout * 1000)) > 0
        except OSError:
            return False

    _mavutil.mavfile.select = _mavfile_select_poll

import json
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd
import omni.client

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

# gps_utils lives in the same directory as this script
_LAUNCH_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_SCRIPTS_DIR)
from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import (
    scale_stage_prim, add_colliders, add_dome_light, get_stage_meters_per_unit,
    reference_root_prims_under_world, dedupe_physics_scenes,
    add_orthographic_camera, add_overhead_camera_publisher,
)

if _GENERATED:
    from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
    from spawn_utils import generate_spawn_configs

if _BUILT:
    # scene_gen goes on sys.path only for the BUILT path, and only after
    # everything alongside this script has been imported: it drops a dozen bare
    # module names (`layout`, `detail`, `disaster`, `scene_generator`) at the
    # front of the search order, and there is no reason to hand the ENV_URL
    # path — or a frozen cell, which needs none of it — that exposure.
    sys.path.insert(0, os.path.normpath(os.path.join(
        _LAUNCH_SCRIPTS_DIR, "..", "..", "..", "scene_gen")))
    from scene_api import build_scene, LOCAL_ARCH_DIR
    from disaster import people as ppl
    from disaster import region as region_poly

if _FROZEN:
    # Stdlib-only (see its docstring), so this costs nothing and needs no
    # scene_gen on the path.
    import frozen_annotations as frozen_gt
    from pxr import Sdf


# --------------------- CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"

#env/stage path and scale
_LOCAL_SCENES_DIR = os.path.normpath(os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "assets", "scenes"))
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/Shipyard.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/scenes/urban/allegheny_county_fire_academy/fire_academy.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/RetroNeighborhood/RetroNeighborhood.stage.usd" # scale = 0.01
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/AbandonedFactory/AbandonedFactory.stage.usd" # scale=1.0
# ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/ConstructionSite/ConstructionSite.stage.usd" # scale = 0.01
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/AbandonedCity.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/downtown_edited_v3_818.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/environments_start_pos/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z_90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/edit_v1_shipyard.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/ModernCityDowntown.stage.usd"

ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Muyang/LevelTest2.usd" # scale = 0.01

STAGE_SCALE = 0.01

# --------------------- GENERATED SCENE (SCENE_CONFIG) ---------------------
# Only read when SCENE_CONFIG is set (and FROZEN_SCENE is not). `AIRSTACK_ASSET_ROOT` repoints
# `airstack://` onto Nucleus for pods whose clone carries no local AEC tree;
# `scene_generator` reads it at IMPORT time, so it must already be in the
# environment when this process starts — nothing here writes it.
if _GENERATED:
    # Pegasus' flat default env, NOT a finished scene: it exists only to give
    # PegasusInterface a World with a PhysicsScene to hang the plat off.
    #
    # THE FROZEN PATH DOES NOT LOAD IT. A frozen cell was exported from a stage
    # that had already loaded it, so `/World/stage` inside the file ALREADY
    # carries the default environment's GroundPlane and SphereLight, composed
    # in. Loading it again and then referencing the cell onto the same prim
    # would put two ground planes and two lights at z = 0.
    BASE_ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
    SCENE_PARENT = "/World/stage/generated"

if _BUILT:
    SEED = int(os.environ.get("MINI_SEED", "11"))
    # None -> scene_api.default_arch_dir(): the local bake if it exists, else
    # the copy under AIRSTACK_ASSET_ROOT on Nucleus. A URL is fine — the bake
    # directory is untracked, so a pod's fresh clone has none, and
    # `os.listdir` cannot enumerate `omniverse://`.
    ARCH_DIR = os.environ.get("ARCH_DIR") or None
    # Survivor ground truth is written with plain `open()`, so it lands on the
    # FILESYSTEM even when the bake it is named after lives on Nucleus.
    PEOPLE_JSON = os.environ.get("PEOPLE_JSON") or os.path.join(
        LOCAL_ARCH_DIR, "humans_{0}.json".format(SEED))
    # REVEALS the survivor markers; it does not create them. They are authored
    # deactivated on every run, so a searcher never sees them by default and
    # anyone who wants to look flips `<parent>/_people_poles` active in the
    # stage tree instead of re-assembling the plat.
    POLES = os.environ.get("PEOPLE_POLES", "").strip().lower() in (
        "1", "true", "yes")
    BURN_FRAC = float(os.environ.get("MINI_BURN_FRAC", "0.45"))
    ELAPSED = float(os.environ.get("MINI_ELAPSED", "0"))

    # SPEC OVERRIDES. A preset names a KIND of scene (suburb, downtown); these
    # pick the instance of it, so "a 250 m undamaged suburb" needs no preset
    # file of its own:
    #
    #   SCENE_CONFIG=suburb REGION_M=250x250 DISASTER_TYPE=none
    #
    # Merged into the high-level spec BEFORE it compiles, so DISASTER_TYPE=none
    # compiles to a config with no disaster rather than one the launcher has to
    # remember to skip. Unset leaves the preset's own value alone.
    def _region_m(raw):
        """REGION_M as "250", "250x250" or "250,250" -> [w, h] metres.

        A bare number is square: "250" and "250x250" mean the same thing to
        anyone asking for a 250 m block.
        """
        if not raw:
            return None
        parts = [p for p in raw.replace("x", ",").replace("X", ",").split(",")
                 if p.strip()]
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            raise SystemExit("REGION_M={0!r}: expected N, NxN or N,N".format(raw))
        # Delimiter-only input ("x", ",", ",,") filters down to NOTHING. Without
        # this it returned [], which is not None, so it was applied as an
        # override and wrote an EMPTY region — compiling fine for
        # `disaster-type: none` and only exploding much later inside the
        # generator, or as an unpack error out of compile_disaster.
        if not vals:
            raise SystemExit("REGION_M={0!r}: no number in it; expected N, NxN "
                             "or N,N".format(raw))
        vals = [vals[0], vals[0]] if len(vals) == 1 else vals[:2]
        if any(v <= 0.0 for v in vals):
            raise SystemExit("REGION_M={0!r}: sides must be positive, got "
                             "{1}".format(raw, vals))
        return vals

    _dtype = os.environ.get("DISASTER_TYPE", "").strip() or None
    _sev = os.environ.get("SEVERITY", "").strip()
    _severity = float(_sev) if _sev else None

    # SEVERITY GATES THE DISASTER, so asking for one without it is a silent
    # no-op. `compile_spec` picks the compiler with
    #     fn = DISASTERS[dtype] if severity > 0.0 else compile_none
    # and the generic presets (`suburb`, `downtown`, `suburb_net`) ship
    # `severity: 0.0`. So `DISASTER_TYPE=wildfire` alone compiles through
    # compile_none and yields a PRISTINE plat that looks like the request
    # worked. Give an asked-for disaster a real default instead; 0.6 is what
    # suburb_wildfire.yaml uses.
    if _dtype and _dtype.lower() != "none" and _severity is None:
        _severity = 0.6
        print("[spawn] DISASTER_TYPE={0} with no SEVERITY -> defaulting to 0.6 "
              "(severity 0 compiles to no disaster at all)".format(_dtype),
              flush=True)

    SPEC_OVERRIDES = {
        "region_m": _region_m(os.environ.get("REGION_M", "").strip()),
        "disaster-type": _dtype,
        "severity": _severity,
    }

    # Which way a house faces. `h["yaw_deg"]` is the frontage TANGENT (along
    # the street) and the house must face ACROSS it at the kerb, so the plat
    # adds this offset. -90 is the code default and what every purpose-built
    # suburb preset sets; expose it so a scene whose houses look turned can be
    # corrected in one restart instead of a source edit.
    _hyaw = os.environ.get("HOUSE_YAW_OFFSET_DEG", "").strip()
    if _hyaw:
        SPEC_OVERRIDES["overrides"] = {
            "suburb_parcel": {"house_yaw_offset_deg": float(_hyaw)}}
        print("[spawn] HOUSE_YAW_OFFSET_DEG={0} (default -90)".format(_hyaw),
              flush=True)
    # MINI_SEED already drives layout/damage/vegetation through build_scene's
    # `seed`; carry it into the spec too so the two cannot disagree.
    if os.environ.get("MINI_SEED", "").strip():
        SPEC_OVERRIDES["seed"] = SEED
    if any(v is not None for v in SPEC_OVERRIDES.values()):
        print("[spawn] spec overrides: {0}".format(
            {k: v for k, v in SPEC_OVERRIDES.items() if v is not None}),
            flush=True)
if _GENERATED:
    # off | ground | all. The plat is ~10^5 prims and `add_colliders` is a
    # Python recursion that prints per mesh, so a full pass is minutes of log
    # spam — and mostly futile: houses and trees are referenced INSTANCEABLE
    # (what keeps 9k trees inside VRAM) and `GetChildren()` does not descend
    # into an instance, so they get no collider either way. `ground` gives the
    # drone the one surface it needs to land on and take off from.
    #
    # A FROZEN cell defaults to the SAME `ground`, deliberately: its
    # `/World/stage/generated/ground` is the same 1,800-mesh sheet the built
    # path collides, at the same prim path, and matching the built path is
    # what makes a frozen run and a rebuilt one comparable. `off` is a
    # reasonable saving on a 24-iteration sweep — the Pegasus default
    # environment's GroundPlane is composed into the cell's `/World/stage` and
    # is itself a collision plane at z = 0, which is all a takeoff needs — but
    # it is a mission's decision to make explicitly, not this script's to make
    # silently.
    COLLIDERS = os.environ.get("SUBURB_COLLIDERS", "ground").strip().lower()

# --------------------- FROZEN DATASET CELL (FROZEN_SCENE) ---------------------
# One `.usd` of the dataset, from EITHER of two places, chosen by
# `FROZEN_DATASET_ROOT`:
#
#   /isaac-sim/final_disaster_dataset      the bind mount (FINAL_DATASET_DIR on
#                                          the host). The default, and the only
#                                          one that works on a machine that
#                                          already has the cells.
#   omniverse://<host>/Projects/SEI-COA/final_disaster_dataset
#                                          Nucleus. THE ONLY OPTION ON AN OSMO
#                                          POD: `final_disaster_dataset/` lives
#                                          outside the repo and is not cloned,
#                                          so a pod has no local copy, and the
#                                          cells are ~300 MB each.
#
# `FROZEN_SCENE` is the same string either way — a bare cell path
# (`Fire/Suburban/level_1/1`), with the filename derived from the dataset
# contract — so switching a mission between a bench and a pod is one variable.
if _FROZEN:
    FROZEN_ROOT = (os.environ.get("FROZEN_DATASET_ROOT", "").strip()
                   or frozen_gt.CONTAINER_ROOT)
    try:
        FROZEN_USD = frozen_gt.resolve_cell(FROZEN_SCENE, FROZEN_ROOT)
    except ValueError as exc:
        # FATAL, and fatal HERE. A path that does not resolve reaches
        # `AddReference` as a silent no-op: the prim composes empty, nothing
        # errors, and the first sign is a black viewport and eight drones
        # hovering over nothing 20 minutes into a pod.
        raise SystemExit(str(exc))
    FROZEN_CELL = frozen_gt.cell_dir(FROZEN_USD)
    _FROZEN_URL = frozen_gt.is_url(FROZEN_USD)

    def _frozen_read_text(path):
        """Read a cell's GT JSON, from disk or from Nucleus.

        `frozen_annotations` is stdlib-only on purpose (it runs in the tests
        and in the offline mission planner), so the `omni.client` half of this
        lives here, where Kit already is, and is injected.
        """
        if not frozen_gt.is_url(path):
            with open(path, encoding="utf-8") as fh:
                return fh.read()
        result, _ver, content = omni.client.read_file(path)
        if result != omni.client.Result.OK:
            print("[frozen-gt] {0}: {1}".format(path, result), flush=True)
            return None
        return bytes(memoryview(content)).decode("utf-8")

    print("[scene] FROZEN cell {0}{1}".format(
        FROZEN_USD, "  (Nucleus)" if _FROZEN_URL else ""), flush=True)

DRONE_USD ="~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

# DOWNWARD tilt of every drone's ZED, in degrees about the body Y axis —
# positive is down. Default 0 is the LEVEL mount every method in this stack
# assumes today; a mission that wants a tilt sets ZED_PITCH_DEG in its `env:`.
#
# Set for CoNavGPT runs. CoNavGPT builds its occupancy map by unprojecting FPV
# depth and rasterising it top-down, so a level camera at 15-40 m AGL spends the
# top half of every frame on sky and never sees the ground under the drone,
# leaving a permanent hole in the map around the robot.
#
# THE PLANNER MUST BE TOLD, because TF walks the URDF and the URDF models NO
# mount pitch: a camera tilted here is a camera TF cannot see, and every point
# would unproject to the wrong bearing without a word of warning.
# search_planner reads this SAME variable as the default for its
# camera_pitch_rad so the two
# cannot drift apart; any other consumer of this tilt must do likewise.
ZED_PITCH_DEG = float(os.environ.get("ZED_PITCH_DEG", "").strip() or 0.0)

# ZED RESOLUTION. Pegasus defaults to 480x300, and TARGET PIXEL HEIGHT is what
# gates detection: measured on a wildfire frame, YOLO needs a person ~20 px
# tall to clear a 0.5 threshold and is blind by ~9 px. At 12 m cruise with the
# camera pitched down, 480x300 puts a person under that floor, so the drone
# flies over people and detects nothing. Raising this is the one fix that does
# NOT cost flight time and does not depend on descending first.
# 720x450 is 1.5x linear, i.e. 1.5x the pixels on target at every altitude.
ZED_WIDTH = int(os.environ.get("ZED_WIDTH", "").strip() or 720)
ZED_HEIGHT = int(os.environ.get("ZED_HEIGHT", "").strip() or 450)

# RTX lidar per drone. Defaults to ON, which is what this launcher has always
# done — an UNSET value means "unchanged", not "off", because the compose file
# forwards ENABLE_LIDAR empty rather than false (each launcher owns its own
# default; example_multi_px4_pegasus_launch_script.py's is off).
#
# Worth turning OFF for a CoNavGPT mission. That method consumes no lidar and no
# point clouds — it builds its occupancy map purely from RGBD depth — and the
# lidar is expensive: measured 2026-08-25 on the house bench, cameras fall from
# ~53 Hz to ~17 Hz and /clock from ~53 to ~15 Hz with it attached. Since an OSMO
# run is bounded in SIM seconds, that is a ~3x multiplier on the WALL CLOCK, and
# therefore on the GPU hours, for identical coverage.
#
# It is NOT off because of the fault §4.1 once blamed on it. That was retested
# with the lidar attached and the scene produced no cudaErrorIllegalAddress, no
# render-graph failure and no dropped frames; the real cause was a stale DDS
# profile in the isaac-sim image. Anything that needs vdb_mapping needs this on.
ENABLE_LIDAR = (os.environ.get("ENABLE_LIDAR", "").strip().lower()
                or "true") in ("1", "true", "yes", "on")

# Lighting
ADD_DOME_LIGHT = False
DOME_LIGHT_PATH = "/World/DomeLight"
DOME_LIGHT_INTENSITY = 3500.0
DOME_LIGHT_EXPOSURE = -5.0

# ---------------------------------------------------------------------------
# FROZEN-CELL LIGHTING. A frozen cell carries NO SKY, and this is why.
#
# `freeze.DEACTIVATE_DEFAULT` turns off `/World/Environment` and
# `/World/stage/Environment` before the flatten — deliberately, so the
# collector does not drag the default environment's assets along and so the
# cell does not ship Pegasus' flat default ground. But the sky and the sun
# live in exactly that prim, so the deactivation takes the LIGHTING with them.
# Nothing puts it back: `ADD_DOME_LIGHT` above is False, the one
# `add_dome_light` call in this file sits in the ENV_URL branch that `_FROZEN`
# never enters, and `pg.load_environment` is skipped outright for a frozen
# cell.
#
# MEASURED (2026-08-30, `Sdf` on the cells themselves, no Kit):
#   * every Fire cell   — 83,215 prims, exactly ONE light: a SphereLight of
#     radius 0.25 m at (0, 0, 2.5) with intensity 1e5. A 25 cm bulb 2.5 m off
#     the ground, lighting a 1 km plate.
#   * every Tornado cell — ZERO lights.
# and downstream, in the 8-robot benchmark run of 2026-08-30 09:09:51:
#   * the 2048x2048 overhead frame is 99.93 % pure RGB (0,0,0); the brightest
#     cells reach R=187 and are the Flow flames, the only emissive thing left.
#   * 23,641+ consecutive frames were rejected as blank by the GCS visualiser's
#     std gate, so `/gcs/sim_ground` carried ZERO messages.
#   * the drones' ZED cameras render that same black world, so YOLO-World was
#     asked for "person" over 81 minutes x 8 drones and had nothing to score.
#
# So this is authored HERE rather than in the freeze: it re-lights all 18
# existing cells with no re-freeze, and it lights every cell IDENTICALLY,
# which is what makes one cell's score comparable with another's.
FROZEN_LIGHT = (os.environ.get("FROZEN_LIGHT", "").strip().lower()
                or "on") in ("1", "true", "yes", "on")
# THE SKY. This is the SAME asset every generated (non-frozen) scene already
# flies under: `scene_gen/config/asset_sets/shared.yaml` sets
#     asset_root: omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/
#     sky:        RetroNeighborhood/RetroNeighborhood.stage.usd
# and `scene_api._apply_sky` -> `scene_generator.resolve_sky` joins the two.
# RetroNeighborhood's `/Environment` holds a dynamic-sky rig plus its dome
# light, and `scene_prep.add_sky` borrows exactly those root prims (the ones
# outside the file's defaultPrim) rather than its geometry, so this is cheap.
#
# It is spelled out here rather than resolved through `scene_generator`
# because a FROZEN run cannot import it: `_BUILT` is
# `bool(SCENE_CONFIG) and not _FROZEN`, and the `sys.path.insert` that puts
# `scene_gen` on the path is gated on `_BUILT`. `test_frozen_lighting` reads
# `shared.yaml` and asserts the two agree, so the copy cannot drift.
#
# Reachability measured 2026-08-30: `omni.client.stat` -> Result.OK. If it
# ever is not reachable, `reference_root_prims_under_world` returns [] and
# `add_sky` falls back to a plain dome — the scene is still lit, just flat,
# and the fallback says so in the log.
FROZEN_SKY = (os.environ.get("FROZEN_SKY", "").strip()
              or "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/"
                 "Stages/RetroNeighborhood/RetroNeighborhood.stage.usd")
# Only used when the sky above cannot be borrowed and `add_sky` falls back to
# an untextured dome.
FROZEN_DOME_INTENSITY = float(os.environ.get("FROZEN_DOME_INTENSITY") or 1200.0)
FROZEN_DOME_EXPOSURE = float(os.environ.get("FROZEN_DOME_EXPOSURE") or 0.0)
# The sun, added ONLY when the sky did not bring a DistantLight of its own —
# two suns is worse than none. A dome alone lights the scene but casts no
# shadow, and the build-time `snaps/overview.png` of every cell has hard sun
# shadows in it. Elevation is measured up from the horizon, azimuth clockwise
# from +Y (north).
FROZEN_SUN = (os.environ.get("FROZEN_SUN", "").strip().lower()
              or "on") in ("1", "true", "yes", "on")
FROZEN_SUN_INTENSITY = float(os.environ.get("FROZEN_SUN_INTENSITY") or 2400.0)
FROZEN_SUN_ELEV_DEG = float(os.environ.get("FROZEN_SUN_ELEV_DEG") or 48.0)
FROZEN_SUN_AZ_DEG = float(os.environ.get("FROZEN_SUN_AZ_DEG") or 135.0)
# The stray bulb described above. Left active it is a blown-out hotspot at the
# origin once real lighting exists, and the origin is inside the search area.
FROZEN_KILL_STRAY_LIGHTS = (
    os.environ.get("FROZEN_KILL_STRAY_LIGHTS", "").strip().lower()
    or "on") in ("1", "true", "yes", "on")
# Anything at or below this radius is a stray point source, not scene lighting.
FROZEN_STRAY_LIGHT_MAX_R_M = 2.0

# ---------------------------------------------------------------------------
# FROZEN-CELL ASSET REBASE. The cell's textures are baked as ABSOLUTE PATHS
# INSIDE THE BUILD CONTAINER, and most of what they point at is git-ignored.
#
# `freeze.export_scene` flattens with `collect=False` by design (positions are
# baked; textures stay by reference), so every look in a cell still points
# wherever it pointed on the build machine. MEASURED on
# `fire_suburban_lvl1_1.usd` (2026-08-30): of 534 asset paths, 248 are
# absolute `/isaac-sim/AirStack/scene_gen/assets/...` — 124 `materials`
# (megascans road/asphalt/pavement, burn, 108 scorch decals), 92 `objaverse`
# prop textures, 32 `aec` (the GRASS, DIRT and tree-bark MDLs).
#
# On a dev box those resolve, because the files are sitting there. On an OSMO
# pod they cannot: `.gitignore` excludes `scene_gen/assets/aec/*` (line 112),
# `objaverse/*` (107) and `materials/scorched/` (143), so a fresh clone has 1
# tracked file under `aec`, 1 under `objaverse` and 380 of 13,164 under
# `materials`. The ground is the most visible casualty — `ground/materials/
# grass`, `grass_rough`, `grass_park` and `dirt` are the four AEC MDLs, and
# they carry the 14 `ground_base` tiles that span the whole 1 km plate.
#
# The same tree IS mirrored on Nucleus at `Projects/SEI-COA/scene_gen/assets/`.
# Measured by rebasing all 248 onto it: 228 resolve, and the only 20 that do
# not are `materials/scorched/scorch_*.png` — per-build soot decals that were
# never uploaded.
#
# So: rewrite the prefix at load time, and ONLY for a path that is genuinely
# absent locally. That makes this a no-op on a machine that has the assets
# (the dev box keeps using its own files) and a fix on a pod, which is the
# same shape as `quake.load_manifest` rebasing stale archetype paths onto
# ARCH_DIR.
FROZEN_REBASE_ASSETS = (
    os.environ.get("FROZEN_REBASE_ASSETS", "").strip().lower()
    or "on") in ("1", "true", "yes", "on")
FROZEN_ASSET_LOCAL_PREFIX = (os.environ.get("FROZEN_ASSET_LOCAL_PREFIX", "").strip()
                             or "/isaac-sim/AirStack/scene_gen/assets/")
FROZEN_ASSET_MIRROR = (os.environ.get("FROZEN_ASSET_MIRROR", "").strip()
                       or os.environ.get("AIRSTACK_ASSET_ROOT", "").strip()
                       or "omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                          "Projects/SEI-COA/scene_gen/assets/")

# GPS world anchor: what world (0, 0, 0) maps to in real GPS coordinates.
# Matches the Lisbon default in px4_config.yaml — change here to relocate the sim world.
WORLD_GPS_ORIGIN = DEFAULT_WORLD_ORIGIN

# Drone spawn configs.
# x_m = East offset from world origin (meters)
# y_m = North offset from world origin (meters)
# z_m = Up offset / height above floor (meters)
# orient = initial quaternion [x, y, z, w]
# spawn location for /Assets/Fire_Academy_Digital_Twin/fire_academy.usd:
# {"domain_id": 1, "x_m": 20.0, "y_m": -7.0, ...}
# {"domain_id": 2, "x_m": 17.0, "y_m":  1.5, ...}

SPAWN_HEIGHT_ABOVE_FLOOR_M = 0.3#0.03
# DRONE_CONFIGS = [
#     {"domain_id": 1, "x_m": 32.0, "y_m": 12.6, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
#     {"domain_id": 2, "x_m": 28.0, "y_m": 14.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
#     {"domain_id": 3, "x_m": 32.0, "y_m": 19.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0}
#     ]


DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": 3.0, "y_m": 3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": 0.75},
    {"domain_id": 2, "x_m": 0.0, "y_m": 0.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": 0.75},
    {"domain_id": 3, "x_m": -3.0, "y_m": -3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": 0.75},
    ]

# SPAWN_CONFIGS / SPAWN_POLY, GENERATED PATH ONLY. The three spots above are
# metres from the origin and mean nothing on a 1600 x 1200 m plat, so the
# generated path takes its fleet from the mission instead: SPAWN_CONFIGS is a
# JSON list of dicts needing only x_m/y_m, SPAWN_POLY randomizes NUM_ROBOTS
# spawns inside a rect or convex polygon. Deliberately NOT applied to the
# ENV_URL path, whose fleet is the list above and stays that way.
if _GENERATED:
    NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS") or 1)
    SPAWN_HEIGHT_ABOVE_FLOOR_M = float(os.environ.get("SPAWN_HEIGHT_M") or 0.5)
    LIDAR_MIN_RANGE_M = 0.75

    # DEFAULT SPAWN, DERIVED — not eyeballed. One drone on a 10.7 m local road
    # at the SE flank of the suburb_wildfire plat:
    #   * 166 s of arrival-time margin outside the severity-0.6 fire front at
    #     MINI_BURN_FRAC=0.45, so it starts in clear air rather than smoke;
    #   * 381 m from the burn centre at (19, 110) — close enough that the scar
    #     is the first thing in frame, far enough not to spawn inside it;
    #   * yaw 138.1 deg = atan2(110 - -144.3, 19 - 302.4), i.e. facing the burn
    #     centre, which as a quaternion [x, y, z, w] about +Z is
    #     [0, 0, sin(69.05 deg), cos(69.05 deg)] = [0, 0, 0.9342, 0.3568].
    # Changing MINI_SEED / MINI_BURN_FRAC / the preset's epicenter invalidates
    # the margin figure; the point stays on road either way.
    _DEFAULT_DRONE_CONFIGS = [
        {"domain_id": 1, "x_m": 302.4, "y_m": -144.3,
         "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M,
         "orient": [0.0, 0.0, 0.9342, 0.3568],
         "lidar_min_range": LIDAR_MIN_RANGE_M},
    ]

    _SPAWN_CONFIGS = os.environ.get("SPAWN_CONFIGS")
    _SPAWN_POLY = os.environ.get("SPAWN_POLY")
    if _SPAWN_CONFIGS:
        DRONE_CONFIGS = json.loads(_SPAWN_CONFIGS)
        for _i, _c in enumerate(DRONE_CONFIGS, start=1):
            _c.setdefault("domain_id", _i)
            _c.setdefault("z_m", SPAWN_HEIGHT_ABOVE_FLOOR_M)
            _c.setdefault("orient", [0.0, 0.0, 0.0, 1.0])
            _c.setdefault("lidar_min_range", LIDAR_MIN_RANGE_M)
        _spawn_seed = "hardcoded"
    elif _SPAWN_POLY:
        _spawn_seed = os.environ.get("SPAWN_SEED") or "{0}-{1}".format(
            time.time_ns(), os.getpid())
        DRONE_CONFIGS = generate_spawn_configs(
            json.loads(_SPAWN_POLY),
            n=NUM_ROBOTS,
            z_m=SPAWN_HEIGHT_ABOVE_FLOOR_M,
            lidar_min_range=LIDAR_MIN_RANGE_M,
            min_dist=float(os.environ.get("SPAWN_MIN_DIST_M") or 3.0),
            margin=float(os.environ.get("SPAWN_MARGIN_M") or 0.0),
            seed=_spawn_seed,
        )
    else:
        DRONE_CONFIGS = [dict(c) for c in _DEFAULT_DRONE_CONFIGS[:NUM_ROBOTS]]
        _spawn_seed = None

    if len(DRONE_CONFIGS) != NUM_ROBOTS:
        # Not fatal — the robot containers are what NUM_ROBOTS actually sizes,
        # and a mismatch shows up as a robot with no airframe rather than a
        # crash. Say so loudly, because that failure otherwise reads as a PX4
        # timeout.
        print("[spawn] WARNING: NUM_ROBOTS={0} but {1} spawn config(s) — robot "
              "containers without an airframe will never report ready"
              .format(NUM_ROBOTS, len(DRONE_CONFIGS)), flush=True)

    print("[spawn] scene: {0}".format(
        "FROZEN " + FROZEN_USD if _FROZEN
        else "SCENE_CONFIG={0} (BUILT in-process, no ENV_URL)".format(
            SCENE_CONFIG)), flush=True)
    print("[spawn] AIRSTACK_ASSET_ROOT={0}"
          .format(os.environ.get("AIRSTACK_ASSET_ROOT") or "<repo>"), flush=True)
    print("[spawn] SPAWN_SEED={0}".format(_spawn_seed), flush=True)
    print("[spawn] DRONE_CONFIGS={0}".format(json.dumps(DRONE_CONFIGS)),
          flush=True)

# Top-down "map" camera. Captures one aerial of the static scene that the
# GCS visualizer turns into a textured ground in Foxglove's 3D panel. The
# camera centers on (OVERHEAD_CENTER_X_M, OVERHEAD_CENTER_Y_M) in world
# meters — leave both 0.0 for the legacy origin-centered behavior.
OVERHEAD_ALTITUDE_M    = 165.0
OVERHEAD_COVERAGE_M    = 225  # per-map knob: world meters per side.
OVERHEAD_CENTER_X_M    = 0.0 #-152     # world-X of camera center / texture center.
OVERHEAD_CENTER_Y_M    = 0.0 #-80     # world-Y of camera center / texture center.
OVERHEAD_PX_PER_METER  = 10.0     # Source-image density. Bump for sharper texture.
OVERHEAD_TOPIC         = "/sim/overhead/image"
OVERHEAD_SPEC_TOPIC    = "/sim/overhead/spec"
OVERHEAD_CENTER_X_TOPIC = "/sim/overhead/center_x"
OVERHEAD_CENTER_Y_TOPIC = "/sim/overhead/center_y"
OVERHEAD_FRAME_ID      = "map"
OVERHEAD_DOMAIN_ID     = 0

# THE PLAT, NOT THE SPAWN CROP. A generated region is 1600 x 1200 m centred on
# the origin and the 225 m default above is a crop of it that is useless as a
# mission map. 1600 m x 1.28 px/m = 2048 px, which is the helper's
# max_resolution — asking for more is silently clamped.
if _GENERATED:
    OVERHEAD_ALTITUDE_M   = float(os.environ.get("OVERHEAD_ALTITUDE_M") or 400.0)
    OVERHEAD_COVERAGE_M   = float(os.environ.get("OVERHEAD_COVERAGE_M") or 1600.0)
    OVERHEAD_CENTER_X_M   = float(os.environ.get("OVERHEAD_CENTER_X_M") or 0.0)
    OVERHEAD_CENTER_Y_M   = float(os.environ.get("OVERHEAD_CENTER_Y_M") or 0.0)
    OVERHEAD_PX_PER_METER = float(os.environ.get("OVERHEAD_PX_PER_METER") or 1.28)
# ---------------------------------------------------------


ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        # Try immediate enable if available (more robust across Kit versions), fall back otherwise
        try:
            ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            ext_manager.set_extension_enabled(ext, True)


def nucleus_stat(url: str) -> bool:
    result, info = omni.client.stat(url)
    return result == omni.client.Result.OK


def wait_for_stage(stage, timeout_s: float = 10.0):
    """Pump the Kit app loop until /World has content (scene fully loaded)."""
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False

class PegasusApp:

    def __init__(self):
        # Write GPS origins immediately so robot containers can read them
        # before this container finishes its heavy USD loading.
        set_gps_origins(DRONE_CONFIGS, world_origin=WORLD_GPS_ORIGIN)

        omni.client.initialize()
        nucleus_stat(f"omniverse://{NUCLEUS_SERVER}")
        if not _GENERATED:
            nucleus_stat(ENV_URL)

        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.timeline.stop()

        # THE FROZEN PATH LOADS NOTHING HERE. `pg.load_environment` references
        # the given USD's DEFAULT PRIM onto /World/stage, and a frozen cell has
        # NO defaultPrim (measured: `Sdf.Layer.defaultPrim` is empty on every
        # cell — the Kit exporter writes 84 `Flattened_Prototype_*` roots, a
        # `/World`, a `/Render` and four cameras, and nominates none of them),
        # so that call would compose an empty prim and hand back a black
        # viewport with no error. `_reference_frozen_scene` names the prim
        # explicitly instead. The World created above already carries a
        # PhysicsScene, which is the only thing the base env was wanted for.
        if not _FROZEN:
            self.pg.load_environment(BASE_ENV_URL if _GENERATED else ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if _FROZEN:
            self._reference_frozen_scene(stage)

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        if _BUILT:
            # The Pegasus default env was loaded only to give the World a valid
            # base; its ground plane and lighting go straight back off, because
            # the generated plat brings its own ground and the config's sky.
            for _name in ("GroundPlane", "Environment"):
                _p = stage.GetPrimAtPath("/World/" + _name)
                if _p and _p.IsValid():
                    _p.SetActive(False)

        dedupe_physics_scenes(stage)

        if _GENERATED:
            # Units FIRST: the plat is authored in metres x ssf, and that is
            # also why nothing rescales it afterwards.
            mpu, s = get_stage_meters_per_unit(stage)
            if _FROZEN:
                self._finish_frozen_scene(stage)
            else:
                self._build_generated_scene(stage, s)
        else:
            self._prepare_env_url_scene(stage)
            # Units
            mpu, s = get_stage_meters_per_unit(stage)

        # Top-down orthographic camera over (0, 0). Publishes one JPEG aerial
        # of the static scene at low rate; the GCS visualizer republishes it
        # as a textured ground for Foxglove's 3D panel.
        cam_path = add_orthographic_camera(
            stage,
            prim_path="/World/MapCamera",
            altitude_m=OVERHEAD_ALTITUDE_M,
            coverage_m=OVERHEAD_COVERAGE_M,
            scene_scale_factor=s,
            center_x_m=OVERHEAD_CENTER_X_M,
            center_y_m=OVERHEAD_CENTER_Y_M,
        )
        add_overhead_camera_publisher(
            parent_graph_path="/World/MapCameraGraph",
            camera_prim_path=cam_path,
            topic=OVERHEAD_TOPIC,
            spec_topic=OVERHEAD_SPEC_TOPIC,
            center_x_topic=OVERHEAD_CENTER_X_TOPIC,
            center_y_topic=OVERHEAD_CENTER_Y_TOPIC,
            frame_id=OVERHEAD_FRAME_ID,
            coverage_m=OVERHEAD_COVERAGE_M,
            center_x_m=OVERHEAD_CENTER_X_M,
            center_y_m=OVERHEAD_CENTER_Y_M,
            pixels_per_meter=OVERHEAD_PX_PER_METER,
            domain_id=OVERHEAD_DOMAIN_ID,
        )

        # Spawn all drones. ZED_TIME_SLICE_GROUPS=2 alternates two groups of
        # four cameras in an eight-robot run: only one group renders per sim
        # tick, while each camera's RGB/depth/info/PCL remain synchronized.
        self._zed_slice_groups = max(
            1, int(os.environ.get("ZED_TIME_SLICE_GROUPS", "1")))
        self._zed_slice_gates = []
        self._zed_slice_tick = 0
        if self._zed_slice_groups > len(DRONE_CONFIGS):
            raise ValueError("ZED_TIME_SLICE_GROUPS cannot exceed the number of drones")
        if self._zed_slice_groups > 1:
            print(f"[zed] staggered camera schedule: {self._zed_slice_groups} groups, "
                  f"{len(DRONE_CONFIGS) / self._zed_slice_groups:g} cameras/tick",
                  flush=True)
        for cfg in DRONE_CONFIGS:
            i = cfg["domain_id"]
            pos = [cfg["x_m"] * s, cfg["y_m"] * s, cfg["z_m"] * s]

            graph_handle = spawn_px4_multirotor_node(
                pegasus_node_name=f"PX4Multirotor_{i}",
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                vehicle_id=i,
                domain_id=i,
                usd_file=DRONE_USD,
                init_pos=pos,
                init_orient=cfg["orient"],
            )

            zed_gate = add_zed_stereo_camera_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                camera_name="ZEDCamera",
                camera_offset=[0.21, 0.0, 0.05],
                camera_rotation_offset=[0.0, ZED_PITCH_DEG, 0.0],
                frame_width=ZED_WIDTH,
                frame_height=ZED_HEIGHT,
                pipeline_mode=os.environ.get("ZED_PIPELINE", "stereo").strip().lower(),
            )
            self._zed_slice_gates.append(
                ((i - 1) % self._zed_slice_groups, zed_gate))

            if ENABLE_LIDAR:
                add_rtx_lidar_subgraph(
                    parent_graph_handle=graph_handle,
                    drone_prim=f"/World/drone{i}/base_link",
                    robot_name=f"robot_{i}",
                    lidar_config="ouster_os1",
                    lidar_topic_name="point_cloud_raw",
                    lidar_offset=[0.0, 0.0, 0.025],
                    lidar_rotation_offset=[0.0, 0.0, 0.0],
                    min_range=cfg["lidar_min_range"],
                )

        # POST-SCALE ONLY, so the generated path never does it: there is no
        # scale_stage_prim on a generated plat, and toggling ~10^5 prims off
        # and on for 8 s buys nothing.
        if not _GENERATED:
            self._clear_post_scale_artifact(stage)

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"
        self.stop_sim = False
        if _GENERATED:
            self._print_drone_banner()

    def _prepare_env_url_scene(self, stage):
        # ----- Scene preparation -----
        # Bring in sky/sun/environment prims that sit at root level in the
        # source USD next to the defaultPrim that pg.load_environment already
        # loaded into /World/stage. reference_root_prims_under_world skips
        # the defaultPrim, so this can't duplicate geometry.
        reference_root_prims_under_world(stage, ENV_URL)

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        if ADD_DOME_LIGHT:
            add_dome_light(stage, DOME_LIGHT_PATH, DOME_LIGHT_INTENSITY, DOME_LIGHT_EXPOSURE)

    def _clear_post_scale_artifact(self, stage):
        # Toggle /World/stage off and on to clear a post-scale visual artifact
        # that lingers until the prim is deactivated/reactivated.
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            stage_prim.SetActive(False)
            app = omni.kit.app.get_app()
            wait_end = time.time() + 8.0
            while time.time() < wait_end:
                app.update()
            stage_prim.SetActive(True)

    # ----------------------- FROZEN DATASET CELL -----------------------

    def _reference_frozen_scene(self, stage):
        """Compose a frozen cell into this stage, prim by named prim.

        A frozen cell's root layer has no defaultPrim, so there is no
        "reference the file" — there is only "reference these prims". What it
        DOES have under `/World` is exactly what `freeze.export_scene`
        composed:

            PhysicsScene   ours already; NOT referenced, or physics is doubled
            stage          the Pegasus base env + `generated/` — the whole plat
            burnGround     the wildfire scar overlay      (fire cells)
            flow_blockage  the Flow flame/smoke emitters  (fire cells)
            scourGround    the mud scour along the track  (tornado cells)

        The sibling list is READ FROM THE FILE rather than hard-coded, via
        `Sdf` — which walks specs without composing anything, so it costs
        milliseconds and cannot be wrong about a disaster whose overlay is
        named something this script has never seen. Hard-coding the three
        names above would silently drop the surge water the first time a
        hurricane cell is exported.

        Instancing survives: the 84 `Flattened_Prototype_*` roots are what the
        `generated/inst` prims point at INSIDE the file, and a reference brings
        the whole composed subtree, so ~6,000 houses and trees still cost one
        prototype each.
        """
        layer = Sdf.Layer.FindOrOpen(FROZEN_USD)
        if layer is None:
            raise RuntimeError(f"could not open frozen scene {FROZEN_USD}")
        world = layer.GetPrimAtPath("/World")
        if world is None:
            raise RuntimeError(
                f"{FROZEN_USD} has no /World prim — is this a frozen dataset "
                f"cell? (roots: {[s.name for s in layer.rootPrims][:8]}...)")
        names = [c.name for c in world.nameChildren
                 if c.name not in ("PhysicsScene",)]
        if "stage" not in names:
            raise RuntimeError(
                f"{FROZEN_USD} has no /World/stage — nothing to fly "
                f"(children: {names})")
        for name in names:
            dest = f"/World/{name}"
            prim = stage.DefinePrim(dest, "Xform")
            prim.GetReferences().AddReference(FROZEN_USD, f"/World/{name}")
            print(f"[frozen] referenced /World/{name} -> {dest}", flush=True)
        # Kit's flatten leaves the export-time camera and the Pegasus default
        # environment in the file as DEACTIVATED prims (freeze-dataset-state:
        # `SetActive(False)` composes, `RemovePrim` does not). They come in
        # inactive and stay inactive; nothing to do here — but say so, because
        # a reviewer looking for the review camera should not have to guess.
        for probe in ("/World/stage/ReviewCamera", "/World/ReviewCamera"):
            p = stage.GetPrimAtPath(probe)
            if p and p.IsValid() and p.IsActive():
                p.SetActive(False)
                print(f"[frozen] deactivated {probe}", flush=True)
        # FROZEN_OVERLAY_USD (2026-09-02): reference an EXTRA authored layer
        # on top of the frozen cell — e.g. the standing-person A/B detection
        # rig. The frozen cell itself is never edited (a finished cell must
        # never be re-serialised — freeze-portable-scenes); the overlay is a
        # plain .usda whose defaultPrim lands at /World/Overlay.
        overlay_usd = os.environ.get("FROZEN_OVERLAY_USD", "").strip()
        if overlay_usd:
            oprim = stage.DefinePrim("/World/Overlay", "Xform")
            oprim.GetReferences().AddReference(overlay_usd)
            print(f"[frozen] OVERLAY referenced {overlay_usd} -> /World/Overlay",
                  flush=True)
        self._wait_for_frozen_geometry(stage)

    def _wait_for_frozen_geometry(self, stage, timeout_s=600.0):
        """Pump the app until the referenced plat has actually COMPOSED.

        `wait_for_stage` is not enough here and would pass instantly: it asks
        whether `/World` has a non-PhysicsScene child, and `DefinePrim` gave it
        one before a byte of the cell was read. What has to be true before
        colliders are cooked and eight drones are spawned is that
        `/World/stage/generated/inst` is populated — that is the ~6,000
        referenced houses and trees, i.e. the scene.

        A timeout is a WARNING, not a raise: a plat that is still streaming is
        recoverable (the drones spawn over a scene that fills in), while
        aborting the launch loses the whole iteration.
        """
        app = omni.kit.app.get_app()
        deadline = time.time() + timeout_s
        t0 = time.time()
        while time.time() < deadline:
            app.update()
            inst = stage.GetPrimAtPath(SCENE_PARENT + "/inst")
            if inst and inst.IsValid() and inst.GetChildren():
                print("[frozen] composed in {0:.0f} s: {1} referenced object(s) "
                      "under {2}/inst".format(
                          time.time() - t0, len(inst.GetChildren()),
                          SCENE_PARENT), flush=True)
                return True
        print("[frozen] WARNING: {0}/inst is still empty after {1:.0f} s — the "
              "cell may be streaming, or the reference resolved to nothing. "
              "Continuing; expect an empty view.".format(
                  SCENE_PARENT, timeout_s), flush=True)
        return False

    def _rebind_frozen_overlays(self, stage):
        """Re-author the material bindings the reference arcs threw away.

        `_reference_frozen_scene` brings each `/World/<name>` in as its OWN
        reference. That is what keeps the cell's PhysicsScene out and lets an
        unknown disaster overlay come along unnamed — but it splits prims that
        were siblings on one stage into separate composition arcs, and USD
        DROPS a relationship whose target lies outside its own arc:

            The relationship target </World/stage/generated/BurnLooks/band_0>
            from </World/burnGround/band_0.material:binding> ... refers to a
            path outside the scope of the reference from </World/burnGround>.
            Ignoring.

        Measured on `fire_suburban_lvl1_1.usd`: all 12 `/World/burnGround/
        band_*` meshes bind to `/World/stage/generated/BurnLooks/band_*`, and
        `/World/burnGround/BurnLooks` does not exist — so every band composes
        with NO material and the burn scar renders DEFAULT GREY. The tornado
        cells' `scourGround` is built the same way, so it is covered here too
        rather than special-cased.

        The repair is a local opinion on the live stage: after composition
        both `/World/burnGround/band_0` and `/World/stage/generated/BurnLooks/
        band_0` are real prims, and a binding authored here is not subject to
        the reference-scope rule that dropped the original.

        The cell itself is still wrong — a future freeze should author the
        looks INSIDE the overlay scope (or compose the whole `/World` as one
        arc) — but that needs a re-freeze, and this does not.
        """
        from pxr import Sdf, UsdShade

        layer = Sdf.Layer.FindOrOpen(FROZEN_USD)
        if layer is None:
            print("[frozen-rebind] could not reopen the cell; skipping",
                  flush=True)
            return
        world = layer.GetPrimAtPath("/World")
        if world is None:
            return

        fixed = already = absent = 0
        for scope in world.nameChildren:
            if scope.name in ("PhysicsScene", "stage"):
                continue
            root = "/World/" + scope.name
            for child in scope.nameChildren:
                rel = child.relationships.get("material:binding")
                if rel is None:
                    continue
                targets = list(rel.targetPathList.explicitItems)
                if not targets:
                    continue
                target = targets[0]
                # Inside its own arc: composed correctly, leave it.
                if target.pathString.startswith(root + "/"):
                    already += 1
                    continue
                prim = stage.GetPrimAtPath(child.path)
                mat = stage.GetPrimAtPath(target)
                if not (prim and prim.IsValid()):
                    continue
                if not (mat and mat.IsValid()):
                    absent += 1
                    continue
                UsdShade.MaterialBindingAPI(prim).Bind(UsdShade.Material(mat))
                fixed += 1

        if fixed or absent:
            print("[frozen-rebind] re-bound {0} cross-scope material binding(s)"
                  "{1}".format(fixed,
                               "; {0} target(s) missing on the stage".format(absent)
                               if absent else ""), flush=True)
        else:
            print("[frozen-rebind] no cross-scope bindings to repair "
                  "({0} already in scope)".format(already), flush=True)

    def _rebase_local_assets(self, stage):
        """Repoint build-machine texture paths at the Nucleus mirror.

        See the FROZEN-CELL ASSET REBASE block near the top of this file for
        the measurement. Only paths that are ACTUALLY ABSENT are rewritten, so
        this does nothing on a machine that has the assets on disk.
        """
        from pxr import Sdf, UsdShade

        if not FROZEN_REBASE_ASSETS or not FROZEN_ASSET_MIRROR:
            print("[frozen-assets] rebase off — looks will resolve only where "
                  "the build machine's paths exist", flush=True)
            return

        local = FROZEN_ASSET_LOCAL_PREFIX
        mirror = FROZEN_ASSET_MIRROR.rstrip("/") + "/"
        seen = kept = moved = 0
        unresolved = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdShade.Shader):
                continue
            for attr in prim.GetAttributes():
                if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                    continue
                val = attr.Get()
                path = getattr(val, "path", "") or ""
                if not path.startswith(local):
                    continue
                seen += 1
                # WHEN THE CELL CAME FROM NUCLEUS, ALWAYS REWRITE — even if the
                # file happens to exist on this machine.
                #
                # An absolute asset path inside a layer anchored on an
                # `omniverse://` URL is resolved AGAINST THAT SERVER, not
                # against the local filesystem. So on a pod, Kit turns
                # `/isaac-sim/AirStack/scene_gen/assets/x.png` into
                # `omniverse://<host>/isaac-sim/AirStack/scene_gen/assets/x.png`
                # — which does not exist — and logs "References an asset that
                # can not be found". Measured 2026-08-30: 659 files were staged
                # onto a pod's local disk at 21:23, a run started at 21:31, and
                # it still logged 505 not-found lines between 21:33 and 21:35
                # for files that were present, non-zero and root-readable the
                # whole time. Putting the bytes on local disk cannot fix a
                # lookup that is being sent to a server.
                #
                # Only a LOCAL cell can trust a local path, and then it is left
                # alone so a dev box keeps using its own files.
                if not _FROZEN_URL and os.path.exists(path):
                    kept += 1
                    continue
                rel = path[len(local):]
                attr.Set(Sdf.AssetPath(mirror + rel))
                moved += 1
                unresolved.append(rel)

        if not seen:
            print("[frozen-assets] no build-machine paths in this cell",
                  flush=True)
            return
        print("[frozen-assets] {0} baked '{1}' path(s): {2} present locally, "
              "{3} repointed at {4}".format(seen, local, kept, moved, mirror),
              flush=True)
        # The 20 known stragglers are per-build scorch decals that were never
        # uploaded. Name the families so a missing look is identifiable from
        # the log rather than from a render.
        if moved:
            fams = {}
            for rel in unresolved:
                fams[rel.split("/")[0]] = fams.get(rel.split("/")[0], 0) + 1
            print("[frozen-assets]   by tree: " + ", ".join(
                "{0}={1}".format(k, v) for k, v in sorted(fams.items())),
                flush=True)

    def _light_frozen_scene(self, stage):
        """Give the frozen cell a sky and a sun — it ships with neither.

        See the FROZEN-CELL LIGHTING block near the top of this file for the
        measurement that made this necessary. Order matters: the strays go off
        BEFORE the dome goes on, so the banner's census reports the lighting
        this scene will actually render with.
        """
        import math

        from pxr import Gf, Sdf, UsdGeom, UsdLux

        if not FROZEN_LIGHT:
            print("[frozen-light] FROZEN_LIGHT=off — the cell will render "
                  "with whatever it shipped with (measured: near-black)",
                  flush=True)
            return

        # 1) The strays. A frozen cell's only light is a 0.25 m SphereLight at
        #    the origin; once a dome exists it is a hotspot on the plate.
        killed = []
        if FROZEN_KILL_STRAY_LIGHTS:
            for prim in stage.Traverse():
                if not prim.IsA(UsdLux.SphereLight):
                    continue
                r = UsdLux.SphereLight(prim).GetRadiusAttr().Get()
                if r is None or float(r) <= FROZEN_STRAY_LIGHT_MAX_R_M:
                    prim.SetActive(False)
                    killed.append(prim.GetPath().pathString)
        if killed:
            print("[frozen-light] deactivated {0} stray point light(s): {1}"
                  .format(len(killed), ", ".join(killed[:4])), flush=True)

        # 2) The sky. `add_sky` is the SAME function, with the SAME asset,
        #    that every generated scene is lit by — so a frozen cell and a
        #    rebuilt one now come up under one sky and one code path. For a
        #    `.usd` it borrows the file's root prims outside its defaultPrim
        #    (RetroNeighborhood's `/Environment`: the dynamic-sky rig and its
        #    dome light); if that cannot be opened it falls back to a plain
        #    dome and says so.
        before_world = {c.GetName() for c in
                        stage.GetPrimAtPath("/World").GetChildren()} \
            if stage.GetPrimAtPath("/World") else set()
        try:
            from scene_prep import add_sky
            add_sky(stage, FROZEN_SKY, prim_path=DOME_LIGHT_PATH,
                    intensity=FROZEN_DOME_INTENSITY,
                    exposure=FROZEN_DOME_EXPOSURE)
        except Exception as exc:                      # noqa: BLE001
            carb.log_warn(f"[frozen-light] add_sky failed ({exc}); falling "
                          f"back to a bare dome light")
            add_dome_light(stage, DOME_LIGHT_PATH, FROZEN_DOME_INTENSITY,
                           FROZEN_DOME_EXPOSURE)

        # The borrowed subtree is a reference to a Nucleus asset, so give it
        # updates to resolve before asking what it brought — the sun decision
        # below depends on the answer, and an unresolved reference looks
        # exactly like a sky with no lights in it.
        app = omni.kit.app.get_app()
        for _ in range(30):
            app.update()
        after_world = {c.GetName() for c in
                       stage.GetPrimAtPath("/World").GetChildren()} \
            if stage.GetPrimAtPath("/World") else set()
        borrowed = sorted(after_world - before_world - {"DomeLight"})
        if borrowed:
            print("[frozen-light] sky borrowed from {0}: /World/{1}".format(
                FROZEN_SKY, ", /World/".join(borrowed)), flush=True)

        # 3) The sun — ONLY if the sky did not bring one. A DistantLight's
        #    direction is its -Z axis, so aim it by rotating -Z onto the
        #    vector pointing FROM the sun TOWARD the ground. Elevation up from
        #    the horizon, azimuth clockwise from +Y.
        has_distant = any(p.IsA(UsdLux.DistantLight) for p in stage.Traverse())
        if FROZEN_SUN and has_distant:
            print("[frozen-light] the sky brought its own DistantLight — "
                  "not adding a second sun", flush=True)
        if FROZEN_SUN and not has_distant:
            el = math.radians(FROZEN_SUN_ELEV_DEG)
            az = math.radians(FROZEN_SUN_AZ_DEG)
            to_ground = Gf.Vec3d(-math.cos(el) * math.sin(az),
                                 -math.cos(el) * math.cos(az),
                                 -math.sin(el)).GetNormalized()
            sun_path = "/World/FrozenSun"
            sun = UsdLux.DistantLight.Define(stage, Sdf.Path(sun_path))
            sun.CreateIntensityAttr(FROZEN_SUN_INTENSITY)
            sun.CreateAngleAttr(0.53)      # the sun's real angular diameter
            xf = UsdGeom.Xformable(sun.GetPrim())
            xf.ClearXformOpOrder()
            xf.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(
                Gf.Vec3d(0, 0, -1), to_ground).GetQuat()))
            print("[frozen-light] sun at '{0}' (elev {1:.0f} deg, az {2:.0f} "
                  "deg, intensity {3:.0f})".format(
                      sun_path, FROZEN_SUN_ELEV_DEG, FROZEN_SUN_AZ_DEG,
                      FROZEN_SUN_INTENSITY), flush=True)

        # 4) Say what the scene is lit by, so the next run's log answers the
        #    question "was it lit?" without anyone opening the USD.
        # `HasAPI(LightAPI)` rather than `IsA(<some light base>)`: the base
        # class was renamed across USD versions (Light -> Boundable/
        # NonboundableLightBase) and every light schema applies LightAPI in
        # all of them, so this is the one spelling that cannot go stale.
        # `Stage.Traverse()` visits ACTIVE prims only, which is exactly the
        # census wanted here — the strays turned off above are meant to be
        # absent from it. Do not read this number as "lights in the file".
        active = [p.GetPath().pathString for p in stage.Traverse()
                  if p.HasAPI(UsdLux.LightAPI)]
        print("[frozen-light] the scene will render lit by {0} active light "
              "prim(s): {1}".format(len(active), ", ".join(active[:6]) or
                                    "<NONE — the scene will render BLACK>"),
              flush=True)

    def _finish_frozen_scene(self, stage):
        """Everything the generated path does AFTER the geometry exists."""
        for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                     "/rtx/pathtracing/fractionalCutoutOpacity"):
            carb.settings.get_settings().set_bool(_key, True)
        app = omni.kit.app.get_app()
        for _ in range(30):
            app.update()
        # Looks first: a texture that cannot resolve renders as a flat colour
        # whatever the lighting is, and the rebase has to happen before
        # anything asks the renderer for a frame.
        self._rebase_local_assets(stage)
        # Then the bindings the per-prim reference arcs dropped — without this
        # the burn scar / mud scour render as untextured grey.
        self._rebind_frozen_overlays(stage)
        # BEFORE the banner: the banner prints a census of what the cell is
        # lit by, and the honest answer has to include what we just added.
        self._light_frozen_scene(stage)
        self._print_frozen_banner(stage)
        self._add_scene_colliders(stage)
        self._write_frozen_annotations()

    def _print_frozen_banner(self, stage):
        gen = stage.GetPrimAtPath(SCENE_PARENT)
        scopes = [c.GetName() for c in gen.GetChildren()] if gen and gen.IsValid() \
            else []
        print("\n" + "=" * 72, flush=True)
        print("FROZEN SCENE - {0}".format(os.path.basename(FROZEN_USD)))
        print("  cell        {0}".format(FROZEN_CELL))
        print("  scopes      {0}".format(
            ", ".join(sorted(scopes)[:12]) or "<none — the reference failed>"))
        print("  colliders   {0}".format(COLLIDERS))
        print("  NEXT        map camera, then {0} PX4 drone(s)"
              .format(len(DRONE_CONFIGS)))
        print("=" * 72 + "\n", flush=True)

    def _write_frozen_annotations(self):
        """The cell's own GT, reshaped into the three names the stack reads.

        Same gate as the generated path (`GT_ANNOTATIONS`) and the same two
        destination directories, so a mission does not have to know which kind
        of scene it is flying. The difference is the SOURCE: nothing is
        measured off the stage, because the cell already shipped the
        measurements — see `utils/frozen_annotations.py`.

        A frozen cell's region file carries `search` and `damage`, NOT `burn`
        and `affected`; the scene overlay must say
        `search_area_scene_key: 'search'` (with `search_area_pad_m: 0.0`, the
        pad being already in the polygon). A `search_area_source: scene` run
        whose overlay still asks for `burn` will die at launch with the
        planner's own "no usable 'burn' polygon (entries: [...])" message,
        which is the right failure: loud, at t=0, and it names the keys the
        file does have.
        """
        if os.environ.get("GT_ANNOTATIONS", "off").strip().lower() not in (
                "1", "true", "yes", "on"):
            return
        scene = os.environ.get("RESULTS_SCENE", "").strip()
        if not scene:
            print("[frozen-gt] GT_ANNOTATIONS is on but RESULTS_SCENE is "
                  "unset — nothing to name the files after, skipping")
            return
        try:
            import scene_annotations as sa
        except ImportError as exc:
            print(f"[frozen-gt] unavailable: {exc}")
            return
        repo = os.path.normpath(
            os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "..", ".."))
        people_doc, hints_doc = frozen_gt.load_cell(
            FROZEN_USD, read_text=_frozen_read_text)
        if hints_doc is None:
            print(f"[frozen-gt] {FROZEN_CELL}: no GT_hints.json — no obstacle "
                  f"boxes and NO SEARCH AREA. A search_area_source: scene run "
                  f"will refuse to start.")
        if people_doc is None:
            print(f"[frozen-gt] {FROZEN_CELL}: no GT_people.json — the scorer "
                  f"has no answer key")
        frozen_gt.write_all(scene, sa.annotation_dirs(repo), people_doc,
                            hints_doc, scene_usd=FROZEN_USD)

    # ----------------------- GENERATED SCENE -----------------------

    def _build_generated_scene(self, stage, s):
        """Author the plat, then everything that needs the geometry to exist."""
        info = {}
        st = build_scene(stage, SCENE_CONFIG, s, arch_dir=ARCH_DIR, seed=SEED,
                         burn_frac=BURN_FRAC, elapsed=ELAPSED, poles=POLES,
                         parent_path=SCENE_PARENT, people_json=PEOPLE_JSON,
                         info_out=info, spec_overrides=SPEC_OVERRIDES)
        # The extra_args form only covers startup; re-assert once the stage is
        # composed so car glass stays see-through for the whole run.
        for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                     "/rtx/pathtracing/fractionalCutoutOpacity"):
            carb.settings.get_settings().set_bool(_key, True)
        app = omni.kit.app.get_app()
        for _ in range(30):
            app.update()
        self._print_scene_banner(st)
        self._add_scene_colliders(stage)
        self._write_scene_annotations(stage, st)

    @staticmethod
    def _add_colliders_trimesh(root_prim):
        """Static colliders for EVERYTHING under root, as exact triangle
        meshes ("none" approximation), skipping point-less stub meshes.

        NOT the stock `add_colliders`: that cooks a CONVEX HULL per mesh,
        and on a flattened dataset cell the debris fields are MERGED meshes
        (the tornado plank field is 1,578 boards in 15 meshes) — a convex
        hull over one of those is a phantom wall across the corridor. A
        static triangle mesh is exact, and PhysX only pays for it on
        contact. Added 2026-09-02 after the first live flight ghosted
        through houses with the default `ground` mode.
        """
        from pxr import UsdPhysics as _UP, UsdGeom as _UG
        applied = skipped = 0
        for p in Usd.PrimRange(root_prim):
            if not (p.IsA(_UG.Mesh) or p.IsA(_UG.Gprim)):
                continue
            if p.IsA(_UG.Mesh):
                pts = p.GetAttribute("points")
                if not pts or not pts.Get():
                    skipped += 1
                    continue
            _UP.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
            if p.IsA(_UG.Mesh):
                _UP.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set(
                    "none")
            applied += 1
        return applied, skipped

    def _add_scene_colliders(self, stage):
        if COLLIDERS == "off":
            print("[scene] colliders: OFF (SUBURB_COLLIDERS=off)", flush=True)
            return
        target = (SCENE_PARENT + "/ground") if COLLIDERS == "ground" else "/World/stage"
        prim = stage.GetPrimAtPath(target)
        if not (prim and prim.IsValid()):
            carb.log_warn("{0} not found - no colliders added.".format(target))
            return
        t0 = time.time()
        if COLLIDERS == "all":
            applied, skipped = self._add_colliders_trimesh(prim)
            note = " ({0} mesh collider(s) as trimesh, {1} empty skipped)".format(
                applied, skipped)
        else:
            add_colliders(prim)
            note = ""
        app = omni.kit.app.get_app()
        for _ in range(10):
            app.update()
        print("[scene] colliders on {0} in {1:.0f}s{2}"
              .format(target, time.time() - t0, note), flush=True)

    def _write_scene_annotations(self, stage, st):
        """Ground truth for a GENERATED scene, from the generator's own record.

        The survivors are the point of these scenes and they are the one class
        nothing else can supply: their positions exist only in the people pass's
        `humans.json`, which is written from the same records the scenario was
        authored from. Reading GT from there means the boxes and the scenario
        cannot disagree.

        Off unless GT_ANNOTATIONS is set — it overwrites the annotation file for
        RESULTS_SCENE, and a hand-authored file for a Nucleus stage of the same
        name is not something to clobber by default.
        """
        if os.environ.get("GT_ANNOTATIONS", "off").strip().lower() not in (
                "1", "true", "yes", "on"):
            return
        scene = os.environ.get("RESULTS_SCENE", "").strip()
        if not scene:
            print("[annotations] GT_ANNOTATIONS is on but RESULTS_SCENE is "
                  "unset — nothing to name the file after, skipping")
            return
        try:
            import scene_annotations as sa
        except ImportError as exc:
            print(f"[annotations] unavailable: {exc}")
            return
        # launch_scripts/ -> isaac-sim/ -> simulation/ -> repo root
        repo = os.path.normpath(
            os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "..", ".."))
        boxes = []
        pj = (st or {}).get("people_json") if isinstance(st, dict) else None
        if pj:
            boxes += sa.boxes_from_people(sa.people_records(pj))
        # Everything else the generator placed that maps to a class, measured
        # off the composed stage. `placements` is only present when the API
        # hands it back; people alone are still worth writing without it.
        placements = (st or {}).get("placements") if isinstance(st, dict) else None
        if placements:
            boxes += sa.boxes_from_placements(stage, placements)
        # THE OBSTACLES go to a SEPARATE file. The GT the GCS draws and the
        # scorer reads is PEOPLE ONLY — that is what the benchmark searches
        # for, and a box per house and tree is noise there. But a by-reference
        # scene has no placements for its houses and trees (each is one
        # referenced prim under <parent>/inst), and the 3D lawnmower needs
        # exactly those to fly over (search_baselines/clearance.py). So they
        # are measured off the stage the same way and written as
        # `<scene>_obstacles.json` beside the GT, where only the planner's
        # known-obstacle loader looks. Cars come from the survivor plan's own
        # scope; the kerb/driveway fleet is inside the referenced plat.
        parent = ((st or {}).get("parent") if isinstance(st, dict) else None) \
            or "/World/stage/generated"
        obstacles = sa.boxes_from_scopes(stage, [
            (parent + "/inst", {"h_": "house", "t_": "tree", "blocker_": "tree"}),
            (parent + "/people_cars", {"": "car"}),
        ])
        if not boxes and not obstacles:
            print("[annotations] nothing to write (no people_json, no placements)")
            return
        if boxes:
            sa.write_annotations(scene, boxes, sa.annotation_dirs(repo))
        if obstacles:
            sa.write_annotations(scene + "_obstacles", obstacles,
                                 sa.annotation_dirs(repo))

        # THE DISASTER-AFFECTED AREA goes to a THIRD file. The boxes above say
        # WHERE the answer is; this says where it is WORTH LOOKING, which is
        # what lets the planner size a search area to the fire instead of to
        # the plat. `burn` is the ground the front reached at `elapsed_s`;
        # `affected` is that ellipse run on until it covers the survivor
        # furthest AHEAD of the front (or 0.20*span, whichever is longer):
        # the people pass stages figures ahead of the front and runs the
        # gridlock queue outbound, so part of the answer key is outside the
        # black by construction. The planner's scene overlay chooses which
        # to fly (`search_area_scene_key`): `burn` + a pad is the search
        # that hugs the damage and gives up the survivors staged beyond it;
        # `affected` finds everyone and, on the 1 km plat, is most of the
        # map again. `region` is the plat, so a consumer can tell "the fire
        # covers a fifth of the map" from "the fire IS the map". See
        # scene_gen/disaster/region.py.
        aff = (st or {}).get("affected_xy") if isinstance(st, dict) else None
        if aff:
            burn = (st or {}).get("burn_xy") or []
            rx0, ry0, rx1, ry1 = [float(v) for v in st["region"]]
            entries = [
                {"class": "burn", "polygon_xy": burn,
                 "t_s": st.get("t_burn_s")},
                {"class": "affected", "polygon_xy": aff,
                 "t_s": st.get("t_affected_s")},
                {"class": "region",
                 "polygon_xy": [[rx0, ry0], [rx1, ry0], [rx1, ry1],
                                [rx0, ry1]]},
                {"class": "meta",
                 "scene_config": st.get("scene_config"),
                 "seed": st.get("seed"),
                 "elapsed_s": st.get("elapsed_s"),
                 "span_s": st.get("span_s"),
                 "fire_origin_m": st.get("fire_origin_m"),
                 "fire_heading_deg": st.get("fire_heading_deg"),
                 # WHICH BOUND SIZED THE POLYGON. "people" means the affected
                 # area was grown until it covered the furthest-ahead survivor
                 # (`people_lead_s` + `margin_s`); "lead_frac" means nobody was
                 # further out than `lead_frac * span` and the model's own
                 # floor won. A consumer that wants to know how much to trust
                 # the area wants this line.
                 "lead_frac": st.get("lead_frac"),
                 "lead_s": st.get("lead_s"),
                 "lead_bound": st.get("lead_bound"),
                 "people_lead_s": st.get("people_lead_s"),
                 "margin_s": st.get("margin_s")},
            ]
            sa.write_annotations(scene + "_region", entries,
                                 sa.annotation_dirs(repo))

            # AND THEN CHECK IT AGAINST THE ANSWER KEY. A search area that
            # does not contain the people is a benchmark that cannot be won,
            # and the only way to know is to put the GT written a few lines
            # above through the polygon written on this one. Every person
            # outside is printed, because "3 of 41 are out" is a real finding
            # — the lead is derived from these same records, so anyone outside
            # means the derivation is wrong — and a silent count is not.
            a = region_poly.polygon_area(aff)
            bb = region_poly.polygon_bbox(aff)
            people = [b for b in boxes if b.get("class") == "person"]
            out = []
            in_burn = 0
            for b in people:
                cx, cy = b["bbox_world"]["center_xyz_m"][:2]
                if not region_poly.point_in_polygon(cx, cy, aff):
                    out.append((cx, cy))
                if burn and region_poly.point_in_polygon(cx, cy, burn):
                    in_burn += 1
            print("[annotations] affected area: {0} pts, {1:.0f} m2 "
                  "(bbox {2:.0f}..{3:.0f}, {4:.0f}..{5:.0f}); t+{6:.0f} s, "
                  "lead {7:.0f} s bound by {8}; people inside {9}/{10}"
                  .format(len(aff), a, bb[0], bb[2], bb[1], bb[3],
                          st.get("t_affected_s") or 0.0,
                          st.get("lead_s") or 0.0, st.get("lead_bound"),
                          len(people) - len(out), len(people)))
            # The number a `search_area_scene_key: burn` run is scored
            # against: survivors on the scorched ground itself. The rest were
            # staged past the front and are outside that search by design.
            if burn:
                print("[annotations] burn area: {0:.0f} m2 at t+{1:.0f} s; "
                      "people inside {2}/{3} (the tight search; the rest "
                      "sit past the front)".format(
                          region_poly.polygon_area(burn),
                          st.get("t_burn_s") or 0.0, in_burn, len(people)))
            for cx, cy in out:
                print("[annotations]   WARNING person at ({0:.1f}, {1:.1f}) is "
                      "OUTSIDE the affected polygon".format(cx, cy))

    def _print_scene_banner(self, st):
        r = st["region"]
        print("\n" + "=" * 72, flush=True)
        print("SCENE BUILT - {0} ({1:.0f} x {2:.0f} m, by reference)".format(
            st["scene_config"], r[2] - r[0], r[3] - r[1]))
        print("  assembly    {0:.0f} s".format(st["seconds"]))
        print("  archetypes  {0}".format(st["arch_dir"]))
        print("  fire        t+{0:.0f} s elapsed over a {1:.0f} s span "
              "(burn_frac {2:.2f}, seed {3})".format(
                  st["elapsed_s"], st["span_s"], st["burn_frac"], st["seed"]))
        print("  houses      {0} referenced ({1} missing); {2}".format(
            st["houses"], st["houses_missing"],
            ", ".join("%s=%d" % kv for kv in sorted(st["house_tally"].items()))))
        print("  trees       {0} referenced ({1} missing); {2}".format(
            st["trees"], st["trees_missing"],
            ", ".join("%s=%d" % kv for kv in sorted(st["tree_tally"].items()))))
        print("  ground scar {0} band(s); {1} Flow emitter(s)".format(
            st["bands"], st["flow"]))
        if st.get("affected_xy"):
            print("  affected    {0:.0f} m2 ellipse at t+{1:.0f} s, lead "
                  "{2:.0f} s by {3} (burn {4:.0f} m2 at t+{5:.0f} s)".format(
                      region_poly.polygon_area(st["affected_xy"]),
                      st["t_affected_s"], st.get("lead_s") or 0.0,
                      st.get("lead_bound"),
                      region_poly.polygon_area(st.get("burn_xy") or []),
                      st["t_burn_s"]))
        print("  people      {0} authored ({1} alive), {2} car(s), "
              "{3} blocker(s) -> {4}".format(
                  st["people"], st["people_alive"], st["cars"], st["blockers"],
                  st["people_json"]))
        for _name in ppl.SCENARIOS:
            print("    {0:<18} {1:>3}".format(
                _name, st["people_tally"].get(_name, 0)))
        if st["poles"]:
            print("  poles       {0} locator marker(s) (PEOPLE_POLES)".format(
                st["poles"]))
        print("  NEXT        colliders, map camera, then {0} PX4 drone(s)"
              .format(len(DRONE_CONFIGS)))
        print("=" * 72 + "\n", flush=True)

    def _print_drone_banner(self):
        print("\n" + "=" * 72, flush=True)
        print("DRONES UP - {0} PX4 multirotor(s) on {1}".format(
            len(DRONE_CONFIGS),
            os.path.basename(FROZEN_USD) if _FROZEN else SCENE_CONFIG))
        for cfg in DRONE_CONFIGS:
            print("  robot_{0}  domain {0}  ({1:.1f}, {2:.1f}, {3:.1f}) m  "
                  "orient {4}".format(cfg["domain_id"], cfg["x_m"], cfg["y_m"],
                                      cfg["z_m"], cfg["orient"]))
        print("  overhead    {0} @ {1:.0f} m coverage, centred "
              "({2:.0f}, {3:.0f})".format(OVERHEAD_TOPIC, OVERHEAD_COVERAGE_M,
                                          OVERHEAD_CENTER_X_M,
                                          OVERHEAD_CENTER_Y_M))
        print("  timeline    {0}".format(
            "PLAY" if self.play_on_start else "STOPPED (PLAY_SIM_ON_START)"))
        print("  PX4 is now booting; mission readiness gates on that, not on "
              "this banner.")
        print("=" * 72 + "\n", flush=True)

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        while simulation_app.is_running() and not self.stop_sim:
            world = World.instance()
            if world is not None and hasattr(world, '_scene'):
                if self._zed_slice_groups > 1:
                    active = self._zed_slice_tick % self._zed_slice_groups
                    for group, gate in self._zed_slice_gates:
                        gate.set(1 if group == active else 0)
                    self._zed_slice_tick += 1
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
