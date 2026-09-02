#!/usr/bin/env bash
# _stage2_env.sh — shared body of stage2_env_1robot.sh / stage2_env_2robot.sh.
# Not run directly: `NROBOTS=1 bash _stage2_env.sh [--cmd]`.
#
# Prints, to stdout:
#   (default)  the key-by-key rationale, then the exact edit_env.py command
#   --cmd      ONLY the command, so it can be piped:
#                  scripts/raven_live/stage2_env_2robot.sh --cmd | bash
#
# It never edits anything itself.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
N="${NROBOTS:?set NROBOTS=1 or 2}"
OUT_DIR="${OUT_DIR:-${REPO}/scripts/raven_live/out}"
SPAWN_FILE="${OUT_DIR}/spawns_${N}robot.json"
FREEZE_OUT_C="${FREEZE_OUT:-/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250}"
RESULTS_SCENE="${RESULTS_SCENE:-RavenSuburbTornado250}"

CMD_ONLY=0
[ "${1:-}" = "--cmd" ] && CMD_ONLY=1

if [ ! -f "$SPAWN_FILE" ]; then
  cat >&2 <<EOF
stage2_env: ${SPAWN_FILE} does not exist.

  Run the gate first — it is what re-validates the two casualties against the
  REAL frozen people file and writes this:

      scripts/raven_live/validate_freeze.sh

  Do NOT paste the SPAWN_CONFIGS out of _plans/raven_test_scene_runbook.md or
  osmo/missions/raven_single_shared_test.yaml: those are a HOST APPROXIMATION
  (that runbook's own caveat — the real build's deck_points can move which
  casualties the planner accepts).
EOF
  exit 1
fi

SPAWNS="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["SPAWN_CONFIGS"])' "$SPAWN_FILE")"
SUBBED="$(python3 -c 'import json,sys; print("YES" if json.load(open(sys.argv[1]))["substituted"] else "no")' "$SPAWN_FILE")"
IDS="$(seq -s, 1 "$N")"
MD5="$(md5sum "${REPO}/.env" | cut -d' ' -f1)"
EE="${REPO}/scripts/raven_live/edit_env.py"

# ── the command ────────────────────────────────────────────────────────────
# One --set per key. Order is cosmetic; edit_env.py updates in place and
# appends what does not exist under its own marked block.
build_cmd() {
  printf '%s --expect-md5 %s' "$EE" "$MD5"
  printf ' \\\n  --set %s' \
    "ISAAC_SIM_SCRIPT_NAME=example_multi_drone_scene_import.py" \
    "FROZEN_SCENE=${FREEZE_OUT_C}" \
    "SCENE_CONFIG=" \
    "RESULTS_SCENE=${RESULTS_SCENE}" \
    "NUM_ROBOTS=${N}"
  printf " \\\\\n  --set 'SPAWN_CONFIGS=%s'" "$SPAWNS"
  printf ' \\\n  --set %s' \
    "SPAWN_HEIGHT_M=1.0" \
    "DRONE_Z_M=1.0" \
    "MAP_ANCHOR_ENU=true" \
    "ENABLE_LIDAR=false" \
    "ZED_WIDTH=480" \
    "ZED_HEIGHT=300" \
    "ZED_PITCH_DEG=0" \
    "GT_ANNOTATIONS=on" \
    "PLAY_SIM_ON_START=true" \
    "AUTOLAUNCH=true" \
    "ISAAC_SIM_USE_STANDALONE=true" \
    "ISAAC_SIM_HEADLESS=false" \
    "RAYFRONTS_MODE=shared" \
    "VLM_URL=http://offboard-compute:8000/v1" \
    "RAYFRONTS_COMPUTE_PROB=True" \
    "START_RAYFRONTS_SERVER=true" \
    "START_DETECTOR_SERVER=false" \
    "START_ITM_SERVER=false" \
    "START_VLM_SERVER=true" \
    "CONAVGPT2_VLM_MODEL=Qwen/Qwen2.5-VL-3B-Instruct" \
    "CONAVGPT2_VLM_QUANT=nf4" \
    "RAYFRONTS_ROBOT_IDS=${IDS}" \
    "RAYFRONTS_CONFIG=shared_humans" \
    "RECORD_BAGS=false"
  printf '\n'
}

if [ "$CMD_ONLY" -eq 1 ]; then
  build_cmd
  exit 0
fi

cat <<EOF
═══════════════════════════════════════════════════════════════════════════════
 STAGE 2 ENV — ${N} robot(s), frozen ${RESULTS_SCENE}, shared off-board
 RayFronts, \`person\` search.

   generated                    $(date -u +%Y-%m-%dT%H:%M:%SZ)
   spawns from                  ${SPAWN_FILE}
   casualty substitution        ${SUBBED}
   .env md5 at generation time  ${MD5}

 Nothing is applied by this script. Read the table, then run the command at
 the bottom (or:  scripts/raven_live/stage2_env_${N}robot.sh --cmd | bash).

 The --expect-md5 in that command is a CONCURRENT-SESSION GUARD: if another
 session touches .env between now and when you run it, edit_env.py refuses
 instead of clobbering. Re-run this script for a fresh md5.
═══════════════════════════════════════════════════════════════════════════════
EOF

cat <<'EOF'

WHICH SCENE ───────────────────────────────────────────────────────────────────
ISAAC_SIM_SCRIPT_NAME  The generic multi-drone importer. Stage 1's
                       suburb_tornado_launch_script.py NEVER spawns a drone
                       (grep-verified: no SPAWN_CONFIGS / spawn_px4_multirotor_
                       node / DRONE_CONFIGS anywhere in it). This is the only
                       launcher with both a drone and the FROZEN_SCENE path.
FROZEN_SCENE           The cell, as an ABSOLUTE container path.
                       frozen_annotations.resolve_cell() tries an absolute spec
                       first, then falls back to "the single .usd in this
                       directory" — our path is not shaped
                       <Disaster>/<Locale>/level_<n>/<k>, so that fallback is
                       the one that fires. validate_freeze.sh checks there IS
                       exactly one.
SCENE_CONFIG=          CLEARED. FROZEN_SCENE beats SCENE_CONFIG (_FROZEN over
                       _BUILT), but a leftover value makes the launcher print
                       "SCENE_CONFIG=... is IGNORED" on every start. Empty, not
                       removed: the compose entry is ${SCENE_CONFIG:-}, so
                       unset and empty reach the container identically.
RESULTS_SCENE          Names the annotation files GT_ANNOTATIONS writes and the
                       scene compare_to_groundtruth.py --scene reads. Needed on
                       offboard-compute TOO (search_planner aborts at
                       construction without it) — that service declares its own
                       ${RESULTS_SCENE:-}, so this one .env key covers
                       isaac-sim, the robots and offboard-compute.
SPAWN_CONFIGS          Re-validated against the REAL frozen GT_people.json, not
                       the runbook's host approximation. domain_id defaults to
                       the list index, so entry 1 -> robot_1 -> domain 1.
NUM_ROBOTS             Sizes the robot-desktop replicas AND (via
                       offboard_compute.sh) the shared mapper's default robot
                       list. The importer warns loudly if it disagrees with the
                       SPAWN_CONFIGS length.

THE DRONE, THE CAMERA, THE FRAMES ─────────────────────────────────────────────
SPAWN_HEIGHT_M         The one the importer actually reads
                       (SPAWN_HEIGHT_ABOVE_FLOOR_M, its default is 0.5);
                       robot.launch.xml reads it too as spawn_height_m.
DRONE_Z_M              NOT read by example_multi_drone_scene_import.py —
                       grep-verified, only modular_house_preview_launch_script
                       .py reads it. Set equal to SPAWN_HEIGHT_M anyway,
                       because the runbook and the mission file both carry it
                       and a future reader must not "fix" a disagreement that
                       does not exist.
MAP_ANCHOR_ENU         Publish the MEASURED world->map instead of identity.
                       `map` is anchored at takeoff and these drones spawn
                       3-80 m off the origin, so identity floats every plan off
                       the sim ground in Foxglove. Consumed by the ROBOT
                       containers, not by isaac-sim.
ENABLE_LIDAR=false     raven_nav/RayFronts are RGB+depth only (no lidar topic
                       in the plan's §2.1 subscription list) and the lidar
                       costs ~3x the sim rate (cameras 53 -> 17 Hz, measured).
                       The compose entry is ${ENABLE_LIDAR:-} ON PURPOSE:
                       EMPTY means "the launcher's own default", which here is
                       TRUE. So this must be the literal string false, never
                       blank.
ZED_WIDTH / ZED_HEIGHT Pegasus' native 480x300; shared_humans.yaml resizes to
                       480x480 regardless.
ZED_PITCH_DEG=0        *** A CHANGE FROM TODAY'S .env, WHICH HAS 30. ***
                       This tilts the camera PRIM in Isaac. The URDF models no
                       mount pitch, so TF cannot see the tilt and every
                       consumer must be told separately — search_planner
                       (CoNavGPT / VLFM) reads this same variable for exactly
                       that reason. RayFronts does NOT: its
                       multi_ros2isaacsim.yaml takes the pose straight off
                       /{robot}/odometry_conversion/odometry with
                       src_coord_system: flu and assumes the camera is aligned
                       with the body. Grep-verified: ZED_PITCH_DEG appears
                       nowhere in raven_nav, semantic_search_task or
                       common/rayfronts_configs. Left at 30, every voxel
                       unprojects 30 deg off and the map is quietly,
                       systematically wrong — which would read as "RayFronts
                       can't find anything", not as a config error.

GROUND TRUTH ──────────────────────────────────────────────────────────────────
GT_ANNOTATIONS=on      Writes <RESULTS_SCENE>.json / _obstacles.json /
                       _region.json into BOTH gcs_visualizer/annotations and
                       raven_nav/annotations at scene-load time. WARNING: that
                       auto-write uses frozen_annotations.people_boxes' UPRIGHT
                       (0.7, 0.7, 1.8) box centred at z+0.9 — correct for a
                       standing survivor, WRONG for a LYING tornado casualty —
                       and it CLOBBERS raven_nav/annotations/<scene>.json every
                       time the scene loads. The checklist has a post-load step
                       that overwrites it correctly with
                       scene_gen/tools/people_json_to_annotations.py.

ISAAC RUN MODE ────────────────────────────────────────────────────────────────
PLAY_SIM_ON_START      The compose entry is ${PLAY_SIM_ON_START} with NO :-
                       default, so an unset value renders EMPTY and
                       PegasusApp.play_on_start becomes False: the timeline
                       never starts, /clock never ticks, and every takeoff
                       times out looking like a PX4 fault. Say it out loud.
AUTOLAUNCH             The container's entrypoint only sends the launch line
                       when this is 'true'.
ISAAC_SIM_HEADLESS     false = keep the GUI, because a human is watching this
                       run. Flip to true for an unattended one.

SHARED RAYFRONTS ──────────────────────────────────────────────────────────────
RAYFRONTS_MODE=shared  semantic_search_task waits on /robot_N/rayfronts/status
                       instead of spawning rayfronts.launch.xml per robot.
                       Passed to the robot containers by BARE NAME
                       (`- RAYFRONTS_MODE`), so an EMPTY value is NOT "unset"
                       and would beat the node default — set it, or leave the
                       key out entirely; never blank.
VLM_URL                Same bare-name rule. Where raven's lvlm_client and
                       CoNavGPT2's vlm_client both look.
RAYFRONTS_COMPUTE_PROB Softmax over the query set rather than raw cosine
                       similarity. Raw similarities top out around 0.16 and the
                       goal's 0.5 gates would reject everything.

OFFBOARD-COMPUTE ──────────────────────────────────────────────────────────────
START_RAYFRONTS_SERVER encoder_server + multi_robot_mapping_server, once, here.
START_DETECTOR_SERVER  OFF. raven + shared rayfronts use no 2D detector; the
                       default true would load YOLO-World + MobileSAM (~1.6
                       GiB) onto the card Isaac already fills.
START_ITM_SERVER       OFF (BLIP-2, VLFM only).
START_VLM_SERVER       ON, so raven's LVLM-guided tier has an endpoint. THIS IS
                       THE FIRST THING TO DROP if the card OOMs — see the
                       fallback ladder in flight_checklist.md.
CONAVGPT2_VLM_MODEL    3B nf4: the single-16GB-card setting. Do NOT flip to 7B
                       here.
RAYFRONTS_ROBOT_IDS    Explicit rather than left to offboard_compute.sh's
                       1..NUM_ROBOTS default, so the mapper's dataset.robot_ids
                       cannot silently disagree with how many robots came up.
RAYFRONTS_CONFIG       WP-B's human-tuned config
                       (common/rayfronts_configs/shared_humans.yaml):
                       vox_size 0.5, max_pts_per_frame 4000,
                       sem_pruning_thresh 1 — all small-target tuning.

HOUSEKEEPING ──────────────────────────────────────────────────────────────────
RECORD_BAGS=false      The ROBOT's own autolaunch recorder. Bags for the
                       recorded runs come from mission_runner.py's Recorder
                       (record.scope: both); both writing at once doubles disk
                       and CPU for nothing.

DELIBERATELY NOT SET ──────────────────────────────────────────────────────────
RAVEN_LVLM             bare-name passthrough; unset = raven_nav's own default
                       (true). Setting it to "" is an EMPTY STRING, which BEATS
                       that default. Only ever set it to the literal "false".
RAVEN_LVLM_RAY_THRESHOLD / RAVEN_LVLM_INTERVAL_S
                       same shape; unset = the 0.9 / 30.0 node defaults. Tune
                       from the first run's debug/voxel_table peaks.
RAYFRONTS_SRC          offboard-compute's own compose entry already mounts
                       common/rayfronts at the default path.
FASTDDS_BUILTIN_TRANSPORTS
                       DO NOT TOUCH. common/fastdds.xml is what actually pins
                       the transport; changing this env var once took the
                       readiness gate from 7/8 to 0/8.
FROZEN_LIGHT / FROZEN_SKY / FROZEN_SUN*
                       unset = the launcher's own "on" defaults, which is what
                       this cell needs. A frozen TORNADO cell ships ZERO lights
                       (freeze.DEACTIVATE_DEFAULT takes the sky away with the
                       default environment); without the re-light the drones'
                       cameras render a black world and nothing is ever
                       detected.
FROZEN_REBASE_ASSETS   unset = "on", a no-op on this machine (the assets are
                       local and present) and the fix on a pod. Leave it.
FROZEN_DATASET_ROOT    unset. Only used when FROZEN_SCENE is relative.
TOR_* / PEOPLE_JSON / FREEZE_*
                       NOT declared in the isaac-sim compose service at all, so
                       they never reach the container from .env — and they are
                       inert on the FROZEN_SCENE path anyway (the people plan
                       only runs under _BUILT). Stage 1 passes them on the
                       pane's command line instead.

ONE MORE DECISION FOR YOU ─────────────────────────────────────────────────────
.env line 179 says LOCAL_PLANNER=mighty. raven_nav publishes a global plan; the
LOCAL planner is what flies it, and the vendored asm_mighty arm has never been
flown on this branch. For a bring-up test whose whole point is the
RayFronts/raven path, consider pinning the known-good one so only one thing is
new:

    scripts/raven_live/edit_env.py --set LOCAL_PLANNER=droan

EOF

echo "THE COMMAND ───────────────────────────────────────────────────────────────────"
echo
build_cmd
echo
