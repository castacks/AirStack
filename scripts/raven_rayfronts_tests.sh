#!/usr/bin/env bash
# raven_rayfronts_tests.sh — THE one command that runs every test in the
# RAVEN single-agent + shared off-board RayFronts build
# (_plans/raven_single_rayfronts_shared_plan.md, §5 WP-C item 3).
#
# Two tiers:
#   (a) HOST, pure-python tests via `uv run` — no ROS, no torch, no rclpy:
#         raven_nav            robot/ros_ws/src/global/planners/raven_nav/test
#         rayfronts (non-ros)  common/rayfronts/tests  (-m "not ros and not
#                               cuda and not torch" — see that dir's conftest)
#         semantic_search_task robot/ros_ws/src/global/planners/
#                               semantic_search_task/test
#         scene_gen             scene_gen/tests/test_people_json_to_annotations.py
#   (b) CONTAINER, a throwaway `docker run --rm` of the robot image (the same
#       image robot-desktop/offboard-compute use — tag derived the way
#       robot/docker/docker-compose.yaml does, via `docker compose config`),
#       repo mounted like compose does PLUS common/rayfronts and
#       common/rayfronts_configs (which no compose file mounts today — see
#       the run-raven-single-shared runbook), ROS + workspace sourced,
#       rayfronts_cpp on PYTHONPATH:
#         raven_nav             ALL tests (includes test/integration once it
#                                exists — nothing special-cased for it)
#         rayfronts             `-m "not cuda"`, plus `-m cuda` with --gpu
#         semantic_search_task  same test dir as the host tier
#
# This script NEVER starts/stops/touches the running `isaac-sim` container —
# the only container it runs is one anonymous, `--rm`, no `--name` throwaway
# for the robot image, always removed on exit.
#
# A suite that hasn't been written yet by another WP (rayfronts host tests
# before WP-B lands them, raven_nav's test/integration before WP-A lands it,
# `-m cuda` before any cuda-marked test exists) reports SKIP, not FAIL — see
# the plan header: "other agents are rewriting raven_nav and rayfronts
# concurrently ... that is fine". Only an actual test FAILURE, an import/
# collection error, or an infra problem (missing image, docker not found)
# makes this script exit non-zero.
#
# Usage:
#   scripts/raven_rayfronts_tests.sh [--gpu] [--host-only] [-h|--help]
#     --gpu        add `--gpus all` to the container run and also execute the
#                  rayfronts `-m cuda` tests (skip both otherwise — no GPU is
#                  touched by a default run). Use briefly; see the runbook
#                  for the VRAM budget before running this on a loaded box.
#     --host-only  run only the HOST tier (uv run) — skip docker entirely.
set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

GPU=false
HOST_ONLY=false
for arg in "$@"; do
  case "$arg" in
    --gpu) GPU=true ;;
    --host-only) HOST_ONLY=true ;;
    -h|--help)
      sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "unknown argument: $arg (see --help)" >&2
      exit 2
      ;;
  esac
done

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${RAVEN_TEST_LOG_DIR:-/tmp/raven_rayfronts_tests_${TS}}"
mkdir -p "$LOG_DIR"

# name|status|summary|logfile, printed as a table at the end.
declare -a SUMMARY=()
OVERALL_RC=0

log() { echo "[raven_rayfronts_tests] $*"; }

env_var() {   # env_var <NAME> — read a bare KEY="value" line from .env
  grep -E "^$1=" "$REPO_ROOT/.env" 2>/dev/null | tail -1 | cut -d= -f2- \
    | sed -e 's/^"//' -e 's/"$//'
}

# ── image tag, derived the way robot/docker/docker-compose.yaml does ────────
image_tag() {
  local tag=""
  if command -v docker >/dev/null 2>&1; then
    tag="$(docker compose config --format json 2>/dev/null \
      | python3 -c 'import json,sys
try:
    d = json.load(sys.stdin)
    print(d["services"]["robot-desktop"]["image"])
except Exception:
    pass' 2>/dev/null)"
  fi
  if [ -z "$tag" ]; then
    # Fallback: hand-build the same template docker-compose.yaml uses
    # (image: &desktop_image ${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:
    #  v${VERSION}_robot-x86-64_${DOCKER_IMAGE_BUILD_MODE}) straight from .env,
    # in case `docker compose config` itself is unavailable.
    local reg name ver mode
    reg="$(env_var PROJECT_DOCKER_REGISTRY)"
    name="$(env_var PROJECT_NAME)"
    ver="$(env_var VERSION)"
    mode="$(env_var DOCKER_IMAGE_BUILD_MODE)"
    tag="${reg}/${name}:v${ver}_robot-x86-64_${mode}"
    log "docker compose config did not resolve an image tag — using the .env-derived fallback: $tag"
  fi
  echo "$tag"
}

# ── result bookkeeping ────────────────────────────────────────────────────
# pytest exit codes: 0 all passed, 1 tests failed, 2 interrupted, 3 internal
# error, 4 usage error, 5 no tests collected. 5 is treated as SKIP (a suite
# nobody has written test cases matching the filter for yet — e.g. no
# cuda-marked rayfronts test exists in the tree today), never as a failure.
record_from_rc() {   # record_from_rc <name> <rc> <summary> <logfile>
  local name="$1" rc="$2" summary="$3" logfile="$4" status
  if [ "$rc" = "SKIP" ]; then
    status=SKIP
  elif [ "$rc" = "0" ]; then
    status=PASS
  elif [ "$rc" = "5" ]; then
    status=SKIP
    summary="no tests collected${summary:+ ($summary)}"
  else
    status=FAIL
    OVERALL_RC=1
  fi
  [ -z "$summary" ] && summary="rc=$rc — see $logfile"
  log "[$name] $status — $summary"
  SUMMARY+=("$name|$status|$summary|$logfile")
}

pytest_summary_line() {   # pytest_summary_line <logfile>
  grep -Eo '[0-9]+ (passed|failed|error(s)?|skipped|deselected|warning[s]?)[, ]*' "$1" \
    | tr -d '\n' | sed 's/[, ]*$//'
}

# ── (a) HOST tier — uv run, no ROS/torch/rclpy ───────────────────────────────
run_host_suite() {   # run_host_suite <name> <workdir> <uv-with-args...> -- <pytest-args...>
  local name="$1" workdir="$2"; shift 2
  local with_args=() pytest_args=() seen_dashdash=false
  for a in "$@"; do
    if [ "$a" = "--" ]; then seen_dashdash=true; continue; fi
    if [ "$seen_dashdash" = false ]; then with_args+=("$a"); else pytest_args+=("$a"); fi
  done
  local logfile="$LOG_DIR/host_${name}.log"
  if [ ! -e "$workdir" ]; then
    record_from_rc "host:$name" SKIP "not present yet: $workdir" "$logfile"
    return
  fi
  log "=== host: $name ($workdir) ==="
  if [ -d "$workdir" ]; then
    ( cd "$workdir" && uv run --no-project "${with_args[@]}" python -m pytest "${pytest_args[@]}" ) \
      > "$logfile" 2>&1
  else
    # A single file target (scene_gen's new test) — run from repo root.
    ( cd "$REPO_ROOT" && uv run --no-project "${with_args[@]}" python -m pytest "${pytest_args[@]}" ) \
      > "$logfile" 2>&1
  fi
  local rc=$?
  record_from_rc "host:$name" "$rc" "$(pytest_summary_line "$logfile")" "$logfile"
}

log "==================== HOST TIER (uv run) ===================="

run_host_suite raven_nav \
  "$REPO_ROOT/robot/ros_ws/src/global/planners/raven_nav" \
  --with numpy --with scipy --with scikit-learn --with pytest \
  --with opencv-python-headless -- \
  test -q

run_host_suite rayfronts \
  "$REPO_ROOT/common/rayfronts/tests" \
  --with pytest --with numpy -- \
  "$REPO_ROOT/common/rayfronts/tests" -q -m "not ros and not cuda and not torch"

run_host_suite semantic_search_task \
  "$REPO_ROOT/robot/ros_ws/src/global/planners/semantic_search_task" \
  --with pytest --with numpy -- \
  test -q

run_host_suite scene_gen_people_json \
  "$REPO_ROOT/scene_gen/tests/test_people_json_to_annotations.py" \
  --with pytest -- \
  scene_gen/tests/test_people_json_to_annotations.py -q

# ── (b) CONTAINER tier — throwaway docker run --rm of the robot image ───────
log "==================== CONTAINER TIER (docker run --rm) ===================="

if [ "$HOST_ONLY" = true ]; then
  log "--host-only passed — skipping the container tier entirely"
elif ! command -v docker >/dev/null 2>&1; then
  record_from_rc "container" FAIL "docker CLI not found on this host" "$LOG_DIR/container_stdout.log"
else
  IMAGE_TAG="$(image_tag)"
  log "image: $IMAGE_TAG"
  if ! docker image inspect "$IMAGE_TAG" >/dev/null 2>&1; then
    record_from_rc "container" FAIL \
      "image $IMAGE_TAG not found locally — build it first (airstack build / docker compose build robot-desktop)" \
      "$LOG_DIR/container_stdout.log"
  else
    # Container-side script: written to $LOG_DIR (mounted in), so results
    # come back as files rather than parsed out of docker's combined stdout.
    CONTAINER_SCRIPT="$LOG_DIR/run_container_tests.sh"
    cat > "$CONTAINER_SCRIPT" <<'CONTAINER_EOF'
#!/usr/bin/env bash
# Runs INSIDE the throwaway robot container. Writes one line per suite to
# $LOGDIR/container_results.tsv (name<TAB>rc<TAB>summary) plus a full pytest
# log per suite — the host side reads the tsv rather than parsing stdout.
#
# NO `set -u` here on purpose: /opt/ros/jazzy/setup.bash itself references
# unset variables (e.g. AMENT_TRACE_SETUP_FILES) — reproduced: `set -u` above
# this line turns that harmless, normal ROS sourcing into an immediate abort
# ("unbound variable") before a single suite runs. ROS's own setup scripts
# are not nounset-safe; don't fight that here.
GPU_MODE="${1:-cpu}"
LOGDIR=/root/raven_test_logs
RESULTS="$LOGDIR/container_results.tsv"
: > "$RESULTS"

echo "CONTAINER-HARNESS-START"

source /opt/ros/jazzy/setup.bash
WS_INSTALL=/root/AirStack/robot/ros_ws/install/setup.bash
if [ -f "$WS_INSTALL" ]; then
  source "$WS_INSTALL"
  echo "[container] sourced workspace install: $WS_INSTALL"
else
  echo "[container] no workspace install at $WS_INSTALL — raven_nav/semantic_search_task still resolve fine via cwd (each suite cd's into its source dir first)"
fi
# Prepended AFTER sourcing: install/setup.bash unconditionally puts ITS OWN
# per-package dirs at the FRONT of PYTHONPATH, so setting ours before
# sourcing would just get pushed behind them — verified empirically (see the
# WP-C report). This makes `import rayfronts` resolve to the LIVE-mounted
# common/rayfronts (not any stale colcon-installed copy) and puts the
# compiled rayfronts_cpp extension (image-baked; not part of the live mount,
# common/rayfronts/rayfronts/csrc has no build/ dir on the host) on the path.
export PYTHONPATH="/root/AirStack/common/rayfronts:/opt/rayfronts/rayfronts/csrc/build:${PYTHONPATH:-}"

# This image's system pytest crashes at PLUGIN LOAD (before collecting a
# single test) against several ament/launch_testing pytest11 entrypoints that
# predate this pytest's hookspec:
#   PluginValidationError: Plugin 'launch_testing' for hook
#   'pytest_pycollect_makemodule' ... Argument(s) {'path'} are declared in
#   the hookimpl but can not be found in the hookspec
# Reproduced with a bare `python3 -m pytest --version` in this exact image —
# disabling these is required for ANY `python3 -m pytest` invocation here,
# not something specific to this build's own test files.
PY_FLAGS=(-p no:launch_testing -p no:launch_ros -p no:ament_xmllint
          -p no:ament_flake8 -p no:ament_copyright -p no:ament_pep257
          -p no:ament_lint -p no:colcon_core_warnings_stderr)

echo "[container] python: $(python3 --version 2>&1)"
echo "[container] rclpy:  $(python3 -c 'import rclpy; print(rclpy.__file__)' 2>&1)"
echo "[container] torch:  $(python3 -c 'import torch; print(torch.__version__, "cuda_available=" + str(torch.cuda.is_available()))' 2>&1)"
echo "[container] rayfronts:     $(python3 -c 'import rayfronts; print(rayfronts.__file__)' 2>&1)"
echo "[container] rayfronts_cpp: $(python3 -c 'import rayfronts_cpp; print(rayfronts_cpp.__file__)' 2>&1)"

run_suite() {   # run_suite <name> <workdir> <pytest-args...>
  local name="$1" workdir="$2"; shift 2
  local log="$LOGDIR/container_${name}.log"
  if [ ! -d "$workdir" ]; then
    printf '%s\t%s\t%s\n' "$name" "SKIP" "directory not found: $workdir" >> "$RESULTS"
    return
  fi
  ( cd "$workdir" && python3 -m pytest "${PY_FLAGS[@]}" "$@" ) > "$log" 2>&1
  local rc=$?
  local summary
  summary="$(grep -Eo '[0-9]+ (passed|failed|error(s)?|skipped|deselected|warning[s]?)[, ]*' "$log" | tr -d '\n' | sed 's/[, ]*$//')"
  [ -z "$summary" ] && summary="rc=$rc"
  printf '%s\t%s\t%s\n' "$name" "$rc" "$summary" >> "$RESULTS"
}

run_suite raven_nav /root/AirStack/robot/ros_ws/src/global/planners/raven_nav \
  test -q
run_suite rayfronts_not_cuda /root/AirStack/common/rayfronts \
  tests -q -m "not cuda"
run_suite semantic_search_task /root/AirStack/robot/ros_ws/src/global/planners/semantic_search_task \
  test -q

if [ "$GPU_MODE" = "gpu" ]; then
  run_suite rayfronts_cuda /root/AirStack/common/rayfronts \
    tests -q -m "cuda"
fi

echo "CONTAINER-HARNESS-DONE"
CONTAINER_EOF
    chmod +x "$CONTAINER_SCRIPT"

    ENV_ARGS=(-e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_DOMAIN_ID=77)
    GPU_ARGS=()
    GPU_MODE_ARG=cpu
    if [ "$GPU" = true ]; then
      GPU_ARGS=(--gpus all)
      GPU_MODE_ARG=gpu
    fi
    MOUNTS=(
      -v "$REPO_ROOT/robot/ros_ws:/root/AirStack/robot/ros_ws"
      -v "$REPO_ROOT/common/ros_packages:/root/AirStack/robot/ros_ws/src/common"
      -v "$REPO_ROOT/common/rayfronts:/root/AirStack/common/rayfronts"
      -v "$REPO_ROOT/common/rayfronts_configs:/root/AirStack/common/rayfronts_configs"
      -v "$REPO_ROOT/common/fastdds.xml:/root/AirStack/robot/ros_ws/src/fastdds.xml"
      -v "$LOG_DIR:/root/raven_test_logs"
    )

    run_container() {   # run_container <extra network args...>
      docker run --rm "$@" \
        "${ENV_ARGS[@]}" "${GPU_ARGS[@]}" "${MOUNTS[@]}" \
        "$IMAGE_TAG" \
        bash /root/raven_test_logs/run_container_tests.sh "$GPU_MODE_ARG"
    }

    log "starting throwaway container (--network none, no --name, --rm) — never touches isaac-sim"
    rm -f "$LOG_DIR/container_results.tsv"
    run_container --network none > "$LOG_DIR/container_stdout.log" 2>&1
    DOCKER_RC=$?

    # The tsv is created (empty) almost immediately, so its mere EXISTENCE
    # says nothing — check the START sentinel (did the container-side script
    # even begin?) to decide whether a network-mode retry is warranted, and
    # the DONE sentinel (did it run to completion?) to decide whether the tsv
    # rows are trustworthy as a complete result set.
    if ! grep -q "^CONTAINER-HARNESS-START$" "$LOG_DIR/container_stdout.log" 2>/dev/null; then
      log "container never started under --network none (docker rc=$DOCKER_RC) — retrying once on the default bridge network"
      run_container > "$LOG_DIR/container_stdout.log" 2>&1
      DOCKER_RC=$?
    fi

    if grep -q "^CONTAINER-HARNESS-DONE$" "$LOG_DIR/container_stdout.log" 2>/dev/null; then
      while IFS=$'\t' read -r name rc summary; do
        [ -z "${name:-}" ] && continue
        record_from_rc "container:$name" "$rc" "$summary" "$LOG_DIR/container_${name}.log"
      done < "$LOG_DIR/container_results.tsv"
    else
      record_from_rc "container" FAIL \
        "container script did not run to completion (docker rc=$DOCKER_RC) — see $LOG_DIR/container_stdout.log" \
        "$LOG_DIR/container_stdout.log"
      # Best-effort: report whatever suites DID finish before it died.
      if [ -s "$LOG_DIR/container_results.tsv" ]; then
        while IFS=$'\t' read -r name rc summary; do
          [ -z "${name:-}" ] && continue
          record_from_rc "container:$name" "$rc" "$summary" "$LOG_DIR/container_${name}.log"
        done < "$LOG_DIR/container_results.tsv"
      fi
    fi
  fi
fi

if [ "$HOST_ONLY" = false ] && [ "$GPU" = false ]; then
  SUMMARY+=("container:rayfronts_cuda|SKIP|--gpu not passed|-")
fi

# ── summary table ────────────────────────────────────────────────────────
echo
log "==================== SUMMARY ===================="
printf '%-32s %-6s %s\n' "SUITE" "STATUS" "DETAIL"
printf '%-32s %-6s %s\n' "-----" "------" "------"
for row in "${SUMMARY[@]}"; do
  IFS='|' read -r name status summary logfile <<< "$row"
  printf '%-32s %-6s %s\n' "$name" "$status" "$summary"
done
echo
log "logs: $LOG_DIR"
if [ "$OVERALL_RC" -ne 0 ]; then
  log "RESULT: FAIL (at least one suite failed — see the table above and its log)"
else
  log "RESULT: PASS (failures/skips above, if any, are expected pre-existing/not-yet-landed cases — see each summary)"
fi
exit "$OVERALL_RC"
