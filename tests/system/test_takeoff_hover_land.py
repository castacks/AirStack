"""Takeoff-hover-land tests — 4-phase chain per velocity.

Per (sim, num_robots, iter, velocity): ready → takeoff → hover → land.
Drone returns to ground at end of each velocity so the next velocity
starts fresh. A local `pytest_collection_modifyitems` hook reorders the
autonomy tests so the full 4-phase chain runs per velocity before pytest
advances to the next velocity.
"""
import bisect
import math
import os
import statistics
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from io import StringIO
from pathlib import Path

import pandas as pd
import pytest

from conftest import (
    ROS_DISTRO_SETUP,
    current_test_id,
    get_metrics,
    get_robot_containers,
    logger,
    ros2_exec,
)

# ── configuration ──────────────────────────────────────────────────────────

TARGET_ALTITUDE_M = 10.0
HOVER_DURATION_S = 10.0
PX4_READY_TIMEOUT_S = 300.0
PX4_POLL_INTERVAL_S = 2.0

# Vision/NatNet EV fusion needs a few extra seconds to settle before PX4 will
# arm. Non-vision keeps the original behavior (no settle) to keep the base fast.
PX4_ARM_SETTLE_VISION_S = 15.0  # post-ready settle, vision profiles only
ARM_RETRY_ATTEMPTS = 3          # total takeoff attempts on arm rejection
ARM_RETRY_BACKOFF_S = 5.0       # wait between retries
MOTION_ABOVE_START_M = 0.3  # z threshold for "drone started moving" (relative to z[0])
SETTLING_WINDOW_S = 1.0     # seconds of trailing samples used for steady-state altitude
MAX_GT_MATCH_AGE_S = 0.1    # drop an odom sample if nearest GT is >100ms away

# Vision profiles only: bound the gap between the mocap pose PX4 fuses and PX4's
# own fused estimate, per axis, for the whole phase (per-axis MAX). z gets more
# slack than xy (baro fusion pulls PX4 z off the pure mocap z). Bounds are
# phase-aware: the tight steady-state bound applies during hover, where a close
# mocap↔PX4 agreement is meaningful; the dynamic bound applies during takeoff
# and landing, where fusion lag (EKF2_EV_DELAY) plus climb/attitude transients
# briefly inflate the per-axis MAX even though the mean stays ~1cm. Tunable —
# tighten as sim allows.
EV_POSE_TOPIC = os.environ.get("NATNET_POSE_TOPIC", "perception/optitrack/drone")
EV_POSE_ERROR_MAX_XY_M = 0.02          # steady-state (hover)
EV_POSE_ERROR_MAX_Z_M = 0.05
EV_POSE_ERROR_MAX_XY_DYNAMIC_M = 0.04  # dynamic (takeoff / landing)
EV_POSE_ERROR_MAX_Z_DYNAMIC_M = 0.08

# Parallel vision takeoffs slow the climb (mocap fusion under shared GPU/CPU
# load). Non-vision keeps the original fixed budget to guard base performance.
TAKEOFF_TIMEOUT_VISION_PER_ROBOT_FACTOR = 1.0  # extra budget per extra robot, vision only

# Full column schemas of `ros2 topic echo --csv` output, in declaration order.
# Covariance arrays expand to 36 comma-separated values each. Downstream code
# reads only the ~9 fields it cares about by name (e.g. "pose.pose.position.z")
# — other columns are parsed but unused.
ODOM_SCHEMA = (
    ["header.stamp.sec", "header.stamp.nanosec",
     "header.frame_id", "child_frame_id",
     "pose.pose.position.x", "pose.pose.position.y", "pose.pose.position.z",
     "pose.pose.orientation.x", "pose.pose.orientation.y",
     "pose.pose.orientation.z", "pose.pose.orientation.w"]
    + [f"pose.covariance[{i}]" for i in range(36)]
    + ["twist.twist.linear.x", "twist.twist.linear.y", "twist.twist.linear.z",
       "twist.twist.angular.x", "twist.twist.angular.y", "twist.twist.angular.z"]
    + [f"twist.covariance[{i}]" for i in range(36)]
)

# Flattened CSV columns of the NatNet PoseStamped (mocap EV input to PX4).
POSE_SCHEMA = [
    "header.stamp.sec", "header.stamp.nanosec", "header.frame_id",
    "pose.position.x", "pose.position.y", "pose.position.z",
    "pose.orientation.x", "pose.orientation.y",
    "pose.orientation.z", "pose.orientation.w",
]

METRIC_UNITS = {
    "ready_duration_sys_s": "s",
    "takeoff_duration_sim_s": "s",
    "land_duration_sim_s": "s",
    "velocity_rmse_m_sim_s": "m/s",
    "attitude_stddev_rad": "rad",
    # Everything else: "m".
}


def _is_vision_profile(cfg):
    """True for external-vision / NatNet mocap stacks."""
    env = cfg.get("extra_env", {}) if cfg else {}
    if str(env.get("LAUNCH_NATNET", "")).lower() in ("1", "true", "yes", "on"):
        return True
    return "vision" in str(env.get("SITL_PARAM_PROFILE", "")).lower()


def _phase_timeout(velocity, num_robots=1, vision=False):
    """Takeoff/land wall-clock budget. Original fixed budget for non-vision;
    vision profiles widen it per extra robot to absorb the slower climb."""
    base = max(30.0, TARGET_ALTITUDE_M / velocity + 15.0)
    if not vision:
        return base
    scale = 1.0 + TAKEOFF_TIMEOUT_VISION_PER_ROBOT_FACTOR * max(0, num_robots - 1)
    return base * scale


# ── pytest hooks ───────────────────────────────────────────────────────────

def pytest_generate_tests(metafunc):
    """Parametrize tests that request `velocity` from --takeoff-velocities.

    Phase-order reordering (so the 4-test chain runs per-velocity, not
    parametrize-first) is done by `pytest_collection_modifyitems` in
    conftest.py — that hook isn't discovered from test modules.
    """
    if "velocity" in metafunc.fixturenames:
        raw = metafunc.config.getoption("--takeoff-velocities")
        vels = [float(v) for v in raw.split(",") if v.strip()]
        metafunc.parametrize("velocity", vels, ids=[f"v{v}" for v in vels])


# ── subprocess / CSV helpers ───────────────────────────────────────────────

def _start_csv_stream(container, topic, domain, setup_bash,
                      duration_s, out_path):
    """Background `ros2 topic echo --csv` streaming to out_path.

    Each message prints as a single CSV line with all primitives flattened in
    declaration order. Callers pick the exact numeric columns they want via
    pandas `usecols` (see `_parse_csv`). `--no-arr`/`--no-str` are deliberately
    NOT used: they replace fields with placeholder strings (e.g. `<string
    length: <0>>`) instead of dropping them, which would break index mapping.

    Returns (popen, file_handle, err_file_handle). Caller must close both
    file handles after the process terminates (see `_finish_captures`).
    """
    cmd = (
        f"source {ROS_DISTRO_SETUP} && source {setup_bash} && "
        f"export ROS_DOMAIN_ID={domain} && "
        f"timeout {int(duration_s)} ros2 topic echo --csv {topic}"
    )
    f = open(out_path, "w")
    ef = open(out_path + ".err", "w")
    try:
        proc = subprocess.Popen(
            ["docker", "exec", container, "bash", "-c", cmd],
            stdout=f, stderr=ef,
        )
    except BaseException:
        f.close()
        ef.close()
        raise
    return proc, f, ef


def _parse_csv(path, schema):
    """Read ros2 `--csv` output. `schema` names every column in the flattened
    CSV in declaration order. Non-CSV lines (stray `WARNING:` prints ros2 emits
    to stdout) are filtered before pandas parses."""
    with open(path) as f:
        good = [line for line in f if line.count(",") >= len(schema) - 1]
    if not good:
        return []
    df = pd.read_csv(StringIO("".join(good)), header=None, names=schema)
    return df.to_dict("records")


def _stamp(row, prefix="header.stamp"):
    """Sim-time seconds from a parsed row."""
    return row[f"{prefix}.sec"] + row[f"{prefix}.nanosec"] * 1e-9


# ── action result parsing ──────────────────────────────────────────────────

def _action_ok(stdout):
    """True when ros2 action send_goal --feedback reports success: true (YAML bool)."""
    return "success: true" in stdout


def _action_message(stdout):
    for line in stdout.splitlines():
        s = line.strip()
        if s.startswith("message:"):
            return s[len("message:"):].strip().strip("'\"")
    return "\n".join(stdout.strip().splitlines()[-5:])


# ── metric computation ────────────────────────────────────────────────────

def _valid_range(start, end):
    """True iff both indices are set and end follows start."""
    return start is not None and end is not None and end > start


def _velocity_rmse(ts, zs, i0, i1, v_cmd):
    """RMSE of dz/dt vs commanded velocity across the [i0, i1] sample range."""
    sq_errs = []
    for i in range(i0 + 1, i1 + 1):
        dt = ts[i] - ts[i - 1]
        if dt > 1e-6:
            sq_errs.append(((zs[i] - zs[i - 1]) / dt - v_cmd) ** 2)
    if not sq_errs:
        return None
    return math.sqrt(sum(sq_errs) / len(sq_errs))


def _tracking_metrics_takeoff(odom, target, velocity):
    zs = [r["pose.pose.position.z"] for r in odom]
    ts = [_stamp(r) for r in odom]
    peak = max(zs)
    # Steady-state altitude at the moment of success: mean of samples within
    # the trailing SETTLING_WINDOW_S. Captures where the drone actually parked,
    # vs `peak` which captures transient overshoot.
    cutoff = ts[-1] - SETTLING_WINDOW_S
    settled = [z for z, t in zip(zs, ts) if t >= cutoff]
    out = {
        # Signed: positive = settled above target, negative = below target.
        "altitude_error_m": round(statistics.mean(settled) - target, 3),
        # Unsigned transient overshoot: 0 if drone never went above target.
        "overshoot_m": round(max(0.0, peak - target), 3),
    }
    # Motion threshold is relative to starting altitude so drones that spawn
    # slightly above ground (landing gear, URDF origin offset) don't register
    # the first sample as "already moving".
    z0 = zs[0]
    first_motion = next((i for i, z in enumerate(zs)
                         if z > z0 + MOTION_ABOVE_START_M), None)
    first_at_target = next((i for i, z in enumerate(zs) if z >= target * 0.95), None)
    if _valid_range(first_motion, first_at_target):
        out["takeoff_duration_sim_s"] = round(ts[first_at_target] - ts[first_motion], 3)
        rmse = _velocity_rmse(ts, zs, first_motion, first_at_target, velocity)
        if rmse is not None:
            out["velocity_rmse_m_sim_s"] = round(rmse, 3)
    return out


def _tracking_metrics_hover(odom):
    """Measure whether the drone stayed put relative to where takeoff left it.

    Reference altitude is the mean over the first SETTLING_WINDOW_S of hover
    (not the takeoff target), so takeoff inaccuracy doesn't leak into hover.
    Hover tests "drone holds position", not "drone is at target".
    """
    xs = [r["pose.pose.position.x"] for r in odom]
    ys = [r["pose.pose.position.y"] for r in odom]
    zs = [r["pose.pose.position.z"] for r in odom]
    ts = [_stamp(r) for r in odom]

    ref_cutoff = ts[0] + SETTLING_WINDOW_S
    ref_z = statistics.mean(z for z, t in zip(zs, ts) if t <= ref_cutoff)

    # Total 3D positional jitter around the mean point. Equal to
    # sqrt(var(x) + var(y) + var(z)) — one axis-agnostic stability number.
    pos_stddev = math.sqrt(statistics.pvariance(xs)
                           + statistics.pvariance(ys)
                           + statistics.pvariance(zs)) if len(odom) > 1 else 0.0
    return {
        # Drift from starting altitude over the full hover window.
        "hover_altitude_mean_error_m": round(abs(statistics.mean(zs) - ref_z), 3),
        "hover_position_stddev_m": round(pos_stddev, 3),
    }


def _tracking_metrics_landing(odom, velocity):
    zs = [r["pose.pose.position.z"] for r in odom]
    ts = [_stamp(r) for r in odom]
    out = {"final_altitude_m": round(zs[-1], 3)}
    peak = max(zs)
    first_descent = next((i for i, z in enumerate(zs) if z < peak * 0.8), None)
    first_at_ground = next((i for i, z in enumerate(zs) if z < 0.5), None)
    if _valid_range(first_descent, first_at_ground):
        out["land_duration_sim_s"] = round(ts[first_at_ground] - ts[first_descent], 3)
        rmse = _velocity_rmse(ts, zs, first_descent, first_at_ground, -velocity)
        if rmse is not None:
            out["velocity_rmse_m_sim_s"] = round(rmse, 3)
    return out


def _gt_metrics(odom, gt):
    """Odom vs ground-truth state-estimation error. Empty dict when GT missing."""
    if not gt:
        return {}
    gt_sorted = sorted(gt, key=_stamp)
    gt_stamps = [_stamp(r) for r in gt_sorted]
    errs, z_biases = [], []
    for row in odom:
        t = _stamp(row)
        i = bisect.bisect_left(gt_stamps, t)
        candidates = []
        if i > 0:
            candidates.append(gt_sorted[i - 1])
        if i < len(gt_sorted):
            candidates.append(gt_sorted[i])
        if not candidates:
            continue
        best = min(candidates, key=lambda r: abs(_stamp(r) - t))
        if abs(_stamp(best) - t) > MAX_GT_MATCH_AGE_S:
            continue  # stale GT — pairing would conflate motion with bias
        ox, oy, oz = (row["pose.pose.position.x"],
                      row["pose.pose.position.y"],
                      row["pose.pose.position.z"])
        gx, gy, gz = (best["pose.pose.position.x"],
                      best["pose.pose.position.y"],
                      best["pose.pose.position.z"])
        errs.append(math.sqrt((ox - gx) ** 2 + (oy - gy) ** 2 + (oz - gz) ** 2))
        z_biases.append(oz - gz)
    if not errs:
        return {}
    return {
        "odometry_error_mean_m": round(statistics.mean(errs), 3),
        "odometry_error_max_m": round(max(errs), 3),
        "odometry_altitude_bias_m": round(statistics.mean(z_biases), 3),
    }


def _ev_pose_metrics(odom, ev):
    """Per-axis |mocap − PX4| over the phase. Empty dict when EV pose missing.

    Pairs the mocap PoseStamped (PX4's EV input) with PX4's fused
    local_position/odom by nearest sim-time stamp. Both are world-ENU in sim, so
    components compare directly — this bounds how far PX4's estimate drifts from
    the mocap it fuses (plus baro pull on z)."""
    if not ev:
        return {}
    ev_sorted = sorted(ev, key=_stamp)
    ev_stamps = [_stamp(r) for r in ev_sorted]
    ex, ey, ez = [], [], []
    for row in odom:
        t = _stamp(row)
        i = bisect.bisect_left(ev_stamps, t)
        candidates = []
        if i > 0:
            candidates.append(ev_sorted[i - 1])
        if i < len(ev_sorted):
            candidates.append(ev_sorted[i])
        if not candidates:
            continue
        best = min(candidates, key=lambda r: abs(_stamp(r) - t))
        if abs(_stamp(best) - t) > MAX_GT_MATCH_AGE_S:
            continue  # stale mocap — pairing would conflate motion with error
        ex.append(abs(row["pose.pose.position.x"] - best["pose.position.x"]))
        ey.append(abs(row["pose.pose.position.y"] - best["pose.position.y"]))
        ez.append(abs(row["pose.pose.position.z"] - best["pose.position.z"]))
    if not ex:
        return {}
    return {
        "ev_pose_error_max_x_m": round(max(ex), 3),
        "ev_pose_error_max_y_m": round(max(ey), 3),
        "ev_pose_error_max_z_m": round(max(ez), 3),
        "ev_pose_error_mean_x_m": round(statistics.mean(ex), 3),
        "ev_pose_error_mean_y_m": round(statistics.mean(ey), 3),
        "ev_pose_error_mean_z_m": round(statistics.mean(ez), 3),
    }


def _check_ev_pose_bounds(n, odom, ev, phase):
    """Record mocap↔PX4 pose error and fail if any axis exceeded its bound.

    Bounds are phase-aware: hover (steady state) uses the tight bound; takeoff
    and landing (dynamic) use a looser bound because fusion lag + climb
    transients inflate the per-axis MAX while the vehicle moves.
    No-op when EV pose is absent (non-vision profiles don't publish mocap)."""
    metrics = _ev_pose_metrics(odom, ev)
    if not metrics:
        return
    _record(n, metrics)
    if phase == "hover":
        xy_limit, z_limit = EV_POSE_ERROR_MAX_XY_M, EV_POSE_ERROR_MAX_Z_M
    else:
        xy_limit, z_limit = EV_POSE_ERROR_MAX_XY_DYNAMIC_M, EV_POSE_ERROR_MAX_Z_DYNAMIC_M
    over = []
    for axis, limit in (("x", xy_limit), ("y", xy_limit), ("z", z_limit)):
        v = metrics[f"ev_pose_error_max_{axis}_m"]
        if v > limit:
            over.append(f"{axis}={v:.3f}m (>{limit:.3f}m)")
    assert not over, (
        f"robot_{n} {phase}: mocap↔PX4 pose error exceeded bounds: "
        + ", ".join(over))


def _record(robot_n, metrics_dict):
    """Record per-robot scalar metrics; unit inferred from the key suffix."""
    m = get_metrics()
    tid = current_test_id()
    for key, value in metrics_dict.items():
        if value is None:
            continue
        unit = METRIC_UNITS.get(key, "m")
        m.record(tid, f"robot_{robot_n}.{key}", value,
                 unit=unit, direction="lower_is_better")


# ── capture bundle helper ──────────────────────────────────────────────────

def _start_captures(robot_container, setup_bash, domain, duration_s, tag,
                    capture_ev=False):
    """Start odom + ground-truth (and optionally mocap EV pose) CSV streams for
    one robot. Returns a handle that `_finish_captures` consumes to wait for
    completion and parse the CSVs. `capture_ev` adds the NatNet PoseStamped
    stream (vision profiles only). The handle carries `duration_s` so the
    caller-less `wait` matches what the in-container streams were capped at."""
    specs = [
        ("odom", f"/robot_{domain}/interface/mavros/local_position/odom"),
        ("gt", f"/robot_{domain}/odom_ground_truth"),
    ]
    if capture_ev:
        specs.append(("ev", f"/robot_{domain}/{EV_POSE_TOPIC}"))
    handle = {"duration_s": duration_s}
    started = []
    try:
        for key, topic in specs:
            path = f"/tmp/auto_r{domain}_{tag}_{key}.csv"
            proc, fh, ef = _start_csv_stream(
                robot_container, topic, domain, setup_bash, duration_s, path)
            handle[key] = (proc, fh, ef, path)
            started.append(handle[key])
    except BaseException:
        for proc, fh, ef, _ in started:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            fh.close()
            ef.close()
        raise
    return handle


def _finish_captures(streams):
    """Stop capture subprocesses and return parsed (odom, gt, ev) samples.
    Callers invoke this right after the action completes, so we actively
    terminate the captures instead of waiting for their internal `timeout N`
    to elapse — otherwise fast takeoffs would block until the full capture
    window expires. gt/ev are empty if their publisher isn't present."""
    keys = [k for k in ("odom", "gt", "ev") if k in streams]
    try:
        for k in keys:
            proc = streams[k][0]
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
    finally:
        for k in keys:
            streams[k][1].close()
            streams[k][2].close()
    odom = _parse_csv(streams["odom"][3], ODOM_SCHEMA)
    gt = _parse_csv(streams["gt"][3], ODOM_SCHEMA)
    ev = _parse_csv(streams["ev"][3], POSE_SCHEMA) if "ev" in streams else []
    if not odom:
        odom_path = streams["odom"][3]
        logger.warning("odom capture empty. stdout head=%r stderr head=%r",
                       Path(odom_path).read_text()[:500],
                       Path(odom_path + ".err").read_text()[:500])
    if not gt:
        logger.warning("ground truth not available — skipping state-estimation error metrics.")
    return odom, gt, ev


# ── per-robot workers (run in parallel for multi-robot) ───────────────────

def _run_parallel(num_robots, fn):
    """Run `fn(n)` for n=1..num_robots concurrently. If any worker raises, the
    exception surfaces after all workers finish (so partial multi-robot
    failures still show all results). Single-robot runs skip the executor."""
    if num_robots == 1:
        fn(1)
        return
    with ThreadPoolExecutor(max_workers=num_robots) as ex:
        list(ex.map(fn, range(1, num_robots + 1)))


def _takeoff_one_robot(n, robot_container, cfg, velocity, num_robots=1):
    vision = _is_vision_profile(cfg)
    timeout = _phase_timeout(velocity, num_robots, vision)
    target = TARGET_ALTITUDE_M
    goal = f"{{target_altitude_m: {target}, velocity_m_s: {velocity}}}"

    # Retry transient arm rejections. A slow climb raises TimeoutExpired (not an
    # arm rejection) and propagates without retrying.
    odom, gt, ev, result = [], [], [], None
    for attempt in range(1, ARM_RETRY_ATTEMPTS + 1):
        streams = _start_captures(robot_container, cfg["robot_setup_bash"],
                                  n, timeout + 5, f"v{velocity}_takeoff",
                                  capture_ev=vision)
        try:
            result = ros2_exec(
                robot_container,
                f'ros2 action send_goal --feedback /robot_{n}/tasks/takeoff '
                f'task_msgs/action/TakeoffTask "{goal}"',
                domain_id=n, setup_bash=cfg["robot_setup_bash"],
                timeout=int(timeout + 10),
            )
        finally:
            odom, gt, ev = _finish_captures(streams)
        if _action_ok(result.stdout):
            break
        msg = _action_message(result.stdout)
        if "arm" in msg.lower() and attempt < ARM_RETRY_ATTEMPTS:
            logger.warning(
                "robot_%d takeoff attempt %d/%d rejected (%s); retrying in %.0fs",
                n, attempt, ARM_RETRY_ATTEMPTS, msg, ARM_RETRY_BACKOFF_S)
            time.sleep(ARM_RETRY_BACKOFF_S)
            continue
        pytest.fail(f"robot_{n} takeoff failed: {msg}")
    if not odom:
        pytest.fail(f"robot_{n} takeoff: no odom samples captured")
    metrics = _tracking_metrics_takeoff(odom, target, velocity)
    metrics.update(_gt_metrics(odom, gt))
    _record(n, metrics)
    err = metrics["altitude_error_m"]
    assert abs(err) <= target * 0.1, (
        f"robot_{n} settled altitude {target + err:.2f}m differs from "
        f"target {target:.1f}m by more than 10%")
    _check_ev_pose_bounds(n, odom, ev, "takeoff")


def _hover_one_robot(n, robot_container, cfg, velocity):
    vision = _is_vision_profile(cfg)
    streams = _start_captures(robot_container, cfg["robot_setup_bash"],
                              n, HOVER_DURATION_S + 2, f"v{velocity}_hover",
                              capture_ev=vision)
    # Passive phase: no blocking action, so we sleep to let the capture
    # collect samples before _finish_captures terminates it.
    time.sleep(HOVER_DURATION_S)
    odom, gt, ev = _finish_captures(streams)
    if not odom:
        pytest.fail(f"robot_{n} hover: no odom samples captured")
    metrics = _tracking_metrics_hover(odom)
    metrics.update(_gt_metrics(odom, gt))
    _record(n, metrics)
    drift = metrics["hover_altitude_mean_error_m"]
    assert drift < 0.5, (
        f"robot_{n} drifted {drift:.2f}m in altitude during hover (>0.5m tolerance)")
    _check_ev_pose_bounds(n, odom, ev, "hover")


def _landing_one_robot(n, robot_container, cfg, velocity, num_robots=1):
    vision = _is_vision_profile(cfg)
    timeout = _phase_timeout(velocity, num_robots, vision)
    streams = _start_captures(robot_container, cfg["robot_setup_bash"],
                              n, timeout + 5, f"v{velocity}_land",
                              capture_ev=vision)
    goal = f"{{velocity_m_s: {velocity}}}"
    result = ros2_exec(
        robot_container,
        f'ros2 action send_goal --feedback /robot_{n}/tasks/land '
        f'task_msgs/action/LandTask "{goal}"',
        domain_id=n, setup_bash=cfg["robot_setup_bash"],
        timeout=int(timeout + 10),
    )
    odom, gt, ev = _finish_captures(streams)
    if not _action_ok(result.stdout):
        pytest.fail(f"robot_{n} land failed: {_action_message(result.stdout)}")
    if not odom:
        pytest.fail(f"robot_{n} land: no odom samples captured")
    metrics = _tracking_metrics_landing(odom, velocity)
    metrics.update(_gt_metrics(odom, gt))
    _record(n, metrics)
    final = metrics["final_altitude_m"]
    assert final < 0.5, f"robot_{n} final altitude {final:.2f}m > 0.5m"
    _check_ev_pose_bounds(n, odom, ev, "landing")


# ── tests ──────────────────────────────────────────────────────────────────

@pytest.mark.takeoff_hover_land
@pytest.mark.timeout(1800)
class TestTakeoffHoverLand:

    @pytest.fixture(scope="session")
    def _failed_envs(self):
        return set()

    @pytest.fixture(scope="session")
    def _ready_envs(self):
        return set()

    @pytest.fixture(autouse=True)
    def _chain_guard(self, request, airstack_env, _failed_envs):
        env_id = (airstack_env["sim"], airstack_env["num_robots"],
                  airstack_env["iteration"])
        if env_id in _failed_envs:
            pytest.skip(f"earlier autonomy test failed in {env_id}")
        yield
        rep = getattr(request.node, "_rep_call", None)
        if rep is not None and rep.failed:
            # Hover failures don't poison the chain — we still want landing
            # to run so the drone comes back to the ground, and the next
            # velocity gets its chance.
            if "test_hover" not in request.node.name:
                _failed_envs.add(env_id)

    @pytest.mark.dependency(name="autonomy_ready")
    def test_px4_ready(self, airstack_env, velocity, _ready_envs):
        """Wait until /robot_N/interface/mavros/local_position/odom is publishing.

        That topic goes live only after PX4's EKF converges and sets a home
        position — the exact precondition PX4's arming preflight requires and
        the topic the test later captures during takeoff. `connected=True` on
        mavros/state fires ~25s earlier and is insufficient (takeoff action
        returns `failed to arm` in that window).

        Skipped on velocities after the first in the same airstack_env.
        """
        env_id = (airstack_env["sim"], airstack_env["num_robots"],
                  airstack_env["iteration"])
        if env_id in _ready_envs:
            logger.info("px4_ready already confirmed for %s; skipping", env_id)
            return

        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]

        started = time.time()
        ready_at = {}
        # Per-robot progress through the two sequential gates.
        connected = set()   # robots that have reported mavros/state.connected=True
        pending = list(range(1, num_robots + 1))
        deadline = started + PX4_READY_TIMEOUT_S

        while pending and time.time() < deadline:
            for n in list(pending):
                # Gate 1: MAVROS ↔ PX4 heartbeat. Fast, reliable signal that
                # the stack is alive.
                if n not in connected:
                    r = ros2_exec(
                        robot_container,
                        f"timeout 5 ros2 topic echo --once --csv "
                        f"--field connected /robot_{n}/interface/mavros/state",
                        domain_id=n, setup_bash=cfg["robot_setup_bash"], timeout=10,
                    )
                    if any(line.strip() == "True" for line in r.stdout.splitlines()):
                        connected.add(n)
                    else:
                        continue  # try again next poll

                # Gate 2: local_position/odom actually publishing (EKF has a
                # valid local origin). Catches the case where connected=True
                # fires long before PX4 is ready for arming.
                r = ros2_exec(
                    robot_container,
                    f"timeout 5 ros2 topic echo --once "
                    f"/robot_{n}/interface/mavros/local_position/odom",
                    domain_id=n, setup_bash=cfg["robot_setup_bash"], timeout=10,
                )
                if r.returncode == 0 and "pose:" in r.stdout:
                    ready_at[n] = round(time.time() - started, 2)
                    pending.remove(n)

            if pending:
                logger.info("px4_ready: connected=%s pending=%s elapsed=%.0fs",
                            sorted(connected), pending, time.time() - started)
                time.sleep(PX4_POLL_INTERVAL_S)

        if pending:
            not_connected = [n for n in pending if n not in connected]
            if not_connected:
                pytest.fail(f"robots {sorted(not_connected)} never reported "
                            f"MAVROS connected=True within "
                            f"{PX4_READY_TIMEOUT_S:.0f}s")
            pytest.fail(f"robots {sorted(pending)} connected but never "
                        f"published local_position/odom within "
                        f"{PX4_READY_TIMEOUT_S:.0f}s")

        for n, dur in ready_at.items():
            _record(n, {"ready_duration_sys_s": dur})

        # Vision profiles need extra time to settle before arming; non-vision arms
        # much faster (original behavior).
        if _is_vision_profile(cfg):
            logger.info("px4_ready: vision profile; settling %.0fs before arming",
                        PX4_ARM_SETTLE_VISION_S)
            time.sleep(PX4_ARM_SETTLE_VISION_S)
        _ready_envs.add(env_id)

    @pytest.mark.dependency(name="autonomy_takeoff", depends=["autonomy_ready"])
    def test_takeoff(self, airstack_env, velocity):
        """Send TakeoffTask per robot in parallel; verify peak altitude and record metrics."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]
        _run_parallel(num_robots,
                      lambda n: _takeoff_one_robot(n, robot_container, cfg, velocity, num_robots))

    @pytest.mark.dependency(name="autonomy_hover", depends=["autonomy_takeoff"])
    def test_hover(self, airstack_env, velocity):
        """Observe odom for HOVER_DURATION_S seconds per robot in parallel; check stability."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]
        _run_parallel(num_robots,
                      lambda n: _hover_one_robot(n, robot_container, cfg, velocity))

    @pytest.mark.dependency(name="autonomy_landing", depends=["autonomy_takeoff"])
    def test_landing(self, airstack_env, velocity):
        """Send LandTask per robot in parallel; verify final altitude and record metrics."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]
        _run_parallel(num_robots,
                      lambda n: _landing_one_robot(n, robot_container, cfg, velocity, num_robots))
