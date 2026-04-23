"""Autonomy tests — 4-phase chain per velocity.

Per (sim, num_robots, iter, velocity): ready → takeoff → hover → land.
Drone returns to ground at end of each velocity so the next velocity
starts fresh. A local `pytest_collection_modifyitems` hook reorders the
autonomy tests so the full 4-phase chain runs per velocity before pytest
advances to the next velocity.
"""
import bisect
import math
import statistics
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from io import StringIO

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
MOTION_ABOVE_START_M = 0.3  # z threshold for "drone started moving" (relative to z[0])
SETTLING_WINDOW_S = 1.0     # seconds of trailing samples used for steady-state altitude
MAX_GT_MATCH_AGE_S = 0.1    # drop an odom sample if nearest GT is >100ms away

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

METRIC_UNITS = {
    "ready_duration_sys_s": "s",
    "takeoff_duration_sim_s": "s",
    "land_duration_sim_s": "s",
    "velocity_rmse_m_sim_s": "m/s",
    "attitude_stddev_rad": "rad",
    # Everything else: "m".
}


def _phase_timeout(velocity):
    """Takeoff/land timeout scaled so 0.5 m/s runs don't time out spuriously."""
    return max(30.0, TARGET_ALTITUDE_M / velocity + 15.0)


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


def _tracking_metrics_hover(odom, target):
    xs = [r["pose.pose.position.x"] for r in odom]
    ys = [r["pose.pose.position.y"] for r in odom]
    zs = [r["pose.pose.position.z"] for r in odom]
    # Total 3D positional jitter around the mean point. Equal to
    # sqrt(var(x) + var(y) + var(z)) — one axis-agnostic stability number.
    if len(odom) > 1:
        pos_stddev = math.sqrt(statistics.pvariance(xs)
                               + statistics.pvariance(ys)
                               + statistics.pvariance(zs))
    else:
        pos_stddev = 0.0
    return {
        "hover_altitude_mean_error_m": round(abs(statistics.mean(zs) - target), 3),
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

def _start_captures(robot_container, setup_bash, domain, duration_s, tag):
    """Start odom + ground-truth CSV streams for one robot. Returns a handle
    that `_finish_captures` later consumes to wait for completion and parse
    both CSVs. The handle carries `duration_s` so the caller-less `wait`
    timeout matches what the in-container streams were capped at."""
    odom_path = f"/tmp/auto_r{domain}_{tag}_odom.csv"
    gt_path = f"/tmp/auto_r{domain}_{tag}_gt.csv"
    odom_proc, odom_fh, odom_ef = _start_csv_stream(
        robot_container, f"/robot_{domain}/interface/mavros/local_position/odom",
        domain, setup_bash, duration_s, odom_path)
    gt_proc, gt_fh, gt_ef = _start_csv_stream(
        robot_container, f"/robot_{domain}/odom_ground_truth",
        domain, setup_bash, duration_s, gt_path)
    return {
        "duration_s": duration_s,
        "odom": (odom_proc, odom_fh, odom_ef, odom_path),
        "gt": (gt_proc, gt_fh, gt_ef, gt_path),
    }


def _finish_captures(streams):
    """Stop capture subprocesses and return parsed (odom, gt) samples.
    Callers invoke this right after the action completes, so we actively
    terminate the captures instead of waiting for their internal `timeout N`
    to elapse — otherwise fast takeoffs would block until the full capture
    window expires. gt will be empty if no ground-truth publisher exists."""
    (odom_proc, odom_fh, odom_ef, odom_path) = streams["odom"]
    (gt_proc, gt_fh, gt_ef, gt_path) = streams["gt"]
    try:
        for proc in (odom_proc, gt_proc):
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
    finally:
        odom_fh.close()
        gt_fh.close()
        odom_ef.close()
        gt_ef.close()
    odom = _parse_csv(odom_path, ODOM_SCHEMA)
    gt = _parse_csv(gt_path, ODOM_SCHEMA)
    if not odom:
        logger.warning("odom capture empty. stdout head=%r stderr head=%r",
                       open(odom_path).read(500),
                       open(odom_path + ".err").read(500))
    if not gt:
        logger.warning("ground truth not available — skipping state-estimation error metrics.")
    return odom, gt


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


def _takeoff_one_robot(n, robot_container, cfg, velocity):
    timeout = _phase_timeout(velocity)
    target = TARGET_ALTITUDE_M
    streams = _start_captures(robot_container, cfg["robot_setup_bash"],
                              n, timeout + 5, f"v{velocity}_takeoff")
    goal = f"{{target_altitude_m: {target}, velocity_m_s: {velocity}}}"
    result = ros2_exec(
        robot_container,
        f'ros2 action send_goal --feedback /robot_{n}/tasks/takeoff '
        f'task_msgs/action/TakeoffTask "{goal}"',
        domain_id=n, setup_bash=cfg["robot_setup_bash"],
        timeout=int(timeout + 10),
    )
    odom, gt = _finish_captures(streams)
    if not _action_ok(result.stdout):
        pytest.fail(f"robot_{n} takeoff failed: {_action_message(result.stdout)}")
    if not odom:
        pytest.fail(f"robot_{n} takeoff: no odom samples captured")
    metrics = _tracking_metrics_takeoff(odom, target, velocity)
    metrics.update(_gt_metrics(odom, gt))
    _record(n, metrics)
    err = metrics["altitude_error_m"]
    assert abs(err) <= target * 0.1, (
        f"robot_{n} settled altitude {target + err:.2f}m differs from "
        f"target {target:.1f}m by more than 10%")


def _hover_one_robot(n, robot_container, cfg, velocity):
    target = TARGET_ALTITUDE_M
    streams = _start_captures(robot_container, cfg["robot_setup_bash"],
                              n, HOVER_DURATION_S + 2, f"v{velocity}_hover")
    # Passive phase: no blocking action, so we sleep to let the capture
    # collect samples before _finish_captures terminates it.
    time.sleep(HOVER_DURATION_S)
    odom, gt = _finish_captures(streams)
    if not odom:
        pytest.fail(f"robot_{n} hover: no odom samples captured")
    metrics = _tracking_metrics_hover(odom, target)
    metrics.update(_gt_metrics(odom, gt))
    _record(n, metrics)
    mean_err = metrics["hover_altitude_mean_error_m"]
    assert mean_err < 0.5, (
        f"robot_{n} hover altitude mean error {mean_err:.2f}m exceeds ±0.5m tolerance")


def _landing_one_robot(n, robot_container, cfg, velocity):
    timeout = _phase_timeout(velocity)
    streams = _start_captures(robot_container, cfg["robot_setup_bash"],
                              n, timeout + 5, f"v{velocity}_land")
    goal = f"{{velocity_m_s: {velocity}}}"
    result = ros2_exec(
        robot_container,
        f'ros2 action send_goal --feedback /robot_{n}/tasks/land '
        f'task_msgs/action/LandTask "{goal}"',
        domain_id=n, setup_bash=cfg["robot_setup_bash"],
        timeout=int(timeout + 10),
    )
    odom, gt = _finish_captures(streams)
    if not _action_ok(result.stdout):
        pytest.fail(f"robot_{n} land failed: {_action_message(result.stdout)}")
    if not odom:
        pytest.fail(f"robot_{n} land: no odom samples captured")
    metrics = _tracking_metrics_landing(odom, velocity)
    metrics.update(_gt_metrics(odom, gt))
    _record(n, metrics)
    final = metrics["final_altitude_m"]
    assert final < 0.5, f"robot_{n} final altitude {final:.2f}m > 0.5m"


# ── tests ──────────────────────────────────────────────────────────────────

@pytest.mark.autonomy
@pytest.mark.timeout(1800)
class TestAutonomy:

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
        pending = list(range(1, num_robots + 1))
        deadline = started + PX4_READY_TIMEOUT_S

        while pending and time.time() < deadline:
            for n in list(pending):
                # --once exits 0 on the first message; the inner `timeout` makes
                # it exit nonzero if nothing is published within the window.
                result = ros2_exec(
                    robot_container,
                    f"timeout 5 ros2 topic echo --once "
                    f"/robot_{n}/interface/mavros/local_position/odom",
                    domain_id=n, setup_bash=cfg["robot_setup_bash"], timeout=10,
                )
                if result.returncode == 0:
                    ready_at[n] = round(time.time() - started, 2)
                    pending.remove(n)
            if pending:
                logger.info("waiting for local_position/odom; pending=%s elapsed=%.0fs",
                            pending, time.time() - started)
                time.sleep(PX4_POLL_INTERVAL_S)

        if pending:
            pytest.fail(f"robots {sorted(pending)} never published "
                        f"local_position/odom within {PX4_READY_TIMEOUT_S:.0f}s")

        for n, dur in ready_at.items():
            _record(n, {"ready_duration_sys_s": dur})
        _ready_envs.add(env_id)

    @pytest.mark.dependency(name="autonomy_takeoff", depends=["autonomy_ready"])
    def test_takeoff(self, airstack_env, velocity):
        """Send TakeoffTask per robot in parallel; verify peak altitude and record metrics."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]
        _run_parallel(num_robots,
                      lambda n: _takeoff_one_robot(n, robot_container, cfg, velocity))

    @pytest.mark.dependency(name="autonomy_hover", depends=["autonomy_takeoff"])
    def test_hover(self, airstack_env, velocity):
        """Observe odom for HOVER_DURATION_S seconds per robot in parallel; check stability."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]
        _run_parallel(num_robots,
                      lambda n: _hover_one_robot(n, robot_container, cfg, velocity))

    @pytest.mark.dependency(name="autonomy_landing", depends=["autonomy_hover"])
    def test_landing(self, airstack_env, velocity):
        """Send LandTask per robot in parallel; verify final altitude and record metrics."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]
        _run_parallel(num_robots,
                      lambda n: _landing_one_robot(n, robot_container, cfg, velocity))
