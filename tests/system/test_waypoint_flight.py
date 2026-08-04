"""Ordered-waypoint navigation flight tests.

Per (sim, num_robots, iter): ready → takeoff → navigate waypoint route → land.

After takeoff the test sends the route to the local planner's NavigateTask
action (``/robot_N/tasks/navigate``) as a ``nav_msgs/Path`` and captures
odometry throughout. Pass/fail is judged by the standalone
``tests/waypoint_checker.py``: the odometry track must pass within
``--waypoint-tolerance`` of every waypoint **in order**, each within
``--waypoint-timeout`` seconds of the previous arrival. Judging on the
odometry track (rather than the action result) keeps the success criterion
independent of any particular planner implementation — swap the global or
local planner and the same test still judges the flight.

Waypoints are given relative to the robot's pose at dispatch (x forward
along initial heading, z up from dispatch altitude) so routes are
spawn-point and sim agnostic.
"""

import time

import pytest

from conftest import (
    current_test_id,
    get_metrics,
    get_robot_containers,
    logger,
    ros2_exec,
)
from system.test_fixed_trajectory import (
    ODOM_SCHEMA,
    TARGET_ALTITUDE_M,
    _action_message,
    _action_ok,
    _finish_captures,
    _landing_one_robot,
    _quat_to_yaw,
    _run_parallel,
    _stamp,
    _start_captures,
    _takeoff_one_robot,
    _transform_to_world,
)
from waypoint_checker import check_track, parse_waypoints

PX4_READY_TIMEOUT_S = 300.0

METRIC_UNITS = {
    "waypoint_success": "",
    "waypoints_reached": "",
    "navigate_action_success": "",
    "route_time_sim_s": "s",
    # Everything else defaults to "m".
}


def _record(robot_n: int, metrics_dict: dict) -> None:
    """Record per-robot scalar metrics; unit inferred from METRIC_UNITS."""
    m = get_metrics()
    tid = current_test_id()
    higher = {"waypoint_success", "waypoints_reached", "navigate_action_success"}
    for key, value in metrics_dict.items():
        if value is None:
            continue
        unit = METRIC_UNITS.get(key, "m")
        direction = "higher_is_better" if key in higher else "lower_is_better"
        m.record(tid, f"robot_{robot_n}.{key}", value, unit=unit, direction=direction)


def _build_navigate_goal(world_pts: list[tuple[float, float, float]],
                         tolerance_m: float) -> str:
    """YAML goal for a NavigateTask send_goal call from world-frame waypoints."""
    poses = ", ".join(
        f"{{pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, "
        f"orientation: {{w: 1.0}}}}}}"
        for x, y, z in world_pts
    )
    return (f"{{global_plan: {{header: {{frame_id: ''}}, poses: [{poses}]}}, "
            f"goal_tolerance_m: {tolerance_m}}}")


def _snapshot_start_pose(robot_container: str, cfg: dict, n: int):
    """World-frame (x, y, z, yaw) of robot n right now, from one odom sample."""
    snap = ros2_exec(
        robot_container,
        f"timeout 5 ros2 topic echo --once --csv "
        f"/robot_{n}/interface/mavros/local_position/odom",
        domain_id=n, setup_bash=cfg["robot_setup_bash"], timeout=10,
    )
    for line in snap.stdout.splitlines():
        parts = line.strip().split(",")
        if len(parts) >= len(ODOM_SCHEMA):
            try:
                row = dict(zip(ODOM_SCHEMA, parts))
                return (
                    float(row["pose.pose.position.x"]),
                    float(row["pose.pose.position.y"]),
                    float(row["pose.pose.position.z"]),
                    _quat_to_yaw(
                        float(row["pose.pose.orientation.x"]),
                        float(row["pose.pose.orientation.y"]),
                        float(row["pose.pose.orientation.z"]),
                        float(row["pose.pose.orientation.w"]),
                    ),
                )
            except (ValueError, KeyError):
                pass
    return 0.0, 0.0, TARGET_ALTITUDE_M, 0.0


def _navigate_one_robot(n: int, robot_container: str, cfg: dict,
                        waypoints_rel: list[tuple[float, float, float]],
                        tolerance_m: float, budget_s: float) -> None:
    route_timeout = budget_s * len(waypoints_rel) + 30.0

    x0, y0, z0, yaw0 = _snapshot_start_pose(robot_container, cfg, n)
    world_pts = _transform_to_world(waypoints_rel, x0, y0, z0, yaw0)
    logger.info("robot_%d waypoint route (world frame): %s", n,
                [(round(x, 1), round(y, 1), round(z, 1)) for x, y, z in world_pts])

    streams = _start_captures(robot_container, cfg["robot_setup_bash"],
                              n, route_timeout + 10, "waypoints")
    goal = _build_navigate_goal(world_pts, tolerance_m)
    result = ros2_exec(
        robot_container,
        f'ros2 action send_goal --feedback /robot_{n}/tasks/navigate '
        f'task_msgs/action/NavigateTask "{goal}"',
        domain_id=n, setup_bash=cfg["robot_setup_bash"],
        timeout=int(route_timeout + 15),
    )
    odom = _finish_captures(streams)

    action_success = _action_ok(result.stdout)
    _record(n, {"navigate_action_success": 1.0 if action_success else 0.0})
    if not action_success:
        logger.warning("robot_%d navigate action did not succeed: %s",
                       n, _action_message(result.stdout))

    if not odom:
        pytest.fail(f"robot_{n} waypoint flight: no odom samples captured")

    rows = [(_stamp(r),
             r["pose.pose.position.x"],
             r["pose.pose.position.y"],
             r["pose.pose.position.z"]) for r in odom]
    verdict = check_track(rows, world_pts, tolerance_m, budget_s)

    reached = sum(1 for w in verdict["waypoints"] if w["reached"])
    metrics = {
        "waypoint_success": 1.0 if verdict["success"] else 0.0,
        "waypoints_reached": float(reached),
        "route_time_sim_s": verdict.get("total_time_s"),
        "worst_closest_approach_m": max(
            w["closest_approach_m"] for w in verdict["waypoints"]),
    }
    _record(n, metrics)
    for w in verdict["waypoints"]:
        logger.info(
            "robot_%d waypoint %d: reached=%s closest=%.2fm elapsed=%ss",
            n, w["index"], w["reached"], w["closest_approach_m"],
            w.get("elapsed_from_prev_s", "n/a"))

    assert verdict["success"], (
        f"robot_{n} reached {reached}/{len(world_pts)} waypoints in order "
        f"(tolerance {tolerance_m}m, budget {budget_s}s/waypoint); "
        f"closest approaches: "
        f"{[w['closest_approach_m'] for w in verdict['waypoints']]}"
    )


# ── test class ─────────────────────────────────────────────────────────────

@pytest.mark.waypoint_flight
@pytest.mark.timeout(2400)
class TestWaypointFlight:
    """Full takeoff → waypoint route → land chain, judged by waypoint_checker.

    Route, tolerance, and per-waypoint budget come from --waypoints,
    --waypoint-tolerance, and --waypoint-timeout.
    """

    @pytest.fixture(scope="session")
    def _failed_envs(self):
        return set()

    @pytest.fixture(autouse=True)
    def _chain_guard(self, request, airstack_env, _failed_envs):
        """Skip tests whose env was poisoned by an earlier failure.

        A waypoint-flight failure does NOT poison the env — landing always
        runs after a successful takeoff. Takeoff or landing failures do.
        """
        env_id = (airstack_env["sim"], airstack_env["num_robots"],
                  airstack_env["iteration"])
        if env_id in _failed_envs:
            pytest.skip(f"earlier waypoint-flight test failed in {env_id}")
        yield
        rep = getattr(request.node, "_rep_call", None)
        if rep is not None and rep.failed:
            if "test_waypoint_route" not in request.node.name:
                _failed_envs.add(env_id)

    @pytest.fixture
    def waypoints_rel(self, request):
        return parse_waypoints(request.config.getoption("--waypoints"))

    @pytest.fixture
    def tolerance_m(self, request):
        return float(request.config.getoption("--waypoint-tolerance"))

    @pytest.fixture
    def budget_s(self, request):
        return float(request.config.getoption("--waypoint-timeout"))

    @pytest.mark.dependency(name="wpf_ready")
    def test_px4_ready(self, airstack_env):
        """Wait until MAVROS is connected and local_position/odom publishes."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        num_robots = airstack_env["num_robots"]

        started = time.time()
        connected: set[int] = set()
        pending = list(range(1, num_robots + 1))
        deadline = started + PX4_READY_TIMEOUT_S

        while pending and time.time() < deadline:
            for n in list(pending):
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
                        continue
                r = ros2_exec(
                    robot_container,
                    f"timeout 5 ros2 topic echo --once "
                    f"/robot_{n}/interface/mavros/local_position/odom",
                    domain_id=n, setup_bash=cfg["robot_setup_bash"], timeout=10,
                )
                if r.returncode == 0 and "pose:" in r.stdout:
                    _record(n, {"ready_duration_sys_s":
                                round(time.time() - started, 2)})
                    pending.remove(n)
            if pending:
                logger.info("px4_ready: connected=%s pending=%s elapsed=%.0fs",
                            sorted(connected), pending, time.time() - started)
                time.sleep(2.0)

        if pending:
            pytest.fail(
                f"robots {sorted(pending)} not ready (MAVROS connected + odom) "
                f"within {PX4_READY_TIMEOUT_S:.0f}s"
            )

    @pytest.mark.dependency(name="wpf_takeoff", depends=["wpf_ready"])
    def test_takeoff(self, airstack_env):
        """Take off to TARGET_ALTITUDE_M at a fixed velocity of 1 m/s."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        _run_parallel(
            airstack_env["num_robots"],
            lambda n: _takeoff_one_robot(n, robot_container, cfg, TARGET_ALTITUDE_M),
        )

    @pytest.mark.dependency(name="wpf_route", depends=["wpf_takeoff"])
    def test_waypoint_route(self, airstack_env, waypoints_rel, tolerance_m, budget_s):
        """Send NavigateTask with the route; judge odometry with waypoint_checker."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        _run_parallel(
            airstack_env["num_robots"],
            lambda n: _navigate_one_robot(n, robot_container, cfg,
                                          waypoints_rel, tolerance_m, budget_s),
        )

    @pytest.mark.dependency(name="wpf_land", depends=["wpf_takeoff"])
    def test_landing(self, airstack_env):
        """Land the drone; runs even when test_waypoint_route fails."""
        cfg = airstack_env["cfg"]
        robot_container = get_robot_containers(airstack_env["robot_pattern"])[0]
        _run_parallel(
            airstack_env["num_robots"],
            lambda n: _landing_one_robot(n, robot_container, cfg),
        )
