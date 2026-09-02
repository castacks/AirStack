"""NativeSetpointStreamer driven end to end against stubbed ROS modules.

The pure tests pin the RULES; this one pins the WIRING — that the node
subscribes and publishes where the launch file says it does, that one tick
carries out a TickAction's instructions in the right order, and that the whole
engage/stream/handback cycle actually reaches PX4 as PositionTargets. Those are
the mistakes a state-machine test cannot see.

SKIPS where a real ROS is installed: in the container the real thing is a live
stack, not this.
"""
import pytest

import ros_stubs

pytestmark = pytest.mark.skipif(
    ros_stubs.ros_is_real(),
    reason='real ROS present — a stub graph proves nothing there')

PID_SVC = 'pid_set_parameters'
MODE_SVC = 'set_trajectory_mode'
SETPOINT_TOPIC = 'setpoint_raw_local'


def _build(monkeypatch, grace, follow=True):
    rec = ros_stubs.install()
    monkeypatch.setenv('MIGHTY_NATIVE_SETPOINTS', '1')
    monkeypatch.setenv('MIGHTY_NATIVE_GRACE_S', str(grace))
    monkeypatch.setenv('MIGHTY_NATIVE_GRACE_FOLLOW', '1' if follow else '0')
    import importlib

    import mighty_bridge.native_setpoint_node as nsn
    importlib.reload(nsn)
    n = nsn.NativeSetpointStreamer()
    rec.ready_services.update({PID_SVC, MODE_SVC})
    return n, rec


@pytest.fixture()
def node(monkeypatch):
    """Grace OFF, so the handoff sequences below read as one timeline. The
    grace window has its own fixture and its own tests."""
    yield _build(monkeypatch, 0.0)


@pytest.fixture()
def node_grace(monkeypatch):
    """Production defaults: a disengage is provisional for 4 s and live
    Goals are FOLLOWED through the grace (MIGHTY_NATIVE_GRACE_FOLLOW=1)."""
    yield _build(monkeypatch, 4.0)


@pytest.fixture()
def node_grace_hold(monkeypatch):
    """Grace in the conservative HOLD flavour (follow off)."""
    yield _build(monkeypatch, 4.0, follow=False)


def tick(n, rec, t):
    ros_stubs.set_clock(t)
    (_period, cb), = [x for x in rec.timers]
    cb()


def feed_goal(rec, t, **kw):
    ros_stubs.set_clock(t)
    rec.subscriptions['goal'](ros_stubs.Goal(**kw))


def feed_engaged(rec, t, value):
    ros_stubs.set_clock(t)
    rec.subscriptions['native_stream_active'](ros_stubs.Bool(value))


def feed_odom(rec, xyz=(1.0, 2.0, 3.0)):
    rec.subscriptions['odometry'](ros_stubs.Odometry(xyz=xyz))


def resolve_last_call(rec, name, successful=True):
    call = rec.calls_to(name)[-1]
    call.future.finish(ros_stubs.set_parameters_response(successful))
    return call


def mute_value(call):
    param, = call.request.parameters
    assert param.name == 'command_muted'
    return param.value.bool_value


# ── wiring ──────────────────────────────────────────────────────────────────

def test_subscribes_and_publishes_on_the_names_the_launch_file_remaps(node):
    n, rec = node
    assert set(rec.subscriptions) == {
        'goal', 'native_stream_active', 'odometry', 'pid_command'}
    assert n.setpoint_pub.topic == SETPOINT_TOPIC
    assert len(rec.timers) == 1


def test_enabled_defaults_from_the_env_var(monkeypatch):
    rec = ros_stubs.install()
    monkeypatch.delenv('MIGHTY_NATIVE_SETPOINTS', raising=False)
    import importlib

    import mighty_bridge.native_setpoint_node as nsn
    importlib.reload(nsn)
    n = nsn.NativeSetpointStreamer()
    assert n.enabled is False
    rec.ready_services.update({PID_SVC, MODE_SVC})
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0)
    for i in range(1, 20):
        tick(n, rec, i * 0.02)
    assert rec.published.get(SETPOINT_TOPIC) is None
    assert rec.calls == [], 'touched the pid with the switch off'


# ── the cycle ───────────────────────────────────────────────────────────────

def test_engage_stream_handback(node):
    n, rec = node
    feed_odom(rec, (10.0, 20.0, 30.0))
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0, p=(1.0, 2.0, 3.0), v=(0.0, 4.0, 0.0),
              a=(0.5, 0.6, 0.7), yaw=9.9, dyaw=0.3)

    # 1. mute requested, NOTHING streamed yet
    tick(n, rec, 0.02)
    assert rec.published.get(SETPOINT_TOPIC) is None
    call = rec.calls_to(PID_SVC)[-1]
    assert mute_value(call) is True

    # still nothing while the call is in flight
    tick(n, rec, 0.04)
    assert rec.published.get(SETPOINT_TOPIC) is None

    # 2. mute confirmed -> PositionTargets, with MIGHTY's numbers verbatim
    call.future.finish(ros_stubs.set_parameters_response(True))
    tick(n, rec, 0.06)
    msg = rec.last(SETPOINT_TOPIC)
    assert msg is not None
    assert msg.coordinate_frame == ros_stubs.PositionTarget.FRAME_LOCAL_NED
    assert msg.type_mask == ros_stubs.PositionTarget.IGNORE_YAW_RATE
    assert (msg.position.x, msg.position.y, msg.position.z) == (1.0, 2.0, 3.0)
    assert (msg.velocity.x, msg.velocity.y, msg.velocity.z) == (0.0, 4.0, 0.0)
    assert (msg.acceleration_or_force.x, msg.acceleration_or_force.y,
            msg.acceleration_or_force.z) == (0.5, 0.6, 0.7)
    assert msg.yaw == pytest.approx(1.5707963, abs=1e-6)   # +y travel, not 9.9
    assert msg.yaw_rate == 0.0
    assert msg.header.frame_id == 'map'

    # 3. disengage: hold at the VEHICLE, tracking point pinned, not yet unmuted
    rec.clear_published()
    feed_engaged(rec, 0.1, False)
    tick(n, rec, 0.12)
    hold = rec.last(SETPOINT_TOPIC)
    assert (hold.position.x, hold.position.y, hold.position.z) == (10., 20., 30.)
    assert (hold.velocity.x, hold.velocity.y, hold.velocity.z) == (0., 0., 0.)
    mode_call = rec.calls_to(MODE_SVC)[-1]
    assert mode_call.request.mode == ros_stubs.TrajectoryMode.Request.ROBOT_POSE
    # every pid call so far is a mute or a deadman refresh — none is an unmute
    assert all(mute_value(c) is True for c in rec.calls_to(PID_SVC)), \
        'un-muted before pinning the tracking point'

    # ... and still not un-muted while that request is unanswered
    tick(n, rec, 0.14)
    assert all(mute_value(c) is True for c in rec.calls_to(PID_SVC))

    # 4. controller acknowledges ROBOT_POSE -> un-mute, still streaming
    mode_call.future.finish(None)
    tick(n, rec, 0.16)
    assert mute_value(rec.calls_to(PID_SVC)[-1]) is False
    assert len(rec.published[SETPOINT_TOPIC]) >= 2

    # 5. pid seen commanding again -> stream stops
    resolve_last_call(rec, PID_SVC)
    for _ in range(3):
        rec.subscriptions['pid_command'](ros_stubs.RollPitchYawrateThrust())
    before = len(rec.published[SETPOINT_TOPIC])
    tick(n, rec, 0.18)
    tick(n, rec, 0.20)
    assert len(rec.published[SETPOINT_TOPIC]) == before


def test_handback_hold_point_is_latched_not_chased(node):
    """Re-reading odometry every tick would be a drift, not a hold, and would
    disagree with the ROBOT_POSE target the pid is about to track."""
    n, rec = node
    feed_odom(rec, (10.0, 20.0, 30.0))
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0)
    tick(n, rec, 0.02)
    resolve_last_call(rec, PID_SVC)
    tick(n, rec, 0.04)
    feed_engaged(rec, 0.06, False)
    tick(n, rec, 0.08)
    feed_odom(rec, (77.0, 88.0, 99.0))        # the vehicle drifted
    tick(n, rec, 0.10)
    hold = rec.last(SETPOINT_TOPIC)
    assert (hold.position.x, hold.position.y) == (10.0, 20.0)


def test_goal_churn_streams_straight_through_the_gaps(node_grace):
    """The raven-search regression, through the real ROS shell: task ends,
    next task starts ~2 s later, repeatedly. No mute/unmute churn, no mode
    calls, and a PositionTarget on every tick."""
    n, rec = node_grace
    feed_odom(rec, (0.0, 0.0, 10.0))
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0, p=(1.0, 0.0, 10.0), v=(2.0, 0.0, 0.0))
    tick(n, rec, 0.02)
    resolve_last_call(rec, PID_SVC)
    rec.clear_published()

    t, ticks = 0.02, 0
    for cycle in range(4):
        for _ in range(100):                  # 2 s engaged
            t += 0.02
            feed_goal(rec, t, p=(float(cycle), 0.0, 10.0), v=(2.0, 0.0, 0.0))
            feed_engaged(rec, t, True)
            tick(n, rec, t)
            ticks += 1
        for _ in range(100):                  # 2 s between tasks
            t += 0.02
            feed_goal(rec, t, p=(99.0, 99.0, 99.0), v=(2.0, 0.0, 0.0))
            feed_engaged(rec, t, False)
            tick(n, rec, t)
            ticks += 1
        for call in rec.calls_to(PID_SVC):    # let the deadman refreshes land
            if not call.future.done:
                call.future.finish(ros_stubs.set_parameters_response(True))

    assert len(rec.published[SETPOINT_TOPIC]) == ticks, 'stream had gaps'
    assert rec.calls_to(MODE_SVC) == []
    assert all(mute_value(c) is True for c in rec.calls_to(PID_SVC)), \
        'un-muted between search goals'
    # Production flavour FOLLOWS the live Goals through the gap — the spline
    # flows continuously (grace_follow_goals, default on via env).
    assert rec.last(SETPOINT_TOPIC).position.x == 99.0


def test_stale_goals_hold_at_the_last_commanded_point(node):
    n, rec = node
    feed_odom(rec, (10.0, 20.0, 30.0))
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0, p=(4.0, 5.0, 6.0), v=(3.0, 0.0, 0.0))
    tick(n, rec, 0.02)
    resolve_last_call(rec, PID_SVC)
    tick(n, rec, 0.04)
    rec.clear_published()
    feed_engaged(rec, 1.0, True)              # heartbeat only, no new goal
    tick(n, rec, 1.02)
    hold = rec.last(SETPOINT_TOPIC)
    # last GOAL, not the vehicle: we are still engaged and still going there
    assert (hold.position.x, hold.position.y, hold.position.z) == (4., 5., 6.)
    assert (hold.velocity.x, hold.velocity.y, hold.velocity.z) == (0., 0., 0.)


# ── refusals ────────────────────────────────────────────────────────────────

def test_a_goal_in_a_foreign_frame_is_refused_not_flown(node):
    n, rec = node
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0, p=(1.0, 2.0, 3.0), frame_id='world')
    for i in range(1, 10):
        tick(n, rec, i * 0.02)
    assert rec.published.get(SETPOINT_TOPIC) is None
    assert rec.calls_to(PID_SVC) == [], 'muted the pid for a goal it refused'
    assert any('world' in log for log in rec.logs if log.startswith('ERROR'))


def test_an_unavailable_pid_service_keeps_us_on_the_legacy_path(node):
    n, rec = node
    rec.ready_services.discard(PID_SVC)
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0)
    for i in range(1, 30):
        tick(n, rec, i * 0.02)
    assert rec.published.get(SETPOINT_TOPIC) is None
    assert rec.calls_to(PID_SVC) == []


def test_a_refused_mute_keeps_us_on_the_legacy_path(node):
    n, rec = node
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0)
    tick(n, rec, 0.02)
    resolve_last_call(rec, PID_SVC, successful=False)
    tick(n, rec, 0.04)
    assert rec.published.get(SETPOINT_TOPIC) is None


def test_a_dead_bridge_gives_the_vehicle_back(node_grace):
    """No engagement heartbeat: the bridge is gone and we must not keep the
    pid muted behind it. With the grace window that now costs
    engage_timeout_s (3) + disengage_grace_s (4) — deliberately measured on
    the production fixture, because that total is the real worst case."""
    n, rec = node_grace
    feed_odom(rec)
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0)
    tick(n, rec, 0.02)
    resolve_last_call(rec, PID_SVC)
    tick(n, rec, 0.04)
    assert rec.last(SETPOINT_TOPIC) is not None
    for i in range(1, 11):                    # 10 s of silence
        feed_goal(rec, i * 1.0)               # MIGHTY still alive, bridge not
        tick(n, rec, i * 1.0 + 0.02)
        for call in rec.calls_to(MODE_SVC):
            if not call.future.done:
                call.future.finish(None)
    assert mute_value(rec.calls_to(PID_SVC)[-1]) is False


def test_the_mute_deadman_is_refreshed_while_streaming(node):
    n, rec = node
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0)
    tick(n, rec, 0.02)
    resolve_last_call(rec, PID_SVC)
    for i in range(3, 53):                    # 1 s at 50 Hz
        t = i * 0.02
        feed_goal(rec, t)
        feed_engaged(rec, t, True)
        tick(n, rec, t)
        for call in rec.calls_to(PID_SVC):
            if not call.future.done:
                call.future.finish(ros_stubs.set_parameters_response(True))
    refreshes = [c for c in rec.calls_to(PID_SVC) if mute_value(c) is True]
    assert 5 <= len(refreshes) <= 7, len(refreshes)


def test_goal_churn_hold_flavour_freezes_on_last_commanded(node_grace_hold, monkeypatch):
    """With follow OFF the grace must NOT chase Goals mighty keeps publishing
    for a finished route — it freezes on the last commanded point."""
    n, rec = node_grace_hold
    feed_odom(rec, (0.0, 0.0, 10.0))
    feed_engaged(rec, 0.0, True)
    feed_goal(rec, 0.0, p=(3.0, 0.0, 10.0), v=(1.0, 0.0, 0.0))
    tick(n, rec, 0.0)
    for call in rec.calls_to(PID_SVC):
        if not call.future.done:
            call.future.finish(ros_stubs.set_parameters_response(True))
    tick(n, rec, 0.1)
    # disengage; mighty keeps publishing a (99,99,99) goal
    feed_engaged(rec, 0.2, False)
    feed_goal(rec, 0.2, p=(99.0, 99.0, 99.0), v=(2.0, 0.0, 0.0))
    tick(n, rec, 0.2)
    tick(n, rec, 0.4)
    assert rec.last(SETPOINT_TOPIC).position.x == 3.0
