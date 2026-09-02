"""The engagement gate and the two handoff orderings.

This is the safety-critical half of the native setpoint path. PX4 must have
exactly one commander at every instant except a deliberate overlap where both
command the same hold, and the pid must never be left muted with nothing
streaming. Every rule below corresponds to a specific way to lose the vehicle.
"""
import pytest

from mighty_bridge.native_setpoint import (
    STATE_GRACE, STATE_HANDBACK, STATE_IDLE, STATE_MUTING, STATE_STREAMING,
    STREAM_GOAL, STREAM_HOLD, STREAM_NONE, HandoffConfig, HandoffStateMachine)


def make(**kw):
    cfg = HandoffConfig(**{'enabled': True, **kw})
    return HandoffStateMachine(cfg)


def make_nograce(**kw):
    """A machine that hands back the instant it is disengaged.

    The handback ORDERING tests below use this so they read as one sequence
    instead of being interleaved with a 4 s wait. The grace window itself —
    including that its expiry runs exactly this handback — is covered in its
    own section further down.
    """
    return make(**{'disengage_grace_s': 0.0, **kw})


def engaged_tick(sm, t, *, engaged=True, engagement_age=0.0, goal_age=0.0):
    return sm.tick(t, engagement_age=engagement_age, engaged=engaged,
                   goal_age=goal_age)


def arm(sm, t=0.0):
    """Drive IDLE -> STREAMING the way the node does: request, confirm, tick."""
    a = engaged_tick(sm, t)
    assert a.request_mute is True
    sm.note_mute_result(t, True, True)
    assert sm.state == STATE_STREAMING
    return engaged_tick(sm, t + 0.02)


def start_handback(sm, t, **kw):
    """Step 0 of the handback plus the trajectory controller's ROBOT_POSE
    acknowledgement, which is what the node reports back before step 1."""
    a = engaged_tick(sm, t, engaged=False, **kw)
    assert a.request_robot_pose is True
    sm.note_robot_pose_result(True)
    return a


# ── disabled is inert ───────────────────────────────────────────────────────

def test_disabled_never_streams_and_never_touches_the_pid():
    sm = HandoffStateMachine(HandoffConfig(enabled=False))
    for i in range(20):
        a = sm.tick(i * 0.02, engagement_age=0.0, engaged=True, goal_age=0.0)
        assert a.stream is STREAM_NONE
        assert a.request_mute is None
        assert a.request_robot_pose is False
        assert a.state == STATE_IDLE


def test_disabling_mid_flight_still_hands_back_in_order():
    sm = make()
    arm(sm, 0.0)
    sm.cfg = HandoffConfig(**{**sm.cfg.__dict__, 'enabled': False})
    a = engaged_tick(sm, 1.0)                 # engaged=True, but the switch is off
    assert a.state == STATE_HANDBACK and a.request_robot_pose
    sm.note_robot_pose_result(True)
    a = engaged_tick(sm, 1.02)
    assert a.request_mute is False


# ── the engagement gate ─────────────────────────────────────────────────────

def test_never_streams_before_the_follower_engages():
    sm = make()
    for i in range(10):
        a = engaged_tick(sm, i * 0.02, engaged=False)
        assert a.stream is STREAM_NONE and a.request_mute is None


def test_never_streams_without_a_fresh_mighty_goal():
    """Muting into silence is the way to lose the vehicle without any error:
    the pid stops, nothing replaces it, PX4's OFFBOARD failsafe trips."""
    sm = make()
    a = engaged_tick(sm, 0.0, goal_age=None)          # MIGHTY never published
    assert a.request_mute is None and sm.state == STATE_IDLE
    a = engaged_tick(sm, 0.02, goal_age=99.0)         # ... or long dead
    assert a.request_mute is None and sm.state == STATE_IDLE


def test_a_stale_engagement_heartbeat_counts_as_disengaged():
    """The bridge republishes at 1 Hz; silence means the bridge died, and a
    dead bridge must not leave us holding the vehicle — though it now costs
    engage_timeout_s AND the grace window before control comes back."""
    sm = make(engage_timeout_s=3.0, disengage_grace_s=4.0)
    arm(sm, 0.0)
    a = engaged_tick(sm, 10.0, engagement_age=5.0)
    assert a.state == STATE_GRACE
    a = engaged_tick(sm, 14.5, engagement_age=9.5)
    assert a.state == STATE_HANDBACK


# ── engage ordering: mute first, THEN stream ────────────────────────────────

def test_engage_mutes_before_the_first_setpoint():
    sm = make()
    a = engaged_tick(sm, 0.0)
    assert a.request_mute is True
    assert a.stream is STREAM_NONE, 'streamed before the mute was confirmed'
    assert sm.state == STATE_MUTING


def test_streaming_starts_only_on_a_CONFIRMED_mute():
    sm = make()
    engaged_tick(sm, 0.0)
    for i in range(5):                      # service still in flight
        a = engaged_tick(sm, 0.02 * (i + 1))
        assert a.stream is STREAM_NONE
    sm.note_mute_result(0.2, True, True)
    a = engaged_tick(sm, 0.22)
    assert a.stream == STREAM_GOAL


def test_a_refused_mute_keeps_us_off_the_vehicle_and_retries():
    """pid_controller missing, or refusing: falling back to the legacy path is
    correct. Streaming anyway would give PX4 two masters."""
    sm = make(mute_retry_s=0.5)
    engaged_tick(sm, 0.0)
    sm.note_mute_result(0.0, True, False)
    a = engaged_tick(sm, 0.1)
    assert a.stream is STREAM_NONE and a.request_mute is None
    a = engaged_tick(sm, 0.6)               # retry window open
    assert a.request_mute is True and a.stream is STREAM_NONE


def test_disengaging_mid_mute_asks_for_the_unmute_anyway():
    """The mute request is async: it may land after we gave up on it. Without
    this the pid stays muted with nothing streaming until its watchdog."""
    sm = make()
    engaged_tick(sm, 0.0)
    a = engaged_tick(sm, 0.02, engaged=False)
    assert a.request_mute is False
    assert sm.state == STATE_IDLE


# ── streaming ───────────────────────────────────────────────────────────────

def test_streaming_refreshes_the_mute_deadman_on_cadence():
    """pid_controller un-mutes itself if the flag is not re-asserted. That is
    what protects the vehicle if THIS node dies, so the refresh is not
    bookkeeping — losing it re-opens the exact hole it closes."""
    sm = make(mute_refresh_s=0.2)
    arm(sm, 0.0)
    refreshes = 0
    for i in range(1, 51):                  # 1 s at 50 Hz
        a = engaged_tick(sm, i * 0.02)
        assert a.stream == STREAM_GOAL
        if a.request_mute is True:
            refreshes += 1
    assert 4 <= refreshes <= 6, refreshes    # ~5 Hz


def test_stale_goals_hold_position_rather_than_replay_a_stale_velocity():
    sm = make(goal_stale_s=0.5, hold_max_s=5.0)
    arm(sm, 0.0)
    a = engaged_tick(sm, 1.0, goal_age=0.9)
    assert a.stream == STREAM_HOLD and sm.state == STATE_STREAMING


def test_goals_stale_for_too_long_hand_the_vehicle_back():
    """Land / RTL / fixed_trajectory all go through the muted pid. If MIGHTY
    dies mid-route and we just kept holding, those tasks would be inert."""
    sm = make(goal_stale_s=0.5, hold_max_s=2.0)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, goal_age=0.9)
    a = engaged_tick(sm, 3.5, goal_age=3.4)
    assert a.state == STATE_HANDBACK


def test_goals_coming_back_resets_the_stale_clock():
    sm = make(goal_stale_s=0.5, hold_max_s=2.0)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, goal_age=0.9)
    engaged_tick(sm, 1.5, goal_age=0.0)      # MIGHTY recovered
    a = engaged_tick(sm, 2.6, goal_age=0.9)  # 1.1 s later, not 2.6
    assert a.state == STATE_STREAMING and a.stream == STREAM_HOLD


def test_a_refused_deadman_refresh_hands_back_before_the_mute_lapses():
    """pid_controller un-mutes itself command_mute_timeout after the last
    accepted refresh. Arriving at that by surprise means it starts commanding
    while we are still streaming — two masters. Leave deliberately instead."""
    sm = make()
    arm(sm, 0.0)
    sm.note_mute_result(0.5, True, False)     # refresh REFUSED
    a = engaged_tick(sm, 0.52)
    assert a.state == STATE_HANDBACK


def test_a_pid_command_while_streaming_is_recorded_as_a_two_masters_fault():
    sm = make()
    arm(sm, 0.0)
    sm.note_pid_command()
    sm.note_pid_command()
    assert sm.pid_commands_while_streaming == 2


# ── the disengage grace window ──────────────────────────────────────────────
# Measured on the first raven-search flight: raven issues a NEW NavigateTask
# every few seconds, so disengage-means-handback produced a continuous
# engage/handback cycle — back-to-back MUTED/UNMUTED in the pid log and flight
# as jerky as the legacy path. A disengage is provisional for
# disengage_grace_s; the next task normally arrives inside it.

def test_a_disengage_does_not_immediately_hand_back():
    sm = make(disengage_grace_s=4.0)
    arm(sm, 0.0)
    a = engaged_tick(sm, 1.0, engaged=False)
    assert a.state == STATE_GRACE
    assert a.stream == STREAM_HOLD, 'stopped streaming on a provisional disengage'
    assert a.request_robot_pose is False
    assert a.request_mute is not False, 'un-muted on a provisional disengage'


def test_goal_churn_costs_no_service_calls_and_never_breaks_the_stream():
    """THE regression this window exists for. raven finishes a NavigateTask and
    starts the next one ~2 s later, over and over. Across the whole burst the
    pid must be muted exactly once (at the very start) and never un-muted, and
    a PositionTarget must go out on every single tick."""
    sm = make(disengage_grace_s=4.0, mute_refresh_s=0.2)
    arm(sm, 0.0)
    unmutes, mode_calls, silent_ticks = 0, 0, 0

    t = 0.0
    for _cycle in range(6):
        # ~3 s engaged, flying goals
        for _ in range(150):
            t += 0.02
            a = engaged_tick(sm, t)
            if a.stream is STREAM_NONE:
                silent_ticks += 1
            if a.request_mute is False:
                unmutes += 1
            if a.request_robot_pose:
                mode_calls += 1
        # ~2 s between tasks: bridge disengaged, MIGHTY still publishing
        for _ in range(100):
            t += 0.02
            a = engaged_tick(sm, t, engaged=False)
            if a.stream is STREAM_NONE:
                silent_ticks += 1
            if a.request_mute is False:
                unmutes += 1
            if a.request_robot_pose:
                mode_calls += 1
            assert a.state == STATE_GRACE

    assert unmutes == 0, 'handed the vehicle back mid-search'
    assert mode_calls == 0, 'churned the trajectory controller mode'
    assert silent_ticks == 0, 'left PX4 without a setpoint'
    assert sm.state == STATE_GRACE


def test_reengaging_within_the_grace_resumes_goals_on_the_SAME_tick():
    """No hold tick between the re-engage and the first fresh Goal: that
    stutter is precisely what the window is meant to remove."""
    sm = make(disengage_grace_s=4.0)
    arm(sm, 0.0)
    a = engaged_tick(sm, 1.0, engaged=False)
    assert a.stream == STREAM_HOLD
    a = engaged_tick(sm, 1.5)                 # next NavigateTask engages
    assert sm.state == STATE_STREAMING
    assert a.stream == STREAM_GOAL
    assert a.request_mute is not False
    assert a.request_robot_pose is False


def test_the_deadman_keeps_being_fed_through_the_grace():
    """The pid un-mutes itself if the flag goes un-refreshed. We still own the
    vehicle during grace, so dropping the refresh here would let the pid start
    commanding underneath us."""
    sm = make(disengage_grace_s=4.0, mute_refresh_s=0.2)
    arm(sm, 0.0)
    refreshes = 0
    for i in range(1, 101):                   # 2 s of grace at 50 Hz
        a = engaged_tick(sm, 1.0 + i * 0.02, engaged=False)
        assert a.state == STATE_GRACE
        if a.request_mute is True:
            refreshes += 1
    assert 9 <= refreshes <= 11, refreshes    # ~5 Hz, unchanged from streaming


def test_grace_expiry_runs_the_existing_ordered_handback():
    sm = make(disengage_grace_s=4.0, handback_confirm_msgs=3,
              handback_timeout_s=10.0)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, engaged=False)
    a = engaged_tick(sm, 4.9, engaged=False)
    assert a.state == STATE_GRACE, 'handed back early'

    a = engaged_tick(sm, 5.1, engaged=False)  # 4.1 s of quiet
    assert a.state == STATE_HANDBACK
    assert a.request_robot_pose is True
    assert a.request_mute is None
    sm.note_robot_pose_result(True)
    a = engaged_tick(sm, 5.12, engaged=False)
    assert a.request_mute is False
    for _ in range(3):
        sm.note_pid_command()
    a = engaged_tick(sm, 5.14, engaged=False)
    assert a.stream is STREAM_NONE and sm.state == STATE_IDLE


def test_engaging_after_the_grace_expired_pays_a_full_re_mute():
    sm = make(disengage_grace_s=4.0, handback_confirm_msgs=1,
              handback_timeout_s=0.5)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, engaged=False)
    engaged_tick(sm, 5.1, engaged=False)
    sm.note_robot_pose_result(True)
    engaged_tick(sm, 5.12, engaged=False)
    sm.note_pid_command()
    engaged_tick(sm, 5.14, engaged=False)
    assert sm.state == STATE_IDLE
    a = engaged_tick(sm, 6.0)
    assert a.request_mute is True and a.stream is STREAM_NONE


def test_the_master_switch_going_off_skips_the_grace():
    """An operator asking for the legacy path back gets it now, not in 4 s."""
    sm = make(disengage_grace_s=4.0)
    arm(sm, 0.0)
    sm.cfg = HandoffConfig(**{**sm.cfg.__dict__, 'enabled': False})
    a = engaged_tick(sm, 1.0)
    assert a.state == STATE_HANDBACK


def test_stale_mighty_goals_skip_the_grace_too():
    """Waiting is not conservative here: MIGHTY is gone, and land / RTL /
    fixed_trajectory are inert behind our mute until we release it."""
    sm = make(disengage_grace_s=4.0, goal_stale_s=0.5, hold_max_s=2.0)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, goal_age=0.9)
    a = engaged_tick(sm, 3.5, goal_age=3.4)   # still ENGAGED, planner silent
    assert a.state == STATE_HANDBACK


def test_a_refused_deadman_refresh_during_grace_hands_back():
    sm = make(disengage_grace_s=4.0)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, engaged=False)
    sm.note_mute_result(1.1, True, False)
    a = engaged_tick(sm, 1.2, engaged=False)
    assert a.state == STATE_HANDBACK


def test_a_pid_command_during_grace_is_a_two_masters_fault():
    sm = make(disengage_grace_s=4.0)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, engaged=False)
    sm.note_pid_command()
    assert sm.pid_commands_while_streaming == 1


def test_grace_zero_restores_hand_back_immediately():
    sm = make(disengage_grace_s=0.0)
    arm(sm, 0.0)
    a = engaged_tick(sm, 1.0, engaged=False)
    assert a.state == STATE_HANDBACK


# ── handback ordering: robot_pose, THEN unmute, THEN stop ───────────────────

def test_handback_holds_pins_the_tracking_point_then_unmutes_then_stops():
    sm = make_nograce(handback_confirm_msgs=3, handback_timeout_s=10.0)
    arm(sm, 0.0)

    # step 0: still streaming (a hold), tracking point pinned, NOT yet unmuted
    a = start_handback(sm, 1.0)
    assert a.state == STATE_HANDBACK
    assert a.stream == STREAM_HOLD
    assert a.request_mute is None, 'un-muted before pinning the tracking point'

    # step 1: unmute, still streaming the hold
    a = engaged_tick(sm, 1.02, engaged=False)
    assert a.request_mute is False
    assert a.stream == STREAM_HOLD
    assert a.request_robot_pose is False

    # step 2+: overlap until the pid is SEEN commanding again
    a = engaged_tick(sm, 1.04, engaged=False)
    assert a.stream == STREAM_HOLD
    for _ in range(3):
        sm.note_pid_command()
    a = engaged_tick(sm, 1.06, engaged=False)
    assert a.stream is STREAM_NONE
    assert sm.state == STATE_IDLE


def test_handback_gives_up_on_the_confirmation_after_the_timeout():
    """pid_controller returns early without publishing when a TF lookup fails,
    so the confirmation may never arrive. The overlap must still be bounded."""
    sm = make_nograce(handback_confirm_msgs=3, handback_timeout_s=1.0)
    arm(sm, 0.0)
    start_handback(sm, 1.0)
    engaged_tick(sm, 1.02, engaged=False)
    a = engaged_tick(sm, 1.5, engaged=False)
    assert a.stream == STREAM_HOLD, 'stopped before the timeout'
    a = engaged_tick(sm, 2.1, engaged=False)
    assert a.stream is STREAM_NONE and sm.state == STATE_IDLE


def test_handback_never_stops_before_the_unmute_is_issued():
    """A zero timeout must not be able to produce "stream stopped, pid still
    muted" — the one state with no commander at all."""
    sm = make_nograce(handback_confirm_msgs=1, handback_timeout_s=0.0)
    arm(sm, 0.0)
    a = start_handback(sm, 1.0)
    assert a.stream == STREAM_HOLD and a.request_mute is None
    sm.note_pid_command()                    # confirmation arrives immediately
    a = engaged_tick(sm, 1.02, engaged=False)
    assert a.request_mute is False, 'finished the handback without un-muting'
    assert a.stream == STREAM_HOLD
    a = engaged_tick(sm, 1.04, engaged=False)
    assert a.stream is STREAM_NONE


def test_the_unmute_waits_for_the_tracking_point_to_be_pinned():
    """mighty_bridge asks the SAME service for TRACK the instant it
    disengages, and TRACK with a cleared trajectory freezes the tracking point
    at the last MIGHTY waypoint. Un-muting before our ROBOT_POSE has been
    answered would wake the pid onto exactly that stale target."""
    sm = make_nograce(handback_pin_s=0.3)
    arm(sm, 0.0)
    a = engaged_tick(sm, 1.0, engaged=False)
    assert a.request_robot_pose is True and a.request_mute is None
    for t in (1.02, 1.1, 1.2):
        a = engaged_tick(sm, t, engaged=False)
        assert a.request_mute is None, 'un-muted before the controller answered'
        assert a.stream == STREAM_HOLD
    sm.note_robot_pose_result(True)
    a = engaged_tick(sm, 1.22, engaged=False)
    assert a.request_mute is False


def test_the_pin_wait_is_bounded_when_the_controller_never_answers():
    sm = make_nograce(handback_pin_s=0.3)
    arm(sm, 0.0)
    engaged_tick(sm, 1.0, engaged=False)
    a = engaged_tick(sm, 1.2, engaged=False)
    assert a.request_mute is None
    a = engaged_tick(sm, 1.4, engaged=False)
    assert a.request_mute is False, 'stalled forever on a silent controller'


def test_a_refused_unmute_is_re_asked_rather_than_shrugged_off():
    """The pid staying muted while we wind the stream down is the worst
    outcome available, so a refusal must not just be logged."""
    sm = make_nograce(handback_confirm_msgs=99, handback_timeout_s=10.0)
    arm(sm, 0.0)
    start_handback(sm, 1.0)
    a = engaged_tick(sm, 1.02, engaged=False)
    assert a.request_mute is False
    sm.note_mute_result(1.03, False, False)          # REFUSED
    a = engaged_tick(sm, 1.04, engaged=False)
    assert a.request_mute is False, 'gave up after one refused un-mute'
    sm.note_mute_result(1.05, False, True)
    a = engaged_tick(sm, 1.06, engaged=False)
    assert a.request_mute is None


def test_reengaging_during_handback_re_mutes_before_streaming_goals_again():
    sm = make_nograce()
    arm(sm, 0.0)
    start_handback(sm, 1.0)
    engaged_tick(sm, 1.02, engaged=False)    # unmuted
    a = engaged_tick(sm, 1.04)               # follower engaged again
    assert sm.state == STATE_MUTING
    assert a.request_mute is True
    assert a.stream == STREAM_HOLD, 'dropped the stream while re-muting'
    sm.note_mute_result(1.06, True, True)
    a = engaged_tick(sm, 1.08)
    assert a.stream == STREAM_GOAL


def test_a_full_cycle_returns_to_a_clean_idle():
    sm = make_nograce(handback_confirm_msgs=1, handback_timeout_s=0.5)
    arm(sm, 0.0)
    start_handback(sm, 1.0)
    engaged_tick(sm, 1.02, engaged=False)
    sm.note_pid_command()
    engaged_tick(sm, 1.04, engaged=False)
    assert sm.state == STATE_IDLE
    # and a second engagement behaves exactly like the first
    a = engaged_tick(sm, 2.0)
    assert a.request_mute is True and a.stream is STREAM_NONE


# ── clock robustness ────────────────────────────────────────────────────────

def test_negative_ages_from_a_sim_time_jump_are_clamped_not_trusted():
    sm = make()
    a = sm.tick(0.0, engagement_age=-5.0, engaged=True, goal_age=-5.0)
    assert a.request_mute is True            # clamped to 0 == fresh, not stale
    sm.note_mute_result(0.0, True, True)
    a = sm.tick(0.02, engagement_age=-1.0, engaged=True, goal_age=-1.0)
    assert a.stream == STREAM_GOAL


@pytest.mark.parametrize('age', [None, 1e9])
def test_never_heard_from_the_bridge_means_disengaged(age):
    sm = make()
    a = sm.tick(0.0, engagement_age=age, engaged=True, goal_age=0.0)
    assert a.request_mute is None and a.stream is STREAM_NONE


# ── grace_follow_goals: the spline flows straight through task swaps ─────────

def test_grace_follow_streams_goals_not_holds():
    sm = make(grace_follow_goals=True)
    arm(sm)
    engaged_tick(sm, 0.1, engaged=False)          # disengage -> GRACE
    assert sm.state == STATE_GRACE
    a = engaged_tick(sm, 0.2, engaged=False)      # fresh goal (goal_age=0)
    assert a.stream == STREAM_GOAL, 'live goals must flow through the grace'


def test_grace_follow_defaults_off_and_holds():
    sm = make()
    arm(sm)
    engaged_tick(sm, 0.1, engaged=False)
    assert sm.state == STATE_GRACE
    a = engaged_tick(sm, 0.2, engaged=False)
    assert a.stream == STREAM_HOLD, 'conservative default: hold, not follow'


def test_grace_follow_still_holds_on_stale_goals():
    sm = make(grace_follow_goals=True)
    arm(sm)
    engaged_tick(sm, 0.1, engaged=False)
    a = engaged_tick(sm, 0.2, engaged=False, goal_age=1.0)  # > goal_stale_s
    assert a.stream == STREAM_HOLD, 'stale goals must never be replayed'
