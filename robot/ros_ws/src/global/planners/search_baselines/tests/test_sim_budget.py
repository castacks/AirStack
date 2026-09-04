#!/usr/bin/env python3
"""The 600 s SIM-time budget: when it starts, and where the time comes from.

    python3 -m pytest robot/ros_ws/src/global/planners/search_baselines/tests/test_sim_budget.py

Pure logic — `_sim_now` and `_sim_budget_spent` are sliced out of
planner_node.py and exec'd on a stub, the way test_search_area_source.py and
test_frame_consistency.py do it. No ROS, no sim.

TWO CLAIMS ARE PINNED HERE.

1. **The budget starts when the METHOD starts, not when the sim booted.**
   `_sim_t0` is latched on the first call, and `_tick` only calls it after
   `_snapshot()` has returned rgb + depth + camera_info + odometry — so the
   scene load, the stack bring-up and the takeoff are not charged to the
   method. A drone that reaches its first data tick at sim t=900 s gets
   600 s from 900, not 600 s from 0 (which would have ended the run before
   the planner ever commanded anything).

2. **The time source falls back to the observation stamps when `/clock` is
   not on this domain.** Isaac publishes `/clock` from a per-drone OmniGraph
   node bound to that drone's ROS domain, so it exists on domains 1..N and not
   on 0, and `dds_router.yaml` does not bridge it. The CoNavGPT2 TEAM planner
   runs on domain 0: without the fallback its clock reads 0.0 forever,
   `run_complete` never latches, and the run ends only when the mission's gate
   times out hours later.
"""

import os
import sys
import textwrap
import time
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PN_PATH = os.path.normpath(os.path.join(HERE, '..', 'search_baselines',
                                        'planner_node.py'))
SRC = open(PN_PATH, encoding='utf-8').read()


def _method_src(name):
    i = SRC.index(f'    def {name}(')
    j = SRC.index('\n    def ', i + 10)
    return SRC[i:j]


def _build(names):
    ns = {'os': os, 'time': time}
    body = '\n'.join(textwrap.dedent(_method_src(n)) for n in names)
    exec('class T:\n' + textwrap.indent(body, '    '), ns)   # noqa: S102
    return ns['T']


T = _build(['_sim_now', '_sim_budget_spent'])


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(m)

    def warn(self, m, **kw):
        self.lines.append(m)


class _Clock:
    def __init__(self, seconds):
        self.seconds = seconds

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self.seconds * 1e9))


def _stamp(seconds):
    return types.SimpleNamespace(sec=int(seconds),
                                 nanosec=int((seconds % 1.0) * 1e9))


class Node(T):
    """The smallest stub the two sliced methods touch."""

    def __init__(self, clock_s=0.0, stamps=(), budget=600.0):
        self._clock = _Clock(clock_s)
        self._stamp = list(stamps)
        self._max_sim_seconds = budget
        self._sim_t0 = None
        self._sim_wall_t0 = None
        self.monitor_starts = 0
        self._log = _Log()

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._log

    def _start_sim_budget_monitor(self):
        # The pure-logic test does not start a background thread; it records
        # the handoff so we can prove the watchdog is armed once at t0.
        self.monitor_starts += 1


# ---------------------------------------------------------------------------
# 1. the budget starts when the method does
# ---------------------------------------------------------------------------

def test_t0_is_the_first_call_not_sim_zero():
    """The sim has been running since bring-up; the budget has not."""
    n = Node(clock_s=900.0)
    assert n._sim_budget_spent() is False       # latches t0 = 900
    assert n._sim_t0 == pytest.approx(900.0)
    n._clock.seconds = 1499.0                   # 599 s of search
    assert n._sim_budget_spent() is False
    n._clock.seconds = 1500.0                   # 600 s exactly
    assert n._sim_budget_spent() is True


def test_the_budget_is_exactly_max_sim_seconds_long():
    n = Node(clock_s=10.0, budget=600.0)
    n._sim_budget_spent()
    for elapsed, spent in ((0.0, False), (300.0, False), (599.9, False),
                           (600.0, True), (601.0, True)):
        n._clock.seconds = 10.0 + elapsed
        assert n._sim_budget_spent() is spent, elapsed


def test_t0_is_announced_with_its_time_source():
    """A run has to be able to say how it timed itself."""
    n = Node(clock_s=42.0)
    n._sim_budget_spent()
    assert any('sim budget 600 s starts NOW' in m for m in n._log.lines), \
        n._log.lines
    assert any('Time source: clock' in m for m in n._log.lines), n._log.lines


def test_budget_watchdog_is_armed_once_when_t0_is_latched():
    n = Node(clock_s=42.0)
    n._sim_budget_spent()
    n._clock.seconds = 43.0
    n._sim_budget_spent()
    assert n.monitor_starts == 1
    assert n._sim_wall_t0 is not None


def test_a_zero_budget_never_ends_the_run():
    n = Node(clock_s=5.0, budget=0.0)
    n._clock.seconds = 1e6
    assert n._sim_budget_spent() is False
    assert n._sim_t0 is None


def test_nothing_is_charged_before_the_first_tick_with_data():
    """`_tick` calls this only after `_snapshot()` returns data. Modelled here
    by not calling it until the stamps exist: t0 is the time of the FIRST
    call, whenever that is, so takeoff and the scene load cost the method
    nothing."""
    n = Node(clock_s=0.0, stamps=[])
    assert n._sim_budget_spent() is False        # no clock, no data -> no t0
    assert n._sim_t0 is None
    n._clock.seconds = 1234.0                    # sim ran on through takeoff
    n._sim_budget_spent()
    assert n._sim_t0 == pytest.approx(1234.0)


# ---------------------------------------------------------------------------
# 2. the time source
# ---------------------------------------------------------------------------

def test_clock_wins_when_it_is_advancing():
    n = Node(clock_s=100.0, stamps=[_stamp(50.0)])
    assert n._sim_now() == pytest.approx(100.0)


def test_stamps_are_used_when_there_is_no_clock_on_this_domain():
    """The CoNavGPT2 team arm on domain 0."""
    n = Node(clock_s=0.0, stamps=[_stamp(120.0), _stamp(125.5), None])
    assert n._sim_now() == pytest.approx(125.5)


def test_the_team_arm_terminates_on_stamps_alone():
    """The regression this fallback exists for: before it, a domain-0 planner
    never spent its budget and the mission's gate timed out instead."""
    stamps = [_stamp(300.0)]
    n = Node(clock_s=0.0, stamps=stamps)
    assert n._sim_budget_spent() is False
    assert n._sim_t0 == pytest.approx(300.0)
    n._stamp[0] = _stamp(899.0)
    assert n._sim_budget_spent() is False
    n._stamp[0] = _stamp(900.0)
    assert n._sim_budget_spent() is True
    assert any('observation stamps' in m for m in n._log.lines), n._log.lines


def test_no_clock_and_no_data_is_not_time_zero():
    """Returning 0.0 would latch t0 at 0 and end the run 600 s of sim later
    measured from the wrong origin; None means "not knowable yet"."""
    n = Node(clock_s=0.0, stamps=[None, None])
    assert n._sim_now() is None
    assert n._sim_budget_spent() is False
    assert n._sim_t0 is None


def test_stamps_are_read_as_sec_plus_nanosec():
    n = Node(clock_s=0.0, stamps=[_stamp(12.25)])
    assert n._sim_now() == pytest.approx(12.25, abs=1e-6)


def test_a_missing_stamp_attribute_is_survivable():
    """`_sim_now` is called from `_tick` before `_stamp` is guaranteed to hold
    anything; it must not raise on a half-constructed node."""
    n = Node(clock_s=0.0)
    del n._stamp
    assert n._sim_now() is None
