"""Tests for mtl_sortie_client.py — no ROS needed.

1. The ROS-free GoalTracker / send_until_accepted logic.
2. The whole Sortie flow against an in-process fake of the rclpy surface it
   uses (ActionClient, ClientGoalHandle, status topic, executor), with fake
   takeoff / search_mission servers that can DROP a goal request, LOSE a goal
   response, reject, or run a foreign goal — the failure modes seen in Isaac.

Run:  python3 -m pytest stacks/mtl_search/scripts/tests -q
"""
from __future__ import annotations

import sys
import threading
import time
import types
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import mtl_sortie_client as msc  # noqa: E402

A, B, C = b"A" * 16, b"B" * 16, b"C" * 16


# =============================================================== core logic
def test_tracker_adopts_first_accepted_response():
    t = msc.GoalTracker()
    t.on_sent(A); t.on_sent(B)
    t.on_response(B, True)
    t.on_response(A, True)
    assert t.adopted == B and t.adopted_via == "goal response"
    t.on_status([(A, msc.STATUS_EXECUTING), (B, msc.STATUS_EXECUTING)])
    assert t.duplicates_to_cancel() == [A]


def test_tracker_status_topic_counts_as_acceptance():
    t = msc.GoalTracker()
    t.on_sent(A)
    t.on_status([(A, msc.STATUS_EXECUTING), (C, msc.STATUS_EXECUTING)])
    assert t.accepted() and t.adopted_via == "status topic"
    assert t.foreign_active == {C}
    assert t.terminal_status() is None
    t.on_status([(A, msc.STATUS_SUCCEEDED)])
    assert t.terminal_status() == msc.STATUS_SUCCEEDED
    assert t.foreign_active == set()


def test_tracker_reject_is_not_rejection_if_status_lists_it():
    t = msc.GoalTracker()
    t.on_sent(A)
    t.on_response(A, False)
    assert t.rejected(A)
    t.on_status([(A, msc.STATUS_ACCEPTED)])
    assert not t.rejected(A)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def _run(tracker, on_send, retries=3, timeout=1.0):
    clock, sent, said = FakeClock(), [], []

    def send():
        gid = bytes([65 + len(sent)]) * 16
        tracker.on_sent(gid); sent.append(gid)
        on_send(len(sent), gid, clock)
        return gid

    def sleep(dt):
        clock.sleep(dt)
        for cb in list(pending):
            if clock.t >= cb[0]:
                pending.remove(cb); cb[1]()

    pending = []
    tracker._pending = pending
    out = msc.send_until_accepted(tracker, send, msc.StepConfig("x", accept_timeout_s=timeout,
                                  retries=retries, reject_grace_s=0.5, retry_pause_s=0.1),
                                  said.append, clock=clock, sleep=sleep)
    return out, sent, said


def test_send_retries_after_dropped_request():
    t = msc.GoalTracker()

    def on_send(n, gid, clock):
        if n == 2:
            t._pending.append((clock.t + 0.2, lambda: t.on_response(gid, True)))
    out, sent, said = _run(t, on_send)
    assert out == "accepted" and len(sent) == 2 and t.adopted == sent[1]
    assert any("not accepted within" in s for s in said)


def test_send_gives_up_after_retries():
    t = msc.GoalTracker()
    out, sent, _ = _run(t, lambda *a: None, retries=3)
    assert out == "timeout" and len(sent) == 3


def test_first_attempt_rejection_is_final():
    t = msc.GoalTracker()
    out, sent, _ = _run(t, lambda n, gid, c: t._pending.append((c.t + 0.1, lambda: t.on_response(gid, False))))
    assert out == "rejected" and len(sent) == 1


def test_retry_rejected_adopts_earlier_attempt_from_status():
    """Attempt 1's response was lost; attempt 2 is rejected as busy; the
    status topic then shows attempt 1 running -> adopt it, don't fail."""
    t = msc.GoalTracker()

    def on_send(n, gid, clock):
        if n == 2:
            first = t.sent[0]
            t._pending.append((clock.t + 0.1, lambda: t.on_response(gid, False)))
            t._pending.append((clock.t + 0.3, lambda: t.on_status([(first, msc.STATUS_EXECUTING)])))
    out, sent, _ = _run(t, on_send)
    assert out == "accepted" and t.adopted == sent[0] and t.adopted_via == "status topic"


def test_retry_rejected_by_foreign_goal_is_busy():
    t = msc.GoalTracker()

    def on_send(n, gid, clock):
        if n == 2:
            t._pending.append((clock.t + 0.1, lambda: (t.on_status([(C, msc.STATUS_EXECUTING)]),
                                                       t.on_response(gid, False))))
    out, _, _ = _run(t, on_send)
    assert out == "busy"


def test_format_msg_compact():
    class P:
        def __init__(self): self.x, self.y, self.z = 1.0, 2.25, 3.0
        def get_fields_and_field_types(self): return {"x": "double", "y": "double", "z": "double"}

    class M:
        def __init__(self): self.phase, self.progress, self.current_position = "SEARCH", 0.123456, P()
        def get_fields_and_field_types(self): return {"phase": "string", "progress": "float", "current_position": "P"}
    assert msc.format_msg(M()) == "phase=SEARCH progress=0.12 current_position=(1.0,2.2,3.0)"


# =============================================================== fake rclpy world
class Msg:
    _fields: tuple = ()

    def __init__(self, **kw):
        for f in self._fields:
            setattr(self, f, kw.get(f, 0.0))

    def get_fields_and_field_types(self):
        return {f: "x" for f in self._fields}


def make_action(name, goal_f, result_f, fb_f):
    Goal = type("Goal", (Msg,), {"_fields": goal_f})
    Result = type("Result", (Msg,), {"_fields": result_f})
    Feedback = type("Feedback", (Msg,), {"_fields": fb_f})
    Resp = type("Response", (Msg,), {"_fields": ("accepted", "stamp")})
    impl = types.SimpleNamespace(SendGoalService=types.SimpleNamespace(Response=Resp))
    return type(name, (), {"Goal": Goal, "Result": Result, "Feedback": Feedback, "Impl": impl})


TakeoffTask = make_action("TakeoffTask", ("target_altitude_m", "velocity_m_s"),
                          ("success", "message"), ("target_altitude_m", "current_altitude_m"))
SearchMission = make_action("SearchMission", ("start_mission", "run_id", "scenario_file"),
                            ("success", "message", "run_id"),
                            ("phase", "progress", "remaining_m", "cross_track_error_m"))


class Future:
    def __init__(self):
        self._done, self._res, self._cbs, self._lock = False, None, [], threading.Lock()

    def set_result(self, r):
        with self._lock:
            self._res, self._done = r, True
            cbs = list(self._cbs)
        for cb in cbs:
            cb(self)

    def done(self):
        return self._done

    def result(self):
        return self._res

    def add_done_callback(self, cb):
        with self._lock:
            if not self._done:
                self._cbs.append(cb); return
        cb(self)


class World:
    """Topic bus + action servers shared by the fake rclpy modules."""

    def __init__(self):
        self.subs = {}          # topic -> [callback]
        self.latched = {}       # topic -> last msg (status topics are transient-local)
        self.servers = {}       # action name -> FakeServer
        self.log = []

    def publish(self, topic, msg):
        self.latched[topic] = msg
        for cb in list(self.subs.get(topic, [])):
            cb(msg)


class FakeServer:
    """behaviours: per incoming request, in order (last one repeats):
       'drop' request lost | 'accept' | 'lose_response' accepted but reply lost | 'reject'"""

    def __init__(self, world, name, action, behaviours, run_s=0.3, success=True, busy_rejects=True):
        self.world, self.name, self.action = world, name, action
        self.behaviours, self.run_s, self.success, self.busy_rejects = list(behaviours), run_s, success, busy_rejects
        self.goals = {}         # gid -> status
        self.results = {}
        self.cancelled = []
        self.received = []
        self.active = None
        world.servers[name] = self

    def _status(self):
        arr = types.SimpleNamespace(status_list=[
            types.SimpleNamespace(goal_info=types.SimpleNamespace(goal_id=types.SimpleNamespace(uuid=list(g))),
                                  status=s) for g, s in self.goals.items()])
        self.world.publish(f"{self.name}/_action/status", arr)

    def handle(self, gid, goal, fb_cb):
        beh = self.behaviours.pop(0) if len(self.behaviours) > 1 else self.behaviours[0]
        if beh == "drop":
            return None, False
        self.received.append(gid)
        if beh == "reject" or (self.busy_rejects and self.active is not None):
            return False, True
        self.goals[gid] = msc.STATUS_EXECUTING
        self.active = gid
        self._status()
        threading.Thread(target=self._execute, args=(gid, fb_cb), daemon=True).start()
        return True, beh != "lose_response"

    def start_foreign(self, gid=C):
        self.goals[gid] = msc.STATUS_EXECUTING
        self.active = gid
        self._status()

    def _execute(self, gid, fb_cb):
        end = time.monotonic() + self.run_s
        while time.monotonic() < end:
            if self.goals.get(gid) == msc.STATUS_CANCELED:
                return
            fb = self.action.Feedback(phase="SEARCH", progress=0.5, current_altitude_m=10.0)
            fb_cb(types.SimpleNamespace(goal_id=types.SimpleNamespace(uuid=list(gid)), feedback=fb))
            time.sleep(0.05)
        self.goals[gid] = msc.STATUS_SUCCEEDED if self.success else msc.STATUS_ABORTED
        self.results[gid] = self.action.Result(success=self.success, message="done", run_id="r")
        if self.active == gid:
            self.active = None
        self._status()

    def cancel(self, gid):
        if self.goals.get(gid) in msc.ACTIVE:
            self.goals[gid] = msc.STATUS_CANCELED
            self.cancelled.append(gid)
            if self.active == gid:
                self.active = None
            self._status()


def install_fake_ros(monkeypatch, world):
    mods = {}

    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        mods[name] = m
        return m

    class Node:
        def __init__(self, name):
            self.name = name

        def create_subscription(self, _type, topic, cb, _qos):
            world.subs.setdefault(topic, []).append(cb)
            if topic in world.latched:
                cb(world.latched[topic])
            return (topic, cb)

        def destroy_subscription(self, s):
            world.subs[s[0]].remove(s[1])

        def destroy_node(self):
            pass

    class ClientGoalHandle:
        def __init__(self, client, goal_id, resp):
            self._client, self.goal_id, self._resp = client, goal_id, resp

        @property
        def accepted(self):
            return bool(self._resp.accepted)

        def get_result_async(self):
            f = Future()
            srv, gid = self._client.server, bytes(self.goal_id.uuid)

            def wait():
                while srv.goals.get(gid) not in msc.TERMINAL:
                    time.sleep(0.01)
                f.set_result(types.SimpleNamespace(result=srv.results.get(gid), status=srv.goals[gid]))
            threading.Thread(target=wait, daemon=True).start()
            return f

        def cancel_goal_async(self):
            self._client.server.cancel(bytes(self.goal_id.uuid))
            f = Future(); f.set_result(True)
            return f

    class ActionClient:
        def __init__(self, node, action_type, name):
            self.node, self.action_type, self.name = node, action_type, name

        @property
        def server(self):
            return world.servers[self.name]

        def wait_for_server(self, timeout_sec=None):
            return self.name in world.servers

        def server_is_ready(self):
            return self.name in world.servers

        def send_goal_async(self, goal, feedback_callback=None, goal_uuid=None):
            fut = Future()
            gid = bytes(goal_uuid.uuid)
            world.log.append((self.name, gid))
            accepted, respond = self.server.handle(gid, goal, feedback_callback or (lambda m: None))
            if respond:
                resp = self.action_type.Impl.SendGoalService.Response(accepted=accepted)
                h = ClientGoalHandle(self, goal_uuid, resp)
                threading.Timer(0.02, fut.set_result, args=(h,)).start()
            return fut

    class Executor:
        def __init__(self, num_threads=1):
            self._stop = threading.Event()

        def add_node(self, n):
            pass

        def spin(self):
            self._stop.wait()

        def shutdown(self, timeout_sec=None):
            self._stop.set()

    class QoS:
        def __init__(self, **kw):
            pass

    enum = types.SimpleNamespace(KEEP_LAST=1, RELIABLE=1, TRANSIENT_LOCAL=1)
    rclpy = mod("rclpy", init=lambda **kw: None, create_node=Node, try_shutdown=lambda: None)
    mod("rclpy.signals", SignalHandlerOptions=types.SimpleNamespace(NO=0))
    mod("rclpy.executors", MultiThreadedExecutor=Executor)
    mod("rclpy.qos", QoSProfile=QoS, ReliabilityPolicy=enum, DurabilityPolicy=enum, HistoryPolicy=enum,
        qos_profile_sensor_data=None)
    act = mod("rclpy.action", ActionClient=ActionClient)
    act.client = mod("rclpy.action.client", ClientGoalHandle=ClientGoalHandle)
    rclpy.action = act
    UUID = type("UUID", (), {"__init__": lambda self, uuid: setattr(self, "uuid", list(uuid))})
    mod("unique_identifier_msgs"); mod("unique_identifier_msgs.msg", UUID=UUID)
    mod("action_msgs"); mod("action_msgs.msg", GoalStatusArray=object)
    mod("std_msgs"); mod("std_msgs.msg", Bool=object)
    mod("mtl_msgs"); mod("mtl_msgs.action", SearchMission=SearchMission)
    mod("task_msgs"); mod("task_msgs.action", TakeoffTask=TakeoffTask)
    for k, v in mods.items():
        monkeypatch.setitem(sys.modules, k, v)


@pytest.fixture
def world(monkeypatch):
    w = World()
    install_fake_ros(monkeypatch, w)
    return w


def run_sortie(world, extra=(), healthy=True):
    args = msc.parse_args(["--robot", "robot_2", "--run-id", "R", "--accept-timeout", "0.4",
                           "--retries", "3", "--settle", "0", "--state-estimate-timeout", "0.3",
                           *extra])
    s = msc.Sortie(args)
    if healthy:
        s.timed_out_flag = False
    try:
        return s.run(), s
    finally:
        s.shutdown()


TK, MS = "/robot_2/tasks/takeoff", "/robot_2/search_mission"


def test_sortie_happy_path(world, capsys):
    FakeServer(world, TK, TakeoffTask, ["accept"])
    FakeServer(world, MS, SearchMission, ["accept"])
    rc, _ = run_sortie(world)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "takeoff: finished SUCCEEDED" in out and "sortie succeeded" in out


def test_sortie_takeoff_request_dropped_then_retried(world, capsys):
    """The Isaac failure: the takeoff request never reaches the server."""
    tk = FakeServer(world, TK, TakeoffTask, ["drop", "accept"])
    FakeServer(world, MS, SearchMission, ["accept"])
    rc, _ = run_sortie(world)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert len(tk.received) == 1 and "resending" in out


def test_sortie_mission_request_dropped_twice(world, capsys):
    FakeServer(world, TK, TakeoffTask, ["accept"])
    ms = FakeServer(world, MS, SearchMission, ["drop", "drop", "accept"])
    rc, _ = run_sortie(world)
    assert rc == 0, capsys.readouterr().out
    assert len(ms.received) == 1


def test_sortie_lost_response_confirmed_by_status_topic(world, capsys):
    """Accepted, but the reply is lost: must NOT resend a second takeoff."""
    tk = FakeServer(world, TK, TakeoffTask, ["lose_response"], run_s=1.0)
    FakeServer(world, MS, SearchMission, ["accept"])
    rc, _ = run_sortie(world)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "confirmed by status topic" in out
    assert len([n for n, _ in world.log if n == TK]) == 1 and tk.cancelled == []


def test_sortie_takeoff_rejected_is_takeoff_failure(world, capsys):
    FakeServer(world, TK, TakeoffTask, ["reject"])
    ms = FakeServer(world, MS, SearchMission, ["accept"])
    rc, _ = run_sortie(world)
    assert rc == msc.EXIT_TAKEOFF
    assert ms.received == []


def test_sortie_takeoff_never_accepted(world, capsys):
    FakeServer(world, TK, TakeoffTask, ["drop"])
    FakeServer(world, MS, SearchMission, ["accept"])
    rc, _ = run_sortie(world)
    assert rc == msc.EXIT_NOT_ACCEPTED


def test_sortie_mission_busy_with_foreign_goal(world, capsys):
    FakeServer(world, TK, TakeoffTask, ["accept"])
    ms = FakeServer(world, MS, SearchMission, ["accept"])
    ms.start_foreign()
    rc, _ = run_sortie(world)
    out = capsys.readouterr().out
    assert rc == msc.EXIT_NOT_ACCEPTED, out
    assert "busy with a goal this sortie did not send" in out


def test_sortie_mission_aborted(world, capsys):
    FakeServer(world, TK, TakeoffTask, ["accept"])
    FakeServer(world, MS, SearchMission, ["accept"], success=False)
    rc, _ = run_sortie(world)
    assert rc == msc.EXIT_SORTIE
    assert "finished ABORTED" in capsys.readouterr().out


def test_sortie_dry_run_skips_takeoff(world, capsys):
    ms = FakeServer(world, MS, SearchMission, ["accept"])
    rc, _ = run_sortie(world, extra=["--dry-run"])
    assert rc == 0 and TK not in world.servers and len(ms.received) == 1


def test_sortie_interrupt_cancels_active_goal(world, capsys):
    FakeServer(world, TK, TakeoffTask, ["accept"])
    ms = FakeServer(world, MS, SearchMission, ["accept"], run_s=30.0)
    args = msc.parse_args(["--robot", "robot_2", "--accept-timeout", "0.4", "--settle", "0"])
    s = msc.Sortie(args)
    s.timed_out_flag = False
    threading.Timer(1.0, s.stop.set).start()
    try:
        rc = s.run()
    finally:
        s.shutdown()
    assert rc == msc.EXIT_INT and len(ms.cancelled) == 1


def test_sortie_cancels_duplicate_accepted_goal(world, capsys):
    """Attempt 1 arrives late and is accepted after attempt 2 was adopted."""
    FakeServer(world, TK, TakeoffTask, ["accept"])
    ms = FakeServer(world, MS, SearchMission, ["drop", "accept"], run_s=1.0, busy_rejects=False)
    orig = ms.handle

    def handle(gid, goal, fb):
        r = orig(gid, goal, fb)
        if len(ms.received) == 1:   # adopted goal is running: now the "late" first request shows up
            threading.Timer(0.2, lambda: (ms.goals.__setitem__(world.log[-2][1], msc.STATUS_EXECUTING),
                                          ms._status())).start()
        return r
    ms.handle = handle
    rc, _ = run_sortie(world)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "duplicate attempt" in out and len(ms.cancelled) == 1
