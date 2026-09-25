# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Minimal hermetic stand-ins for rclpy + the message packages the MTL nodes use.

Unit tests must not need a ROS install (tests/colcon_unit_test_packages.yaml runs
them on a plain ubuntu runner), so the node modules are imported against these
fakes: a controllable clock, publishers that record, subscriptions/timers whose
callbacks the test calls directly. Install with :func:`install` via
``monkeypatch`` so nothing leaks into other test modules.
"""

from __future__ import annotations

import types


class Msg:
    """Auto-vivifying message: unknown attributes become nested Msg objects."""

    _lists: tuple[str, ...] = ()

    def __init__(self, **kw):
        for name in self._lists:
            object.__setattr__(self, name, [])
        for k, v in kw.items():
            object.__setattr__(self, k, v)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        v = Msg()
        object.__setattr__(self, name, v)
        return v


def _msg(name, lists=(), **consts):
    cls = type(name, (Msg,), dict(consts, _lists=tuple(lists)))
    return cls


class Quaternion(Msg):
    def __init__(self, **kw):
        super().__init__(**{"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, **kw})


class Vector3(Msg):
    def __init__(self, **kw):
        super().__init__(**{"x": 0.0, "y": 0.0, "z": 0.0, **kw})


Point = Vector3
Point32 = Vector3


class Polygon(Msg):
    _lists = ("points",)


class PolygonStamped(Msg):
    def __init__(self, **kw):
        super().__init__(**kw)
        object.__setattr__(self, "polygon", Polygon())


# ---------------------------------------------------------------- time / clock
class Duration:
    def __init__(self, ns):
        self.nanoseconds = int(ns)


class Time:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = int(nanoseconds)

    def __sub__(self, other):
        return Duration(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return Msg(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9, _ns=self.nanoseconds)

    @staticmethod
    def from_msg(m):
        ns = getattr(m, "_ns", None)
        if isinstance(ns, int):
            return Time(nanoseconds=ns)
        sec = m.sec if isinstance(m.sec, int) else 0
        nsec = m.nanosec if isinstance(m.nanosec, int) else 0
        return Time(nanoseconds=sec * 10**9 + nsec)


class SimClock:
    t_ns = 0

    def now(self):
        return Time(nanoseconds=SimClock.t_ns)

    @staticmethod
    def advance(seconds):
        SimClock.t_ns += int(round(seconds * 1e9))


# ---------------------------------------------------------------- node
class _Param:
    def __init__(self, v):
        self.value = v


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    warning = warn

    def error(self, m):
        self.lines.append(("error", m))


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class Client:
    def __init__(self, name):
        self.name = name
        self.calls = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        self.calls.append(req)


class Node:
    overrides: dict = {}

    def __init__(self, name):
        self.name = name
        self.subs, self.pubs, self.timers, self.clients = {}, {}, [], {}
        self._logger = _Logger()
        self._clock = SimClock()

    def declare_parameter(self, name, default):
        return _Param(Node.overrides.get(name, default))

    def create_subscription(self, _type, topic, cb, _qos):
        self.subs[topic] = cb
        return object()

    def create_publisher(self, _type, topic, _qos):
        self.pubs[topic] = Publisher(topic)
        return self.pubs[topic]

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def create_client(self, _type, name):
        self.clients[name] = Client(name)
        return self.clients[name]

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


class _Broadcaster:
    def __init__(self, _node):
        self.sent = []

    def sendTransform(self, t):  # noqa: N802 (tf2_ros API)
        self.sent.append(t)


def install(monkeypatch):
    """Register the fakes in sys.modules (auto-undone by pytest's monkeypatch)."""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        monkeypatch.setitem(__import__("sys").modules, name, m)
        return m

    qos = mod("rclpy.qos", QoSProfile=lambda **kw: kw, ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1),
              DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1), qos_profile_sensor_data="sensor")
    node = mod("rclpy.node", Node=Node)
    execs = mod("rclpy.executors", ExternalShutdownException=RuntimeError)
    tmod = mod("rclpy.time", Time=Time)
    mod("rclpy", node=node, qos=qos, executors=execs, time=tmod, init=lambda **k: None,
        spin=lambda n: None, ok=lambda: False, shutdown=lambda: None)

    status_consts = dict(IDLE=0, INGRESS=1, SEARCH=2, COMPLETE=3, ABORTED=4)
    mod("mtl_msgs"), mod("mtl_msgs.msg", SearchPlan=_msg("SearchPlan"),
                          FollowerStatus=_msg("FollowerStatus", **status_consts))
    mod("airstack_msgs"), mod("airstack_msgs.msg", Odometry=_msg("TrackingPoint"))

    class _TMReq(Msg):
        PAUSE, ROBOT_POSE, TRACK, ADD_SEGMENT, REWIND = 0, 1, 2, 3, 4

    mod("airstack_msgs.srv", TrajectoryMode=types.SimpleNamespace(Request=_TMReq))
    mod("geometry_msgs"), mod("geometry_msgs.msg", PointStamped=_msg("PointStamped"),
                               TransformStamped=_msg("TransformStamped"), Vector3=Vector3, Point=Point,
                               Point32=Point32, Quaternion=Quaternion, PolygonStamped=PolygonStamped)
    mod("nav_msgs"), mod("nav_msgs.msg", Odometry=_msg("Odometry"))
    mod("std_msgs"), mod("std_msgs.msg", Bool=_msg("Bool"), Empty=_msg("Empty"), String=_msg("String"))
    mod("visualization_msgs"), mod("visualization_msgs.msg",
                                   Marker=_msg("Marker", CYLINDER=3, ADD=0),
                                   MarkerArray=_msg("MarkerArray", ["markers"]))
    mod("tf2_ros", TransformBroadcaster=_Broadcaster, StaticTransformBroadcaster=_Broadcaster)
    SimClock.t_ns = 0
    Node.overrides = {}


def odom(x, y, z, yaw=0.0, frame="map"):
    import math
    o = Msg()
    o.header.frame_id = frame
    o.pose.pose.position = Vector3(x=x, y=y, z=z)
    o.pose.pose.orientation = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    o.twist.twist.linear = Vector3()
    return o
