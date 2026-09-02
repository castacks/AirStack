"""Minimal stand-ins for the ROS 2 modules `native_setpoint_node` imports.

Same idea as raven_nav/test/ros_stubs.py: enough of rclpy and the message
packages to CONSTRUCT the node, drive its callbacks and its timer, and record
everything it published and every service call it made. It is a stub, not a
simulator — message classes are attribute bags, so this checks the node's own
wiring and ordering, not the wire format. Anything that has to be exact on the
wire (the type_mask, the frame constant) is pinned in test_native_setpoint.py
against real values instead.

Skips itself where a real ROS is installed, so it can never shadow whatever
runs in the container.
"""
from __future__ import annotations

import sys
import types
from dataclasses import dataclass, field
from typing import Any, Dict, List

# The PositionTarget constants the node asserts against at construction. Real
# values, deliberately: a stub that agreed with native_setpoint.py by
# construction would make the guard untestable.
POSITION_TARGET_CONSTANTS = {
    'FRAME_LOCAL_NED': 1, 'FRAME_LOCAL_OFFSET_NED': 7, 'FRAME_BODY_NED': 8,
    'FRAME_BODY_OFFSET_NED': 9, 'IGNORE_PX': 1, 'IGNORE_PY': 2,
    'IGNORE_PZ': 4, 'IGNORE_VX': 8, 'IGNORE_VY': 16, 'IGNORE_VZ': 32,
    'IGNORE_AFX': 64, 'IGNORE_AFY': 128, 'IGNORE_AFZ': 256, 'FORCE': 512,
    'IGNORE_YAW': 1024, 'IGNORE_YAW_RATE': 2048,
}


def ros_is_real() -> bool:
    try:
        import rclpy  # noqa: F401,PLC0415
        return True
    except Exception:      # noqa: BLE001
        return False


class Vec3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Quat:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class Header:
    def __init__(self):
        self.stamp = 0
        self.frame_id = ''


class PositionTarget:
    def __init__(self):
        self.header = Header()
        self.coordinate_frame = 0
        self.type_mask = 0
        self.position = Vec3()
        self.velocity = Vec3()
        self.acceleration_or_force = Vec3()
        self.yaw = 0.0
        self.yaw_rate = 0.0


for _name, _value in POSITION_TARGET_CONSTANTS.items():
    setattr(PositionTarget, _name, _value)


class Goal:
    def __init__(self, p=(0., 0., 0.), v=(0., 0., 0.), a=(0., 0., 0.),
                 yaw=0.0, dyaw=0.0, frame_id='map'):
        self.header = Header()
        self.header.frame_id = frame_id
        self.p, self.v, self.a, self.j = (Vec3(*p), Vec3(*v), Vec3(*a), Vec3())
        self.yaw, self.dyaw = yaw, dyaw


class Odometry:
    def __init__(self, xyz=(0., 0., 0.), quat=(0., 0., 0., 1.)):
        self.header = Header()
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=Vec3(*xyz),
                                       orientation=Quat(*quat)))
        self.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=Vec3(), angular=Vec3()))


class Bool:
    def __init__(self, data=False):
        self.data = data


class RollPitchYawrateThrust:
    def __init__(self):
        self.roll = self.pitch = self.yaw_rate = 0.0
        self.thrust = Vec3()


class TrajectoryModeRequest:
    PAUSE, ROBOT_POSE, TRACK, ADD_SEGMENT, REWIND = 0, 1, 2, 3, 4

    def __init__(self):
        self.mode = 0


class TrajectoryMode:
    Request = TrajectoryModeRequest


class ParameterType:
    PARAMETER_BOOL = 1
    PARAMETER_INTEGER = 2
    PARAMETER_DOUBLE = 3
    PARAMETER_STRING = 4


class ParameterValue:
    def __init__(self, type=0, bool_value=False, **kw):   # noqa: A002
        self.type = type
        self.bool_value = bool_value
        for k, v in kw.items():
            setattr(self, k, v)


class Parameter:
    def __init__(self, name='', value=None):
        self.name, self.value = name, value


class SetParametersRequest:
    def __init__(self):
        self.parameters = []


class SetParameters:
    Request = SetParametersRequest


# ── recording harness ───────────────────────────────────────────────────────

class Future:
    """A future whose completion the TEST decides, so service ordering and
    the "still in flight" window are both observable."""

    def __init__(self):
        self._cbs = []
        self._result = None
        self.done = False

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self.done:
            cb(self)

    def result(self):
        if isinstance(self._result, Exception):
            raise self._result
        return self._result

    def finish(self, result):
        self._result = result
        self.done = True
        for cb in list(self._cbs):
            cb(self)


@dataclass
class ServiceCall:
    name: str
    request: Any
    future: Future


@dataclass
class Recorder:
    published: Dict[str, List[Any]] = field(default_factory=dict)
    calls: List[ServiceCall] = field(default_factory=list)
    logs: List[str] = field(default_factory=list)
    timers: List[Any] = field(default_factory=list)
    subscriptions: Dict[str, Any] = field(default_factory=dict)
    #: service names service_is_ready() should answer True for
    ready_services: set = field(default_factory=set)

    def last(self, topic):
        msgs = self.published.get(topic) or []
        return msgs[-1] if msgs else None

    def calls_to(self, name):
        return [c for c in self.calls if c.name == name]

    def clear_published(self):
        self.published.clear()


RECORDER = Recorder()


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        RECORDER.published.setdefault(self.topic, []).append(msg)


class _Client:
    def __init__(self, name):
        self.srv_name = name

    def service_is_ready(self):
        return self.srv_name in RECORDER.ready_services

    def call_async(self, req):
        fut = Future()
        RECORDER.calls.append(ServiceCall(self.srv_name, req, fut))
        return fut


class _Logger:
    def _log(self, level, msg, **_kw):
        RECORDER.logs.append(f'{level}: {msg}')
    info = lambda self, m, **k: self._log('INFO', m)      # noqa: E731
    warn = lambda self, m, **k: self._log('WARN', m)      # noqa: E731
    warning = lambda self, m, **k: self._log('WARN', m)   # noqa: E731
    error = lambda self, m, **k: self._log('ERROR', m)    # noqa: E731
    debug = lambda self, m, **k: self._log('DEBUG', m)    # noqa: E731


class _Time:
    def __init__(self, seconds=0.0):
        self.nanoseconds = int(seconds * 1e9)

    def to_msg(self):
        return self.nanoseconds


class _Clock:
    seconds = 0.0

    def now(self):
        return _Time(_Clock.seconds)


def set_clock(seconds: float) -> None:
    _Clock.seconds = float(seconds)


class _Node:
    def __init__(self, name, **_kw):
        self._name = name
        self._params = {}
        self._clock = _Clock()

    def declare_parameter(self, name, default):
        self._params.setdefault(name, default)
        return types.SimpleNamespace(value=self._params[name])

    def set_stub_parameter(self, name, value):
        self._params[name] = value

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_publisher(self, _type, topic, _qos):
        return _Publisher(topic)

    def create_subscription(self, _type, topic, cb, _qos, callback_group=None):
        RECORDER.subscriptions[topic] = cb
        return object()

    def create_client(self, _type, name, callback_group=None):
        return _Client(name)

    def create_timer(self, period, cb, callback_group=None):
        RECORDER.timers.append((period, cb))
        return object()

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


def install():
    """Put the stubs into sys.modules. Returns the shared Recorder."""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    rclpy = mod('rclpy', init=lambda *a, **k: None,
                shutdown=lambda *a, **k: None, spin=lambda *a, **k: None,
                spin_until_future_complete=lambda *a, **k: None,
                ok=lambda *a, **k: True)
    mod('rclpy.node', Node=_Node)
    rclpy.node = sys.modules['rclpy.node']
    mod('rclpy.callback_groups', ReentrantCallbackGroup=lambda: object(),
        MutuallyExclusiveCallbackGroup=lambda: object())
    rclpy.callback_groups = sys.modules['rclpy.callback_groups']
    mod('rclpy.executors', MultiThreadedExecutor=object,
        SingleThreadedExecutor=object)
    rclpy.executors = sys.modules['rclpy.executors']

    class _Enum:
        def __getattr__(self, item):
            return item
    mod('rclpy.qos', QoSProfile=lambda **kw: kw,
        QoSReliabilityPolicy=_Enum(), QoSDurabilityPolicy=_Enum(),
        QoSHistoryPolicy=_Enum())
    rclpy.qos = sys.modules['rclpy.qos']

    mod('airstack_msgs')
    mod('airstack_msgs.srv', TrajectoryMode=TrajectoryMode)
    mod('dynus_interfaces')
    mod('dynus_interfaces.msg', Goal=Goal)
    mod('mav_msgs')
    mod('mav_msgs.msg', RollPitchYawrateThrust=RollPitchYawrateThrust)
    mod('mavros_msgs')
    mod('mavros_msgs.msg', PositionTarget=PositionTarget)
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=Odometry)
    mod('rcl_interfaces')
    mod('rcl_interfaces.msg', Parameter=Parameter,
        ParameterType=ParameterType, ParameterValue=ParameterValue)
    mod('rcl_interfaces.srv', SetParameters=SetParameters)
    mod('std_msgs')
    mod('std_msgs.msg', Bool=Bool)

    RECORDER.published.clear()
    RECORDER.calls.clear()
    RECORDER.logs.clear()
    RECORDER.timers.clear()
    RECORDER.subscriptions.clear()
    RECORDER.ready_services.clear()
    set_clock(0.0)
    return RECORDER


def set_parameters_response(successful=True):
    """What a real rcl_interfaces/SetParameters response looks like to the
    node: `.results` of objects with `.successful`."""
    return types.SimpleNamespace(
        results=[types.SimpleNamespace(successful=successful)])
