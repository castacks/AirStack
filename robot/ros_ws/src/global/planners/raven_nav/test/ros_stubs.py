"""Minimal stand-ins for the ROS 2 modules `raven_nav_node` imports.

Only used by `test_node_smoke.py`, and only on a machine with NO real ROS
installed — enough of rclpy, the message packages and `sensor_msgs_py` to
construct the node, drive its callbacks and its tick, and record everything it
publishes. It is a stub, not a simulator: message classes are plain attribute
bags, so this checks the node's own logic and plumbing, not the wire format.
The real thing is `test/integration/test_node_roundtrip.py`.
"""
from __future__ import annotations

import sys
import types
from dataclasses import dataclass, field
from typing import Any, Dict, List


def ros_is_real() -> bool:
    try:
        import rclpy  # noqa: F401,PLC0415
        return True
    except Exception:      # noqa: BLE001
        return False


# ── message bags ────────────────────────────────────────────────────────────
def _bag(name, **fields):
    def __init__(self, **kw):
        for k, v in fields.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)
    return type(name, (), {'__init__': __init__, '__repr__':
                           lambda s: f'<{name} {s.__dict__}>'})


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=0.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Pose:
    def __init__(self):
        self.position = _Vec()
        self.orientation = _Vec(w=1.0)


class _Header:
    def __init__(self):
        self.stamp = 0.0
        self.frame_id = ''


class _PoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()


class _Path:
    def __init__(self):
        self.header = _Header()
        self.poses: List[_PoseStamped] = []


class _Marker:
    ARROW = 0
    CUBE = 1
    ADD = 0
    DELETE = 2

    def __init__(self):
        self.header = _Header()
        self.ns = ''
        self.id = 0
        self.type = 0
        self.action = 0
        self.points = []
        self.pose = _Pose()
        self.scale = _Vec()
        self.color = _Vec()
        self.lifetime = types.SimpleNamespace(sec=0)


class _MarkerArray:
    def __init__(self):
        self.markers = []


class _PointField:
    FLOAT32 = 7

    def __init__(self, name='', offset=0, datatype=7, count=1):
        self.name, self.offset, self.datatype, self.count = \
            name, offset, datatype, count


class _Cloud:
    """What our stub `point_cloud2.create_cloud` returns / consumes."""

    def __init__(self, header=None, fields=(), points=()):
        self.header = header
        self.fields = list(fields)
        self.points = [list(p) for p in points]


def _create_cloud(header, fields, points):
    return _Cloud(header, fields, points)


def _read_points(msg, field_names=None, skip_nans=False):
    names = [f.name for f in msg.fields]
    idx = [names.index(n) for n in (field_names or names)]
    return [[row[i] for i in idx] for row in msg.points]


# ── rclpy ───────────────────────────────────────────────────────────────────
@dataclass
class Recorder:
    published: Dict[str, List[Any]] = field(default_factory=dict)
    logs: List[str] = field(default_factory=list)
    timers: List[Any] = field(default_factory=list)
    subscriptions: Dict[str, Any] = field(default_factory=dict)
    topics: List[str] = field(default_factory=list)

    def last(self, topic):
        msgs = self.published.get(topic) or []
        return msgs[-1] if msgs else None


RECORDER = Recorder()


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        RECORDER.published.setdefault(self.topic, []).append(msg)

    def get_subscription_count(self):
        return 1


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


class _Node:
    def __init__(self, name, context=None, cli_args=None,
                 use_global_arguments=True, **_kw):
        self._name = name
        self._context = context
        self._cli_args = cli_args
        self._use_global_arguments = use_global_arguments
        self._params = {}
        self._clock = _Clock()

    def declare_parameter(self, name, default):
        self._params[name] = default
        return types.SimpleNamespace(value=default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def has_parameter(self, name):
        return name in self._params

    def create_publisher(self, _type, topic, _qos):
        return _Publisher(topic)

    def create_subscription(self, _type, topic, cb, _qos):
        RECORDER.subscriptions[topic] = cb
        return object()

    def create_timer(self, period, cb):
        RECORDER.timers.append((period, cb))
        return object()

    def add_on_set_parameters_callback(self, cb):
        self._on_set = cb

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return self._clock

    def get_topic_names_and_types(self):
        return [(t, []) for t in RECORDER.topics]

    def destroy_node(self):
        pass


def install(monkeypatch=None):
    """Put the stubs into sys.modules. Returns the shared Recorder."""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    rclpy = mod('rclpy', init=lambda *a, **k: None,
                shutdown=lambda *a, **k: None, spin=lambda *a, **k: None,
                ok=lambda *a, **k: True)
    mod('rclpy.node', Node=_Node)
    rclpy.node = sys.modules['rclpy.node']

    # `main()` catches ExternalShutdownException alongside KeyboardInterrupt
    # (rclpy's SIGINT handler tears the default context down before spin()
    # returns). The stub `rclpy` is a plain module, not a package, so a
    # submodule import only resolves if it is in sys.modules already.
    class ExternalShutdownException(Exception):
        pass

    mod('rclpy.executors', ExternalShutdownException=ExternalShutdownException,
        SingleThreadedExecutor=object, MultiThreadedExecutor=object)
    rclpy.executors = sys.modules['rclpy.executors']

    class _Enum:
        def __getattr__(self, item):
            return item
    mod('rclpy.qos', QoSProfile=lambda **kw: kw,
        ReliabilityPolicy=_Enum(), DurabilityPolicy=_Enum(),
        HistoryPolicy=_Enum())
    rclpy.qos = sys.modules['rclpy.qos']

    mod('rcl_interfaces')
    mod('rcl_interfaces.msg',
        SetParametersResult=_bag('SetParametersResult', successful=True))
    mod('geometry_msgs')
    mod('geometry_msgs.msg', PolygonStamped=_bag('PolygonStamped'),
        PoseStamped=_PoseStamped, Point=_Vec, Point32=_Vec)
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=_bag('Odometry'), Path=_Path)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', NavSatFix=_bag('NavSatFix'), Image=_bag('Image'),
        PointCloud2=_Cloud, PointField=_PointField)
    mod('sensor_msgs_py')
    mod('sensor_msgs_py.point_cloud2', create_cloud=_create_cloud,
        read_points=_read_points)
    sys.modules['sensor_msgs_py'].point_cloud2 = \
        sys.modules['sensor_msgs_py.point_cloud2']
    mod('std_msgs')
    mod('std_msgs.msg', String=_bag('String', data=''),
        Bool=_bag('Bool', data=False), Empty=_bag('Empty'),
        Header=_Header, ColorRGBA=_bag('ColorRGBA', r=0.0, g=0.0, b=0.0, a=1.0))
    mod('visualization_msgs')
    mod('visualization_msgs.msg', Marker=_Marker, MarkerArray=_MarkerArray)
    mod('coordination_bringup')
    mod('coordination_bringup.frame_utils',
        gps_to_enu=lambda lat, lon, alt: (1000.0, 2000.0, float(alt)),
        dir_to_quat=lambda *a: (0.0, 0.0, 0.0, 1.0))
    mod('coordination_msgs')
    mod('coordination_msgs.msg',
        CoverageGrid=_bag('CoverageGrid', resolution=0.0, origin_x=0.0,
                          origin_y=0.0, width=0, height=0, data=b''))
    RECORDER.published.clear()
    RECORDER.logs.clear()
    RECORDER.timers.clear()
    RECORDER.subscriptions.clear()
    RECORDER.topics.clear()
    return RECORDER


def set_clock(seconds: float) -> None:
    _Clock.seconds = float(seconds)


def make_cloud(fields, rows):
    return _Cloud(_Header(), [_PointField(name=n) for n in fields], rows)
