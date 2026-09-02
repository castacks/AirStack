"""END-TO-END smoke: fake robots -> shared RayFronts -> real raven_nav.

This is the only test in the tree that runs the REAL processes of the
"single-agent RAVEN + shared off-board RayFronts" build against each other.
Nothing is mocked except the robots:

    fake robot_1 (ROS_DOMAIN_ID=1)  ─┐
    fake robot_2 (ROS_DOMAIN_ID=2)  ─┤ rgb / depth / camera_info / odometry /
                                     │ NavSatFix, published from timers
                                     ▼
        python3 -m rayfronts.encoder_server        encoder=dummy, device=cpu
                       ▲ unix socket, transport=cpu
                       │
        python3 -m rayfronts.multi_robot_mapping_server
                --config-name shared_humans --config-dir <repo>/common/rayfronts_configs
                dataset.robot_ids=[1,2] encoder=client
                       │  ONE SemanticRayFrontiersMap, GPS-anchored per robot
                       ▼
    /robot_i/rayfronts/status                 (both domains)
    /robot_i/rayfronts/msg_serv/voxels_sim/*  (both domains, robot-local frame)
    /robot_i/rayfronts/msg_serv/rays_sim/*
    /robot_i/rayfronts/msg_serv/frontiers
                       │
                       ▼
        python3 -c 'from raven_nav.raven_nav_node import main; main()'
                ROBOT_NAME=robot_1  ROS_DOMAIN_ID=1  (domain == robot id is
                what makes raven's `{rf}` prefix land on the shared server)
                       │
                       ▼
    /robot_1/navigation_mode, /robot_1/raven_nav/discoveries, /robot_1/global_plan

Everything runs on CPU inside a throwaway robot container; no Isaac Sim, no
GPU.  Run it with::

    scripts/raven_rayfronts_tests.sh --e2e

WHAT THE FAKE WORLD IS
----------------------
Each robot flies straight down +x_flu at 1 m/s at 3 m AGL looking forward.  Its
camera sees a 128x128 pinhole frame (fx=fy=64 -> 90 deg FOV):

* depth  — a flat 5 m plane, except the top quarter of the frame which is
  ``inf`` ("sky").  The plane gives occupied voxels, the free space in front of
  it gives empty voxels, and the boundary of the observed frustum gives
  FRONTIERS.  The sky band is what lets the mapper cast semantic RAYS at all.
* rgb    — a background colour with a 64x64 centred patch of a "person"
  colour.  ``DummyEncoder`` is a fixed linear map, so the colour whose language
  embedding is closest to the prompt ``person`` (and the one closest to
  ``road``) is SOLVED FOR at test time rather than guessed — see
  :func:`_solve_colours`.  ``compute_cos_sim`` softmaxes ``100 * cos``, so the
  winning label takes essentially the whole probability mass and the patch
  reads as a confident person cluster.

Robot 2 is spawned 120 m east / 80 m south of robot 1; its NavSatFix is
generated so that ``gps_to_enu(fix) - odom`` is exactly that offset, which is
how the anchoring assertion has a ground truth.

WHAT IS NOT COVERED
-------------------
A real encoder (radseg), CUDA-IPC transport, real imagery, real flight
dynamics, the PX4/mavros side, and the semantic_search_task spawner.  Those
need Isaac Sim and a GPU.
"""
import json
import math
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np
import pytest

pytestmark = pytest.mark.e2e

rclpy = pytest.importorskip('rclpy', reason='e2e needs ROS 2')
torch = pytest.importorskip('torch', reason='e2e needs torch')
pytest.importorskip('rayfronts', reason='e2e needs the rayfronts package')

from nav_msgs.msg import Odometry, Path                            # noqa: E402
from rclpy.executors import SingleThreadedExecutor                 # noqa: E402
from rclpy.node import Node                                        # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2                            # noqa: E402
from std_msgs.msg import Header, String                            # noqa: E402

from rayfronts import multi_robot_common as mrc                    # noqa: E402
from rayfronts.image_encoders import DummyEncoder                  # noqa: E402

try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:                                                # pragma: no cover
    SignalHandlerOptions = None

# ── layout ──────────────────────────────────────────────────────────────────
REPO = pathlib.Path(__file__).resolve().parents[8]
RAYFRONTS_DIR = REPO / 'common' / 'rayfronts'
RAYFRONTS_CONFIGS = REPO / 'common' / 'rayfronts_configs'
RAVEN_PKG_DIR = pathlib.Path(__file__).resolve().parents[2]
BACKGROUND_TXT = RAYFRONTS_CONFIGS / 'background_humans.txt'
CSRC_BUILD = '/opt/rayfronts/rayfronts/csrc/build'

# ── the fake world ──────────────────────────────────────────────────────────
# domain == robot id: raven derives `{rf}` from ROS_DOMAIN_ID, so the two must
# agree or it subscribes to a prefix nobody publishes (plan section 2.1).
ROBOTS = (1, 2)
SPAWN_XY = {1: (0.0, 0.0), 2: (120.0, -80.0)}
IMG_W = IMG_H = 128
FX = FY = 64.0                 # 90 deg horizontal FOV
PLANE_DEPTH_M = 5.0
SKY_ROWS = 32                  # top quarter of the depth frame is `inf`
PATCH = slice(32, 96)          # 64x64 centred "person" patch
FRAME_HZ = 10.0
SPEED_MPS = 1.0
ALT_AGL = 3.0

TARGET_LABEL = 'person'

# ── QoS (must match the producers/consumers exactly) ────────────────────────
RELIABLE_10 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
BEST_EFFORT_1 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
BEST_EFFORT_10 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
# rayfronts' status/guiding topics and raven's search_area are TRANSIENT_LOCAL.
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
# Ros2MessagingService publishes RELIABLE depth 5 and subscribes to
# new_text_query RELIABLE depth 5.
RELIABLE_5 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)


# --------------------------------------------------------------------------- #
# rclpy plumbing (private context per domain, exactly like
# common/rayfronts/tests/ros_helpers.py)
# --------------------------------------------------------------------------- #

def _make_context(domain_id):
    ctx = rclpy.Context()
    kwargs = dict(context=ctx, domain_id=int(domain_id))
    if SignalHandlerOptions is not None:
        kwargs['signal_handler_options'] = SignalHandlerOptions.NO
    rclpy.init(**kwargs)
    return ctx


class _SpinningNode:
    """A node on its own private context, spun by its own thread."""

    def __init__(self, name, domain_id):
        self.context = _make_context(domain_id)
        self.node = Node(name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:                                 # noqa: BLE001
            pass

    def shutdown(self):
        for fn in (lambda: self.executor.remove_node(self.node),
                   self.node.destroy_node,
                   self.context.try_shutdown):
            try:
                fn()
            except Exception:                             # noqa: BLE001
                pass


def _header(t, frame='map'):
    h = Header()
    h.stamp.sec = int(t)
    h.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1_000_000_000
    h.frame_id = frame
    return h


# --------------------------------------------------------------------------- #
# Choosing the scene colours so DummyEncoder actually says "person"
# --------------------------------------------------------------------------- #

def _encoder():
    """A local DummyEncoder identical to the one `encoder=dummy` will serve."""
    return DummyEncoder(device='cpu', feat_dim=16, lang_dim=8, patch_size=16,
                        seed=17)


def _solve_tiles(labels, wanted=(TARGET_LABEL, 'road')):
    """Solve for 16x16 RGB TILES whose DummyEncoder embedding names a label.

    A flat colour is not enough.  ``DummyEncoder``'s language space is 8-D with
    random per-label directions, while the set of embeddings reachable by a
    CONSTANT colour is the image of the unit RGB cube under one 8x3 matrix — a
    3-D cone that, measured, contains no point where ``person`` beats all 33
    background labels (an exhaustive 21^3 sweep of the cube returns margin
    0.0, i.e. nothing beats black).  The encoder is a 16x16 stride-16
    convolution though, so the real control surface is the whole 3x16x16 tile:
    768 free variables into a 16-D feature, wildly underdetermined.

    So: pick the feature ``f`` whose projection points along the label's text
    embedding (``pinv(proj) @ t``), pull back the minimum-norm tile that
    produces it (``pinv(W)``) around mid-grey, clip into [0, 1], quantise to
    uint8 — and scan the magnitude, keeping whichever scale actually maximises
    the cosine margin once clipping and quantisation have had their say.  Every
    candidate is scored by RUNNING the encoder, not by the linear model, so the
    returned margin is the real one.

    Returns:
      ``{label: (tile_3x16x16_float01, cosine_margin_over_runner_up)}``.  The
      tiles are what the ENCODER must see; `_FakeRobot._make_rgb` reverses the
      channels on the way out (the dataset flips them back).
    """
    enc = _encoder()
    labels = list(labels)
    T = torch.nn.functional.normalize(
        enc.encode_prompts(labels).double(), dim=-1)               # (L, 8)
    P = enc._proj.double()                                         # (8, 16)
    W = enc._w.double().reshape(enc.feat_dim, -1)                  # (16, 768)
    W_pinv = torch.linalg.pinv(W)
    P_pinv = torch.linalg.pinv(P)
    base = torch.full((W.shape[1],), 0.5, dtype=torch.float64)
    W_base = W @ base

    def score(tile):
        f = torch.nn.functional.conv2d(
            tile.unsqueeze(0).float(), enc._w, stride=enc.patch_size)
        lang = torch.nn.functional.normalize(
            (enc._proj @ f.reshape(-1)).double(), dim=-1)
        return T @ lang                                            # (L,)

    out = {}
    for label in wanted:
        i = labels.index(label)
        f_dir = P_pinv @ T[i]
        best = None
        for alpha in torch.linspace(0.1, 12.0, 120).tolist():
            x = (base + W_pinv @ (alpha * f_dir - W_base)).clamp(0.0, 1.0)
            # uint8 round trip: the tile travels as an 8-bit ROS Image.
            tile = torch.round(x.reshape(3, enc.patch_size, enc.patch_size)
                               * 255.0) / 255.0
            cos = score(tile)
            margin = float(cos[i] - torch.cat([cos[:i], cos[i + 1:]]).max())
            if best is None or margin > best[1]:
                best = (tile, margin)
        out[label] = best
    return out


# --------------------------------------------------------------------------- #
# Fake robot
# --------------------------------------------------------------------------- #

class _FakeRobot(_SpinningNode):
    """Publishes everything the shared mapper AND raven subscribe to."""

    def __init__(self, robot_id, spawn_xy, tile_target, tile_background):
        super().__init__(f'fake_robot_{robot_id}', robot_id)
        self.robot_id = robot_id
        self.name = f'robot_{robot_id}'
        self.spawn_xy = np.asarray(spawn_xy, dtype=float)
        self.frames = 0
        self._t0 = time.time()

        n, p = self.node, f'/robot_{robot_id}'
        self.info_pub = n.create_publisher(
            CameraInfo, f'{p}/sensors/front_stereo/left/camera_info',
            BEST_EFFORT_1)
        self.rgb_pub = n.create_publisher(
            Image, f'{p}/sensors/front_stereo/left/image_rect', RELIABLE_10)
        self.depth_pub = n.create_publisher(
            Image, f'{p}/sensors/front_stereo/left/depth_ground_truth',
            RELIABLE_10)
        # What rayfronts' MultiRobotRos2Subscriber reads...
        self.odom_pub = n.create_publisher(
            Odometry, f'{p}/odometry_conversion/odometry', RELIABLE_10)
        # ...and what raven_nav reads (the spawner remaps this in production).
        self.raven_odom_pub = n.create_publisher(
            Odometry, f'{p}/odometry', RELIABLE_10)
        # rayfronts anchors off mavros' fused fix, raven off the raw one.
        self.fix_pub = n.create_publisher(
            NavSatFix, f'{p}/interface/mavros/global_position/global',
            BEST_EFFORT_1)
        self.raven_fix_pub = n.create_publisher(
            NavSatFix, f'{p}/interface/mavros/global_position/raw/fix',
            BEST_EFFORT_10)

        self._rgb_bytes = self._make_rgb(tile_target, tile_background)
        self._depth_bytes = self._make_depth()

        n.create_timer(1.0 / FRAME_HZ, self._tick)
        n.create_timer(0.2, self._tick_info)

    # -- static payloads ----------------------------------------------------- #

    @staticmethod
    def encoder_image(tile_target, tile_background):
        """The (3, 128, 128) tensor the ENCODER will see, tiles laid out 8x8.

        128 / 16 = 8 conv cells per side and the patch spans cells 2..5, so
        every convolution window is one pure tile — no blending at the feature
        level, which is what makes the solved margin the margin the map sees.
        """
        def grid(tile, ny, nx):
            return tile.repeat(1, ny, nx)
        img = grid(tile_background, IMG_H // 16, IMG_W // 16)
        img = img.clone()
        img[:, PATCH, PATCH] = grid(tile_target,
                                    (PATCH.stop - PATCH.start) // 16,
                                    (PATCH.stop - PATCH.start) // 16)
        return img

    @classmethod
    def _make_rgb(cls, tile_target, tile_background):
        """`rgb8` bytes whose channels are REVERSED.

        `MultiRobotRos2Subscriber._decode` does ``bgr[..., (2, 1, 0)]`` on
        whatever it receives (a legacy assumption inherited from the
        single-robot Ros2Subscriber, which isaac-sim feeds BGRA). Publish the
        reverse so the encoder sees the tiles `_solve_tiles` solved for.
        """
        img = cls.encoder_image(tile_target, tile_background)   # (3, H, W)
        hwc = np.round(img.numpy().transpose(1, 2, 0) * 255.0).astype(np.uint8)
        return np.ascontiguousarray(hwc[:, :, ::-1]).tobytes()

    @staticmethod
    def _make_depth():
        arr = np.full((IMG_H, IMG_W), PLANE_DEPTH_M, dtype=np.float32)
        arr[:SKY_ROWS, :] = np.inf     # sky: out of range -> semantic rays
        return arr.tobytes()

    def camera_info(self, t):
        m = CameraInfo()
        m.header = _header(t)
        m.width, m.height = IMG_W, IMG_H
        m.k = [FX, 0.0, IMG_W / 2.0, 0.0, FY, IMG_H / 2.0, 0.0, 0.0, 1.0]
        return m

    # -- per-tick payloads --------------------------------------------------- #

    def odom_xyz(self):
        return np.array([SPEED_MPS * (self.frames / FRAME_HZ), 0.0, ALT_AGL])

    def _odometry(self, t, xyz):
        m = Odometry()
        m.header = _header(t)
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = float(xyz[0])
        m.pose.pose.position.y = float(xyz[1])
        m.pose.pose.position.z = float(xyz[2])
        m.pose.pose.orientation.w = 1.0
        return m

    def _image(self, t, data, encoding, step):
        m = Image()
        m.header = _header(t)
        m.height, m.width = IMG_H, IMG_W
        m.encoding = encoding
        m.is_bigendian = 0
        m.step = step
        m.data = data
        return m

    def _navsat(self, t, xyz):
        """A fix whose ENU is ``spawn + odom``, so boot_enu measures ``spawn``."""
        world = self.spawn_xy + xyz[:2]
        lat0, lon0, alt0 = (mrc.DEFAULT_ORIGIN_LAT, mrc.DEFAULT_ORIGIN_LON,
                            mrc.DEFAULT_ORIGIN_ALT)
        m = NavSatFix()
        m.header = _header(t)
        m.latitude = world[1] / 111320.0 + lat0
        m.longitude = (world[0] / (111320.0 * math.cos(math.radians(lat0)))
                       + lon0)
        m.altitude = alt0 + float(xyz[2])
        m.status.status = 0
        return m

    def _tick_info(self):
        self.info_pub.publish(self.camera_info(time.time()))

    def _tick(self):
        t = time.time()
        xyz = self.odom_xyz()
        odom = self._odometry(t, xyz)
        # Pose FIRST: the dataset's anchoring drops a GPS fix that arrives
        # before it has ever seen odometry.
        self.odom_pub.publish(odom)
        self.raven_odom_pub.publish(odom)
        self.rgb_pub.publish(
            self._image(t, self._rgb_bytes, 'rgb8', IMG_W * 3))
        self.depth_pub.publish(
            self._image(t, self._depth_bytes, '32FC1', IMG_W * 4))
        fix = self._navsat(t, xyz)
        self.fix_pub.publish(fix)
        self.raven_fix_pub.publish(fix)
        self.frames += 1


class _Sniffer(_SpinningNode):
    """Subscribes to a set of topics on one domain and keeps every message."""

    def __init__(self, name, domain_id, subs):
        super().__init__(name, domain_id)
        self.got = {}
        self._lock = threading.Lock()
        for topic, msg_type, qos in subs:
            self.got[topic] = []
            self.node.create_subscription(
                msg_type, topic,
                (lambda m, t=topic: self._on(t, m)), qos)

    def _on(self, topic, msg):
        with self._lock:
            self.got[topic].append(msg)

    def last(self, topic):
        with self._lock:
            msgs = self.got.get(topic) or []
            return msgs[-1] if msgs else None

    def count(self, topic):
        with self._lock:
            return len(self.got.get(topic) or [])

    def topics(self):
        return dict(self.node.get_topic_names_and_types())


# --------------------------------------------------------------------------- #
# Subprocess helper
# --------------------------------------------------------------------------- #

class _Proc:
    def __init__(self, name, cmd, cwd, env, log_path):
        self.name = name
        self.cmd = list(cmd)
        self.log_path = pathlib.Path(log_path)
        self._fh = open(self.log_path, 'wb')
        self.popen = subprocess.Popen(
            self.cmd, cwd=str(cwd), env=env,
            stdout=self._fh, stderr=subprocess.STDOUT,
            start_new_session=True)

    @property
    def alive(self):
        return self.popen.poll() is None

    def tail(self, n=80):
        try:
            self._fh.flush()
        except Exception:                                 # noqa: BLE001
            pass
        try:
            lines = self.log_path.read_text(errors='replace').splitlines()
        except OSError as exc:
            return f'<no log: {exc}>'
        return '\n'.join(lines[-n:])

    def stop(self, grace=8.0):
        """SIGINT -> SIGTERM -> SIGKILL the process group. Returns the rc."""
        if self.popen.poll() is None:
            # SIGINT first: the mapping server's handler transitions to IDLE
            # and shuts the dataset down cleanly.
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    os.killpg(os.getpgid(self.popen.pid), sig)
                except (ProcessLookupError, PermissionError, OSError):
                    break
                try:
                    self.popen.wait(timeout=grace / 2)
                    break
                except subprocess.TimeoutExpired:
                    continue
            if self.popen.poll() is None:
                try:
                    os.killpg(os.getpgid(self.popen.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
                try:
                    self.popen.wait(timeout=grace)
                except subprocess.TimeoutExpired:
                    pass
        try:
            self._fh.close()
        except Exception:                                 # noqa: BLE001
            pass
        return self.popen.returncode


def _subprocess_env(**extra):
    env = dict(os.environ)
    pp = [str(RAYFRONTS_DIR)]
    if os.path.isdir(CSRC_BUILD):
        pp.append(CSRC_BUILD)
    if env.get('PYTHONPATH'):
        pp.append(env['PYTHONPATH'])
    env['PYTHONPATH'] = os.pathsep.join(pp)
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    env['PYTHONUNBUFFERED'] = '1'
    env.pop('RAVEN_E2E', None)
    env.update({k: str(v) for k, v in extra.items()})
    return env


def _wait_for(pred, timeout, period=0.25):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if pred():
                return True
        except Exception:                                 # noqa: BLE001
            pass
        time.sleep(period)
    try:
        return bool(pred())
    except Exception:                                     # noqa: BLE001
        return False


# --------------------------------------------------------------------------- #
# The chain
# --------------------------------------------------------------------------- #

class Chain:
    """Everything the tests poke at, plus a failure reporter that dumps logs."""

    def __init__(self, tmp):
        self.tmp = pathlib.Path(tmp)
        self.procs = []
        self.nodes = []
        self.robots = {}
        self.sniffers = {}
        self.query_talker = None
        self.labels = []
        self.raven_results = self.tmp / 'raven_results'
        self.raven_results.mkdir(parents=True, exist_ok=True)

    # -- reporting ----------------------------------------------------------- #
    def report(self):
        out = []
        for r in self.robots.values():
            out.append(f'[{r.name}] frames published: {r.frames}')
        for p in self.procs:
            state = 'running' if p.alive else f'exited rc={p.popen.returncode}'
            out.append(f'\n===== {p.name} ({state}) — {p.log_path} =====\n'
                       f'{p.tail(120)}')
        return '\n'.join(out)

    def fail(self, msg):
        pytest.fail(f'{msg}\n\n{self.report()}', pytrace=False)

    def require(self, cond, msg):
        if not cond:
            self.fail(msg)

    # -- status -------------------------------------------------------------- #
    def status(self, robot_id):
        msg = self.sniffers[robot_id].last(f'/robot_{robot_id}/rayfronts/status')
        if msg is None:
            return None
        try:
            return json.loads(msg.data)
        except (ValueError, TypeError):
            return None

    def teardown(self):
        for p in reversed(self.procs):
            p.stop()
        for n in self.nodes:
            n.shutdown()


@pytest.fixture(scope='module')
def chain():
    # RAVEN_E2E_LOG_DIR keeps the subprocess logs somewhere that survives the
    # container (the harness points it at the mounted log dir); without it they
    # live in a tempdir that is removed on the way out.
    keep = os.environ.get('RAVEN_E2E_LOG_DIR', '').strip()
    if keep:
        tmp = pathlib.Path(keep)
        tmp.mkdir(parents=True, exist_ok=True)
    else:
        tmp = pathlib.Path(tempfile.mkdtemp(prefix='raven-e2e-'))
    print(f'[e2e] working dir / process logs: {tmp}')
    c = Chain(tmp)
    try:
        _bring_up(c)
        yield c
    finally:
        c.teardown()
        if not keep:
            shutil.rmtree(tmp, ignore_errors=True)


def _bring_up(c):
    assert BACKGROUND_TXT.exists(), f'missing {BACKGROUND_TXT}'
    background = [l.strip() for l in
                  BACKGROUND_TXT.read_text().splitlines() if l.strip()]
    c.labels = background + [TARGET_LABEL]

    tiles = _solve_tiles(c.labels)
    tile_p, margin_p = tiles[TARGET_LABEL]
    tile_b, margin_b = tiles['road']
    print(f'[e2e] solved scene tiles — person margin {margin_p:+.4f}, '
          f'background(road) margin {margin_b:+.4f}')
    # Precondition, not an assertion about the product: if the fake scene does
    # not read as a person there is nothing for the voxel/ray behaviours to do
    # and a later failure would be blamed on the wrong code.
    assert margin_p > 0.02, (
        f'the synthetic scene does not read as `{TARGET_LABEL}` to the dummy '
        f'encoder (margin {margin_p:.4f}); _solve_tiles needs widening')

    # 1. fake robots first — the mapping server BLOCKS in its constructor until
    #    a CameraInfo lands.
    for rid in ROBOTS:
        r = _FakeRobot(rid, SPAWN_XY[rid], tile_p, tile_b)
        c.robots[rid] = r
        c.nodes.append(r)

    # 2. sniffers, per domain. They must exist BEFORE the mapper starts
    #    publishing: Ros2MessagingService only publishes a layer that already
    #    has a subscriber (`_has_subscriber`).
    for rid in ROBOTS:
        pfx = f'/robot_{rid}/rayfronts/msg_serv'
        s = _Sniffer(f'e2e_sniffer_{rid}', rid, [
            (f'/robot_{rid}/rayfronts/status', String, LATCHED),
            (f'{pfx}/voxels_sim/all', PointCloud2, RELIABLE_10),
            (f'{pfx}/rays_sim/all', PointCloud2, RELIABLE_10),
            (f'{pfx}/frontiers', PointCloud2, RELIABLE_10),
        ])
        c.sniffers[rid] = s
        c.nodes.append(s)

    # 3. encoder server (dummy encoder, CPU, private socket).
    sock = c.tmp / 'encoder.sock'
    c.procs.append(_Proc(
        'encoder_server',
        [sys.executable, '-m', 'rayfronts.encoder_server',
         'encoder=dummy',
         f'encoder_server.socket={sock}',
         'encoder_server.device=cpu',
         'compile=false', 'amp=false',
         f'hydra.run.dir={c.tmp / "hydra_encoder"}'],
        cwd=RAYFRONTS_DIR, env=_subprocess_env(),
        log_path=c.tmp / 'encoder_server.log'))
    if not _wait_for(lambda: sock.exists(), 90.0):
        c.fail(f'encoder server never created {sock}')

    # 4. the shared mapping server.
    #
    # Deviations from the deployed command line, all for a CPU smoke:
    #   encoder=dummy on the server side (no radseg checkpoint / GPU)
    #   -> lang_dim is 8, so the config's PCA (out_dim 100) cannot be fitted:
    #      mapping.feat_compressor=null + querying.compressed=false.
    #   vis=null              — the visualiser adds 2 more nodes per domain and
    #                           publishes nothing this test reads.
    #   frame_skip=0, querying.period / messaging_publish_period=2
    #                         — the shipped values (10 / 10 / 10) would need
    #                           minutes of fake flight per query round.
    #   fronti_subsampling=3, fronti_subsampling_min_fronti=1
    #                         — the shipped 5 / 9 needs a much bigger swept
    #                           volume than 40 m of straight flight produces.
    #   128x128 frames        — the shipped 480x480 is pure CPU cost here.
    # Everything structural (topics, anchoring, frames, query bookkeeping,
    # status schema, the ClientEncoder transport) is the real thing.
    c.procs.append(_Proc(
        'mapping_server',
        [sys.executable, '-m', 'rayfronts.multi_robot_mapping_server',
         '--config-dir', str(RAYFRONTS_CONFIGS),
         '--config-name', 'shared_humans',
         'dataset.robot_ids=[1,2]',
         f'dataset.rgb_resolution=[{IMG_H},{IMG_W}]',
         f'dataset.depth_resolution=[{IMG_H},{IMG_W}]',
         'dataset.frame_skip=0',
         'dataset.sync_slop_s=0.05',
         'dataset.anchor_samples=5',
         'dataset.intrinsics_timeout_s=120',
         'encoder=client',
         f'encoder.socket={sock}',
         'encoder.transport=cpu',
         'encoder.connect_timeout_s=120',
         # `~vis` and not `vis=null`: hydra rejects a NoneType override of a
         # config GROUP ("Config group override must be a string or a list").
         # feat_compressor below is a VALUE inside the mapping node, so there
         # `=null` is the right form.
         '~vis',
         'mapping.feat_compressor=null',
         'querying.compressed=false',
         'querying.period=2',
         'messaging_publish_period=2',
         'mapping.fronti_subsampling=3',
         'mapping.fronti_subsampling_min_fronti=1',
         'status_period_s=0.5',
         'compile=false', 'amp=false',
         f'hydra.run.dir={c.tmp / "hydra_mapper"}'],
        cwd=RAYFRONTS_DIR, env=_subprocess_env(),
        log_path=c.tmp / 'mapping_server.log'))

    # 5. the text query, on robot_1's domain only — the shared server must
    #    make it a column for BOTH robots.
    c.query_talker = _SpinningNode('e2e_query_talker', 1)
    c.nodes.append(c.query_talker)
    c.query_pub = c.query_talker.node.create_publisher(
        String, '/robot_1/rayfronts/msg_serv/new_text_query', RELIABLE_5)


# --------------------------------------------------------------------------- #
# 1. anchoring + status
# --------------------------------------------------------------------------- #

def test_both_robots_anchor_and_status_reports_growing_frames(chain):
    """`/robot_i/rayfronts/status` on BOTH domains: anchored, frames climbing.

    This is the topic `semantic_search_task` gates the mission on in shared
    mode, so its schema and its liveness are the contract, not a detail.
    """
    ok = _wait_for(
        lambda: all(chain.status(r) is not None for r in ROBOTS), 180.0)
    chain.require(ok, 'no /robot_i/rayfronts/status on one of the domains')

    ok = _wait_for(
        lambda: all((chain.status(r) or {}).get('anchored') for r in ROBOTS),
        120.0)
    chain.require(ok, 'a robot never anchored: '
                      f'{ {r: chain.status(r) for r in ROBOTS} }')

    first = {r: chain.status(r)['frames_robot'] for r in ROBOTS}
    ok = _wait_for(
        lambda: all(chain.status(r)['frames_robot'] > first[r] + 3
                    for r in ROBOTS), 90.0)
    chain.require(ok, f'frames_robot did not grow past {first}: '
                      f'{ {r: chain.status(r) for r in ROBOTS} }')

    for r in ROBOTS:
        st = chain.status(r)
        assert tuple(st.keys()) == mrc.STATUS_KEYS, st
        assert st['robot'] == f'robot_{r}'
        assert st['domain'] == r
        assert st['frames_total'] >= st['frames_robot'] > 0
    # ONE map: both robots see the same total.
    assert chain.status(1)['frames_total'] > 0
    assert (chain.status(1)['frames_total'] >=
            chain.status(1)['frames_robot'] + 1)


def test_boot_enu_matches_the_spawn_offsets(chain):
    """The GPS anchor reproduces each robot's spawn offset within 1 m.

    This is what makes the shared map coherent: robot_2's odom origin is 120 m
    east / 80 m south of robot_1's, and nothing but NavSatFix + odometry tells
    the server so.
    """
    chain.require(
        _wait_for(lambda: all((chain.status(r) or {}).get('anchored')
                              for r in ROBOTS), 120.0),
        'robots not anchored')
    for r in ROBOTS:
        boot = chain.status(r)['boot_enu']
        assert boot is not None, f'robot_{r} anchored with boot_enu=None'
        want = SPAWN_XY[r]
        assert abs(boot[0] - want[0]) < 1.0, (r, boot, want)
        assert abs(boot[1] - want[1]) < 1.0, (r, boot, want)
        # z is an MSL-datum difference and must NOT be applied (plan 2.2).
        assert abs(boot[2]) < 1e-6, boot


# --------------------------------------------------------------------------- #
# 2. queries + per-robot map outputs
# --------------------------------------------------------------------------- #

def test_text_query_from_robot_1_becomes_a_shared_column(chain):
    """`person` published on robot_1's topic shows up in BOTH robots' status."""
    def _seen():
        for r in ROBOTS:
            st = chain.status(r)
            if not st or TARGET_LABEL not in st.get('queries', []):
                return False
        return True

    deadline = time.time() + 120.0
    while time.time() < deadline and not _seen():
        chain.query_pub.publish(String(data=TARGET_LABEL))
        time.sleep(1.0)
    chain.require(_seen(), 'the `person` query never reached both robots: '
                           f'{ {r: chain.status(r) for r in ROBOTS} }')

    st = chain.status(1)
    # The background vocabulary must be in there too, or compute_prob=True
    # makes every softmax score ~1.0 and no threshold means anything.
    missing = [l for l in chain.labels if l not in st['queries']]
    assert not missing, f'background labels missing from the query set: {missing}'
    assert st['queries'] == chain.status(2)['queries'], 'per-robot query drift'


def test_voxels_sim_all_is_published_into_both_domains(chain):
    """PointCloud2 with `sim_*` columns, per robot, in that robot's frame."""
    ok = _wait_for(
        lambda: all(chain.sniffers[r].count(
            f'/robot_{r}/rayfronts/msg_serv/voxels_sim/all') > 0
            for r in ROBOTS), 180.0)
    chain.require(ok, 'no voxels_sim/all on one of the domains')

    for r in ROBOTS:
        cloud = chain.sniffers[r].last(
            f'/robot_{r}/rayfronts/msg_serv/voxels_sim/all')
        names = [f.name for f in cloud.fields]
        assert names[:3] == ['x', 'y', 'z'], names
        sims = [n for n in names if n.startswith('sim_')]
        assert len(sims) == len(chain.status(r)['queries']), (names,
                                                              chain.status(r))
        assert cloud.width * cloud.height > 0
        assert cloud.header.frame_id == 'map'


def test_a_person_query_topic_exists_on_robot_1(chain):
    """`q{k}_person` — the per-query topic raven parses the column order out of.

    `MultiRobotRos2MessagingService._needs` creates one publisher per query
    column while deciding whether anything is subscribed, so the topic exists
    as soon as the shared map has run a query, subscriber or not.
    """
    def _match():
        want = f'_{mrc.sanitize_topic_name(TARGET_LABEL)}'
        hits = []
        for rid in ROBOTS:
            for name in chain.sniffers[rid].topics():
                if (name.startswith(f'/robot_{rid}/rayfronts/msg_serv/')
                        and '/q' in name and name.endswith(want)):
                    hits.append(name)
        return hits

    chain.require(_wait_for(lambda: bool(_match()), 120.0),
                  'no q{k}_person topic on either domain')
    print('[e2e] person query topics:', sorted(_match()))


def test_frontiers_reach_robot_1(chain):
    """The layer raven's Frontier behaviour needs to leave `idle`."""
    ok = _wait_for(
        lambda: chain.sniffers[1].count(
            '/robot_1/rayfronts/msg_serv/frontiers') > 0, 180.0)
    chain.require(ok, 'the shared map never published frontiers to robot_1 — '
                      'raven cannot leave idle without them')
    cloud = chain.sniffers[1].last('/robot_1/rayfronts/msg_serv/frontiers')
    assert [f.name for f in cloud.fields][:3] == ['x', 'y', 'z']
    assert cloud.width * cloud.height > 0


def _person_points(cloud, person_col):
    """(N, 4) of x, y, z, sim_person out of a `voxels_sim/all` cloud.

    Coordinates are RDF (what rayfronts publishes and raven converts): for a
    robot-local FLU point ``(fx, fy, fz)`` the cloud carries
    ``(-fy, -fz, fx)``, so column 2 is the robot's forward axis.
    """
    fields = ('x', 'y', 'z', f'sim_{person_col}')
    try:
        arr = point_cloud2.read_points_numpy(
            cloud, field_names=fields, skip_nans=True)
    except AttributeError:                                # pragma: no cover
        arr = np.array([list(p) for p in point_cloud2.read_points(
            cloud, field_names=fields, skip_nans=True)], dtype=np.float32)
    return np.asarray(arr, dtype=np.float64).reshape(-1, 4)


def test_the_map_is_shared_each_robot_sees_the_others_person(chain):
    """THE point of the whole build: one map, fanned out per robot, per frame.

    robot_2 is spawned 120 m east / 80 m south of robot_1 and never talks to
    it.  If the shared server is doing its job, robot_1's own `voxels_sim/all`
    carries a confident `person` cluster ~140 m down its OWN forward axis (its
    person at ~21 m, plus robot_2's at 120 + ~20 m re-expressed in robot_1's
    map frame) — and robot_2 symmetrically sees robot_1's at ~-99 m.  A frame
    bug, a missing boot_enu shift or a per-robot map would each break this and
    nothing else in this file would notice.
    """
    labels = chain.status(1)['queries']
    col = labels.index(TARGET_LABEL)

    def clusters(rid):
        cloud = chain.sniffers[rid].last(
            f'/robot_{rid}/rayfronts/msg_serv/voxels_sim/all')
        if cloud is None:
            return None
        pts = _person_points(cloud, col)
        hot = pts[pts[:, 3] > 0.9]
        return hot

    # robot_1 must see something ~140 m ahead and ~80 m to starboard (RDF
    # x = -y_flu, and robot_2 sits at y = -80).
    def peer_visible():
        hot = clusters(1)
        return hot is not None and np.any((hot[:, 2] > 100.0) &
                                          (hot[:, 0] > 60.0))

    chain.require(_wait_for(peer_visible, 120.0),
                  "robot_1's voxels_sim/all never carried robot_2's person — "
                  'the map is not shared, or boot_enu is not being applied')

    def own_visible():
        hot = clusters(2)
        # robot_1's person, in robot_2's frame: 21 - 120 = -99 m forward,
        # 0 - (-80) = +80 m north -> RDF x = -80.
        return hot is not None and np.any((hot[:, 2] < -50.0) &
                                          (hot[:, 0] < -60.0))

    chain.require(_wait_for(own_visible, 120.0),
                  "robot_2's voxels_sim/all never carried robot_1's person")

    for rid in ROBOTS:
        hot = clusters(rid)
        if hot is None or not len(hot):
            continue
        print(f'[e2e] robot_{rid}: {len(hot)} person voxels, '
              f'forward span [{hot[:, 2].min():.1f}, {hot[:, 2].max():.1f}] m')


# --------------------------------------------------------------------------- #
# 3. the real raven_nav against the shared map
# --------------------------------------------------------------------------- #

@pytest.fixture(scope='module')
def raven(chain):
    """Start the REAL raven_nav_node on robot_1's domain, once the map is live.

    `query_labels` is taken from the status topic rather than hard-coded:
    the background vocabulary's column order comes out of a python `set`, so it
    is only knowable at runtime, and raven scores the column whose INDEX
    matches its target label.  (raven also self-corrects from the
    `rays_sim/q{k}_*` topic names once rays exist — this just removes the
    dependency on rays ever being cast.)
    """
    chain.require(
        _wait_for(lambda: (chain.status(1) or {}).get('queries'), 180.0),
        'no query columns to start raven with')
    labels = chain.status(1)['queries']
    chain.require(TARGET_LABEL in labels,
                  f'`{TARGET_LABEL}` is not a column: {labels}')

    def _lit(xs):
        return '[' + ','.join("'" + x.replace("'", '') + "'" for x in xs) + ']'

    env = _subprocess_env(ROBOT_NAME='robot_1', ROS_DOMAIN_ID='1',
                          RAVEN_LVLM='false', RAYFRONTS_MODE='shared')
    p = _Proc(
        'raven_nav',
        [sys.executable, '-c',
         'from raven_nav.raven_nav_node import main; main()',
         '--ros-args',
         '-p', f'query_labels:={_lit(labels)}',
         '-p', f"target_labels:=['{TARGET_LABEL}']",
         '-p', 'timer_period:=0.25',
         '-p', 'min_altitude_agl:=1.5',
         '-p', 'max_altitude_agl:=30.0',
         '-p', 'lvlm_enabled:=false',
         '-p', 'nav_output_enabled:=true',
         '-p', 'voxel_min_cluster_size:=8',
         '-p', 'voxel_score_threshold:=0.6',
         '-p', 'score_threshold:=0.6',
         '-p', 'coverage_complete_threshold:=0.999',
         '-p', 'results_dump_period_s:=1.0',
         '-p', f'results_dir:={chain.raven_results}',
         ],
        cwd=RAVEN_PKG_DIR, env=env, log_path=chain.tmp / 'raven_nav.log')
    chain.procs.append(p)

    s = _Sniffer('e2e_raven_sniffer', 1, [
        ('/robot_1/navigation_mode', String, RELIABLE_10),
        ('/robot_1/raven_nav/discoveries', String, RELIABLE_10),
        ('/robot_1/raven_nav/confirmed_targets', String, RELIABLE_10),
        ('/robot_1/global_plan', Path, RELIABLE_10),
    ])
    chain.nodes.append(s)
    yield p, s


def test_raven_nav_leaves_idle_on_the_shared_map(chain, raven):
    """The pass bar: raven picks a real behaviour off the SHARED map.

    Frontier / Ray / Voxel are all acceptable — which one wins depends on what
    a flat synthetic scene produces, and the point of this assertion is that
    raven consumed the shared server's clouds at all.
    """
    proc, sniff = raven

    def _modes():
        with sniff._lock:
            return [m.data for m in sniff.got['/robot_1/navigation_mode']]

    ok = _wait_for(lambda: any(m != 'idle' for m in _modes()), 180.0)
    if not ok:
        chain.require(proc.alive, 'raven_nav exited')
    chain.require(ok, f'raven_nav never left idle; modes seen: {set(_modes())}')

    active = {m for m in _modes() if m != 'idle'}
    assert active & {'frontier', 'ray', 'voxel'}, active
    print('[e2e] navigation modes observed:', sorted(set(_modes())))


def test_raven_nav_publishes_valid_discoveries_json(chain, raven):
    _proc, sniff = raven
    ok = _wait_for(
        lambda: sniff.count('/robot_1/raven_nav/discoveries') > 0, 120.0)
    chain.require(ok, 'no /robot_1/raven_nav/discoveries')
    payload = json.loads(sniff.last('/robot_1/raven_nav/discoveries').data)
    assert isinstance(payload, list)
    for d in payload:
        assert {'instance_id', 'label', 'cx', 'cy', 'cz', 'status'} <= set(d), d
    print(f'[e2e] discoveries: {len(payload)} entries')


def test_raven_nav_emits_a_global_plan(chain, raven):
    """Bonus: an actual Path on `/robot_1/global_plan`.

    Not the pass bar — a behaviour can legitimately hold its waypoint — so this
    is xfail-able rather than fatal, but with frontiers present it should fire.
    """
    _proc, sniff = raven
    if not _wait_for(lambda: sniff.count('/robot_1/global_plan') > 0, 90.0):
        pytest.xfail('no global_plan within 90 s — behaviour produced no '
                     f'waypoint on this synthetic scene\n{chain.report()}')
    plan = sniff.last('/robot_1/global_plan')
    assert plan.header.frame_id == 'map'
    assert len(plan.poses) >= 1
    for ps in plan.poses:
        assert 1.5 - 1e-6 <= ps.pose.position.z <= 30.0 + 1e-6, ps.pose.position


# NOTE: must stay LAST in the file — it stops the raven_nav process.
def test_raven_nav_exits_cleanly_on_sigint(chain, raven):
    """Regression guard for the `rcl_shutdown already called` crash.

    `main()` used to call `rclpy.shutdown()` unconditionally in its `finally`,
    but rclpy's own SIGINT handler has already shut the default context down by
    then, so every clean Ctrl-C / `docker stop` / spawner teardown ended in an
    RCLError traceback and a non-zero exit — a normal end-of-mission that reads
    as a planner crash to whatever reaps it.
    """
    proc, _sniff = raven
    chain.require(proc.alive, 'raven_nav is already dead')
    rc = proc.stop()
    chain.require(rc == 0,
                  f'raven_nav exited {rc} on SIGINT (0 expected); '
                  'if the tail below shows `rcl_shutdown already called`, '
                  'main() regressed')
    assert 'rcl_shutdown already called' not in proc.tail(200)
    assert (chain.raven_results / 'robot_1.json').exists(), \
        'the forced end-of-mission results dump did not happen'
    payload = json.loads((chain.raven_results / 'robot_1.json').read_text())
    from raven_nav.results import RESULT_KEYS
    assert tuple(payload.keys()) == RESULT_KEYS
    assert payload['robot'] == 'robot_1'
    assert payload['boot_enu'] is not None
