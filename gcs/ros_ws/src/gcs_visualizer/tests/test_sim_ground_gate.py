#!/usr/bin/env python3
"""The sim_ground gate: never build the plate from a frame that has not loaded.

    python3 -m pytest gcs/ros_ws/src/gcs_visualizer/tests/test_sim_ground_gate.py

Pure logic. The five methods are sliced out of foxglove_visualizer_node.py and
exec'd on a stub, the way test_sim_budget.py does it. No ROS, no sim, no GCS.

THE REGRESSION THIS PINS. `_on_overhead_image` used to build the marker from
the FIRST overhead Image it saw and then tear the subscriptions down. A frozen
cell is ~250 MB referenced in with its textures still streaming off Nucleus, so
Isaac's first rendered overhead frames are black. On 2026-08-29 every bag in a
24-iteration sweep carried exactly one `/gcs/sim_ground` message and it was
empty — one message, latched, blank, and no second chance for the whole run.

Two properties are pinned here:

1. **Blankness is MEASURED, not timed.** A warm-up alone cannot know how long a
   1 km cell takes to arrive off a particular Nucleus server; a per-channel std
   floor can tell a loaded frame from an unloaded one directly.
2. **More than one message reaches the bag.** The marker is republished until
   the settle window closes and rebuilt once from the final frame, so a
   recorder that subscribed late still finds it — a latched message that was
   published before the recorder existed is a message the bag does not have.
"""

import os
import sys
import textwrap
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_PATH = os.path.normpath(os.path.join(
    HERE, '..', 'gcs_visualizer', 'foxglove_visualizer_node.py'))
SRC = open(SRC_PATH, encoding='utf-8').read()


def _method_src(name):
    i = SRC.index(f'    def {name}(')
    j = SRC.index('\n    def ', i + 10)
    return SRC[i:j]


class _Image:                       # only needed for the type annotation
    pass


def _build(names):
    ns = {'np': np, 'time': _FakeTime, 'Image': _Image}
    body = '\n'.join(textwrap.dedent(_method_src(n)) for n in names)
    exec('class T:\n' + textwrap.indent(body, '    '), ns)   # noqa: S102
    return ns['T']


class _FakeTime:
    """`time.monotonic()` under test control — a settle window measured in
    real seconds would make this suite take a minute."""
    now = 0.0

    @staticmethod
    def monotonic():
        return _FakeTime.now


T = _build(['_overhead_is_loaded', '_on_overhead_image',
            '_republish_sim_ground', '_finish_sim_ground',
            '_try_build_pending'])


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(m)

    def warn(self, m, **kw):
        self.lines.append(m)


def frame(width=64, height=64, encoding='rgba8', fill=None, noisy=False):
    ch = 4 if encoding.endswith('a8') else 3
    if noisy:
        rng = np.random.default_rng(0)
        arr = rng.integers(0, 255, (height, width, ch), dtype=np.uint8)
    else:
        arr = np.full((height, width, ch), 0 if fill is None else fill,
                      dtype=np.uint8)
    m = _Image()
    m.width, m.height, m.encoding = width, height, encoding
    m.data = arr.tobytes()
    return m


class Node(T):
    """The smallest stub the five sliced methods touch."""

    def __init__(self, warmup=20.0, min_std=4.0, settle=45.0, period=15.0):
        self._ground_published = False
        self._ground_warmup_s = warmup
        self._ground_min_std = min_std
        self._ground_settle_s = settle
        self._ground_repeat_period_s = period
        self._overhead_first_seen = None
        self._ground_accepted_at = None
        self._latest_overhead = None
        self._ground_marker = None
        self._ground_builds = 0
        self._ground_blank_skips = 0
        self._ground_repeat_timer = None
        self._pending_image = None
        self._specs = True
        self._log = _Log()
        self.published = []
        self.built_from = []
        self.timers_created = 0
        self.destroyed_subs = []
        self._overhead_sub = 'sub'
        self._overhead_spec_sub = 'sub'
        self._overhead_cx_sub = 'sub'
        self._overhead_cy_sub = 'sub'
        self._ground_pub = types.SimpleNamespace(
            publish=lambda m: self.published.append(m))

    # --- collaborators the sliced code calls -----------------------------
    def _specs_ready(self):
        return self._specs

    def _build_sim_ground_marker(self, msg):
        self.built_from.append(msg)
        self._ground_marker = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=None), src=msg)
        self._ground_builds += 1
        self._ground_pub.publish(self._ground_marker)

    def create_timer(self, period, cb):
        self.timers_created += 1
        return types.SimpleNamespace(cancel=lambda: None, period=period, cb=cb)

    def destroy_timer(self, t):
        pass

    def destroy_subscription(self, s):
        self.destroyed_subs.append(s)

    def get_logger(self):
        return self._log

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: 'stamp'))


@pytest.fixture(autouse=True)
def _reset_clock():
    _FakeTime.now = 0.0
    yield


# ---------------------------------------------------------------------------
# 1. blankness is measured
# ---------------------------------------------------------------------------

def test_a_black_frame_is_not_loaded():
    n = Node()
    assert n._overhead_is_loaded(frame(fill=0)) is False


def test_a_flat_grey_frame_is_not_loaded():
    """Isaac renders a uniform clear colour before the scene arrives; it is not
    black, and a brightness test would pass it."""
    n = Node()
    assert n._overhead_is_loaded(frame(fill=128)) is False


def test_a_textured_frame_is_loaded():
    n = Node()
    assert n._overhead_is_loaded(frame(noisy=True)) is True


@pytest.mark.parametrize('enc', ['rgba8', 'rgb8', 'bgr8', 'bgra8'])
def test_every_encoding_isaac_publishes_decodes(enc):
    n = Node()
    assert n._overhead_is_loaded(frame(encoding=enc, noisy=True)) is True
    assert n._overhead_is_loaded(frame(encoding=enc, fill=0)) is False


def test_a_frame_that_cannot_be_decoded_is_not_loaded():
    """Never raise inside a subscription callback over a malformed frame."""
    n = Node()
    m = frame(noisy=True)
    m.data = m.data[:-7]                     # not divisible by width*height*ch
    assert n._overhead_is_loaded(m) is False


def test_a_big_frame_is_strided_not_fully_decoded():
    """2048x2048 arrives every sim tick; the gate must stay cheap."""
    n = Node()
    assert n._overhead_is_loaded(frame(2048, 2048, noisy=True)) is True


# ---------------------------------------------------------------------------
# 2. the build waits
# ---------------------------------------------------------------------------

def test_the_first_frame_does_not_build():
    """THE 2026-08-29 BUG, PINNED: the first frame is the one most likely to be
    the black one, and it used to be the only one ever used."""
    n = Node()
    n._on_overhead_image(frame(noisy=True))
    assert n._ground_builds == 0
    assert n._overhead_first_seen == 0.0


def test_nothing_builds_during_the_warmup():
    n = Node(warmup=20.0)
    n._on_overhead_image(frame(noisy=True))
    for t in (1.0, 10.0, 19.9):
        _FakeTime.now = t
        n._on_overhead_image(frame(noisy=True))
    assert n._ground_builds == 0


def test_a_blank_frame_after_the_warmup_still_does_not_build():
    n = Node(warmup=20.0)
    n._on_overhead_image(frame(fill=0))
    _FakeTime.now = 100.0
    for _ in range(5):
        n._on_overhead_image(frame(fill=0))
    assert n._ground_builds == 0
    assert n._ground_blank_skips == 5
    assert n._ground_published is False       # still listening


def test_the_first_loaded_frame_after_the_warmup_builds_and_starts_repeating():
    n = Node(warmup=20.0)
    n._on_overhead_image(frame(fill=0))
    _FakeTime.now = 25.0
    good = frame(noisy=True)
    n._on_overhead_image(good)
    assert n._ground_builds == 1
    assert n.built_from == [good]
    assert n.timers_created == 1
    assert n._ground_accepted_at == 25.0
    assert n._ground_published is False       # subscriptions stay open


def test_the_subscription_is_not_torn_down_at_the_first_build():
    """The old code unsubscribed here, which is what made the blank marker
    permanent."""
    n = Node(warmup=0.0)
    n._on_overhead_image(frame(noisy=True))
    _FakeTime.now = 1.0
    n._on_overhead_image(frame(noisy=True))
    assert n.destroyed_subs == []


def test_a_cached_image_waiting_on_specs_goes_through_the_gate():
    """`_try_build_pending` used to call the builder directly, bypassing both
    gates — with the OLDEST frame in hand."""
    n = Node(warmup=20.0)
    n._specs = False
    blank = frame(fill=0)
    n._on_overhead_image(blank)
    assert n._pending_image is blank
    n._specs = True
    _FakeTime.now = 60.0                      # warm-up long over
    n._try_build_pending()
    assert n._ground_builds == 0, 'built the plate from the cached blank frame'
    assert n._pending_image is None


# ---------------------------------------------------------------------------
# 3. more than one message reaches the bag
# ---------------------------------------------------------------------------

def _accept(n, at=25.0):
    n._on_overhead_image(frame(fill=0))
    _FakeTime.now = at
    n._on_overhead_image(frame(noisy=True))
    return n


def test_the_marker_is_republished_until_the_settle_window_closes():
    n = _accept(Node(warmup=20.0, settle=45.0, period=15.0))
    assert len(n.published) == 1
    for t in (40.0, 55.0, 69.0):
        _FakeTime.now = t
        n._republish_sim_ground()
    assert len(n.published) == 4, [p for p in n.published]
    assert n._ground_builds == 1              # republish, not rebuild
    assert n._ground_published is False


def test_the_settle_window_ends_in_one_rebuild_from_the_newest_frame():
    n = _accept(Node(warmup=20.0, settle=45.0))
    _FakeTime.now = 60.0
    newest = frame(noisy=True)
    n._on_overhead_image(newest)              # cached as _latest_overhead
    _FakeTime.now = 71.0                      # 25 + 45 = 70 -> settled
    n._republish_sim_ground()
    assert n._ground_builds == 2
    assert n.built_from[-1] is newest
    assert n._ground_published is True


def test_the_rebuild_happens_at_most_once():
    n = _accept(Node(warmup=20.0, settle=45.0))
    n._ground_builds = 2                      # pretend the rebuild already ran
    _FakeTime.now = 200.0
    n._republish_sim_ground()
    assert n._ground_builds == 2
    assert n._ground_published is True


def test_finishing_tears_down_every_overhead_subscription():
    n = _accept(Node(warmup=20.0))
    n._finish_sim_ground()
    assert len(n.destroyed_subs) == 4
    assert n._ground_published is True
    assert n._latest_overhead is None


def test_nothing_is_published_after_the_teardown():
    n = _accept(Node(warmup=20.0))
    n._finish_sim_ground()
    before = len(n.published)
    _FakeTime.now = 500.0
    n._republish_sim_ground()
    n._on_overhead_image(frame(noisy=True))
    assert len(n.published) == before
    assert n._ground_builds == 1
