"""Map-readiness hold: raven freezes while the shared mapper is down OR its
fresh map is still rebuilding after a restart (user 2026-09-02 night — the
better signal than pure status-staleness, which resumes before the map is
usable)."""
import json
import types

import pytest

import ros_stubs

pytestmark = pytest.mark.skipif(
    ros_stubs.ros_is_real(), reason='pure logic; runs on the stub host')


def _node_cls():
    ros_stubs.install()
    import importlib
    import raven_nav.raven_nav_node as rn
    importlib.reload(rn)
    return rn.RavenNavNode


def _bare():
    """A RavenNavNode with just the hold state, no rclpy."""
    n = _node_cls().__new__(_node_cls())
    n._rayfronts_shared = True
    n._rf_hold_timeout_s = 5.0
    n._rf_resume_vox = 300
    n._last_rf_status_ts = None
    n._rf_prev_frames = None
    n._rf_map_ready = None
    n._clock = {'t': 100.0}
    n._now = lambda: n._clock['t']            # type: ignore[method-assign]
    return n


def _status(frames, vox):
    return types.SimpleNamespace(
        data=json.dumps({'frames_total': frames, 'vox_count': vox}))


def test_before_first_status_never_holds():
    n = _bare()
    assert n._map_unusable() is False


def test_alive_with_a_full_map_does_not_hold():
    n = _bare()
    n._rf_status_cb(_status(frames=500, vox=8000))
    assert n._map_unusable() is False


def test_full_silence_holds():
    n = _bare()
    n._rf_status_cb(_status(frames=500, vox=8000))
    n._clock['t'] += 6.0                        # > timeout, no new status
    assert n._map_unusable() is True


def test_restart_holds_until_map_rebuilds():
    n = _bare()
    n._rf_status_cb(_status(frames=800, vox=9000))     # healthy
    assert n._map_unusable() is False
    n._rf_status_cb(_status(frames=5, vox=50))         # frames RESET = restart
    assert n._map_unusable() is True, 'empty rebuilt map must hold'
    n._rf_status_cb(_status(frames=20, vox=120))       # still rebuilding
    assert n._map_unusable() is True
    n._rf_status_cb(_status(frames=60, vox=400))       # >= resume floor
    assert n._map_unusable() is False, 'resume once vox recovers'


def test_disabled_by_zero_timeout():
    n = _bare()
    n._rf_hold_timeout_s = 0.0
    n._rf_status_cb(_status(frames=5, vox=1))
    n._clock['t'] += 100.0
    assert n._map_unusable() is False
