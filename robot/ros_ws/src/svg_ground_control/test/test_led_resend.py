"""led_controller keep-alive: resend() must follow resend_hz.

The keep-alive threshold used to be ``0.5 / max(0.1, 1.0)`` (always 0.5 s),
so raising resend_hz sped up the timer but not the keep-alive, and at the
default 2 Hz a tick landing a hair early skipped its packet.
"""

from __future__ import annotations

import pytest

pytest.importorskip("rclpy")

from svg_ground_control import led_controller as lc  # noqa: E402


def make(resend_hz):
    """A LedController with just the state resend() reads (no ROS, no socket)."""
    node = lc.LedController.__new__(lc.LedController)
    node.resend_hz = resend_hz
    node.last_cbf_msg = None
    node.cbf_stale_s = 1.0
    node.cbf_rgbw = (255, 0, 0, 0)
    d = lc.LedState('drone_1', (0, 80, 0, 0))
    d.addr = ('127.0.0.1', 47900)
    node.drones = {'drone_1': d}
    node.sent = []

    def send(drone, color, mode):
        node.sent.append(drone.name)
        drone.last_sent = (color, mode)
        drone.last_send_time = lc.time.time()

    node.send = send
    return node, d


def run_ticks(node, monkeypatch, period, n, jitter=0.0):
    t = [1000.0]
    monkeypatch.setattr(lc.time, 'time', lambda: t[0])
    for _ in range(n):
        node.resend()
        t[0] += period - jitter


@pytest.mark.parametrize('hz', [2.0, 5.0, 10.0])
def test_keepalive_goes_out_on_every_tick_at_resend_hz(monkeypatch, hz):
    node, _ = make(hz)
    # 1 ms early on every tick: the old strict "> 0.5" check skipped these.
    run_ticks(node, monkeypatch, 1.0 / hz, 20, jitter=0.001)
    assert len(node.sent) == 20


def test_no_extra_packets_between_ticks(monkeypatch):
    node, _ = make(2.0)
    run_ticks(node, monkeypatch, 0.1, 10)    # t = 0.0 .. 0.9, 5x the timer rate
    # t=0 sends (nothing sent yet), then one keep-alive once >= 0.45 s passed
    assert len(node.sent) == 2


def test_color_change_still_sends_immediately(monkeypatch):
    node, d = make(2.0)
    run_ticks(node, monkeypatch, 0.5, 1)
    d.base = (0, 0, 80, 0)
    node.resend()                      # same timestamp, new color
    assert len(node.sent) == 2
