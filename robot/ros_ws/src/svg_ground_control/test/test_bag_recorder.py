"""Tests for the ``bag_recorder`` node's switch and status snapshot.

Constructs the real ``BagRecorderNode`` and drives its ``~/record``
(``std_srvs/SetBool``) service the way the Foxglove basestation panel does.
The recorder command is swapped for a long ``sleep`` (which, like ``ros2 bag
record``, exits on SIGINT) so no ROS bag is written and no ``ros2`` CLI is
needed. Skipped where rclpy is unavailable.
"""

from __future__ import annotations

import json
import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402

from svg_ground_control import bag_recorder as br  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


class FakeRecorder(br.BagRecorderNode):
    """The real node with the recorder command replaced by a sleeping process
    that creates its output directory, as ``ros2 bag record`` does."""

    def build_command(self, path):
        return ["sh", "-c", f"mkdir -p '{path}' && exec sleep 60"]


def make_node(tmp_path, **params):
    overrides = [Parameter("bag_dir", value=str(tmp_path)),
                 Parameter("status_hz", value=50.0)]
    overrides += [Parameter(k, value=v) for k, v in params.items()]
    return FakeRecorder(parameter_overrides=overrides)


def record(node, on: bool):
    req = SetBool.Request()
    req.data = on
    return node.on_record(req, SetBool.Response())


def test_switch_starts_and_stops_one_bag_per_start(tmp_path):
    node = make_node(tmp_path, bag_prefix="unit")
    try:
        assert node.build_status()["recording"] is False

        res = record(node, True)
        assert res.success and "recording all topics to" in res.message
        assert node.recording()
        status = node.build_status()
        assert status["recording"] is True
        assert status["path"].startswith(str(tmp_path / "unit_"))
        # The child creates its output directory; give it a moment.
        deadline = time.monotonic() + 5.0
        while not os.path.isdir(status["path"]) and time.monotonic() < deadline:
            time.sleep(0.02)
        assert os.path.isdir(status["path"])
        assert status["duration_s"] >= 0.0

        # A second "on" is idempotent: same bag, no second process.
        pid = node.proc.pid
        res = record(node, True)
        assert res.success and "already recording" in res.message
        assert node.proc.pid == pid

        bag = status["path"]
        res = record(node, False)
        assert res.success and res.message.startswith("stopped:")
        assert not node.recording()
        status = node.build_status()
        assert status["recording"] is False
        assert status["path"] is None
        assert status["last_bag"] == bag
        assert status["error"] is None

        # Off while off is harmless.
        res = record(node, False)
        assert res.success and res.message == "not recording"

        # Starting again makes a NEW bag directory.
        record(node, True)
        assert node.build_status()["path"] != bag
        record(node, False)
    finally:
        node.destroy_node()


def test_trigger_services_and_status_message(tmp_path):
    node = make_node(tmp_path)
    try:
        res = node.on_start(Trigger.Request(), Trigger.Response())
        assert res.success and node.recording()
        msg_data = []
        node.status_pub.publish = lambda msg: msg_data.append(msg.data)  # type: ignore[assignment]
        node.publish_status()
        snap = json.loads(msg_data[-1])
        assert snap["recording"] is True and snap["bag_dir"] == str(tmp_path)
        assert snap["storage"] == "mcap"
        res = node.on_stop(Trigger.Request(), Trigger.Response())
        assert res.success and not node.recording()
    finally:
        node.destroy_node()


def test_recorder_dying_on_its_own_is_reported(tmp_path):
    node = make_node(tmp_path)
    try:
        record(node, True)
        # Simulate the recorder crashing (disk full, bad storage plugin ...).
        node.proc.kill()
        node.proc.wait()
        status = node.build_status()
        assert status["recording"] is False
        assert "exited on its own" in status["error"]
        assert status["last_result"] == status["error"]
        # And the switch can be turned back on afterwards.
        res = record(node, True)
        assert res.success and node.recording()
        assert node.build_status()["error"] is None
        record(node, False)
    finally:
        node.destroy_node()


def test_record_on_start_and_destroy_stops_cleanly(tmp_path):
    node = make_node(tmp_path, record_on_start=True)
    assert node.recording()
    proc = node.proc
    node.destroy_node()
    assert proc.poll() is not None, "destroy_node must stop the recorder"
    assert node.last_result.startswith("stopped:")


def test_unwritable_bag_dir_is_refused(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("not a directory")
    node = make_node(blocker)
    try:
        res = record(node, True)
        assert not res.success and "cannot create bag_dir" in res.message
        assert not node.recording()
        assert node.build_status()["error"] == res.message
    finally:
        node.destroy_node()


def test_command_line_reflects_parameters(tmp_path):
    node = br.BagRecorderNode(parameter_overrides=[
        Parameter("bag_dir", value=str(tmp_path)),
        Parameter("include_hidden", value=True),
        Parameter("extra_args", value=["--max-cache-size", "0"]),
    ])
    try:
        cmd = node.build_command("/x/bag")
        assert cmd[:4] == ["ros2", "bag", "record", "--all-topics"]
        assert cmd[cmd.index("--storage") + 1] == "mcap"
        assert cmd[cmd.index("--output") + 1] == "/x/bag"
        assert "--include-hidden-topics" in cmd
        assert cmd[-2:] == ["--max-cache-size", "0"]
    finally:
        node.destroy_node()
