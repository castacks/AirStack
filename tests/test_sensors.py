"""Sensor stream and LiDAR validation — runs after ``test_liveliness`` (see ``_MODULE_ORDER``).

Uses the same ``airstack_env`` parametrization as liveliness. With ``class``-scoped
fixtures this module performs its **own** stack bring-up when selected; combined
``-m "liveliness and sensors"`` therefore runs two bring-up cycles per
``(sim, num_robots, iteration)``. Use ``-m sensors`` alone to exercise only sensor
checks (still one full ``airstack up``).
"""
import time

import pytest

from conftest import current_test_id, get_metrics, logger, wait_for_first_message
from sensor_probes import (
    STABLE_HZ_DURATION_S,
    STABLE_HZ_WINDOW,
    check_lidar_filtered_cloud_sanity,
    check_realtime_factor,
    check_robot_filtered_lidar,
    check_robot_stereo_hz,
    check_sim_publishing,
)
from test_liveliness import _check_sentinel_nodes, _poll_until


@pytest.mark.sensors
@pytest.mark.timeout(1800)
class TestSensors:

    @pytest.mark.dependency(name="sensors_sim_ready")
    def test_sim_clock_available(self, airstack_env):
        """Wait for ``/clock`` on the sim container (same readiness gate as liveliness)."""
        cfg = airstack_env["cfg"]
        m = get_metrics()
        tid = current_test_id()
        start = airstack_env["up_started_at"]
        if (
            wait_for_first_message(
                airstack_env["sim_container"],
                "/clock",
                domain_id=1,
                setup_bash=cfg["sim_setup_bash"],
                timeout=600,
            )
            is None
        ):
            m.record(tid, "sensors_sim_ready_duration_s", "timeout", unit="s")
            pytest.fail("sim never published /clock within 600s")
        m.record(tid, "sensors_sim_ready_duration_s", round(time.time() - start, 2), unit="s")

    @pytest.mark.dependency(name="sensors_nodes", depends=["sensors_sim_ready"])
    def test_sentinel_nodes_present(self, airstack_env):
        """ROS 2 nodes required before trusting sensor pipelines."""
        last_msg = [""]

        def ready():
            ok, msg = _check_sentinel_nodes(airstack_env)
            last_msg[0] = msg
            return ok

        _poll_until(
            ready,
            timeout=300,
            interval=5,
            fail_msg=lambda: f"sentinel nodes not ready after 300s: {last_msg[0]}",
        )

    @pytest.mark.dependency(name="sensors_sim_hz", depends=["sensors_sim_ready"])
    def test_sim_topic_publish_rates(self, airstack_env):
        """Hz on sim ``/clock`` + stereo + depth (batched for Isaac)."""
        ok, msg, _ = check_sim_publishing(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(name="sensors_robot_stereo", depends=["sensors_sim_hz"])
    def test_robot_stereo_bridge_rates(self, airstack_env):
        """Hz on robot DDS for stereo + depth (bridge path)."""
        ok, msg, _ = check_robot_stereo_hz(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(name="sensors_robot_lidar", depends=["sensors_robot_stereo"])
    def test_robot_filtered_lidar_stream(self, airstack_env):
        """Filtered LiDAR ``echo --once`` per robot (isaacsim only; skipped elsewhere)."""
        ok, msg, _ = check_robot_filtered_lidar(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(name="sensors_rtf", depends=["sensors_sim_ready"])
    def test_sim_clock_realtime_factor(self, airstack_env):
        """RTF from ``/clock``; fails only if sim near-stalled (RTF < 0.1)."""
        ok, msg, rtf = check_realtime_factor(airstack_env)
        if rtf is not None:
            get_metrics().record(
                current_test_id(),
                "sim.realtime_factor",
                round(rtf, 3),
                unit="",
                direction="higher_is_better",
            )
        assert ok, msg

    @pytest.mark.dependency(
        name="sensors_lidar_cloud_sanity",
        depends=["sensors_robot_lidar", "sensors_nodes"],
    )
    def test_lidar_filtered_cloud_sanity(self, airstack_env):
        """Point cloud geometry vs ``near_range_m`` (isaacsim only)."""
        ok, msg, _ = check_lidar_filtered_cloud_sanity(airstack_env)
        assert ok, msg

    @pytest.mark.dependency(
        depends=[
            "sensors_sim_hz",
            "sensors_robot_stereo",
            "sensors_robot_lidar",
            "sensors_lidar_cloud_sanity",
        ]
    )
    def test_sensor_streams_stable(self, airstack_env, request):
        """Poll sensor topic rates over ``--stable-duration`` (Hz time-series to metrics)."""
        duration = request.config.getoption("--stable-duration")
        interval = request.config.getoption("--stable-interval")
        m = get_metrics()
        tid = current_test_id()
        series = {}
        elapsed = 0
        try:
            while elapsed < duration:
                time.sleep(interval)
                elapsed += interval
                ok_sim, msg_sim, rates_sim = check_sim_publishing(
                    airstack_env,
                    duration=STABLE_HZ_DURATION_S,
                    window=STABLE_HZ_WINDOW,
                )
                ok_rsd, msg_rsd, rates_rsd = check_robot_stereo_hz(
                    airstack_env,
                    duration=STABLE_HZ_DURATION_S,
                    window=STABLE_HZ_WINDOW,
                )
                ok_lidar, msg_lidar, rates_lidar = check_robot_filtered_lidar(
                    airstack_env,
                    duration=STABLE_HZ_DURATION_S,
                    window=STABLE_HZ_WINDOW,
                )
                rates = {**rates_sim, **rates_rsd, **rates_lidar}
                for topic, hz in rates.items():
                    key = topic.lstrip("/").replace("/", ".") + ".hz"
                    series.setdefault(key, []).append({"t": elapsed, "value": hz or 0.0})
                if not (ok_sim and ok_rsd and ok_lidar):
                    pytest.fail(
                        f"sensor instability at t={elapsed}s: sim_hz={msg_sim} | "
                        f"robot_stereo_hz={msg_rsd} | robot_lidar={msg_lidar}"
                    )
        finally:
            for key, samples in series.items():
                if samples:
                    m.record_list(tid, f"{key}_samples", samples)
