from pathlib import Path

import pytest

from conftest import (AIRSTACK_ROOT, airstack_cmd, docker_exec, logger,
                      read_log_tail, wait_for_container)


def _warn_if_prebuilt(*ws_paths):
    """Log a warning if any of the given workspace directories already contain
    build/install/log dirs. Doesn't fail the test — just signals that what we
    measure may be an INCREMENTAL build, not a clean one."""
    dirty = [p for p in ws_paths
             if any((Path(AIRSTACK_ROOT) / p / sub).is_dir()
                    for sub in ("build", "install"))]
    if dirty:
        logger.warning(
            "Workspace(s) %s already have build artifacts — this test may "
            "measure an incremental build, not a clean one. Run "
            "`./airstack.sh clean` first if you want a cold-build measurement.",
            dirty)


@pytest.mark.build_packages
@pytest.mark.timeout(1200)
class TestColconBuilds:

    def test_colcon_build_robot(self):
        _warn_if_prebuilt("robot/ros_ws")
        try:
            result = airstack_cmd("up", "robot-desktop",
                                  env_overrides={"AUTOLAUNCH": "false", "DISPLAY": ""},
                                  timeout=120)
            assert result.returncode == 0, f"airstack up failed:\n{read_log_tail()}"

            container = wait_for_container("robot.*desktop", timeout=60)
            assert container, "Robot container not found"

            build = docker_exec(container, "bash -ic bws", timeout=600)
            assert build.returncode == 0, f"colcon build failed:\n{read_log_tail()}"
        finally:
            airstack_cmd("down")

    def test_colcon_build_gcs(self):
        _warn_if_prebuilt("gcs/ros_ws")
        try:
            result = airstack_cmd("up", "gcs",
                                  env_overrides={"AUTOLAUNCH": "false", "DISPLAY": ""},
                                  timeout=120)
            assert result.returncode == 0, f"airstack up failed:\n{read_log_tail()}"

            container = wait_for_container("gcs", timeout=60)
            assert container, "GCS container not found"

            build = docker_exec(container, "bash -ic bws", timeout=600)
            assert build.returncode == 0, f"colcon build failed:\n{read_log_tail()}"
        finally:
            airstack_cmd("down")

    def test_colcon_build_ms_airsim(self):
        _warn_if_prebuilt("simulation/ms-airsim/ros_ws")
        try:
            result = airstack_cmd(
                "up", "ms-airsim",
                env_overrides={"AUTOLAUNCH": "false", "DISPLAY": "",
                               "COMPOSE_PROFILES": "ms-airsim",
                               "URDF_FILE": "robot_descriptions/iris/urdf/iris_stereo.ms-airsim.urdf"},
                timeout=120,
            )
            assert result.returncode == 0, f"airstack up failed:\n{read_log_tail()}"

            container = wait_for_container("ms-airsim", timeout=60)
            assert container, "ms-airsim container not found"

            build = docker_exec(
                container,
                "cd /root/ros_ws && colcon build --symlink-install",
                timeout=600,
            )
            assert build.returncode == 0, f"colcon build failed:\n{read_log_tail()}"
        finally:
            airstack_cmd("down")
