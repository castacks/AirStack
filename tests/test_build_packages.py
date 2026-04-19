import pytest
from conftest import airstack_cmd, wait_for_container, docker_exec, read_log_tail


@pytest.mark.build_packages
@pytest.mark.timeout(1200)
class TestColconBuilds:

    def test_colcon_build_robot(self):
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
