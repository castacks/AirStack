import pytest
from conftest import airstack_cmd, log_name, read_log_tail


@pytest.mark.build_docker
@pytest.mark.timeout(3600)
class TestDockerBuilds:

    def test_build_robot_desktop(self):
        log = log_name()
        result = airstack_cmd("image-build", "robot-desktop", timeout=3600, log_name=log)
        assert result.returncode == 0, f"robot-desktop build failed (exit {result.returncode}):\n{read_log_tail(log)}"

    def test_build_gcs(self):
        log = log_name()
        result = airstack_cmd("image-build", "gcs", timeout=3600, log_name=log)
        assert result.returncode == 0, f"gcs build failed (exit {result.returncode}):\n{read_log_tail(log)}"

    def test_build_isaac_sim(self):
        log = log_name()
        result = airstack_cmd("image-build", "isaac-sim", timeout=3600, log_name=log)
        assert result.returncode == 0, f"isaac-sim build failed (exit {result.returncode}):\n{read_log_tail(log)}"

    def test_build_ms_airsim(self):
        log = log_name()
        result = airstack_cmd("image-build", "ms-airsim", timeout=3600, log_name=log)
        assert result.returncode == 0, f"ms-airsim build failed (exit {result.returncode}):\n{read_log_tail(log)}"

    # TODO: Test other profiles that build their own docker containers