import pytest
from conftest import airstack_cmd, read_log_tail, docker_image_size_mb, get_metrics


@pytest.mark.build_docker
@pytest.mark.timeout(3600)
class TestDockerBuilds:

    def _build_and_record(self, service, env=None):
        result = airstack_cmd("image-build", service, timeout=3600)
        assert result.returncode == 0, f"{service} build failed (exit {result.returncode}):\n{read_log_tail()}"

        size = docker_image_size_mb(service, env=env)
        if size is not None:
            get_metrics().record(f"docker.{service}", "image_size_mb", size, unit="MB")

    def test_build_robot_desktop(self):
        self._build_and_record("robot-desktop")

    def test_build_gcs(self):
        self._build_and_record("gcs")

    def test_build_isaac_sim(self):
        self._build_and_record("isaac-sim")

    def test_build_ms_airsim(self):
        self._build_and_record("ms-airsim", env={"COMPOSE_PROFILES": "ms-airsim"})

    # TODO: Test other profiles that build their own docker containers
