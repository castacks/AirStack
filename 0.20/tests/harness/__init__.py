"""AirStack test harness — helpers split out of conftest.py by concern.

conftest.py holds the pytest hooks and fixtures and imports what it needs from here.
Public helpers are re-exported so tests and conftest can ``from harness import <name>``.
"""
from harness.commands import (
    ROS_DISTRO_SETUP,
    airstack_cmd,
    current_log,
    docker_exec,
    read_log_tail,
    ros2_env,
    ros2_exec,
)
from harness.containers import (
    container_running,
    docker_image_size_mb,
    find_all_containers,
    find_container,
    get_robot_containers,
    missing_images,
    sample_compute_usage,
    wait_for_container,
)
from harness.discovery import (
    AIRSTACK_ROOT,
    COLCON_UNIT_TEST_PACKAGES_YAML,
    TESTS_DIR,
    colcon_test_robot_command,
    collection_is_broad,
    format_pytest_addopts,
    load_colcon_unit_test_config,
    repo_path,
    unit_test_dirs,
    unit_test_files,
)
from harness.diagnostics import collect_failure_diagnostics
from harness.metrics import MetricsRecorder, current_test_id, get_metrics
from harness.session import logger
from harness.sim import (
    SIM_CONFIG,
    SimulatorHealthError,
    parallel_echo_once_robot_topics,
    parallel_sample_hz,
    sample_hz,
    wait_for_first_message,
)

__all__ = [
    # discovery
    "AIRSTACK_ROOT", "COLCON_UNIT_TEST_PACKAGES_YAML", "TESTS_DIR", "repo_path",
    "colcon_test_robot_command", "collection_is_broad", "format_pytest_addopts",
    "load_colcon_unit_test_config", "unit_test_dirs", "unit_test_files",
    # session
    "logger", "collect_failure_diagnostics",
    # commands
    "ROS_DISTRO_SETUP", "airstack_cmd", "current_log", "docker_exec",
    "read_log_tail", "ros2_env", "ros2_exec",
    # containers
    "find_all_containers", "find_container", "get_robot_containers",
    "container_running", "wait_for_container", "missing_images",
    "sample_compute_usage", "docker_image_size_mb",
    # metrics
    "MetricsRecorder", "get_metrics", "current_test_id",
    # sim
    "SIM_CONFIG", "SimulatorHealthError", "wait_for_first_message", "sample_hz", "parallel_sample_hz",
    "parallel_echo_once_robot_topics",
]
