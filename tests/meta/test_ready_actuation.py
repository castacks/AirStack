"""Fail-closed effective MAVROS actuation readiness, without a ROS runtime."""
from pathlib import Path
import subprocess

import pytest

READY = Path(__file__).resolve().parents[2] / '.airstack/modules/ready.sh'


@pytest.mark.parametrize('value,status,expected', [
    ('Double value is: 1.0', 0, 0),
    ('Double value is: nan', 0, 1),
    ('Double value is: inf', 0, 1),
    ('Double value is: 0.0', 0, 1),
    ('Double value is: 0.5', 0, 1),
    ('Integer value is: 1', 0, 1),
    ('Parameter not set', 0, 1),
    ('Double value is: 1.0', 1, 1),
])
def test_effective_thrust_scaling(value, status, expected):
    script = '''
source "$1"
_ready_ros2_exec() {
    [[ "$1" == 'robot-container' && "$2" == '7' && "$4" == '6' ]] || return 99
    if [[ "$3" == 'ros2 param get /fleet_drone/interface/mavros/setpoint_raw thrust_scaling' ]]; then
        printf '%s\\n' "$VALUE"
        return "$STATUS"
    fi
    [[ "$3" == 'ros2 topic echo --once --csv --field data /fleet_drone/interface/actuation_ready' ]] || return 99
    echo True
}
_gate_px4_actuation_ok 7 fleet_drone robot-container
'''
    result = subprocess.run(['bash', '-c', script, 'probe', str(READY)],
                            env={'PATH': '/usr/bin:/bin', 'VALUE': value, 'STATUS': str(status)},
                            capture_output=True, text=True)
    assert result.returncode == expected, result.stderr


@pytest.mark.parametrize('observed,expected', [('False', 1), ('', 1), ('not-a-bool', 1),
                                             ('True\n---', 0), ('True\nFalse', 1)])
def test_initializer_report_is_strict(observed, expected):
    script = '''
source "$1"
_ready_ros2_exec() {
    if [[ "$3" == 'ros2 param get /fleet_drone/interface/mavros/setpoint_raw thrust_scaling' ]]; then
        echo 'Double value is: 1.0'
    else
        printf '%s\\n' "$OBSERVED"
    fi
}
_gate_px4_actuation_ok 7 fleet_drone robot-container
'''
    result = subprocess.run(['bash', '-c', script, 'probe', str(READY)],
                            env={'PATH': '/usr/bin:/bin', 'OBSERVED': observed}, capture_output=True)
    assert result.returncode == expected


def test_actuation_failure_reaches_verdict():
    script = '''
source "$1"
resolve_launch_var() { echo true; }
_robot_containers() { echo robot-container; }
_container_identity() { printf 'fleet_drone\\t7\\n'; }
_ready_poll() { local predicate="$3"; [[ "$predicate" != '_gate_px4_actuation_ok' ]]; }
_gate_clock_epoch_ok() { return 0; }
log_error() { :; }
declare -A results=()
_ready_run_gates >/dev/null
[[ "$READY_OVERALL" == 1 && "${results[px4_actuation_fleet_drone]}" == failed ]]
'''
    result = subprocess.run(['bash', '-c', script, 'probe', str(READY)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
