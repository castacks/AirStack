"""Actual interface plugin vs mocked MAVROS, refusing any nonempty/live domain."""
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '198':
    raise SystemExit('Required empty isolated ROS_DOMAIN_ID=198')
import rclpy
from ament_index_python.packages import get_package_prefix
from airstack_msgs.srv import RobotCommand
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Bool

rclpy.init()
node = rclpy.create_node('admission_probe', namespace='/fleet_alpha/interface')
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=.05)
if any(name != 'admission_probe' for name, _ in node.get_node_names_and_namespaces()):
    raise SystemExit('Domain not empty; refusing synthetic publications')
state = node.create_publisher(State, 'mavros/state', 10)
ground = node.create_publisher(ExtendedState, 'mavros/extended_state', 10)
case = {}
events = []
ready = []
authority = []
get_times = []


def get_parameters(request, response):
    assert request.names == ['thrust_scaling'], request.names
    get_times.append(time.monotonic())
    if case.get('delay_get'):
        time.sleep(.7)
    response.values = [ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1)
                       if case.get('integer') else
                       ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=case['value'])]
    return response


def set_parameters(request, response):
    assert len(request.parameters) == 1
    p = request.parameters[0]
    assert p.name == 'thrust_scaling' and p.value.type == ParameterType.PARAMETER_DOUBLE
    assert p.value.double_value == 1.0
    events.append(('set', p.value.double_value))
    if case.get('delay_set'):
        time.sleep(.7)
    if not case.get('reject_set') and not case.get('bad_readback'):
        case['value'] = 1.0
    response.results = [SetParametersResult(successful=not case.get('reject_set'))]
    return response


def mode(request, response):
    events.append(('mode', request.custom_mode))
    response.mode_sent = True
    return response


def arm(request, response):
    events.append(('arm', request.value))
    response.success = True
    return response


get_service = node.create_service(GetParameters, 'mavros/setpoint_raw/get_parameters', get_parameters)
set_service = node.create_service(SetParameters, 'mavros/setpoint_raw/set_parameters', set_parameters)
node.create_service(SetMode, 'mavros/set_mode', mode)
node.create_service(CommandBool, 'mavros/cmd/arming', arm)
node.create_subscription(Bool, 'actuation_ready', lambda message: ready.append(message.data), 10)
node.create_subscription(Bool, 'has_control', lambda message: authority.append(message.data), 10)
command = node.create_client(RobotCommand, 'robot_command')


def pump(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not case.get('missing_state'):
            state.publish(State(connected=True, armed=case.get('armed', False),
                                mode=case.get('mode', 'AUTO.LOITER')))
        if not case.get('missing_ground'):
            ground.publish(ExtendedState(landed_state=2 if case.get('airborne') else 1))
        rclpy.spin_once(node, timeout_sec=.01)


def invoke(value):
    started = time.monotonic()
    future = command.call_async(RobotCommand.Request(command=value))
    while not future.done() and time.monotonic() - started < 4.:
        pump(.02)
    assert future.done(), ('unbounded command', value, events)
    return future.result().success


results = []
names = ['success', 'reject_set', 'bad_readback', 'integer', 'delay_set', 'delay_get',
         'missing_state', 'missing_ground', 'armed', 'airborne', 'invalid_config', 'missing_child']
names += ['armed_then_ground', 'airborne_then_ground']
names += ['config_quoted', 'config_integer', 'config_wrong', 'missing_get']
for name in names:
    case.clear(); case.update(value=math.nan)
    case[name] = True
    if name == 'armed_then_ground':
        case['armed'] = True
    if name == 'airborne_then_ground':
        case['airborne'] = True
    events.clear(); ready.clear(); get_times.clear(); authority.clear()
    if name == 'missing_child':
        node.destroy_service(set_service)
    if name == 'missing_get':
        node.destroy_service(get_service)
    binary = sys.argv[1] if len(sys.argv) > 1 else str(
        Path(get_package_prefix('robot_interface')) / 'lib/robot_interface/robot_interface_node')
    args = [binary, '--ros-args', '-r', '__ns:=/fleet_alpha/interface',
            '-p', 'actuation_startup_timeout_s:=1.5']
    if name == 'invalid_config':
        args += ['-p', 'mavros_actuation_config:=/nonexistent/rrm-actuation.yaml']
    if name.startswith('config_'):
        path = Path(__file__).parent / 'fixtures' / (name.removeprefix('config_') + '.yaml')
        args += ['-p', 'mavros_actuation_config:=' + str(path)]
    proc = subprocess.Popen(args)
    try:
        assert command.wait_for_service(timeout_sec=5.)
        # No state has been published: a command cannot bootstrap initialization.
        future = command.call_async(RobotCommand.Request(command=RobotCommand.Request.ARM))
        deadline = time.monotonic() + 3.
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert future.done() and not future.result().success
        assert not events, events
        if name in ('armed_then_ground', 'airborne_then_ground'):
            pump(.4)
            case['armed'] = False; case['airborne'] = False
            pump(1.6)
        else:
            pump(2.0)
        writes = sum(event[0] == 'set' for event in events)
        if name == 'success':
            assert any(ready) and writes == 1, (name, events, ready)
            # Actual plugin telemetry: auto-disarm can retain OFFBOARD mode.
            # No DISARM service is called for this transition.
            for armed_value, mode_value, expected in (
                (True, 'OFFBOARD', True), (False, 'OFFBOARD', False),
                (True, 'AUTO.LAND', False), (True, 'OFFBOARD', True),
                (False, 'AUTO.LOITER', False)):
                case.update(armed=armed_value, mode=mode_value)
                pump(.25)
                mark = len(authority)
                pump(.15)
                assert len(authority) > mark and all(v is expected for v in authority[mark:]), (
                    'armed authority', armed_value, mode_value, expected, authority[mark:])
            # has_control=false on disarmed OFFBOARD must not bypass arm()'s
            # direct mode check: LOITER is still required BEFORE rearming.
            case.update(armed=False, mode='OFFBOARD')
            pump(.25)
            mark = len(events)
            assert invoke(RobotCommand.Request.ARM)
            assert events[mark:] == [('mode', 'AUTO.LOITER'), ('arm', True)], events[mark:]
            case['mode'] = 'AUTO.LOITER'
            pump(.15)
            for c in (RobotCommand.Request.ARM, RobotCommand.Request.REQUEST_CONTROL,
                      RobotCommand.Request.TAKEOFF):
                assert invoke(c), (name, c, events)
            # A previously successful readback never admits a regressed child.
            for invalid in (math.nan, 0.5, math.inf):
                case['value'] = invalid
                for c in (RobotCommand.Request.ARM, RobotCommand.Request.REQUEST_CONTROL,
                          RobotCommand.Request.TAKEOFF):
                    mark = len(events)
                    assert not invoke(c), (invalid, c, events)
                    assert len(events) == mark, events
            assert invoke(RobotCommand.Request.LAND)
            assert invoke(RobotCommand.Request.DISARM)
            assert sum(event[0] == 'set' for event in events) == 1, events
        else:
            assert not any(ready), (name, ready)
            for c in (RobotCommand.Request.ARM, RobotCommand.Request.REQUEST_CONTROL,
                      RobotCommand.Request.TAKEOFF):
                mark = len(events)
                assert not invoke(c), (name, c, events)
                assert len(events) == mark, (name, events)
            assert invoke(RobotCommand.Request.LAND)
            assert invoke(RobotCommand.Request.DISARM)
            if name in ('missing_state','missing_ground','armed','airborne','invalid_config','missing_child',
                        'armed_then_ground','airborne_then_ground','config_quoted','config_integer','config_wrong'):
                assert writes == 0, (name, events)
            else:
                assert writes == 1, (name, events)
        results.append(dict(case=name, writes=writes, gets=len(get_times), events=list(events)))
    finally:
        proc.terminate(); proc.wait(timeout=5)
        assert proc.returncode == 0, (name, 'unclean plugin shutdown', proc.returncode)
        pump(.2)
        if name == 'missing_child':
            set_service = node.create_service(SetParameters, 'mavros/setpoint_raw/set_parameters', set_parameters)
        if name == 'missing_get':
            get_service = node.create_service(GetParameters, 'mavros/setpoint_raw/get_parameters', get_parameters)
print(json.dumps(dict(status='PASS', domain=198, scenarios=results), indent=2))
node.destroy_node(); rclpy.shutdown()
