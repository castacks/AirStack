"""Start the gamepad teleop on its own, before (and independently of) the
swarm commander.

    # Real drone: sticks -> /svg/drone_1/teleop_command, using the device and
    # tuning from the config's safe_teleop block (teleop_controller: xbox_usb).
    ros2 launch svg_ground_control teleop.launch.py \
        config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/teleop_real.yaml

    # Sim squeeze with a hand-flown intruder:
    ros2 launch svg_ground_control teleop.launch.py drone:=drone_3

Once a second, this terminal prints the stick reading and the velocity being
published (print_hz). Move the sticks: the numbers must follow. Only then
start the commander in another terminal:

    ros2 launch svg_ground_control ground_control.launch.py config:=<same> ...

Nothing here talks to the drone directly. safe_teleop publishes velocities
that the commander forwards only after it is running, the drone is listed in
its `teleop_drones`, and /swarm_commander/start has been called. A drone that
is not yet up only means `vz` stays at zero ("odometry stale"): the altitude
hold needs the drone's height, the horizontal axes do not.

What starts: the driver node(s) for `teleop_controller` (xbox_usb -> joy_node)
and one safe_teleop for `drone`. The registry is
svg_ground_control/safe_teleop/controllers.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from svg_ground_control.safe_teleop.launch_helpers import (
    config_teleop_drones, resolve_controller, teleop_actions)


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)
    drone = LaunchConfiguration('drone').perform(context).strip()
    controller = LaunchConfiguration('teleop_controller').perform(context)
    print_hz = float(LaunchConfiguration('print_hz').perform(context))

    listed = config_teleop_drones(config_path)
    if not drone:
        if not listed:
            raise RuntimeError(
                f'{config_path} has no teleop_drones and no drone:= was given; '
                'pass drone:=<name> (and list it in the commander\'s '
                'teleop_drones, or it will never be forwarded)')
        drone = listed[0]
    others = [n for n in listed if n != drone]

    profile = resolve_controller(config_path, controller)   # KeyError lists names
    return teleop_actions(config_path, drone, profile,
                          extra_teleop_params={'print_hz': print_hz},
                          others=others)


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config', 'swarm_sim.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='The same swarm config the commander will use (its '
                        'safe_teleop block supplies teleop_controller and tuning)'),
        DeclareLaunchArgument(
            'drone', default_value='',
            description='Drone to fly. Empty = first entry of the config\'s '
                        'teleop_drones'),
        DeclareLaunchArgument(
            'teleop_controller', default_value='',
            description='Input device, an entry of safe_teleop/controllers.py '
                        "(xbox_usb). Empty = the config's value, else xbox_usb"),
        DeclareLaunchArgument(
            'print_hz', default_value='1.0',
            description='How often safe_teleop prints the stick reading and '
                        'the published velocity (0 = silent)'),
        OpaqueFunction(function=launch_setup),
    ])
