## Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


# Declare all launch arguments corresponding to the bash script options
launch_args = [
    DeclareLaunchArgument('version', default_value='4.1.0', description='Specify the version of Isaac Sim to use. Isaac Sim will be run from default install root folder for the specified version. Leave empty to use latest version of Isaac Sim.'),
    
    DeclareLaunchArgument('install_path', default_value='', description='If Isaac Sim is insalled in a non-default location, provide a specific path to Isaac Sim installation root folder. (If defined, "version" parameter will be ignored)'),
    
    DeclareLaunchArgument('use_internal_libs', default_value='false', description='Set to true if you wish to use internal ROS libraries shipped with Isaac Sim.'),
    
    DeclareLaunchArgument('dds_type', default_value='fastdds', description='Set to "fastdds" or "cyclonedds" (Cyclone only supported for ROS Humble) to run Isaac Sim with a specific dds type.'),
    
    DeclareLaunchArgument('gui', default_value='', description='Provide the path to a usd file to open it when starting Isaac Sim in standard gui mode. If left empty, Isaac Sim will open an empty stage in standard gui mode.'),
    
    DeclareLaunchArgument('standalone', default_value='', description='Provide the path to the python file to open it and start Isaac Sim in standalone workflow. If left empty, Isaac Sim will open an empty stage in standard Gui mode.'),
    
    DeclareLaunchArgument('play_sim_on_start', default_value='false', description='If enabled and Isaac Sim will start playing the scene after it is loaded. (Only applicable when in standard gui mode and loading a scene)'),
    
    DeclareLaunchArgument('ros_distro', default_value='humble', description='Provide ROS version to use. Only Humble and Foxy is supported.'),
    
    DeclareLaunchArgument('ros_installation_path', default_value='', description='If ROS is installed in a non-default location (as in not under /opt/ros/), provide the path to your main setup.bash file for your ROS install. (/path/to/custom/ros/install/setup.bash)'),

    DeclareLaunchArgument('headless', default_value='', description='Set to "native" or "webrtc" to run Isaac Sim with different headless modes, if left blank, Isaac Sim will run in the regular GUI workflow. This parameter can be overridden by "standalone" parameter.'),

]

# List of parameters to check
parameters_to_check = [
    'version', 'install_path', 'use_internal_libs', 'dds_type',
    'standalone', 'play_sim_on_start', 'ros_distro', 'ros_installation_path', 'gui'
]

def launch_setup(context):

    def add_parameter_argument(param_name, command_list):
        param_value = LaunchConfiguration(param_name).perform(context) # Here you'll get the runtime config value

        if param_value != '':
            if param_value == 'true':
                return command_list + ['--' + param_name.replace('_', '-')]
            elif param_value == 'false':
                return command_list
            else:
                return command_list + ['--' + param_name.replace('_', '-'), param_value,]
        else:
            return command_list

    bash_command = []

    # Add parameters to the command list if they are set
    for param in parameters_to_check:
        bash_command = add_parameter_argument(param, bash_command)
        
    # Run isaac sim as a ROS2 node with default parameters. Parameters can be overridden here or via launch arguments from other launch files. 
    isaacsim_node = Node(
        package='isaacsim', executable='run_isaacsim.py',
        name='isaacsim', output="screen", 
        parameters=[{
            'version': LaunchConfiguration('version'),
            'install_path': LaunchConfiguration('install_path'),
            'use_internal_libs': LaunchConfiguration('use_internal_libs'),
            'dds_type': LaunchConfiguration('dds_type'),
            'gui': LaunchConfiguration('gui'),
            'standalone': LaunchConfiguration('standalone'),
            'play_sim_on_start': LaunchConfiguration('play_sim_on_start'),
            'ros_distro': LaunchConfiguration('ros_distro'),
            'ros_installation_path': LaunchConfiguration('ros_installation_path'),
            'headless': LaunchConfiguration('headless'),
            'isaac_args': LaunchConfiguration('isaac_args')
        }]
    )
    return [isaacsim_node]


# Create and return the launch description with all declared arguments and the execute launch_setup
def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
