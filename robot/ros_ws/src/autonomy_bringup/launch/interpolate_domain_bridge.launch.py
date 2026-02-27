"""
interpolate_domain_bridge.launch.py
====================================
Launches a domain_bridge node using a YAML config file that supports variable
interpolation before being passed to the node.

Supported substitution syntax (mirrors ROS 2 launch XML syntax):
  $(env  VAR_NAME)   – replaced with the value of the environment variable VAR_NAME.
                       Raises RuntimeError if the variable is not set.
  $(var  VAR_NAME)   – replaced with the value supplied via the 'args' launch argument.
                       Raises RuntimeError if the variable was not provided.

Launch arguments:
  config_file  (required)
      Absolute path to the domain bridge YAML config file.
      Typically resolved with find-pkg-share in the calling launch file, e.g.:
        $(find-pkg-share autonomy_bringup)/config/my_bridge.yaml

  args  (optional, default: "")
      Space-separated key:=value pairs that resolve $(var key) tokens in the
      config file, e.g.:  "gcs_domain:=0  robot_domain:=5"

Example (XML caller):
  <include file="$(find-pkg-share autonomy_bringup)/launch/interpolate_domain_bridge.launch.py">
    <arg name="config_file" value="$(find-pkg-share autonomy_bringup)/config/bridge.yaml" />
    <arg name="args"        value="gcs_domain:=0" />
  </include>
"""

import os
import re
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_domain_bridge(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    args_str = LaunchConfiguration('args').perform(context)

    # Parse args string: space-separated "key:=value" pairs, e.g. "gcs_domain:=0 foo:=bar"
    variables = {}
    if args_str.strip():
        for token in args_str.strip().split():
            if ':=' in token:
                key, value = token.split(':=', 1)
                variables[key.strip()] = value.strip()

    # Read the config file
    with open(config_file, 'r') as f:
        content = f.read()

    # Interpolate $(env VAR_NAME)
    def replace_env(match):
        var_name = match.group(1)
        value = os.environ.get(var_name)
        if value is None:
            raise RuntimeError(
                f"interpolate_domain_bridge: environment variable '{var_name}' is not set"
            )
        return value

    content = re.sub(r'\$\(env\s+([\w]+)\)', replace_env, content)

    # Interpolate $(var VAR_NAME)
    def replace_var(match):
        var_name = match.group(1)
        if var_name not in variables:
            raise RuntimeError(
                f"interpolate_domain_bridge: variable '{var_name}' was not provided in 'args'"
            )
        return variables[var_name]

    content = re.sub(r'\$\(var\s+([\w]+)\)', replace_var, content)

    # Write interpolated content to a temporary file so domain_bridge can read it
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='domain_bridge_'
    )
    tmp.write(content)
    tmp.close()

    return [
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            arguments=[tmp.name],
            output='screen',
            respawn=True,
            respawn_delay=1.0,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description=(
                'Absolute path to the domain bridge YAML config file. '
                'Supports $(env ENV_VAR) and $(var key) substitution syntax.'
            ),
        ),
        DeclareLaunchArgument(
            'args',
            default_value='',
            description=(
                'Space-separated key:=value pairs used to resolve $(var key) '
                'substitutions in the config file, e.g. "gcs_domain:=0 foo:=bar".'
            ),
        ),
        OpaqueFunction(function=launch_domain_bridge),
    ])
