"""
interpolate_domain_bridge.launch.py
====================================
Launches a domain_bridge node using a YAML config file that supports variable
interpolation and config inheritance before being passed to the node.

Supported substitution syntax (mirrors ROS 2 launch XML syntax):
  $(env  VAR_NAME)            – replaced with the value of the environment variable VAR_NAME.
                                Raises RuntimeError if the variable is not set.
  $(var  VAR_NAME)            – replaced with the value supplied via the 'args' launch argument.
                                Raises RuntimeError if the variable was not provided.
  $(find-pkg-share PKG_NAME)  – replaced with the share directory path of the given ROS 2 package.
                                Raises RuntimeError if the package is not found.

Config inheritance via 'extends:':
  A YAML config file may contain a top-level 'extends:' key whose value is a path to
  another YAML config file (typically using $(find-pkg-share) to locate it).  The base
  file is loaded first (recursively, so chains are supported), and then the keys in the
  extending file are deep-merged on top of it.  Colliding keys are overwritten by the
  extending file.  The 'extends:' key itself is stripped from the final merged config.

  Example:
    extends: "$(find-pkg-share my_pkg)/config/base_bridge.yaml"
    topics:
      extra_topic:
        type: std_msgs/msg/String
        from_domain: 0
        to_domain: 1

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

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_find_pkg_share(content):
    """Replace all $(find-pkg-share PKG) tokens with the package's share directory path."""
    def replace_find_pkg(match):
        pkg_name = match.group(1)
        try:
            return get_package_share_directory(pkg_name)
        except Exception:
            raise RuntimeError(
                f"interpolate_domain_bridge: ROS 2 package '{pkg_name}' not found"
            )
    return re.sub(r'\$\(find-pkg-share\s+([\w-]+)\)', replace_find_pkg, content)


def _deep_merge(base, override):
    """Return a new dict that is *override* deep-merged on top of *base*.

    Merge rules:
    - Both values are dicts  → merge recursively (override wins on collisions).
    - Both values are lists  → concatenate (base entries first, then override
                               entries that are not already present in base).
    - All other cases        → override value wins outright.
    """
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        elif key in result and isinstance(result[key], list) and isinstance(value, list):
            # Append override entries that don't already exist in the base list
            merged = list(result[key])
            for item in value:
                if item not in merged:
                    merged.append(item)
            result[key] = merged
        else:
            result[key] = value
    return result


def _load_and_merge_config(config_file):
    """Load a domain bridge YAML config, recursively resolving 'extends:' chains.

    Processing order for each file:
      1. Read raw content.
      2. Resolve $(find-pkg-share) tokens so that 'extends:' paths can be opened.
      3. Parse YAML.
      4. If an 'extends:' key is present, load the base file recursively and
         deep-merge the current file's keys on top of the base (current wins).
      5. Return the merged dict (without the 'extends:' key).

    Note: $(env) and $(var) tokens are intentionally left unresolved here so
    that they can be substituted as a final step after all files are merged.
    """
    with open(config_file, 'r') as f:
        content = f.read()

    # Resolve $(find-pkg-share) so paths in 'extends:' can be followed
    content = _resolve_find_pkg_share(content)

    data = yaml.safe_load(content)
    if data is None:
        data = {}

    if 'extends' in data:
        base_file = data.pop('extends')
        base_data = _load_and_merge_config(base_file)
        data = _deep_merge(base_data, data)

    return data


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

    # Load, inherit (extends:), and deep-merge all config files into one dict
    merged_data = _load_and_merge_config(config_file)

    # Serialize back to a YAML string so token substitution can be applied uniformly
    content = yaml.dump(merged_data, default_flow_style=False, allow_unicode=True)

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
        LogInfo(msg=f"[domain_bridge] Final interpolated config:\n{content}"),
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
                'Supports $(find-pkg-share PKG), $(env ENV_VAR), $(var key) substitution '
                'syntax and an "extends:" key for config inheritance.'
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
