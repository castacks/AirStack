"""
interpolate_dds_router.launch.py
=================================
Launches an eProsima DDS Router process using a YAML config file that supports
variable interpolation and config inheritance before being passed to the router.

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
    extends: "$(find-pkg-share my_pkg)/config/base_router.yaml"
    participants:
      - name: "extra"
        kind: "local"
        domain: 5

Merge-control YAML tags:
  !override  – applied to a list or dict value in the extending file; the tagged
               value completely replaces the corresponding base value instead of
               being appended/merged.  Example:
                 allowlist: !override
                   - name: "rt/my_robot/only_this_topic"
  !reset     – applied to any value; removes the key from the merged result
               entirely.  Example:
                 allowlist: !reset

Launch arguments:
  config_file  (required)
      Absolute path to the DDS Router YAML config file.
      Typically resolved with find-pkg-share in the calling launch file, e.g.:
        $(find-pkg-share autonomy_bringup)/config/my_router.yaml

  args  (optional, default: "")
      Space-separated key:=value pairs that resolve $(var key) tokens in the
      config file, e.g.:  "gcs_domain:=0  robot_domain:=5"

Example (XML caller):
  <include file="$(find-pkg-share autonomy_bringup)/launch/interpolate_dds_router.launch.py">
    <arg name="config_file" value="$(find-pkg-share autonomy_bringup)/config/router.yaml" />
    <arg name="args"        value="gcs_domain:=0" />
  </include>
"""

import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


# ── Merge-control sentinel types ─────────────────────────────────────────────

class _OverrideValue:
    """Wraps a value tagged with !override; replaces the base value outright."""
    __slots__ = ('value',)

    def __init__(self, value):
        self.value = value


class _ResetValue:
    """Sentinel for !reset; causes the key to be removed from the merged result."""
    __slots__ = ()


def _make_loader():
    """Return a yaml.SafeLoader subclass that recognises !override and !reset tags."""
    class _Loader(yaml.SafeLoader):
        pass

    def _override_ctor(loader, node):
        if isinstance(node, yaml.SequenceNode):
            value = loader.construct_sequence(node, deep=True)
        elif isinstance(node, yaml.MappingNode):
            value = loader.construct_mapping(node, deep=True)
        else:
            value = loader.construct_scalar(node)
        return _OverrideValue(value)

    def _reset_ctor(loader, node):
        return _ResetValue()

    _Loader.add_constructor('!override', _override_ctor)
    _Loader.add_constructor('!reset', _reset_ctor)
    return _Loader


def _normalize(data):
    """Unwrap/strip any leftover _OverrideValue/_ResetValue markers from the tree."""
    if isinstance(data, _OverrideValue):
        return _normalize(data.value)
    if isinstance(data, _ResetValue):
        return None
    if isinstance(data, dict):
        return {
            k: _normalize(v)
            for k, v in data.items()
            if not isinstance(v, _ResetValue)
        }
    if isinstance(data, list):
        return [_normalize(item) for item in data if not isinstance(item, _ResetValue)]
    return data


# ─────────────────────────────────────────────────────────────────────────────


def _resolve_find_pkg_share(content):
    """Replace all $(find-pkg-share PKG) tokens with the package's share directory path."""
    def replace_find_pkg(match):
        pkg_name = match.group(1)
        try:
            return get_package_share_directory(pkg_name)
        except Exception:
            raise RuntimeError(
                f"interpolate_dds_router: ROS 2 package '{pkg_name}' not found"
            )
    return re.sub(r'\$\(find-pkg-share\s+([\w-]+)\)', replace_find_pkg, content)


def _deep_merge(base, override):
    """Return a new dict that is *override* deep-merged on top of *base*.

    Merge rules:
    - Value is _ResetValue    → remove the key from the result entirely.
    - Value is _OverrideValue → replace the base value outright (no merge).
    - Both values are dicts   → merge recursively (override wins on collisions).
    - Both values are lists   → concatenate (base entries first, then override
                                entries that are not already present in base).
    - All other cases         → override value wins outright.
    """
    result = base.copy()
    for key, value in override.items():
        if isinstance(value, _ResetValue):
            result.pop(key, None)
        elif isinstance(value, _OverrideValue):
            result[key] = value.value
        elif key in result and isinstance(result[key], dict) and isinstance(value, dict):
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
    """Load a DDS Router YAML config, recursively resolving 'extends:' chains.

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

    data = yaml.load(content, Loader=_make_loader())
    if data is None:
        data = {}

    if 'extends' in data:
        base_file = data.pop('extends')
        base_data = _load_and_merge_config(base_file)
        data = _deep_merge(base_data, data)

    return _normalize(data)


def launch_dds_router(context, *args, **kwargs):
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
                f"interpolate_dds_router: environment variable '{var_name}' is not set"
            )
        return value

    content = re.sub(r'\$\(env\s+([\w]+)\)', replace_env, content)

    # Interpolate $(var VAR_NAME)
    def replace_var(match):
        var_name = match.group(1)
        if var_name not in variables:
            raise RuntimeError(
                f"interpolate_dds_router: variable '{var_name}' was not provided in 'args'"
            )
        return variables[var_name]

    content = re.sub(r'\$\(var\s+([\w]+)\)', replace_var, content)

    # Write interpolated content to a temporary file so ddsrouter can read it
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='dds_router_'
    )
    tmp.write(content)
    tmp.close()

    return [
        LogInfo(msg=f"[dds_router] Final interpolated config:\n{content}"),
        ExecuteProcess(
            cmd=['ddsrouter', '-c', tmp.name],
            env={
                **os.environ,
                # ddsrouter is installed under /usr/local and needs its runtime libs.
                # Scope this path to ddsrouter only to avoid perturbing ROS 2 RMW loading.
                'LD_LIBRARY_PATH': '/usr/local/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
            },
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description=(
                'Absolute path to the DDS Router YAML config file. '
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
        OpaqueFunction(function=launch_dds_router),
    ])
