"""The opt-in gossip transport must not leak into other DDS processes."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[3]
COORD = ROOT / 'common/ros_packages/coordination/coordination_bringup/launch'
MODE = ('LARGE_DATA?max_msg_size=65500&sockets_size=16MB&non_blocking=true'
        '&tcp_negotiation_timeout=50')


def load(path):
    spec = importlib.util.spec_from_file_location('transport_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def environment(action, context):
    return {perform_substitutions(context, key): perform_substitutions(context, value)
            for key, value in (action.env or action.additional_env)}


@pytest.mark.parametrize('override,expected', [(None, 'LARGE_DATA'), (MODE, MODE)])
def test_gossip_processes_scoped_environment(monkeypatch, override, expected):
    monkeypatch.setenv('ROBOT_NAME', 'robot_test')
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    monkeypatch.delenv('GOSSIP_BUILTIN_TRANSPORTS', raising=False)
    if override:
        monkeypatch.setenv('GOSSIP_BUILTIN_TRANSPORTS', override)
    context = LaunchContext()
    desc = AnyLaunchDescriptionSource(str(COORD / 'gossip.launch.xml')).get_launch_description(context)
    node = next(entity for entity in desc.entities if isinstance(entity, Node))
    assert environment(node, context)['FASTDDS_BUILTIN_TRANSPORTS'] == expected
    bridge = load(COORD / 'gcs_gossip_bridge.launch.py').generate_launch_description().entities[0]
    assert environment(bridge, context)['FASTDDS_BUILTIN_TRANSPORTS'] == expected
    assert context.environment['FASTDDS_BUILTIN_TRANSPORTS'] == 'LARGE_DATA'


@pytest.mark.parametrize('gossip_only,expected', [(False, 'LARGE_DATA'), (True, MODE)])
def test_only_gossip_router_uses_override(monkeypatch, tmp_path, gossip_only, expected):
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    monkeypatch.setenv('GOSSIP_BUILTIN_TRANSPORTS', MODE)
    config = tmp_path / 'router.yaml'
    config.write_text('participants: []\nallowlist:\n  - name: ' +
                      ('rt/gossip/peers' if gossip_only else 'rt/robot_1/odometry') + '\n')
    context = LaunchContext()
    context.launch_configurations.update(config_file=str(config), args='', log_filter='DDSROUTER')
    module = load(ROOT / 'robot/ros_ws/src/autonomy_bringup/launch/interpolate_dds_router.launch.py')
    actions = module.launch_dds_router(context)
    assert environment(actions[-1], context)['FASTDDS_BUILTIN_TRANSPORTS'] == expected
    assert context.environment['FASTDDS_BUILTIN_TRANSPORTS'] == 'LARGE_DATA'
