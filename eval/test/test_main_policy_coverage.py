"""The AirStack evaluation exposes adapters for every SAFE main-study policy."""

from pathlib import Path

import safe_core
import yaml

from safe_core.records.experiment_yaml import experiment_config_path
from safe_core.run.policies import discover_policies
from eval.agents.agents import get_profile


def test_safe_main_study_policies_have_source_adapters():
    safe_root = Path(safe_core.__file__).resolve().parent.parent
    experiments = safe_root / 'safe_navigation' / 'aerial' / 'experiments'
    config = yaml.safe_load(Path(experiment_config_path(
        'isaac_moving_primitive', str(experiments))).read_text())
    source_names = config['agents']
    discovered = discover_policies(str(Path(__file__).resolve().parents[1] / 'agents'))
    assert source_names
    for name in source_names:
        assert name in discovered, name
        profile = get_profile(name)
        assert profile['name'] == name
        assert profile['source'] == 'safe'
        assert profile['control_mode'] in ('velocity', 'setpoint', 'attitude_thrust')


def test_airstack_baselines_are_not_safe_policies():
    for name in ('droan', 'airstackrandom', 'airstackaggressive', 'airstackconservative'):
        assert get_profile(name).get('source') != 'safe'
        assert 'SAFE_EVAL_STATE_HZ' not in get_profile(name)['agent_env']


def test_learned_camera_contract_reaches_launch_profile():
    for name in ('diffphysdrone', 'depthnav', 'diffaero', 'vitfly', 'agileautonomy'):
        env = get_profile(name)['agent_env']
        assert int(env['SAFE_POV_RES']) > 0
        assert float(env['SAFE_POV_HFOV_DEG']) > 0
