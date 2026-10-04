"""The AirStack evaluation exposes adapters for every SAFE main-study policy."""

from pathlib import Path

import safe_core
import yaml

from safe_core.utils.benchmark_utils import discover_policies
from eval.agents.agents import get_profile


def test_safe_main_study_policies_have_source_adapters():
    safe_root = Path(safe_core.__file__).resolve().parent.parent
    config = yaml.safe_load((safe_root / 'integrations' / 'aerial_nav' /
                             'experiments' / 'main_moving.yaml').read_text())
    source_names = config['agents']
    discovered = discover_policies(str(Path(__file__).resolve().parents[1] / 'agents'))
    assert len(source_names) == 16
    for source_name in source_names:
        air_name = 'safe' + source_name
        assert air_name in discovered, source_name
        profile = get_profile(air_name)
        assert profile['name'] == air_name
        assert profile['control_mode'] in ('velocity', 'setpoint', 'attitude_thrust')


def test_learned_camera_contract_reaches_launch_profile():
    for name in ('safediffphysdrone', 'safedepthnav', 'safediffaero',
                 'safevitfly', 'safeagileautonomy'):
        env = get_profile(name)['agent_env']
        assert int(env['SAFE_POV_RES']) > 0
        assert float(env['SAFE_POV_HFOV_DEG']) > 0
