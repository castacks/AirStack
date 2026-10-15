"""CPU checks for the source-backed domain and scenario contract."""

from pathlib import Path

import numpy as np
import safe_core
import yaml

from eval.sim.environment_config import get_domain_registry
from eval.sim.safe_source_scene import sample_scenario


def test_main_procedural_domains_come_from_safe_registry():
    from safe_core.utils.benchmark_io import experiment_config_path
    experiments = (Path(safe_core.__file__).resolve().parent.parent
                   / 'integrations' / 'navigation' / 'aerial' / 'experiments')
    main = yaml.safe_load(Path(experiment_config_path(
        'main_moving', str(experiments))).read_text())
    domains = get_domain_registry()
    # Photoreal stages register only where their bake exists, so only the procedural
    # ones are guaranteed on every machine.
    for name in main['env_domains']:
        if name in domains:
            assert domains[name].source_domain == name
    for name in ('primitive_sparse', 'primitive_dense', 'floating', 'empty'):
        assert domains[name].source_domain == name


def test_source_scenario_seed_and_goal_distance():
    import integrations.navigation.common.config.domains_isaac  # noqa: F401
    from integrations.navigation.common.config.domains import (
        get_domain_registry as safe_domains, scale_procedural_domains)

    scale_procedural_domains(60, sim_backend='isaac')
    cfg = safe_domains('isaac')['primitive_sparse']
    kw = dict(seed=213, goal_distance=60, perturbation_seed=341,
              humans=True, agent_radius=.31, obstacle_radius=.3, scenario_index=0)
    first, _, difficulty = sample_scenario(cfg, **kw)
    second, _, _ = sample_scenario(cfg, **kw)
    # A procedural arena has no occupancy grid, so no density band applies.
    assert difficulty is None
    for key in ('agent_location', 'target_location', 'static_cylinder_locations',
                'dynamic_obstacle_locations', 'dynamic_obstacle_velocities'):
        np.testing.assert_array_equal(first[key], second[key])
    assert abs(np.linalg.norm(first['target_location'][:2]
                              - first['agent_location'][:2]) - 60) < .01
