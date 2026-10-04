"""CPU checks for the source-backed domain and scenario contract."""

import numpy as np

from eval.sim.environment_config import get_domain_registry
from eval.sim.safe_source_scene import sample_scenario


MAIN_DOMAINS = (
    'dynamic_sparse', 'dynamic_dense', 'forest_dense', 'forest_sparse',
    'construction', 'snowy_village', 'abandoned_factory',
    'retro_neighborhood', 'conifer_forest', 'english_college', 'bazaar',
)


def test_all_main_domains_come_from_safe_registry():
    domains = get_domain_registry()
    assert all(domains[name].source_domain == name for name in MAIN_DOMAINS)


def test_source_scenario_seed_and_goal_distance():
    import integrations.aerial_nav.config.domains_isaac  # noqa: F401
    from integrations.aerial_nav.config.domains import (
        get_domain_registry as safe_domains, scale_procedural_domains)

    scale_procedural_domains(60, sim_backend='isaac')
    cfg = safe_domains('isaac')['dynamic_sparse']
    kw = dict(seed=213, goal_distance=60, perturbation_seed=341,
              humans=True, agent_radius=.31, obstacle_radius=.3)
    first, _ = sample_scenario(cfg, **kw)
    second, _ = sample_scenario(cfg, **kw)
    for key in ('agent_location', 'target_location', 'static_cylinder_locations',
                'dynamic_obstacle_locations', 'dynamic_obstacle_velocities'):
        np.testing.assert_array_equal(first[key], second[key])
    assert abs(np.linalg.norm(first['target_location'][:2]
                              - first['agent_location'][:2]) - 60) < .01
