"""Reproduce the matched bundle / independent-greedy study specifications."""
from copy import deepcopy
import json
from pathlib import Path
import random
import re

import yaml

HERE = Path(__file__).resolve().parent
GT_COUNTS = {  # existing 300s GT analysis, 3r_full_comms/per_run.json
    'DowntownWest': 12, 'ModernCityDowntown': 14, 'RetroNeighborhood': 24,
    'FireAcademy': 10, 'ConstructionSite': 7, 'AbandonedFactory': 7,
    'MilitaryBase': 7, 'Shipyard': 50,
}
ARMS = [('auction_l1', 'auction', 1),
        ('auction_l6', 'auction', 6), ('auction_l12', 'auction', 12),
        ('greedy_l1', 'independent_greedy', 1)]


def main():
    template = yaml.safe_load((HERE / 'raven_3robot_full_no_either_s1_20261012.yaml').read_text())
    original = yaml.safe_load((HERE / 'multi_raven_mission.yaml').read_text())
    positions = {(e['RESULTS_SCENE'], int(re.search(r'_s([123])', e['name'])[1])):
                 e['SPAWN_CONFIGS'] for e in original['environments']}
    scenes = template['environments']
    manifest = {'experiment': 'bundle_study_20261014', 'robots': 3,
                'comms_range_m': 5000, 'horizon_sim_s': 300,
                'workflow': 'osmo/workflows/airstack-mission-6h.yaml',
                'pool': 'dsta', 'priority': 'HIGH', 'jobs': [], 'scenes': []}
    for env in scenes:
        pts = env['search_area']['points']
        area = abs(sum(p['x'] * q['y'] - q['x'] * p['y']
                       for p, q in zip(pts, pts[1:] + pts[:1]))) / 2
        n = GT_COUNTS[env['RESULTS_SCENE']]
        manifest['scenes'].append({'scene': env['RESULTS_SCENE'],
                                  'search_polygon_area_m2': area,
                                  'query_target_count': n,
                                  'targets_per_hectare': n / (area / 10000)})
    for start in (1,):
        order = list(range(len(scenes)))
        random.Random(20261014 + start).shuffle(order)
        # All arms use identical scene order and matched s1 starts.
        for label, mode, limit in ARMS:
            spec = deepcopy(template)
            spec['name'] = f'raven_bundle_{label}_s{start}_20261014'
            # Reuse the existing NAS base; mission name isolates every arm/start.
            spec['nas_dest'] = '/volume4/dsta/raven-ablations/'
            spec['env'].update(RAVEN_SHARE_RAYS='true', RAVEN_SHARE_BBS='true',
                               RAVEN_ASSIGNMENT_MODE=mode)
            spec['environments'] = [deepcopy(scenes[i]) for i in order]
            for e in spec['environments']:
                e['name'] = f"{e['RESULTS_SCENE'].lower()}_s{start}"
                e['SPAWN_CONFIGS'] = positions[e['RESULTS_SCENE'], start]
                assert len(json.loads(e['SPAWN_CONFIGS'])) == 3
            for step in spec['steps']:
                if step.get('action', {}).get('task') == 'semantic_search':
                    step['action']['goal']['bundle_len'] = limit
            spec['record']['robot_topics'] += [
                '/robot_{n}/debug/auction_solve',
                '/robot_{n}/odometry_conversion/odometry']
            path = HERE / (spec['name'] + '.yaml')
            path.write_text(yaml.safe_dump(spec, sort_keys=False, width=100))
            manifest['jobs'].append({'mission': f'osmo/missions/{path.name}',
                                     'arm': label, 'assignment_mode': mode,
                                     'bundle_len': limit, 'start': start,
                                     'trials': [e['name'] for e in spec['environments']]})
    assert len(manifest['jobs']) == 4
    (HERE / 'bundle_study_20261014.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
