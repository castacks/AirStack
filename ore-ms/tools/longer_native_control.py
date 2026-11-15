"""Native first-write RGB control for the frozen, exposed 600-group cohort."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from longer_fixed_thermal import REC, path_stats, read, sha, write
from selected_motion_trajectory import rigid

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/longer-native-control/run-001'
PREVIOUS = ROOT / 'results/longer-fixed-thermal/run-001'


def pose(values):
    v = np.asarray(values, dtype=float)
    assert v.shape == (7,) and np.isfinite(v).all()
    assert abs(np.linalg.norm(v[3:]) - 1) < 1e-5
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(v[3:]).as_matrix()
    t[:3, 3] = v[:3]
    return t


def capture():
    OUT.mkdir(parents=True, exist_ok=False)
    receipts = []
    basis = np.eye(4)
    basis[:3, :3] = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]
    for condition in ['day', 'night', 'rain']:
        source_path = REC / 'inputs' / condition / 'source-manifest.json'
        source = read(source_path)
        event_path = REC / 'captures' / condition / 'rgb/arbiter-events/events.jsonl'
        events = {}
        for line in event_path.read_text().splitlines():
            event = json.loads(line)
            if event['kind'] == 'motion_result_written' and event['first_write_for_submission']:
                key = (event['from_idx'], event['frame_idx'])
                assert key not in events
                events[key] = event
        rows = []
        checks = 0
        for i in range(599):
            event = events.get((i, i + 1))
            result = {'status': 'UNAVAILABLE_NATIVE_FIRST_WRITE', 'transform': None}
            if event is not None:
                expected = [source['records'][j]['cameras']['rgb']['left']['recorded_time_ns']
                            for j in [i, i + 1]]
                assert [event['time0_ns'], event['time1_ns']] == expected
                if not event['original_need_interp'] and event['derived_camera_delta_xyzw'] is not None:
                    delta = pose(event['derived_camera_delta_xyzw'])
                    reconstructed = np.linalg.solve(pose(event['parent_pose_camera_map_xyzw']),
                                                    pose(event['committed_pose_camera_map_xyzw']))
                    assert np.allclose(delta, reconstructed, rtol=0, atol=1e-8)
                    optical = basis @ delta @ basis.T
                    result = {'status': 'NATIVE_FIRST_WRITE', 'transform': optical.tolist(),
                              'event_sequence': event['sequence'], 'stage': event['stage']}
                    checks += 1
            rows.append({'pair': i, 'methods': {'native_rgb': result}})
        target = OUT / f'predictions-{condition}.json'
        write(target, {'rows': rows, 'references_read': False, 'sequence': source['sequence']})
        receipts.append({'condition': condition, 'prediction_sha256': sha(target),
                         'source_sha256': sha(source_path), 'events_sha256': sha(event_path),
                         'independent_delta_checks': checks})
    write(OUT / 'prediction-lock.json', {'rows': receipts, 'source_sha256': sha(__file__),
          'references_read': False, 'contract': 'supplied_same_ID_no_duration_scaling'})


def score():
    lock = read(OUT / 'prediction-lock.json')
    previous_lock = read(PREVIOUS / 'prediction-lock.json')
    previous = read(PREVIOUS / 'scores.json')['summary']
    summary = {}
    for receipt in lock['rows']:
        c = receipt['condition']
        p = OUT / f'predictions-{c}.json'
        assert sha(p) == receipt['prediction_sha256']
        prior_pin = next(x for x in previous_lock['rows'] if x['condition'] == c)
        assert sha(PREVIOUS / f'predictions-{c}.json') == prior_pin['prediction_sha256']
        rows = read(p)['rows']
        refs = [rigid(np.loadtxt(REC / 'references' / c / f'{i:06}.txt').reshape(4, 4))[0]
                for i in range(600)]
        def stats(start, end):
            result = path_stats(rows, refs, 'native_rgb', start, end)
            if result['status'] == 'COMPLETE':
                result.pop('positions'); result.pop('reference')
            return result
        native = {'available': sum(r['methods']['native_rgb']['transform'] is not None for r in rows),
                  'full_path': stats(0, 599),
                  'blocks': [{'start': i, **stats(i, i + 99)} for i in range(0, 600, 100)]}
        summary[c] = {'native_rgb': native,
                      'replay_rgb': previous[c]['methods']['rgb'],
                      'fixed_point1': previous[c]['methods']['fixed_point1']}
    # Same rain recording, same first100 IDs and original native control.
    old = read(ROOT / 'results/fixed-thermal-trajectory/run-001/scores.json')
    expected = old['summary']['rain']['methods']['native_rgb']
    actual = summary['rain']['native_rgb']['blocks'][0]
    for key in ['position_rmse_m', 'orientation_rmse_deg', 'final_position_error_m']:
        assert abs(actual[key] - expected[key]) < 1e-8, (key, actual[key], expected[key])
    write(OUT / 'scores.json', {'summary': summary, 'prediction_lock_sha256': sha(OUT / 'prediction-lock.json'),
          'rain_first100_reproduces': True, 'scope': 'Saved native first-write increments, exposed cohort; no new native replay or timing correction.'})
    for c, methods in summary.items():
        print(c, {m: {'available': x.get('available'), 'path': x['full_path']}
                  for m, x in methods.items()})


if __name__ == '__main__':
    import sys
    {'capture': capture, 'score': score}[sys.argv[1]]()
