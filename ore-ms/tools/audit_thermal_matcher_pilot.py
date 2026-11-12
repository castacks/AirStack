"""Recompute pilot errors from saved transforms without importing its scorer."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/thermal-matcher-pilot/run-001'


def audit():
    scores = json.loads((OUT/'vo-scores.json').read_text())
    reference_rows = json.loads((ROOT/'results/native-information-weights/run-001/scores-rain.json').read_text())['rows']
    refs = {r['pair']: np.array(r['reference_delta']) for r in reference_rows}
    expected_common = [i for i in range(19) if all(
        rows[i]['transform'] is not None for rows in scores['rows_by_arm'].values())]
    assert scores['common_pair_ids'] == expected_common
    count = 0
    for arm, rows in scores['rows_by_arm'].items():
        errors = {}
        assert scores['coverage'][arm] == sum(r['transform'] is not None for r in rows)
        for row in rows:
            if row['transform'] is None:
                assert row['errors'] is None
                continue
            p, r = np.array(row['transform']), refs[row['pair']]
            rotation = r[:3, :3].T@p[:3, :3]
            sine = np.linalg.norm([rotation[2, 1]-rotation[1, 2],
                                  rotation[0, 2]-rotation[2, 0],
                                  rotation[1, 0]-rotation[0, 1]])/2
            angle = np.degrees(np.arctan2(sine, (np.trace(rotation)-1)/2))
            translation = np.linalg.norm(p[:3, 3]-r[:3, 3])
            errors[row['pair']] = {'translation_m': translation, 'rotation_deg': angle}
            for key, value in errors[row['pair']].items():
                np.testing.assert_allclose(row['errors'][key], value, rtol=1e-9, atol=1e-8)
                count += 1
        for metric in ['translation_m', 'rotation_deg']:
            a = np.array([errors[i][metric] for i in expected_common])
            if not len(a):
                continue
            values = {'mean': a.mean(), 'median': np.median(a),
                      'rmse': np.sqrt(np.dot(a, a)/len(a)), 'p95': np.percentile(a, 95),
                      'maximum': a.max()}
            for key, value in values.items():
                np.testing.assert_allclose(scores['common_metrics'][arm][metric][key], value, rtol=1e-9, atol=1e-8)
                count += 1
    for a, b in zip(scores['rows_by_arm']['original'], scores['rows_by_arm']['passthrough']):
        np.testing.assert_array_equal(a['transform'], b['transform'])
    flow_files = sorted((OUT/'flow').glob('*.npz'))
    assert len(flow_files) == 19
    for path in flow_files:
        with np.load(path, allow_pickle=False) as data:
            assert data['original'].shape == data['replacement'].shape == (1, 2, 256, 640)
            assert data['covisibility'].shape == (1, 256, 640)
            assert all(np.isfinite(data[k]).all() for k in data.files)
            assert np.all((data['covisibility'] >= 0)&(data['covisibility'] <= 1))
    decision = json.loads((OUT/'decision.json').read_text())
    m = scores['common_metrics']
    checks = {'common_support_nonempty': len(expected_common) > 0,
              'coverage_not_worse': scores['coverage']['adapted'] >= scores['coverage']['original']}
    for key in ['mean', 'rmse']:
        checks['translation_'+key+'_improves'] = m['adapted']['translation_m'][key] < m['original']['translation_m'][key]
        checks['rotation_'+key+'_not_worse'] = m['adapted']['rotation_deg'][key] <= m['original']['rotation_deg'][key]
    assert checks == decision['checks']
    assert decision['decision'] == ('ADVANCE_TO_SEPARATE_RECORDING_COMPARISON' if all(checks.values()) else 'STOP_THIS_CONFIGURATION')
    result = {'status': 'PASS', 'independent_metric_values': count,
              'exact_passthrough_intervals': 19, 'finite_flow_packets': len(flow_files),
              'scope': 'Saved motion/metric/decision and flow-schema audit; does not rerun networks or independently validate physical references'}
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)


if __name__ == '__main__':
    audit()
