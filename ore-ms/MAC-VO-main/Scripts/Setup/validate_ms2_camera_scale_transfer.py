"""Independent saved-array audit; does not import the fitting implementation."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def validate(root, source, output):
    if output.exists():
        raise FileExistsError(output)
    protocol = json.loads((source / 'protocol.json').read_text())
    report = json.loads((source / 'report.json').read_text())
    checks = {'hashes': 0, 'grid_objectives': 0, 'test_risks': 0, 'historical_risks': 0, 'summary_fields': 0, 'split_and_choice_checks': 0}

    def close(a, b, group):
        assert np.isclose(a, b, rtol=1e-12, atol=1e-12), (group, a, b)
        checks[group] += 1

    paths = {'rain': 'ms2-day-cross-modal-replication/rain-control/metric',
             'day': 'ms2-day-cross-modal-replication/day/metric',
             'night': 'ms2-night-cross-modal-replication/night/metric'}
    data, old = {}, {}
    for path, expected in protocol['frozen_input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected
        checks['hashes'] += 1
    assert len(protocol['frozen_input_sha256']) == 63
    expected_grid = [{'kind': 'finite', 'log10_c': j / 20} for j in range(-120, 121)] + [{'kind': 'always_rgb'}, {'kind': 'always_thr'}]
    assert protocol['grid'] == expected_grid
    for rec, sub in paths.items():
        folder = root / 'results' / sub
        old[rec] = json.loads((folder / 'report.json').read_text())['rows']
        data[rec] = []
        assert [r['stem'] for r in old[rec]] == [f'{j:06d}' for j in range(20)]
        for row in old[rec]:
            with np.load(folder / f"metric-{row['stem']}.npz", allow_pickle=False) as f:
                data[rec].append({k: f[k] for k in f.files})

    def decisions(a, policy):
        n = int(a['both_valid_mask'].sum())
        if policy['kind'] == 'always_rgb':
            return np.full(n, 1.)
        if policy['kind'] == 'always_thr':
            return np.full(n, 0.)
        # Enumerated comparisons independently construct a decision vector.
        rgb = a['rgb_metric_variance'][a['both_valid_mask']] * 10 ** policy['log10_c']
        thr = a['thr_metric_variance'][a['both_valid_mask']]
        w = np.full(n, .5)
        w[rgb < thr] = 1.
        w[rgb > thr] = 0.
        return w

    def risk(a, anchor, power, w):
        r, t = a[f'{anchor}_rgb_error_m'] ** power, a[f'{anchor}_thr_error_m'] ** power
        # Separate selected RGB/thermal/tie sums, rather than weighted vector mean.
        return float((r[w == 1].sum() + t[w == 0].sum() + ((r[w == .5] + t[w == .5]) / 2).sum()) / len(w))

    assert set(report['folds']) == set(paths)
    for test, fold in report['folds'].items():
        train = [rec for rec in paths if rec != test]
        assert fold['train_recordings'] == train and fold['test_recording'] == test
        checks['split_and_choice_checks'] += 1
        curves = []
        for rec in train:
            curve = []
            for i, policy in enumerate(expected_grid):
                values = []
                for a in data[rec]:
                    rand = float((a['midpoint_rgb_error_m'].sum() + a['midpoint_thr_error_m'].sum()) / (2 * len(a['midpoint_rgb_error_m'])))
                    values.append(risk(a, 'midpoint', 1, decisions(a, policy)) / rand)
                value = float(sum(values) / len(values))
                close(value, fold['training_objectives'][rec][i], 'grid_objectives')
                curve.append(value)
            curves.append(curve)
        means = (np.array(curves[0]) + np.array(curves[1])) / 2
        for i, value in enumerate(means):
            close(value, fold['mean_training_objectives'][i], 'grid_objectives')
        # Independently reproduce exact production reduction for argmin tie sensitivity.
        exact = []
        for policy in expected_grid:
            rec_values = []
            for rec in train:
                frame_values = []
                for a in data[rec]:
                    w = decisions(a, policy)
                    r, t = a['midpoint_rgb_error_m'], a['midpoint_thr_error_m']
                    frame_values.append(np.mean(w*r + (1-w)*t) / np.mean((r+t)/2))
                rec_values.append(np.mean(frame_values))
            exact.append(float(np.mean(rec_values)))
        def tie(i):
            p = expected_grid[i]
            return (0, abs(p['log10_c']), p['log10_c']) if i < 241 else (1, i-241, 0)
        selected = sorted(range(243), key=lambda i: (exact[i], tie(i)))[0]
        fixed = 241 if exact[241] <= exact[242] else 242
        assert fold['selected_index'] == selected and fold['policy'] == expected_grid[selected]
        assert fold['training_fixed_policy'] == expected_grid[fixed]
        assert means[selected] <= means.min() + 1e-12
        checks['split_and_choice_checks'] += 2
        assert len(fold['rows']) == 20
        for j, (a, row, hist) in enumerate(zip(data[test], fold['rows'], old[test], strict=True)):
            assert row['stem'] == hist['stem'] == f'{j:06d}'
            assert row['availability'] == {k: hist[k] for k in row['availability']}
            w = decisions(a, expected_grid[selected])
            close(w.mean(), row['fitted_rgb_fraction'], 'test_risks')
            assert np.array_equal(decisions(a, {'kind':'finite','log10_c':0}), a['learned_propagated'])
            checks['split_and_choice_checks'] += 1
            policies = {'fitted':w, 'original':a['learned_propagated'], 'camera_scale':a['camera_scale'],
                        'always_rgb':decisions(a, expected_grid[241]), 'always_thr':decisions(a, expected_grid[242]),
                        'random':np.full(len(w),.5), 'training_fixed':decisions(a, expected_grid[fixed])}
            for anchor in ['midpoint','rgb','thr']:
                for loss, power in [('mean_l2',1),('mse',2)]:
                    rs = row['risks'][anchor][loss]
                    r, t = a[f'{anchor}_rgb_error_m']**power, a[f'{anchor}_thr_error_m']**power
                    oracle = float(sum(min(x,y) for x,y in zip(r,t,strict=True)) / len(r))
                    close(oracle, rs['oracle'], 'test_risks')
                    for name, sel in policies.items():
                        value = risk(a, anchor, power, sel)
                        close(value, rs[name], 'test_risks')
                        assert value >= oracle - 1e-10
                    old_risks = hist['scores'][anchor][loss]
                    for old_policy, new_policy in [('learned_propagated','original'),('camera_scale','camera_scale')]:
                        close(old_risks[old_policy]['uncertainty'], rs[new_policy], 'historical_risks')
                    for old_key,new_key in [('rgb','always_rgb'),('thr','always_thr'),('random','random'),('oracle','oracle')]:
                        close(old_risks['learned_propagated'][old_key],rs[new_key],'historical_risks')
        for anchor in ['midpoint','rgb','thr']:
            for loss in ['mean_l2','mse']:
                for baseline, stats in fold['summary'][anchor][loss].items():
                    ratios = sorted(row['risks'][anchor][loss]['fitted']/row['risks'][anchor][loss][baseline] for row in fold['rows'])
                    for key,value in [('min',ratios[0]),('max',ratios[-1]),('median',(ratios[9]+ratios[10])/2),('below_one',sum(x<1 for x in ratios))]:
                        close(value,stats[key],'summary_fields')
    # Ties and limiting policies verified with a synthetic equal-score case.
    a={'both_valid_mask':np.array([True,True]),'rgb_metric_variance':np.array([1.,2.]),'thr_metric_variance':np.array([1.,2.])}
    assert np.array_equal(decisions(a,{'kind':'finite','log10_c':0}),[.5,.5])
    assert np.array_equal(decisions(a,{'kind':'always_rgb'}),[1.,1.])
    assert np.array_equal(decisions(a,{'kind':'always_thr'}),[0.,0.])
    result={'status':'PASS','checks':checks,'scope':'saved-array decisions, objectives and risks; historical physical/reference validity not revalidated',
            'validator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'protocol_sha256':hashlib.sha256((source/'protocol.json').read_bytes()).hexdigest(),
            'report_sha256':hashlib.sha256((source/'report.json').read_bytes()).hexdigest()}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    validate(a.root,a.source,a.output)
