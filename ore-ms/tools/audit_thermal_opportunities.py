"""Independent numerical reconstruction; no producer/scorer imports."""
import json
from pathlib import Path

import numpy as np
from scipy.linalg import logm

ROOT = Path(__file__).resolve().parents[1]
PREVIOUS = ROOT/'results/native-information-weights/run-001'
OUT = ROOT/'results/thermal-opportunities/run-001'


def read(p):
    return json.loads(p.read_text())


def cue(r):
    if not all(e['valid'] for e in r['experts'].values()):
        return None
    a, b = (r['experts'][c] for c in ['rgb', 'thermal'])
    values = []
    for e in [a, b]:
        p = np.asarray(e['information_inverse'])
        blocks = [p[:3, :3], p[3:, 3:]]
        v = [e['diagnostic']['factors'], e['diagnostic']['residual_mean_square_px']]
        v += [np.trace(block) for block in blocks]
        v += [np.linalg.eigvalsh(block)[-1]/np.linalg.eigvalsh(block)[0] for block in blocks]
        values.append(v)
    relative = logm(np.asarray(b['transform'])@np.linalg.inv(a['transform']))
    assert np.max(np.abs(relative.imag)) < 1e-8
    relative = relative.real
    angle = np.linalg.norm([relative[2, 1], relative[0, 2], relative[1, 0]])
    return np.r_[np.log(values[1])-np.log(values[0]), np.log1p(angle), np.log1p(np.linalg.norm(relative[:3, 3]))]


def audit():
    cases = ['day', 'night', 'rain']
    raw = {c: read(PREVIOUS/'raw'/f'{c}.json') for c in cases}
    old = {c: read(PREVIOUS/f'scores-{c}.json') for c in cases}
    summary = read(OUT/'summary.json')
    checked_errors, checked_choices, checked_runs = 0, 0, 0
    for held in cases:
        pred = read(OUT/'predictions'/f'{held}.json')
        diag = read(OUT/f'diagnostic-{held}.json')
        model = pred['model']
        assert model['held_recording'] not in model['training_recordings']
        x, y, w = [], [], []
        for c in cases:
            if c == held:
                continue
            labels = {r['pair']: r for r in old[c]['rows']}
            valid = [(r, cue(r)) for r in raw[c]['rows']]
            valid = [(r, z) for r, z in valid if z is not None]
            assert len(valid) == model['training_counts'][raw[c]['recording']]
            for r, z in valid:
                e = labels[r['pair']]['errors']
                x.append(z); y.append(e['native_rgb']['rotation_deg']-e['native_thermal']['rotation_deg'])
                w.append(1/(2*len(valid)))
        x, y, w = np.array(x), np.array(y), np.array(w)
        mean = np.average(x, weights=w, axis=0)
        scale = np.sqrt(np.average((x-mean)**2, weights=w, axis=0)); scale[scale == 0] = 1
        np.testing.assert_allclose(model['mean'], mean, rtol=1e-8, atol=1e-10)
        np.testing.assert_allclose(model['scale'], scale, rtol=1e-8, atol=1e-10)
        z = np.column_stack([np.ones(len(x)), (x-mean)/scale])
        ridge = np.diag([0.]+[1.]*8)
        beta = np.linalg.lstsq(np.vstack([np.sqrt(w)[:, None]*z, ridge]),
                               np.r_[np.sqrt(w)*y, np.zeros(9)], rcond=None)[0]
        np.testing.assert_allclose(model['beta'], beta, rtol=1e-7, atol=1e-9)
        labels = {r['pair']: r for r in old[held]['rows']}
        for r, p, d in zip(raw[held]['rows'], pred['rows'], diag['rows'], strict=True):
            a = cue(r)
            valid = [c for c, e in r['experts'].items() if e['valid']]
            camera = valid[0] if len(valid) == 1 else None if not valid else 'rgb'
            if a is not None:
                gain = np.r_[1., (a-mean)/scale]@beta
                camera = 'thermal' if gain > 0 else 'rgb'
                np.testing.assert_allclose(p['predicted_gain_deg'], gain, rtol=1e-7, atol=1e-9)
            assert camera == p['camera'] == d['camera']
            checked_choices += 1
            truth = np.asarray(labels[r['pair']]['reference_delta'])
            for cam in valid:
                t = np.asarray(r['experts'][cam]['transform'])
                rotation = truth[:3, :3].T@t[:3, :3]
                v = [rotation[2, 1]-rotation[1, 2], rotation[0, 2]-rotation[2, 0], rotation[1, 0]-rotation[0, 1]]
                angle = np.rad2deg(np.arctan2(np.linalg.norm(v)/2, (np.trace(rotation)-1)/2))
                distance = np.linalg.norm(t[:3, 3]-truth[:3, 3])
                np.testing.assert_allclose([angle, distance], [d['errors'][cam]['rotation_deg'], d['errors'][cam]['translation_m']], rtol=1e-8, atol=1e-9)
                checked_errors += 2
        paired = [r for r in diag['rows'] if all(e is not None for e in r['errors'].values())]
        assert len(paired) == diag['paired_intervals']
        for k in ['rotation_deg', 'translation_m']:
            differences = [r['errors']['rgb'][k]-r['errors']['thermal'][k] for r in paired]
            expected_pairs = {r['pair'] for r, d in zip(paired, differences) if d > 1e-12}
            recorded_pairs = set()
            for run in diag['opportunities'][k]['thermal_win_runs']:
                ids = set(range(run['first_pair'], run['last_pair']+1))
                assert not recorded_pairs.intersection(ids)
                members = [r for r in paired if r['pair'] in ids]
                assert len(members) == run['intervals'] == len(ids)
                assert all(a['source_ids'][1] == b['source_ids'][0] for a, b in zip(members, members[1:]))
                np.testing.assert_allclose(run['summed_duration_s'], sum((r['common_interval_ns'][1]-r['common_interval_ns'][0])/1e9 for r in members))
                recorded_pairs.update(ids); checked_runs += 1
            assert expected_pairs == recorded_pairs
            assert len(expected_pairs) == diag['opportunities'][k]['thermal_wins']
            for method in diag['paired_mean_errors']:
                losses = []
                for r in paired:
                    e = r['errors']
                    choice = {'rgb': 'rgb', 'thermal': 'thermal', 'cue': r['camera'], 'train_best_fixed': r['fixed_camera']}.get(method)
                    if method.endswith('_oracle'):
                        key = 'rotation_deg' if method == 'rotation_oracle' else 'translation_m'
                        use_thermal = e['thermal'][key] < e['rgb'][key]
                        if method == 'joint_oracle':
                            use_thermal = all(e['thermal'][j] <= e['rgb'][j] for j in ['rotation_deg', 'translation_m'])
                        choice = 'thermal' if use_thermal else 'rgb'
                    losses.append(e[choice][k])
                np.testing.assert_allclose(diag['paired_mean_errors'][method][k], np.mean(losses), atol=1e-12)
        assert summary['conditions'][held]['paired_mean_errors'] == diag['paired_mean_errors']
    for m in summary['equal_record_paired_mean_errors']:
        for k in ['rotation_deg', 'translation_m']:
            np.testing.assert_allclose(summary['equal_record_paired_mean_errors'][m][k],
                np.mean([summary['conditions'][c]['paired_mean_errors'][m][k] for c in cases]), atol=1e-12)
    return {'status': 'PASS', 'producer_or_scorer_imported': False, 'native_error_components': checked_errors,
            'selection_checks': checked_choices, 'run_checks': checked_runs,
            'training_models_reconstructed_with_augmented_least_squares': 3}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)
