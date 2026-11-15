"""Independently accumulate locked paths and compare native-control receipts."""
from pathlib import Path
import json
import hashlib
import datetime
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT/'results/native-thermal-constraints/run-001'
REC = ROOT/'results/longer-fixed-thermal/recovery'


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit():
    scores = read(RUN/'scores.json')
    lock = read(RUN/'prediction-lock.json')
    assert scores['prediction_lock_sha256'] == sha(RUN/'prediction-lock.json')
    freeze = read(RUN/'freeze.json')
    assert lock['freeze_sha256'] == sha(RUN/'freeze.json')
    assert freeze['source_sha256'] == sha(ROOT/'tools/native_thermal_constraints.py')
    checks = 3
    comparisons = []
    for receipt in lock['rows']:
        c = receipt['condition']
        prediction_path = RUN/f'predictions-{c}.json'
        assert sha(prediction_path) == receipt['prediction_sha256']
        for path, expected in freeze['input_pins'][c].items():
            assert sha(ROOT/path) == expected
            checks += 1
        predictions = read(prediction_path)['rows']
        assert [row['pair'] for row in predictions] == list(range(599))
        refs = [np.loadtxt(REC/'references'/c/f'{i:06}.txt').reshape(4,4) for i in range(600)]
        for reference in refs:
            raw_rotation = reference[:3,:3]
            assert np.linalg.norm(raw_rotation.T@raw_rotation-np.eye(3)) < 1e-5
            u, _, vt = np.linalg.svd(raw_rotation)
            assert np.linalg.det(u@vt) > 0
            reference[:3,:3] = u@vt
        native = read(ROOT/f'results/longer-native-control/run-001/predictions-{c}.json')['rows']
        for i, row in enumerate(predictions):
            current = row['methods']['native_rgb_replay']['transform']
            original = native[i]['methods']['native_rgb']['transform']
            assert (current is None) == (original is None)
            if current is not None:
                delta = np.linalg.solve(np.array(original), np.array(current))
                assert np.linalg.norm(delta[:3,3]) < 1e-4
                assert Rotation.from_matrix(delta[:3,:3]).magnitude() < 1e-4
                checks += 2
        for method, result in scores['summary'][c].items():
            for start, end, reported in [(0,599,result['full_path'])]+[
                    (b['start'],b['start']+99,b) for b in result['blocks']]:
                transforms = [row['methods'][method]['transform'] for row in predictions[start:end]]
                missing = [start+i for i,t in enumerate(transforms) if t is None]
                if missing:
                    assert reported['status'] == 'INCOMPLETE_NO_GAP_FILL'
                    assert reported['missing_pairs'] == missing
                    checks += 2
                    continue
                rotation = np.eye(3); translation = np.zeros(3)
                origin = refs[start]
                distances = [0.]; angles = [0.]
                for i, value in enumerate(transforms):
                    t = np.asarray(value)
                    # Scorer's recorded roundoff policy projects proper rotations.
                    u, _, vt = np.linalg.svd(t[:3,:3]); projected = u@vt
                    assert np.linalg.det(projected)>0
                    translation = translation + rotation@t[:3,3]
                    rotation = rotation@projected
                    reference = np.linalg.solve(origin, refs[start+i+1])
                    distances.append(sum(float(x)**2 for x in translation-reference[:3,3]))
                    # Reference files also have finite-decimal serialization.
                    u, _, vt = np.linalg.svd(reference[:3,:3]); rr = u@vt
                    angles.append(float(Rotation.from_matrix(rr.T@rotation).magnitude())**2)
                position = float(np.sqrt(sum(distances)/len(distances)))
                orientation = float(np.rad2deg(np.sqrt(sum(angles)/len(angles))))
                assert abs(position-reported['position_rmse_m']) < 1e-6
                assert abs(orientation-reported['orientation_rmse_deg']) < 1e-5
                checks += 2
        comparisons.append({'condition': c, 'native_reproduction_passed': receipt['reproduction_passed'],
                            'supported': len(receipt['reproduction_errors']),
                            'max_translation_reproduction_m': max(x['translation_m'] for x in receipt['reproduction_errors']),
                            'max_rotation_reproduction_rad': max(x['rotation_rad'] for x in receipt['reproduction_errors'])})
    result = {'at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'checks': checks, 'comparisons': comparisons, 'status': 'PASS',
              'source_sha256': sha(Path(__file__))}
    (RUN/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result)


if __name__ == '__main__':
    audit()
