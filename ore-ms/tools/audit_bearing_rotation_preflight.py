"""Audit saved packet sampling/masks, not the neural covariance computation."""
from pathlib import Path
import json
import hashlib
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/bearing-rotation-preflight/run-001'


def audit():
    plan = json.loads((ROOT/'experiments/bearing-rotation-assay-protocol-2026-11-12.json').read_text())
    for name, digest in plan['pins'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest, name
    rows = json.loads((OUT/'acquisition.json').read_text())['rows']
    total = matrices = 0
    for row in rows:
        i = row['pair']; path = OUT/'packets'/f'{i:03}.npz'
        assert hashlib.sha256(path.read_bytes()).hexdigest() == row['packet_sha256']
        with np.load(path, allow_pickle=False) as z, np.load(
                ROOT/f'results/thermal-matcher-pilot/run-001/flow/{i:03}.npz', allow_pickle=False) as old:
            uv, covariance = z['source_uv'], z['covariance']
            # Decode each compressed array once, not once per sampled pixel.
            flows = {key: old[key] for key in ['original', 'replacement']}
            assert len(uv) == 585
            assert np.isfinite(covariance).all() and np.all(np.linalg.eigvalsh(covariance) > 0)
            mask = np.ones(585, bool)
            for field, key in [('original_uv', 'original'), ('replacement_uv', 'replacement')]:
                expected = np.array([point+flows[key][0, :, point[1], point[0]] for point in uv])
                np.testing.assert_array_equal(z[field], expected)
                mask &= np.isfinite(expected).all(1)&(expected >= 0).all(1)&(expected < [640, 256]).all(1)
            np.testing.assert_array_equal(z['common'], mask)
            assert int(mask.sum()) == row['common_support']
            total += int(mask.sum()); matrices += len(covariance)
    result = {'status': 'PASS_SAVED_INPUT_AUDIT', 'source_and_packet_hashes': len(plan['pins']),
        'covariance_matrices_finite_spd': matrices, 'common_grid_observations': total, 'packets': len(rows),
        'scope': 'Cached endpoint sampling, support masks, saved covariance SPD/schema and hashes; not a neural covariance rerun or calibration test',
        'real_pose_solves': 0, 'real_reference_scores': 0}
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)


if __name__ == '__main__':
    audit()
