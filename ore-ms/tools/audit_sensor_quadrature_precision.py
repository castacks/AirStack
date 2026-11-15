"""Retain reference-integrator error diagnostics without changing image bounds."""
import argparse
import math
from pathlib import Path

import numpy as np
from scipy.integrate import quad

from audit_sensor_image_adapter import sample
from qualify_native_frontend import now, read, sha, write


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--protocol', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    supplement = read(args.protocol)
    for path, digest in supplement['pins'].items():
        assert sha(path) == digest, path
    plan = read(supplement['render_protocol'])
    folder = Path(supplement['render_folder'])
    inventory = read(folder/'inventory.json')
    h, w = plan['shape']
    inverseK = np.linalg.inv(plan['K'])
    span = (h-1)*plan['row_period_s']+(w-1)*plan['row_period_s']/w
    normalization = -math.expm1(-plan['history_tau'])
    grid = [(u, v) for u in [64, 128, 256, 384, 512, 576] for v in [48, 96, 160, 208]]
    rows = []
    for record in inventory['records']:
        assert sha(record['path']) == record['sha256']
        with np.load(record['path']) as z:
            texture, values = z['texture'], z['continuous']
        for a in [1, 3]:
            for i, tref in enumerate(plan['reference_times_s']):
                for side in [0, 1]:
                    for u, v in grid:
                        r = tref+(v*plan['row_period_s']+u*plan['row_period_s']/w-.5*span if a == 3 else 0.)
                        ray = inverseK@np.array([u, v, 1.])
                        integrand = lambda age: math.exp(-age/plan['tau_s'])/plan['tau_s']/normalization*sample(texture, ray, r-age, record['motion'], side, plan)[0]
                        result = quad(integrand, 0, plan['history_tau']*plan['tau_s'], epsabs=1e-9, epsrel=1e-9, limit=200, full_output=1)
                        value, error, info = result[:3]
                        discrepancy = abs(value-values[a, i, side, v, u])*255
                        error_code = error*255
                        assert error_code <= supplement['reference_error_code_max']
                        assert discrepancy+error_code <= .5+255e-8
                        rows.append({'case': record['id'], 'arm': inventory['arms'][a], 'frame': i, 'side': side,
                                     'pixel': [u, v], 'difference_code': discrepancy, 'reference_estimated_error_code': error_code,
                                     'error_plus_estimate_code': discrepancy+error_code,
                                     'reference_evaluations': info['neval'], 'diagnostic': result[3] if len(result) > 3 else None})
    write(args.output, {'created_utc': now(), 'status': 'PASS', 'checks': 2*len(rows), 'integrals': len(rows),
                        'max_reference_estimated_error_code': max(r['reference_estimated_error_code'] for r in rows),
                        'max_error_plus_estimate_code': max(r['error_plus_estimate_code'] for r in rows),
                        'warning_integrals': sum(r['diagnostic'] is not None for r in rows), 'rows': rows,
                        'scope': 'Reference precision diagnostics supplement. Same images and0.5code bound; no renderer/auditor tolerance relaxation or learned predictions.'})
    print('PASS', len(rows), 'memory integrals; max reference error estimate', max(r['reference_estimated_error_code'] for r in rows),
          'code; max discrepancy plus estimate', max(r['error_plus_estimate_code'] for r in rows), flush=True)


if __name__ == '__main__':
    main()
