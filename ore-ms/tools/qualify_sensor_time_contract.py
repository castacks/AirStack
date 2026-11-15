"""Source/math readiness for thermal memory; no image/model experiment."""
import argparse
import ast
import math
from pathlib import Path

import numpy as np
from scipy.integrate import quad
from scipy.linalg import expm

from qualify_native_frontend import read, write, sha, now


def qualify(plan, output):
    for p, h in plan['pins'].items():
        assert sha(p) == h, p
    checks, rows = 0, []
    def close(label, observed, expected):
        nonlocal checks
        np.testing.assert_allclose(observed, expected, rtol=1e-9, atol=1e-10)
        checks += 1
        rows.append({'check': label, 'observed': np.asarray(observed).tolist(), 'expected': np.asarray(expected).tolist()})
    tree = ast.parse(Path(plan['dataset_source']).read_text())
    constants = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id in ['thermal_time_constant', 'readout_delay', 'readout_row_duration']:
            constants[node.target.id] = ast.literal_eval(node.value)
    close('published-source thermal time constant, not MS2 calibration', constants['thermal_time_constant'], .008)
    close('published-source trigger/readout delay', constants['readout_delay'], .0005)
    close('published-source full-resolution row period', constants['readout_row_duration'], 27.8e-6)
    tau = constants['thermal_time_constant']
    # Quadrature uses physical time u; closed moments are derived analytically.
    for k in [0, 1, 2, 3]:
        observed = quad(lambda u: u**k*np.exp(-u/tau)/tau, 0, np.inf, epsabs=1e-13, epsrel=1e-12)[0]
        close('infinite-history response moment '+str(k), observed, math.factorial(k)*tau**k)
    for scaled in [0., .5, 1., 2., 5.]:
        t = scaled*tau
        observed = quad(lambda u: np.exp(-u/tau)/tau, 0, t, epsabs=1e-13)[0]
        close('step response at '+str(scaled)+'tau', observed, -np.expm1(-t/tau))
    L = 8*tau
    mass = -np.expm1(-L/tau)
    observed = quad(lambda u: np.exp(-u/tau)/(tau*mass), 0, L, epsabs=1e-13)[0]
    close('normalized finite-history DC response', observed, 1.)
    mean = quad(lambda u: u*np.exp(-u/tau)/(tau*mass), 0, L, epsabs=1e-13)[0]
    close('finite-history response age', mean, tau-L/np.expm1(L/tau))
    tail = quad(lambda u: np.exp(-u/tau)/tau, L, np.inf, epsabs=1e-13)[0]
    close('omitted eight-tau response mass', tail, np.exp(-8))
    r, v, a = .013, .7, .4
    ramp = lambda s: 1+v*s
    quadratic = lambda s: 1+v*s+.5*a*s*s
    integrated_ramp = quad(lambda u: np.exp(-u/tau)/tau*ramp(r-u), 0, np.inf, epsabs=1e-13)[0]
    integrated_quadratic = quad(lambda u: np.exp(-u/tau)/tau*quadratic(r-u), 0, np.inf, epsabs=1e-13)[0]
    close('affine radiance effective time', integrated_ramp, ramp(r-tau))
    close('quadratic radiance response', integrated_quadratic, quadratic(r)-tau*(v+a*r)+a*tau*tau)
    close('a single delayed sample misses quadratic response', integrated_quadratic-quadratic(r-tau), .5*a*tau*tau)
    # Constant body twist: a shared timestamp shift cancels from relative pose.
    Xi = np.array([[0., .1, .2, .3], [-.1, 0., -.1, .1], [-.2, .1, 0., -.2], [0., 0., 0., 0.]])
    t0, t1 = .1, .2
    instantaneous = np.linalg.solve(expm(t0*Xi), expm(t1*Xi))
    shifted = np.linalg.solve(expm((t0-tau)*Xi), expm((t1-tau)*Xi))
    close('constant-body-twist shared shift invariance', shifted, instantaneous)
    close('constant-body-twist relative pose', instantaneous, expm((t1-t0)*Xi))
    # Contrast with acceleration: even an ideal effective-time interpretation
    # changes relative displacement, independently of any neural estimator.
    position = lambda t: .5*t*t
    original_dx = position(t1)-position(t0)
    delayed_dx = position(t1-tau)-position(t0-tau)
    close('accelerated translation contrast', delayed_dx-original_dx, -tau*(t1-t0))
    # Test readout ordering on a declared grid, not an MS2 camera calibration.
    h, w = 12, 16
    row_period = constants['readout_row_duration']
    y, x = np.mgrid[:h, :w]
    readout = y*row_period+x*row_period/w
    close('row stride', np.diff(readout, axis=0), np.full((h-1, w), row_period))
    close('pixel stride', np.diff(readout, axis=1), np.full((h, w-1), row_period/w))
    close('declared-grid total readout span', readout[-1, -1]-readout[0, 0], (h-1)*row_period+(w-1)*row_period/w)
    # Downsampling preserves physical span only if the row time is adjusted.
    close('full-row period across two-row downsample step', (2*y)*row_period, y*(2*row_period))
    write(output, {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'rows': rows,
                   'source_defaults': constants, 'protocol_sha256': sha(plan['protocol_path']),
                   'source_parameters_are_not_ms2_calibration': True,
                   'omitted_eight_tau_mass': tail,
                   'accelerated_relative_displacement_change_m': delayed_dx-original_dx,
                   'scope': 'Analytic identities independently compared with quadrature/matrix exponentials. Qualifies timing/target math, not an image renderer, physical profile, correspondence labels or learned-model performance.'})
    for p, h in plan['pins'].items():
        assert sha(p) == h, p
    print('PASS', checks, 'source/math checks; no images or model inference', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--protocol', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    qualify(read(args.protocol), Path(args.output))
