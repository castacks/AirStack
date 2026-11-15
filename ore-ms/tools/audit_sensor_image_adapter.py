"""Independent raw-grid rays, adaptive response quadrature and pose targets."""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.integrate import quad
from scipy.linalg import expm

from qualify_native_frontend import now, read, sha, write


def state(t, motion, plan):
    if motion == 'stationary':
        return np.eye(3), np.zeros(3)
    if motion == 'twist':
        w = plan['twist']['yaw_rate_rad_s']
        W = np.array([[0., 0., w], [0., 0., 0.], [-w, 0., 0.]])
        Q = cv2.Rodrigues(np.array([0., w*t, 0.]))[0]
        velocity = np.array(plan['twist']['body_velocity_m_s'])
        C = ((np.eye(3)-Q)@W+np.array([[0., 0., 0.], [0., w*w*t, 0.], [0., 0., 0.]]))@velocity/(w*w)
        return Q, C
    angle = plan['accelerating']['yaw_rate_rad_s']*t+.5*plan['accelerating']['yaw_acceleration_rad_s2']*t*t
    C = np.array(plan['accelerating']['velocity_m_s'])*t+np.array(plan['accelerating']['acceleration_m_s2'])*t*t/2
    return cv2.Rodrigues(np.array([0., angle, 0.]))[0], C


def sample(texture, ray, t, motion, side, plan):
    Q, C = state(t, motion, plan)
    origin = C+Q@np.array([side*plan['baseline_m'], 0., 0.])
    direction = Q@ray
    distance = (plan['plane_depth_m']-origin[2])/direction[2]
    if direction[2] <= 0 or distance <= 0:
        return 0., False
    point = origin+distance*direction
    K = np.array(plan['K'])
    projected = K@point
    u, v = projected[:2]/projected[2]
    h, w = texture.shape
    valid = 1 <= u <= w-2 and 1 <= v <= h-2
    if u < 0 or u > w-1 or v < 0 or v > h-1:
        return 0., valid
    x, y = math.floor(u), math.floor(v)
    x1, y1 = min(x+1, w-1), min(y+1, h-1)
    a, b = u-x, v-y
    value = ((1-b)*((1-a)*texture[y, x]+a*texture[y, x1])+
             b*((1-a)*texture[y1, x]+a*texture[y1, x1]))
    return value, valid


def audit(plan, folder, output):
    checks = 0
    maximum_quad_code = 0.
    def check(x, message):
        nonlocal checks
        assert x, message
        checks += 1
    def close(a, b, atol=1e-10, rtol=1e-9):
        nonlocal checks
        np.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
        checks += 1
    for path, digest in plan['pins'].items():
        check(sha(path) == digest, path)
    for name, digest in read(folder/'artifact-lock.json')['files'].items():
        check(sha(folder/name) == digest, 'artifact hash')
    inventory = read(folder/'inventory.json')
    check(len(inventory['records']) == 9 and len({r['id'] for r in inventory['records']}) == 9, 'nine cases')
    check(inventory['arms'] == ['sharp', 'memory', 'rolling', 'combined'], 'four arms')
    check(inventory['protocol_sha256'] == sha(plan['protocol_path']), 'protocol binding')
    K = np.array(plan['K'])
    inverseK = np.linalg.inv(K)
    h, w = plan['shape']
    span = (h-1)*plan['row_period_s']+(w-1)*plan['row_period_s']/w
    grid = [(u, v) for u in [64, 128, 256, 384, 512, 576] for v in [48, 96, 160, 208]]
    normalization = -math.expm1(-plan['history_tau'])
    close(quad(lambda a: math.exp(-a/plan['tau_s'])/plan['tau_s'], 0, plan['history_tau']*plan['tau_s'])[0], normalization)
    check(math.exp(-plan['history_tau'])*255 < .1, 'bounded omitted-history radiance contribution')
    max_difference, square_sum, total = 0., 0., 0
    source_images = {}
    # Recreate the source's pinned intensity preparation without using renderer
    # functions; that preparation itself was independently qualified previously.
    from qualify_native_frontend import recipe
    for condition, path in plan['textures'].items():
        source_images[condition] = recipe()(cv2.imread(path, cv2.IMREAD_UNCHANGED)).astype(float)/255
    points = 0
    for record in inventory['records']:
        check(sha(record['path']) == record['sha256'], 'case hash')
        with np.load(record['path']) as z:
            images, values, valid = [z[k] for k in ['images', 'continuous', 'valid']]
            texture, left, right, target, offsets, times = [z[k] for k in ['texture', 'world_from_left', 'world_from_right', 'target_to_source', 'readout_offsets_s', 'reference_times_s']]
            check(images.shape == values.shape == valid.shape == (4, 2, 2, h, w), 'view schema')
            check(images.dtype == np.uint8 and valid.dtype == bool, 'dtypes')
            check(np.isfinite(values).all() and values.min() >= -1e-12 and values.max() <= 1+1e-12, 'bounded intensity')
            check(np.array_equal(texture, source_images[record['condition']]), 'texture identity')
            check(float(np.max(np.abs(images.astype(float)-values*255))) <= .5000000001, 'single quantization')
            check(np.array_equal(z['common_view_support'], valid.all(0)), 'per-view shared support')
            check(np.array_equal(z['common_pixel_support'], valid.all((0, 1, 2))), 'common coordinate support')
            close(z['common_pixel_support'].mean(), record['common_pixel_fraction'])
            check(record['common_pixel_fraction'] > .5, 'adequate declared support')
            coarse, coarse_valid = z['coarse_continuous'], z['coarse_valid']
            error = (values-coarse)[valid&coarse_valid]*255
            max_difference = max(max_difference, float(np.max(np.abs(error))))
            square_sum += float(error@error)
            total += len(error)
        for i, t in enumerate(times):
            close(t, plan['reference_times_s'][i])
            Q, C = state(t, record['motion'], plan)
            close(left[i, :3, :3], Q)
            close(left[i, :3, 3], C)
            close(left[i, 3], [0, 0, 0, 1])
            close(right[i, :3, :3], Q)
            close(right[i, :3, 3], C+Q@np.array([plan['baseline_m'], 0., 0.]))
        close(target, np.linalg.solve(left[0], left[1]))
        close(offsets[0, 0], -.5*span)
        close(offsets[-1, -1], .5*span)
        close(np.diff(offsets, axis=0), np.full((h-1, w), plan['row_period_s']))
        close(np.diff(offsets, axis=1), np.full((h, w-1), plan['row_period_s']/w))
        if record['motion'] == 'twist':
            Xi = np.zeros((4, 4))
            rate = plan['twist']['yaw_rate_rad_s']
            Xi[:3, :3] = [[0, 0, rate], [0, 0, 0], [-rate, 0, 0]]
            Xi[:3, 3] = plan['twist']['body_velocity_m_s']
            close(left, np.stack([expm(t*Xi) for t in times]))
            close(target, expm((times[1]-times[0])*Xi))
        if record['motion'] == 'stationary':
            close(target, np.eye(4))
            for a in range(1, 4):
                check(np.array_equal(images[a], images[0]), 'stationary image null')
                close(values[a], values[0], atol=1e-12)
        for a, arm in enumerate(inventory['arms']):
            for i, tref in enumerate(times):
                for side in [0, 1]:
                    for u, v in grid:
                        r = float(tref)
                        if arm in ['rolling', 'combined']:
                            r += v*plan['row_period_s']+u*plan['row_period_s']/w-.5*span
                        ray = inverseK@np.array([u, v, 1.])
                        if arm in ['memory', 'combined']:
                            if record['motion'] == 'stationary':
                                p, support = sample(texture, ray, r, record['motion'], side, plan)
                                expected = p*quad(lambda age: math.exp(-age/plan['tau_s'])/plan['tau_s']/normalization,
                                                  0, plan['history_tau']*plan['tau_s'], epsabs=1e-10)[0]
                            else:
                                expected = quad(lambda age: math.exp(-age/plan['tau_s'])/plan['tau_s']/normalization*
                                                sample(texture, ray, r-age, record['motion'], side, plan)[0],
                                                0, plan['history_tau']*plan['tau_s'], epsabs=1e-9, epsrel=1e-9, limit=200)[0]
                        else:
                            expected, support = sample(texture, ray, r, record['motion'], side, plan)
                        discrepancy = abs(expected-values[a, i, side, v, u])*255
                        maximum_quad_code = max(maximum_quad_code, discrepancy)
                        check(discrepancy <= .5+255e-8, 'independent adaptive-time integral')
                        # Independent interior support at history endpoints and
                        # intermediate times; no flow correspondences inferred.
                        ages = np.linspace(0, plan['history_tau']*plan['tau_s'], 9) if arm in ['memory', 'combined'] else [0.]
                        support = all(sample(texture, ray, r-age, record['motion'], side, plan)[1] for age in ages)
                        check(bool(valid[a, i, side, v, u]) == support, 'sampled geometric support')
                        points += 1
    convergence = inventory['convergence'][-1]
    close(max_difference, convergence['max_code_difference'], rtol=1e-7, atol=1e-8)
    close(math.sqrt(square_sum/total), convergence['rms_code_difference'], rtol=1e-7, atol=1e-10)
    check(max_difference <= plan['integration_max_code'] and math.sqrt(square_sum/total) <= plan['integration_rms_code'], 'selected grid convergence')
    check(total == convergence['supported_values'], 'convergence population')
    # Exact zero-time-support limits of the software adapter, with physics
    # identities specified before rendering. These do not run a learned model.
    from sensor_image_adapter import view
    texture = [source_images['day']]
    modified = dict(plan, tau_s=0.)
    sharp, _ = view(texture, 'accelerating', .2, 0, 'sharp', 8, modified)
    memory, _ = view(texture, 'accelerating', .2, 0, 'memory', 8, modified)
    close(sharp, memory, atol=1e-12)
    modified = dict(plan, row_period_s=0.)
    rolling, _ = view(texture, 'accelerating', .2, 0, 'rolling', 8, modified)
    close(sharp, rolling, atol=1e-12)
    memory, _ = view(texture, 'accelerating', .2, 0, 'memory', 8, modified)
    combined, _ = view(texture, 'accelerating', .2, 0, 'combined', 8, modified)
    close(memory, combined, atol=1e-12)
    write(output, {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'cases': 9, 'views': 144,
                   'independently_checked_pixels': points, 'max_adaptive_quadrature_code_error': maximum_quad_code,
                   'integration_nodes': inventory['integration_nodes'], 'grid_max_code_difference': max_difference,
                   'grid_rms_code_difference': math.sqrt(square_sum/total),
                   'scope': 'Image/ray/timing/integration/pose-label qualification only; no learned predictions or physical calibration.'})
    print('PASS', checks, 'checks;', points, 'independent pixel integrals; max code error', maximum_quad_code, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--protocol', required=True)
    p.add_argument('--folder', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    audit(read(args.protocol), Path(args.folder), Path(args.output))
