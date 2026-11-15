"""Continuous-time planar radiance proxy with four thermal timing controls.

No learned frontend is imported or executed. The profile is a virtual camera,
not a calibration of MS2 or a reproduction of the TRGS renderer.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates

from qualify_native_frontend import now, read, recipe, sha, write

ARMS = ['sharp', 'memory', 'rolling', 'combined']


def kinematics(t, motion, plan):
    t = np.asarray(t)
    if motion == 'stationary':
        return np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
    if motion == 'twist':
        w = plan['twist']['yaw_rate_rad_s']
        vx, _, vz = plan['twist']['body_velocity_m_s']
        theta = w*t
        return theta, (vx*np.sin(theta)+vz*(1-np.cos(theta)))/w, (vx*(np.cos(theta)-1)+vz*np.sin(theta))/w
    if motion == 'accelerating':
        v, a = np.array(plan['accelerating']['velocity_m_s']), np.array(plan['accelerating']['acceleration_m_s2'])
        return (plan['accelerating']['yaw_rate_rad_s']*t+.5*plan['accelerating']['yaw_acceleration_rad_s2']*t*t,
                v[0]*t+.5*a[0]*t*t, v[2]*t+.5*a[2]*t*t)
    raise ValueError(motion)


def pose(t, motion, plan, side=0):
    theta, x, z = kinematics(float(t), motion, plan)
    c, s = np.cos(theta), np.sin(theta)
    Q = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Q, np.array([x, 0., z])+Q@np.array([side*plan['baseline_m'], 0., 0.])
    return T


def readout_offsets(plan):
    h, w = plan['shape']
    y, x = np.mgrid[:h, :w]
    raw = y*plan['row_period_s']+x*plan['row_period_s']/w
    return raw-.5*(raw[-1, -1]+raw[0, 0])


def irradiance(textures, t, motion, side, plan):
    h, w = plan['shape']
    K = np.array(plan['K'])
    v, u = np.mgrid[:h, :w]
    rx, ry = (u-K[0, 2])/K[0, 0], (v-K[1, 2])/K[1, 1]
    theta, x, z = kinematics(t, motion, plan)
    c, s = np.cos(theta), np.sin(theta)
    ox, oz = x+side*plan['baseline_m']*c, z-side*plan['baseline_m']*s
    dx, dz = c*rx+s, -s*rx+c
    scale = (plan['plane_depth_m']-oz)/dz
    U = K[0, 0]*(ox+scale*dx)/plan['plane_depth_m']+K[0, 2]
    V = K[1, 1]*(scale*ry)/plan['plane_depth_m']+K[1, 2]
    valid = (dz > 0)&(scale > 0)&(U >= 1)&(U <= w-2)&(V >= 1)&(V <= h-2)
    values = np.stack([map_coordinates(texture, [V, U], order=1, mode='constant', cval=0., prefilter=False) for texture in textures])
    values[:, (dz <= 0)|(scale <= 0)] = 0.
    return values, valid


def view(textures, motion, tref, side, arm, n, plan):
    rolling = arm in ['rolling', 'combined']
    memory = arm in ['memory', 'combined'] and plan['tau_s'] > 0
    t = tref+readout_offsets(plan) if rolling else tref
    # Exact static-scene/DC limit, also checked independently by the auditor.
    if not memory or motion == 'stationary':
        return irradiance(textures, t, motion, side, plan)
    nodes, weights = np.polynomial.legendre.leggauss(n)
    ages = .5*(nodes+1)*plan['history_tau']*plan['tau_s']
    weights = .5*plan['history_tau']*weights*np.exp(-ages/plan['tau_s'])/(-np.expm1(-plan['history_tau']))
    weights /= weights.sum()  # Discrete DC preservation; negligible GL roundoff.
    values = np.zeros((len(textures), *plan['shape']))
    valid = np.ones(plan['shape'], bool)
    for age in [0., plan['history_tau']*plan['tau_s']]:
        _, mask = irradiance(textures, t-age, motion, side, plan)
        valid &= mask
    for age, weight in zip(ages, weights):
        sample, mask = irradiance(textures, t-age, motion, side, plan)
        values += weight*sample
        valid &= mask
    return values, valid


def level(textures, n, plan):
    result = {}
    for motion in plan['motions']:
        values = np.zeros((len(textures), 4, 2, 2, *plan['shape']))
        valid = np.zeros((4, 2, 2, *plan['shape']), bool)
        for a, arm in enumerate(ARMS):
            for i, tref in enumerate(plan['reference_times_s']):
                for side in [0, 1]:
                    values[:, a, i, side], valid[a, i, side] = view(textures, motion, tref, side, arm, n, plan)
        result[motion] = (values, valid)
        print('integration grid', n, motion, flush=True)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--protocol', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    plan = read(args.protocol)
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    conditions = list(plan['textures'])
    textures = [recipe()(cv2.imread(plan['textures'][c], cv2.IMREAD_UNCHANGED)).astype(float)/255 for c in conditions]
    previous = level(textures, plan['integration_grids'][0], plan)
    convergence = []
    chosen, n = None, None
    for n in plan['integration_grids'][1:]:
        current = level(textures, n, plan)
        maximum, squares, count = 0., 0., 0
        for motion in plan['motions']:
            a, ma = previous[motion]
            b, mb = current[motion]
            mask = np.broadcast_to(ma&mb, a.shape)
            difference = (a-b)[mask]*255
            maximum = max(maximum, float(np.max(np.abs(difference))))
            squares += float(difference@difference)
            count += len(difference)
        rms = np.sqrt(squares/count)
        passed = bool(maximum <= plan['integration_max_code'] and rms <= plan['integration_rms_code'])
        convergence.append({'higher_grid': n, 'max_code_difference': maximum, 'rms_code_difference': float(rms), 'supported_values': count, 'pass': passed})
        write(out/f'convergence-{n:03}.json', convergence[-1])
        print('convergence', convergence[-1], flush=True)
        if passed:
            chosen = current
            break
        previous = current
    if chosen is None:
        write(out/'stop.json', {'status': 'INTEGRATION_NOT_QUALIFIED', 'history': convergence})
        raise RuntimeError('Predeclared integration grids exhausted; no model inference allowed')
    records = []
    for motion, (values, valid) in chosen.items():
        left = np.array([pose(t, motion, plan) for t in plan['reference_times_s']])
        right = np.array([pose(t, motion, plan, 1) for t in plan['reference_times_s']])
        truth = np.linalg.solve(left[0], left[1])
        for j, condition in enumerate(conditions):
            continuous = values[j]
            assert continuous.min() >= -1e-12 and continuous.max() <= 1+1e-12
            images = np.rint(np.round(np.clip(continuous, 0, 1)*255, 10)).astype(np.uint8)
            key = condition+'-'+motion
            path = out/(key+'.npz')
            np.savez_compressed(path, images=images, continuous=continuous, valid=valid,
                                coarse_continuous=previous[motion][0][j], coarse_valid=previous[motion][1],
                                common_view_support=valid.all(0), common_pixel_support=valid.all((0, 1, 2)),
                                world_from_left=left, world_from_right=right, target_to_source=truth,
                                reference_times_s=plan['reference_times_s'], readout_offsets_s=readout_offsets(plan),
                                texture=textures[j])
            mosaic = np.concatenate([np.concatenate(images[a, :, 0], axis=1) for a in range(4)], axis=0)
            cv2.imwrite(str(out/(key+'.png')), mosaic)
            records.append({'id': key, 'motion': motion, 'condition': condition, 'path': str(path), 'sha256': sha(path),
                            'integration_nodes': n, 'common_pixel_fraction': float(valid.all((0, 1, 2)).mean())})
    write(out/'inventory.json', {'created_utc': now(), 'records': records, 'arms': ARMS,
                                 'integration_nodes': n, 'convergence': convergence, 'protocol_sha256': sha(args.protocol),
                                 'scope': 'Nine artificial texture/motion cases,144views. Virtual profile, no exact integrated-flow labels or learned predictions.'})
    write(out/'artifact-lock.json', {'files': {p.name: sha(p) for p in out.iterdir() if p.is_file()}, 'created_utc': now()})
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path
    print('Rendered9cases/144views at', n, 'nodes; no neural inference', flush=True)


if __name__ == '__main__':
    main()
