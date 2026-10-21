"""Audit a supplied MSA calibration on fixed EXP-020 matches, without fitting."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .audit_td3_geometry import pose_matrix, transform_difference
from .td3_calibration_reconcile import (camera, rectified_pixels, sampson_pixels,
                                        triangulation, score)
from .td3_tracking_uncertainty import sha, percentiles


def msa_camera(path):
    data = yaml.safe_load(path.read_text())
    if data['distortion_model'] != 'equidistant':
        raise ValueError('Expected supplied equidistant calibration')
    k = np.array(data['camera_matrix']['data'], float).reshape(3, 3)
    d = np.array(data['distortion_coefficients']['data'], float)
    if d.shape != (5,) or d[4] != 0:
        raise ValueError('Only four fisheye coefficients plus zero padding supported')
    return data, k, d[:4]


def reinterpret_rays(rays, old_k, old_d, new_k, new_d):
    """Undo old lens interpretation at fixed raw sensor pixel coordinates."""
    pixels = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), old_k, old_d)[0]
    old_back = cv2.undistortPointsIter(
        pixels, old_k, old_d, None, None,
        (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-13)).reshape(-1, 2)
    xy = cv2.fisheye.undistortPoints(pixels, new_k, new_d).reshape(-1, 2)
    back = cv2.fisheye.distortPoints(xy.reshape(-1, 1, 2), new_k, new_d)
    return np.column_stack([xy, np.ones(len(xy))]), pixels.reshape(-1, 2), dict(
        old_ray_roundtrip_max=float(np.max(np.abs(old_back-rays[:, :2]))),
        new_pixel_roundtrip_max=float(np.max(np.abs(back-pixels))))


def run(root, output):
    output.mkdir(parents=True, exist_ok=False)
    source = root/'data/calibration/msa_cal'
    config_path = root/'data/calibration/upstream/get_thermal_data.yaml'
    config = yaml.safe_load(config_path.read_text())
    extrinsics = yaml.safe_load((source/'extrinsics.yaml').read_text())
    poses = {s: pose_matrix(extrinsics['thermal_'+s]['value']) for s in ['left', 'right']}
    transform = np.linalg.inv(poses['right'])@poses['left']
    urdf = ET.parse(source/'extrinsics.urdf').getroot()
    for element in urdf.iter():
        element.tag = element.tag.split('}')[-1]
    urdf_controls = {}
    for side in ['left', 'right']:
        joint = urdf.find(f"joint[@name='thermal_{side}_joint']")
        origin = joint.find('origin')
        t = np.eye(4)
        t[:3, 3] = np.fromstring(origin.attrib['xyz'], sep=' ')
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.attrib['rpy'], sep=' ')).as_matrix()
        urdf_controls[side] = transform_difference(t, poses[side])
        assert joint.find('parent').attrib['link'] == extrinsics['thermal_'+side]['parent']
    cameras = [msa_camera(source/f'thermal_{s}.intrinsics.yaml') for s in ['left', 'right']]
    k0, d0 = cameras[0][1:]; k1, d1 = cameras[1][1:]
    rect = cv2.fisheye.stereoRectify(k0, d0, k1, d1, (640, 512),
        transform[:3, :3].copy(), transform[:3, 3].copy(),
        flags=cv2.CALIB_ZERO_DISPARITY, balance=0, fov_scale=1)
    files = [Path(__file__), config_path, root/'msa_cal.zip', *source.iterdir()]
    cases = []
    old_stored = np.array(config['calib']['cam1']['T_cn_cnm1'])
    (output/'arrays').mkdir()
    cv2.setNumThreads(1)
    for run_name in ['run-001', 'rotation-run-001']:
        original = root/'results/calibration-reconcile'/run_name
        for path in sorted((original/'observations').glob('*.npz')):
            files.append(path)
            with np.load(path) as arrays:
                old = [arrays['rays0'], arrays['rays1']]
            rays, pixels, controls = [], [], []
            for i in range(2):
                a, b = camera(config, f'cam{i}')
                r, p, c = reinterpret_rays(old[i], a, b, *cameras[i][1:])
                rays.append(r); pixels.append(p); controls.append(c)
            tri = triangulation(*rays, transform, k0, k1)
            residual = sampson_pixels(*rays, transform, k0, k1)
            delta = rectified_pixels(rays[0], rect[0], rect[2])-rectified_pixels(rays[1], rect[1], rect[3])
            eligible = tri['eligible']
            metrics = dict(count=len(rays[0]), eligible_count=int(eligible.sum()),
                positive_eligible_fraction=float(tri['positive'][eligible].mean()) if eligible.any() else None,
                sampson_px=percentiles(residual), vertical_px=percentiles(np.abs(delta[:, 1])),
                raw_pixel_roundtrips=controls,
                raw_pixels_in_sensor=[int(((p[:, 0]>=0)&(p[:, 0]<640)&(p[:, 1]>=0)&(p[:, 1]<512)).sum()) for p in pixels])
            baselines = {name: score(*old, t, config)[0] for name, t in
                         [('producer_inverse', np.linalg.inv(old_stored)), ('kalibr_as_stored', old_stored)]}
            cases.append(dict(case=path.stem, source_run=run_name, msa=metrics, baselines=baselines))
            np.savez_compressed(output/'arrays'/f'{path.stem}.npz', rays0=rays[0], rays1=rays[1],
                raw_pixels0=pixels[0], raw_pixels1=pixels[1], sampson_px=residual, rectified_delta=delta, **tri)
    # Synthetic physical points exercise this exact direction and scoring code.
    xyz = np.column_stack([np.linspace(-1, 1, 50), np.linspace(-.3, .3, 50)**2, np.linspace(2, 20, 50)])
    target = xyz@transform[:3, :3].T+transform[:3, 3]
    a = xyz/xyz[:, 2:3]; b = target/target[:, 2:3]
    synthetic = triangulation(a, b, transform, k0, k1)
    assert synthetic['positive'].all()
    assert np.max(np.abs(synthetic['xyz']-xyz)) < 1e-9
    assert len(cases) == 24 and len(set(c['case'] for c in cases)) == 24
    for row in cases:
        for c in row['msa']['raw_pixel_roundtrips']:
            assert c['old_ray_roundtrip_max'] < 1e-8 and c['new_pixel_roundtrip_max'] < 1e-8
    analysis = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), base_commit='3bfce192f',
        status='DIAGNOSTIC_ONLY_NO_CALIBRATION_ADOPTED', opencv=cv2.__version__,
        provenance='CMAIR.Viking.Vicky.2025-01-26.15-13-39.01; applicability to October recordings unconfirmed',
        transform_direction='T_right_raw_optical_from_left_raw_optical', T_right_from_left=transform.tolist(),
        baseline_m=float(np.linalg.norm(transform[:3, 3])), source_relative_rotation_deg=float(np.degrees(Rotation.from_matrix(transform[:3, :3]).magnitude())),
        source_urdf_controls=urdf_controls, synthetic_max_xyz_error=float(np.max(np.abs(synthetic['xyz']-xyz))),
        cases=cases, files_sha256={str(p.resolve()):sha(p) for p in files})
    (output/'analysis.json').write_text(json.dumps(analysis, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in analysis.items() if k not in ['cases', 'files_sha256']}, indent=2))
    for c in cases:
        m=c['msa']; print(c['case'], m['count'], m['positive_eligible_fraction'], m['sampson_px']['p10_p50_p90'][1], m['vertical_px']['p10_p50_p90'][1])


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/workspace'))
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args(); run(args.root, args.output)
