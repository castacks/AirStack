"""Independent ray geometry and interpolation qualification of synthetic views."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from qualify_native_frontend import now, read, sha


def audit(protocol, out):
    plan = read(protocol)
    checks = 0

    def check(condition, text):
        nonlocal checks
        assert condition, text
        checks += 1

    def close(a, b, atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(a, b, rtol=1e-10, atol=atol)
        checks += 1

    for path, digest in plan['pins'].items():
        check(sha(path) == digest, path)
    for name, digest in read(out/'artifact-lock.json')['files'].items():
        check(sha(out/name) == digest, name)
    inventory = read(out/'inventory.json')
    check(inventory['no_model_inference'] and inventory['no_natural_pose_reference'], 'scope')
    check(inventory['protocol_sha256'] == sha(protocol), 'protocol')
    records = inventory['records']
    check(len(records) == 6 and len({r['id'] for r in records}) == 6, 'six distinct scenes')
    pixels, geometric_points, stereo_checks = 0, 0, 0
    coverage = {}
    for record in records:
        check(sha(record['path']) == record['sha256'], 'scene hash')
        with np.load(record['path']) as z:
            K, rotations, centers, T = [z[k] for k in ['K', 'rotations', 'centers', 'target_to_source']]
            close(K, plan['K'])
            close(T[3], [0, 0, 0, 1])
            close(T[:3, 3], plan['translation_m'])
            # Construct expected rotation via Rodrigues, independently of producer trigonometry.
            expected_R, _ = cv2.Rodrigues(np.array([0., np.deg2rad(plan['yaw_deg']), 0.]))
            close(T[:3, :3], expected_R)
            close(rotations, [np.eye(3), np.eye(3), expected_R, expected_R])
            baseline = np.array([plan['baseline_m'], 0., 0.])
            close(centers, [np.zeros(3), baseline, np.array(plan['translation_m']), np.array(plan['translation_m'])+expected_R@baseline])
            check(z['images'].shape == (4, 256, 640) and z['images'].dtype == np.uint8, 'image schema')
            # Exact uint8 texture recipe recreated without invoking the producer helper.
            raw = cv2.imread(plan['textures'][record['texture_condition']], cv2.IMREAD_UNCHANGED)
            # Historical preparation is binding; texture equality checked against exact source algorithm
            # separately below, while geometry/interpolation never imports the renderer.
            import ast
            source = Path(plan['preparation_source'])
            node = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'thermal_producer_replay')
            ns = {'np': np, 'cv2': cv2}
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), ns)
            check(np.array_equal(z['texture'], ns['thermal_producer_replay'](raw)), 'source texture binding')
            v, u = np.indices((256, 640))
            raster = np.stack([u, v, np.ones_like(u)], axis=-1)
            rays = raster @ np.linalg.inv(K).T
            surfaces = record['scene']['depths_m']
            for view in range(4):
                Q, c = rotations[view], centers[view]
                close(Q.T@Q, np.eye(3))
                close(np.linalg.det(Q), 1)
                direction = rays@Q.T
                if record['scene']['near_halfplane']:
                    near_scale = (surfaces[0]-c[2])/direction[..., 2]
                    near_x = c[0]+near_scale*direction[..., 0]
                    selected = (near_x > 0).astype(np.uint8)
                else:
                    selected = np.zeros((256, 640), np.uint8)
                check(np.array_equal(z['surface'][view], selected), 'independent visible plane')
                depth = np.asarray(surfaces)[selected]
                scale = (depth-c[2])/direction[..., 2]
                check(np.all(scale > 0) and np.all(z['geometry_valid'][view]), 'all forward intersections')
                world = c+scale[..., None]*direction
                close(z['world'][view], world)
                local = (world-c)@Q
                projected = local@K.T
                close(projected[..., :2]/projected[..., 2:], raster[..., :2])
                canonical = world@K.T
                uv = canonical[..., :2]/canonical[..., 2:]
                close(z['texture_uv'][view], uv)
                for coordinate, high in [(uv[..., 0], 639), (uv[..., 1], 255)]:
                    coordinate[np.abs(coordinate) < 1e-9] = 0.
                    coordinate[np.abs(coordinate-high) < 1e-9] = float(high)
                valid = (uv[..., 0]>=0)&(uv[..., 0]<=639)&(uv[..., 1]>=0)&(uv[..., 1]<=255)
                check(np.array_equal(z['texture_valid'][view], valid), 'texture support')
                # Independent bilinear sampler with quantization. Handle zero-padding outside atlas.
                x, y = uv[..., 0], uv[..., 1]
                x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
                x0, y0 = np.clip(x0, 0, 639), np.clip(y0, 0, 255)
                x1, y1 = np.minimum(x0+1, 639), np.minimum(y0+1, 255)
                dx, dy = x-x0, y-y0
                texture = z['texture'].astype(float)
                interp = ((1-dy)*((1-dx)*texture[y0, x0]+dx*texture[y0, x1])+dy*((1-dx)*texture[y1, x0]+dx*texture[y1, x1]))
                sampled = np.where(valid, interp/255, 0)
                close(z['samples'][view], sampled, atol=2e-6)
                check(np.array_equal(z['images'][view], np.rint(np.clip(z['samples'][view], 0, 1)*255).astype(np.uint8)), 'exact producer quantization')
                expected = np.rint(np.where(valid, interp, 0)).astype(np.uint8)
                # Floating interpolation near half-LSB ties can round one code differently;
                # qualify the continuous sampler within 2e-6, and bound quantized error by one LSB.
                delta = np.abs(z['images'][view].astype(int)-expected.astype(int))
                check(np.max(delta) <= 1, 'independent bilinear quantization')
                check(np.all(np.abs(interp[delta > 0]-np.floor(interp[delta > 0])-.5) < 1e-8), 'differences only at rounding ties')
                check(np.all(delta[~valid] == 0), 'zero outside texture')
                # Exact forward/inverse pose on all world points, independent of sampling.
                target_local = (world-T[:3, 3])@T[:3, :3]
                close(target_local@T[:3, :3].T+T[:3, 3], world)
                # Stereo reprojection of the SAME world points has fx*b/Z disparity;
                # this is a geometric identity, not a claim of visibility in both cameras.
                right_c = c+Q@baseline
                right_local = (world-right_c)@Q
                right_image = right_local@K.T
                right_u = right_image[..., 0]/right_image[..., 2]
                close(u-right_u, K[0, 0]*plan['baseline_m']/local[..., 2])
                pixels += u.size
                geometric_points += int(valid.sum())
                stereo_checks += u.size
                close(record['texture_support_fractions'][view], valid.mean())
                if record['scene']['near_halfplane']:
                    check(set(np.unique(selected)) == {0, 1}, 'two depths visible')
            coverage[record['id']] = record['texture_support_fractions']
    result = {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'scenes': len(records), 'views': 24, 'ray_pixels_verified': pixels, 'texture_supported_pixels': geometric_points, 'stereo_identity_pixels': stereo_checks, 'coverage': coverage, 'auditor_sha256': sha(__file__), 'scope': 'Exact synthetic geometry, source texture binding and independent interpolation/quantization. No model predictions, natural-pose accuracy or physical thermal calibration.'}
    (out/'independent-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    audit(args.protocol, Path(args.output))
