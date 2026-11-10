"""Independent Pillow decoding and scalar native-pixel depth-support audit."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/depth-mean-ablation/recovered-rain-001'


def read(p):
    return json.loads(p.read_text())


def digest(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def scalar_sample(image, uv):
    if not all(math.isfinite(float(v)) for v in uv):
        return 0., False
    x, y = [math.floor(float(v)+.5) for v in uv]
    if not (0 <= y < image.shape[0] and 0 <= x < image.shape[1]):
        return 0., False
    value = int(image[y, x])/256
    return value, value > 0


def audit():
    plan = read(ROOT/'experiments/depth-label-retrieval-freeze-2026-11-09.json')
    for rel, h in plan['pins'].items():
        assert digest(ROOT/rel) == h, rel
    acq = read(OUT/'acquisition.json'); report = read(OUT/'qualification.json')
    manifest = read(OUT/'exp054-manifest.json')
    maps = {}
    for r in acq['maps']:
        path = ROOT/r['path']
        assert digest(path) == r['sha256'] == manifest['member_sha256'][r['archive_member']]
        with Image.open(path) as im:
            image = np.asarray(im).copy()
        assert np.issubdtype(image.dtype, np.integer) and image.min() >= 0 and image.max() <= 65535
        maps[r['camera'], r['source_id']] = image
    source = read(ROOT/'results/pose-covariance/accuracy-001/rain/source-manifest.json')
    image_checks = 0
    for r in source['records'][:20]:
        for camera in ['rgb', 'thr']:
            for side in ['left', 'right']:
                item = r['cameras'][camera][side]
                path = ROOT/item['path']
                old = f'ore-ms/results/ms2-rain-replication/pilot/sync_data/{source["sequence"]}/{camera}/img_{side}/{r["stem"]}.png'
                assert digest(path) == item['sha256'] == manifest['member_sha256'][old]
                image_checks += 1
    checked, sample_checks, maximum = 0, 0, 0
    for camera, sensor in [('rgb', 'rgb'), ('thermal', 'thr')]:
        capture = ROOT/'results/pose-covariance/accuracy-001/rain/captures'/camera
        with np.load(capture/'tensor-map.npz', allow_pickle=False) as saved:
            for row in (r for r in report['cases'] if r['camera'] == camera):
                i = row['pair']; checked += 1
                if i == 19:
                    assert row['status'] == 'target_label_missing' and row['joint_factors'] is None
                    continue
                mask = (saved['edge/match2frame1/mapping'] == i)&(saved['edge/match2frame2/mapping'] == i+1)
                uvs = [saved[f'match//pixel{j}_uv'][mask] for j in [1, 2]]
                depth = [maps[sensor, f'{j:06}'] for j in [i, i+1]]
                zz, vv = [], []
                for image, uv in zip(depth, uvs):
                    samples = [scalar_sample(image, point) for point in uv]
                    zz.append(np.array([v[0] for v in samples])); vv.append(np.array([v[1] for v in samples]))
                    sample_checks += len(samples)
                joint = []
                for n in range(len(uvs[0])):
                    valid = bool(vv[0][n] and vv[1][n])
                    for j in [0, 1]:
                        if not valid:
                            break
                        k = saved['frames//K'][i+j]; bl = float(saved['frames//baseline'][i+j])
                        right = float(uvs[j][n, 0])-float(k[0, 0])*bl/zz[j][n]
                        valid = 0 <= right < depth[j].shape[1]-1
                    joint.append(valid)
                assert row['native_factors'] == len(joint)
                assert row['source_supported'] == int(vv[0].sum())
                assert row['target_supported'] == int(vv[1].sum())
                assert row['raw_joint_factors'] == int((vv[0]&vv[1]).sum())
                assert row['joint_factors'] == sum(joint) < 3
                maximum = max(maximum, sum(joint))
                assert row['status'] == 'insufficient_joint_factors'
                file = ROOT/row['support_file']; assert digest(file) == row['support_sha256']
                with np.load(file, allow_pickle=False) as support:
                    for key, value in [('source_depth_m', zz[0]), ('target_depth_m', zz[1]), ('source_valid', vv[0]), ('target_valid', vv[1]), ('joint_mask', joint)]:
                        np.testing.assert_array_equal(support[key], value)
    assert checked == 40 and report['eligible_cases'] == 0 and report['four_arm_solves'] == 0
    return {'status': 'PASS', 'producer_imported': False, 'frozen_hashes_verified': len(plan['pins']),
            'pillow_maps_decoded': len(maps), 'actual_source_image_hash_checks': image_checks,
            'native_pair_slots': checked, 'scalar_endpoint_samples': sample_checks,
            'maximum_joint_factors': int(maximum), 'eligible_pairs': 0, 'four_arm_solves': 0}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)
