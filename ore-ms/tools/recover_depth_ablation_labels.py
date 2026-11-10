"""Exact historical rain depth recovery and native-factor support qualification."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import tarfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/depth-mean-ablation/recovered-rain-001'
RECOVERY = Path('/root/ore-ms-recovery/depth-label-retrieval/exp054')
SEQUENCE = '_2021-08-06-16-19-00'


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def read(p):
    return json.loads(p.read_text())


def write(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)


def restore():
    from native_depth_proxy import png_schema
    manifest = read(RECOVERY/'manifest.json')
    archive = RECOVERY/'exp-054.tar.gz'
    assert sha(archive) == manifest['file_sha256'][archive.name]
    source = read(ROOT/'results/pose-covariance/accuracy-001/rain/source-manifest.json')
    assert source['sequence'] == SEQUENCE
    prefix = 'ore-ms/results/ms2-rain-replication/pilot/'
    # Bind images by original sequence/camera/side/stem and archived SHA, without
    # extracting the image payloads or opening old model/accuracy outputs.
    image_checks = []
    for r in source['records'][:20]:
        for cam in ['rgb', 'thr']:
            for side in ['left', 'right']:
                name = prefix+f'sync_data/{SEQUENCE}/{cam}/img_{side}/{r["stem"]}.png'
                assert manifest['member_sha256'][name] == r['cameras'][cam][side]['sha256']
                image_checks.append({'name': name, 'sha256': manifest['member_sha256'][name]})
    selected = {prefix+f'proj_depth/{SEQUENCE}/{cam}/depth/{i:06}.png':
                (cam, i) for cam in ['rgb', 'thr'] for i in range(20)}
    records = []
    with tarfile.open(archive) as stream:
        for member in stream:
            if member.name not in selected:
                continue
            assert member.isfile() and 0 < member.size <= 32*1024*1024
            cam, i = selected[member.name]
            data = stream.extractfile(member).read()
            digest = hashlib.sha256(data).hexdigest()
            assert digest == manifest['member_sha256'][member.name]
            png_schema(data, [384, 1224] if cam == 'rgb' else [256, 640])
            target = OUT/'labels'/SEQUENCE/cam/'depth'/f'{i:06}.png'
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as f:
                f.write(data)
            records.append({'camera': cam, 'source_id': f'{i:06}', 'path': str(target.relative_to(ROOT)),
                            'sha256': digest, 'archive_member': member.name})
    assert len(records) == len(selected) == 40
    write(OUT/'acquisition.json', {'recording': SEQUENCE, 'original_experiment': 'EXP054',
          'archive_sha256': sha(archive), 'backup_manifest_sha256': sha(RECOVERY/'manifest.json'),
          'maps': records, 'native_input_image_checks': image_checks,
          'product': 'own-camera single-scan projected depth, uint16 metres*256, zero invalid',
          'source_frame_20_available': False, 'new_publisher_archive_downloads': 0,
          'scope': 'first20 sourceframes only, RGB/thermal, no old numerical/model outputs restored'})
    print({'exact_depth_maps_restored': len(records), 'image_hashes_matched': len(image_checks)})


def sample(depth, uv):
    """Nearest original projected pixel; no interpolation/hole filling."""
    uv = np.asarray(uv, dtype=float)
    finite = np.isfinite(uv).all(1)
    xy = np.zeros_like(uv, dtype=np.int64)
    xy[finite] = np.floor(uv[finite]+.5).astype(np.int64)
    inbound = finite & (xy[:, 0] >= 0) & (xy[:, 0] < depth.shape[1]) & (xy[:, 1] >= 0) & (xy[:, 1] < depth.shape[0])
    value = np.zeros(len(uv), dtype=float)
    value[inbound] = depth[xy[inbound, 1], xy[inbound, 0]].astype(float)/256
    return value, inbound & (value > 0)


def qualify():
    import cv2
    from native_information_weights import native_readout
    from pose_covariance_replay import pose_xyzw
    acquisition = read(OUT/'acquisition.json')
    maps = {}
    for m in acquisition['maps']:
        path = ROOT/m['path']; assert sha(path) == m['sha256']
        depth = cv2.imread(str(path), -1)
        assert depth.dtype == np.uint16
        assert list(depth.shape) == ([384, 1224] if m['camera'] == 'rgb' else [256, 640])
        maps[m['camera'], m['source_id']] = depth
    cases = []
    for camera, sensor in [('rgb', 'rgb'), ('thermal', 'thr')]:
        capture = ROOT/'results/pose-covariance/accuracy-001/rain/captures'/camera
        events = [json.loads(s) for s in (capture/'arbiter-events/events.jsonl').read_text().splitlines()]
        events = {(e['from_idx'], e['frame_idx']): e for e in events if e['kind'] == 'motion_result_written' and e['first_write_for_submission']}
        with np.load(capture/'tensor-map.npz', allow_pickle=False) as saved:
            for i in range(20):
                row = {'camera': camera, 'pair': i, 'source_ids': [f'{i:06}', f'{i+1:06}']}
                if (sensor, f'{i+1:06}') not in maps:
                    cases.append({**row, 'status': 'target_label_missing', 'joint_factors': None}); continue
                mask = (saved['edge/match2frame1/mapping'] == i)&(saved['edge/match2frame2/mapping'] == i+1)
                uv1, uv2 = [saved[f'match//pixel{j}_uv'][mask].astype(float) for j in [1, 2]]
                z1, valid1 = sample(maps[sensor, f'{i:06}'], uv1)
                z2, valid2 = sample(maps[sensor, f'{i+1:06}'], uv2)
                k1, k2 = saved['frames//K'][i].astype(float), saved['frames//K'][i+1].astype(float)
                b1, b2 = float(saved['frames//baseline'][i]), float(saved['frames//baseline'][i+1])
                raw_joint = valid1 & valid2
                # Reference-implied stereo right endpoint must also be inbound.
                disp1, disp2 = np.zeros(len(z1)), np.zeros(len(z2))
                np.divide(k1[0, 0]*b1, z1, out=disp1, where=valid1)
                np.divide(k2[0, 0]*b2, z2, out=disp2, where=valid2)
                width = maps[sensor, f'{i:06}'].shape[1]
                joint = raw_joint & (uv1[:, 0]-disp1 >= 0)&(uv1[:, 0]-disp1 < width-1)&(uv2[:, 0]-disp2 >= 0)&(uv2[:, 0]-disp2 < width-1)
                count = int(joint.sum())
                row.update(native_factors=int(mask.sum()), source_supported=int(valid1.sum()),
                           target_supported=int(valid2.sum()), raw_joint_factors=int(raw_joint.sum()), joint_factors=count,
                           status='insufficient_joint_factors', information_rank_qualified=False)
                if count >= 3:
                    event = events[i, i+1]
                    parent = pose_xyzw(event['parent_pose_camera_map_xyzw'])
                    target = pose_xyzw(event['optimizer_pose_before_write_native_parameterization'])
                    rays = np.column_stack([z1[joint], (uv1[joint, 0]-k1[0, 2])*z1[joint]/k1[0, 0], (uv1[joint, 1]-k1[1, 2])*z1[joint]/k1[1, 1]])
                    world = rays@parent[:3, :3].T+parent[:3, 3]
                    points = (world-target[:3, 3])@target[:3, :3]
                    try:
                        native_readout(points, uv2[joint], disp2[joint], saved['match//pixel2_uv_cov'][mask][joint],
                                       saved['match//pixel2_disp_cov'][mask][joint], k2, b2)
                        row.update(status='eligible_conditional_information', information_rank_qualified=True)
                    except ValueError as exc:
                        row.update(status='invalid_conditional_information', information_reason=str(exc))
                dest = OUT/'factor-support'/f'{camera}-{i:06}.npz'; dest.parent.mkdir(exist_ok=True)
                np.savez_compressed(dest, joint_mask=joint, source_depth_m=z1, target_depth_m=z2,
                                    source_valid=valid1, target_valid=valid2, source_uv=uv1, target_uv=uv2)
                row['support_file'] = str(dest.relative_to(ROOT)); row['support_sha256'] = sha(dest)
                cases.append(row)
    result = {'recording': SEQUENCE, 'expected_native_pair_slots': 40, 'restored_depth_maps': 40,
              'cases': cases, 'statuses': dict(Counter(r['status'] for r in cases)),
              'eligible_cases': sum(r.get('information_rank_qualified', False) for r in cases),
              'support_rule': 'source and target own-camera nonzero nearest projected pixel; reference-implied right endpoints inbound; >=3 retained factors and valid6DoF conditional information',
              'pose_reference_errors_read': False, 'new_frontend_calls': 0, 'four_arm_solves': 0}
    write(OUT/'qualification.json', result)
    print({k: v for k, v in result.items() if k != 'cases'})


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['restore', 'qualify'])
    args = ap.parse_args()
    {'restore': restore, 'qualify': qualify}[args.action]()
