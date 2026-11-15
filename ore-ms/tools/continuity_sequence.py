"""Fixed synthetic interruption assay with explicit disconnected-frame semantics.

Uses native dense predictions/selection/covariance and the existing optical IRLS
harness. This is not the historical full native VO or a new relocalization system.
"""
import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from qualify_native_frontend import read, write, sha, now, recipe, deterministic, load_model
from render_known_pose_thermal import render
from paired_frontend_assessment import native_algorithms, pick, packet, fit


def verify(plan):
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path


def edges():
    return [(i, i+1) for i in range(9)]+[(2, 6), (0, 6), (0, 0)]


def assemble(transforms):
    """Only measurement transforms enter this function; no renderer/truth input."""
    def chain(start, indices):
        result = {indices[0]: start}
        for a, b in zip(indices[:-1], indices[1:]):
            edge = transforms.get((a, b))
            result[b] = None if result[a] is None or edge is None else result[a]@edge
        return result
    continuous = chain(np.eye(4), list(range(10)))
    retained = chain(np.eye(4), [0, 1, 2, 6, 7, 8, 9])
    local = chain(np.eye(4), [6, 7, 8, 9])
    # A hold-pose splice is a diagnostic assumption of zero gap motion.
    splice = {i: None if retained[2] is None or local[i] is None else retained[2]@local[i] for i in local}
    anchor = chain(transforms.get((0, 6)), [6, 7, 8, 9])
    return {'uninterrupted': continuous, 'retained_gap': retained,
            'local_restart': local, 'hold_pose_splice': splice,
            'visual_anchor': anchor}


def render_sequences(plan, out):
    out.mkdir(parents=True, exist_ok=False)
    K = np.array(plan['K'])
    poses = []
    for i in range(10):
        T = np.eye(4)
        angle = np.deg2rad(i*plan['yaw_per_frame_deg'])
        T[:3, :3] = [[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]]
        T[:3, 3] = i*np.array(plan['center_increment_m'])
        poses.append(T)
    records = []
    for condition, source in plan['textures'].items():
        texture = recipe()(cv2.imread(source, cv2.IMREAD_UNCHANGED))
        for scene in plan['scenes']:
            images, supports = [], []
            for T in poses:
                for side in [0, 1]:
                    center = T[:3, 3]+T[:3, :3]@np.array([side*plan['baseline_m'], 0., 0.])
                    view = render(texture, K, T[:3, :3], center, scene['depths_m'], scene['near_halfplane'])
                    images.append(view['images'])
                    supports.append(view['texture_valid'])
            key = condition+'-'+scene['id']
            path = out/(key+'.npz')
            np.savez_compressed(path, images=np.array(images).reshape(10, 2, 256, 640),
                                texture_valid=np.array(supports).reshape(10, 2, 256, 640),
                                world_from_camera=np.array(poses), texture=texture)
            cv2.imwrite(str(out/(key+'.png')), np.concatenate([images[i*2] for i in [0, 2, 6, 9]], axis=1))
            records.append({'id': key, 'condition': condition, 'scene': scene, 'path': str(path), 'sha256': sha(path)})
    write(out/'inventory.json', {'created_utc': now(), 'protocol_sha256': sha(plan['protocol_path']), 'records': records})


def capture(plan, out):
    import torch
    out.mkdir(parents=True, exist_ok=False)
    deterministic()
    model, loading = load_model(plan['checkpoint'])
    assert loading['migration'] == 'none'
    state = lambda: {k: hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k, v in model.state_dict().items()}
    initial = state()
    selector, covariance = native_algorithms()
    rows = []
    camera = {'K': plan['K'], 'baseline_m': plan['baseline_m']}
    for scene in read(plan['inventory'])['records']:
        assert sha(scene['path']) == scene['sha256']
        with np.load(scene['path']) as z:
            images = z['images'].astype(np.float32)/255  # No true-pose array read.
        for a, b in edges():
            source = np.stack([images[a, 0], images[b, 0], images[a, 0]])
            target = np.stack([images[a, 1], images[b, 1], images[b, 0]])
            first = torch.from_numpy(np.repeat(source[:, None], 3, axis=1)).cuda()
            second = torch.from_numpy(np.repeat(target[:, None], 3, axis=1)).cuda()
            with torch.inference_mode():
                f, c = model.inference(first, second)
            assert torch.isfinite(f).all() and torch.isfinite(c).all() and (c > 0).all()
            f, c = f.cpu().numpy(), c.cpu().numpy()
            folder = out/scene['id']/f'{a:02}-{b:02}'
            folder.mkdir(parents=True)
            dense = folder/'dense.npz'
            np.savez_compressed(dense, flow=f, covariance=c)
            ids = pick(f, c, camera, selector, plan['selection_seed'])
            try:
                p = packet(f, c, camera, ids, covariance)
                packet_path = folder/'packet.npz'
                np.savez_compressed(packet_path, **p)
                result = fit(p, plan['solver'])
                result.update(packet_path=str(packet_path), packet_sha256=sha(packet_path), points=len(p['X0']))
            except (ValueError, np.linalg.LinAlgError) as exc:
                result = {'successful': False, 'status': 'INVALID_PACKET', 'reason': str(exc)}
            row = {'scene': scene['id'], 'source': a, 'target': b, 'dense_path': str(dense),
                   'dense_sha256': sha(dense), 'source_input_sha256': hashlib.sha256(source.tobytes()).hexdigest(),
                   'target_input_sha256': hashlib.sha256(target.tobytes()).hexdigest(),
                   'proposed_ids': ids.tolist(), 'fit': result}
            rows.append(row)
            write(folder/'receipt.json', row)
            print(scene['id'], a, b, result['status'], flush=True)
    assert initial == state()
    write(out/'predictions.json', {'created_utc': now(), 'rows': rows, 'loading': loading,
                                  'model_state_unchanged': True, 'true_pose_read': False})
    write(out/'prediction-lock.json', {'created_utc': now(), 'files': {str(p.relative_to(out)): sha(p) for p in out.rglob('*') if p.is_file()}})


def error(est, target):
    delta = np.linalg.inv(target)@est
    return {'translation_m': float(np.linalg.norm(delta[:3, 3])),
            'rotation_deg': float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude()))}


def score(plan, out):
    capture_dir = Path(plan['capture_output'])
    for path, digest in read(capture_dir/'prediction-lock.json')['files'].items():
        assert sha(capture_dir/path) == digest
    predictions = read(capture_dir/'predictions.json')['rows']
    reports = []
    for scene in read(plan['inventory'])['records']:
        with np.load(scene['path']) as z:
            truth = z['world_from_camera']
        selected = [r for r in predictions if r['scene'] == scene['id']]
        measurements = {(r['source'], r['target']): np.array(r['fit']['target_to_source']) if r['fit']['successful'] else None for r in selected}
        trajectories = assemble(measurements)
        arms = {}
        for name, poses in trajectories.items():
            values = []
            for i in [6, 7, 8, 9]:
                T = poses.get(i)
                target = np.linalg.inv(truth[6])@truth[i] if name == 'local_restart' else truth[i]
                values.append({'frame': i, 'available': T is not None, 'pose': None if T is None else T.tolist(),
                               'error': None if T is None else error(T, target)})
            usable = [v['error'] for v in values if v['available']]
            arms[name] = {'coordinate_frame': 'restart_at6_unconnected_to_world' if name == 'local_restart' else 'original_world',
                          'global_recovery_claim': False,
                          'available_postgap': len(usable), 'planned_postgap': 4, 'rows': values,
                          'translation_rmse_m': None if not usable else float(np.sqrt(np.mean([v['translation_m']**2 for v in usable]))),
                          'rotation_rmse_deg': None if not usable else float(np.sqrt(np.mean([v['rotation_deg']**2 for v in usable])))}
        null = next(r for r in selected if r['source'] == r['target'])
        reports.append({'scene': scene['id'], 'arms': arms, 'edge_successes': sum(r['fit']['successful'] for r in selected),
                        'null_error': None if not null['fit']['successful'] else error(np.array(null['fit']['target_to_source']), np.eye(4))})
    out.mkdir(parents=True, exist_ok=False)
    write(out/'scores.json', {'created_utc': now(), 'protocol_sha256': sha(plan['protocol_path']), 'records': reports,
                             'postgap_population': [6, 7, 8, 9], 'local_restart_has_no_global_score': True,
                             'scope': 'Six exposed artificial sequences, fixed optical IRLS harness; descriptive baseline mechanism comparison, not physical recovery or new method.'})
    print('Scored', len(reports), 'sequences; local restart errors remain local', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['render', 'capture', 'score'])
    ap.add_argument('--protocol', required=True)
    args = ap.parse_args()
    plan = read(args.protocol)
    verify(plan)
    {'render': render_sequences, 'capture': capture, 'score': score}[args.mode](plan, Path(plan[args.mode+'_output']))
    verify(plan)


if __name__ == '__main__':
    main()
