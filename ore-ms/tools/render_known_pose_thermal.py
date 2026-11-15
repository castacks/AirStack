"""Small exact-geometry stereo scenes with existing thermal image textures."""
import argparse
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates

from qualify_native_frontend import now, read, recipe, sha, write


def render(texture, K, rotation, center, depths, near_halfplane):
    height, width = texture.shape
    v, u = np.mgrid[:height, :width]
    local = np.stack([(u-K[0, 2])/K[0, 0], (v-K[1, 2])/K[1, 1], np.ones_like(u)], axis=-1)
    directions = local @ rotation.T
    candidates = []
    distances = []
    for index, depth in enumerate(depths):
        scale = (depth-center[2])/directions[..., 2]
        point = center + scale[..., None]*directions
        valid = (scale > 0) & (directions[..., 2] > 0)
        if near_halfplane and index == 0:
            valid &= point[..., 0] <= 0
        candidates.append(point)
        distances.append(np.where(valid, scale, np.inf))
    distances = np.stack(distances)
    selected = np.argmin(distances, axis=0)
    world = np.take_along_axis(np.stack(candidates), selected[None, ..., None], axis=0)[0]
    geometry_valid = np.isfinite(np.min(distances, axis=0))
    canonical_u = K[0, 0]*world[..., 0]/world[..., 2]+K[0, 2]
    canonical_v = K[1, 1]*world[..., 1]/world[..., 2]+K[1, 2]
    for coordinate, high in [(canonical_u, width-1), (canonical_v, height-1)]:
        coordinate[np.abs(coordinate) < 1e-9] = 0.
        coordinate[np.abs(coordinate-high) < 1e-9] = float(high)
    texture_valid = geometry_valid & (canonical_u >= 0) & (canonical_u <= width-1) & (canonical_v >= 0) & (canonical_v <= height-1)
    sampled = map_coordinates(texture.astype(float)/255, [canonical_v, canonical_u], order=1, mode='constant', cval=0., prefilter=False)
    grayscale = np.rint(np.clip(sampled, 0, 1)*255).astype(np.uint8)
    return {'images': grayscale, 'samples': sampled, 'world': world, 'surface': selected.astype(np.uint8), 'geometry_valid': geometry_valid, 'texture_valid': texture_valid, 'texture_uv': np.stack([canonical_u, canonical_v], axis=-1)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    plan, out = read(args.protocol), Path(args.output)
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path
    out.mkdir(parents=True, exist_ok=False)
    K = np.asarray(plan['K'])
    angle = np.deg2rad(plan['yaw_deg'])
    R = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    t = np.asarray(plan['translation_m'])
    baseline = np.array([plan['baseline_m'], 0, 0])
    rotations = np.stack([np.eye(3), np.eye(3), R, R])
    centers = np.stack([np.zeros(3), baseline, t, t+R@baseline])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    records = []
    prepare = recipe()
    for condition, source in plan['textures'].items():
        raw = cv2.imread(source, cv2.IMREAD_UNCHANGED)
        texture = prepare(raw)
        assert texture.dtype == np.uint8 and texture.shape == (256, 640)
        for scene in plan['scenes']:
            key = condition+'-'+scene['id']
            views = [render(texture, K, Q, c, scene['depths_m'], scene['near_halfplane']) for Q, c in zip(rotations, centers, strict=True)]
            repeat = render(texture, K, rotations[2], centers[2], scene['depths_m'], scene['near_halfplane'])
            assert all(np.array_equal(repeat[k], views[2][k]) for k in repeat), 'render repeat'
            arrays = {k: np.stack([view[k] for view in views]) for k in views[0]}
            arrays.update(texture=texture, K=K, rotations=rotations, centers=centers, target_to_source=T)
            path = out/(key+'.npz')
            np.savez_compressed(path, **arrays)
            mosaic = np.concatenate([np.concatenate([arrays['images'][0], arrays['images'][1]], axis=1), np.concatenate([arrays['images'][2], arrays['images'][3]], axis=1)], axis=0)
            cv2.imwrite(str(out/(key+'.png')), mosaic)
            records.append({'id': key, 'texture_condition': condition, 'scene': scene, 'path': str(path), 'sha256': sha(path), 'texture_support_fractions': arrays['texture_valid'].mean((1, 2)).tolist(), 'views': 4, 'pixels_per_view': 256*640})
    write(out/'inventory.json', {'created_utc': now(), 'records': records, 'no_model_inference': True, 'no_natural_pose_reference': True, 'protocol_sha256': sha(args.protocol)})
    write(out/'artifact-lock.json', {'created_utc': now(), 'files': {p.name: sha(p) for p in out.iterdir() if p.is_file()}})
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path
    print('Rendered', len(records), 'scenes; no model inference')


if __name__ == '__main__':
    main()
