"""Audit supplied MS2 cross-camera geometry; candidates are not correspondences.

No model outputs are read. Run with isolated numpy/scipy/Pillow dependencies.
"""
import argparse
import hashlib
import json
import platform
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quantiles(values):
    return np.quantile(values, [0, .25, .5, .75, 1]).tolist() if len(values) else None


def audit(root, output, recovery):
    if output.exists():
        raise FileExistsError(output)
    base = root / 'results/ms2-rain-replication'
    pilot = base / 'pilot'
    seq = '_2021-08-06-16-19-00'
    calibration = base / 'calibration.json'
    raw = json.loads(calibration.read_text())
    c = {key: np.asarray(value['values'], dtype=float) for key, value in raw.items()}
    inputs = [calibration, base / 'source-manifest.json']
    for modality in ('rgb', 'thr'):
        inputs.append(pilot / 'sync_data' / seq / modality / 'img_left_timestamp.txt')
        inputs.extend(pilot / 'proj_depth' / seq / modality / 'depth' / f'{i:06d}.png' for i in range(20))
    inputs.extend([pilot / 'sync_data' / seq / 'readme.txt', pilot / 'proj_depth' / seq / 'readme.txt'])
    frozen = {str(p.relative_to(root)): digest(p) for p in inputs}
    recorded = json.loads(recovery.read_text())['member_sha256']
    for name, value in frozen.items():
        assert recorded['ore-ms/' + name] == value, name
    rotations = {}
    for modality in ('rgb', 'thr'):
        r = c[f'R_nir2{modality}']
        rotations[modality] = {'determinant': float(np.linalg.det(r)), 'orthogonality_max': float(np.abs(r.T @ r - np.eye(3)).max())}
        assert abs(np.linalg.det(r) - 1) < 1e-6
        assert rotations[modality]['orthogonality_max'] < 1e-6
    # X_rgb = R_nr X_nir + t_nr; X_thr = R_nt X_nir + t_nt.
    r = c['R_nir2thr'] @ c['R_nir2rgb'].T
    t = (c['T_nir2thr'] - r @ c['T_nir2rgb']).reshape(3) / 1000
    assert np.allclose(r.T @ r, np.eye(3), atol=1e-12, rtol=0)
    rows = []
    scalar_checks = 0
    for source, target, rotation, translation in [('rgb', 'thr', r, t), ('thr', 'rgb', r.T, -r.T @ t)]:
        ks, kt = c[f'K_{source}L'], c[f'K_{target}L']
        for index in range(20):
            stem = f'{index:06d}'
            ds, dt = [np.asarray(Image.open(pilot / 'proj_depth' / seq / mod / 'depth' / f'{stem}.png'), dtype=np.float64) / 256 for mod in (source, target)]
            sy, sx = np.nonzero(ds > 0)
            ty, tx = np.nonzero(dt > 0)
            z = ds[sy, sx]
            points = np.column_stack(((sx - ks[0, 2]) * z / ks[0, 0], (sy - ks[1, 2]) * z / ks[1, 1], z))
            projected = points @ rotation.T + translation
            back = (projected - translation) @ rotation
            assert np.allclose(points, back, atol=1e-10, rtol=0)
            uv = projected[:, :2] / projected[:, 2, None] * [kt[0, 0], kt[1, 1]] + [kt[0, 2], kt[1, 2]]
            assert np.allclose(points[:, :2] / points[:, 2, None] * [ks[0, 0], ks[1, 1]] + [ks[0, 2], ks[1, 2]], np.column_stack((sx, sy)), atol=1e-10, rtol=0)
            # Independently reconstruct selected points and projections with scalar arithmetic.
            for j in sorted(set([0, len(z)//2, len(z)-1])):
                original = [(int(sx[j])-float(ks[0,2]))*float(z[j])/float(ks[0,0]), (int(sy[j])-float(ks[1,2]))*float(z[j])/float(ks[1,1]), float(z[j])]
                q = [sum(float(rotation[a,b])*original[b] for b in range(3))+float(translation[a]) for a in range(3)]
                independent = [float(kt[0,0])*q[0]/q[2]+float(kt[0,2]), float(kt[1,1])*q[1]/q[2]+float(kt[1,2])]
                assert np.allclose(independent, uv[j], atol=1e-10, rtol=0)
                scalar_checks += 1
            h, w = dt.shape
            valid = (projected[:,2] > 0) & (uv[:,0] >= 0) & (uv[:,0] <= w-1) & (uv[:,1] >= 0) & (uv[:,1] <= h-1)
            candidates = uv[valid]
            tree = cKDTree(np.column_stack((tx,ty)))
            distance, nearest = tree.query(candidates)
            for j in sorted(set([0, len(candidates)//2, len(candidates)-1])):
                brute = np.sqrt((tx-candidates[j,0])**2+(ty-candidates[j,1])**2).min()
                assert abs(brute-distance[j]) < 1e-10
                scalar_checks += 1
            reference_z = dt[ty[nearest], tx[nearest]]
            difference = projected[valid,2]-reference_z
            row = {'source':source,'target':target,'stem':stem,'source_supported':len(z),'positive_depth':int((projected[:,2]>0).sum()),'target_supported':len(tx),'projected_inside_target':int(valid.sum()),'nearest_label_distance_px_quartiles':quantiles(distance),'radii':[]}
            for radius in (.5,1.,2.):
                keep = distance <= radius
                row['radii'].append({'radius_px':radius,'candidate_count':int(keep.sum()),'unique_target_pixels':int(len(np.unique(nearest[keep]))),'fraction_of_source_support':float(keep.sum()/len(z)),'absolute_depth_difference_m_quartiles':quantiles(abs(difference[keep])),'absolute_relative_depth_difference_quartiles':quantiles(abs(difference[keep])/reference_z[keep])})
            rows.append(row)
    times = {}
    for mod in ('rgb','thr'):
        text=(pilot/'sync_data'/seq/mod/'img_left_timestamp.txt').read_text().splitlines()
        times[mod] = [int(v.strip()) for v in text[:20]]
        manifest = json.loads((base/'source-manifest.json').read_text())
        assert all(times[mod][i] == manifest['records'][i]['cameras'][mod]['left']['recorded_time_ns'] for i in range(20))
    deltas = [times['thr'][i]-times['rgb'][i] for i in range(20)]
    assert all(digest(root/name)==sha for name,sha in frozen.items())
    result={'status':'PASS','scope':'documented-transform geometric candidates, not verified shared physical points','timestamp':datetime.now(ZoneInfo('America/New_York')).isoformat(),'python':platform.python_version(),'numpy':np.__version__,'rotation_checks':rotations,'rgb_to_thr_rotation':r.tolist(),'rgb_to_thr_translation_m':t.tolist(),'camera_separation_m':float(np.linalg.norm(t)),'left_thr_minus_rgb_time_ns':deltas,'absolute_left_time_difference_ms_quartiles':quantiles(np.abs(deltas)/1e6),'scalar_projection_and_nearest_checks':scalar_checks,'input_sha256':frozen,'recovery_manifest_sha256':digest(recovery),'script_sha256':digest(Path(__file__)),'rows':rows}
    output.mkdir(parents=True)
    (output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','input_sha256','rgb_to_thr_rotation')},indent=2))
    for source in ('rgb','thr'):
        group=[row for row in rows if row['source']==source]
        print(source,'median projected-in-FOV fraction',float(np.median([a['projected_inside_target']/a['source_supported'] for a in group])))
        for index,radius in enumerate((.5,1.,2.)):
            selected=[a['radii'][index] for a in group]
            print(' radius',radius,'median candidate fraction',float(np.median([a['fraction_of_source_support'] for a in selected])),'median of per-image depth discrepancy medians',float(np.median([a['absolute_depth_difference_m_quartiles'][2] for a in selected if a['candidate_count']])),'relative',float(np.median([a['absolute_relative_depth_difference_quartiles'][2] for a in selected if a['candidate_count']])))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--recovery-manifest',type=Path,required=True)
    args=parser.parse_args()
    audit(args.root.resolve(),args.output,args.recovery_manifest)
