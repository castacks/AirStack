"""Fixed candidate correspondence/visibility pilot; never certifies physical matches."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quartiles(a):
    return np.quantile(a, [0, .25, .5, .75, 1]).tolist() if len(a) else None


def nearest(query, target):
    """Nearest label in normalized coordinates, rejecting ambiguous equal distances."""
    n = len(query)
    if not len(target):
        return np.full(n, -1, dtype=np.int64), np.full(n, np.inf), np.zeros(n, bool)
    distances, indices = cKDTree(target).query(query, k=2)
    return indices[:, 0], distances[:, 0], distances[:, 1] - distances[:, 0] > 1e-12


def frontmost(uv, z, valid, shape):
    """Unique nearest depth in each rounded target pixel, among observed source points."""
    h, w = shape
    indices = np.flatnonzero(valid)
    result = np.zeros(len(z), bool)
    if not len(indices):
        return result
    xy = np.rint(uv[indices]).astype(np.int64)
    cell = xy[:, 1] * w + xy[:, 0]
    minimum = np.full(h * w, np.inf)
    np.minimum.at(minimum, cell, z[indices])
    at_front = np.abs(z[indices] - minimum[cell]) <= 1e-12
    multiplicity = np.bincount(cell[at_front], minlength=h*w)
    result[indices] = at_front & (multiplicity[cell] == 1)
    return result


def cloud(depth, k):
    y, x = np.nonzero(depth > 0)
    z = depth[y, x]
    xy = np.column_stack((x, y))
    points = np.column_stack(((x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z))
    return xy, points


def direction(points, target_xy, k, rotation, translation, shape):
    transformed = points @ rotation.T + translation
    positive = transformed[:, 2] > 0
    uv = np.full((len(points), 2), np.nan)
    uv[positive] = transformed[positive, :2] / transformed[positive, 2, None] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    h, w = shape
    inside = positive & np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] <= w-1) & (uv[:, 1] >= 0) & (uv[:, 1] <= h-1)
    candidates = np.flatnonzero(inside)
    normalized_target = (target_xy - [k[0, 2], k[1, 2]]) / [k[0, 0], k[1, 1]]
    normalized_query = (uv[candidates] - [k[0, 2], k[1, 2]]) / [k[0, 0], k[1, 1]]
    j, distance, unique = nearest(normalized_query, normalized_target)
    match = np.full(len(points), -1, np.int64)
    errors = np.full(len(points), np.inf)
    unambiguous = np.zeros(len(points), bool)
    match[candidates], errors[candidates], unambiguous[candidates] = j, distance, unique
    zfront = frontmost(uv, transformed[:, 2], inside, shape)
    return {'points': transformed, 'uv': uv, 'inside': inside, 'match': match, 'distance': errors, 'unique': unambiguous, 'front': zfront}


def fixtures():
    j, d, unique = nearest(np.array([[0., 0.]]), np.array([[-1., 0.], [1., 0.]]))
    assert d[0] == 1 and not unique[0]
    j, d, unique = nearest(np.array([[0., 0.]]), np.empty((0, 2)))
    assert j[0] == -1 and np.isinf(d[0]) and not unique[0]
    uv = np.array([[1., 1.], [1.1, 1.1], [2., 2.], [2.1, 2.1]])
    assert frontmost(uv, np.array([2., 3., 4., 4.]), np.ones(4, bool), (4, 4)).tolist() == [True, False, False, False]
    k = np.eye(3)
    a = direction(np.array([[1., 1., 1.], [0., 0., -1.]]), np.array([[1, 1]]), k, np.eye(3), np.zeros(3), (4, 4))
    assert a['inside'].tolist() == [True, False] and a['match'][0] == 0 and a['distance'][0] == 0
    a = direction(np.empty((0, 3)), np.empty((0, 2)), k, np.eye(3), np.zeros(3), (4, 4))
    assert len(a['match']) == 0
    return 5


def overlay(root, pilot, seq, stem, pair, output):
    rgb = Image.open(pilot/'sync_data'/seq/'rgb/img_left'/f'{stem}.png').convert('RGB')
    thermal = np.asarray(Image.open(pilot/'sync_data'/seq/'thr/img_left'/f'{stem}.png'), float)
    lo, hi = np.percentile(thermal, [1, 99])
    display = np.uint8(np.clip((thermal-lo)/max(hi-lo, 1), 0, 1)*255)
    thr = Image.fromarray(display).convert('RGB')
    canvas = Image.new('RGB', (rgb.width+thr.width, max(rgb.height, thr.height)+40), 'white')
    canvas.paste(rgb, (0, 40)); canvas.paste(thr, (rgb.width, 40))
    draw = ImageDraw.Draw(canvas)
    draw.text((10, 10), f'ID {stem}: fixed evenly spaced candidate IDs; thermal display-only 1-99% stretch', fill='black')
    n = len(pair['rgb_xy'])
    selected = np.unique(np.linspace(0, n-1, min(12, n), dtype=int)) if n else []
    for label, i in enumerate(selected):
        a = pair['rgb_xy'][i]; b = pair['thr_xy'][i]
        color = ((label*71+60)%256, (label*113+90)%256, (label*47+130)%256)
        for x, y in [(int(a[0]), int(a[1])+40), (int(b[0])+rgb.width, int(b[1])+40)]:
            draw.ellipse((x-4, y-4, x+4, y+4), outline=color, width=2)
            draw.text((x+5, y-10), str(label), fill=color)
    canvas.save(output/f'overlay-{stem}.png')
    # Paired patches make structural agreement inspectable without model scores.
    patches = Image.new('RGB', (6*170, 245), 'white')
    pd = ImageDraw.Draw(patches)
    selected = np.unique(np.linspace(0, n-1, min(6, n), dtype=int)) if n else []
    for col, i in enumerate(selected):
        a = pair['rgb_xy'][i]; b = pair['thr_xy'][i]
        for image, xy, radius, top in [(rgb, a, 32, 35), (thr, b, 16, 135)]:
            x, y = map(int, xy)
            tile = image.crop((x-radius, y-radius, x+radius, y+radius)).resize((96, 96), Image.Resampling.NEAREST)
            patches.paste(tile, (col*170, top))
        pd.text((col*170, 2), f'pair {i}', fill='black')
        pd.text((col*170, 15), f'{a.tolist()} / {b.tolist()}', fill='black')
    patches.save(output/f'patches-{stem}.png')


def support(root, output, recovery):
    if output.exists():
        raise FileExistsError(output)
    base = root/'results/ms2-rain-replication'
    pilot, seq = base/'pilot', '_2021-08-06-16-19-00'
    metadata = json.loads((base/'calibration.json').read_text())
    c = {k: np.array(v['values'], float) for k, v in metadata.items()}
    r = c['R_nir2thr'] @ c['R_nir2rgb'].T
    t = (c['T_nir2thr'] - r @ c['T_nir2rgb']).reshape(3)/1000
    epsilon = 1/c['K_rgbL'][0, 0]
    original = json.loads((root/'results/ms2-cross-modal-feasibility/run-002/audit.json').read_text())['input_sha256']
    frozen = dict(original)
    for mod in ('rgb', 'thr'):
        for i in range(20):
            p = pilot/'sync_data'/seq/mod/'img_left'/f'{i:06d}.png'
            frozen[str(p.relative_to(root))] = sha(p)
    expected = json.loads(recovery.read_text())['member_sha256']
    for p, value in frozen.items():
        assert sha(root/p) == value == expected['ore-ms/'+p], p
    config = {'ids': [f'{i:06d}' for i in range(20)], 'source_commit': '609ea6f80', 'angular_radius_normalized': float(epsilon), 'depth_relative_sensitivities': [0, .01, .02, .05], 'depth_floor_m': 2/256, 'primary_uses_depth_threshold': False, 'nearest_tie_tolerance': 1e-12, 'zbuffer': 'unique nearest positive Z per rounded target pixel; exact-depth ties excluded', 'overlays': ['000000','000009','000019'], 'script_sha256': sha(__file__)}
    output.mkdir(parents=True)
    (output/'protocol.json').write_text(json.dumps(config, indent=2)+'\n')
    fixture_count = fixtures()
    rows, independent = [], 0
    for stem in config['ids']:
        depth = {m: np.asarray(Image.open(pilot/'proj_depth'/seq/m/'depth'/f'{stem}.png'), float)/256 for m in ('rgb','thr')}
        rgb_xy, rgb_p = cloud(depth['rgb'], c['K_rgbL'])
        thr_xy, thr_p = cloud(depth['thr'], c['K_thrL'])
        fw = direction(rgb_p, thr_xy, c['K_thrL'], r, t, depth['thr'].shape)
        bw = direction(thr_p, rgb_xy, c['K_rgbL'], r.T, -r.T@t, depth['rgb'].shape)
        mask = np.ones(len(rgb_p), bool)
        stages = {'rgb_nonzero': int(mask.sum())}
        steps = [('forward_in_FOV', fw['inside']), ('forward_unique_nearest', fw['unique']), ('forward_angular_bound', fw['distance'] <= epsilon), ('forward_frontmost', fw['front'])]
        for name, condition in steps:
            mask &= condition
            stages[name] = int(mask.sum())
        safe = np.maximum(fw['match'], 0)
        steps = [('reverse_in_FOV', bw['inside'][safe]), ('reverse_unique_nearest', bw['unique'][safe]), ('reverse_angular_bound', bw['distance'][safe] <= epsilon), ('reverse_frontmost', bw['front'][safe]), ('mutual_nearest', bw['match'][safe] == np.arange(len(rgb_p)))]
        for name, condition in steps:
            mask &= condition
            stages[name] = int(mask.sum())
        ids = np.flatnonzero(mask); j = fw['match'][ids]
        assert len(np.unique(j)) == len(j)
        xyz_thr_in_rgb = thr_p[j]@r - t@r
        pairs = {'rgb_xy': rgb_xy[ids], 'thr_xy': thr_xy[j], 'rgb_reference_xyz': rgb_p[ids], 'thr_reference_xyz_in_rgb': xyz_thr_in_rgb, 'forward_angular_distance': fw['distance'][ids], 'reverse_angular_distance': bw['distance'][j], 'source_supported_rgb_xy': rgb_xy, 'source_supported_thr_xy': thr_xy, 'rgb_support_mask': mask}
        dz_thr = fw['points'][ids, 2]-thr_p[j, 2]
        dz_rgb = bw['points'][j, 2]-rgb_p[ids, 2]
        mismatch = np.linalg.norm(rgb_p[ids]-xyz_thr_in_rgb, axis=1)
        pairs.update({'depth_difference_thr_m': dz_thr, 'depth_difference_rgb_m': dz_rgb, 'reference_point_distance_m': mismatch})
        sensitivities = []
        for rel in config['depth_relative_sensitivities']:
            agree = (abs(dz_thr) <= np.maximum(config['depth_floor_m'], rel*thr_p[j,2])) & (abs(dz_rgb) <= np.maximum(config['depth_floor_m'], rel*rgb_p[ids,2]))
            pairs[f'depth_consistent_{rel:g}'] = agree
            sensitivities.append({'relative_bound': rel, 'count': int(agree.sum()), 'fraction_of_candidates': float(agree.mean()) if len(agree) else None})
        # Scalar matrix actions and brute-force candidate query, separate from KD implementation.
        for points, k, rotation, translation, target_xy, result in [(rgb_p,c['K_thrL'],r,t,thr_xy,fw),(thr_p,c['K_rgbL'],r.T,-r.T@t,rgb_xy,bw)]:
            inside = np.flatnonzero(result['inside'])
            for idx in (inside[np.unique(np.linspace(0,len(inside)-1,min(3,len(inside)),dtype=int))] if len(inside) else []):
                q = [sum(float(rotation[a,b])*float(points[idx,b]) for b in range(3))+float(translation[a]) for a in range(3)]
                uv = [float(k[a,a])*q[a]/q[2]+float(k[a,2]) for a in range(2)]
                assert np.allclose(uv,result['uv'][idx],atol=1e-10,rtol=0)
                distances = np.sqrt(((target_xy[:,0]-uv[0])/k[0,0])**2+((target_xy[:,1]-uv[1])/k[1,1])**2)
                assert abs(distances.min()-result['distance'][idx]) < 1e-12
                independent += 2
        np.savez_compressed(output/f'pairs-{stem}.npz', **pairs)
        row = {'stem':stem,'stages':stages,'thr_nonzero':len(thr_p),'candidate_fraction_rgb':len(ids)/len(rgb_p),'candidate_fraction_thr':len(ids)/len(thr_p),'point_distance_m_quartiles':quartiles(mismatch),'signed_depth_thr_m_quartiles':quartiles(dz_thr),'signed_depth_rgb_m_quartiles':quartiles(dz_rgb),'depth_sensitivities':sensitivities}
        rows.append(row)
        if stem in config['overlays']:
            overlay(root,pilot,seq,stem,pairs,output)
    assert all(sha(root/p)==value for p,value in frozen.items())
    result={'status':'PASS','physical_correspondences_verified':False,'new_model_inference':False,'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'protocol':config,'fixture_checks':fixture_count,'independent_scalar_projection_and_query_checks':independent,'input_sha256':frozen,'rows':rows}
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Fixed support pilot PASS:',len(rows),'pairs;',len(frozen),'frozen inputs;',independent,'scalar checks.')
    for key in rows[0]['stages']:
        print(key, 'min/median/max', np.quantile([a['stages'][key] for a in rows],[0,.5,1]).tolist())
    print('3D mismatch per-image medians min/median/max',np.quantile([a['point_distance_m_quartiles'][2] for a in rows],[0,.5,1]).tolist())
    for i, rel in enumerate(config['depth_relative_sensitivities']):
        print('Depth bound',rel,'median candidate fraction',float(np.median([a['depth_sensitivities'][i]['fraction_of_candidates'] for a in rows])))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--recovery-manifest',type=Path,required=True)
    args=p.parse_args()
    support(args.root.resolve(),args.output,args.recovery_manifest)
