"""Fixed same-modality patch diagnostics for CATS signed disparity conventions."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2
import numpy as np
from scipy.ndimage import map_coordinates


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def dump(p, data):
    p.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def patches(image, x, y, radius=4):
    h, w = image.shape
    inside = (x >= radius) & (x <= w - 1 - radius) & (y >= radius) & (y <= h - 1 - radius)
    dy, dx = np.mgrid[-radius:radius+1, -radius:radius+1]
    xx, yy = x[:, None] + dx.ravel(), y[:, None] + dy.ravel()
    values = map_coordinates(image, [yy, xx], order=1, mode='constant', cval=np.nan, prefilter=False)
    values[~inside] = np.nan
    return values, inside


def ncc(a, b):
    a, b = a - a.mean(axis=1, keepdims=True), b - b.mean(axis=1, keepdims=True)
    den = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
    valid = np.isfinite(den) & (den > 0)
    result = np.full(len(a), np.nan)
    result[valid] = np.sum(a[valid] * b[valid], axis=1) / den[valid]
    return result


def stats(values):
    finite = values[np.isfinite(values)]
    return {'n': len(values), 'valid_ncc': len(finite), 'unavailable_ncc': len(values)-len(finite),
            'median_ncc': float(np.median(finite)) if len(finite) else None,
            'q25_q75_ncc': np.quantile(finite, [.25, .75]).tolist() if len(finite) else None}


def run(source, output):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    declaration = json.loads((source / 'sample-declaration.json').read_text())
    sample = source / 'sample' / declaration['scene']
    access = json.loads((source / 'access-validation.json').read_text())
    hashes = {str(source / 'sample' / name): h for name, h in access['sample_sha256'].items()}
    assert all(sha(p) == h for p, h in hashes.items())
    protocol = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
        'scene': declaration['scene'], 'hypotheses': ['left_plus', 'left_minus', 'right_plus', 'right_minus', 'left_zero'],
        'interpretation': 'stored d at hypothesized anchor; opposite x=anchor x +/- d; y unchanged',
        'sampling': 'up to1024 evenly spaced indices from row-major finite sparse labels; no mask/sign removal',
        'patch': '9x9 zero-mean NCC, bilinear sampling; no offset fitting, no feature/model inference',
        'cross_modality': 'mask/reference counts only; intensity NCC not used to infer cross-modality convention',
        'source_sha256': sha(__file__), 'input_sha256': hashes}
    dump(output / 'protocol.json', protocol)
    report = {}
    for camera in ('color', 'thermal', 'cross'):
        mask = cv2.imread(str(sample / f'rectified/{camera}/mask.png'), cv2.IMREAD_UNCHANGED)
        d = np.loadtxt(sample / f'gt_disparity/{camera}/gt_disparity.txt', delimiter=',')
        leftname = 'left_color_default.png' if camera == 'color' else 'left_thermal_default.png'
        rightname = 'right_thermal_default.png' if camera == 'thermal' else 'right_color_default.png'
        raw = [cv2.imread(str(sample / f'rectified/{camera}' / p), cv2.IMREAD_UNCHANGED) for p in (leftname, rightname)]
        black = [np.all(a == 0, axis=2) if a.ndim == 3 else a == 0 for a in raw]
        counts = {str(value): {'pixels': int((mask == value).sum()),
            'finite_reference': int(((mask == value) & np.isfinite(d)).sum()),
            'left_allzero': int(((mask == value) & black[0]).sum()),
            'right_allzero_same_coordinate': int(((mask == value) & black[1]).sum())} for value in (0, 255)}
        item = {'mask_reference_counts': counts, 'hypotheses': {}}
        report[camera] = item
        if camera == 'cross':
            item['ncc_status'] = 'NOT_APPLIED_CROSS_MODALITY'
            continue
        images = [cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(float) if a.ndim == 3 else a.astype(float) for a in raw]
        ids = np.flatnonzero(np.isfinite(d))
        ids = ids[np.linspace(0, len(ids)-1, min(1024, len(ids)), dtype=int)]
        y, x = np.unravel_index(ids, d.shape)
        disparity = d[y, x]
        saved = {'x': x, 'y': y, 'stored_disparity': disparity, 'mask_value': mask[y, x]}
        for hypothesis in protocol['hypotheses']:
            anchor, direction = hypothesis.split('_')
            index = 0 if anchor == 'left' else 1
            shift = {'plus': disparity, 'minus': -disparity, 'zero': np.zeros(len(ids))}[direction]
            a, inbound_a = patches(images[index], x, y)
            b, inbound_b = patches(images[1-index], x+shift, y)
            values = ncc(a, b)
            saved[hypothesis] = values
            saved[hypothesis+'_inbounds'] = inbound_a & inbound_b
            summary = {'all': stats(values), 'out_of_patch_bounds': int((~(inbound_a & inbound_b)).sum())}
            for value in (0, 255):
                for sign, selected in [('all', np.ones(len(ids), bool)), ('negative', disparity < 0), ('zero', disparity == 0), ('positive', disparity > 0)]:
                    summary[f'mask_{value}_{sign}'] = stats(values[(mask[y, x] == value) & selected])
            item['hypotheses'][hypothesis] = summary
        item['paired_left_minus_vs_plus'] = {}
        for group, selected in [('all', np.ones(len(ids), bool)), ('mask_255', mask[y, x] == 255)]:
            a, b = saved['left_minus'], saved['left_plus']
            common = selected & np.isfinite(a) & np.isfinite(b)
            item['paired_left_minus_vs_plus'][group] = {'available_pairs': int(common.sum()),
                'minus_better': int(np.sum(a[common] > b[common])), 'equal': int(np.sum(a[common] == b[common])),
                'plus_better': int(np.sum(a[common] < b[common])),
                'median_paired_ncc_difference_minus_plus': float(np.median(a[common]-b[common])) if common.any() else None}
        np.savez_compressed(output / f'{camera}-patch-diagnostics.npz', **saved)
    assert all(sha(p) == h for p, h in hashes.items())
    dump(output / 'report.json', report)
    print(json.dumps({c: {k: v['mask_255_all'] for k, v in r['hypotheses'].items()} for c, r in report.items()}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    run(args.source, args.output)
