"""Frozen EXP045 stereo/depth proxy diagnostic; no inference or physical GT claim."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2
import numpy as np
from scipy.stats import rankdata

VARIANTS = ('depth', 'depth_multi', 'depth_filtered')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def correlation(uncertainty, error):
    if len(error) < 2 or np.ptp(uncertainty) == 0 or np.ptp(error) == 0:
        return None
    return float(np.corrcoef(rankdata(uncertainty), rankdata(error))[0, 1])


def metrics(residual_px, covariance_px, focal):
    if not len(residual_px):
        return {'count': 0, 'availability': 'empty-support'}
    result = {'count': len(residual_px), 'availability': 'available'}
    for units, residual, covariance in [
            ('pixel', residual_px, covariance_px),
            ('normalized_plane', residual_px / focal, covariance_px / focal ** 2)]:
        norm = np.linalg.norm(residual, axis=1)
        trace = covariance.sum(axis=1)
        values = {'l2_median': np.median(norm), 'l2_p90': np.percentile(norm, 90),
                  'signed_u_median': np.median(residual[:, 0]),
                  'signed_v_median': np.median(residual[:, 1]),
                  'abs_u_median': np.median(np.abs(residual[:, 0])),
                  'abs_v_median': np.median(np.abs(residual[:, 1])),
                  'abs_u_p90': np.percentile(np.abs(residual[:, 0]), 90),
                  'abs_v_p90': np.percentile(np.abs(residual[:, 1]), 90),
                  'covariance_trace_median': np.median(trace)}
        result[units] = {k: float(v) for k, v in values.items()}
        result[units]['uncertainty_error_spearman'] = correlation(trace, norm)
    return result


def aggregate(records):
    result = {}
    for modality in ('rgb', 'thr'):
        result[modality] = {}
        for variant in VARIANTS:
            group = [r for r in records if r['modality'] == modality and r['variant'] == variant]
            assert len(group) == 20
            result[modality][variant] = {}
            for support in ('own_support', 'common_support'):
                summaries = [r[support] for r in group]
                summary = {'images': 20, 'available_images': sum(s['count'] > 0 for s in summaries),
                           'support_count_sum_not_independent_n': sum(s['count'] for s in summaries)}
                for units in ('pixel', 'normalized_plane'):
                    summary[units] = {}
                    keys = set().union(*(s.get(units, {}).keys() for s in summaries))
                    for key in sorted(keys):
                        values = [s[units][key] for s in summaries if s.get(units, {}).get(key) is not None]
                        summary[units][key] = ({'available_images': len(values), 'median': float(np.median(values)),
                                                'min': min(values), 'max': max(values)} if values else
                                               {'available_images': 0, 'median': None, 'min': None, 'max': None})
                result[modality][variant][support] = summary
    return result


def run(root, manifest_path, protocol_path, output):
    root = Path(root).resolve()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    started = datetime.now(ZoneInfo('America/New_York')).isoformat()
    manifest = json.loads(Path(manifest_path).read_text())
    protocol = json.loads(Path(protocol_path).read_text())
    assert not protocol['physical_cross_modal_error_eligible']
    assert manifest['sequence'] == protocol['sequence']
    assert [r['stem'] for r in manifest['records']] == [f'{i:06d}' for i in range(20)]
    frozen = dict(protocol['frozen_inputs'])
    for path in (manifest_path, protocol_path, Path(__file__)):
        frozen[str(Path(path).resolve())] = sha(path)
    reports = {}
    stereo = {}
    for modality, name in [('rgb', 'rgb-run-001'), ('thr', 'thermal-run-001')]:
        folder = root / 'results/ms2-frontend-smoke' / name
        path = folder / 'report.json'
        frozen[str(path)] = sha(path)
        report = json.loads(path.read_text())
        assert report['status'] == 'PASS' and report['frozen_inputs_unchanged']
        assert report['arguments']['modalities'] == [modality]
        frozen.update(report['frozen_inputs'])
        checkpoint = report['modalities'][modality]
        frozen[checkpoint['checkpoint']] = checkpoint['checkpoint_sha256']
        records = [p for p in report['pairs'] if p['modality'] == modality and p['kind'] == 'stereo']
        assert len(records) == 20 and {tuple(p['indices']) for p in records} == {(i, i) for i in range(20)}
        for record in records:
            path = folder / record['file']
            frozen[str(path)] = record['sha256']
            stereo[modality, record['indices'][0]] = (path, record)
        reports[modality] = report
    assert all(sha(p) == h for p, h in frozen.items()), 'Frozen prerequisite mismatch'
    (output / 'frozen-inputs.json').write_text(json.dumps(frozen, indent=2) + '\n')
    config = {'primary': 'depth', 'sensitivities': list(VARIANTS[1:]),
              'source_ids': protocol['source_ids'], 'population': 'nonzero depth and 0 <= u-fx*B/Z < width-1',
              'common_support': 'intersection of all3 depth support masks within each modality/frame',
              'residual': '[flow_x+fx*B/Z, flow_y]', 'normalized_residual': '[du/fx,dv/fy]',
              'normalized_variance': '[var_u/fx^2,var_v/fy^2]',
              'association': 'per-image Spearman(trace(diagonal variance),L2 residual)',
              'aggregation': 'median/min/max of20 image summaries; no pixel pooling',
              'physical_cross_modal_error_eligible': False, 'joint_timing_eligible': 0,
              'new_model_inference': False}
    (output / 'protocol.json').write_text(json.dumps(config, indent=2) + '\n')
    records = []
    for modality in ('rgb', 'thr'):
        for i, row in enumerate(manifest['records']):
            camera = row['cameras'][modality]
            focal = np.asarray([camera['K'][0][0], camera['K'][1][1]], dtype=np.float64)
            path, source = stereo[modality, i]
            with np.load(path, allow_pickle=False) as data:
                flow = data['flow'][0].astype(np.float64).transpose(1, 2, 0)
                covariance = data['covariance'][0].astype(np.float64).transpose(1, 2, 0)
                assert data['flow'].dtype == data['covariance'].dtype == np.float32
                assert data['source_ids'].tolist() == [row['stem'], row['stem']]
                times = [camera[s]['recorded_time_ns'] for s in ('left', 'right')]
                assert data['input_time_ns'].tolist() == times
                assert data['camera_times_ns'].tolist() == times + times
            expected = (*camera['left']['shape'][:2], 2)
            assert flow.shape == covariance.shape == expected
            if not (np.isfinite(flow).all() and np.isfinite(covariance).all() and (covariance > 0).all()):
                raise ValueError(f'Preserve invalid original output: {modality}/{i}')
            maps = {}
            masks = {}
            for variant in VARIANTS:
                rec = row['references'][modality][variant]
                depth = cv2.imread(str(root / rec['path']), cv2.IMREAD_UNCHANGED)
                assert depth.dtype == np.uint16 and depth.shape == flow.shape[:2]
                assert rec['meters_per_code'] == 1 / 256 and rec['zero_invalid']
                z = depth.astype(np.float64) / 256
                disparity = np.zeros_like(z)
                nonzero = z > 0
                disparity[nonzero] = focal[0] * camera['baseline_m'] / z[nonzero]
                right_u = np.arange(z.shape[1])[None, :] - disparity
                mask = nonzero & (right_u >= 0) & (right_u < z.shape[1] - 1)
                established = protocol['cameras'][modality]['depth_support'][variant][i]
                assert int(nonzero.sum()) == established['nonzero_depth']
                assert int(mask.sum()) == established['right_image_in_bounds']
                maps[variant] = (z, disparity)
                masks[variant] = mask
            common = np.logical_and.reduce(list(masks.values()))
            for variant in VARIANTS:
                z, disparity = maps[variant]
                mask = masks[variant]
                y, x = np.nonzero(mask)
                target = np.column_stack((-disparity[mask], np.zeros(len(x))))
                prediction = flow[mask]
                variance = covariance[mask]
                residual = prediction - target
                shared = common[mask]
                name = f'{modality}-{variant}-{row["stem"]}.npz'
                np.savez_compressed(output / name, x=x.astype(np.int32), y=y.astype(np.int32),
                                    depth_m=z[mask], target_flow=target, flow=prediction,
                                    covariance=variance, residual_pixel=residual,
                                    residual_normalized=residual / focal,
                                    covariance_normalized=variance / focal ** 2,
                                    common_support=shared)
                record = {'modality': modality, 'variant': variant, 'stem': row['stem'],
                          'depth_nonzero': int((z > 0).sum()), 'out_of_bounds_excluded': int((z > 0).sum() - len(x)),
                          'source_file': str(path), 'file': name, 'sha256': sha(output / name),
                          'timing': protocol['pairs'][i],
                          'own_support': metrics(residual, variance, focal),
                          'common_support': metrics(residual[shared], variance[shared], focal)}
                records.append(record)
                print(modality, variant, row['stem'], 'support', len(x), 'common', int(shared.sum()), flush=True)
    assert all(sha(p) == h for p, h in frozen.items())
    result = {'scope': 'conditional projected-reference agreement; no physical accuracy/calibration/winner claim',
              'started_at': started, 'finished_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
              'status': 'PASS', 'numpy': np.__version__, 'opencv': cv2.__version__,
              'frozen_inputs_unchanged': True, 'frozen_input_count': len(frozen),
              'new_model_inference': False, 'physical_cross_modal_error_eligible': False,
              'records': records, 'aggregate': aggregate(records)}
    (output / 'report.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('PASS all120 per-image depth-variant cases; physical timing gate unchanged.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--protocol-audit', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run(a.root, a.manifest, a.protocol_audit, a.output)
