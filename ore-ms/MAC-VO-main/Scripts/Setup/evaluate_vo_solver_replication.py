"""Separate reference-assisted evaluation of pre-refinement original/repair VO.

Uses the existing nominal-body and recorded-SLAM proxy contract, not independent
physical truth. No scale/alignment/threshold fit or production gate is created.
"""
import argparse
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import json
from pathlib import Path
import numpy as np
from Scripts.Setup.vo_solver_replication import read, sha, write, verify
from Scripts.Setup.td3_reference import load_reference
from Scripts.Setup.arbiter_pose_proxy import at, body_delta, transport, losses
from Scripts.Setup.audit_td3_geometry import pose_matrix


def stats(values):
    a = np.asarray(values, dtype=float)
    return {'count': len(a), 'mean': float(a.mean()), 'rmse': float(np.sqrt(np.mean(a*a)))} if len(a) else {'count': 0, 'mean': None, 'rmse': None}


def evaluate(root, output):
    if (output/'evaluation.json').exists(): raise FileExistsError('evaluation already exists')
    protocol = read(output/'protocol.json'); verify(root, protocol, evaluation=True); execution = read(output/'execution.json')
    if execution['status'] != 'PASS_EXECUTION_CONTROLS_ONLY' or execution['protocol_sha256'] != sha(output/'protocol.json') or set(execution['parity']) != {'rgb', 'thermal'} or any(p['status'] != 'PASS' for p in execution['parity'].values()): raise ValueError('execution/preservation gate failed')
    old = root/'results/arbiter-pose-proxy/run-002'; proof = read(old/'validation.json'); declaration = read(old/'protocol.json')
    if proof['status'] != 'PASS' or proof['report_sha256'] != sha(old/'pose-proxy.json') or proof['protocol_sha256'] != sha(old/'protocol.json') or declaration['independent_ground_truth'] or declaration['physical_body_transform_verified']: raise ValueError('expected validated qualified proxy contract')
    labels = {r['pair_ordinal']: r for r in read(old/'pose-proxy.json')['rows'] if r['window'] == 'middle'}
    if set(labels) != set(range(99)): raise ValueError('prior middle proxy population differs')
    extrinsics = {c: next(r['experts'][c]['nominal_body_from_camera'] for r in labels.values() if r['experts'][c] is not None) for c in ('rgb', 'thermal')}
    for row in labels.values():
        for c, expert in row['experts'].items():
            if expert is not None and expert['nominal_body_from_camera'] != extrinsics[c]: raise ValueError('nominal rig transform varies in prior rows')
    source = root/'data/tartandrive3/20251027/night_mac_salon_warehousefig8'; reference, metadata = load_reference(source)
    files = [output/'protocol.json', output/'execution.json', Path(__file__), Path(__file__).with_name('arbiter_pose_proxy.py'),
             Path(__file__).with_name('td3_reference.py'), Path(__file__).with_name('audit_td3_geometry.py')]
    for spec in protocol['runs']:
        files += [p for p in (root/spec['output_relative']).rglob('*') if p.is_file()]
    frozen = {str(p.relative_to(root)): sha(p) for p in sorted(set(files))}
    write(output/'evaluation-protocol.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'execution_protocol_sha256': sha(output/'protocol.json'),
        'input_sha256': frozen, 'prior_evaluation_sha256': protocol['evaluation_input_sha256'], 'reference_metadata': metadata,
        'primary': 'translation_m', 'secondary': 'rotation_deg', 'interval': 'original RGB transition', 'physical_body_transform_verified': False,
        'independent_ground_truth': False, 'fit_performed': False, 'scope': 'all99 first pre-refinement motion writes, terminal stage explicit; nominal-body/reference-assisted recorded-SLAM proxy'})
    rows = []; runs = {}; source_rows = protocol['input_rows']
    for spec in protocol['runs']:
        directory = root/spec['output_relative']; camera, arm = spec['camera'], spec['arm']; report = read(directory/'report.json')
        if report['status'] != 'PASS_EXECUTION_ONLY' or report['input_rows'] != source_rows or report['completed_frames'] != 100: raise ValueError('run input/coverage mismatch')
        summary = read(directory/'arbiter-events/summary.json')
        if summary['status'] != 'COMPLETE' or summary['observer_errors']: raise ValueError('observer errors prohibit evaluation')
        events = [json.loads(line) for line in (directory/'arbiter-events/events.jsonl').read_text().splitlines()]
        adapter = read(directory/'solver-adapter.json') if arm != 'original' else None
        runs[arm+'/'+camera] = {'tracking_diagnostics': report['tracking_diagnostics'], 'tracking': report['tracking'], 'events': len(events),
            'solver_statuses': dict(Counter(r['status'] for r in adapter['records'])) if adapter else None,
            'solver_elapsed_ns': [r['elapsed_ns'] for r in adapter['records']] if adapter else None,
            'cuda_peak_allocated_bytes': adapter['cuda_peak_allocated_bytes'] if adapter else None,
            'adapter_error': adapter['error'] if adapter else None, 'first_motion_stages': {}}
        for pair in range(99):
            common = [source_rows[j]['rgb_left']['time_ns'] for j in (pair, pair+1)]
            own = [source_rows[j][camera+'_left']['time_ns'] for j in (pair, pair+1)]
            if labels[pair]['common_interval_ns'] != common: raise ValueError('prior reference interval differs')
            r0, r1 = at(reference, common); a, b = at(reference, own); ref_delta = np.linalg.inv(r0)@r1
            np.testing.assert_allclose(ref_delta, labels[pair]['reference_delta'], rtol=1e-9, atol=1e-9)
            health = [e for e in events if e['kind'] == 'pair_end' and [e['time0_ns'], e['time1_ns']] == own]
            if len(health) != 1: raise ValueError('unique exact pair health required')
            matches = [e for e in events if e['kind'] == 'motion_result_written' and e['first_write_for_submission'] and e['from_idx'] == pair and e['frame_idx'] == pair+1 and [e['time0_ns'], e['time1_ns']] == own]
            if len(matches) > 1: raise ValueError('ambiguous motion')
            row = {'arm': arm, 'camera': camera, 'pair_ordinal': pair, 'sensor_interval_ns': own, 'common_interval_ns': common,
                'original_need_interp': health[0]['original_need_interp'], 'motion_event_sequence': None, 'motion_stage': None,
                'nominal_body_from_camera': extrinsics[camera],
                'reference_poses': {'common_start': r0.tolist(), 'common_end': r1.tolist(), 'own_start': a.tolist(), 'own_end': b.tolist()},
                'own_body_delta': None, 'common_body_delta': None, 'own_loss': None, 'common_loss': None, 'unavailable_reason': None}
            if not matches or health[0]['original_need_interp'] or health[0]['execution_error']:
                row['unavailable_reason'] = 'unsupported_or_no_first_motion'
            else:
                event = matches[0]
                if event['original_need_interp'] or event['derived_delta_unavailable_reason'] is not None: raise ValueError('invalid motion write')
                e = np.array(row['nominal_body_from_camera']); delta = body_delta(pose_matrix(event['derived_camera_delta_xyzw']), e)
                aligned = transport(delta, r0, a, b, r1)
                row.update(motion_event_sequence=event['sequence'], motion_stage=event['stage'], own_body_delta=delta.tolist(), common_body_delta=aligned.tolist(),
                    own_loss=losses(delta, np.linalg.inv(a)@b), common_loss=losses(aligned, ref_delta))
            rows.append(row)
        runs[arm+'/'+camera]['first_motion_stages'] = dict(Counter(r['motion_stage'] for r in rows if r['arm'] == arm and r['camera'] == camera and r['motion_stage'] is not None))
    matched = {}
    for camera in ('rgb', 'thermal'):
        lookup = {(r['arm'], r['pair_ordinal']): r for r in rows if r['camera'] == camera}
        groups = {}
        for arm in ('original', 'passthrough', 'repair'):
            values = [r for r in rows if r['camera'] == camera and r['arm'] == arm and r['common_loss'] is not None]
            groups[arm] = {'available': len(values), 'missing': 99-len(values), 'pair_ordinals': [r['pair_ordinal'] for r in values],
                'own_loss': {k: stats([r['own_loss'][k] for r in values]) for k in ('translation_m', 'rotation_deg')},
                'common_loss': {k: stats([r['common_loss'][k] for r in values]) for k in ('translation_m', 'rotation_deg')}}
        ids = [i for i in range(99) if all(lookup[arm, i]['common_loss'] is not None for arm in ('original', 'repair'))]
        matched[camera] = {'full': groups, 'original_repair_pair_ordinals': ids, 'original_repair_matched': {arm: {k: stats([lookup[arm, i]['common_loss'][k] for i in ids]) for k in ('translation_m', 'rotation_deg')} for arm in ('original', 'repair')}}
    for path, digest in frozen.items():
        if sha(root/path) != digest: raise ValueError('output/source changed during evaluation')
    verify(root, protocol, evaluation=True)
    write(output/'evaluation.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'evaluation_protocol_sha256': sha(output/'evaluation-protocol.json'),
        'runs': runs, 'matched': matched, 'rows': rows, 'fit_performed': False, 'independent_ground_truth': False,
        'scope': 'qualified reference-assisted pre-refinement interval losses only; no fitted alignment or physical accuracy/generalization/real-time claim'})
    print(json.dumps(matched, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); evaluate(a.root, a.output)
