"""Compare fresh v2/v3 outputs and independently audit the observed event stream."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def validate(baseline, observed, output):
    import torch
    import pypose as pp
    old, new = [json.loads((p/'report.json').read_text()) for p in [baseline, observed]]
    for key, value in old.items():
        if key == 'started_at_utc':
            continue
        assert new[key] == value, key
    assert old['status'] == new['status'] == 'PASS_EXECUTION_ONLY'
    assert (baseline/'effective-config.json').read_bytes() == (observed/'effective-config.json').read_bytes()
    paths = sorted(str(p.relative_to(baseline)) for p in baseline.rglob('*.npz'))
    assert paths == sorted(str(p.relative_to(observed)) for p in observed.rglob('*.npz'))
    array_count = 0
    for path in paths:
        with np.load(baseline/path,allow_pickle=False) as a, np.load(observed/path,allow_pickle=False) as b:
            assert a.files == b.files, path
            for key in a.files:
                assert a[key].dtype == b[key].dtype and a[key].shape == b[key].shape, (path,key)
                assert a[key].tobytes() == b[key].tobytes(), (path,key)
                array_count += 1
    summary = json.loads((observed/'arbiter-events/summary.json').read_text())
    assert summary['status'] == 'COMPLETE' and not summary['observer_errors']
    assert new['arbiter_event_recording'] == summary
    events = [json.loads(line) for line in (observed/'arbiter-events/events.jsonl').read_text().splitlines()]
    assert [e['sequence'] for e in events] == list(range(len(events)))
    clocks = [e['observation_monotonic_ns'] for e in events]
    assert all(type(t) is int for t in clocks) and all(a <= b for a,b in zip(clocks,clocks[1:]))
    frames = old['completed_frames'];modality=old['modality']
    rows = old['input_rows'];times=[int(r[modality+'_left']['time_ns']) for r in rows]
    begins = [e for e in events if e['kind']=='pair_begin']
    ends = [e for e in events if e['kind']=='pair_end']
    assert len(begins) == len(ends) == frames-1
    for i, (begin,end) in enumerate(zip(begins,ends)):
        assert begin['interval'] == {'time0_ns':times[i],'time1_ns':times[i+1]}
        assert end['time0_ns']==times[i] and end['time1_ns']==times[i+1] and not end['execution_error']
        with np.load(observed/'uncertainty'/f'pair-{i:03d}.npz') as raw:
            assert end['selected_count'] == len(raw['selected_uv0'])
            assert end['inbound_count'] == int(raw['selected_inbound'].sum())
            assert end['retained_count'] == int(raw['outlier_keep_inbound'].sum())
            points=raw['selected_uv0'][raw['selected_inbound']][raw['outlier_keep_inbound']]
            h,w=raw['height_width'];cells=set((int(y*3//h),int(x*3//w)) for x,y in points)
            assert end['occupied_3x3_cells'] == len(cells)
            covariance=raw['grid_raw_flow_cov'];valid=np.isfinite(covariance).all(axis=1)&(covariance[:,0]>0)&(covariance[:,1]>0)
            assert end['grid_count'] == len(covariance) and end['grid_positive_uu_vv_count']==int(valid.sum())
            if valid.any():
                expected=np.quantile(covariance[valid,:2].sum(axis=1),[.1,.5,.9]).astype(float)
                np.testing.assert_allclose(end['grid_trace_q10_q50_q90_px2'],expected,rtol=2e-7,atol=1e-8)
            else:
                assert end['grid_trace_q10_q50_q90_px2'] is None
    with np.load(observed/'tracking-flags.npz') as flags:
        assert [e['original_need_interp'] for e in ends] == flags['before_refinement'][1:].tolist()
    submissions={e['submission_id']:e for e in events if e['kind']=='optimization_submitted'}
    returned={e['job']['submission_id']:e for e in events if e['kind']=='optimization_submit_returned'}
    assert len(submissions)==summary['optimization_submissions']
    writes=[e for e in events if e['kind']=='motion_result_written']
    firsts=[e for e in writes if e['first_write_for_submission']]
    assert len(firsts)==len(submissions)
    refine_begin=next(e for e in events if e['kind']=='refinement_begin')
    assert all(e['sequence']<refine_begin['sequence'] for e in writes)
    for e in writes:
        submitted=submissions[e['submission_id']];completion=returned[e['submission_id']]
        assert submitted['sequence'] < completion['sequence'] < e['sequence']
        assert e['time0_ns']==submitted['time0_ns'] and e['time1_ns']==submitted['time1_ns']
        assert e['optimizer_convergence'] is None and e['pose_covariance'] is None
        assert e['derived_delta_unavailable_reason'] is None
        parent=np.array(e['parent_pose_camera_map_xyzw'],float);target=np.array(e['committed_pose_camera_map_xyzw'],float)
        parent[3:]/=np.linalg.norm(parent[3:]);target[3:]/=np.linalg.norm(target[3:])
        expected=(pp.SE3(torch.tensor(parent)).Inv() @ pp.SE3(torch.tensor(target))).tensor().numpy()
        np.testing.assert_allclose(e['derived_camera_delta_xyzw'],expected,rtol=1e-10,atol=1e-10)
    # Sparse tracking may repeatedly flush an older solved graph interval.
    assert all(0 <= e['from_idx'] < e['frame_idx'] < frames for e in firsts)
    assert len({e['submission_id'] for e in firsts}) == len(firsts)
    assert all(e['write_ordinal_for_submission'] == 1 for e in firsts)
    assert all((e['write_ordinal_for_submission'] == 1) == e['first_write_for_submission'] for e in writes)
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),
        'script_sha256':sha(__file__),'npz_files':len(paths),'bitwise_equal_arrays':array_count,
        'frames':frames,'pairs':frames-1,'event_count':len(events),'motion_first_writes':len(firsts),
        'repeated_motion_writes':len(writes)-len(firsts),'summary_sha256':sha(observed/'arbiter-events/summary.json'),
        'events_sha256':sha(observed/'arbiter-events/events.jsonl'),
        'original_unsupported_pairs':sum(e['original_need_interp'] for e in ends),
        'scope':'same-runtime standalone v2 versus shared sparse-support replay; not historical-runtime parity or pose accuracy'}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--observed',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();validate(a.baseline,a.observed,a.output)
