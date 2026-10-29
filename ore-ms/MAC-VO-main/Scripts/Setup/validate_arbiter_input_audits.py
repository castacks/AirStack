"""Independent bilinear/NCC and temporal feature checks; no model training."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics
from zoneinfo import ZoneInfo

import cv2
import numpy as np


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def quantile(values, q):
    v = sorted(float(x) for x in values)
    at = (len(v)-1)*q
    lo, hi = math.floor(at), math.ceil(at)
    return v[lo]+(at-lo)*(v[hi]-v[lo])


def manual_patches(image, x, y):
    dy, dx = np.mgrid[-4:5, -4:5]
    xx, yy = x[:, None]+dx.ravel(), y[:, None]+dy.ravel()
    h, w = image.shape
    valid = (x >= 4) & (x <= w-5) & (y >= 4) & (y <= h-5)
    xf, yf = np.floor(xx).astype(int), np.floor(yy).astype(int)
    wx, wy = xx-xf, yy-yf
    get = lambda a, b: image[np.clip(b, 0, h-1), np.clip(a, 0, w-1)]
    values = ((1-wx)*(1-wy)*get(xf, yf) + wx*(1-wy)*get(xf+1, yf)
              + (1-wx)*wy*get(xf, yf+1) + wx*wy*get(xf+1, yf+1))
    values[~valid] = np.nan
    return values, valid


def manual_ncc(a, b):
    result = []
    for x, y in zip(a, b):
        if not np.isfinite(x).all() or not np.isfinite(y).all():
            result.append(np.nan)
            continue
        x, y = x-statistics.mean(x), y-statistics.mean(y)
        den = math.sqrt(math.fsum(float(t*t) for t in x)*math.fsum(float(t*t) for t in y))
        result.append(math.fsum(float(u*v) for u, v in zip(x, y))/den if den else np.nan)
    return np.array(result)


def validate(root, cats, temporal, output):
    checks = 0
    def close(a, b):
        nonlocal checks
        assert np.allclose(a, b, rtol=1e-10, atol=1e-10, equal_nan=True), (a, b)
        checks += int(np.size(a))
    p = json.loads((cats/'protocol.json').read_text())
    r = json.loads((cats/'report.json').read_text())
    for path, h in p['input_sha256'].items():
        assert sha(path) == h
    # Locate by declaration's scene root; do not infer hypotheses from source output.
    scene = root/'results/cats-intake/outdoor-v2/sample'/p['scene']
    for camera in ('color', 'thermal', 'cross'):
        d = np.loadtxt(scene/f'gt_disparity/{camera}/gt_disparity.txt', delimiter=',')
        mask = cv2.imread(str(scene/f'rectified/{camera}/mask.png'), 0)
        names = ['left_color_default.png', 'right_color_default.png'] if camera == 'color' else ['left_thermal_default.png', 'right_thermal_default.png'] if camera == 'thermal' else ['left_thermal_default.png', 'right_color_default.png']
        raw = [cv2.imread(str(scene/f'rectified/{camera}'/n), cv2.IMREAD_UNCHANGED) for n in names]
        for value in (0, 255):
            out = r[camera]['mask_reference_counts'][str(value)]
            close(int(np.sum(mask == value)), out['pixels'])
            close(int(np.sum((mask == value) & np.isfinite(d))), out['finite_reference'])
            for j, key in [(0, 'left_allzero'), (1, 'right_allzero_same_coordinate')]:
                black = (raw[j] == 0).all(axis=2) if raw[j].ndim == 3 else raw[j] == 0
                close(int(np.sum((mask == value) & black)), out[key])
        if camera == 'cross':
            assert r[camera]['ncc_status'] == 'NOT_APPLIED_CROSS_MODALITY'
            continue
        images = [cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(float) if a.ndim == 3 else a.astype(float) for a in raw]
        with np.load(cats/f'{camera}-patch-diagnostics.npz') as f:
            saved = {k: f[k] for k in f.files}
        ids = np.flatnonzero(np.isfinite(d));ids=ids[np.linspace(0,len(ids)-1,min(1024,len(ids)),dtype=int)]
        y, x = np.unravel_index(ids, d.shape);delta=d[y,x]
        close(x,saved['x']);close(y,saved['y']);close(delta,saved['stored_disparity']);close(mask[y,x],saved['mask_value'])
        values = {}
        for hypothesis in ['left_plus','left_minus','right_plus','right_minus','left_zero']:
            anchor,direction=hypothesis.split('_');j=int(anchor=='right');shift=delta if direction=='plus' else -delta if direction=='minus' else np.zeros(len(x))
            a,insidea=manual_patches(images[j],x,y);b,insideb=manual_patches(images[1-j],x+shift,y)
            ncc=manual_ncc(a,b);values[hypothesis]=ncc;close(ncc,saved[hypothesis]);close(insidea&insideb,saved[hypothesis+'_inbounds'])
            out=r[camera]['hypotheses'][hypothesis];close(int(np.sum(~(insidea&insideb))),out['out_of_patch_bounds'])
            groups={'all':np.ones(len(x),bool)}
            for maskvalue in [0,255]:
                for sign,sel in [('all',np.ones(len(x),bool)),('negative',delta<0),('zero',delta==0),('positive',delta>0)]:groups[f'mask_{maskvalue}_{sign}']=(mask[y,x]==maskvalue)&sel
            for group,sel in groups.items():
                v=ncc[sel];finite=v[np.isfinite(v)];s=out[group]
                close([len(v),len(finite),len(v)-len(finite)],[s['n'],s['valid_ncc'],s['unavailable_ncc']])
                if len(finite):close(statistics.median(finite),s['median_ncc']);close([quantile(finite,.25),quantile(finite,.75)],s['q25_q75_ncc'])
                else:assert s['median_ncc'] is None and s['q25_q75_ncc'] is None
            for group,sel in [('all',np.ones(len(x),bool)),('mask_255',mask[y,x]==255)]:
                if 'left_minus' in values and 'left_plus' in values:
                    a,b=values['left_minus'],values['left_plus'];common=sel&np.isfinite(a)&np.isfinite(b);s=r[camera]['paired_left_minus_vs_plus'][group]
                    close([int(common.sum()),int(np.sum(a[common]>b[common])),int(np.sum(a[common]==b[common])),int(np.sum(a[common]<b[common]))],[s['available_pairs'],s['minus_better'],s['equal'],s['plus_better']])
                    close(statistics.median(a[common]-b[common]),s['median_paired_ncc_difference_minus_plus'])
    tp=json.loads((temporal/'protocol.json').read_text());tr=json.loads((temporal/'report.json').read_text())
    for rel,h in tp['input_sha256'].items():assert sha(root/rel)==h
    for rel,h in tp['source_sha256'].items():assert sha(root/'MAC-VO-main'/rel)==h
    for case,out in tr['cases'].items():
        directory=root/'results/gnss-and-tracking/reporting-001'/case/out['modality'];source=json.loads((directory/'report.json').read_text())
        with np.load(directory/'tracking-flags.npz') as f:before,after=f['before_refinement'],f['after_refinement']
        for i,row in enumerate(out['records']):
            with np.load(directory/'uncertainty'/f'pair-{i:03d}.npz') as f:a={k:f[k] for k in f.files}
            start,end=[source['input_rows'][j][out['modality']+'_left']['time_ns'] for j in [i,i+1]]
            assert row['time0_ns']==start and row['time1_ns']==end and row['duration_ns']==end-start
            close([start,end,end-start],[row['time0_ns'],row['time1_ns'],row['duration_ns']])
            assert int(a['time0_ns'])==start
            inbound=np.flatnonzero(a['selected_inbound']);kept=inbound[a['outlier_keep_inbound']]
            uv=a['selected_uv0'][kept];h,w=a['height_width'];cells=set((int(y*3//h),int(x*3//w)) for x,y in uv)
            close([len(a['selected_uv0']),len(inbound),len(kept),len(cells)],[row[k] for k in ['candidate_count','inbound_count','retained_count','occupied_3x3_cells']])
            close(bool(before[i+1]),row['original_need_interp'])
            cov=a['grid_raw_flow_cov'];finite=np.isfinite(cov).all(axis=1);valid=finite&(cov[:,0]>0)&(cov[:,1]>0);v=[float(x+y) for x,y in cov[valid,:2]]
            close([len(cov),int(finite.sum()),int(valid.sum())],[row[k] for k in ['grid_count','grid_covariance_finite_count','grid_positive_uu_vv_count']])
            if v:close([quantile(v,q) for q in [.1,.5,.9]],row['grid_trace_q10_q50_q90_px2'])
            else:assert row['grid_trace_q10_q50_q90_px2'] is None
        for key,summary in out['summary'].items():
            vals=[row[key] for row in out['records']];close([min(vals),statistics.median(vals),max(vals)],[summary[k] for k in ['min','median','max']])
        close(int(before[1:].sum()),out['original_unsupported']);close(int(after.sum()),out['post_refinement_flags'])
    rgb,thermalrows=[tr['cases'][n]['records'] for n in ['long-middle-rgb','long-middle-thermal']]
    close(sum((a['time0_ns'],a['time1_ns'])==(b['time0_ns'],b['time1_ns']) for a,b in zip(rgb,thermalrows)),tr['alignment']['exact_same_intervals'])
    for key in ['time0_ns','time1_ns']:
        delta=[a[key]-b[key] for a,b in zip(rgb,thermalrows)];s=tr['alignment'][key+'_rgb_minus_thermal_ns'];close([min(delta),statistics.median(delta),max(delta)],[s[k] for k in ['min','median','max']])
    # Verify patch implementation on synthetic signed shifts, zero shift, constant and boundary cases.
    from .cats_convention_audit import patches,ncc
    rng=np.random.default_rng(0);left=rng.normal(size=(40,70));x=np.arange(15,50,dtype=float);y=np.full(len(x),20.)
    for shift in [-6,0,6]:
        right=np.roll(left,-shift,axis=1);a,_=patches(left,x,y);b,_=patches(right,x-shift,y);close(ncc(a,b),np.ones(len(x)))
    a,inside=patches(left,np.array([0.,4.,65.,69.]),np.full(4,20.));close(inside,[False,True,True,False])
    assert np.isnan(ncc(np.ones((1,81)),np.ones((1,81)))).all()
    a,_=patches(left,x+.123,y+.456);b,_=manual_patches(left,x+.123,y+.456);close(a,b)
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'checks':checks,
        'script_sha256':sha(__file__),'cats_report_sha256':sha(cats/'report.json'),'temporal_report_sha256':sha(temporal/'report.json'),
        'synthetic_cases':['negative/zero/positive displacement','patch boundaries','constant patches unavailable','fractional bilinear agreement'],
        'scope':'independent convention diagnostics/feature arithmetic and preservation, not authoritative disparity semantics or VO/gate performance'}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--cats',type=Path,required=True);p.add_argument('--temporal',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    validate(a.root,a.cats,a.temporal,a.output)
