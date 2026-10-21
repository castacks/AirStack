"""Fixed original/MSA thermal uncertainty diagnostics; no new inference or correction."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .td3_night_geometry import measure, masks, summarize, verify
from .td3_tracking_uncertainty import sha, retained_mask, rank_association, percentiles
from .td3_temporal_context import pair_summary, partial_rank, temporal_checks, CONTEXT, TARGETS


def prepare(root, output):
    replay=root/'results/msa-replay'
    old=json.loads((replay/'prepared-001/plan.json').read_text())
    verify(old['files_sha256'])
    files={};cases=[]
    for c in old['cases']:
        original=Path(c['original'])
        recorded=(original if c['clip']=='day' else root/'results/night-uncertainty/run-001/recorded'/c['window']/'thermal')
        new=replay/'vo-001'/c['case']/'thermal'
        baseline=json.loads((original/'report.json').read_text())
        rows=c['rows'];assert rows==baseline['input_rows']
        for arm,directory in [('original',recorded),('msa',new)]:
            paths=sorted((directory/'uncertainty').glob('pair-*.npz'))
            if len(paths)!=19:raise ValueError('Expected 19 diagnostic pairs')
            report=json.loads((directory/'report.json').read_text())
            if report['input_rows']!=rows:raise ValueError('Recorder rows changed')
            proxy=replay/f'{c["case"]}-{arm}-proxy-reference.npz'
            cases.append(dict(case=c['case'],clip=c['clip'],window=c['window'],arm=arm,
                directory=str(directory),proxy=str(proxy),pairs=list(map(str,paths)),rows=rows))
            for p in [proxy,directory/'report.json',directory/'effective-config.json',*paths]:files[str(p)]=sha(p)
    helpers=['td3_night_geometry.py','td3_reprojection.py','td3_reference.py','td3_tracking_uncertainty.py','td3_temporal_context.py']
    paths=[Path(__file__),replay/'prepared-001/plan.json',replay/'analysis.json',replay/'validation.json',root/'results/night-geometry/run-001/analysis.json']
    paths.extend(Path(__file__).parent/p for p in helpers)
    paths.extend(Path(__file__).parents[1]/'UnitTest'/p for p in ['test_td3_night_geometry.py','test_td3_temporal_context.py','test_td3_reprojection.py','test_td3_tracking_uncertainty.py'])
    files.update({str(p):sha(p) for p in paths});verify(files)
    output.mkdir(parents=True,exist_ok=False)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='a729420c9',cases=cases,files_sha256=files,
        protocol=dict(primary='All fixed-grid source pixels; window medians of within-pair summaries, no pooling',
            populations=['grid','retained'],sensitivities=['inbound','inbound_both_depths_max50m','image thirds','source depth bins'],
            temporal_context=CONTEXT,targets=TARGETS,pairs_per_case=19,scope='Recorded SLAM proxy, original nominal body anchor; no physical calibration/probability/causal claim'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def write_csv(path, records):
    keys=list(dict.fromkeys(k for r in records for k in r))
    with path.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(records)


def run(output):
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256'])
    groups_records=[];window_records=[];temporal_pairs=[];temporal_cases=[];retention=[]
    for case in plan['cases']:
        with np.load(case['proxy']) as proxy:
            cameras=proxy['T_world_camera_ned'];times=proxy['time_ns'].reshape(-1)
        if not np.array_equal(times,[row['thermal_left']['time_ns'] for row in case['rows']]):raise ValueError('Proxy timestamp mismatch')
        local=[];local_temporal=[]
        dest=output/'points'/case['case']/case['arm'];dest.mkdir(parents=True)
        for i,path in enumerate(case['pairs']):
            relative=np.linalg.inv(cameras[i+1])@cameras[i]
            motion=dict(translation_m=float(np.linalg.norm(relative[:3,3])),rotation_deg=float(np.degrees(Rotation.from_matrix(relative[:3,:3]).magnitude())))
            with np.load(path) as pair:
                if int(pair['time0_ns'])!=int(times[i]):raise ValueError('Pair timestamp mismatch')
                keep=retained_mask(pair['selected_inbound'],pair['outlier_keep_inbound'])
                covariance=pair['grid_raw_flow_cov'][:,:2]
                retention.append(dict(case=case['case'],arm=case['arm'],pair=i,grid=len(covariance),selected=len(keep),
                    inbound=int(pair['selected_inbound'].sum()),retained=int(keep.sum()),
                    variance_components_below_floor_fraction=float((covariance<.0625).mean())))
                for population,prefix in [('grid','grid'),('retained','selected')]:
                    data=measure(pair,prefix,relative)
                    if population=='retained':data={k:v[keep] for k,v in data.items()}
                    masks_here=masks(data,int(pair['height_width'][0]))
                    np.savez_compressed(dest/f'{population}-{i:03d}.npz',**data,**{'mask_'+k:v for k,v in masks_here.items()},
                        K=pair['K'],relative=relative,time0_ns=times[i],time1_ns=times[i+1])
                    for group,mask in masks_here.items():
                        row=dict(case=case['case'],arm=case['arm'],pair=i,population=population,group=group,**motion,**summarize(data,mask))
                        groups_records.append(row);local.append(row)
                    if population=='grid':
                        for group in ['all','inbound']:
                            row=dict(case=case['case'],arm=case['arm'],pair=i,population=group,**motion,**pair_summary(data,masks_here[group]))
                            temporal_pairs.append(row);local_temporal.append(row)
        for population in ['grid','retained']:
            for group in masks_here:
                rows=[r for r in local if r['population']==population and r['group']==group]
                result=dict(case=case['case'],clip=case['clip'],window=case['window'],arm=case['arm'],population=population,group=group,
                    pairs=len(rows),total_points=sum(r['count'] for r in rows))
                for metric in ['trace',*TARGETS]:
                    values=[r[metric]['p10_p50_p90'][1] for r in rows if r[metric]['finite_count']]
                    result[metric+'_pair_median']=float(np.median(values)) if values else None
                    result[metric+'_finite_points']=sum(r[metric]['finite_count'] for r in rows)
                for metric in TARGETS:
                    values=[r[metric+'_rho'] for r in rows if r[metric+'_rho'] is not None]
                    result[metric+'_rho_median']=float(np.median(values)) if values else None
                    result[metric+'_rho_defined_pairs']=len(values)
                window_records.append(result)
        temporal_dest=output/'temporal';temporal_dest.mkdir(exist_ok=True)
        for group in ['all','inbound']:
            rows=[r for r in local_temporal if r['population']==group]
            context=np.array([[r[k] for k in CONTEXT] for r in rows],float)
            x=np.array([r['trace'] for r in rows],float);index=np.arange(19)
            saved={k:np.array([r[k] for r in rows]) for k in ['pair','trace',*CONTEXT,*TARGETS]}
            for target in TARGETS:
                y=np.array([r[target] for r in rows],float)
                adjusted,residuals,valid=partial_rank(x,y,context)
                timed,time_residuals,time_valid=partial_rank(x,y,np.column_stack([context,index]))
                temporal_cases.append(dict(case=case['case'],arm=case['arm'],population=group,target=target,pairs=19,
                    raw_rho=rank_association(x,y),flow_rho=rank_association(saved['flow_px'],y),
                    **{'adjusted_'+k:v for k,v in adjusted.items()},**{'time_adjusted_'+k:v for k,v in timed.items()},**temporal_checks(x,y,index)))
                saved.update({target+'_rank_residuals':residuals,target+'_valid':valid,target+'_time_rank_residuals':time_residuals,target+'_time_valid':time_valid})
            np.savez_compressed(temporal_dest/f'{case["case"]}-{case["arm"]}-{group}.npz',**saved)
        print(case['case'],case['arm'],'19 pairs',flush=True)
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],cases=window_records,
        pairs=groups_records,temporal_pairs=temporal_pairs,temporal_cases=temporal_cases,retention=retention,
        verified_hashes=verify(plan['files_sha256']),source_pair_files=228,
        epipolar_degenerate_pairs=sum(r['translation_m']<=1e-6 for r in groups_records if r['population']=='grid' and r['group']=='all'),
        scope='Same six windows and recorded SLAM proxy; no new inference, sensor correction, pooled significance or independent accuracy validation')
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    for name,rows in [('cases',window_records),('pairs',groups_records),('temporal-cases',temporal_cases),('temporal-pairs',temporal_pairs),('retention',retention)]:write_csv(output/f'{name}.csv',rows)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('--root',type=Path,default=Path('/workspace'));parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.root,args.output)
    else:run(args.output)
