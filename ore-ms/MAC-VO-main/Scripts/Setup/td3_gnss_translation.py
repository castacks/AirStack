"""Fixed-window position-only GNSS cross-checks; no axis/pose alignment or GT claim."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import numpy as np
import yaml
from .td3_reference import read_ns,load_reference,camera_reference,matrices
from .td3_tracking_uncertainty import sha,percentiles
from .td3_night_geometry import verify

OFFSETS_NS=(-100000000,0,100000000)


def positions_at(source_ns,positions,query_ns):
    source_ns=np.asarray(source_ns);query_ns=np.asarray(query_ns);positions=np.asarray(positions,dtype=np.float64)
    if source_ns.dtype.kind not in 'iu' or query_ns.dtype.kind not in 'iu':raise ValueError('Integer nanosecond timestamps required')
    if source_ns.ndim!=1 or query_ns.ndim!=1 or len(source_ns)<2 or positions.shape!=(len(source_ns),3):raise ValueError('Aligned XYZ position/time arrays required')
    if not np.isfinite(positions).all() or np.any(np.diff(source_ns)<=0):raise ValueError('Finite positions and strictly increasing source times required')
    if np.any(query_ns<source_ns[0]) or np.any(query_ns>source_ns[-1]):raise ValueError('GNSS extrapolation prohibited')
    seconds=(source_ns-source_ns[0])/1e9;query=(query_ns-source_ns[0])/1e9
    local=positions-positions[0]
    return np.column_stack([np.interp(query,seconds,local[:,axis]) for axis in range(3)])


def magnitudes(positions):
    positions=np.asarray(positions,dtype=np.float64)
    if positions.ndim!=2 or positions.shape[1]!=3 or len(positions)<2 or not np.isfinite(positions).all():raise ValueError('Finite XYZ path with at least two rows required')
    local=positions-positions[0]
    return dict(xy=np.linalg.norm(local[:,:2],axis=1),xyz=np.linalg.norm(local,axis=1),vertical=np.abs(local[:,2]))


def prepare(root,output):
    output.mkdir(parents=True,exist_ok=False);cases=[];files={}
    for group,prefix in [('new-night-validation','short'),('long-night-validation','long')]:
        path=root/'results'/group/'prepared-001/plan.json';p=json.loads(path.read_text());files[str(path)]=sha(path)
        for case in p['cases']:
            for modality in ['rgb','thermal']:
                directory=root/'results'/group/'vo-001'/case['case']/modality
                cases.append(dict(case=prefix+'-'+case['case']+'-'+modality,modality=modality,directory=str(directory),
                    rows=case['rows'],dataset=case['dataset'],reference_dataset=case['reference_dataset'],
                    gnss=str(Path(case['reference_dataset'])/'sensors/novatel_gps_odom')))
    path=root/'results/msa-replay/prepared-001/plan.json';p=json.loads(path.read_text());files[str(path)]=sha(path)
    for name in ['night-early','night-middle','night-late']:
        case=next(c for c in p['cases'] if c['case']==name)
        cases.append(dict(case='canonical-'+name+'-thermal',modality='thermal',directory=str(root/'results/msa-replay/vo-001'/name/'thermal'),
            rows=case['rows'],dataset=case['dataset'],reference_dataset=case['reference_dataset'],gnss=str(root/'results/reference-timing/inputs/night/sensors/novatel_gps_odom')))
    for case in cases:
        directory=Path(case['directory']);gnss=Path(case['gnss']);source=Path(case['reference_dataset'])/'super_odometry/odometry'
        paths=[directory/'report.json',*map(lambda name:gnss/name,['info.yaml','interp_data.txt','interp_timestamps.txt']),
            *map(lambda name:source/name,['info.yaml','interp_data.txt','interp_timestamps.txt'])]
        if (directory/'trajectory.npz').exists():paths.append(directory/'trajectory.npz')
        paths += list((directory/'uncertainty').glob('pair-*.npz'))
        names=['thermal_left_processed_camera_info','multisense_left_rect_color_camera_info']
        paths += [Path(case['dataset'])/'sensors'/name/'data.yaml' for name in names if (Path(case['dataset'])/'sensors'/name/'data.yaml').exists()]
        for path in paths:files[str(path)]=sha(path)
    paths=[Path(__file__),Path(__file__).parents[1]/'UnitTest/test_td3_gnss_translation.py',root/'data/calibration/yamaha.yaml',
        Path(__file__).with_name('td3_reference.py'),Path(__file__).with_name('td3_tracking_uncertainty.py'),
        root/'docs/research/source-replies-2026-10-22.md']
    for path in paths:files[str(path)]=sha(path)
    verify(files);plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='b203bb34d',cases=cases,files_sha256=files,
        offsets_ns=list(OFFSETS_NS),protocol='Float64 local position interpolation, original image timestamps; separately report exported gps_frame GNSS XY/3D, SLAM body/camera3D, VO3D. No alignment/correction/GT or independent-error claim.')
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256']);records=[];source_stats={}
    for case in plan['cases']:
        directory=Path(case['directory']);report=json.loads((directory/'report.json').read_text());assert report['input_rows']==case['rows']
        times=np.array([r[case['modality']+'_left']['time_ns'] for r in case['rows']],np.int64)
        gnss=Path(case['gnss']);data=np.loadtxt(gnss/'interp_data.txt',dtype=np.float64);source_ns=read_ns(gnss/'interp_timestamps.txt')
        if data.shape!=(len(source_ns),13):raise ValueError('Unexpected GNSS export schema')
        metadata=yaml.safe_load((gnss/'info.yaml').read_text())
        if metadata!={'frame_id':'map','child_frame_id':'gps_frame'}:raise ValueError('Unexpected GNSS frames')
        # Validate the entire source once, even when only window endpoints are queried.
        positions_at(source_ns,data[:,:3],source_ns[:2])
        source_stats[str(gnss)]=dict(rows=len(source_ns),frame_metadata=metadata,source_gap_ms=percentiles(np.diff(source_ns)/1e6),
            source_max_gap_ms=float(np.diff(source_ns).max()/1e6),float32_coordinate_spacing_m=np.spacing(np.abs(data[:,:3]).max(axis=0).astype(np.float32)).tolist(),
            covariance_or_receiver_quality_in_schema=False)
        reference,_=load_reference(case['reference_dataset']);body=reference.at(times);camera=camera_reference(reference,times,case['dataset'],Path('/workspace/data/calibration/yamaha.yaml'),case['modality'])
        body_mag=magnitudes(body[:,:3,3])['xyz'];camera_mag=magnitudes(camera[:,:3,3])['xyz'];vo_mag=None
        if (directory/'trajectory.npz').exists():
            with np.load(directory/'trajectory.npz') as p:
                assert np.array_equal(p['time_ns'].reshape(-1),times);vo=matrices(p['poses_camera_ned']);vo_mag=magnitudes(vo[:,:3,3])['xyz']
        counts=[]
        for path in sorted((directory/'uncertainty').glob('pair-*.npz')):
            with np.load(path) as p:counts.append(int(p['outlier_keep_inbound'].sum()))
        if len(counts)!=len(times)-1:raise ValueError('Incomplete saved raw tracking diagnostics')
        unsupported=sum(n<10 for n in counts)
        for offset in plan['offsets_ns']:
            query=times+offset;row=dict(case=case['case'],modality=case['modality'],frames=len(times),duration_s=float((times[-1]-times[0])/1e9),offset_ns=offset,
                runner_status=report['status'],pre_refinement_unsupported_pairs=unsupported,transition_pairs=len(counts),
                stored_interpolation_flags=report.get('tracking',{}).get('interpolated_frames'),vo_net_3d_m=None if vo_mag is None else float(vo_mag[-1]),
                slam_body_net_3d_m=float(body_mag[-1]),nominal_camera_slam_net_3d_m=float(camera_mag[-1]))
            if np.any(query<source_ns[0]) or np.any(query>source_ns[-1]):
                row['status']='OUTSIDE_GNSS_SUPPORT';records.append(row);continue
            local=positions_at(source_ns,data[:,:3],query);gnss_mag=magnitudes(local)
            selected=(source_ns>=query[0])&(source_ns<=query[-1]);inside=data[selected]
            right=np.searchsorted(source_ns,query,side='left').clip(0,len(source_ns)-1);left=np.maximum(right-1,0)
            nearest=np.minimum(np.abs(source_ns[right]-query),np.abs(source_ns[left]-query))
            row.update(status='SUPPORTED',gnss_net_xy_m=float(gnss_mag['xy'][-1]),gnss_net_3d_m=float(gnss_mag['xyz'][-1]),gnss_endpoint_abs_vertical_m=float(gnss_mag['vertical'][-1]),
                vo_minus_gnss_net_m=None if vo_mag is None else float(vo_mag[-1]-gnss_mag['xyz'][-1]),
                gnss_source_rows_in_window=int(selected.sum()),nearest_sample_distance_ms=percentiles(nearest/1e6),
                source_step_3d_m=percentiles(np.linalg.norm(np.diff(inside[:,:3],axis=0),axis=1)),
                supplied_linear_speed_mps=percentiles(np.linalg.norm(inside[:,7:10],axis=1)))
            dest=output/'points'/case['case'];dest.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(dest/f'offset-{offset}.npz',time_ns=times,gnss_query_ns=query,gnss_local_positions=local,
                gnss_xy=gnss_mag['xy'],gnss_xyz=gnss_mag['xyz'],gnss_abs_vertical=gnss_mag['vertical'],slam_body_xyz=body_mag,
                nominal_camera_slam_xyz=camera_mag,**({} if vo_mag is None else {'vo_xyz':vo_mag}))
            records.append(row)
        print(case['case'],'3 fixed offsets',flush=True)
    (output/'analysis.json').write_text(json.dumps(dict(created_at_utc=datetime.now(timezone.utc).isoformat(),cases=records,sources=source_stats,
        verified_hashes=verify(plan['files_sha256']),scope='Separately generated GNSS positional magnitudes; different physical origins; quality/RTK/timing/axes remain unresolved. Not pose error or calibrated GT.'),indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run']);parser.add_argument('--root',type=Path,default=Path('/workspace'));parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.action=='prepare':prepare(args.root,args.output)
    else:run(args.output)
