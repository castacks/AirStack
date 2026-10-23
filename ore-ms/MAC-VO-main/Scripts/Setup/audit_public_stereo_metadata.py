"""Inspect public STheReO camera/reference metadata without inference or calibration fitting."""
import argparse,ast,hashlib,json,re
from decimal import Decimal
from pathlib import Path
import numpy as np


def camera(path):
    text=Path(path).read_text();result={key:int(re.search(r'^'+key+r':\s*(\d+)',text,re.M)[1]) for key in ['image_width','image_height']}
    for name in ['camera_matrix','rectification_matrix','projection_matrix']:
        section=text.split(name+':',1)[1];rows=int(re.search(r'rows:\s*(\d+)',section)[1]);cols=int(re.search(r'cols:\s*(\d+)',section)[1]);values=ast.literal_eval(re.search(r'data:\s*(\[[^\]]+\])',section,re.S)[1]);a=np.asarray(values,float).reshape(rows,cols)
        if not np.isfinite(a).all():raise ValueError('Nonfinite camera matrix')
        result[name]=a.tolist()
    return result


def poses(path):
    fields=[line.strip().split(',') for line in Path(path).read_text().splitlines() if line.strip()]
    if not all(len(r)==7 for r in fields):raise ValueError('Unexpected pose schema')
    times=[int(Decimal(r[0])*1000000000) for r in fields];values=np.asarray([[float(x) for x in r[1:]] for r in fields],float)
    if not np.isfinite(values).all() or not all(b>a for a,b in zip(times,times[1:])):raise ValueError('Invalid pose series')
    gaps=np.diff(times)/1e9
    return dict(rows=len(times),time0_ns=times[0],time1_ns=times[-1],duration_s=(times[-1]-times[0])/1e9,gap_s_p50_p90_p99_max=[*np.percentile(gaps,[50,90,99]).tolist(),float(gaps.max())],gaps_above_0_25s=int((gaps>.25).sum()),positions_min=values[:,:3].min(0).tolist(),positions_max=values[:,:3].max(0).tolist(),angle_columns_min=values[:,3:].min(0).tolist(),angle_columns_max=values[:,3:].max(0).tolist()),times,values


def audit(directory):
    directory=Path(directory);base=directory/'extracted';calib=base/'calibration';cameras={name:camera(calib/(name+'.yaml')) for name in ['left','right','thermal_14bit_left','thermal_14bit_right']};baseline={}
    for label,left,right,extrinsic in [('rgb','left','right','rgb_right_2_rgb_left'),('thermal','thermal_14bit_left','thermal_14bit_right','thermal_right_2_thermal_left')]:
        p=np.array(cameras[right]['projection_matrix']);p0=np.array(cameras[left]['projection_matrix']);assert p.shape==p0.shape==(3,4)
        text=(calib/'extrinsic'/extrinsic).read_text();v=np.array([float(x) for x in re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?',text)]).reshape(4,4);projection=abs(p[0,3]/p[0,0]-p0[0,3]/p0[0,0]);norm=float(np.linalg.norm(v[:3,3]));baseline[label]=dict(projection_baseline_m=float(projection),extrinsic_translation_norm_m=norm,difference_m=float(projection-norm),relative_difference=float(projection/norm-1),raw_extrinsic=v.tolist(),raw_rotation_orthogonality_error=float(np.linalg.norm(v[:3,:3].T@v[:3,:3]-np.eye(3))))
    summaries={};series={}
    for name in ['local_pose','global_pose']:
        summary,times,values=poses(base/'pose'/(name+'.csv'));summaries[name]=summary;series[name]=(times,values)
    assert series['local_pose'][0]==series['global_pose'][0]
    summaries['matching_timestamps']=True;summaries['matching_angle_columns']=bool(np.array_equal(series['local_pose'][1][:,3:],series['global_pose'][1][:,3:]))
    files={str(p):hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in directory.rglob('*') if p.is_file() and p.suffix!='json'}
    return dict(scope='Actual public metadata inspection,no images,inference,pose conversion or calibration fitting',cameras=cameras,baselines=baseline,poses=summaries,files_sha256=files)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(audit(a.directory),indent=2,allow_nan=False)+'\n')
