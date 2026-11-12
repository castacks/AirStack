"""Saved-output metrics audit, with explicit original and valid-snippet counts."""
import ast
from types import SimpleNamespace
import cv2
import numpy as np
from thermal_sfm_reproduction import AUTHOR,ASSETS,OUT,SEQUENCES,PLAN,ROOT,sha,write,verify
import json

DEPTH_KEYS=('abs_rel','sq_rel','rmse','rmse_log','log10','a1','a2','a3')


def source_function(filename,name,args=None):
    tree=ast.parse((AUTHOR/filename).read_text())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    scope={'np':np,'args':args}
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(AUTHOR/filename),'exec'),scope)
    return scope[name]


def depth_rows(scene):
    pred=np.load(OUT/scene/'depth/predictions.npy',mmap_mode='r')
    files=sorted((ASSETS/'prepared'/scene/'Depth_T').glob('*.npy'))
    images=sorted((ASSETS/'prepared'/scene/'Thermal').glob('*.png'))
    assert len(pred)==len(files)==len(images)
    author=source_function('eval_vivid/eval_depth.py','compute_depth_errors',SimpleNamespace(dataset='VIVID'))
    rows=[]
    for i,path in enumerate(files):
        assert path.stem==images[i].stem
        gt=np.load(path);assert np.isfinite(gt).all()
        inv=1/(np.array(pred[i])+1e-6)
        resized=1/(cv2.resize(inv,(gt.shape[1],gt.shape[0]))+1e-6)
        mask=(gt>1e-3)&(gt<80);assert mask.any()
        truth,values=gt[mask],resized[mask]
        ratio=float(np.median(truth)/np.median(values))
        values=np.clip(values*ratio,1e-3,80)
        residual=truth-values
        logarithm=np.log(truth/values)
        threshold=np.maximum(truth/values,values/truth)
        own=np.array([np.mean(np.abs(residual)/truth),np.mean(residual**2/truth),
            np.linalg.norm(residual)/np.sqrt(len(residual)),np.linalg.norm(logarithm)/np.sqrt(len(residual)),
            np.mean(np.abs(np.log10(truth/values))),*(np.mean(threshold<1.25**n) for n in [1,2,3])])
        expected=np.array(author(truth,values))
        np.testing.assert_allclose(own,expected,rtol=2e-6,atol=2e-6)
        rows.append({'stem':path.stem,'ratio':ratio,'valid_pixels':int(mask.sum()),
                     'metrics':dict(zip(DEPTH_KEYS,map(float,own)))})
    return rows


def pose_rows(scene):
    pred=np.load(OUT/scene/'pose/predictions.npy')
    gt=np.loadtxt(ASSETS/'prepared'/scene/'poses_T.txt').reshape(-1,3,4)
    count=len(list((ASSETS/'prepared'/scene/'Thermal').glob('*.png')))
    assert len(pred)==len(gt)==count and pred.shape[1:]==(5,3,4)
    assert np.array_equal(pred[count-4:],np.zeros_like(pred[count-4:])), 'Author unused allocated rows differ'
    author=source_function('test_pose.py','compute_pose_error')
    rows=[]
    for i in range(count-4):
        truth=gt[i:i+5].copy();first=truth[0].copy()
        truth[:,:,-1]-=first[:,-1]
        truth=np.linalg.inv(first[:,:3])@truth
        candidate=pred[i]
        scale=float(np.sum(truth[:,:,3]*candidate[:,:,3])/np.sum(candidate[:,:,3]**2))
        ate=float(np.sqrt(np.sum((truth[:,:,3]-scale*candidate[:,:,3])**2))/5)
        angles=[]
        for a,b in zip(truth,candidate):
            relative=a[:,:3]@np.linalg.inv(b[:,:3])
            angles.append(np.arctan2(np.linalg.norm([relative[0,1]-relative[1,0],relative[1,2]-relative[2,1],relative[2,0]-relative[0,2]]),np.trace(relative)-1))
        own=np.array([ate,float(np.sum(angles)/5)])
        expected=np.array(author(truth,candidate))
        np.testing.assert_allclose(own,expected,rtol=1e-10,atol=1e-10)
        rows.append({'snippet_index':i,'scale':scale,'ATE':ate,'RE':float(own[1])})
    values=np.array([[r['ATE'],r['RE']] for r in rows])
    # Original author errors buffer is float32 with four unfilled zero rows.
    buffer=np.zeros((count,2),np.float32);buffer[:count-4]=values
    return {'allocated_rows':count,'actual_snippets':count-4,'unused_zero_rows':4,
        'source_mean':buffer.mean(0).astype(float).tolist(),'source_std':buffer.std(0).astype(float).tolist(),
        'valid_mean':values.mean(0).tolist(),'valid_std':values.std(0).tolist(),'rows':rows}


def main():
    verify();lock=json.loads((OUT/'prediction-lock.json').read_text())
    for name,digest in lock['pins'].items():assert sha(OUT/name)==digest,name
    reports={};pooled=[]
    for scene in SEQUENCES:
        rows=depth_rows(scene);pooled.extend(rows)
        depth={k:float(np.mean([r['metrics'][k] for r in rows])) for k in DEPTH_KEYS}
        pose=pose_rows(scene)
        write(OUT/scene/'depth-metrics-audit.json',{'status':'PASS_SOURCE_METRIC_EQUIVALENCE','rows':rows,'mean':depth})
        write(OUT/scene/'pose-metrics-audit.json',{'status':'PASS_SOURCE_METRIC_EQUIVALENCE',**pose})
        reports[scene]={'frames':len(rows),'depth':depth,'pose':{k:v for k,v in pose.items() if k!='rows'}}
    primary={k:float(np.mean([r['metrics'][k] for r in pooled])) for k in DEPTH_KEYS}
    macro={k:float(np.mean([reports[s]['depth'][k] for s in SEQUENCES])) for k in DEPTH_KEYS}
    plan=json.loads(PLAN.read_text());target=plan['paper_targets']
    depth_checks={k:abs(primary[k]-v)<=0.0005 for k,v in target['depth'].items()}
    pose_checks={s:{kind:all(abs(a-b)<=0.00005 for a,b in zip(reports[s]['pose'][kind],values)) for kind,values in targets.items()} for s,targets in target['pose'].items()}
    write(OUT/'summary.json',{'status':'MODERN_RUNTIME_SOURCE_PROTOCOL_EXECUTED',
        'reports':reports,'primary_pooled_depth':primary,'secondary_sequence_macro_depth':macro,
        'paper_depth_checks':depth_checks,'paper_pose_checks':pose_checks,
        'paper_rounded_targets_all_match':all(depth_checks.values()) and all(all(v.values()) for v in pose_checks.values()),
        'auditor_sha256':sha(__file__),'depth_metric_checks':len(pooled)*8,'pose_metric_checks':sum(r['pose']['actual_snippets']*2 for r in reports.values()),
        'scope':'Per-image median-scaled depth and per-snippet reference-scaled pose; not unscaled metric stereo VO or independent model-training split qualification'})
    print(json.dumps({'pooled_depth':primary,'reports':reports,'depth_checks':depth_checks,'pose_checks':pose_checks},indent=2))


if __name__=='__main__':main()
