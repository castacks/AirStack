"""Independent width interpolation, scalar errors and per-record gates."""
import json
import numpy as np
import cv2
from ms2_depth_transfer import ROOT,OUT,PLAN,DATA,sha,verify,write


def audit():
    verify();lock=json.loads((OUT/'prediction-lock.json').read_text())
    assert lock['freeze_sha256']==sha(PLAN)
    for n,d in lock['pins'].items():assert sha(OUT/n)==d
    summary=json.loads((OUT/'summary.json').read_text());checks=0;passed={}
    for record in json.loads(DATA.read_text())['records']:
        condition=record['condition'];saved=summary['records'][condition];reconstructed=[]
        for row,reported in zip(record['rows'],saved['rows'],strict=True):
            assert row['source_id']==reported['source_id']
            gt=cv2.imread(str(ROOT/row['depth']['path']),-1).astype(float)/256
            with np.load(OUT/'predictions'/condition/(row['source_id']+'.npz')) as z:
                inv=1/(z['mono_native'].astype(float)+1e-6)
                x=(np.arange(640)+.5)/2-.5
                resized=np.vstack([np.interp(x,np.arange(320),line) for line in inv])
                candidate=1/(resized+1e-6);baseline=z['mact_native'].astype(float)
            reference=np.isfinite(gt)&(gt>1e-3)&(gt<80)
            b=reference&np.isfinite(baseline)&(baseline>0)
            c=reference&np.isfinite(candidate)&(candidate>0);mask=b&c
            for name,actual in [('reference_pixels',reference.sum()),('baseline_pixels',b.sum()),('candidate_pixels',c.sum()),('common_pixels',mask.sum())]:assert reported[name]==actual;checks+=1
            preserved=not np.any(b&~c);assert reported['candidate_preserves_baseline_support']==preserved
            values={}
            if not mask.any():assert reported['metrics'] is None
            else:
                target=gt[mask]
                def median(a):
                    a=np.sort(a);return (a[(len(a)-1)//2]+a[len(a)//2])/2
                for name,full in [('mact',baseline),('mono',candidate),('unscaled_mact',baseline)]:
                    raw=full[mask];scale=median(target)/median(raw) if name!='unscaled_mact' else 1.
                    residual=raw*scale-target
                    value={'abs_rel':float(np.sum(np.abs(residual)/target)/len(target)),
                        'rmse':float(np.linalg.norm(residual)/np.sqrt(len(target)))}
                    if name!='unscaled_mact':value['scale']=float(scale)
                    for key,v in value.items():np.testing.assert_allclose(v,reported['metrics'][name][key],rtol=1e-9,atol=1e-9);checks+=1
                    values[name]=value
            reconstructed.append({'metrics':values if mask.any() else None,'preserved':preserved})
        valid=[r for r in reconstructed if r['metrics'] is not None]
        means={name:{key:float(np.mean([r['metrics'][name][key] for r in valid])) if valid else None for key in ['abs_rel','rmse']} for name in ['mact','mono','unscaled_mact']}
        for name in means:
            for key in means[name]:np.testing.assert_allclose(means[name][key],saved['equal_frame_metrics'][name][key],rtol=1e-9,atol=1e-9);checks+=1
        gate={'all20frames_scored':len(valid)==len(reconstructed)==20,'no_support_loss':all(r['preserved'] for r in reconstructed)}
        for key in ['abs_rel','rmse']:gate[key+'_improves']=bool(valid) and means['mono'][key]<means['mact'][key]
        assert gate==saved['checks'] and all(gate.values())==saved['pass'];passed[condition]=all(gate.values())
    expected='ADVANCE_TO_SEPARATE_SCALE_AND_VO_CONTRACT' if all(passed.values()) else 'CLOSE_THIS_CHECKPOINT_TRANSFER_CONFIGURATION_NO_RETUNING'
    assert summary['decision']==expected
    write(OUT/'audit.json',{'status':'PASS','checks':checks,'recording_pass':passed,'auditor_sha256':sha(__file__),
        'source':'Independent NumPy width interpolation and scalar metric reconstruction; shared raw data/OpenCV PNG decoder, no network rerun'})
    print('PASS',checks,passed,expected)


if __name__=='__main__':audit()
