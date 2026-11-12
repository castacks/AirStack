"""Independent matrix-power mapping and saved-error/decision audit.

Shared extrinsic lookup is not independent physical calibration qualification.
No optimizer/network is rerun and no fit/threshold is altered.
"""
import json
import numpy as np
from scipy.linalg import logm,expm
from radiometric_accuracy import ROOT,OUT,PLAN,ARMS,CASES,LOADER,sha,write,verify
from pose_covariance_accuracy import transforms


def read(p):return json.loads(p.read_text())


def pose(q):
    rotation=np.asarray(q,dtype=float)[3:];rotation=rotation/np.linalg.norm(rotation)
    x,y,z,w=rotation;v=np.array([x,y,z]);s=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    out=np.eye(4);out[:3,:3]=(w*w-v@v)*np.eye(3)+2*np.outer(v,v)+2*w*s;out[:3,3]=q[:3]
    return out


def rotation_angle(r):
    sine=np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])/2
    return float(np.degrees(np.arctan2(sine,(np.trace(r)-1)/2)))


def stats(a):
    a=np.asarray(a);return {'count':len(a),'mean':float(a.sum()/len(a)),'median':float(np.median(a)),
        'rmse':float(np.sqrt(np.dot(a,a)/len(a))),'p95':float(np.percentile(a,95)),'maximum':float(max(a))}


def audit():
    verify();lock=read(OUT/'global-prediction-lock.json')
    assert lock['freeze_sha256']==sha(PLAN)
    for n,d in lock['pins'].items():assert sha(OUT/n)==d
    summary=read(OUT/'accuracy-summary.json');checks=0;passed={}
    for c in CASES:
        base=ROOT/'results/unweighted-longer/run-001/inputs'/c
        source=read(base/'source-manifest.json');meta=read(base/'metadata.json')
        frames=transforms(read(ROOT/meta['calibration']),LOADER)
        result=read(OUT/c/'accuracy-scores.json');errors={a:{} for a in ARMS}
        for arm in ARMS:
            mode='rgb' if arm=='rgb' else 'thermal';sensor='rgb' if arm=='rgb' else 'thr'
            lines=(OUT/c/arm/mode/'arbiter-events/events.jsonl').read_text().splitlines()
            writes=[json.loads(l) for l in lines];writes=[e for e in writes if e['kind']=='motion_result_written' and e['first_write_for_submission']]
            for i in range(99):
                row=result['rows'][i];assert row['pair']==i
                a,b=[int(source['records'][k]['cameras'][sensor]['left']['recorded_time_ns']) for k in [i,i+1]]
                t0,t1=[int(source['records'][k]['cameras']['rgb']['left']['recorded_time_ns']) for k in [i,i+1]]
                candidates=[e for e in writes if e['from_idx']==i and e['frame_idx']==i+1]
                assert len(candidates)<=1
                mapped=None
                if candidates and candidates[0]['derived_camera_delta_xyzw'] is not None:
                    e=candidates[0];assert [e['time0_ns'],e['time1_ns']]==[a,b]
                    if max(a-t0,t0-b,a-t1,t1-b,0)<=b-a:
                        converted=frames[mode]@pose(e['derived_camera_delta_xyzw'])@np.linalg.inv(frames[mode])
                        mapped=expm(logm(converted)*((t1-t0)/(b-a)))
                        assert np.max(np.abs(np.imag(mapped)))<1e-8;mapped=np.real(mapped)
                entry=row['arms'][arm]
                assert (mapped is None)==(entry['transform'] is None)
                if mapped is not None:
                    np.testing.assert_allclose(mapped,entry['transform'],rtol=1e-7,atol=1e-7)
                ref=row['reference_delta']
                if mapped is not None and ref is not None:
                    relative=np.linalg.inv(ref)@mapped
                    value={'translation_m':float(np.linalg.norm(relative[:3,3])), 'rotation_deg':rotation_angle(relative[:3,:3])}
                    for k in value:np.testing.assert_allclose(value[k],entry['error'][k],rtol=1e-6,atol=1e-6)
                    errors[arm][i]=value
                else:assert entry['error'] is None
                checks+=1
        shared=sorted(set.intersection(*(set(errors[a]) for a in ARMS)))
        assert shared==result['common_pair_ids']
        for arm in ARMS:
            assert sorted(errors[arm])==result['coverage_pair_ids'][arm]
            for k in ['translation_m','rotation_deg']:
                expected=stats([errors[arm][i][k] for i in shared])
                for stat,value in expected.items():
                    np.testing.assert_allclose(value,result['metrics'][arm][k][stat],rtol=1e-6,atol=1e-6);checks+=1
        expected={'common_nonempty':bool(shared),'thermal_coverage_preserved':set(errors['original'])<=set(errors['refined'])}
        rgb={'common_nonempty':bool(shared),'rgb_coverage_preserved':set(errors['rgb'])<=set(errors['refined'])}
        # Decision comparisons use the saved exact producer scalars; independent
        # metric reconstruction above checks them at declared numerical tolerance.
        for k in ['translation_m','rotation_deg']:
            for stat in ['mean','rmse']:
                a,b,d=[result['metrics'][arm][k][stat] for arm in ['refined','original','rgb']]
                expected[k+'_'+stat]=bool(shared) and (a<b if k=='translation_m' else a<=b)
                rgb[k+'_'+stat]=bool(shared) and a<d
        assert expected==result['checks'] and rgb==result['rgb_checks']
        assert all(expected.values())==result['pass']
        assert all(rgb.values())==result['rgb_pass']
        assert summary['records'][c]=={k:v for k,v in result.items() if k!='rows'}
        passed[c]=result['pass']
    assert summary['decision']==('ADVANCE_TO_SEPARATE_RECORDING_VALIDATION' if all(passed.values()) else 'CLOSE_THIS_CONFIGURATION_NO_RETUNING')
    write(OUT/'accuracy-audit.json',{'status':'PASS','checks':checks,'auditor_sha256':sha(__file__),
        'mapping':'independent quaternion formula and SciPy logm/expm',
        'limits':'shared calibration lookup and producer reference deltas; not independent physical reference qualification'})
    print('PASS independent saved-output audit',checks,flush=True)


if __name__=='__main__':audit()

