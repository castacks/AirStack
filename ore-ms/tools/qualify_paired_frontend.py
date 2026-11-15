"""Pre-ensemble mechanics, native algorithm/selection/covariance null controls."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
import paired_frontend_assessment as m

SETTINGS={'max_iterations':100,'score_tolerance':1e-6,'rank_ratio':1e-10,'line_search_steps':25,'armijo':1e-4}

def main():
    r=m.ROOT;out=r/'results/paired-frontend-qualification';checks=0
    def close(a,b,atol=1e-8):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=1e-7,atol=atol);checks+=1
    rng=np.random.default_rng(20261114);X=rng.normal(size=(64,3))*[1,1,.3]+[0,0,10];T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([.02,-.03,.01]).as_matrix();T[:3,3]=[.12,-.03,.02]
    p={'X0':X@T[:3,:3].T+T[:3,3],'X1':X,'C0':np.broadcast_to(np.eye(3)*.01,(64,3,3)).copy(),'C1':np.broadcast_to(np.eye(3)*.02,(64,3,3)).copy(),'length':np.array(10.)}
    result=m.fit(p,SETTINGS);assert result['successful'];checks+=1;close(result['target_to_source'],T)
    p0={**p,'X0':X.copy()};zero=m.fit(p0,SETTINGS);assert zero['successful'] and len(zero['history'])==1;checks+=1
    x,C=m.transport_cov(T,T,np.eye(6));close(x,np.zeros(6));close(C,np.eye(6),atol=1e-7)
    q=p['X1']@T[:3,:3].T+T[:3,3];rr,A,L,D=m.frozen_system(p,T)
    for j in range(6):
        eps=1e-6;d=D@np.eye(6)[j]*eps;v=[]
        for sign in [1,-1]:
            U=m.exp_se3(sign*d)@T;res=X@U[:3,:3].T+U[:3,3]-p['X0'];v.append(np.linalg.solve(L,res[...,None])[...,0].ravel())
        close(A[:,j],(v[0]-v[1])/(2*eps),atol=1e-7)
    selector,covariance=m.native_algorithms();rec=m.read(out/'recovery.json');row=rec['rows'][0];assert rec['rows'][1]['K']==row['K'];checks+=1
    with np.load(out/'run-001/outputs.npz') as z:f=z['flow'];c=z['covariance']
    ids=m.pick(f,c,row,selector,20261114);ids2=m.pick(f,c,row,selector,20261114);assert np.array_equal(ids,ids2) and 3<=len(ids)<=200;checks+=1
    packet=m.packet(f,c,row,ids,covariance);packet2=m.packet(f,c,row,ids2,covariance)
    for k in packet:assert np.array_equal(packet[k],packet2[k]);checks+=1
    np.savez_compressed(out/'null-native-packet.npz',**packet)
    first=m.fit(packet,SETTINGS);second=m.fit(packet2,SETTINGS);assert first==second;checks+=1
    # Preserve the real baseline fit status; it is not an outcome-selected solver tuning gate.
    report={'created_utc':m.now(),'status':'PASS_SHARED_INPUT_MECHANICS','checks':checks,'solver':SETTINGS,'day_null_selected_points':len(ids),'day_null_accepted_points':len(packet['source_ids']),'day_null_fit':first,'identical_query_and_fit_receipts':True,'known_transform_recovered':True,'source_sha256':{str(r/'tools'/s):m.sha(r/'tools'/s) for s in ['paired_frontend_assessment.py','qualify_paired_frontend.py','pose_covariance_replay.py']},'scope':'No MonteCarlo jitter or reference pose values. Native algorithms extracted with detached inputs; explicit frozen-weight IRLS 3D harness, not full native robust/stereo BA or MACI2 VI system.'}
    m.write(out/'shared-mechanics-qualification.json',report);print(json.dumps({'status':report['status'],'checks':checks,'native_selected':len(ids),'native_accepted':len(packet['source_ids']),'null_fit_status':first['status']}))
if __name__=='__main__':main()
