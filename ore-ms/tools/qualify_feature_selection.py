"""Qualify mechanics on analytic cases and input validity, before real subset fits."""
import itertools
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import feature_selection_comparison as m

ROOT=Path(__file__).resolve().parents[1]
SETTINGS={'max_iterations':100,'gradient_relative':1e-9,'rank_ratio':1e-12,'line_search_steps':24,'armijo':1e-4}

def main():
    out=ROOT/'results/feature-selection';out.mkdir(exist_ok=True);checks=0
    def ok(x):
        nonlocal checks
        assert x;checks+=1
    def close(x,y,atol=1e-8):
        nonlocal checks
        np.testing.assert_allclose(x,y,atol=atol,rtol=1e-8);checks+=1
    rng=np.random.default_rng(20261113);X=rng.normal(size=(96,3))*[2,1,.5]+[0,0,8]
    R=Rotation.from_rotvec([.03,-.02,.04]).as_matrix();t=np.array([.12,-.08,.04])
    C=rng.normal(size=(96,3,3));S=C@C.transpose(0,2,1)*.001+np.eye(3)*.01
    p={'ids':np.arange(96),'uv':np.column_stack([rng.uniform(0,640,96),rng.uniform(0,256,96)]),'X0':X@R.T+t,'X1':X,'S':S,'length':np.array(8.)}
    qR=Rotation.from_rotvec([.01,.02,-.01]).as_matrix();qt=np.array([.1,-.1,.05]);r,A=m.system(p,qR,qt)
    for j in range(6):
        delta=np.zeros(6);delta[j]=1e-6;v=[]
        for sign in [1,-1]:
            dr=Rotation.from_rotvec(sign*delta[3:]).as_matrix();rr,_=m.system(p,dr@qR,dr@qt+sign*float(p['length'])*delta[:3]);v.append(rr)
        close(A[:,j],(v[0]-v[1])/2e-6,atol=1e-7)
    synthetic=[]
    for name,method,seed in m.arms([20261113,20261114,20261115,20261116]):
        idx=m.select(p,method,32,seed,(256,640));ok(len(idx)==(96 if method=='all_pool' else 32) and len(np.unique(idx))==len(idx));ok(np.array_equal(idx,m.select(p,method,32,seed,(256,640))))
        result=m.solve(m.subset(p,idx),SETTINGS);ok(result['successful']);T=np.array(result['target_to_source']);close(T[:3,:3],R);close(T[:3,3],t)
        ok(np.all(np.diff(result['cost_history'])<=1e-10));synthetic.append({'method':name,'selected':len(idx),'status':result['status'],'iterations':result['iterations'],'final_cost':result['final_cost']})
    # Exact identity and full-pool parity, including bitwise solve receipts.
    small=m.subset(p,np.arange(23));base=m.solve(small,SETTINGS)
    for _,method,seed in m.arms([20261113,20261114,20261115,20261116]):
        idx=m.select(small,method,32,seed,(256,640));ok(np.array_equal(idx,np.arange(23)));ok(m.solve(m.subset(small,idx),SETTINGS)==base)
    ident={**p,'X0':p['X1'].copy()};ok(m.solve(ident,SETTINGS)['iterations']==0)
    line={**p,'X0':np.column_stack([np.zeros(96),np.zeros(96),np.arange(96)+1.]),'X1':np.column_stack([np.zeros(96),np.zeros(96),np.arange(96)+1.])};ok(m.solve(line,SETTINGS)['status']=='DEGENERATE_GEOMETRY')
    # Unit invariance and a common covariance-scale control for information selection.
    idx=m.select(p,'information',32)
    scaled={**p,'X0':p['X0']*100,'X1':p['X1']*100,'S':p['S']*10000,'length':p['length']*100}
    close(m.info_blocks(p),m.info_blocks(scaled));ok(np.array_equal(idx,m.select(scaled,'information',32)))
    ok(np.array_equal(idx,m.select({**p,'S':p['S']*4},'information',32)))
    blocks=m.info_blocks(p);ridge=np.trace(blocks.sum(0))*1e-6/6
    expected=int(np.argmax([np.linalg.slogdet(np.eye(6)*ridge+b)[1] for b in blocks]));ok(m.select(p,'information',1).tolist()==[expected])
    reliable={**p,'S':np.eye(3)[None]*np.arange(1,97)[:,None,None]};ok(m.select(reliable,'reliability',32).tolist()==list(range(32)))
    # All accepted real factors checked for finite means/positive depth/SPD, no selection or fit.
    inventory=m.read(ROOT/'results/feature-selection-intake/inventory.json');packets=[]
    for stream in inventory['streams']:
      ok(m.sha(stream['graph_path'])==stream['graph_sha256'])
      with np.load(stream['graph_path']) as z:
       for pair in range(19):
        packet=m.prepare(z,pair);ok(len(packet['ids'])>=3);ok(float(packet['length'])>0)
        expected=(z['match//obs1_covTc'][packet['ids']]+z['match//obs2_covTc'][packet['ids']]).astype(float)
        # Native [depth,right,down] -> optical [right,down,depth].
        close(packet['S'],expected[:,[1,2,0]][:,:,[1,2,0]],atol=1e-12)
        eigen=np.linalg.eigvalsh(packet['S']);ok(np.min(eigen)>0)
        packets.append({'condition':stream['condition'],'camera':stream['camera'],'pair':pair,'factors':len(packet['ids']),'length_m':float(packet['length']),'covariance_minimum_eigenvalue':float(np.min(eigen))})
    ok(len(packets)==114 and sum(p['factors'] for p in packets)==15987)
    m.write(out/'qualification.json',{'created_utc':m.stamp(),'status':'PASS','checks':checks,'solver':SETTINGS,'synthetic':synthetic,'real_packet_validity':packets,'real_subsets_selected':False,'real_poses_fitted':False,'provided_reference_values_parsed':False,'source_sha256':{str(ROOT/'tools'/f):m.sha(ROOT/'tools'/f) for f in ['feature_selection_comparison.py','qualify_feature_selection.py']}})
    print('PASS',checks,'checks;114input packets/15987factors valid; no real subsets or poses')
if __name__=='__main__':main()
