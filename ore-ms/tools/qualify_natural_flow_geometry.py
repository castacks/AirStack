"""Pre-inference conventions and analytic geometry checks; no pose values parsed."""
import ast,json,types
from pathlib import Path
import numpy as np
from natural_flow_geometry import geometry,sample,sha,write,stamp

ROOT=Path(__file__).resolve().parents[1]
REC=Path('/root/ore-ms-evidence-recovery/persistent-window/extracted/ore-ms')
OUT=ROOT/'results/natural-flow-geometry/conventions'

def main():
    source=OUT/'MS2_dataset.py'; text=source.read_text(); tree=ast.parse(text)
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='DataLoader_MS2')
    node=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='set_extrinsics')
    assert 'Abloste pose (w.r.t first image of the sequence)' in text
    configs={'day':REC/'results/ms2-day-replication/calibration.json','night':REC/'results/ms2-night-pilot/calibration.json','rain':REC/'results/pose-covariance/accuracy-001/rain/calibration.json'}
    inputs=json.loads((ROOT/'results/natural-flow-reference-intake/inputs/metadata.json').read_text())['rows']
    matrices={}; checks=0; pins={str(source):sha(source)}
    for c,path in configs.items():
        data=json.loads(path.read_text()); cal={k:np.array(v['values'],dtype=float) for k,v in data.items()}
        fake=types.SimpleNamespace(load=lambda *a,**k:types.SimpleNamespace(item=lambda:cal),concatenate=np.concatenate,linalg=np.linalg,matmul=np.matmul)
        ns={'np':fake,'torch':types.SimpleNamespace(as_tensor=np.asarray),'osp':__import__('os').path}
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
        result=ns['set_extrinsics'](types.SimpleNamespace(root='/unused',seq_list=['unused']))
        E=np.eye(4); E[:3,:3]=cal['R_nir2thr']@cal['R_nir2rgb'].T
        E[:3,3]=(cal['T_nir2thr']-E[:3,:3]@cal['T_nir2rgb']).ravel()/1000
        np.testing.assert_allclose(E[:3],result['RGB2THR'],atol=1e-12); checks+=1
        np.testing.assert_allclose(E[:3,:3].T@E[:3,:3],np.eye(3),atol=1e-10); checks+=1
        for row in [r for r in inputs if r['condition']==c]:
            np.testing.assert_array_equal(row['K'],cal['K_thrL']); checks+=1
        assert cal['T_thrR'][0,0]<0 and np.all(cal['T_thrR'][1:]==0); checks+=1
        matrices[c]=E.tolist(); pins[str(path)]=sha(path)
    # Plane with analytically known optical source->target translation.
    K=np.array([[100.,0,100],[0,100,50],[0,0,1]])
    d=np.zeros((256,640),np.uint16); d[50,100]=2560; target=d.copy(); I=np.eye(4)
    g=geometry(d,target,K,I); np.testing.assert_allclose(g['reference_flow'],[[0,0]],atol=1e-12); assert g['tier_masks'][:,0].all(); checks+=2
    T=np.eye(4); T[0,3]=-1; target[:]=0; target[50,90]=2560
    g=geometry(d,target,K,T); np.testing.assert_allclose(g['reference_flow'],[[-5,0]],atol=1e-12); assert g['tier_masks'][:,0].all(); checks+=2
    # A rotated, translated rig: conjugation matches direct world-camera composition.
    E=np.eye(4); E[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]; E[:3,3]=[.2,-.3,.1]
    P0=np.eye(4); P1=np.eye(4); P1[:3,3]=[1,2,3]
    A=E@np.linalg.solve(P1,P0)@np.linalg.inv(E)
    B=np.linalg.solve(P1@np.linalg.inv(E),P0@np.linalg.inv(E))
    np.testing.assert_allclose(A,B,atol=1e-14); checks+=1
    # Source border remains in geometry but cannot sample half-resolution flow.
    d[:]=0; d[0,0]=2560; g=geometry(d,d,K,I); assert g['tier_masks'][:,0].all() and not g['sampling_mask'][0]; checks+=1
    d[:]=0; g=geometry(d,d,K,I); assert g['tier_masks'].shape==(3,0); checks+=1
    # Independently known linear vector field and half-pixel center sampling.
    yy,xx=np.mgrid[:128,:320]; f=np.stack([2*xx+3*yy,5*xx-yy]).astype(float)
    xy=(np.array([[100.,50.],[639.,255.]])+.5)/2-.5
    np.testing.assert_allclose(sample(f,xy[:1]),[[2*xy[0,0]+3*xy[0,1],5*xy[0,0]-xy[0,1]]],atol=1e-12); checks+=1
    # Collision: nearer projected source point wins the rounded target cell.
    d[:]=0; d[50,100]=2560; d[50,101]=256
    T=np.eye(4); T[2,3]=100
    g=geometry(d,d,K,T); assert g['counts']['target_collisions']==1 and g['tier_masks'][0].tolist()==[False,True]; checks+=1
    write(OUT/'qualification.json',{'created_utc':stamp(),'status':'PASS_NOMINAL_PROVIDED_CONVENTION','checks':checks,'thermal_from_rgb':matrices,'source_pins':pins,'pose_values_parsed':False,'source_pose_interpretation':'camera optical to sequence-first camera, inherited official-loader interpretation; no physical export guarantee','depth_units':'uint16 code /256 metres, pinhole Z','depth_units_primary_source':'https://sites.google.com/view/multi-spectral-stereo-dataset/data-format','limitations':'Nominal same-ID pairing; physical timing/visibility not established. No scores or inversion fitting.'})
    print('PASS',checks,'convention/analytic checks')
if __name__=='__main__':main()
