import os,subprocess,sys
import numpy as np
from Scripts.Setup.flow_cycle_sampling import bilinear,summaries,forbidden_read


def test_affine_fractional_and_borders():
 y,x=np.mgrid[:4,:5];field=np.stack((2*x+3*y,-x+4*y),axis=-1).astype(np.float32);uv=np.array([[.2,1.7],[4,3],[0,0],[4,1.5]])
 r=bilinear(field,uv);np.testing.assert_allclose(r['sampled'],np.column_stack((2*uv[:,0]+3*uv[:,1],-uv[:,0]+4*uv[:,1])),atol=1e-12);assert r['sample_valid'].all();np.testing.assert_allclose(r['weights'].sum(1),1)


def test_random_scalar_interpolation():
 field=np.random.default_rng(0).normal(size=(9,8,2));points=np.random.default_rng(1).uniform([0,0],[7,8],size=(43,2));r=bilinear(field,points)
 expected=[]
 for x,y in points:
  a,b=int(np.floor(x)),int(np.floor(y));dx,dy=x-a,y-b;expected.append((1-dx)*(1-dy)*field[b,a]+dx*(1-dy)*field[b,a+1]+(1-dx)*dy*field[b+1,a]+dx*dy*field[b+1,a+1])
 np.testing.assert_allclose(r['sampled'],expected,atol=1e-14)


def test_outside_and_nonfinite():
 f=np.zeros((3,3,2));f[1,1]=np.nan;points=np.array([[-.001,0],[3,0],[0,np.nan],[np.inf,1],[.5,.5],[2,2]])
 r=bilinear(f,points);assert r['in_bounds'].tolist()==[False,False,False,False,True,True];assert r['sample_valid'].tolist()==[False]*5+[True];assert np.isnan(r['sampled'][:5]).all();assert (r['indices'][:4]==-1).all()


def test_inverse_translation_and_normalization():
 uv0=np.array([[1.,1.],[2,2]]);uv1=uv0+[2,-.5];field=np.broadcast_to([-2.,.5],(5,6,2)).copy();r=bilinear(field,uv1);k=np.diag([3.,4.,1]);cov=np.tile([9.,16.,0.],(2,1));closure,valid,s=summaries(uv0,uv1,np.ones(2,bool),r['sampled'],r['sample_valid'],k,cov)
 assert valid.all() and np.array_equal(closure,np.zeros((2,2)));assert s['closure_angular_q25_q50_q75']==[0,0,0] and s['angular_covariance_median']==2
 _,_,s=summaries(uv0,uv0,np.ones(2,bool),np.tile([3.,4.],(2,1)),np.ones(2,bool),k,cov);np.testing.assert_allclose(s['closure_angular_q25_q50_q75'],np.sqrt(2))


def test_empty_valid_is_unavailable():
 uv=np.array([[1.,1.]]);_,mask,s=summaries(uv,uv,np.array([False]),np.zeros((1,2)),np.array([True]),np.eye(3),np.zeros((1,3)))
 assert not mask.any() and s['valid_closure_count']==0 and s['closure_angular_q25_q50_q75']==[None]*3 and s['angular_covariance_median'] is None


def test_label_paths():
 for p in ('/data/super_odometry/odometry/data.txt','evaluation.json','/tmp/trajectory.npz',b'pose-proxy.json'):assert forbidden_read(p)
 for p in ('/dataset/manifest.json','pair-000.npz','effective-config.json','capture.json',10):assert not forbidden_read(p)


def test_real_audit_hook(tmp_path):
 target=tmp_path/'evaluation.json';target.write_text('{}');allowed=tmp_path/'capture.json';code='from Scripts.Setup.flow_cycle_sampling import install_label_guard;from pathlib import Path;import sys;d=install_label_guard();Path(sys.argv[2]).write_text("ok");assert Path(sys.argv[2]).read_text()=="ok"\ntry:Path(sys.argv[1]).read_text()\nexcept PermissionError:assert len(d)==1\nelse:raise AssertionError("guard bypass")'
 subprocess.run([sys.executable,'-c',code,str(target),str(allowed)],check=True)
