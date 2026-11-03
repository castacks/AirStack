"""Serialization roundoff is bounded; gross nonrigidity must remain unscored."""
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from ms2_broader_evaluate import reference_pose

def test_small_roundoff_and_gross_defect(tmp_path):
    p=tmp_path/'pose.txt';a=np.eye(4);a[0,0]+=1e-7;a[:3,3]=[1,2,3];np.savetxt(p,a)
    result,report=reference_pose(p)
    assert report['valid'] and report['rotation_max_correction']<1e-6
    np.testing.assert_allclose(result[:3,:3],np.eye(3),atol=1e-12)
    np.testing.assert_array_equal(result[:3,3],[1,2,3])
    a[0,0]=1.01;np.savetxt(p,a)
    result,report=reference_pose(p)
    assert result is None and not report['valid'] and report['reason']=='nonrigid'

def test_reflection_and_nonfinite_not_repaired(tmp_path):
    p=tmp_path/'pose.txt';a=np.eye(4);a[0,0]=-1;np.savetxt(p,a)
    assert reference_pose(p)[0] is None
    a=np.eye(4);a[0,3]=np.nan;np.savetxt(p,a)
    assert reference_pose(p)[0] is None

def test_range_reader_exact_bounds_cache_and_seek(tmp_path,monkeypatch):
    import io
    import ms2_broader_intake as intake
    data=b'abcdefghijklmnopqrstuvwxyz';calls=[]
    class Response(io.BytesIO):
        status=206
        def __init__(self,start,stop):
            super().__init__(data[start:stop+1]);self.headers={'Content-Range':f'bytes {start}-{stop}/{len(data)}'}
    def request(req,timeout):
        bounds=req.get_header('Range').removeprefix('bytes=');a,b=map(int,bounds.split('-'));calls.append((a,b));return Response(a,b)
    monkeypatch.setattr(intake.urllib.request,'urlopen',request)
    r=intake.RangeReader('https://source.invalid/archive',len(data),tmp_path);r.block=8
    r.seek(6);assert r.read(7)==data[6:13];assert calls==[(0,7),(8,15)]
    r.seek(7);assert r.read(3)==data[7:10];assert len(calls)==2
    r.seek(-3,2);assert r.read(10)==b'xyz';assert r.read(10)==b''
