"""Independent NumPy-only saved-output checks; no torch/model imports."""
import sys,json,hashlib
from pathlib import Path
import numpy as np


def main(folder):
    p=Path(folder);r=json.loads((p/'report.json').read_text());assert hashlib.sha256((p/'arrays.npz').read_bytes()).hexdigest()==r['arrays_sha256'];checks=1
    with np.load(p/'arrays.npz') as a:
      for row in r['cases']:
        n=row['case'];flow=a[n+'_flow'];alpha=a[n+'_alpha'];truth=a[n+'_truth'];mask=a[n+'_mask'];images=a[n+'_input_uint8'];assert images.dtype==np.uint8 and images.shape==(2,125,157)
        yy,xx=np.mgrid[:125,:157];dx,dy=(0,0) if n=='identity' else (4,-3)
        expected=(xx+dx>=0)&(xx+dx<157)&(yy+dy>=0)&(yy+dy<125);assert np.array_equal(mask,expected)
        assert np.array_equal(images[1,(yy+dy)[mask],(xx+dx)[mask]],images[0,yy[mask],xx[mask]])
        assert np.array_equal(truth,np.broadcast_to(np.array([dx,dy])[:,None,None],(2,125,157)))
        assert np.isfinite(flow).all() and np.isfinite(alpha).all()
        e=np.sqrt((flow[0].astype(float)-dx)**2+(flow[1].astype(float)-dy)**2)[mask]
        assert abs(e.mean()-row['epe_mean'])<1e-12
        assert abs(np.sqrt(np.mean(e*e))-row['epe_rmse'])<1e-12
        assert int(mask.sum())==row['valid_pixels'];checks+=8
    result={'status':'PASS','checks':checks,'scope':'Independent saved coordinate identity, mask, true displacement and EPE reconstruction; does not rerun neural inference or certify covariance.'};(p/'independent-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main(sys.argv[1])
