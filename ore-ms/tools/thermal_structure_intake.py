"""Exploratory raw-count thermal spatial/temporal descriptors; no pose fitting."""
import hashlib,json,math
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/thermal-structure-intake/run-001'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 OUT.mkdir(parents=True,exist_ok=False);rows=[];pairs=[];checks=[]
 for condition in ['day','night','rain']:
  paths=sorted((ROOT/'results/u2flow-thermal-warp/inputs'/condition).glob('*.png'));assert len(paths)==20
  previous=None
  for p in paths:
   before=digest(p);raw=np.asarray(Image.open(p));assert raw.shape==(256,640) and np.issubdtype(raw.dtype,np.integer)
   x=raw.astype(float);lo,hi=np.percentile(x,[5,95]);span=float(hi-lo);z=(x-lo)/span if span>0 else np.zeros_like(x)
   gx=(z[1:-1,2:]-z[1:-1,:-2])/2;gy=(z[2:,1:-1]-z[:-2,1:-1])/2
   tensor=np.array([[np.mean(gx*gx),np.mean(gx*gy)],[np.mean(gx*gy),np.mean(gy*gy)]]);ev=np.linalg.eigvalsh(tensor)
   row={'condition':condition,'frame':p.stem,'sha256':before,'dtype':str(raw.dtype),'raw_p95_minus_p5':span,'normalized_gradient_rms':float(np.sqrt(np.mean(gx*gx+gy*gy))),'structure_eigenvalues':ev.tolist(),'minor_fraction':float(ev[0]/ev.sum()) if ev.sum()>0 else None}
   rows.append(row)
   if previous is not None:
    prevstem,a=previous;ac=a-a.mean();bc=z-z.mean();den=np.linalg.norm(ac)*np.linalg.norm(bc)
    pairs.append({'condition':condition,'source':prevstem,'target':p.stem,'unregistered_normalized_abs_change':float(np.mean(np.abs(z-a))),'unregistered_correlation':float(np.sum(ac*bc)/den) if den>0 else None})
   if p==paths[0]:
    # Scalar arithmetic over fixed interior lattice, independent from array slicing.
    values=[]
    for y in range(1,255,19):
     for xx in range(1,639,23):
      sx=(float(z[y,xx+1])-float(z[y,xx-1]))/2;sy=(float(z[y+1,xx])-float(z[y-1,xx]))/2
      assert abs(sx-gx[y-1,xx-1])<1e-14 and abs(sy-gy[y-1,xx-1])<1e-14
      values.append(sx*sx+sy*sy)
    checks.append({'condition':condition,'scalar_gradient_checks':len(values)*2})
   assert digest(p)==before;previous=(p.stem,z)
 summary={}
 for c in ['day','night','rain']:
  rr=[r for r in rows if r['condition']==c];pp=[r for r in pairs if r['condition']==c]
  summary[c]={k:float(np.median([r[k] for r in rr])) for k in ['raw_p95_minus_p5','normalized_gradient_rms','minor_fraction']}
  summary[c].update({k:float(np.median([r[k] for r in pp])) for k in ['unregistered_normalized_abs_change','unregistered_correlation']})
 result={'scope':'Exploratory first20 frames/condition, raw sensor counts not calibrated temperature; per-image normalization; unregistered change is not flow error; no RGB comparison or VO gain measured.','baseline':'53afa19d1','source_sha256':digest(Path(__file__)),'frames':rows,'pairs':pairs,'summary':summary,'checks':checks}
 (OUT/'descriptors.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({'frames':len(rows),'pairs':len(pairs),'summary':summary,'checks':checks},indent=2))
if __name__=='__main__':main()
