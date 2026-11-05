"""Replay fixed EXP145 inputs in their ORIGINAL pinned producer runtime."""
import sys,json,hashlib
from pathlib import Path
import cv2,numpy as np
from orb_ms2_development import thermal_function
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/stereo-reference-diagnostic/run-001/producer-replay.json'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def run():
 source=read(ROOT/'results/orb-finalization/run-001/source-manifest.json');replay=thermal_function();rows=[]
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  p=read(ROOT/'results/orb-finalization/run-001/captures'/(mode+'-paired')/'plan.json');items={Path(x['path']).name+':'+Path(x['path']).parent.name:x for x in p['prepared_images']}
  for i in range(0,2000,100):
   row=source['records'][i]
   for side in ['left','right']:
    rawpath=ROOT/row['cameras'][m][side]['path'];item=items[row['stem']+'.png:'+side];path=Path(item['path'])
    raw=cv2.imread(str(rawpath),-1);im=cv2.imread(str(path),-1);out=raw if m=='rgb' else replay(raw)
    if sha(rawpath)!=row['cameras'][m][side]['sha256'] or sha(path)!=item['sha256'] or not np.array_equal(im,out):raise ValueError('original producer/source replay')
    rows.append(dict(modality=mode,index=i,side=side,raw_sha256=sha(rawpath),canonical_sha256=sha(path),pixel_sha256=hashlib.sha256(out.tobytes()).hexdigest(),exact=True))
 result=dict(passed=True,opencv=cv2.__version__,numpy=np.__version__,runtime='sha256:1542a9a2fe4f645aa7b3db1d4d002c0a0b5941a91db105c5587db62b68bd8445',images=rows,script_sha256=sha(__file__))
 with OUT.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print('EXACT ORIGINAL PRODUCER REPLAY',len(rows),'images',cv2.__version__)
if __name__=='__main__':run()
