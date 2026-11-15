from pathlib import Path
import json,tarfile,hashlib
TMP=Path('/tmp/thermal-intake');OUT=Path('/root/AirStack/ore-ms/results/longer-fixed-thermal/recovery');OUT.mkdir(parents=True,exist_ok=True)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
proof=[]
for label in ['decisive','accuracy','fusion']:
 m=json.load(open(TMP/(label+'-manifest.json')));arc=TMP/(label+'.tar.gz')
 expected=m['archive']['sha256'] if label=='decisive' else m['files'][{'accuracy':'accuracy-artifacts.tar.gz','fusion':'translation-fusion-artifacts.tar.gz'}[label]]['sha256'];assert sha(arc)==expected
 count=0
 with tarfile.open(arc) as t:
  for x in t:
   if not x.isfile():continue
   dest=None
   if label=='decisive':
    prefix='ore-ms/results/decisive-uncertainty-evaluation/run-001/'
    if not x.name.startswith(prefix):continue
    sub=x.name.removeprefix(prefix)
    if sub.startswith('captures/') and sub.endswith(('tensor-map.npz','arbiter-events/events.jsonl','effective-config.json')):dest=OUT/sub
    if sub.startswith('inputs/') and sub.endswith(('calibration.json','source-manifest.json')):dest=OUT/sub
   elif label=='accuracy':
    prefix='results/pose-covariance/accuracy-001/rain/'
    if x.name.startswith(prefix):
     sub=x.name.removeprefix(prefix)
     if sub.startswith('captures/') and sub.endswith(('tensor-map.npz','arbiter-events/events.jsonl','effective-config.json')):dest=OUT/'captures/rain'/sub.removeprefix('captures/')
     if sub in ['calibration.json','source-manifest.json']:dest=OUT/'inputs/rain'/sub
   else:
    prefix='ore-ms/results/pose-covariance/accuracy-001/references/'
    if x.name.startswith(prefix) and x.name.endswith('.txt'):dest=OUT/'references'/x.name.removeprefix(prefix)
   if dest is None:continue
   b=t.extractfile(x).read();assert hashlib.sha256(b).hexdigest()==m['member_sha256'][x.name];dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(b);count+=1
 proof.append({'archive':label,'sha256':expected,'verified_restored_members':count})
 print(label,count,flush=True)
(OUT/'recovery-proof.json').write_text(json.dumps(proof,indent=2)+'\n')
