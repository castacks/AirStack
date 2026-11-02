"""Metadata only: enumerate pinned archives and exact declared ZIP entries."""
import datetime
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile
from Scripts.Setup.remote_zip import HTTPRangeFile

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/prospective-objective-metadata/attempt-002'
DECLARATION=ROOT/'docs/research/prospective-objective-selection-2026-11-02.json'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads(DECLARATION.read_text())
amendment=ROOT/'docs/research/prospective-objective-availability-amendment-2026-11-02.json'
a=json.loads(amendment.read_text());assert sha(__file__)==a['metadata_script_sha256'];p['selection_rule']=a['new_selection_rule'];p['availability_amendment']={'path':str(amendment.relative_to(ROOT)),'sha256':sha(amendment)}
for rel,digest in p['protected_pins'].items():assert sha(ROOT/rel)==digest,rel
for pin in p['models'].values():assert sha(ROOT/pin['path'])==pin['sha256']
OUT.mkdir(parents=True,exist_ok=False)
(OUT/'selection-protocol.json').write_bytes(DECLARATION.read_bytes())
archives=[];transferred=0;cases=[]
with (OUT/'metadata-range-requests.jsonl').open('x') as log:
 def record(row):
  global transferred
  transferred+=row['bytes'];assert transferred<=p['metadata_budget_bytes']
  log.write(json.dumps(row)+'\n');log.flush()
 for case in p['cases']:
  env=case['environment'];url=f"https://huggingface.co/api/datasets/theairlabcmu/tartanair/tree/{p['source_revision']}/{env}/Easy?recursive=false&expand=false&limit=100"
  with urllib.request.urlopen(url,timeout=45) as response:
   raw=response.read(1024*1024+1);assert len(raw)<=1024*1024
  tree=json.loads(raw);(OUT/(env+'-tree.json')).write_bytes(raw)
  indices=[];case_archives=[]
  for kind,archive_name in [('image_left','image_left.zip'),('flow_flow','flow_flow.zip'),('flow_mask','flow_mask.zip')]:
   entry=next(e for e in tree if e['path']==f'{env}/Easy/{archive_name}')
   public=f"https://huggingface.co/datasets/theairlabcmu/tartanair/resolve/{p['source_revision']}/{entry['path']}"
   reader=HTTPRangeFile(public,entry['size'],record=record,budget=p['metadata_budget_bytes'])
   with zipfile.ZipFile(reader) as archive:
    index=[{'name':i.filename,'size':i.file_size,'compressed_size':i.compress_size,'crc32':i.CRC,'header_offset':i.header_offset,'method':i.compress_type,'flags':i.flag_bits} for i in archive.infolist()]
   (OUT/(env+'-'+kind+'-index.json')).write_text(json.dumps(index,indent=2)+'\n');indices.append({e['name']:e for e in index})
   case_archives.append({'environment':env,'kind':kind,'public_url':public,'archive_size':entry['size'],'archive_lfs_sha256_asserted_not_recomputed':entry['lfs']['oid']})
   print(env,kind,'metadata',reader.transferred,flush=True)
  candidates=sorted({n.split('/')[2] for n in indices[0] if len(n.split('/'))>=4 and n.split('/')[2].startswith('P')})
  eligible=[]
  for trajectory in candidates:
   prefix=f"{env}/Easy/{trajectory}/";names=[[prefix+f'image_left/{i:06d}_left.png' for i in range(case['pairs']+1)]]+[[prefix+f'flow/{i:06d}_{i+1:06d}_{suffix}.npy' for i in range(case['pairs'])] for suffix in ('flow','mask')]
   if all(all(n in index for n in nn) for nn,index in zip(names,indices)):eligible.append((trajectory,names))
  assert eligible,'no complete prefix available in '+env
  trajectory,names=eligible[0];cases.append({**case,'trajectory':trajectory,'recording_id':f'tartanair-v1/{env}/Easy/{trajectory}'})
  for item,index,nn in zip(case_archives,indices,names):
   members=[index[n] for n in nn];assert all(e['method'] in (0,8) and not e['flags']&1 for e in members)
   archives.append({**item,'selected_members':members})
  print('SELECTED_METADATA_ONLY',env,trajectory,flush=True)
p['cases']=cases
plan={**p,'status':'FROZEN_METADATA_COMPLETE_NO_DECODED_PAYLOADS','selection_protocol_sha256':sha(DECLARATION),'archives':archives,'metadata_transferred_bytes':transferred,'metadata_sha256':{str(f.relative_to(ROOT)):sha(f) for f in OUT.iterdir() if f.is_file()},'raw_root_relative':'data/tartanair-prospective-evaluation/run-001/source','prepared_root_relative':'data/tartanair-prospective-evaluation/run-001/prepared'}
with (OUT/'data-protocol.json').open('x') as stream:json.dump(plan,stream,indent=2);stream.write('\n')
print('METADATA_ONLY_PASS',len(archives),sum(len(a['selected_members']) for a in archives),'members')
