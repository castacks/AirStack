"""Metadata only: enumerate pinned archives and exact declared ZIP entries."""
import datetime
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile
from Scripts.Setup.remote_zip import HTTPRangeFile

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/prospective-objective-evaluation/run-001'
DECLARATION=ROOT/'docs/research/prospective-objective-selection-2026-11-02.json'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads(DECLARATION.read_text())
for rel,digest in p['protected_pins'].items():assert sha(ROOT/rel)==digest,rel
for pin in p['models'].values():assert sha(ROOT/pin['path'])==pin['sha256']
OUT.mkdir(parents=True,exist_ok=False)
(OUT/'selection-protocol.json').write_bytes(DECLARATION.read_bytes())
archives=[];transferred=0
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
  for kind,archive_name in [('image_left','image_left.zip'),('flow','flow.zip')]:
   entry=next(e for e in tree if e['path']==f'{env}/Easy/{archive_name}')
   public=f"https://huggingface.co/datasets/theairlabcmu/tartanair/resolve/{p['source_revision']}/{entry['path']}"
   reader=HTTPRangeFile(public,entry['size'],record=record,budget=p['metadata_budget_bytes'])
   with zipfile.ZipFile(reader) as archive:
    index=[{'name':i.filename,'size':i.file_size,'compressed_size':i.compress_size,'crc32':i.CRC,'header_offset':i.header_offset,'method':i.compress_type,'flags':i.flag_bits} for i in archive.infolist()]
    (OUT/(env+'-'+kind+'-index.json')).write_text(json.dumps(index,indent=2)+'\n')
    prefix=f"{env}/Easy/{case['trajectory']}/";names=([prefix+f'image_left/{i:06d}_left.png' for i in range(case['pairs']+1)] if kind=='image_left' else [prefix+f'flow/{i:06d}_{i+1:06d}_{suffix}.npy' for i in range(case['pairs']) for suffix in ('flow','mask')])
    members=[]
    for name in names:
     found=next(e for e in index if e['name']==name);assert found['method'] in (0,8) and not found['flags']&1;members.append(found)
   archives.append({'environment':env,'kind':kind,'public_url':public,'archive_size':entry['size'],'archive_lfs_sha256_asserted_not_recomputed':entry['lfs']['oid'],'selected_members':members})
   print(env,kind,len(members),'metadata',reader.transferred,flush=True)
plan={**p,'status':'FROZEN_METADATA_COMPLETE_NO_DECODED_PAYLOADS','selection_protocol_sha256':sha(DECLARATION),'archives':archives,'metadata_transferred_bytes':transferred,'metadata_sha256':{str(f.relative_to(ROOT)):sha(f) for f in OUT.iterdir() if f.is_file()},'raw_root_relative':'data/tartanair-prospective-evaluation/run-001/source','prepared_root_relative':'data/tartanair-prospective-evaluation/run-001/prepared'}
with (OUT/'data-protocol.json').open('x') as stream:json.dump(plan,stream,indent=2);stream.write('\n')
print('METADATA_ONLY_PASS',len(archives),sum(len(a['selected_members']) for a in archives),'members')
