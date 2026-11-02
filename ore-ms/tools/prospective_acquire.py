"""Fetch only predeclared members; verify source central entries and ZIP CRC."""
from pathlib import Path
import datetime,hashlib,json,sys,zipfile
sys.path.insert(0,'/root/AirStack/ore-ms/MAC-VO-main')
from Scripts.Setup.remote_zip import HTTPRangeFile
ROOT=Path('/root/AirStack/ore-ms');OUT=ROOT/'results/prospective-objective-evaluation/run-001';plan=json.loads((OUT/'data-protocol.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for rel,d in plan['protected_pins'].items():assert sha(ROOT/rel)==d,rel
for rel,d in plan['metadata_sha256'].items():assert sha(ROOT/rel)==d,rel
records=[];total=0
with (OUT/'payload-range-requests.jsonl').open('x') as log:
 def record(row):
  global total
  total+=row['bytes'];assert total<=plan['payload_budget_bytes']
  log.write(json.dumps(row)+'\n');log.flush()
 try:
  for archive in plan['archives']:
   reader=HTTPRangeFile(archive['public_url'],archive['archive_size'],record=record)
   with zipfile.ZipFile(reader) as z:
    for selected in archive['selected_members']:
     info=z.getinfo(selected['name']);assert (info.CRC,info.file_size,info.compress_size,info.header_offset,info.compress_type)==tuple(selected[k] for k in ('crc32','size','compressed_size','header_offset','method'))
     body=z.read(info);assert len(body)==selected['size']
     path=ROOT/plan['raw_root_relative']/selected['name'];assert path.resolve().is_relative_to((ROOT/plan['raw_root_relative']).resolve());path.parent.mkdir(parents=True,exist_ok=True)
     with path.open('xb') as f:f.write(body)
     records.append({'archive':archive['public_url'],'member':selected['name'],'path':str(path.relative_to(ROOT)),'bytes':len(body),'sha256':hashlib.sha256(body).hexdigest(),'zip_crc32_verified':info.CRC})
   print(archive['environment'],archive['kind'],'members',len(archive['selected_members']),'transfer',reader.transferred,flush=True)
  assert len(records)==98
  result={'status':'PASS_DECLARED_MEMBER_ACQUISITION','created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'protocol_sha256':sha(OUT/'data-protocol.json'),'transferred_bytes':total,'member_count':len(records),'members':records,'whole_archive_hash_recomputed':False,'protected_project_reserves_accessed':False}
 except BaseException as error:
  result={'status':'FAILED','error_type':type(error).__name__,'error':str(error),'transferred_bytes':total,'members':records};raise
 finally:(OUT/'acquisition.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='members'},indent=2))
