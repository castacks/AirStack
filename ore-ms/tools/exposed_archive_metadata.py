"""Bounded ZIP directory metadata only for five already exposed TD3 archives."""
import hashlib,json,struct,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/exposed-data-adequacy/archive-metadata-001'
CONFIG=['rclone','--config','/root/.config/rclone/ore-ms-recovery.conf','--box-root-folder-id','428559630945']
def sha(data):return hashlib.sha256(data).hexdigest()
def save(path,value):
 with path.open('x') as f:json.dump(value,f,indent=2);f.write('\n')
def run():
 plan=json.loads((OUT/'plan.json').read_text());ledger=json.loads((ROOT/'experiments/recording-exposure-ledger-2026-10-31.json').read_text());reports=[]
 for entry in plan['archives']:
  name=entry['Name'];assert any(name in r['aliases'] and r['exposure_status']=='development_exposed' for r in ledger['records'])
  size=entry['Size'];dest=OUT/name;dest.mkdir(exist_ok=False);ranges=[]
  def fetch(offset,count,label):
   result=subprocess.run(CONFIG+['cat','ore_ms:'+name,'--offset',str(offset),'--count',str(count)],capture_output=True,check=True)
   data=result.stdout;assert len(data)==count,(name,label,len(data),count)
   p=dest/(label+'.bin')
   with p.open('xb') as f:f.write(data)
   ranges.append({'offset':offset,'count':count,'sha256':sha(data),'file':str(p.relative_to(ROOT))});return data
  tail_size=min(plan['tail_max_bytes'],size);tail=fetch(size-tail_size,tail_size,'footer');idx=tail.rfind(b'PK\x05\x06');assert idx>=0
  e=struct.unpack_from('<4s4H2LH',tail,idx);assert idx+22+e[7]==len(tail) and e[1]==e[2]==0
  total,cd_size,cd_offset=e[4],e[5],e[6];zip64=False
  if total==65535 or cd_size==4294967295 or cd_offset==4294967295:
   loc=struct.unpack_from('<4sLQL',tail,idx-20);assert loc[0]==b'PK\x06\x07' and loc[1]==0 and loc[3]==1
   z=struct.unpack('<4sQ2H2L4Q',fetch(loc[2],56,'zip64-eocd'))
   assert z[0]==b'PK\x06\x06' and z[4]==z[5]==0 and z[6]==z[7]
   total,cd_size,cd_offset=z[7],z[8],z[9];zip64=True
  assert 0<=cd_offset<cd_offset+cd_size<=size
  if cd_size>plan['central_directory_max_bytes_per_archive']:
   report={'archive':name,'status':'DIRECTORY_EXCEEDS_FIXED_CAP','directory_bytes':cd_size,'member_payload_read':False,'ranges':ranges}
  else:
   cd=fetch(cd_offset,cd_size,'central-directory');position=0;members=[]
   while position<len(cd):
    h=struct.unpack_from('<4s6H3L5H2L',cd,position);assert h[0]==b'PK\x01\x02'
    n,x,c=h[10:13];filename=cd[position+46:position+46+n].decode('utf-8' if h[3]&2048 else 'cp437')
    members.append({'name':filename,'uncompressed_bytes':h[9],'compressed_bytes':h[8],'crc32':h[7]});position+=46+n+x+c
   assert position==len(cd) and len(members)==total
   terms=['rtk','bestpos','bestvel','navsat','fix_status','solution_status','inscov','inspva','gpgga','gprmc','covariance']
   quality=[m['name'] for m in members if any(t in m['name'].lower() for t in terms)]
   context=[m['name'] for m in members if any(t in m['name'].lower() for t in ['novatel','gps','gnss','gq7'])]
   save(dest/'members.json',members)
   report={'archive':name,'status':'INDEXED_METADATA_ONLY','zip64':zip64,'directory_bytes':cd_size,'member_count':total,
    'quality_name_leads':quality,'gnss_context_names':context,'member_payload_read':False,'ranges':ranges}
  save(dest/'report.json',report);reports.append(report);print(name,report['status'],report.get('member_count'),flush=True)
 save(OUT/'summary.json',{'reports':reports,'scope':plan['scope'],'member_payload_read':False,'full_archive_sha_verified':False})
 save(OUT/'intake-lock.json',{'source_sha256':sha(Path(__file__).read_bytes()),'plan_sha256':sha((OUT/'plan.json').read_bytes()),
    'sha256':{str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in OUT.rglob('*') if p.is_file()}})
if __name__=='__main__':run()
