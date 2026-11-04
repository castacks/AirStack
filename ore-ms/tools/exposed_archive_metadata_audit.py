"""Independently parse bounded ZIP indexes with the standard-library ZIP reader."""
import hashlib,io,json,struct,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/exposed-data-adequacy/archive-metadata-001'
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 checks=0
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 plan=read(OUT/'plan.json');lock=read(OUT/'intake-lock.json');summary=read(OUT/'summary.json')
 check(lock['plan_sha256']==sha(OUT/'plan.json'),'plan lock')
 check(lock['source_sha256']==sha(ROOT/'tools/exposed_archive_metadata.py'),'source lock')
 for path,digest in lock['sha256'].items():check(sha(ROOT/path)==digest,path)
 for path,digest in read(OUT/'source-lock.json').items():check(sha(ROOT/path)==digest,'before-index source')
 check(summary['member_payload_read'] is False and summary['full_archive_sha_verified'] is False,'scope flags')
 check({r['archive'] for r in summary['reports']}=={r['Name'] for r in plan['archives']} and len(summary['reports'])==5,'five selected archives')
 total=transferred=0
 for report in summary['reports']:
  directory=OUT/report['archive'];metadata=next(x for x in plan['archives'] if x['Name']==report['archive'])
  tail=(directory/'footer.bin').read_bytes();pos=tail.rfind(b'PK\x05\x06');e=struct.unpack_from('<4s4H2LH',tail,pos)
  check(pos+22+e[7]==len(tail) and e[1]==e[2]==0,'EOCD metadata')
  count,size,offset=e[4],e[5],e[6]
  if count==65535 or size==4294967295 or offset==4294967295:
   locator=struct.unpack_from('<4sLQL',tail,pos-20);check(locator[0]==b'PK\x06\x07','ZIP64 locator')
   z=struct.unpack('<4sQ2H2L4Q',(directory/'zip64-eocd.bin').read_bytes())
   check(z[0]==b'PK\x06\x06' and z[4]==z[5]==0 and z[6]==z[7],'ZIP64 metadata');count,size,offset=z[7:10]
  else:check(e[3]==e[4],'single disk counts')
  check(0<=offset<offset+size<=metadata['Size'],'directory original bounds')
  for record in report['ranges']:
   p=ROOT/record['file'];check(p.stat().st_size==record['count'] and sha(p)==record['sha256'],'range length/hash');transferred+=record['count']
  check(report['member_payload_read'] is False,'no member payload')
  if size>plan['central_directory_max_bytes_per_archive']:
   check(report['status']=='DIRECTORY_EXCEEDS_FIXED_CAP' and not (directory/'central-directory.bin').exists(),'fixed cap');continue
  cd=(directory/'central-directory.bin').read_bytes();check(len(cd)==size and report['directory_bytes']==size,'directory size')
  # A metadata-only wrapper points the independent reader at the original CD.
  # No original local header or compressed member payload exists in this wrapper.
  wrapper=cd
  if count>=65535:
   wrapper+=struct.pack('<4sQ2H2L4Q',b'PK\x06\x06',44,45,45,0,0,count,count,len(cd),0)
   wrapper+=struct.pack('<4sLQL',b'PK\x06\x07',0,len(cd),1)
  wrapper+=struct.pack('<4s4H2LH',b'PK\x05\x06',0,0,min(count,65535),min(count,65535),len(cd),0,0)
  with zipfile.ZipFile(io.BytesIO(wrapper)) as archive:entries=archive.infolist()
  saved=read(directory/'members.json');check(len(entries)==len(saved)==count==report['member_count'],'independent member count')
  for original,entry in zip(entries,saved):
   check(original.filename==entry['name'] and original.CRC==entry['crc32'],'independent filename and CRC')
   if entry['uncompressed_bytes']!=4294967295:check(original.file_size==entry['uncompressed_bytes'],'member size field')
   if entry['compressed_bytes']!=4294967295:check(original.compress_size==entry['compressed_bytes'],'compressed size field')
  quality_terms=['rtk','bestpos','bestvel','navsat','fix_status','solution_status','inscov','inspva','gpgga','gprmc','covariance']
  check(report['quality_name_leads']==[x.filename for x in entries if any(t in x.filename.lower() for t in quality_terms)],'quality filename filter')
  check(report['gnss_context_names']==[x.filename for x in entries if any(t in x.filename.lower() for t in ['novatel','gps','gnss','gq7'])],'GNSS context filter')
  total+=len(entries)
 result={'pass':True,'checks':checks,'indexed_archives':5,'member_names':total,'range_bytes':transferred,
  'member_payload_read':False,'full_archive_sha_verified':False,'independent_reader':'Python zipfile; directory-only synthetic wrapper',
  'summary_sha256':sha(OUT/'summary.json'),'source_sha256':sha(Path(__file__))}
 with (OUT/'audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print('PASS',checks,'metadata checks;',total,'member names;',transferred,'range bytes')
if __name__=='__main__':main()
