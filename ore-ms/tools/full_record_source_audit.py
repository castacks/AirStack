"""Independent full visual source/TAR/timestamp/PNG/HTTP receipt auditor."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import zlib

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/bounded-full-record'
COUNT=0


def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(x,label):
 global COUNT
 COUNT+=1
 if not x:raise ValueError(label)


def audit(output,allow_omitted_range_cache=False):
 p=read(ROOT/'experiments/full-record-intake-2026-11-04.json')
 need(sha(ROOT/'experiments/full-record-intake-2026-11-04.json')==sha(OUT/'intake-lock.json'),'intake lock')
 for rel,digest in p['source_sha256'].items():need(sha(ROOT/rel)==digest,'frozen source '+rel)
 prior=read(OUT/'prior-recovery.json')['camera_members'];summary=[];cache_checked=0;cache_omitted=0
 for c in p['cases']:
  condition=c['condition'];n=c['source_groups'];seq=c['sequence'];base=ROOT/c['source_base']
  a=read(OUT/'acquisition'/(condition+'.json'));manifest=read(OUT/'manifests'/(condition+'.json'))
  records=manifest['records'];need(len(records)==n and [r['stem'] for r in records]==[f'{i:06}' for i in range(n)],'complete original manifest axis')
  members={x['name']:x for x in read(base/'sync_data-tar-index.json') if x['regular']}
  times={(m,s):[int(v) for v in (base/'pilot/sync_data'/seq/m/('img_'+s+'_timestamp.txt')).read_text().split()] for m in ['rgb','thr'] for s in ['left','right']}
  native={(r['camera'],r['side'],r['stem']):r for r in a['camera_members']}
  need(len(native)==len(a['camera_members'])==4*n,'allfourunique source members')
  old=read(ROOT/c['prefix_manifest'])['records'];chunks=0;origins={};payload_bytes=0
  for m in ['rgb','thr']:
   for side in ['left','right']:
    need(len(times[(m,side)])==n and all(x<y for x,y in zip(times[(m,side)],times[(m,side)][1:])),'exact chronological clock count')
    for i,row in enumerate(records):
     stem=f'{i:06}';r=native[(m,side,stem)];entry=members[f'{seq}/{m}/img_{side}/{stem}.png'];rec=row['cameras'][m][side]
     need(r['tar_name']==entry['name'] and r['tar_offset_data']==entry['offset_data'] and r['bytes']==entry['size'],'original TAR identity')
     need(r['path']==rec['path'] and r['sha256']==rec['sha256'] and rec['recorded_time_ns']==times[(m,side)][i],'manifest lineage')
     data=(ROOT/r['path']).read_bytes();payload_bytes+=len(data)
     need(len(data)==entry['size'] and hashlib.sha256(data).hexdigest()==r['sha256'],'rawsource bytes/hash')
     need(data.startswith(bytes([137,80,78,71,13,10,26,10])),'PNG magic')
     off=8;tags=[]
     while off<len(data):
      size=int.from_bytes(data[off:off+4],'big');tag=data[off+4:off+8];payload=data[off+8:off+8+size]
      end=off+12+size;need(end<=len(data),'bounded PNG chunk')
      stored=int.from_bytes(data[off+8+size:end],'big')
      need(zlib.crc32(payload,zlib.crc32(tag))&0xffffffff==stored,'independent PNG CRC')
      if not tags:
       w,h,depth,color,*_=struct.unpack('>IIBBBBB',payload)
       need(tag==b'IHDR' and [h,w]==c['schema'][m]['shape'][:2] and depth==(8 if m=='rgb' else 16) and color==(2 if m=='rgb' else 0),'native PNG schema')
      tags.append(tag);chunks+=1;off=end
      if tag==b'IEND':need(size==0 and end==len(data),'complete PNG end');break
     need(tags[-1]==b'IEND','PNG end required')
     if r['path'] in prior:need(r['origin']=='prior_recovery' and prior[r['path']]==r['sha256'],'prior source byte identity')
     else:need(r['origin']=='current_http206_tar','new member provenance')
     origins[r['origin']]=origins.get(r['origin'],0)+1
     if i<101:
      for key in ['recorded_time_ns','sha256','dtype','shape']:need(rec[key]==old[i]['cameras'][m][side][key],'first101 source exact '+key)
  need(a['png_crc_chunks']==chunks and a['source_groups']==n and a['all_original_members'] is True,'producer coverage/CRC count')
  need(a['reference_payloads_extracted'] is False and a['full_current_archive_hash_verified'] is False,'bounded acquisition claims')
  initial=read(OUT/'receipts'/(condition+'-initial.json'))
  need(initial['status']==206 and initial['bytes']==1024 and initial['content_range']==f"bytes 0-1023/{c['archive_bytes']}",'initial current archive range')
  receipts=sorted((OUT/'compressed-ranges'/condition).glob('*.json'))
  for receipt in receipts:
   r=read(receipt);length=r['end']-r['start']+1
   need(r['status']==206 and r['archive_bytes']==c['archive_bytes'] and r['content_range']==f"bytes {r['start']}-{r['end']}/{c['archive_bytes']}",'exact HTTP206 receipt')
   binary=receipt.with_suffix('.bin')
   if binary.exists():need(binary.stat().st_size==length and sha(binary)==r['sha256'],'actual compressed range receipt bytes');cache_checked+=1
   elif allow_omitted_range_cache:cache_omitted+=1
   else:raise ValueError('missing range cache; explicit archive-omission mode required')
  summary.append({'condition':condition,'original_source_groups':n,'camera_pngs':4*n,'camera_payload_bytes':payload_bytes,'png_crc_chunks':chunks,'origins':origins,'http_range_receipts':len(receipts)})
  print('SOURCE AUDITED',condition,n,'groups',flush=True)
 result={'passed':True,'auditor_sha256':sha(Path(__file__)),'independent_checks':COUNT,'cases':summary,
 'compressed_ranges_rehashed':cache_checked,'compressed_range_payloads_explicitly_omitted':cache_omitted,
 'original_source_groups':sum(c['source_groups'] for c in p['cases']),'new_reference_values_accessed':False,
 'physical_sync_verified':False,'benchmark_execution_ready':False}
 with Path(output).open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))


if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--allow-omitted-range-cache',action='store_true');a=ap.parse_args();audit(a.output,a.allow_omitted_range_cache)
