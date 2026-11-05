"""EXP141 fixed complete visual-only intake with retained range/TAR/PNG evidence."""
import argparse
import bisect
import concurrent.futures
import csv
import hashlib
import json
import os
from pathlib import Path
import struct
import tarfile
import urllib.request
import zlib

from ms2_broader_intake import RangeReader

ROOT=Path(__file__).resolve().parents[1]
PLAN=ROOT/'experiments/full-record-intake-2026-11-04.json'
OUT=ROOT/'results/bounded-full-record'
SOURCE=OUT/'source-001'


def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')


def freeze():
 parent=read(ROOT/'experiments/whole-record-preflight-2026-11-04.json')
 prefix=read(ROOT/'experiments/droid-longer-prefix-2026-11-04.json')
 prior=read(ROOT/'experiments/ms2-broader-range-2026-11-02.json')
 cases=[];files=[Path(__file__),ROOT/'tools/ms2_broader_intake.py',
 ROOT/'experiments/whole-record-preflight-2026-11-04.json',ROOT/'results/whole-record-preflight/run-001/source-axis.csv',
 ROOT/'results/whole-record-preflight/source-001/broader-recovery-manifest.json',
 ROOT/'experiments/droid-longer-prefix-2026-11-04.json',ROOT/'experiments/ms2-broader-range-2026-11-02.json']
 files+=list((SOURCE/'wheels').glob('*.whl'))
 for c in parent['cases']:
  old=next(x for x in prior['cases'] if x['recording']==c['condition'])
  pc=next(x for x in prefix['runs'] if x['condition']==c['condition'])
  s=read(ROOT/pc['manifest']);n=len((ROOT/c['source_base']/'pilot/sync_data'/c['sequence']/'rgb/img_left_timestamp.txt').read_text().split())
  case={k:c[k] for k in ['condition','sequence','source_base']}
  case.update(source_groups=n,archive_url=old['archive_url'],archive_bytes=old['archive_bytes'],prefix_manifest=pc['manifest'],
    calibration=pc['calibration'],schema={m:{'shape':s['records'][0]['cameras'][m]['left']['shape'],
    'bit_depth':8 if m=='rgb' else 16,'color_type':2 if m=='rgb' else 0} for m in ['rgb','thr']})
  cases.append(case);files += [ROOT/pc['manifest'],ROOT/pc['calibration']]
  for item in c['restored']:files.append(ROOT/item['path'])
  for row in s['records']:
   for m in ['rgb','thr']:
    for side in ['left','right']:files.append(ROOT/row['cameras'][m][side]['path'])
 plan={'date':'2026-11-04 America/New_York','parent_commit':'5c472df43ff7cd29a216194bbc54e5788c1c5f5e',
 'cases':cases,'all_original_stems':True,'network_prefetch_workers':4,'source_sha256':{str(f.relative_to(ROOT)):sha(f) for f in files},
 'prior_packet_sha256':'be8efa7e58d0eb98359bff85c0a659efa172e04a211664d65765995e39c658f9',
 'intake_dependency':'indexed_bzip2==1.6.0 isolated CPython3.11wheel',
 'reference_payload_extraction':False,'protected_payload_access':False,'reference_scoring':False,
 'physical_sync_verified':False,'benchmark_execution_ready':False}
 write(PLAN,plan);write(OUT/'intake-lock.json',plan)
 write(ROOT/'experiments/recording-exposure-ledger-2026-11-04-full-record-engineering.json',{
 'scope':'prospective additional visual frame exposure in three already development-exposed records; no new recording assignments',
 'recordings':[{'recording':'ms2/'+c['sequence'],'original_stems':'000000 through '+f"{c['source_groups']-1:06}",'groups':c['source_groups']} for c in cases],
 'active_test':[],'active_validation':[],'reported_reserves_unchanged':True,'new_reference_values_accessed':False,'intake_plan_sha256':sha(PLAN)})
 print('FROZEN',sum(c['source_groups'] for c in cases),'source groups',len(plan['source_sha256']),'pins')


def verify():
 p=read(PLAN)
 if sha(PLAN)!=sha(OUT/'intake-lock.json'):raise ValueError('intake plan changed')
 for rel,h in p['source_sha256'].items():
  if sha(ROOT/rel)!=h:raise ValueError('changed input '+rel)
 return p


def raw_path(seq,m,side,stem):return OUT/'raw'/seq/m/('img_'+side)/(stem+'.png')


def recover():
 p=verify();known={};provenance=[]
 for c in p['cases']:
  for row in read(ROOT/c['prefix_manifest'])['records']:
   for m in ['rgb','thr']:
    for side in ['left','right']:
     rec=row['cameras'][m][side];dest=raw_path(c['sequence'],m,side,row['stem']);dest.parent.mkdir(parents=True,exist_ok=True)
     if dest.exists():
      if sha(dest)!=rec['sha256']:raise ValueError('conflicting resident prefix')
     else:os.link(ROOT/rec['path'],dest)
     known[str(dest.relative_to(ROOT))]=rec['sha256'];provenance.append({'path':str(dest.relative_to(ROOT)),'sha256':rec['sha256'],'source':'resident_prefix'})
 packet=Path('/root/ore-ms-recovery/ms2-broader-range/ms2-broader-range.tar.gz')
 if sha(packet)!=p['prior_packet_sha256']:raise ValueError('prior packet changed')
 registry=read(ROOT/'results/whole-record-preflight/source-001/broader-recovery-manifest.json')['member_sha256']
 expected={name:h for name,h in registry.items() if name.startswith('ore-ms/results/ms2-broader-range/run-001/source/') and '/img_' in name and name.endswith('.png')}
 found=0
 with tarfile.open(packet,'r:gz') as tar:
  for member in tar:
   if member.name not in expected:continue
   if not member.isfile():raise ValueError('nonregular prior camera member')
   data=tar.extractfile(member).read();h=hashlib.sha256(data).hexdigest()
   if h!=expected[member.name]:raise ValueError('prior member hash mismatch')
   rel=Path(member.name).relative_to('ore-ms/results/ms2-broader-range/run-001/source')
   if rel.parts[0] not in [c['sequence'] for c in p['cases']]:raise ValueError('undeclared sequence')
   dest=OUT/'raw'/rel;dest.parent.mkdir(parents=True,exist_ok=True)
   if dest.exists():
    if sha(dest)!=h:raise ValueError('prior/resident differs')
   else:dest.write_bytes(data)
   known[str(dest.relative_to(ROOT))]=h;found+=1
 if found!=3600:raise ValueError('incomplete prior camera recovery')
 write(OUT/'prior-recovery.json',{'camera_members':known,'prior_archive_members_verified':found,
 'resident_camera_members':len(provenance),'total_distinct_camera_members':len(known),'new_reference_values_accessed':False})
 print('RECOVERED',len(known),'distinct PNGs before remote intake',flush=True)


def png(data,schema):
 if data[:8]!=b'\x89PNG\r\n\x1a\n':raise ValueError('PNG signature')
 at=8;chunks=0;header=None;ended=False
 while at<len(data):
  if at+12>len(data):raise ValueError('truncated PNG chunk')
  n=struct.unpack('>I',data[at:at+4])[0];tag=data[at+4:at+8];end=at+12+n
  if end>len(data):raise ValueError('truncated PNG payload')
  block=data[at+8:at+8+n];expected=struct.unpack('>I',data[at+8+n:end])[0]
  if zlib.crc32(tag+block)&0xffffffff!=expected:raise ValueError('PNG CRC')
  if chunks==0:
   if tag!=b'IHDR' or n!=13:raise ValueError('PNG header')
   w,h,depth,color,compression,filtering,interlace=struct.unpack('>IIBBBBB',block)
   header=[h,w,depth,color]
   if [h,w]!=schema['shape'][:2] or depth!=schema['bit_depth'] or color!=schema['color_type']:raise ValueError('PNG schema')
  chunks+=1;at=end
  if tag==b'IEND':
   if n!=0 or at!=len(data):raise ValueError('invalid PNG end')
   ended=True;break
 if not ended:raise ValueError('missing PNG end')
 return chunks


def acquire(condition):
 import indexed_bzip2
 p=verify();c=next(c for c in p['cases'] if c['condition']==condition)
 recovery=read(OUT/'prior-recovery.json')['camera_members'];base=ROOT/c['source_base'];seq=c['sequence']
 index={m['name']:m for m in read(base/'sync_data-tar-index.json') if m['regular']}
 selected=[]
 for m in ['rgb','thr']:
  for side in ['left','right']:
   for i in range(c['source_groups']):
    stem=f'{i:06}';entry=index[f'{seq}/{m}/img_{side}/{stem}.png']
    selected.append((entry,m,side,stem))
 selected.sort(key=lambda x:x[0]['offset_data'])
 reader=RangeReader(c['archive_url'],c['archive_bytes'],OUT/'compressed-ranges'/condition)
 # Bind a current response to historical size and initial bytes; old indexes are
 # still checked at every actual decoded TAR member below.
 req=urllib.request.Request(c['archive_url'],headers={'Range':'bytes=0-1023','User-Agent':'ore-ms-whole-record-intake/1'})
 with urllib.request.urlopen(req,timeout=45) as r:
  data=r.read(1025)
  if r.status!=206 or r.headers.get('Content-Range')!=f"bytes 0-1023/{c['archive_bytes']}" or len(data)!=1024 or data[:3]!=b'BZh':raise ValueError('current archive response mismatch')
  write(OUT/'receipts'/(condition+'-initial.json'),{'status':r.status,'content_range':r.headers['Content-Range'],'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
 offsets={int(k):v for k,v in read(base/'archives/sync_data-block-offsets.json').items()}
 ordered=sorted((v,k) for k,v in offsets.items());uncompressed=[v for v,k in ordered];chunks=set()
 missing=[x for x in selected if str(raw_path(seq,x[1],x[2],x[3]).relative_to(ROOT)) not in recovery]
 for entry,*_ in missing:
  a=max(0,bisect.bisect_right(uncompressed,entry['offset_data']-512)-1);b=min(len(ordered)-1,bisect.bisect_left(uncompressed,entry['offset_data']+entry['size'])+1)
  chunks.update(range((ordered[a][1]//8)//reader.block,((ordered[b][1]+7)//8)//reader.block+1))
 def fetch(chunk):
  local=RangeReader(reader.url,reader.length,reader.cache);local.seek(chunk*local.block);local.read(min(local.block,local.length-local.tell()))
 chunks={v for v in chunks if v*reader.block<reader.length}
 print(condition,'prefetch',len(chunks),'chunks;',len(missing),'new camera members',flush=True)
 with concurrent.futures.ThreadPoolExecutor(max_workers=p['network_prefetch_workers']) as pool:
  for n,_ in enumerate(pool.map(fetch,sorted(chunks))):
   if n%100==0:print(condition,'prefetch',n,'/',len(chunks),flush=True)
 records=[];crc_chunks=0
 with indexed_bzip2.open(reader,parallelization=1) as stream:
  stream.set_block_offsets(offsets)
  for i,(entry,m,side,stem) in enumerate(selected):
   target=raw_path(seq,m,side,stem);relative=str(target.relative_to(ROOT));old=recovery.get(relative)
   if old is not None:
    data=target.read_bytes()
    if hashlib.sha256(data).hexdigest()!=old:raise ValueError('recovered source changed')
    origin='prior_recovery'
   else:
    stream.seek(entry['offset_data']-512);header=tarfile.TarInfo.frombuf(stream.read(512),'utf-8','strict')
    if header.name!=entry['name'] or header.size!=entry['size'] or not header.isfile():raise ValueError('current TAR header mismatch')
    data=stream.read(entry['size'])
    if len(data)!=entry['size']:raise ValueError('short camera member')
    origin='current_http206_tar'
   crc_chunks+=png(data,c['schema'][m]);digest=hashlib.sha256(data).hexdigest()
   if target.exists():
    if sha(target)!=digest:raise ValueError('conflicting current source')
   else:target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
   records.append({'path':relative,'sha256':digest,'bytes':len(data),'camera':m,'side':side,'stem':stem,'origin':origin,'tar_name':entry['name'],'tar_offset_data':entry['offset_data']})
   if i%1000==0:print(condition,'members',i,'/',len(selected),flush=True)
 write(OUT/'acquisition'/(condition+'.json'),{'condition':condition,'camera_members':records,'png_crc_chunks':crc_chunks,
 'source_groups':c['source_groups'],'all_original_members':len(records)==4*c['source_groups'],'full_current_archive_hash_verified':False,
 'current_selected_tar_headers_verified':True,'reference_payloads_extracted':False})
 print('COMPLETE',condition,len(records),'PNGs;',crc_chunks,'CRC chunks',flush=True)


if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('command',choices=['freeze','recover','acquire','verify']);ap.add_argument('--condition',choices=['day','night','rain']);a=ap.parse_args()
 if a.command=='acquire':acquire(a.condition)
 elif a.command=='verify':print('verified',len(verify()['source_sha256']),'pins')
 else:globals()[a.command]()
