"""EXP151 day-only bounded acquisition, inherited EXP147 reader/schema."""
import bisect,threading,tarfile,concurrent.futures
from cross_recording_stereo_depth import ROOT,OUT,PARAMS,read,write,sha,verify
from native_depth_proxy import BudgetReader,png_schema
def acquire():
 import indexed_bzip2
 p=verify();cache=OUT/'compressed-ranges';offsets={int(k):v for k,v in read(ROOT/p['block_offsets']).items()};ordered=sorted((v,k) for k,v in offsets.items());positions=[v for v,k in ordered];chunks=set();shared=(set(),threading.Lock());reader=BudgetReader(p['archive_url'],p['archive_bytes'],cache,shared)
 for m in p['members']:
  a=max(0,bisect.bisect_right(positions,m['offset_data']-512)-1);b=min(len(ordered)-1,bisect.bisect_left(positions,m['offset_data']+m['size'])+1)
  chunks.update(range((ordered[a][1]//8)//reader.block,((ordered[b][1]+7)//8)//reader.block+1))
 if len(chunks)*reader.block>PARAMS['compressed_budget_bytes']:raise ValueError('prefetch budget')
 def fetch(i):
  r=BudgetReader(reader.url,reader.length,cache,shared);r.seek(i*r.block);r.read(min(r.block,r.length-r.tell()))
 print('Prefetch',len(chunks),'compressed chunks',flush=True)
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(fetch,sorted(chunks)))
 records=[]
 with indexed_bzip2.open(reader,parallelization=1) as stream:
  stream.set_block_offsets(offsets)
  for m in sorted(p['members'],key=lambda m:m['offset_data']):
   stream.seek(m['offset_data']-512);header=stream.read(512);info=tarfile.TarInfo.frombuf(header,'utf-8','strict')
   if not (info.isfile() and info.name==m['name'] and info.size==m['size']):raise ValueError('TAR exact member')
   data=stream.read(m['size']);png_schema(data,m['shape']);dest=OUT/'labels'/m['name'];dest.parent.mkdir(parents=True,exist_ok=True)
   if dest.exists():
    if dest.read_bytes()!=data:raise ValueError('conflicting source label')
   else:dest.write_bytes(data)
   old=ROOT/p['source_base']/'pilot/proj_depth'/m['name'];overlap=None
   if old.exists():
    overlap=old.read_bytes()==data
    if not overlap:raise ValueError('historical label overlap changed')
   hp=OUT/'tar-headers'/m['modality']/(f"{m['index']:06}.bin");hp.parent.mkdir(parents=True,exist_ok=True);hp.write_bytes(header)
   records.append(dict(name=m['name'],path=str(dest.relative_to(ROOT)),sha256=sha(dest),bytes=len(data),header_path=str(hp.relative_to(ROOT)),header_sha256=sha(hp),historical_overlap_exact=overlap))
   print('Acquired',m['modality'],m['index'],flush=True)
 ranges=sorted(cache.glob('*.bin'));report=dict(members=records,member_count=len(records),compressed_range_files=len(ranges),compressed_bytes=sum(f.stat().st_size for f in ranges),current_full_archive_sha_verified=False,historical_full_archive_sha256=p['historical_full_archive_sha256'],range_metadata_sha256={str(f.relative_to(ROOT)):sha(f) for f in cache.glob('*.json')})
 write(OUT/'acquisition.json',report);verify();print('ACQUIRED',len(records),'selected depth maps')
