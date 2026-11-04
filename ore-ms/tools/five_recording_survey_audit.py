"""Independent Decimal association and complete fifteen-third sampling audit."""
import bisect,collections,json,zlib
from decimal import Decimal,InvalidOperation
from five_recording_survey import ROOT,OUT,PLAN,INDEX,STREAMS,CALIB,read,write,sha,verify


def independent_pairs(streams,S,C):
 ts={k:[r['time_ns'] for r in v] for k,v in streams.items()};used={k:-1 for k in streams};result=[]
 def choose(key,center,tolerance,lower=None,upper=None):
  low=center-tolerance if lower is None else max(center-tolerance,lower)
  high=center+tolerance if upper is None else min(center+tolerance,upper)
  valid=range(max(used[key]+1,bisect.bisect_left(ts[key],low)),bisect.bisect_right(ts[key],high))
  return min(valid,key=lambda j:(abs(ts[key][j]-center),ts[key][j],j),default=None)
 for L in streams['rgb_left']:
  r=choose('rgb_right',L['time_ns'],S);t=choose('thermal_left',L['time_ns'],C)
  if r is None or t is None:continue
  R=streams['rgb_right'][r];T=streams['thermal_left'][t];low=max(L['time_ns'],R['time_ns'])-C;high=min(L['time_ns'],R['time_ns'])+C
  if not low<=T['time_ns']<=high:continue
  u=choose('thermal_right',T['time_ns'],S,low,high)
  if u is None:continue
  U=streams['thermal_right'][u]
  result.append({'rgb_left':L,'rgb_right':R,'thermal_left':T,'thermal_right':U,
   'offsets_ns':{'rgb_stereo':R['time_ns']-L['time_ns'],'thermal_stereo':U['time_ns']-T['time_ns'],
       'thermal_left_minus_rgb_left':T['time_ns']-L['time_ns']}})
  used.update(rgb_right=r,thermal_left=t,thermal_right=u)
 return result


def audit():
 plan=verify();lock=read(OUT/'metadata-lock.json');checks=0
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 check(lock['plan_sha256']==sha(PLAN),'plan lock')
 for path,digest in lock['sha256'].items():check(sha(ROOT/path)==digest,path)
 summary=read(OUT/'metadata-summary.json');check(len(summary['recordings'])==5 and summary['total_thirds']==15,'all recordings/thirds')
 qualified=0;allthirds=[]
 for entry in summary['recordings']:
  report=read(ROOT/entry['report']);clip=report['archive'][:-4]
  inventory=read(INDEX/report['archive']/'members.json');members={m['name'].split('/',1)[1]:m for m in inventory if '/' in m['name']}
  expected=[f'sensors/{s}/timestamps.txt' for s in list(STREAMS.values())+['thermal_left','thermal_right']]+[f'sensors/{s}/data.yaml' for s in CALIB]
  missing=[p for p in expected if p not in members];check(missing==report['missing_metadata'],'missing source metadata')
  for path,record in report['files'].items():
   data=(ROOT/path).read_bytes();item=next(x for x in inventory if x['name']==record['member'])
   check(len(data)==record['bytes']==item['uncompressed_bytes'] and sha(ROOT/path)==record['sha256'],'source bytes/SHA')
   check(zlib.crc32(data)&0xffffffff==record['zip_crc32']==item['crc32'],'independent member CRC')
  streams={}
  for key,name in STREAMS.items():
   p=ROOT/entry['report'];source=p.parent/'source/sensors'/name/'timestamps.txt';records=[];seen=set();previous=None
   if not source.exists():
    check(report['streams'][key]==[] and report['stream_stats'][key]=={'missing_metadata':True},'missing timeline');streams[key]=[];continue
   stats={'rows':0,'nonfinite':0,'missing_image':0,'duplicate_ns':0,'out_of_order':0,'unparseable':0}
   for i,line in enumerate(source.read_text().splitlines()):
    stats['rows']+=1
    try:d=Decimal(line.strip())
    except InvalidOperation:stats['unparseable']+=1;continue
    if not d.is_finite():stats['nonfinite']+=1;continue
    ns=int((d*Decimal('1000000000')).to_integral_value());image=f'sensors/{name}/{i:08d}.png'
    if image not in members:stats['missing_image']+=1;continue
    if ns in seen:stats['duplicate_ns']+=1;continue
    if previous is not None and ns<previous:stats['out_of_order']+=1
    previous=ns;seen.add(ns);records.append({'index':i,'time_ns':ns,'path':image})
   records=sorted(records,key=lambda r:(r['time_ns'],r['index']));stats['retained']=len(records)
   check(report['streams'][key]==records and report['stream_stats'][key]==stats,'source timeline/index semantics');streams[key]=records
  pairs=independent_pairs(streams,plan['stereo_max_ns'],plan['cross_max_ns'])
  check(pairs==report['paired'] and len(pairs)==entry['paired_groups']==report['paired_groups'],'greedy pairing exact')
  for pair in pairs:
   R0,R1,T0,T1=[pair[k]['time_ns'] for k in STREAMS]
   check(abs(R1-R0)<=2000000 and abs(T1-T0)<=2000000 and max(abs(x-y) for x in [R0,R1] for y in [T0,T1])<=30000000,'four endpoint bounds')
  for key in STREAMS:check(len({r[key]['index'] for r in pairs})==len(pairs),'no acquisition reuse')
  rgb=streams['rgb_left'];first=rgb[0]['time_ns'] if rgb else None;end=rgb[-1]['time_ns']+1 if rgb else None
  check(len(report['windows'])==3,'all thirds')
  for third,w in enumerate(report['windows']):
   candidates=[]
   if rgb:
    lo=first+(end-first)*third//3;hi=first+(end-first)*(third+1)//3
    check(w['rgb_elapsed_bounds_ns']==[lo,hi],'elapsed-time thirds')
    for j in range(len(pairs)-99):
     rows=pairs[j:j+100]
     if not all(lo<=row['rgb_left']['time_ns']<hi for row in rows):continue
     valid=True
     for a,b in zip(rows,rows[1:]):
      if not all(0<b[key]['time_ns']-a[key]['time_ns']<=150000000 for key in STREAMS):valid=False;break
      if max(a['rgb_left']['time_ns'],a['thermal_left']['time_ns'])>=min(b['rgb_left']['time_ns'],b['thermal_left']['time_ns']):valid=False;break
     if valid:candidates.append(j)
   status='MISSING_REQUIRED_METADATA' if missing else 'NO_RGB_ACQUISITIONS' if not rgb else 'QUALIFIED' if candidates else 'NO_100_GROUP_WINDOW_UNDER_FROZEN_RULE'
   check(w['status']==status and w['third']==third,'third classification')
   if status=='QUALIFIED':
    check(w['manifest_start']==min(candidates) and w['rows']==pairs[min(candidates):min(candidates)+100],'first eligible, no outcome selection');qualified+=1
   else:check(w['rows']==[] and w.get('manifest_start') is None,'ineligible not substituted')
   allthirds.append({'recording_id':report['recording_id'],'third':third,'status':status})
  check(entry['thirds']==[w['status'] for w in report['windows']],'summary third states')
  check(report['image_decoded'] is False and report['native_executed'] is False,'metadata only')
 check(summary['qualified_windows']==qualified and not summary['image_decoded'] and not summary['native_executed'],'summary scope')
 write(OUT/'preflight-audit.json',{'pass':True,'checks':checks,'recordings':5,'thirds':allthirds,'qualified_windows':qualified,
   'source_pins':len(plan['input_source_sha256']),'image_decoded':False,'native_executed':False,
   'metadata_summary_sha256':sha(OUT/'metadata-summary.json')})
 print('PASS',checks,'checks;',qualified,'qualified windows out of15')

if __name__=='__main__':audit()
