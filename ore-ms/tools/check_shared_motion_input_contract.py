"""Bounded metadata review. No images, predictions or reference values read."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path


def transpose(A):return [list(r) for r in zip(*A)]
def matmul(A,B):return [[sum(a*b for a,b in zip(row,col)) for col in zip(*B)] for row in A]
def det(A):return sum(A[0][j]*(A[1][(j+1)%3]*A[2][(j+2)%3]-A[1][(j+2)%3]*A[2][(j+1)%3]) for j in range(3))
def summary(a):
    s=sorted(a); n=len(s)
    return {'min_ns':s[0],'median_ns':s[n//2] if n%2 else (s[n//2-1]+s[n//2])/2,'max_ns':s[-1]}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);args=ap.parse_args()
    root=Path(__file__).resolve().parents[1];protocol=Path(args.protocol);pin=json.loads(protocol.read_text())
    for key in ['source_sha256','input_sha256']:
        for rel,h in pin[key].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==h,rel
    source=json.loads((root/pin['manifest']).read_text());paired=json.loads((root/pin['paired']).read_text());cal=json.loads((root/pin['calibration']).read_text())
    assert source['sequence']==pin['sequence']==paired['source_sequence']
    assert paired['source_manifest_sha256']==hashlib.sha256((root/pin['manifest']).read_bytes()).hexdigest()
    rows=source['records'][:pin['frames']]; pairs=paired['pairs'][:pin['frames']]
    assert [r['stem'] for r in rows]==pin['stems']==[p['source_stem'] for p in pairs]
    clocks={m+'_'+side:[] for m in ['rgb','thr'] for side in ['left','right']}
    image_availability=[];frames=[]; geometry=[]
    for row,pair in zip(rows,pairs):
        entry={'stem':row['stem'],'times_ns':{}}
        for mod in ['rgb','thr']:
            camera=row['cameras'][mod];K=cal['K_'+mod+'L']['values'];right_t=[v[0] for v in cal['T_'+mod+'R']['values']];right_R=cal['R_'+mod+'R']['values']
            assert camera['K']==K and K[0][0]>0 and K[1][1]>0 and K[2]==[0,0,1]
            assert right_R==[[1,0,0],[0,1,0],[0,0,1]] and right_t[0]<0 and right_t[1:]==[0,0]
            assert abs(camera['baseline_m']+right_t[0]/1000)<1e-12
            for side in ['left','right']:
                item=camera[side];p=pair[('thermal' if mod=='thr' else mod)+'_'+side];time=item['recorded_time_ns']
                assert type(time) is int and p['time_ns']==time and p['path']==item['path'] and p['source_stem']==row['stem']
                clocks[mod+'_'+side].append(time);entry['times_ns'][mod+'_'+side]=time
                image_availability.append({'stem':row['stem'],'camera':mod+'_'+side,'present':(root/item['path']).is_file()})
        entry['cross_left_offset_ns']=entry['times_ns']['thr_left']-entry['times_ns']['rgb_left']
        entry['rgb_stereo_offset_ns']=entry['times_ns']['rgb_right']-entry['times_ns']['rgb_left']
        entry['thermal_stereo_offset_ns']=entry['times_ns']['thr_right']-entry['times_ns']['thr_left']
        frames.append(entry)
    for k,values in clocks.items():assert all(b>a for a,b in zip(values,values[1:])),k
    Rrgb=cal['R_nir2rgb']['values'];Rthr=cal['R_nir2thr']['values'];Q=matmul(Rthr,transpose(Rrgb))
    t_rgb=[v[0]/1000 for v in cal['T_nir2rgb']['values']];t_thr=[v[0]/1000 for v in cal['T_nir2thr']['values']]
    t=[a-sum(b*c for b,c in zip(row,t_rgb)) for a,row in zip(t_thr,Q)]
    gram=matmul(Q,transpose(Q));orth=max(abs(gram[i][j]-int(i==j)) for i in range(3) for j in range(3));determinant=det(Q)
    spatial=orth<pin['rotation_tolerance'] and abs(determinant-1)<pin['rotation_tolerance']
    for mod in ['rgb','thr']:geometry.append({'modality':mod,'K':cal['K_'+mod+'L']['values'],'baseline_m':rows[0]['cameras'][mod]['baseline_m'],'shape':rows[0]['cameras'][mod]['left']['shape']})
    intervals=[]
    for i in range(1,len(rows)):
        rgb=[clocks['rgb_left'][i-1],clocks['rgb_left'][i]];thr=[clocks['thr_left'][i-1],clocks['thr_left'][i]]
        intervals.append({'source_stems':[rows[i-1]['stem'],rows[i]['stem']],'rgb_left_endpoints_ns':rgb,'thermal_left_endpoints_ns':thr,'exact_common_endpoints':rgb==thr,'duration_difference_ns':(thr[1]-thr[0])-(rgb[1]-rgb[0])})
    exact=sum(x['exact_common_endpoints'] for x in intervals)
    result={'timestamp_utc':datetime.now(timezone.utc).isoformat(),'protocol_sha256':hashlib.sha256(protocol.read_bytes()).hexdigest(),'sequence':pin['sequence'],'frames':frames,'intervals':intervals,'geometry':geometry,'thermal_from_rgb_optical':{'R':Q,'t_m':t,'rotation_orthogonality_max_abs':orth,'rotation_determinant':determinant,'scope':'Nominal published rectified calibration, not newly measured mounting.'},'summaries':{'cross_left_offsets':summary([x['cross_left_offset_ns'] for x in frames]),'rgb_stereo_offsets':summary([x['rgb_stereo_offset_ns'] for x in frames]),'thermal_stereo_offsets':summary([x['thermal_stereo_offset_ns'] for x in frames]),'interval_duration_differences':summary([x['duration_difference_ns'] for x in intervals])},'exact_common_intervals':exact,'attempted_intervals':len(intervals),'nominal_spatial_geometry_compatible':spatial,'physical_frame_or_clock_verified':paired['physical_frame_or_clock_verified'],'image_availability':image_availability,'decision':'STOP_THIS_STRICT_SHARED_INTERVAL_CONFIGURATION' if exact!=len(intervals) or not spatial or not paired['physical_frame_or_clock_verified'] else 'REVIEW_MEASUREMENT_BINDING_NEXT','scope':'Metadata only. Recorded-time mismatch does not prove physical exposure mismatch. No timestamp correction, new pairing, pixel/feature/reference reads or training.'}
    dest=Path(args.output);dest.mkdir(parents=True,exist_ok=False);(dest/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['decision','exact_common_intervals','attempted_intervals','nominal_spatial_geometry_compatible','summaries']}))
if __name__=='__main__':main()
