"""Descriptive ranking of saved MAC RGB/thermal uncertainty against LK disagreement."""
import argparse,hashlib,json
from datetime import datetime,timezone
from pathlib import Path
import numpy as np

POPULATIONS={'primary':('lk_valid','lk_valid'),'tight':('lk_tight','lk_tight'),
             'texture':('lk_texture','lk_texture'),'all_uncertainty':('all','lk_valid')}

def sha(path):return hashlib.file_digest(Path(path).open('rb'),'sha256').hexdigest()
def save(path,data):Path(path).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
def choice(first,second):
    if first is None or second is None or first==second:return None
    return 'rgb' if first<second else 'thermal'

def rank(values):
    values=np.asarray(values,float);order=np.argsort(values,kind='stable');result=np.empty(len(values),float);start=0
    while start<len(values):
        stop=start+1
        while stop<len(values) and values[order[stop]]==values[order[start]]:stop+=1
        result[order[start:stop]]=(start+stop-1)/2.;start=stop
    return result

def association(x,y):
    if len(x)<3:return None
    x,y=rank(x),rank(y)
    if not np.std(x) or not np.std(y):return None
    return float(np.corrcoef(x,y)[0,1])

def summarize(rows,units='angular'):
    evaluated=[r for r in rows if r['eligible'] and r['winner'] is not None and r['choice_'+units] is not None]
    costs=[r for r in rows if r['eligible'] and r['error_rgb_rad'] is not None and r['error_thermal_rad'] is not None and r['choice_'+units] is not None]
    correct=sum(r['choice_'+units]==r['winner'] for r in evaluated);labels={label:sum(r['winner']==label for r in evaluated) for label in ['rgb','thermal']}
    recalls={label:sum(r['winner']==label and r['choice_'+units]==label for r in evaluated)/labels[label] if labels[label] else None for label in labels}
    selected=[r['error_'+r['choice_'+units]+'_rad'] for r in costs];rgb=[r['error_rgb_rad'] for r in costs];thermal=[r['error_thermal_rad'] for r in costs];oracle=[min(x,y) for x,y in zip(rgb,thermal)]
    switches=[r for r in costs if r['choice_'+units]=='thermal'];positive=[r for r in evaluated if all(r[k] is not None and r[k]>0 for k in ['uncertainty_rgb_rad2','uncertainty_thermal_rad2','error_rgb_rad','error_thermal_rad'])]
    mean=lambda x:float(np.mean(x)) if x else None
    return dict(rows=len(rows),timing_eligible=sum(r['eligible'] for r in rows),rank_evaluated=len(evaluated),correct=correct,agreement=correct/len(evaluated) if evaluated else None,
        labels=labels,recalls=recalls,balanced_accuracy=mean(list(recalls.values())) if all(v is not None for v in recalls.values()) else None,
        always_rgb_agreement=labels['rgb']/len(evaluated) if evaluated else None,always_thermal_agreement=labels['thermal']/len(evaluated) if evaluated else None,
        choices={label:sum(r['choice_'+units]==label for r in costs) for label in ['rgb','thermal']},cost_rows=len(costs),mean_selected_rad=mean(selected),mean_rgb_rad=mean(rgb),mean_thermal_rad=mean(thermal),mean_oracle_rad=mean(oracle),mean_oracle_regret_rad=mean([s-o for s,o in zip(selected,oracle)]),
        thermal_switches=len(switches),helpful_switches=sum(r['error_thermal_rad']<r['error_rgb_rad'] for r in switches),harmful_switches=sum(r['error_thermal_rad']>r['error_rgb_rad'] for r in switches),neutral_switches=sum(r['error_thermal_rad']==r['error_rgb_rad'] for r in switches),
        ratio_pairs=len(positive),log_ratio_rho=association([np.log(r['uncertainty_rgb_rad2']/r['uncertainty_thermal_rad2']) for r in positive],[np.log(r['error_rgb_rad']/r['error_thermal_rad']) for r in positive]))

def run(source,output):
    source=Path(source);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    inherited=json.loads((source/'plan.json').read_text());case_map={c['case']:c for c in inherited['cases']};windows=sorted({c['case'].rsplit('-',1)[0] for c in inherited['cases'] if c['case'].startswith(('salon-so-day-','long-salon-'))})
    assert len(windows)==6
    files={str(source/'plan.json'):sha(source/'plan.json'),str(source/'analysis.json'):sha(source/'analysis.json'),str(Path(__file__)):sha(__file__)}
    for window in windows:
        for modality in ['rgb','thermal']:
            for i in range(19):
                p=source/'points'/f'{window}-{modality}'/f'{i:03d}.npz';files[str(p)]=sha(p)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='1e5839d16',windows=windows,pairs_per_window=19,files_sha256=files,
        populations=POPULATIONS,protocol=dict(primary='Median ray covariance trace and median model-LK angular disagreement on independent per-modality LK-valid masks',choice='Lower raw median covariance, exact ties abstain',timing='Both stereo<=2ms and cross<=30ms at both pair endpoints; no retiming/interpolation',scope='Descriptive comparison on previously inspected same-recording windows; not physical accuracy, calibrated probability, training or held-out evaluation'))
    save(output/'plan.json',plan);records=[]
    for window in windows:
        rgb_case=case_map[window+'-rgb'];thermal_case=case_map[window+'-thermal'];assert len(rgb_case['rows'])==len(thermal_case['rows'])==20
        for i in range(19):
            rr=rgb_case['rows'][i:i+2];tr=thermal_case['rows'][i:i+2];assert rr==tr
            offsets=[];eligibility=True
            for row in rr:
                left=row['rgb_left']['time_ns'];right=row['rgb_right']['time_ns'];tl=row['thermal_left']['time_ns'];rt=row['thermal_right']['time_ns']
                offset=dict(rgb_stereo_ns=right-left,thermal_stereo_ns=rt-tl,cross_ns=tl-left);offsets.append(offset)
                eligibility &= abs(right-left)<=2000000 and abs(rt-tl)<=2000000 and abs(tl-left)<=30000000
            assert all(rr[1][key]['time_ns']>rr[0][key]['time_ns'] for key in ['rgb_left','rgb_right','thermal_left','thermal_right'])
            with np.load(source/'points'/f'{window}-rgb'/f'{i:03d}.npz') as rgb, np.load(source/'points'/f'{window}-thermal'/f'{i:03d}.npz') as thermal:
                for population,(u_mask,e_mask) in POPULATIONS.items():
                    row=dict(window=window,pair=i,population=population,eligible=bool(eligibility),offsets=offsets,
                             times={k:[r[k]['time_ns'] for r in rr] for k in ['rgb_left','rgb_right','thermal_left','thermal_right']})
                    for modality,data in [('rgb',rgb),('thermal',thermal)]:
                        um=data['mask_'+u_mask];em=data['mask_'+e_mask];assert data['covariance_valid'].all()
                        for label,field,mask in [('uncertainty','variance_rad2',um),('native','variance_px2',um),('error','disagreement_rad',em)]:
                            # Match EXP030's float64 quantile reductions, including native float32 traces.
                            x=np.asarray(data[field][mask],dtype=float);finite=np.isfinite(x);assert np.all(x[finite]>=0)
                            key=({'uncertainty':'uncertainty_'+modality+'_rad2','native':'uncertainty_'+modality+'_px2','error':'error_'+modality+'_rad'})[label]
                            row[key]=float(np.median(x[finite])) if finite.any() else None;row[key+'_finite_count']=int(finite.sum());row[key+'_nonfinite_count']=int((~finite).sum())
                        row[modality+'_grid']=len(um);row[modality+'_u_count']=int(um.sum());row[modality+'_error_count']=int(em.sum())
                    row['choice_angular']=choice(row['uncertainty_rgb_rad2'],row['uncertainty_thermal_rad2']);row['choice_pixel']=choice(row['uncertainty_rgb_px2'],row['uncertainty_thermal_px2']);row['winner']=choice(row['error_rgb_rad'],row['error_thermal_rad']);records.append(row)
    summaries=[]
    for population in POPULATIONS:
        for window in [*windows,'pooled']:
            rows=[r for r in records if r['population']==population and (window=='pooled' or r['window']==window)]
            for units in ['angular','pixel']:summaries.append(dict(population=population,window=window,units=units,**summarize(rows,units)))
    for p,digest in files.items():assert sha(p)==digest
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),numpy=np.__version__,records=records,summaries=summaries,verified_hashes=len(files))
    save(output/'analysis.json',analysis)
    for row in summaries:
        if row['population']=='primary':print(row,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.source,a.output)
