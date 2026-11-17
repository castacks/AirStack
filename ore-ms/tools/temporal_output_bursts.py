"""Temporal assay over immutable selected pair proposals; no geometry/model calls."""
import argparse,json
from pathlib import Path
from collections import Counter
from paired_failure_strata import runs
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1];VARIANTS=['original','count','bottom'];POLICIES=['rgb','thermal','rgb_first','thermal_first'];ENDPOINTS=['missing','R1','R5'];VIEWS=['full','moving']
def lines(p):return [json.loads(s) for s in Path(p).read_text().splitlines()]
def state(event,view,endpoint):
    if view=='moving' and not event['moving']:return 'outside'
    if endpoint!='missing' and not event['common']:return 'unknown'
    if event['sensor'] is None:return 'missing'
    if endpoint=='missing':return 'good'
    return 'good' if event['error_deg']<=int(endpoint[1:]) else 'disagree'
def describe(states,times):
    episodes=[];n=len(states)
    for ids in runs([i for i,s in enumerate(states) if s in ['missing','disagree']]):
        first,last=ids[0],ids[-1];left=states[first-1] if first else 'record_boundary';right=states[last+1] if last+1<n else 'record_boundary';episodes.append({'first_index':first,'last_index':last,'first_anchor_ns':times[first],'last_anchor_ns':times[last],'length':len(ids),'missing':sum(states[i]=='missing' for i in ids),'disagree':sum(states[i]=='disagree' for i in ids),'left_state':left,'right_state':right,'left_censored':left!='good','right_censored':right!='good','sampled_span_ns':times[last]-times[first],'observed_return_lag_ns':times[last+1]-times[first] if right=='good' else None})
    counts=Counter(states);lengths=[e['length'] for e in episodes];lags=[e['observed_return_lag_ns'] for e in episodes if e['observed_return_lag_ns'] is not None];summary={'anchors':n,**{k:counts[k] for k in ['good','missing','disagree','unknown','outside']},'assessed':sum(counts[k] for k in ['good','missing','disagree']),'bad':counts['missing']+counts['disagree'],'episodes':len(episodes),'length_histogram':{str(k):v for k,v in sorted(Counter(lengths).items())},'longest_length':max(lengths) if lengths else None,'longest_sampled_span_ns':max((e['sampled_span_ns'] for e in episodes),default=None),'left_censored':sum(e['left_censored'] for e in episodes),'right_censored':sum(e['right_censored'] for e in episodes),'observed_returns':len(lags),'conditional_max_return_lag_ns':max(lags,default=None)};return episodes,summary
class Assay:
    def __init__(self,config):self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output']
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def load(self,c):
        source=ROOT/c['source'];pred=lines(source/'predictions.jsonl');scores=read(source/('scores.json' if c['tier']=='supplied_reference' else 'disagreements.json'));choices=read(source/'choices.json')
        if 'recording' in pred[0]:pred=[r for r in pred if r['recording']==c['name']];scores=[r for r in scores if r['recording']==c['name']];choices=[r for r in choices if r['recording']==c['name']]
        return {(r['variant'],r['frontend'],r['arm'],r['index']):r for r in pred},{(r['variant'],r['frontend'],r['arm'],r['index']):r for r in scores},{(r['variant'],r['frontend'],r['index']):r for r in choices}
    def freeze(self):
        files=[self.config,Path(__file__),ROOT/self.cfg['protocol'],ROOT/'tools/paired_failure_strata.py',ROOT/'tools/deployment_lighting.py',ROOT/'tools/audit_temporal_output_bursts.py',ROOT/'tools/test_temporal_output_bursts.py'];plan={}
        for name,c in self.cfg['records'].items():
            source=ROOT/c['source'];assert read(source/'audit.json')['status']=='pass';assert sha(source/'predictions.jsonl')==read(source/'prediction-lock.json')['predictions_sha256'] and sha(source/'choices.json')==read(source/'choice-lock.json')['choices_sha256'];done=read(source/'completion.json');sf='scores' if c['tier']=='supplied_reference' else 'disagreements';assert sha(source/(sf+'.json'))==done[sf+'_sha256'];assert sha(source/'summary.json')==done['summary_sha256'];inputs=read(ROOT/c['matching']/'matcher-inputs.json');meta={(r['arm'],r['index']):r for r in inputs};n=c['anchors'];assert len(inputs)==len(meta)==2*n;times=[meta['rgb',i]['anchor_ns'] for i in range(n)];assert all(meta['thermal',i]['anchor_ns']==times[i] for i in range(n)) and all(b-a==500000000 for a,b in zip(times,times[1:]));pop=read(ROOT/c['population']);common=pop['common_reference' if c['tier']=='supplied_reference' else 'common_gyro'];moving=pop['paired_rotation_ge2deg'];assert set(moving)<=set(common)<=set(range(n));pred,scores,ds=self.load(c);expected={(v,f,a,i) for v in VARIANTS for f in ['sift','learned'] for a in ['rgb','thermal'] for i in range(n)};assert set(pred)==set(scores)==expected and set(ds)=={(v,f,i) for v in VARIANTS for f in ['sift','learned'] for i in range(n)}
            for key,p in pred.items():assert scores[key]['status']==p['status']
            for (v,f,i),d in ds.items():
                for policy in POLICIES:
                    picked=d['policies'][policy];a=picked['sensor']
                    if a:
                        assert pred[v,f,a,i]['status']=='ok' and picked['endpoints']=={k:meta[a,i][k] for k in ['source','target','times_ns']}
                        if i in common:assert scores[v,f,a,i]['R_error' if c['tier']=='supplied_reference' else 'disagreement_deg'] is not None
            plan[name]={'anchors':n,'anchor_ns':times,'common':common,'moving':moving,'tier':c['tier']};files.extend(source/f for f in ['input-freeze.json','predictions.jsonl','prediction-lock.json','choices.json','choice-lock.json',sf+'.json','summary.json','completion.json','audit.json']);files.extend([ROOT/c['matching']/'matcher-inputs.json',ROOT/c['population']])
        self.out.mkdir(parents=True,exist_ok=False);write(self.out/'preflight.json',plan);files.append(self.out/'preflight.json');write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'expected_events':56400,'expected_groups':432,'anchors':2350,'anchor_step_ns':500000000,'runtime_image':self.cfg['runtime_image'],'scope':'Full source/status/error/cohort/endpoints/grid preflight before temporal summaries'});print('All2350anchors and56400selected events preflighted',flush=True)
    def run(self):
        self.verify();plan=read(self.out/'preflight.json');events=[];episodes=[];summaries=[]
        for name,c in self.cfg['records'].items():
            p=plan[name];n=p['anchors'];common=set(p['common']);moving=set(p['moving']);pred,scores,ds=self.load(c)
            for v in VARIANTS:
                for f in ['sift','learned']:
                    for policy in POLICIES:
                        seq=[];key={'recording':name,'tier':c['tier'],'variant':v,'frontend':f,'policy':policy}
                        for i in range(n):
                            d=ds[v,f,i]['policies'][policy];a=d['sensor'];error=scores[v,f,a,i]['R_error' if c['tier']=='supplied_reference' else 'disagreement_deg'] if a else None;e={**key,'index':i,'anchor_ns':p['anchor_ns'][i],'sensor':a,'endpoints':d['endpoints'],'error_deg':error,'common':i in common,'moving':i in moving};events.append(e);seq.append(e)
                        for view in VIEWS:
                            for endpoint in ENDPOINTS:
                                states=[state(e,view,endpoint) for e in seq];ee,summary=describe(states,p['anchor_ns']);extra={**key,'view':view,'endpoint':endpoint};episodes.extend(extra|e for e in ee);summaries.append(extra|summary)
        sm={(s['recording'],s['variant'],s['frontend'],s['policy'],s['view'],s['endpoint']):s for s in summaries};deltas=[]
        for key,s in sm.items():
            if key[1]!='original':continue
            for before,after in [('original','count'),('original','bottom'),('count','bottom')]:
                a,b=[sm[(key[0],v)+key[2:]] for v in [before,after]];d={k:s[k] for k in ['recording','tier','frontend','policy','view','endpoint']};d.update(before=before,after=after,**{field+'_delta':(b[field] or 0)-(a[field] or 0) for field in ['bad','episodes','longest_length']});deltas.append(d)
        assert len(events)==56400 and len(summaries)==len(deltas)==432
        for name,data in [('events',events),('episodes',episodes),('summary',summaries),('deltas',deltas)]:write(self.out/(name+'.json'),data)
        self.verify();write(self.out/'completion.json',{**{name+'_sha256':sha(self.out/(name+'.json')) for name in ['events','episodes','summary','deltas']},'event_rows':len(events),'episodes':len(episodes),'groups':len(summaries),'new_predictions':0,'scope':'Sampled pair-proposal error/missing blocks, separate nominal reference tiers; no stateful VO recovery'});print('Temporal groups complete',len(summaries),'episodes',len(episodes),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','run']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Assay(a.config),a.stage)()
