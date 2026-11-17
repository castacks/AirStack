"""Independent source-to-event and streaming episode/censor/summary audit."""
import json,hashlib
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parents[1]
VARIANTS=['original','count','bottom'];POLICIES=['rgb','thermal','rgb_first','thermal_first'];KEY=['recording','tier','variant','frontend','policy','view','endpoint']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def lines(p):return [json.loads(s) for s in Path(p).read_text().splitlines()]
def stream(states,times):
    result=[];start=None
    for j in range(len(states)+1):
        bad=j<len(states) and states[j] in ('missing','disagree')
        if bad and start is None:start=j
        if not bad and start is not None:
            stop=j-1;left=states[start-1] if start>0 else 'record_boundary';right=states[j] if j<len(states) else 'record_boundary';counter=Counter(states[start:j]);result.append({'first_index':start,'last_index':stop,'first_anchor_ns':times[start],'last_anchor_ns':times[stop],'length':j-start,'missing':counter['missing'],'disagree':counter['disagree'],'left_state':left,'right_state':right,'left_censored':left in ('unknown','outside','record_boundary'),'right_censored':right in ('unknown','outside','record_boundary'),'sampled_span_ns':times[stop]-times[start],'observed_return_lag_ns':times[j]-times[start] if right=='good' else None});start=None
    return result
def main(config):
    cfg=read(config);out=ROOT/cfg['output'];checks=0
    def check(v,label):
        nonlocal checks
        assert v,label
        checks+=1
    freeze=read(out/'input-freeze.json')
    for path,h in freeze['pins'].items():check(sha(ROOT/path)==h,'frozen '+path)
    done=read(out/'completion.json')
    for name in ['events','episodes','summary','deltas']:check(sha(out/(name+'.json'))==done[name+'_sha256'],'locked '+name)
    events=read(out/'events.json');em={(e['recording'],e['variant'],e['frontend'],e['policy'],e['index']):e for e in events};summaries=read(out/'summary.json');sm={tuple(s[k] for k in KEY):s for s in summaries};check(len(events)==len(em)==freeze['expected_events']==done['event_rows']==56400,'unique full event population');check(len(summaries)==len(sm)==freeze['expected_groups']==done['groups']==432,'unique full groups');expected_episodes=[];expected_summaries=[];plan=read(out/'preflight.json')
    for name,c in cfg['records'].items():
        src=ROOT/c['source'];parent=read(src/'summary.json');check(read(src/'audit.json')['status']=='pass','parent audit retained');pred=lines(src/'predictions.jsonl');scores=read(src/('scores.json' if c['tier']=='supplied_reference' else 'disagreements.json'));ds=read(src/'choices.json')
        if 'recording' in pred[0]:pred=[p for p in pred if p['recording']==name];scores=[p for p in scores if p['recording']==name];ds=[p for p in ds if p['recording']==name]
        pm={(p['variant'],p['frontend'],p['arm'],p['index']):p for p in pred};errors={(p['variant'],p['frontend'],p['arm'],p['index']):p for p in scores};dm={(d['variant'],d['frontend'],d['index']):d for d in ds};meta={(p['arm'],p['index']):p for p in read(ROOT/c['matching']/'matcher-inputs.json')};n=c['anchors'];times=[meta['rgb',i]['anchor_ns'] for i in range(n)];check(all(meta['thermal',i]['anchor_ns']==times[i] for i in range(n)) and all(times[i]-times[i-1]==500000000 for i in range(1,n)),'common exact 500ms chronology');pop=read(ROOT/c['population']);common=set(pop['common_reference' if c['tier']=='supplied_reference' else 'common_gyro']);moving=set(pop['paired_rotation_ge2deg']);check(plan[name]=={'anchors':n,'anchor_ns':times,'common':pop['common_reference' if c['tier']=='supplied_reference' else 'common_gyro'],'moving':pop['paired_rotation_ge2deg'],'tier':c['tier']},'source preflight contract');check(len(pred)==len(pm)==len(scores)==len(errors)==12*n and len(ds)==len(dm)==6*n,'source geometry/choice population')
        for v in VARIANTS:
            for f in ['sift','learned']:
                for policy in POLICIES:
                    seq=[];base={'recording':name,'tier':c['tier'],'variant':v,'frontend':f,'policy':policy}
                    for i in range(n):
                        d=dm[v,f,i]['policies'][policy];a=d['sensor'];order=[policy] if policy in ['rgb','thermal'] else ['rgb','thermal'] if policy=='rgb_first' else ['thermal','rgb'];expected_sensor=next((a for a in order if pm[v,f,a,i]['status']=='ok'),None);check(a==expected_sensor,'independent status policy');endpoints={k:meta[a,i][k] for k in ['source','target','times_ns']} if a else None;check(d['endpoints']==endpoints,'own sensor endpoints');error=errors[v,f,a,i]['R_error' if c['tier']=='supplied_reference' else 'disagreement_deg'] if a else None;expected=base|{'index':i,'anchor_ns':times[i],'sensor':a,'endpoints':endpoints,'error_deg':error,'common':i in common,'moving':i in moving};e=em[name,v,f,policy,i];check(e==expected,'source event identity/error/time/cohort');seq.append(e)
                    for view in ['full','moving']:
                        for endpoint in ['missing','R1','R5']:
                            states=[]
                            for i,e in enumerate(seq):
                                if view=='moving' and i not in moving:state='outside'
                                elif endpoint!='missing' and i not in common:state='unknown'
                                elif e['sensor'] is None:state='missing'
                                elif endpoint=='missing':state='good'
                                else:state='good' if e['error_deg']<=(1 if endpoint=='R1' else 5) else 'disagree'
                                states.append(state)
                            episodes=stream(states,times);meta_key=base|{'view':view,'endpoint':endpoint};expected_episodes.extend(meta_key|e for e in episodes);counter=Counter(states);hist=Counter(e['length'] for e in episodes);lags=[e['observed_return_lag_ns'] for e in episodes if e['observed_return_lag_ns'] is not None];expected=meta_key|{'anchors':n,**{k:counter[k] for k in ['good','missing','disagree','unknown','outside']},'assessed':n-counter['unknown']-counter['outside'],'bad':counter['missing']+counter['disagree'],'episodes':len(episodes),'length_histogram':{str(k):v for k,v in sorted(hist.items())},'longest_length':max(hist,default=None),'longest_sampled_span_ns':max((e['sampled_span_ns'] for e in episodes),default=None),'left_censored':sum(e['left_censored'] for e in episodes),'right_censored':sum(e['right_censored'] for e in episodes),'observed_returns':len(lags),'conditional_max_return_lag_ns':max(lags,default=None)};key=tuple(meta_key[k] for k in KEY);check(sm[key]==expected,'independent full temporal summary');expected_summaries.append(expected)
                            for e in episodes:check(e['sampled_span_ns']==(e['length']-1)*500000000 and (e['observed_return_lag_ns'] is None or e['observed_return_lag_ns']==e['length']*500000000),'span/conditional lag, no imputation')
                            group='paired_rotation_ge2deg' if view=='moving' else 'all_declared' if endpoint=='missing' else 'common_reference' if c['tier']=='supplied_reference' else 'common_gyro';m=parent[name][v][f][group][policy] if c['tier']=='supplied_reference' else parent[v]['frontends'][f][group][policy]
                            if endpoint=='missing':check(expected['bad']==m['anchors']-m['outputs'],'availability marginal parent parity')
                            else:check(expected['bad']==(m['rotation_eligible']-m[endpoint] if c['tier']=='supplied_reference' else m['gyro_eligible']-m['within1deg' if endpoint=='R1' else 'within5deg']),'angular marginal parent parity')
    check(read(out/'episodes.json')==expected_episodes and len(expected_episodes)==done['episodes'],'all maximal episode rows/boundaries/censor flags');check(summaries==expected_summaries,'all ordered groups');deltas=[]
    for s in expected_summaries:
        if s['variant']!='original':continue
        for before,after in [('original','count'),('original','bottom'),('count','bottom')]:
            a,b=[sm[tuple((v if k=='variant' else s[k]) for k in KEY)] for v in [before,after]];d={k:s[k] for k in ['recording','tier','frontend','policy','view','endpoint']};d['before']=before;d['after']=after
            for field in ['bad','episodes','longest_length']:d[field+'_delta']=(b[field] if b[field] is not None else 0)-(a[field] if a[field] is not None else 0)
            deltas.append(d)
    check(read(out/'deltas.json')==deltas and len(deltas)==432,'all exact paired deltas');check(done['new_predictions']==0,'no new predictions');result={'status':'pass','checks':checks,'event_rows':len(events),'episode_rows':len(expected_episodes),'groups':len(sm),'auditor_sha256':sha(__file__),'scope':'Independent source/policy/error binding and streaming gap/censor/run/histogram/lag/delta audit. Parent marginal parity; separate nominal tiers; no continuous outage or VO recovery claim.'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
