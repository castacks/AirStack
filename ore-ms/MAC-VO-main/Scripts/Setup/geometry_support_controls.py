"""Fixed current-support and geometry sensitivities; no reference reads or fitting."""
import math
import numpy as np
CAMERAS=('rgb','thermal')
METHODS=('geometry_admission','recovery','raw','history','rgb_only','thermal_only','retained_count','occupied_cells','retained_fraction','information_rank','information_per_factor')


def health_features(prefix_row,timeline):
    deadline=prefix_row['controller_deadline_ns'];out={}
    for camera,e in prefix_row['experts'].items():
        available=e['available_by_prefix'] and e['valid_model_projection']
        if not available:out[camera]=None;continue
        byseq={x['event']['sequence']:x['event'] for x in timeline if x['kind']=='observer_event' and x['camera']==camera and x['controller_monotonic_ns']<=deadline}
        source=e['source'];health=byseq[source['health_event_sequence']];motion=byseq[source['motion_event_sequence']]
        assert health['kind']=='pair_end' and motion['kind']=='motion_result_written'
        assert [health['time0_ns'],health['time1_ns']]==e['native_interval_ns']==[motion['time0_ns'],motion['time1_ns']]
        geo,=[x for x in byseq.values() if x['kind']=='native_disparity_information' and x['submission_id']==motion['submission_id']]
        counts=[health[k] for k in ['selected_count','retained_count','occupied_3x3_cells']]
        if any(type(x)is not int or x<0 for x in counts) or counts[1]>counts[0] or counts[2]>9:raise ValueError('valid causal counts required')
        if type(geo['information']['factors'])is not int or geo['information']['factors']<=0:raise ValueError('positive native factor count required')
        out[camera]={'retained_count':counts[1],'occupied_cells':counts[2],'retained_fraction':counts[1]/max(counts[0],1),
            'native_factors':geo['information']['factors'],'health_sequence':health['sequence'],'motion_sequence':motion['sequence'],'geometry_sequence':geo['sequence']}
    return out


def choose(feature,available):
    eligible=[c for c in CAMERAS if available[c]]
    if not eligible:return 'ABSTAIN'
    if any(feature.get(c) is None or not math.isfinite(feature[c]) for c in eligible):raise ValueError('missing/nonfinite feature for available expert')
    return max(eligible,key=lambda c:(feature[c],c=='rgb')).upper()


def augment(base_row,features):
    actions={'geometry_admission':base_row['actions']['geometry_current'],'recovery':base_row['actions']['candidate'],
        **{m:base_row['actions'][m] for m in ['raw','history','rgb_only','thermal_only']}}
    availability=base_row['native_available']
    for camera in CAMERAS:
        if availability[camera]!=(features[camera] is not None):raise ValueError('feature/source availability mismatch')
    for key in ['retained_count','occupied_cells','retained_fraction']:
        actions[key]=choose({c:features[c][key] if features[c] else None for c in CAMERAS},availability)
    score=base_row['normalized_information_min_eigenvalue']
    actions['information_rank']=choose(score,availability)
    actions['information_per_factor']=choose({c:score[c]/features[c]['native_factors'] if features[c] else None for c in CAMERAS},availability)
    return {**{k:v for k,v in base_row.items() if k!='actions'},'actions':actions,'support_features':features,
        'both_supported':all(availability.values()),'labels_used':False}


def evaluate(rows,labels):
    if set(labels)!={r['sample_id'] for r in rows}:raise ValueError('exact population required')
    def stats(selected,method):
        valid=[r for r in selected if r['actions'][method]!='ABSTAIN'];values=[]
        for r in valid:
            loss=labels[r['sample_id']][r['actions'][method].lower()]
            if loss is None:raise ValueError('selected unavailable motion')
            if any(not math.isfinite(v) or v<0 for v in loss.values()):raise ValueError('finite nonnegative losses required')
            values.append(loss)
        return {'scored':len(valid),'ids':[r['sample_id'] for r in valid],**{metric:float(np.sqrt(np.mean([v[metric]**2 for v in values]))) if values else None for metric in ['translation_m','rotation_deg']}}
    groups={}
    for scope,selected in [('all',rows),('both_supported',[r for r in rows if r['both_supported']])]:
        methods={m:stats(selected,m) for m in METHODS};matched={}
        for method in METHODS[1:]:
            common=[r for r in selected if r['actions']['geometry_admission']!='ABSTAIN' and r['actions'][method]!='ABSTAIN']
            matched[method]={'ids':[r['sample_id'] for r in common],'geometry_admission':stats(common,'geometry_admission'),method:stats(common,method)}
        groups[scope]={'population':len(selected),'ids':[r['sample_id'] for r in selected],'methods':methods,'matched':matched}
    required=['raw','history','rgb_only','thermal_only','retained_count','occupied_cells','retained_fraction']
    matched=groups['both_supported']['matched'];coverage=groups['all']['methods']
    passed=coverage['geometry_admission']['scored']>=coverage['raw']['scored'] and all(matched[m]['ids'] and matched[m]['geometry_admission']['translation_m']<matched[m][m]['translation_m'] and matched[m]['geometry_admission']['rotation_deg']<=matched[m][m]['rotation_deg'] for m in required)
    return {'groups':groups,'primary_criterion_passed':bool(passed),'sensitivity_selected_as_primary':False,'candidate_adopted':False,'independent_ground_truth':False}
