"""Per-recording prospective criterion and non-pooled all-model controls for EXP064."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from ms2_frozen_suite import declared,SUITE_SHA,sha


def main(root,suite,output):
    if output.exists():raise FileExistsError(output)
    protocol=json.loads((suite/'suite-protocol.json').read_text());records={};inputs={str(suite/'suite-protocol.json'):sha(suite/'suite-protocol.json')};source_sets={}
    prior_paths=[root/'results/ms2-day-replication/source-manifest.json',root/'results/ms2-rain-replication/source-manifest.json',root/'results/ms2-timestamp-semantics/source-protocol-input-manifest.json',root/'results/ms2-unexamined-recording/dataset/source-manifest.json'];prior=set()
    for path in prior_paths:
        for row in json.loads(path.read_text())['records']:
            for camera in row['cameras'].values():
                for side in ['left','right']:prior.add(camera[side]['sha256'])
    for condition in protocol['records']:
        _,record,base=declared(root,suite,condition);path=base/'frozen-evaluation/report.json';report=json.loads(path.read_text());inputs[str(path)]=sha(path)
        assert report['primary']=='uncertainty' and report['sequence']==record['sequence']
        for filename in ['source-validation.json','geometry-validation.json','evaluation-validation.json']:
            p=base/filename;assert json.loads(p.read_text())['status']=='PASS';inputs[str(p)]=sha(p)
        source=json.loads((base/'dataset/source-manifest.json').read_text());images=[camera[side]['sha256'] for row in source['records'] for camera in row['cameras'].values() for side in ['left','right']];source_sets[condition]=set(images)
        assert not source_sets[condition]&prior,'New source images overlap previously examined recordings'
        rows=report['rows'];assert [r['stem'] for r in rows]==protocol['ids'];good=[r for r in rows if r['availability']['both_valid']]
        comparisons={}
        for anchor in ['midpoint','rgb','thr']:
            comparisons[anchor]={}
            for loss in ['mean_l2','mse']:
                comparisons[anchor][loss]={}
                for family in ['constant','uncertainty','geometry','combined']:
                    comparisons[anchor][loss][family]={}
                    names=rows[0]['risks'][anchor][loss]
                    for comparator in names:
                        if comparator==family:continue
                        diffs=[r['risks'][anchor][loss][family]-r['risks'][anchor][loss][comparator] for r in good]
                        ratios=[r['risks'][anchor][loss][family]/r['risks'][anchor][loss][comparator] for r in good]
                        stats={'available_frames':len(good),'better':sum(x<0 for x in diffs),'equal':sum(x==0 for x in diffs),'worse':sum(x>0 for x in diffs),'min':min(ratios) if good else None,'median':float(np.median(ratios)) if good else None,'max':max(ratios) if good else None}
                        comparisons[anchor][loss][family][comparator]=stats
        primary=comparisons['midpoint']['mean_l2']['uncertainty'];criterion=bool(good) and all(primary[k]['median']<1 for k in ['always_rgb','always_thr','scalar_fit'])
        records[condition]={'sequence':record['sequence'],'available_frames':len(good),'unavailable_stems':report['unavailable_stems'],'source_images':len(images),'source_distinct_hashes':len(source_sets[condition]),'availability_totals':{k:sum(r['availability'][k] for r in rows) for k in rows[0]['availability']},'criterion_met_on_available_frames':criterion,'comparisons':comparisons}
    overlap={}
    conditions=list(source_sets)
    for i,a in enumerate(conditions):
        for b in conditions[i+1:]:
            common=source_sets[a]&source_sets[b];overlap[a+'_'+b]=len(common);assert not common,'Suite recordings share source image bytes'
    for p,h in inputs.items():assert sha(p)==h
    result={'suite':'EXP064','primary':'uncertainty','suite_sha256':SUITE_SHA,'criterion':'each recording midpoint meanL2 median U-only / bothfixed / scalarfit strictly below1','criterion_met_on_available_frames':all(x['criterion_met_on_available_frames'] for x in records.values()),'complete_frame_coverage':all(x['available_frames']==20 for x in records.values()),'prior_distinct_image_hashes':len(prior),'cross_recording_image_hash_overlap':overlap,'records':records,'frozen_summary_inputs':inputs,'no_pooled_pixel_or_frame_risk':True,'script_sha256':sha(__file__)}
    output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'criterion_met':result['criterion_met_on_available_frames'],'coverage_complete':result['complete_frame_coverage'],'records':{k:{'criterion_met':v['criterion_met_on_available_frames'],'primary':v['comparisons']['midpoint']['mean_l2']['uncertainty']} for k,v in records.items()}},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.root,a.suite,a.output)
