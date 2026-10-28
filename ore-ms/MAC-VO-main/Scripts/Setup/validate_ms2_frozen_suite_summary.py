"""Independent per-recording aggregation, exact ties and prospective criterion audit."""
import argparse,hashlib,json,statistics
from pathlib import Path


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(root,suite,source,output):
    if output.exists():raise FileExistsError(output)
    report=json.loads((source/'report.json').read_text());protocol=json.loads((suite/'suite-protocol.json').read_text())
    assert digest(suite/'suite-protocol.json')==report['suite_sha256']=='9781971e743186d0d65f977b3a60d51882e531d1ef33840d1784e0be80435888'
    counts={'input_hashes':0,'statistics':0,'tie_counts':0,'availability_totals':0,'criterion_checks':0,'source_images':0}
    for p,h in report['frozen_summary_inputs'].items():assert digest(p)==h;counts['input_hashes']+=1
    previous=set();by_record={};criteria=[];coverage=[]
    prior=[root/'results/ms2-day-replication/source-manifest.json',root/'results/ms2-rain-replication/source-manifest.json',root/'results/ms2-timestamp-semantics/source-protocol-input-manifest.json',root/'results/ms2-unexamined-recording/dataset/source-manifest.json']
    for p in prior:
        for row in json.loads(p.read_text())['records']:
            for camera in row['cameras'].values():
                for side in ['left','right']:previous.add(camera[side]['sha256'])
    assert len(previous)==report['prior_distinct_image_hashes']
    for condition,declared in protocol['records'].items():
        base=suite/condition;case=json.loads((base/'frozen-evaluation/report.json').read_text());summary=report['records'][condition]
        assert case['sequence']==summary['sequence']==declared['sequence'] and case['primary']=='uncertainty'
        assert [r['stem'] for r in case['rows']]==[f'{i:06d}' for i in range(20)]
        available=[r for r in case['rows'] if r['availability']['both_valid']>0];assert summary['available_frames']==len(available)
        assert summary['unavailable_stems']==[r['stem'] for r in case['rows'] if r['availability']['both_valid']==0]
        for k,v in summary['availability_totals'].items():assert sum(r['availability'][k] for r in case['rows'])==v;counts['availability_totals']+=1
        for anchor,losses in summary['comparisons'].items():
            for loss,models in losses.items():
                for family,comparisons in models.items():
                    for comparator,stats in comparisons.items():
                        values=[];better=equal=worse=0
                        for r in available:
                            a,b=r['risks'][anchor][loss][family],r['risks'][anchor][loss][comparator];values.append(a/b)
                            if a<b:better+=1
                            elif a>b:worse+=1
                            else:equal+=1
                        assert [stats[k] for k in ['better','equal','worse']]==[better,equal,worse] and better+equal+worse==len(available);counts['tie_counts']+=3
                        expected={'available_frames':len(available),'min':min(values) if values else None,'median':statistics.median(values) if values else None,'max':max(values) if values else None}
                        for k,v in expected.items():
                            if v is None:assert stats[k] is None
                            else:assert abs(stats[k]-v)<=1e-12*max(1,abs(v))
                            counts['statistics']+=1
        comparator_medians=[]
        for comparator in ['always_rgb','always_thr','scalar_fit']:
            ratios=[r['risks']['midpoint']['mean_l2']['uncertainty']/r['risks']['midpoint']['mean_l2'][comparator] for r in available]
            comparator_medians.append(statistics.median(ratios) if ratios else None)
        criterion=bool(available) and all(v<1 for v in comparator_medians);assert criterion==summary['criterion_met_on_available_frames'];criteria.append(criterion);coverage.append(len(available)==20);counts['criterion_checks']+=1
        src=json.loads((base/'dataset/source-manifest.json').read_text());hashes=[]
        for row in src['records']:
            for camera in row['cameras'].values():
                for side in ['left','right']:
                    rec=camera[side];assert digest(root/rec['path'])==rec['sha256'];hashes.append(rec['sha256']);counts['source_images']+=1
        by_record[condition]=set(hashes);assert not by_record[condition]&previous
        assert len(hashes)==summary['source_images']==80 and len(set(hashes))==summary['source_distinct_hashes']
    assert all(criteria)==report['criterion_met_on_available_frames'] and all(coverage)==report['complete_frame_coverage'];counts['criterion_checks']+=2
    conditions=list(by_record)
    for i,a in enumerate(conditions):
        for b in conditions[i+1:]:assert len(by_record[a]&by_record[b])==report['cross_recording_image_hash_overlap'][a+'_'+b]==0
    result={'status':'PASS','checks':counts,'validator_sha256':digest(__file__),'summary_sha256':digest(source/'report.json'),'criterion_met':all(criteria),'coverage_complete':all(coverage),'scope':'per-recording aggregation, source nonoverlap, exact ties and declared criterion; no new fit or statistical significance'}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--suite',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.root,a.suite,a.source,a.output)
