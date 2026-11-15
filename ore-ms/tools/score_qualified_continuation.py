"""Apply the frozen reproduction gate per record; retain unqualified diagnostics.

The original all-or-nothing score guard remains unchanged. This evaluator does
not relax it: a failed record is explicitly ineligible for a native-VO claim.
"""
from pathlib import Path
import datetime
import sys
import numpy as np
import fixed_rotation_continuation as experiment

native=experiment.native
ROOT=experiment.ROOT
RUN=experiment.OUT/'run-001'
OUT=experiment.OUT/'evaluation-001'


def main():
    OUT.mkdir(parents=True,exist_ok=False)
    lock=native.read(RUN/'prediction-lock.json')
    protocol={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_sha256':native.sha(__file__),'original_experiment_sha256':native.sha(experiment.__file__),
        'prediction_lock_sha256':native.sha(RUN/'prediction-lock.json'),
        'native_translation_bound_m':1e-3,'native_rotation_bound_rad':1e-4,
        'per_record_gate':True,'failed_record_accuracy_is_diagnostic_only':True,
        'predictions_replaced':False,'new_references_parsed':False}
    native.write(OUT/'protocol.json',protocol);summary={}
    for entry in lock['rows']:
        c=entry['condition'];p=RUN/f'predictions-{c}.json';assert native.sha(p)==entry['prediction_sha256']
        payload=native.read(p);rows=payload['rows']
        failures=[x for x in payload['new_native_reproduction'] if x['translation_m']>=1e-3 or x['rotation_rad']>=1e-4]
        qualified=not failures;refs=[];invalid={}
        for i in range(1200):
            try:refs.append(experiment.feedback.rigid(np.loadtxt(experiment.OUT/'recovery/references'/c/f'{i:06}.txt').reshape(4,4))[0])
            except ValueError as error:refs.append(None);invalid[i]=str(error)
        methods={}
        for method in experiment.METHODS:
            def stats(start,end):
                bad=[i for i in invalid if start<=i<=end]
                if bad:return {'status':'UNQUALIFIED_REFERENCE','reference_source_ids':bad}
                result=native.path_stats(rows,refs,method,start,end)
                if result['status']=='COMPLETE':result.pop('positions');result.pop('reference')
                return result
            methods[method]={'available_new':sum(r['methods'][method]['transform'] is not None for r in rows[599:]),
                'new_span':stats(599,1199),'full_path':stats(0,1199),
                'new_blocks':[{'start':i,**stats(i,i+99)} for i in range(600,1200,100)]}
        source=native.read(native.REC/'inputs'/c/'source-manifest.json')['records']
        duration=(source[1199]['cameras']['rgb']['left']['recorded_time_ns']-source[599]['cameras']['rgb']['left']['recorded_time_ns'])/1e9
        summary[c]={'sequence':experiment.SEQUENCES[c],
            'native_reproduction_gate_passed':qualified,'native_reproduction_failures':failures,
            'native_reproduction_supported':len(payload['new_native_reproduction']),
            'accuracy_interpretation':'qualified_saved_measurement_replay' if qualified else 'diagnostic_only_native_reproduction_gate_failed',
            'new_span_recorded_rgb_seconds':duration,'invalid_references':invalid,'methods':methods}
        print('EVALUATED',c,summary[c]['accuracy_interpretation'],{m:v['new_span'] for m,v in methods.items()},flush=True)
    native.write(OUT/'scores.json',{'summary':summary,'protocol_sha256':native.sha(OUT/'protocol.json'),
        'prediction_lock_sha256':native.sha(RUN/'prediction-lock.json'),
        'scope':'Frozen gate unchanged; per-record eligibility. Failed record metrics retained only as diagnostics.'})


if __name__=='__main__':main()
