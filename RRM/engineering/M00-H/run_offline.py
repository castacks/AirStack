"""Run H synthetic tests, save bounded local evidence; no runtime services."""
from pathlib import Path
import datetime
import io
import json
import unittest
from h_support import file_sha,decode
from h_preflight import evaluate
import test_h


def suite():
    return unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(cls)
        for cls in [test_h.IntegrationTests,test_h.GuardTests,test_h.ReliabilityTests,test_h.AdapterTests,test_h.PreflightTests])


def run():
    root=Path(__file__).resolve().parents[3];here=Path(__file__).resolve().parent;e=root/'RRM/evidence/M00-H'
    sources={str(p.relative_to(root)):file_sha(p) for p in sorted(here.rglob('*')) if p.is_file()
             and not set(p.relative_to(here).parts)&{'notebook','__pycache__','.tmp','build'}
             and (p.suffix=='.py' or 'fixtures' in p.parts)}
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite())
    report={'status':'PASS' if result.wasSuccessful() else 'FAIL','test_count':result.testsRun,
            'failures':len(result.failures),'errors':len(result.errors),'skipped':len(result.skipped),
            'run_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'baseline_commit':'45d026e2b8c46b0e1c44013d8dd9d7ed49e3d5ac','source_sha256':sources,
            'synthetic_only':True,'log':stream.getvalue()}
    (e/'M00H_TEST_RESULTS.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    build=decode((e/'build/BUILD_RESULT.json').read_bytes())
    assessment=evaluate(root,build,report)
    (e/'M00H_PREFLIGHT_RESULTS.json').write_text(json.dumps(assessment,indent=2,sort_keys=True)+'\n')
    return report,assessment


if __name__=='__main__':
    report,assessment=run()
    print(json.dumps({'tests':report['test_count'],'status':report['status'],
                      'preflight':{k:v for k,v in assessment.items() if 'STATUS' in k or 'READINESS' in k}},indent=2))
    raise SystemExit(0 if report['status']=='PASS' else 1)
