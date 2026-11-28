"""Only I corrections and H adapters they wrap; prior full H suite remains sealed."""
from pathlib import Path
import datetime
import io
import json
import unittest
import test_regression
from test_h import AdapterTests
from native_environment import sha

HERE=Path(__file__).resolve().parent

def suite():
    return unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(test_regression.FocusedTests),
                              unittest.defaultTestLoader.loadTestsFromTestCase(AdapterTests)])

def run():
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite())
    report={'status':'PASS' if result.wasSuccessful() else 'FAIL','tests':result.testsRun,
            'failures':len(result.failures),'errors':len(result.errors),'skipped':len(result.skipped),
            'baseline_commit':'f5b14e08cbf0ef54bfe3c986ea377fbe50adbf61',
            'run_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'scope':'I corrected checker/build/type/recorder mappings plus three affected H adapter tests; no broad H rerun',
            'source_sha256':{p.name:sha(p) for p in sorted(HERE.glob('*.py')) if p.name!='validate_milestone.py'},
            'production_runtime_executed':False,'log':stream.getvalue()}
    (HERE/'M00I_REGRESSION_RESULTS.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    return report

if __name__=='__main__':
    result=run();print(json.dumps(result,indent=2));raise SystemExit(0 if result['status']=='PASS' else 1)
