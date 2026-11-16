"""Run frozen finite/audit tests with an effective camera-choice corruption.

The original frozen test assigns RGB to an initial choice that is already RGB
because thermal input is unsupported. Retain that failed no-op test and source;
replace only that test fixture action here, without changing frozen runtime,
production policies, audits or results.
"""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_paired_priority_controls import FiniteCases,SemanticAudit
class EffectiveSemanticAudit(SemanticAudit):
    def test_reject_rehashed_camera_choice(self):
        def mutate(d):
            c=d[0]['policies']['thermal_first'];c['sensor']='thermal' if c['sensor']=='rgb' else 'rgb'
        self.corrupt('choices.json',mutate,'choice-lock.json','choices_sha256')
suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in [FiniteCases,EffectiveSemanticAudit]])
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
