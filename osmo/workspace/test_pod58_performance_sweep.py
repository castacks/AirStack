import tempfile
import unittest
from pathlib import Path

import yaml
from prepare_pod58_performance_sweep import prepare
from remaining_pod58_queue import prepare as production


class PerformanceSweepTests(unittest.TestCase):
    def test_matched_candidates_preserve_scene_and_gates(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as tmp:
            source = next(p for p in production(root, Path(tmp) / 'production')
                          if p.stem == 'remaining58_earthquakesuburbanl3v1_lawnmower')
            baseline = yaml.safe_load(source.read_text())
            specs = [yaml.safe_load(p.read_text()) for p in
                     prepare(source, Path(tmp) / 'diagnostics')]
            self.assertEqual(len(specs), 4)
            candidates = []
            for spec in specs:
                self.assertNotIn('nas_dest', spec)
                self.assertTrue(spec['name'].startswith('diagnostic_'))
                self.assertEqual(spec['environments'], baseline['environments'])
                self.assertEqual(spec['record'], baseline['record'])
                self.assertEqual(spec['env']['SEARCH_MAX_SIM_SECONDS'], '50')
                self.assertEqual(spec['env']['ZED_TIME_SLICE_BURST'], '8')
                self.assertEqual(spec['env']['ZED_WIDTH'], baseline['env']['ZED_WIDTH'])
                self.assertEqual(len(spec['steps']), len(baseline['steps']) + 2)
                candidates.append(tuple(spec['env'][key] for key in
                    ('ZED_TIME_SLICE_GROUPS', 'ISAAC_SIM_GPU_PHYSICS',
                     'ISAAC_SIM_FABRIC_SCENE_DELEGATE')))
            self.assertEqual(candidates, [('8', 'false', 'true'),
                ('8', 'false', 'false'), ('32', 'false', 'true'),
                ('8', 'true', 'true')])


if __name__ == '__main__':
    unittest.main()
