import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml
from prepare_raven_camera_sweep import prepare

ROOT = Path(__file__).resolve().parents[2]
module_spec = importlib.util.spec_from_file_location(
    'camera_probe', ROOT / 'scripts/measure_camera_sim_window.py')
probe = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(probe)


class CameraSweepTests(unittest.TestCase):
    def test_idle_window_is_in_denominator(self):
        result = probe.summarize({'rgb': [1, 1, 1.03, 4, 4.03]}, 50, 1000)
        self.assertEqual(result['rgb']['unique_frames'], 4)
        self.assertAlmostEqual(result['rgb']['sim_fps'], 0.08)
        self.assertAlmostEqual(result['rgb']['wall_fps'], 0.004)

    def test_candidates_change_only_camera_schedule_and_diagnostic_envelope(self):
        source = ROOT / 'osmo/missions/raven_firesuburbanl1v1_raven_remaining_2gpu1.yaml'
        baseline = yaml.safe_load(source.read_text())
        with tempfile.TemporaryDirectory() as tmp:
            paths = prepare(source, Path(tmp), 2, 3)
            self.assertEqual(len(paths), 4)
            for path, skips in zip(paths, (4, 8, 16, 24)):
                candidate = yaml.safe_load(path.read_text())
                self.assertNotIn('nas_dest', candidate)
                self.assertEqual(candidate['environments'], baseline['environments'])
                self.assertEqual(candidate['record'], baseline['record'])
                self.assertEqual(candidate['env']['ZED_TIME_SLICE_GROUPS'], str(8 + skips))
                self.assertEqual(candidate['env']['ZED_TIME_SLICE_BURST'], '8')
                for key in ('ENABLE_LIDAR', 'ZED_PIPELINE', 'ZED_WIDTH', 'ZED_HEIGHT', 'RAYFRONTS_CONFIG'):
                    self.assertEqual(candidate['env'].get(key), baseline['env'].get(key))
                search = next(s['action'] for s in candidate['steps']
                              if s.get('action', {}).get('task') == 'semantic_search')
                self.assertEqual(search['goal']['max_sim_seconds'], 50)
                search['goal']['max_sim_seconds'] = 600.0
                probe_steps = [s for s in candidate['steps'] if 'raven_camera_benchmark' in s.get('run', {}).get('cmd', '')]
                self.assertEqual(len(probe_steps), 2)
                self.assertEqual([s for s in candidate['steps'] if s not in probe_steps], baseline['steps'])


if __name__ == '__main__':
    unittest.main()
