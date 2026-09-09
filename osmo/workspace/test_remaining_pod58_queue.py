import ast
import tempfile
import unittest
from pathlib import Path

import yaml

from remaining_pod58_queue import completion_command, prepare


class RemainingQueueTests(unittest.TestCase):
    def test_preserves_production_and_selects_only_outstanding_cells(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as tmp:
            paths = prepare(root, Path(tmp))
            self.assertEqual(len(paths), 35)
            self.assertEqual(len(set(paths)), 35)
            self.assertIn('remaining58_hurricaneurbanl2v1_frontier.yaml', [p.name for p in paths])
            for path in paths:
                spec = yaml.safe_load(path.read_text())
                self.assertEqual(spec['iterations'], 1)
                self.assertEqual(len(spec['environments']), 1)
                self.assertEqual(spec['environment_order'], 'round_robin')
                self.assertEqual(str(spec['env']['ZED_TIME_SLICE_GROUPS']), '8')
                self.assertEqual(spec['env']['ZED_HYDRA_TIME_SLICE'], 'true')
                self.assertEqual(str(spec['env']['ZED_TIME_SLICE_BURST']), '8')
                self.assertEqual(str(spec['env']['SEARCH_MAX_SIM_SECONDS']), '600')
                self.assertEqual(spec['env']['ISAAC_SIM_ACTIVE_GPU'], '2')
                self.assertEqual(spec['env']['OFFBOARD_COMPUTE_GPU'], '2')
                self.assertEqual(spec['env']['ISAAC_SIM_GPU_PHYSICS'], 'false')
                self.assertTrue(spec['record']['required'])
                self.assertEqual(spec['on_step_failure'], 'abort_iteration')
                takeoffs = [s['action'] for s in spec['steps'] if s.get('action', {}).get('task') == 'takeoff']
                self.assertEqual(len(takeoffs), 1)
                self.assertEqual(takeoffs[0]['timeout_s'], 900)
                self.assertEqual(takeoffs[0]['feedback_timeout_s'], 900)
                guards = [s['run'] for s in spec['steps'] if 'benchmark_completion_guard' in s.get('run', {}).get('cmd', '')]
                self.assertEqual(len(guards), 2)
                self.assertTrue(all(g['timeout_s'] == 21600 for g in guards))

            retry_names = {p.stem for p in paths if p.stem.startswith('remaining58_retry_')}
            self.assertEqual(retry_names, {
                'remaining58_retry_tornadourbanl1v1_vlfm',
                'remaining58_retry_tornadourbanl2v1_conavgpt2_team',
                'remaining58_retry_tornadourbanl3v1_lawnmower',
            })

    def test_completion_guard_requires_true_latched_message(self):
        for team in (True, False):
            cmd = completion_command(team)
            python = cmd.split("<<'PY'\n", 1)[1].rsplit('\nPY', 1)[0]
            ast.parse(python)
            self.assertIn('TRANSIENT_LOCAL', python)
            self.assertIn('done[0] or msg.data', python)
            self.assertIn("raise SystemExit('RUN_DID_NOT_COMPLETE')", python)
            self.assertNotIn('ros2 topic echo', cmd)


if __name__ == '__main__':
    unittest.main()
