import unittest
import numpy as np
from depth_mean_ablation_preflight import label_coverage, pose_error


class PreflightTest(unittest.TestCase):
    def test_rejects_other_sequence_and_nonconsecutive_stem(self):
        members = ['ore-ms/results/a/labels/other/rgb/depth/000000.png',
                   'ore-ms/results/a/labels/target/rgb/depth/000100.png',
                   'ore-ms/results/a/labels/target/thr/depth/000000.png']
        result = label_coverage('target', ['000000', '000001'], members)
        self.assertEqual(result['expected_own_camera_maps'], 4)
        self.assertEqual(result['available_exact_maps'], ['target/thr/depth/000000.png'])
        self.assertEqual(len(result['missing_exact_maps']), 3)

    def test_pose_tolerance_components_and_quaternion_sign(self):
        a = [0, 0, 0, 0, 0, 0, 1]
        self.assertEqual(pose_error(a, [0, 0, 0, 0, 0, 0, -1]), {'translation_m': 0., 'rotation_deg': 0.})
        angle = np.deg2rad(.02)
        e = pose_error(a, [.002, 0, 0, 0, 0, np.sin(angle/2), np.cos(angle/2)])
        self.assertAlmostEqual(e['translation_m'], .002)
        self.assertAlmostEqual(e['rotation_deg'], .02)


if __name__ == '__main__':
    unittest.main()
