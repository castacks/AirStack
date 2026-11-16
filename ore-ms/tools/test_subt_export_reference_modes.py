"""Reference-present/null regression plus semantic missing-label controls."""
import contextlib
import copy
import csv
import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import qualify_subt_export as qualifier
import audit_subt_export as auditor

ROOT = Path(__file__).resolve().parents[1]


class ReferenceModes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (ROOT / 'results').mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix='subt-reference-tests-', dir=ROOT / 'results')
        cls.base = Path(cls.tmp.name)
        source = ROOT / 'results/outdoornight-qualification/source'
        for name in ['intrinsics.yaml', 'extrinsics.yaml']:
            (cls.base / name).write_bytes((source / name).read_bytes())
        cls.start = 1_600_000_000_000_000_000
        fields = ['timestamp', 'q_x', 'q_y', 'q_z', 'q_w', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z', 'lin_acc_x', 'lin_acc_y', 'lin_acc_z']
        imu = io.StringIO(); writer = csv.writer(imu); writer.writerow(fields)
        poses = io.StringIO(); pw = csv.writer(poses); pw.writerow(['timestamp', 'x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w'])
        for i in range(1151):
            q = Rotation.from_rotvec([0, 0, .001 * i]).as_quat()
            writer.writerow([cls.start + i * 10_000_000, *q, 0, 0, .1, 0, 0, 9.81])
            pw.writerow([cls.start + i * 10_000_000, 0, 0, 0, *q])
        with zipfile.ZipFile(cls.base / 'reference.zip', 'w') as z:
            z.writestr('ref/imu_data.csv', imu.getvalue()); z.writestr('ref/odometry_data.csv', poses.getvalue())
        with zipfile.ZipFile(cls.base / 'images.zip', 'w') as z:
            for folder, shape, dtype in [('cam_0', (480, 640, 3), np.uint8), ('thermal', (512, 640), np.uint16)]:
                image = (np.arange(np.prod(shape)).reshape(shape) % 200).astype(dtype)
                ok, png = cv2.imencode('.png', image); assert ok
                for i in range(24): z.writestr(f'fixture/{folder}/{i}.png', png.tobytes())
                z.writestr(f'fixture/{folder}/timestamps.txt', '\n'.join(str(cls.start + i * 500_000_000) for i in range(24)))
            z.writestr('fixture/imu/imu_data.csv', imu.getvalue())
        cls.configs = {}
        for mode in ['present', 'absent']:
            rel = lambda p: str(p.relative_to(ROOT))
            config = {'archive': rel(cls.base / 'images.zip'), 'reference_archive': rel(cls.base / 'reference.zip') if mode == 'present' else None,
                      'intrinsics': rel(cls.base / 'intrinsics.yaml'), 'extrinsics': rel(cls.base / 'extrinsics.yaml'),
                      'protocol': 'docs/research/outdoornight-qualification-protocol-2026-11-16.md',
                      'output': rel(cls.base / mode), 'gray_output': rel(cls.base / (mode + '-gray')), 'prefix': 'fixture'}
            path = cls.base / (mode + '.json'); path.write_text(json.dumps(config)); cls.configs[mode] = path
            with contextlib.redirect_stdout(io.StringIO()):
                qualifier.run(path); auditor.main(path)

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def test_reference_present_retains_labels(self):
        r = json.loads((self.base / 'present/qualification.json').read_text())
        self.assertTrue(r['reference_available']); self.assertTrue(r['observed_axes_time_consistency_pass'])
        self.assertEqual(r['anchors'], 23); self.assertEqual(r['paired_moving_anchors'], 23)
        self.assertLess(r['gyro_moving_p95_deg'], 1e-6)

    def test_null_reference_has_no_accuracy_claim(self):
        r = json.loads((self.base / 'absent/qualification.json').read_text())
        self.assertFalse(r['reference_available']); self.assertEqual(r['input_qualified'], 46)
        self.assertEqual(r['common_reference_anchors'], 0); self.assertIsNone(r['gyro_moving_median_deg'])
        self.assertIsNone(r['export_imu_equal_all_shared'])

    def semantic_audit(self, mutation):
        out = self.base / 'absent'; read = lambda name: json.loads((out / name).read_text())
        report, images, intervals, cal = [read(n) for n in ['qualification.json', 'images.json', 'intervals.json', 'calibration.json']]
        ts = {a: [r['time_ns'] for r in images if r['arm'] == a] for a in ['rgb', 'thermal']}
        extr = yaml.safe_load((self.base / 'extrinsics.yaml').read_text())
        with zipfile.ZipFile(self.base / 'images.zip') as z:
            imu = auditor.parse(z.read('fixture/imu/imu_data.csv'))
        mutation(report, intervals)
        with self.assertRaises(AssertionError):
            auditor.audit_without_reference(out, report, images, intervals, ts, cal, extr, imu)

    def test_reject_invented_reference(self):
        self.semantic_audit(lambda r, rows: rows[0].update(reference_supported=True))

    def test_reject_wrong_sensor_endpoint(self):
        self.semantic_audit(lambda r, rows: rows[0].update(source=rows[0]['target']))

    def test_reject_invented_imu_agreement(self):
        self.semantic_audit(lambda r, rows: r.update(export_imu_equal_all_shared=True))


if __name__ == '__main__': unittest.main()
