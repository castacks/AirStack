"""Gate checks on independently generated evidence and reference boundary behavior."""
import ast
import csv
import json
import math
import os
from pathlib import Path
import unittest

import numpy as np

import reference as ref

HERE = Path(__file__).resolve().parent


def evidence(name):
    return json.loads((HERE / 'evidence' / f'{name}_checks.json').read_text())


class VerificationTests(unittest.TestCase):
    def test_reference_import_boundary_and_limits(self):
        declared = {'math', 'numpy', 'scipy'}
        source = ast.parse((HERE/'reference.py').read_text())
        imports = {alias.name.split('.')[0] for node in ast.walk(source)
                   if isinstance(node, ast.Import) for alias in node.names}
        imports |= {node.module.split('.')[0] for node in ast.walk(source)
                    if isinstance(node, ast.ImportFrom)}
        self.assertTrue(imports <= declared)
        self.assertEqual(ref.transfer(4, [(1, 0, 0)]), (1, 4, 0))
        r,t = ref.two_stream(1,1)
        self.assertAlmostEqual(r+t,1,places=12)
        self.assertEqual(ref.ou(0,6,1),(1,0))
        self.assertTrue(ref.raster(1,1,ref.square(0,0,1,1)))
        self.assertFalse(ref.raster(1.0001,1,ref.square(0,0,1,1)))

    def test_transfer_and_detectors(self):
        past = evidence('legacy')
        self.assertLess(max(past['rgb_absolute_errors']),1e-11)
        self.assertLess(past['lwir_absolute_error'],.005)
        rgb = evidence('rgb')
        self.assertEqual(len(rgb['rows']),24)
        self.assertLess(rgb['max_absolute'],1e-11)
        self.assertLess(rgb['spectral_error'],1e-11)
        for row in rgb['noise']:
            variance = row['expected_variance_e2']
            self.assertLess(abs(row['variance_e2']/variance-1),.08)
            self.assertLess(abs(row['mean_e']-row['level_e']),
                            5*math.sqrt(variance/20000)+150/4095)
        self.assertEqual(rgb['detector_rails'],[[0,False],[4081,False],
                                               [4095,True],[4095,True]])
        lwir = evidence('lwir')
        self.assertLess(lwir['planck_relative_max'],1e-12)
        errors = [row['absolute_error'] for row in lwir['refinements']]
        self.assertLess(errors[-1],.005)
        self.assertLess(errors[-1]/errors[-2],.30)
        self.assertLess(max(abs(row[1]-row[2]) for row in lwir['crossover']),1e-10)
        self.assertEqual([row[1:] for row in lwir['detector_rails']],[
            [0,True,False],[0,False,False],[1588,False,False],[4095,False,False],
            [4095,False,True]])

    def test_geometry_environment_and_coupling(self):
        geo = evidence('geometry')
        self.assertEqual(geo['rays'],3300)
        self.assertEqual(geo['first_hit_mismatch'],0)
        self.assertLess(geo['max_range_m'],1e-10)
        self.assertLess(geo['max_projection_px'],1e-10)
        ou = evidence('stochastic')
        self.assertGreaterEqual(ou['n'],20000)
        self.assertLess(max(max(row[1:]) for row in ou['transition_errors']),1e-12)
        self.assertLess(max(abs(a-b) for a,b in zip(ou['covariance_row'],
                        ou['reference_covariance']))/1.1**2,.04)
        control_variance = ou['conditional_reference'][1]
        self.assertLess(abs(ou['conditional_variance']/control_variance-1),.04)
        self.assertLess(abs(ou['means'][3]-ou['conditional_reference'][0])/math.sqrt(
            control_variance),.04)
        biases = [row[-1] for row in ou['euler']]
        self.assertTrue(all(a>b for a,b in zip(biases,biases[1:])))
        self.assertTrue(math.isfinite(ou['long_run_max_abs']))
        clear, degraded, clear_again = ou['clear_degraded_clear_burden']
        self.assertEqual(clear,clear_again)
        self.assertLess(clear,degraded)
        combined = evidence('multispectral')
        self.assertEqual(len(combined['cases']),7)
        for row in combined['cases']:
            self.assertEqual(row['length_m'],row['reference_hit_depth_m'])
            self.assertEqual(row['material'],row['reference_material'])
            for actual, expected in zip(row['actual'],row['reference']):
                self.assertLess(abs(actual-expected),max(.005,.003*abs(expected)))
            for actual, expected in zip(row['detector_codes'],row['independent_detector_codes']):
                self.assertLessEqual(abs(actual-expected),1)
        self.assertLess(combined['advected_burden_mapping_error'],1e-12)
        self.assertLess(combined['isothermal_residual'],1e-11)
        target, block = combined['cases'][1:3]
        self.assertTrue(target['target_center_visible'])
        self.assertFalse(block['target_center_visible'])
        self.assertTrue(all(x>y for x,y in zip(block['RGB550_LWIR11_transmission'],
                                               target['RGB550_LWIR11_transmission'])))

    def test_posterior_and_forecast_outcomes(self):
        posterior = evidence('posterior')
        self.assertLess(np.ptp(posterior['range_extinction_transmission']),1e-14)
        for row in posterior['rows']:
            self.assertGreaterEqual(row['scored'],128)
            self.assertLess(row['max_burden_discrepancy'],.015)
        forecast = evidence('forecast')
        self.assertEqual(forecast['n'],4096)
        self.assertLess(forecast['horizon_covariance_error'],1e-11)
        self.assertLess(forecast['projection_error'],1e-11)
        learned = evidence('learned')
        self.assertTrue(learned['same_issue_scores_before_future_shift'])
        self.assertTrue(all(s['generated']==256 for s in learned['scores'].values()))

    def test_fusion_accounting(self):
        fusion = evidence('fusion')
        self.assertEqual(fusion['independent_clusters'],48)
        self.assertEqual(fusion['frames'],48*3*9*4)
        self.assertEqual(set(fusion['summary']),{'B01','B02','B03','B04','A01'})
        decision_path = os.environ.get('M11_DECISIONS')
        if decision_path is None:
            self.skipTest('provide regenerated decision CSV through M11_DECISIONS')
        with Path(decision_path).open() as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(len(rows),5*fusion['frames'])
        for arm,summary in fusion['summary'].items():
            selection = [r for r in rows if r['arm']==arm]
            self.assertEqual(len(selection),fusion['frames'])
            self.assertEqual(sum(r['issued']=='True' and r['correct']=='True'
                                 for r in selection),summary['correct_issued'])
            self.assertEqual(sum(r['issued']=='True' and r['feasible']=='False'
                                 for r in selection),summary['negative_scene_issues'])


if __name__=='__main__':
    unittest.main()
