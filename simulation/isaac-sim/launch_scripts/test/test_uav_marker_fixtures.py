"""Passive fixture construction tests; no Kit or live stage imports."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import os
import subprocess
import sys

SPEC = importlib.util.spec_from_file_location(
    'uav_marker_fixtures', Path(__file__).parents[1]/'uav_marker_fixtures.py')
FIXTURES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FIXTURES)


class FixtureSpecTests(unittest.TestCase):
    def test_nominal_preserves_original_geometry_and_color(self):
        case = FIXTURES.fixture_spec()
        red, blue = case['fixtures']
        self.assertEqual(red, dict(id='red_panel', position=[4., 0., 1.2],
                                  color=[1., .02, .02], size_m=[.6]*3))
        self.assertEqual(blue, dict(id='blue_distractor', position=[4., -1.5, 1.2],
                                   color=[.02, .1, 1.], size_m=[.6]*3))
        self.assertFalse(case['rendered_qualified'])

    def test_cases_are_independent_and_outside_bounded_radius(self):
        for name in FIXTURES.SCENARIOS:
            case = FIXTURES.fixture_spec(name)
            self.assertEqual(case['units'], 'metres')
            self.assertEqual(case['frame'], 'world')
            ids = [item['id'] for item in case['fixtures']]
            self.assertEqual(len(ids), len(set(ids)))
            for item in case['fixtures']:
                self.assertGreater(item['position'][0]-item['size_m'][0]/2, 2.)
                self.assertGreater(min(item['size_m']), 0.)
            case['fixtures'][0]['position'][0] = 999.
            self.assertNotEqual(FIXTURES.fixture_spec(name)['fixtures'][0]['position'][0], 999.)
        for name in (None, '', '../nominal', 'NOMINAL', 1):
            with self.assertRaises(ValueError):
                FIXTURES.fixture_spec(name)

    def test_reference_projection_declares_split_partial_and_full_cover(self):
        def bounds(item):
            x, y, z = item['position']
            dx, dy, dz = item['size_m']
            camera_x, _, camera_z = FIXTURES.REFERENCE_CAMERA_M
            distance = x-dx/2-camera_x
            return ((y-dy/2)/distance, (y+dy/2)/distance,
                    (z-dz/2-camera_z)/distance, (z+dz/2-camera_z)/distance)

        for name in ('split-occlusion', 'partial-occlusion', 'full-occlusion'):
            red, _, occluder = FIXTURES.fixture_spec(name)['fixtures']
            left, right, bottom, top = bounds(red)
            gray_left, gray_right, gray_bottom, gray_top = bounds(occluder)
            self.assertLess(gray_left, left)
            self.assertGreater(gray_right, right)
            if name == 'split-occlusion':
                self.assertLess(bottom, gray_bottom)
                self.assertLess(gray_bottom, gray_top)
                self.assertLess(gray_top, top)
            elif name == 'partial-occlusion':
                self.assertLess(bottom, gray_bottom)
                self.assertLess(gray_bottom, top)
                self.assertGreater(gray_top, top)
            else:
                self.assertLess(gray_bottom, bottom)
                self.assertGreater(gray_top, top)

    def test_manifest_is_new_and_does_not_mutate_spec(self):
        case = FIXTURES.fixture_spec('full-occlusion')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'case.json'
            FIXTURES.write_manifest(path, case)
            before = path.read_bytes()
            with self.assertRaises(FileExistsError):
                FIXTURES.write_manifest(path, FIXTURES.fixture_spec())
            self.assertEqual(path.read_bytes(), before)

    def test_invalid_entry_configuration_fails_before_simulator_import(self):
        entry = Path(__file__).parents[1]/'uav_marker_qualification.py'
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory)/'existing.json'
            existing.write_text('preserve')
            for settings, error in (
                    ({'UAV_MARKER_SCENARIO': 'unknown'}, 'unknown UAV'),
                    ({'UAV_MARKER_SCENARIO': 'full-occlusion',
                      'UAV_DIAGNOSTIC_PITCH_RAD': '.1'}, 'zero diagnostic pitch'),
                    ({'UAV_FIXTURE_MANIFEST': str(existing)}, 'output must be new')):
                environment = dict(os.environ)
                environment.update(UAV_MARKER_SCENARIO='nominal', UAV_DIAGNOSTIC_PITCH_RAD='0',
                                   UAV_FIXTURE_MANIFEST='')
                environment.update(settings)
                result = subprocess.run([sys.executable, str(entry)], env=environment,
                                        text=True, capture_output=True, timeout=5)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(error, result.stderr)
                self.assertNotIn('ModuleNotFoundError', result.stderr)
            self.assertEqual(existing.read_text(), 'preserve')


class UsdAuthoringTests(unittest.TestCase):
    def test_private_stage_bounds_colliders_and_materials(self):
        try:
            from pxr import Usd, UsdGeom, UsdPhysics
        except ImportError as error:
            self.skipTest('optional private-stage USD test: '+str(error))
        for name in FIXTURES.SCENARIOS:
            stage = Usd.Stage.CreateInMemory()
            UsdGeom.SetStageMetersPerUnit(stage, 1.)
            case = FIXTURES.fixture_spec(name)
            FIXTURES.author_fixtures(stage, case)
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            for item in case['fixtures']:
                path = '/World/UAVFixtures/'+item['id']
                prim = stage.GetPrimAtPath(path)
                self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI))
                self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))
                box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                for axis in range(3):
                    expected = item['position'][axis]-item['size_m'][axis]/2
                    self.assertAlmostEqual(box.GetMin()[axis], expected, places=7)
                    expected = item['position'][axis]+item['size_m'][axis]/2
                    self.assertAlmostEqual(box.GetMax()[axis], expected, places=7)
                self.assertEqual(str(prim.GetRelationship('material:binding').GetTargets()[0]),
                                 path+'Material')
                shader = stage.GetPrimAtPath(path+'Shader')
                self.assertEqual(shader.GetAttribute('info:id').Get(), 'UsdPreviewSurface')
                for attribute in ('inputs:diffuseColor', 'inputs:emissiveColor'):
                    color = shader.GetAttribute(attribute).Get()
                    for actual, expected in zip(color, item['color']):
                        self.assertAlmostEqual(actual, expected, places=6)


if __name__ == '__main__':
    unittest.main()
