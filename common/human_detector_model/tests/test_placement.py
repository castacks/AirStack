from dataclasses import replace
import unittest

from human_detector import (BODY_PARTS, Detector, DetectorConfig,
                            from_placement_evidence, parts_from_placement)


class PlacementTests(unittest.TestCase):
    def test_scene_full_exposure_and_unlisted_parts(self):
        full = {'visible_parts': ['feet', 'shins', 'thighs', 'hips', 'torso', 'arms', 'shoulders', 'head']}
        self.assertEqual(parts_from_placement(full), {p: 1.0 for p in BODY_PARTS})
        self.assertEqual(parts_from_placement({'visible_parts': ['head']}),
                         {p: float(p == 'head') for p in BODY_PARTS})

    def test_partial_segmented_legs_are_not_assumed_completely_exposed(self):
        self.assertEqual(parts_from_placement({'visible_parts': ['feet', 'thighs']}),
                         {p: 0.0 for p in BODY_PARTS})
        parts = parts_from_placement({'visible_parts': ['left_thigh', 'left_shin', 'left_foot', 'right_foot']})
        self.assertEqual(parts['left_leg'], 1.0)
        self.assertEqual(parts['right_leg'], 0.0)

    def test_canonical_parts_and_generic_limbs(self):
        for part in BODY_PARTS:
            self.assertEqual(parts_from_placement({'visible_parts': [part]}),
                             {p: float(p == part) for p in BODY_PARTS})
        self.assertEqual(parts_from_placement({'visible_parts': ['arms', 'legs']}),
                         {p: float(p not in ('head', 'torso')) for p in BODY_PARTS})

    def test_no_silent_fallback_to_planned_exposure(self):
        placement = {'planned_visible_parts': ['head']}
        with self.assertRaises(ValueError):
            parts_from_placement(placement)
        self.assertEqual(parts_from_placement(placement, field='planned_visible_parts')['head'], 1.0)
        placement['visible_parts'] = []
        self.assertFalse(any(parts_from_placement(placement).values()))

    def test_missing_and_invalid_metadata_raise(self):
        for value in ({}, {'visible_parts': None}, {'visible_parts': 'head'},
                      {'visible_parts': [True]}, {'visible_parts': ['hed']}, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parts_from_placement(value)
        with self.assertRaises(ValueError):
            parts_from_placement({'pose': ['head']}, field='pose')

    def test_pixels_and_fraction_stay_camera_measurements(self):
        row = {'pixels': 500, 'visibleBodyFraction': .7, 'rangeMeters': 2}
        placement = {'visible_parts': ['head'], 'covered_frac': .9, 'pose': 'lying'}
        a = from_placement_evidence(row, placement)
        b = from_placement_evidence({**row, 'rangeMeters': 100}, {**placement, 'pose': 'standing', 'covered_frac': 0})
        self.assertEqual(a, b)
        self.assertEqual(a.visible_body_fraction, .7)
        self.assertIsNone(a.contrast)
        self.assertIsNone(a.clutter)
        with self.assertRaises(ValueError):
            from_placement_evidence({'pixels': 500}, placement)

    def test_hidden_parts_reduce_probability_and_weight_can_disable_them(self):
        row = {'pixels': 800, 'visibleBodyFraction': .5}
        head = from_placement_evidence(row, {'visible_parts': ['head']})
        arm = from_placement_evidence(row, {'visible_parts': ['left_arm']})
        none = from_placement_evidence(row, {'visible_parts': []})
        d = Detector()
        self.assertGreater(d.probability(head), d.probability(arm))
        self.assertLess(d.probability(none), 1e-7)
        self.assertEqual(d.explain(head)['missing_features'], ['contrast', 'clutter'])
        disabled = Detector(replace(DetectorConfig(), parts_weight=0))
        self.assertEqual(disabled.probability(head), disabled.probability(none))

    def test_zero_pixel_view_overrides_exposed_placement(self):
        o = from_placement_evidence({'pixels': 0, 'visibleBodyFraction': 0}, {'visible_parts': ['head']})
        self.assertFalse(any(o.visible_parts.values()))
        self.assertEqual(Detector().probability(o), 0)


if __name__ == '__main__':
    unittest.main()
