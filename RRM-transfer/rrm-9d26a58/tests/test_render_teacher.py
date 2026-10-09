"""Renderer adapter boundary tests; synthetic labels, not live renderer coverage."""
import copy
import unittest

from test_visual_evaluation import fixture
from rrm.render_teacher import build_render_pair, reference_stamp_ns, semantic_regions, qualify_probe_colors


def payload():
    value = fixture()
    return dict(image=value['image'], semantic_pixels=[[0] * 48 for _ in range(30)],
                id_to_labels={'0': {'class': 'BACKGROUND'}, '1': {'class': 'blue_marker'},
                              '2': {'class': 'orange_marker'}},
                stage_entities=[dict(e, exists=True) for e in value['registration']['entities']],
                manifest=value['manifest'], launcher_sha256=value['launcher_sha256'],
                producer_ref='synthetic-renderer', episode_id='isolated-test', scene_revision='r1',
                reference_numerator=1, reference_denominator=60)


class RenderTeacherTests(unittest.TestCase):
    def test_frozen_color_gate_accepts_saturated_colors_rejects_legacy_gold(self):
        self.assertTrue(qualify_probe_colors({'blue_marker': [[0, 30, 220]],
                                              'orange_marker': [[220, 70, 0]]})['passed'])
        self.assertFalse(qualify_probe_colors({'orange_marker': [[249, 217, 143]]})['passed'])
        self.assertFalse(qualify_probe_colors({'blue_marker': [[249, 248, 249]]})['passed'])
        good, bad = [0, 30, 220], [249, 248, 249]
        self.assertTrue(qualify_probe_colors({'blue_marker': [good] * 9 + [bad]})['passed'])
        self.assertFalse(qualify_probe_colors({'blue_marker': [good] * 8 + [bad] * 2})['passed'])
        for samples in ({}, {'wrong': [[0, 0, 255]]}, {'blue_marker': [[True, 0, 255]]}):
            with self.assertRaises(ValueError): qualify_probe_colors(samples)

    def test_rational_time_uses_integer_truncation_not_wall_clock(self):
        self.assertEqual(reference_stamp_ns(1, 60), 16666666)
        self.assertEqual(reference_stamp_ns(1000000000001, 1000000000), 1000000000001)
        for numerator, denominator in ((-1, 60), (1, 0), (True, 2), (1, 2.0)):
            with self.assertRaises(ValueError):
                reference_stamp_ns(numerator, denominator)

    def test_actual_pixel_extent_produces_half_open_region(self):
        value = payload()
        value['semantic_pixels'][2][3] = 1
        value['semantic_pixels'][4][7] = 1
        original = copy.deepcopy(value)
        pair = build_render_pair(**value)
        self.assertEqual(pair['pairing_report']['status'], 'BOUND_FOR_ASSESSMENT')
        blue, orange = pair['teacher']['labels']
        self.assertEqual(blue['image_region'], [3, 2, 8, 5])
        self.assertEqual(blue['visibility'], 'VISIBLE')
        self.assertEqual(orange['visibility'], 'UNKNOWN')
        self.assertTrue(orange['exists'])
        self.assertFalse(pair['execution_dispatch'])
        self.assertEqual(value, original)

    def test_measured_absence_not_missing_pixels_establishes_absence(self):
        value = payload(); value['stage_entities'][0]['exists'] = False
        pair = build_render_pair(**value)
        self.assertEqual(pair['teacher']['labels'][0]['visibility'], 'ABSENT')
        value['semantic_pixels'][0][0] = 1
        with self.assertRaises(ValueError): build_render_pair(**value)

    def test_stage_identity_completeness_and_duplicates(self):
        for change in ('missing', 'duplicate', 'kind', 'prim', 'exists'):
            value = payload()
            if change == 'missing': value['stage_entities'].pop()
            elif change == 'duplicate': value['stage_entities'][1] = value['stage_entities'][0]
            else: value['stage_entities'][0][{'kind': 'kind', 'prim': 'scene_prim', 'exists': 'exists'}[change]] = 'wrong'
            with self.subTest(change=change), self.assertRaises(ValueError): build_render_pair(**value)

    def test_malformed_masks_and_unknown_pixel_ids(self):
        for mask in ([], [[]], [[0], [0, 1]], [[True]], [[-1]], [[99]], [['1']]):
            with self.subTest(mask=mask), self.assertRaises(ValueError):
                semantic_regions(mask, {'0': {'class': 'BACKGROUND'}}, {'blue_marker'})
        with self.assertRaises(ValueError):
            semantic_regions([[1]], {'1': {'class': 'blue_marker'}, '2': {'class': 'blue_marker'}}, {'blue_marker'})

    def test_image_and_segmentation_dimensions_must_match(self):
        value = payload(); value['semantic_pixels'] = [[0]]
        with self.assertRaisesRegex(ValueError, 'image_dimensions_mismatch'):
            build_render_pair(**value)

    def test_malformed_semantic_mapping_never_becomes_a_teacher(self):
        for mapping in ({'00': {'class': 'BACKGROUND'}}, {'0': {'class': []}},
                        {'0': None}, {0: {'class': 'BACKGROUND'}}):
            with self.subTest(mapping=mapping), self.assertRaises(ValueError):
                semantic_regions([[0]], mapping, {'blue_marker'})


if __name__ == '__main__':
    unittest.main()
