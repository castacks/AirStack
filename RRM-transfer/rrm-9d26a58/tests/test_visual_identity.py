"""Synthetic identity completeness checks, not learned negative-case results."""
import unittest

from rrm.contracts import Truth
from rrm.visual_world_builder import complete_visual_identities, parse_visual_candidate
from test_visual_world_builder import _context, _ready


class IdentityTests(unittest.TestCase):
    def snapshot(self, predicates=('exists', 'kind'), truth='TRUE'):
        claims = [dict(subject='red_crate', predicate=key,
                       obj='crate' if key == 'kind' else None, truth=truth) for key in predicates]
        return parse_visual_candidate(_ready(claims), _context()).snapshot

    def qualify(self, snapshot, now=10.2, localized=False, catalog=None):
        return complete_visual_identities(snapshot, entity_catalog=catalog or _context().entity_catalog,
                                           now_monotonic_s=now, require_localized=localized)

    def test_complete_identity_is_not_localization(self):
        self.assertEqual(self.qualify(self.snapshot()), ('red_crate',))
        self.assertEqual(self.qualify(self.snapshot(), localized=True), ())
        self.assertEqual(self.qualify(self.snapshot(('exists', 'kind', 'localized')), localized=True), ('red_crate',))

    def test_partial_identity_never_filled_from_catalog(self):
        for predicates in (('exists',), ('kind',), ('exists', 'localized'), ('kind', 'localized')):
            with self.subTest(predicates=predicates):
                self.assertEqual(self.qualify(self.snapshot(predicates)), ())

    def test_false_unknown_contradictory_and_stale_do_not_qualify(self):
        for truth in ('FALSE', 'UNKNOWN'):
            self.assertEqual(self.qualify(self.snapshot(truth=truth)), ())
        original = self.snapshot()
        conflicting = original.model_copy(update={'evidence': original.evidence +
            (original.evidence[0].model_copy(update={'truth': Truth.FALSE}),)})
        self.assertEqual(self.qualify(conflicting), ())
        self.assertEqual(self.qualify(original, now=11.21), ())
        self.assertEqual(self.qualify(original, now=10.1), ())

    def test_catalog_mismatch_or_unsupported_entity_never_qualifies(self):
        original = self.snapshot()
        self.assertEqual(self.qualify(original, catalog={'red_crate': 'marker'}), ())
        self.assertEqual(self.qualify(original, catalog={'absent_marker': 'marker'}), ())

    def test_two_complete_identities_are_not_unique_target_selection(self):
        a = self.snapshot()
        b = parse_visual_candidate(_ready([
            dict(subject='loading_bay_marker', predicate='exists', obj=None, truth='TRUE'),
            dict(subject='loading_bay_marker', predicate='kind', obj='marker', truth='TRUE'),
        ]), _context()).snapshot
        both = a.model_copy(update={'evidence': a.evidence + b.evidence})
        self.assertEqual(self.qualify(both), ('loading_bay_marker', 'red_crate'))

    def test_invalid_clock_and_catalog_rejected(self):
        for clock in (True, float('inf'), float('nan'), -1.):
            with self.assertRaises(ValueError): self.qualify(self.snapshot(), now=clock)
        for catalog in ({}, {'red_crate': ''}, {'red_crate': 1}):
            with self.assertRaises(ValueError):
                complete_visual_identities(self.snapshot(), entity_catalog=catalog, now_monotonic_s=10.2)


if __name__ == '__main__': unittest.main()
