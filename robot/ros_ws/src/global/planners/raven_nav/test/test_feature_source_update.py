"""Compare source replacement with the original full-concatenation algorithm."""
import numpy as np
import pytest
from raven_nav.semantic_map import SemanticMapFusion


def legacy_update(fusion, source, xyz, scores, labels, stamp):
    # Original source-row selection/copying, retained as an independent oracle.
    labels = tuple(labels)
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    scores = np.asarray(scores, dtype=np.float32)
    if stamp <= fusion.versions.get(source, -1):
        return False
    valid = np.isfinite(xyz).all(axis=1) & np.isfinite(scores).all(axis=1)
    xyz, scores = xyz[valid], scores[valid]
    keys = np.floor(xyz.astype(np.float64) / fusion.resolution).astype(np.int64)
    old = fusion.sources.get(source)
    if old is not None:
        keys = np.concatenate([old[0], keys])
        values = np.concatenate([old[1], np.c_[xyz, scores]])
    else:
        values = np.c_[xyz, scores]
    _, reverse_idx = np.unique(keys[::-1], axis=0, return_index=True)
    idx = len(keys) - 1 - reverse_idx
    fusion.sources[source] = keys[idx], values[idx], labels
    fusion.versions[source] = stamp
    fusion._cached = None
    return True


@pytest.mark.parametrize('columns', [2, 100, 1156])
def test_incremental_source_rows_are_bit_identical(columns):
    rng = np.random.default_rng(77)
    actual, expected = SemanticMapFusion(), SemanticMapFusion()
    labels = [str(i) for i in range(columns)]
    for revision in range(1, 13):
        source = 'robot_' + str(revision % 3)
        xyz = rng.integers(-8, 9, (80, 3)).astype(np.float32) * .25
        scores = rng.standard_normal((80, columns), dtype=np.float32)
        # Same-cell duplicates, finite filtering, old rows retained after pruning.
        xyz[-1] = xyz[0]
        if revision % 3 == 0:
            scores[1, 0] = np.nan
            xyz[2, 0] = np.inf
        if revision % 4 == 0:
            xyz, scores = xyz[:8], scores[:8]
        if revision == 11:
            xyz, scores = xyz[:0], scores[:0]
        assert actual.update(source, xyz, scores, labels, revision)
        assert legacy_update(expected, source, xyz, scores, labels, revision)
        assert actual.versions == expected.versions
        for name in actual.sources:
            np.testing.assert_array_equal(actual.sources[name][0], expected.sources[name][0])
            # Compare the float32 bit patterns, including signed zero.
            np.testing.assert_array_equal(actual.sources[name][1].view(np.uint32),
                                          expected.sources[name][1].view(np.uint32))
        for got, want in zip(actual.arrays(), expected.arrays()):
            np.testing.assert_array_equal(got.view(np.uint32), want.view(np.uint32))
        cached = actual._cached
        assert not actual.update(source, xyz, scores, labels, revision)
        assert actual._cached is cached


@pytest.mark.parametrize('has_invalid', [False, True])
def test_source_rows_do_not_alias_caller_arrays(has_invalid):
    fusion = SemanticMapFusion()
    xyz = np.array([[0., -0., 0.], [.5, 0., 0.]], dtype=np.float32)
    scores = np.array([[1., -0.], [2., 3.]], dtype=np.float32)
    if has_invalid:
        scores[1, 0] = np.nan
    fusion.update('r1', xyz, scores, ['a', 'b'], 1)
    saved = fusion.sources['r1'][1].copy()
    xyz[:] = 900
    scores[:] = 900
    np.testing.assert_array_equal(fusion.sources['r1'][1].view(np.uint32), saved.view(np.uint32))
