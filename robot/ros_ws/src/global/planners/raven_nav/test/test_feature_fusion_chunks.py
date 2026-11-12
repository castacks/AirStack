"""Full feature fusion must remain bit-identical across channel chunks."""
import numpy as np
import pytest
from raven_nav.semantic_map import SemanticMapFusion


def original_arrays(fusion):
    sources = list(fusion.sources.values())
    keys = np.concatenate([source[0] for source in sources])
    _, inverse, count = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    total = np.zeros((len(count), sources[0][1].shape[1]), dtype=np.float64)
    offset = 0
    for _, values, _ in sources:
        total[inverse[offset:offset+len(values)]] += values
        offset += len(values)
    merged = (total / count[:, None]).astype(np.float32)
    return merged[:, :3], merged[:, 3:]


@pytest.mark.parametrize('dimensions', [2, 60, 61, 64, 100, 1156])
def test_full_feature_fusion_exact_after_updates_and_pruning(dimensions):
    rng = np.random.default_rng(119)
    fusion = SemanticMapFusion(.5)
    labels = [f'f{i}' for i in range(dimensions)]
    for stamp in [1, 2]:
        for source in ['r1', 'r2', 'r3']:
            xyz = rng.integers(-8, 9, (151, 3)).astype(np.float32) * .5
            values = rng.normal(size=(151, dimensions)).astype(np.float32)
            fusion.update(source, xyz, values, labels, stamp)
            expected = original_arrays(fusion)
            actual = fusion.arrays()
            np.testing.assert_array_equal(actual[0], expected[0])
            np.testing.assert_array_equal(actual[1], expected[1])
            assert fusion.arrays() is actual
