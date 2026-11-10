from types import SimpleNamespace
import unittest
from unittest.mock import patch

import torch

import evaluate_stereo_mean as evaluation
from Module.Frontend.Frontend import FlowFormerCovFrontend as Frontend


class Stub(torch.nn.Module):
    def __init__(self):
        super().__init__(); self.offset = 1.
    def inference(self, a, b):
        shape = (len(a), 2, a.shape[2], a.shape[3])
        return torch.full(shape, self.offset), torch.full(shape, 8.)


def initialize(frontend, config):
    frontend.config = config; frontend.model = Stub()


class FrontendControlTest(unittest.TestCase):
    def run_case(self, arm):
        def change(model):
            model.offset = 9.
        with patch.object(Frontend, '__init__', initialize), \
             patch.object(Frontend, 'inference_2_depth', staticmethod(lambda f, c, *_: (f, c))), \
             patch.object(Frontend, 'inference_2_match', staticmethod(lambda f, c: (f, c))), \
             patch.object(evaluation, 'load_mean_head', change), evaluation.mean_frontend(arm) as receipts:
            frontend = Frontend(SimpleNamespace(device='cpu', enforce_positive_disparity=False))
            frame = SimpleNamespace(imageL=torch.zeros((1, 3, 8, 8)), imageR=torch.ones((1, 3, 8, 8)))
            depth, match = frontend.estimate_pair(frame, frame)
            initial = frontend.estimate_depth(frame)
            return depth, match, initial, receipts

    def test_changed_mean_does_not_replace_temporal_or_variance_outputs(self):
        depth, match, initial, receipts = self.run_case('adapted')
        self.assertTrue((depth[0] == 9).all())
        self.assertTrue((initial[0] == 9).all())
        self.assertTrue((match[0] == 1).all())
        self.assertTrue((depth[1] == 8).all())
        self.assertTrue((match[1] == 8).all())
        self.assertEqual(len(receipts), 2)

    def test_zero_change_copy_preserves_all_outputs(self):
        depth, match, initial, receipts = self.run_case('passthrough')
        self.assertTrue(torch.equal(depth[0], match[0]))
        self.assertTrue(torch.equal(initial[0], depth[0]))
        self.assertTrue(all(r['zero_copy_equal'] for r in receipts))


if __name__ == '__main__':
    unittest.main()
