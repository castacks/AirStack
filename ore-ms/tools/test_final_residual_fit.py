"""Adapter integration contracts beyond the existing descent operator tests."""
import unittest
import torch
from final_residual_fit import apply_stereo, residual_frontend
from mean_descent_contract import zero_head


class StereoIntegration(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(3)
        self.flow = torch.randn(2, 2, 16, 24)
        self.capture = {'hidden': torch.randn(2, 256, 2, 3),
                        'mask': torch.randn(2, 576, 2, 3),
                        'head_calls': 12, 'upsample_calls': 24}

    def test_zero_preserves_entire_batch_exactly(self):
        actual = apply_stereo(self.flow, self.capture, zero_head('cpu', torch.float32))
        self.assertTrue(torch.equal(actual, self.flow))

    def test_only_stereo_horizontal_changes(self):
        before = self.flow.clone()
        head = zero_head('cpu', torch.float32)
        with torch.no_grad():
            head[0].fill_(.001); head[1].fill_(.01)
        actual = apply_stereo(self.flow, self.capture, head)
        self.assertFalse(torch.equal(actual[:1, :1], before[:1, :1]))
        self.assertTrue(torch.equal(actual[1:], before[1:]))
        self.assertTrue(torch.equal(actual[:, 1:], before[:, 1:]))
        self.assertTrue(torch.equal(self.flow, before))
        # Temporal features/masks cannot affect the stereo correction.
        self.capture['hidden'][1].fill_(999)
        self.capture['mask'][1].fill_(-999)
        self.assertTrue(torch.equal(actual, apply_stereo(self.flow, self.capture, head)))

    def test_incomplete_decoder_capture_rejected(self):
        self.capture['head_calls'] = 11
        with self.assertRaises(AssertionError):
            apply_stereo(self.flow, self.capture, zero_head('cpu', torch.float32))

    def test_frontend_restored_after_exception(self):
        from Module.Frontend.Frontend import FlowFormerCovFrontend as Frontend
        old = Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair
        with self.assertRaisesRegex(RuntimeError, 'intentional'):
            with residual_frontend('passthrough'):
                self.assertIsNot(Frontend.estimate_pair, old[2])
                raise RuntimeError('intentional')
        self.assertEqual((Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair), old)


if __name__ == '__main__':
    unittest.main()
