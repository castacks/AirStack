import unittest
import numpy as np
import torch

from mean_descent_contract import armijo_step, convex_upsample, residual_prediction, total_loss, zero_head


def fixture():
    return {'hidden': torch.ones((1, 256, 2, 2), dtype=torch.float64),
            'probability': torch.full((1, 9, 8, 8, 2, 2), 1/9, dtype=torch.float64),
            'base': torch.ones((1, 1, 16, 16), dtype=torch.float64),
            'target': torch.zeros((16, 16), dtype=torch.float64),
            'valid': torch.ones((16, 16), dtype=torch.bool)}


class DescentContractTest(unittest.TestCase):
    def test_native_convex_upsampling_against_scalar_neighbors(self):
        flow = torch.arange(1., 13., dtype=torch.float64).reshape(1, 1, 3, 4)
        probability = torch.arange(1., 9*8*8*3*4+1, dtype=torch.float64).reshape(1, 9, 8, 8, 3, 4)
        probability /= probability.sum(1, keepdim=True)
        result = convex_upsample(flow, probability).numpy()
        expected = np.zeros((1, 1, 24, 32))
        for y in range(24):
            for x in range(32):
                cy, cx = y//8, x//8
                expected[0, 0, y, x] = sum(float(flow[0, 0, yy, xx])*8*float(probability[0, (yy-cy+1)*3+(xx-cx+1), y%8, x%8, cy, cx])
                    for yy in range(cy-1, cy+2) for xx in range(cx-1, cx+2) if 0 <= yy < 3 and 0 <= xx < 4)
        np.testing.assert_allclose(result, expected, atol=1e-13)

    def test_zero_residual_is_exact_identity(self):
        cache = fixture(); head = zero_head('cpu', torch.float64)
        self.assertTrue(torch.equal(residual_prediction(head, cache), cache['base']))
        total_loss(head, [cache], True)
        self.assertTrue(all(p.grad is not None and torch.isfinite(p.grad).all() for p in head))
        self.assertIsNone(cache['hidden'].grad)
        self.assertIsNone(cache['base'].grad)

    def test_directional_derivative_of_fixed_feature_objective(self):
        cache = fixture(); cache['base'] *= .5
        head = zero_head('cpu', torch.float64); total_loss(head, [cache], True)
        norm = torch.sqrt(sum(p.grad.square().sum() for p in head))
        directions = [-p.grad.clone()/norm for p in head]
        epsilon = 1e-6; losses = []
        with torch.no_grad():
            for sign in [-1, 1]:
                for p, d in zip(head, directions):
                    p.copy_(sign*epsilon*d)
                losses.append(total_loss(head, [cache]))
        derivative = (losses[1]-losses[0])/(2*epsilon)
        self.assertAlmostEqual(derivative, -float(norm), places=6)

    def test_actual_objective_decreases_with_backtracking(self):
        head = zero_head('cpu', torch.float64); cache = fixture()
        result = armijo_step(head, [cache], initial_step=1.)
        self.assertGreater(len(result['trials']), 1)
        self.assertLess(result['after'], result['before'])
        self.assertFalse(result['trials'][0]['accepted'])
        self.assertTrue(result['trials'][-1]['accepted'])

    def test_rejected_step_restores_parameters(self):
        head = zero_head('cpu', torch.float64); original = [p.clone() for p in head]
        with self.assertRaises(ValueError):
            armijo_step(head, [fixture()], initial_step=1., max_trials=1)
        self.assertTrue(all(torch.equal(a, b) for a, b in zip(head, original)))


if __name__ == '__main__':
    unittest.main()
