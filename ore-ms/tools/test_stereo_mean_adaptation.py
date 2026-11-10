from pathlib import Path
import tempfile
import unittest

import numpy as np
import torch

from stereo_mean_adaptation import PREFIX, configure, deny_evaluation_data, objective, sparse_target


class MeanAdaptationTest(unittest.TestCase):
    def test_sparse_target_sign_units_and_right_support(self):
        depth = np.array([[0, 256, 512, 1024]], dtype=np.uint16)
        target, valid = sparse_target(depth, 2, 1)
        np.testing.assert_array_equal(target, [[0, -2, -1, -.5]])
        np.testing.assert_array_equal(valid, [[False, False, True, True]])
        with self.assertRaises(ValueError):
            sparse_target(depth.astype(float), 2, 1)

    def test_loss_uses_only_horizontal_label_mask(self):
        pred = torch.tensor([[[[1., 100.]], [[-55., 55.]]]], requires_grad=True)
        loss = objective(pred, torch.tensor([[0., -10.]]), torch.tensor([[True, False]]))
        self.assertEqual(float(loss), .5)
        loss.backward()
        torch.testing.assert_close(pred.grad, torch.tensor([[[[1., 0.]], [[0., 0.]]]]))
        with self.assertRaises(ValueError):
            objective(pred, torch.zeros((1, 2)), torch.zeros((1, 2), dtype=torch.bool))

    def test_exact_parameter_policy_and_horizontal_update(self):
        model = torch.nn.Module(); model.memory_decoder = torch.nn.Module()
        model.memory_decoder.update_block = torch.nn.Module()
        model.memory_decoder.update_block.flow_head = torch.nn.Module()
        model.memory_decoder.update_block.flow_head.conv2 = torch.nn.Conv2d(1, 2, 1)
        model.memory_decoder.cov_update = torch.nn.Linear(1, 1)
        before = {n: v.clone() for n, v in model.state_dict().items()}
        self.assertEqual(configure(model), [PREFIX+'weight', PREFIX+'bias'])
        optimizer = torch.optim.Adam([p for p in model.parameters() if p.requires_grad], lr=1e-5, weight_decay=0)
        pred = model.memory_decoder.update_block.flow_head.conv2(torch.ones((1, 1, 1, 2)))
        objective(pred, torch.ones((1, 2))*10, torch.ones((1, 2), dtype=torch.bool)).backward()
        for p in model.parameters():
            if p.requires_grad:
                p.grad[1].zero_()
        optimizer.step()
        for n, v in model.state_dict().items():
            if n.startswith(PREFIX):
                self.assertTrue(torch.equal(v[1], before[n][1]))
                self.assertFalse(torch.equal(v[0], before[n][0]))
            else:
                self.assertTrue(torch.equal(v, before[n]))

    def test_excluded_pixel_and_prior_score_reads_denied(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'held.png'; path.write_bytes(b'held')
            good = Path(d)/'training.png'; good.write_bytes(b'train')
            score = Path(d)/'scores-old.json'; score.write_text('{}')
            row = {k: {'path': str(path)} for k in ['left', 'right', 'depth']}
            data = {'records': [{'condition': 'rain', 'rows': [row]}]}
            with deny_evaluation_data(data):
                self.assertEqual(good.read_bytes(), b'train')
                for p in [path, score]:
                    with self.assertRaises(ValueError):
                        p.read_bytes()
            self.assertEqual(path.read_bytes(), b'held')


if __name__ == '__main__':
    unittest.main()
