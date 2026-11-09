"""Numerical model checks, not empirical camera calibration tests."""
import unittest
import json
from pathlib import Path
import tempfile

import numpy as np

from pose_covariance_replay import (
    covariance_intersection, exp_se3, gaussian_model, information_inverse,
    log_se3, objective_change, register, replay_capture, skew, transport_covariance, validate_data,
)


def fixture(n=24, seed=7):
    rng = np.random.default_rng(seed)
    q = rng.normal(size=(n, 3)) + [4, 0, 0]
    t = exp_se3(np.array([.1, -.04, .02, .02, -.04, .03]))
    p = q@t[:3, :3].T+t[:3, 3]
    cs = np.tile(np.diag([2, 3, 5])*1e-5, (n, 1, 1))
    ct = np.tile(np.diag([7, 2, 4])*1e-5, (n, 1, 1))
    return p, q, cs, ct, t


class NumericalTests(unittest.TestCase):
    def test_replay_preserves_original_prefix_and_terminal_stage(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/'arbiter-events').mkdir()
            uv = np.array([[10, 10], [20, 10], [10, 20], [20, 20]], dtype=float)
            arrays = {'frames//K': np.tile([[100., 0, 15], [0, 100, 15], [0, 0, 1]], (3, 1, 1)),
                      'frames//time_ns': np.array([1000, 2000, 3000], dtype=np.int64),
                      'edge/match2frame1/mapping': np.repeat([0, 1], 4),
                      'edge/match2frame2/mapping': np.repeat([1, 2], 4)}
            for side in [1, 2]:
                arrays[f'match//pixel{side}_uv'] = np.tile(uv, (2, 1))
                arrays[f'match//pixel{side}_d'] = np.tile([2, 3, 4, 3], 2)[:, None]
                arrays[f'match//obs{side}_covTc'] = np.tile(np.eye(3)*1e-4, (8, 1, 1))
            np.savez(root/'tensor-map.npz', **arrays)
            events = [{'kind': 'motion_result_written', 'first_write_for_submission': True,
                       'from_idx': i, 'frame_idx': i+1, 'time0_ns': 1000*(i+1),
                       'time1_ns': 1000*(i+2), 'original_need_interp': False,
                       'stage': 'pair_processing' if i == 0 else 'termination', 'sequence': i,
                       'derived_camera_delta_xyzw': [0, 0, 0, 0, 0, 0, 1]} for i in range(2)]
            (root/'arbiter-events/events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
            short = replay_capture(root, root/'short.json', max_groups=2)
            all_rows = replay_capture(root, root/'all.json')
            self.assertEqual(len(short['rows']), 1)
            self.assertEqual(short['total_source_groups'], 3)
            self.assertEqual(all_rows['rows'][-1]['native_write_stage'], 'termination')
            self.assertEqual(all_rows['status_counts'], {'converged': 2})
            with self.assertRaises(ValueError):
                replay_capture(root, root/'too-long.json', max_groups=4)
            with self.assertRaises(FileExistsError):
                replay_capture(root, root/'short.json', max_groups=2)

    def test_se3_roundtrip_translation_first(self):
        for scale in [0, 1e-9, .001, .3, 1]:
            x = scale*np.array([.1, -.3, .4, -.1, .5, .7])
            np.testing.assert_allclose(log_se3(exp_se3(x)), x, atol=2e-12)

    def test_full_objective_gradient_finite_difference(self):
        p, q, cs, ct, t = fixture()
        t = exp_se3([.01, -.02, .03, .01, .02, -.03])@t
        cost, g, _, _ = gaussian_model(t, p, q, cs, ct)
        numerical = []
        for e in np.eye(6):
            h = 1e-6
            numerical.append((gaussian_model(exp_se3(h*e)@t, p, q, cs, ct)[0]
                              - gaussian_model(exp_se3(-h*e)@t, p, q, cs, ct)[0])/(2*h))
        np.testing.assert_allclose(g, numerical, atol=1e-5, rtol=2e-7)
        self.assertTrue(np.isfinite(cost))

    def test_covariance_derivative_is_not_dropped(self):
        p, q, cs, ct, t = fixture()
        _, g, h, hm = gaussian_model(t, p, q, cs, ct)
        # Zero residual still has a covariance-orientation/logdet gradient.
        self.assertGreater(np.linalg.norm(g[3:]), .01)
        self.assertGreater(np.linalg.norm(h-hm), .01)
        np.testing.assert_allclose(h[:3], hm[:3], atol=1e-8)

    def test_stable_objective_change_matches_direct_gaussian(self):
        p, q, cs, ct, t = fixture()
        step = np.array([.001, -.003, .004, -.002, .001, -.004])
        expected = gaussian_model(exp_se3(step)@t, p, q, cs, ct)[0]-gaussian_model(t, p, q, cs, ct)[0]
        self.assertAlmostEqual(objective_change(t, step, p, q, cs, ct), expected, places=9)

    def test_stable_loss_preserves_sub_ulp_decrease(self):
        q = np.array([[3., 1, 2], [3., 2, 1], [2., 1, 3]])
        p = q.copy()
        p[:, 0] -= 1e-7
        # Large irreducible y residual makes direct total-loss subtraction fail.
        p[:, 1] -= 1e6
        cov = np.tile(np.eye(3)/2, (3, 1, 1))
        step = np.array([-1e-7, 0, 0, 0, 0, 0])
        residual = q-p
        expected = np.sum(step[0]*residual[:, 0]+.5*step[0]**2)
        stable = objective_change(np.eye(4), step, p, q, cov, cov)
        self.assertLess(stable, 0)
        self.assertAlmostEqual(stable, expected, delta=1e-27)
        direct = gaussian_model(exp_se3(step), p, q, cov, cov)[0]-gaussian_model(np.eye(4), p, q, cov, cov)[0]
        self.assertEqual(direct, 0)

    def test_noiseless_isotropic_solution(self):
        p, q, cs, ct, truth = fixture()
        cs[:] = np.eye(3)*1e-4
        ct[:] = np.eye(3)*2e-4
        result = register(p, q, cs, ct)
        self.assertEqual(result['status'], 'converged')
        np.testing.assert_allclose(result['transform'], truth, atol=1e-8)
        self.assertIsNotNone(result['covariance'])
        costs = np.array([x['nll'] for x in result['history']])
        self.assertTrue(np.all(np.diff(costs) <= 1e-9))

    def test_nonconvergence_has_no_covariance(self):
        p, q, cs, ct, _ = fixture()
        r = register(p, q, cs, ct, max_iterations=0)
        self.assertEqual(r['status'], 'iteration_limit')
        self.assertIsNone(r['covariance'])
        self.assertIsNone(r['mean_only_covariance'])

    def test_invalid_covariances_are_rejected_without_clipping(self):
        p, q, cs, ct, _ = fixture()
        for value in [0., -1., np.nan]:
            c = ct.copy()
            c[0, 0, 0] = value
            with self.assertRaises(ValueError):
                register(p, q, cs, c)
        c = ct.copy()
        c[0, 0, 1] = .1
        with self.assertRaises(ValueError):
            register(p, q, cs, c)
        with self.assertRaises(ValueError):
            validate_data(p, q[:-1], cs, ct)

    def test_geometric_degeneracy_is_not_finite_certainty(self):
        q = np.column_stack((np.arange(10.), np.zeros((10, 2))))
        cov = np.tile(np.eye(3)*1e-4, (10, 1, 1))
        result = register(q, q, cov, cov)
        self.assertEqual(result['status'], 'rank_deficient')
        self.assertIsNone(result['covariance'])
        with self.assertRaises(ValueError):
            information_inverse(np.diag([1, 1, 1, 1, 1, 0]))

    def test_source_coordinate_change_transports_information(self):
        p, q, cs, ct, t = fixture()
        a = exp_se3([.6, -.2, .1, .2, -.1, .05])
        r = a[:3, :3]
        ad = np.block([[r, skew(a[:3, 3])@r], [np.zeros((3, 3)), r]])
        c1, _, h1, _ = gaussian_model(t, p, q, cs, ct)
        c2, _, h2, _ = gaussian_model(a@t, p@r.T+a[:3, 3], q, r@cs@r.T, ct)
        self.assertAlmostEqual(c1, c2, places=9)
        np.testing.assert_allclose(information_inverse(h2), ad@information_inverse(h1)@ad.T,
                                   atol=1e-14, rtol=1e-9)

    def test_generic_transport_matches_analytic_adjoint(self):
        _, _, _, _, t = fixture()
        a = exp_se3([1, .2, -.4, .2, -.3, .1])
        c = np.diag([1, 2, 3, .1, .2, .3])
        out, j = transport_covariance(lambda x: a@x@np.linalg.inv(a), t, c)
        r = a[:3, :3]
        ad = np.block([[r, skew(a[:3, 3])@r], [np.zeros((3, 3)), r]])
        np.testing.assert_allclose(j, ad, atol=5e-9)
        np.testing.assert_allclose(out, ad@c@ad.T, atol=1e-8)

    def test_ci_does_not_halve_identical_correlated_uncertainty(self):
        c = np.diag([1, 2, 3, .1, .2, .3])
        a = np.arange(6.)
        result = covariance_intersection(a, c, a, c)
        np.testing.assert_allclose(result['mean'], a)
        np.testing.assert_allclose(result['covariance'], c)
        self.assertEqual(result['weight'], .5)

    def test_ci_complementary_directions_and_symmetry(self):
        a, b = np.zeros(6), np.ones(6)
        ca, cb = np.diag([.1]*3+[10]*3), np.diag([10]*3+[.1]*3)
        x = covariance_intersection(a, ca, b, cb)
        y = covariance_intersection(b, cb, a, ca)
        np.testing.assert_allclose(x['mean'], y['mean'])
        np.testing.assert_allclose(x['covariance'], y['covariance'])
        self.assertAlmostEqual(x['weight'], .5)
        self.assertLess(x['mean'][0], .02)
        self.assertGreater(x['mean'][5], .98)

    def test_local_covariance_monte_carlo(self):
        p, q, cs, ct, truth = fixture(n=32)
        rng = np.random.default_rng(134)
        norms, errors, covs, states = [], [], [], []
        for _ in range(160):
            pn = p + np.einsum('nij,nj->ni', np.linalg.cholesky(cs), rng.normal(size=p.shape))
            qn = q + np.einsum('nij,nj->ni', np.linalg.cholesky(ct), rng.normal(size=q.shape))
            result = register(pn, qn, cs, ct, initial=truth)
            states.append(result['status'])
            if result['status'] != 'converged':
                continue
            err = log_se3(result['transform']@np.linalg.inv(truth))
            c = result['covariance']
            norms.append(err@np.linalg.solve(c, err))
            errors.append(err)
            covs.append(c)
        self.assertGreaterEqual(len(norms), 156, str({s: states.count(s) for s in set(states)}))
        # Broad finite-sample numerical tolerances; no physical calibration claim.
        self.assertGreater(np.mean(norms), 4.5)
        self.assertLess(np.mean(norms), 7.5)
        ratios = np.var(errors, axis=0, ddof=1)/np.mean(covs, axis=0).diagonal()
        self.assertTrue(np.all((ratios > .6) & (ratios < 1.5)), str(ratios))
        print({'synthetic_replicates': 160, 'converged': len(norms),
               'mean_squared_mahalanobis': float(np.mean(norms)),
               'empirical_to_predicted_variance': ratios.tolist()})


if __name__ == '__main__':
    unittest.main()
