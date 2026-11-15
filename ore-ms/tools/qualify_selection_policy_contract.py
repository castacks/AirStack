"""Exact expectation and independent gradient/visibility qualification; no training."""
import argparse
import itertools
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import selection_policy_contract as p
from paired_frontend_assessment import fit
from pose_covariance_replay import log_se3
from qualify_native_frontend import now, read, sha


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    plan, out = read(args.protocol), Path(args.output)
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path
    out.mkdir(parents=True, exist_ok=False)
    checks = 0
    def check(value, message):
        nonlocal checks
        assert value, message
        checks += 1
    def close(a, b, rtol=2e-5, atol=1e-7):
        nonlocal checks
        np.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
        checks += 1
    rng = np.random.default_rng(plan['seed'])
    import torch
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    all_arrays, receipts = {}, {}
    def qualify_gradient(name, features, theta, k, losses):
        sequences = list(itertools.permutations(range(len(features)), k))
        result = [p.ordered_log_probability_and_score(theta, features, s) for s in sequences]
        probability = np.exp([r[0] for r in result])
        score = np.asarray([r[1] for r in result])
        close(probability.sum(), 1, rtol=0, atol=1e-12)
        close(probability@score, np.zeros(len(theta)), rtol=0, atol=1e-12)
        gradient = (probability*np.asarray(losses))@score
        def independent_expectation(t):
            # Independent scalar product formulation, not producer logprob/score.
            value = 0.
            for sequence, loss in zip(sequences, losses, strict=True):
                remaining = set(range(len(features)))
                likelihood = 1.
                for chosen in sequence:
                    weights = np.exp(features[list(sorted(remaining))]@t)
                    likelihood *= np.exp(features[chosen]@t)/np.sum(weights)
                    remaining.remove(chosen)
                value += likelihood*loss
            return value
        step = 1e-5
        finite = np.array([(independent_expectation(theta+np.eye(len(theta))[j]*step)-independent_expectation(theta-np.eye(len(theta))[j]*step))/(2*step) for j in range(len(theta))])
        close(gradient, finite)
        tensor = torch.tensor(theta, dtype=torch.float64, requires_grad=True)
        F = torch.tensor(features, dtype=torch.float64)
        total = tensor.sum()*0
        for sequence, loss in zip(sequences, losses, strict=True):
            available = list(range(len(features)))
            log_probability = tensor.sum()*0
            for chosen in sequence:
                logits = F[available]@tensor
                log_probability = log_probability+F[chosen]@tensor-torch.logsumexp(logits, 0)
                available.remove(chosen)
            total = total+log_probability.exp()*float(loss)
        total.backward()
        close(gradient, tensor.grad.numpy())
        # Independent other-draw baseline is unbiased; no differentiation through rewards/baseline.
        loo = sum(probability[i]*probability[j]*(losses[i]-losses[j])*score[i] for i in range(len(sequences)) for j in range(len(sequences)))
        close(gradient, loo)
        constant = (probability*np.ones(len(probability)))@score
        close(constant, np.zeros_like(theta), rtol=0, atol=1e-12)
        all_arrays[name+'_probabilities'] = probability
        all_arrays[name+'_scores'] = score
        all_arrays[name+'_gradient'] = gradient
        all_arrays[name+'_finite_difference'] = finite
        all_arrays[name+'_torch_gradient'] = tensor.grad.numpy()
        receipts[name] = {'sequences': len(sequences), 'expected_loss': float(probability@losses), 'gradient': gradient.tolist(), 'max_gradient_fd_difference': float(np.max(np.abs(gradient-finite)))}
        return sequences, probability
    F = rng.normal(size=(4, 3))
    theta = np.array([.2, -.15, .3])
    seq = list(itertools.permutations(range(4), 2))
    losses = np.array([sum((i+1)**2 for i in s)/20 for s in seq])
    seq, probability = qualify_gradient('generic', F, theta, 2, losses)
    histogram = Counter(tuple(p.sample_subset(theta, F, 2, plan['seed']+i)) for i in range(4096))
    frequency = np.array([histogram[s]/4096 for s in seq])
    check(np.all(np.abs(frequency-probability) <= 6*np.sqrt(probability*(1-probability)/4096)+2/4096), 'sampler matches ordered probabilities')
    check(np.array_equal(p.sample_subset(theta, F, 2, 7), p.sample_subset(theta, F, 2, 7)), 'seed repeat')
    check(all(len(set(s)) == 2 for s in histogram), 'no duplicate sampling')
    for k in [0, 1, 4, 6]:
        sampled = p.sample_subset(np.zeros(3), F, k, 9)
        check(len(sampled) == min(k, 4) and len(set(sampled)) == len(sampled), 'budget boundaries')
    full = list(itertools.permutations(range(4)))
    qualify_gradient('all_candidates', F, theta, 4, np.ones(len(full)))
    close(p.top_subset(np.zeros(3), F, 2), [0, 1], rtol=0, atol=0)
    close(p.residual_utility_weights(theta, F).mean(), 1, rtol=0, atol=1e-12)
    mean, scale = p.training_statistics(np.column_stack([F, np.ones(4)]))
    normalized = p.standardize(np.column_stack([F, np.ones(4)]), mean, scale)
    close(normalized.mean(0), np.zeros(4), rtol=0, atol=1e-12)
    check(scale[-1] == 1 and np.all(normalized[:, -1] == 0), 'constant training feature')
    # Actual unchanged IRLS solves provide a frozen pose-loss table, not trained parameters.
    K = np.asarray(plan['K'])
    X1 = np.array([[-.7,-.3,4.], [.8,-.4,4.5], [.5,.8,5.], [-.8,.6,4.2], [.1,.1,6.]])
    R = Rotation.from_rotvec([.008,-.006,.004]).as_matrix()
    T = np.eye(4);T[:3,:3],T[:3,3] = R,[.03,-.02,.08]
    X0 = X1@R.T+T[:3,3]+np.array([[.002,0,0],[-.001,.001,0],[0,-.002,.001],[.001,0,-.002],[-.002,.001,.001]])
    uv = (X0@K.T);uv = uv[:,:2]/uv[:,2:]
    packet = {'source_uv':uv,'X0':X0,'X1':X1,'C0':np.broadcast_to(np.diag([.001,.002,.003]),(5,3,3)).copy(),'C1':np.broadcast_to(np.diag([.002,.001,.004]),(5,3,3)).copy(),'length':np.array(np.median(X0[:,2]))}
    features = p.utility_features(packet,np.full((5,2),.25),np.arange(10).reshape(5,2)*.1,K)
    check(features.shape == (5,8), 'eight fixed features')
    samples, loss_table = {}, {}
    for subset in itertools.combinations(range(5),3):
        selected = {key:(value[list(subset)] if key!='length' else value) for key,value in packet.items()}
        result = fit(selected,plan['solver'])
        check(result['successful'], 'pose-table fit convergence')
        error = log_se3(np.asarray(result['target_to_source'])@np.linalg.inv(T))
        loss_table[subset] = p.bounded_pose_loss(error)
        samples[str(subset)] = result
    sequence = list(itertools.permutations(range(5),3))
    losses = np.array([loss_table[tuple(sorted(s))] for s in sequence])
    qualify_gradient('pose_table',features,np.linspace(-.1,.1,8),3,losses)
    check(p.bounded_pose_loss(np.zeros(6)) == 0 and p.bounded_pose_loss(np.ones(6),False) == 1, 'bounded failure loss')
    check(all(0 <= value < 1 for value in loss_table.values()), 'successful bounded task losses')
    # Independent scalar ray/plane intersection for every varied motion/geometry profile.
    tested = 0
    for geometry in plan['geometries']:
        for motion in plan['motions']:
            rotation = Rotation.from_rotvec(np.deg2rad(motion['rotvec_deg'])).as_matrix()
            center = np.asarray(motion['translation_m'])
            pixels = np.array([[50.,50.],[200.,160.],[400.,100.],[600.,200.]])
            world,surface,valid = p.ray_hit(pixels,K,rotation,center,geometry)
            for i, uv0 in enumerate(pixels):
                direction = rotation@np.linalg.solve(K,np.r_[uv0,1.])
                hits = []
                for index,depth in enumerate(geometry['depths_m']):
                    length = (depth-center[2])/direction[2]
                    point = center+length*direction
                    if length > 0 and (index==1 or np.dot(point[:2],geometry['near_normal_xy']) <= geometry['near_offset_m']):
                        hits.append((length,index,point))
                best = min(hits,key=lambda h:h[0])
                close(world[i],best[2],rtol=1e-10,atol=1e-10)
                check(surface[i]==best[1] and valid[i], 'closest permitted plane')
                tested += 1
    geometry = {'depths_m':[4.,8.],'near_normal_xy':[1.,0.],'near_offset_m':0.}
    points = np.array([[.2,0,8.],[-.2,0,4.],[100.,0,4.],[0.,0,-1.]])
    uv, visible = p.visible_projection(points,K,np.eye(3),[-.6,0,0],geometry)
    check(visible.tolist()==[False,True,False,False], 'occlusion/near/field/behind')
    check(0 < uv[0,0] < 640 and 0 < uv[0,1] < 256, 'occluded point projects inside image')
    _, original = p.visible_projection(np.array([[-.2,0,8.]]),K,np.eye(3),[0,0,0],geometry)
    _, revealed = p.visible_projection(np.array([[-.2,0,8.]]),K,np.eye(3),[.6,0,0],geometry)
    check(not original[0] and revealed[0], 'true disocclusion')
    _, boundary = p.visible_projection(np.array([[0,0,4.],[0,0,8.]]),K,np.eye(3),[0,0,0],geometry)
    check(boundary.tolist()==[True,False], 'closed foreground boundary')
    old = read(plan['prior_renderer_inventory'])
    inherited = 0
    for record in old['records']:
        if record['scene']['id']!='two-depths':continue
        with np.load(record['path']) as z:
            # Sparse fixed raster probe over all four already-audited camera views.
            vv,uu = np.mgrid[0:256:17,0:640:19]
            pix = np.stack([uu,vv],-1)
            for view in range(4):
                world,surface,valid = p.ray_hit(pix,K,z['rotations'][view],z['centers'][view],geometry)
                close(world,z['world'][view,vv,uu],rtol=1e-10,atol=1e-10)
                check(np.array_equal(surface,z['surface'][view,vv,uu]),'previous renderer surface equivalence')
                inherited += int(valid.sum())
    np.savez_compressed(out/'gradient-arrays.npz',**all_arrays)
    (out/'pose-fit-receipts.json').write_text(json.dumps(samples,indent=2)+'\n')
    report = {'created_utc':now(),'status':'PASS','checks':checks,'gradient_receipts':receipts,'pose_table_fits':len(samples),'sampler_trials':4096,'independent_geometry_rays':tested,'prior_renderer_world_probes':inherited,'source_sha256':sha(__file__),'producer_sha256':sha(Path(p.__file__)),'protocol_sha256':sha(args.protocol),'scope':'Exact subset-policy gradient and analytic visibility qualification. No native corpus inference, utility training, corpus accuracy, weight-only derivative qualification or novel-method claim.'}
    (out/'qualification.json').write_text(json.dumps(report,indent=2)+'\n')
    for path,digest in plan['pins'].items():assert sha(path)==digest,path
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
