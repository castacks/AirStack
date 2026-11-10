"""Single training-frame directional probe; no optimizer step or new candidate."""
import torch

from stereo_mean_adaptation import (OUT, build, configure, deny_evaluation_data,
                                    freeze_check, load_pair, objective, read, setup, sha, write)


def main():
    freeze_check(); setup()
    locked = sha(OUT/'final-mean-head.pth')
    dataset = read(OUT/'dataset.json'); model = build(); names = configure(model)
    parameters = [(n, p) for n, p in model.named_parameters() if n in names]
    row = next(r for r in dataset['records'] if r['condition'] == 'day')['rows'][0]
    with deny_evaluation_data(dataset):
        a, b, target, valid = load_pair(row)
        prediction, _ = model(a, b); loss = objective(prediction[-1], target, valid); loss.backward()
        norm = torch.sqrt(sum(p.grad[0].square().sum() for _, p in parameters))
        originals = [p.detach().clone() for _, p in parameters]
        directions = [-p.grad.detach().clone()/norm for _, p in parameters]
        for d in directions:
            d[1].zero_()
        probes = []
        with torch.no_grad():
            for epsilon in [1e-5, 5e-6]:
                values = []
                for sign in [-1., 1.]:
                    for (_, p), original, d in zip(parameters, originals, directions):
                        p.copy_(original+sign*epsilon*d)
                    prediction, _ = model(a, b)
                    values.append(float(objective(prediction[-1], target, valid)))
                probes.append({'epsilon': epsilon, 'loss_negative_step': values[0], 'loss_positive_step': values[1],
                               'full_forward_directional_derivative': (values[1]-values[0])/(2*epsilon)})
            for (_, p), original in zip(parameters, originals):
                p.copy_(original)
            prediction, _ = model(a, b)
            restored_loss = float(objective(prediction[-1], target, valid))
    assert sha(OUT/'final-mean-head.pth') == locked
    result = {'training_condition': 'day', 'source_id': '000000', 'posthoc_diagnostic': True,
              'initial_training_loss': float(loss.detach()), 'restored_loss': restored_loss,
              'autograd_directional_derivative': -float(norm), 'probes': probes,
              'optimizer_steps': 0, 'new_checkpoint': False, 'locked_candidate_unchanged': True,
              'scope': 'head reused in recurrent iterations with native coordinate detach; diagnostic is not an optimization retry'}
    write(OUT/'gradient-direction-probe.json', result); print(result)


if __name__ == '__main__':
    main()
