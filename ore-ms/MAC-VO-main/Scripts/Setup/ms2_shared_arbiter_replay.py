"""Opt-in MS2 worker adaptation; existing shared-controller policy is unchanged."""
import argparse
import json
from pathlib import Path
from Scripts.Setup import shared_arbiter_replay as controller
from Scripts.Setup.shared_arbiter_replay import worker as base_worker
from Scripts.Setup.ms2_execution_adapter import adapted_runner, execution_contract


def worker(camera, options, pipe):
    # Module-level target remains picklable under multiprocessing spawn.
    with adapted_runner():
        return base_worker(camera, options, pipe)


def run(args):
    if Path(args.output).exists():
        raise FileExistsError(args.output)
    provenance = execution_contract(args.dataset, args.manifest)
    original = controller.worker
    controller.worker = worker
    try:
        return controller.run(args)
    finally:
        controller.worker = original
        if Path(args.output).is_dir():
            with (Path(args.output) / 'adapter-contract.json').open('x') as stream:
                json.dump(provenance, stream, indent=2); stream.write('\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('dataset', 'manifest', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--timeout-seconds', type=float, default=300)
    parser.add_argument('--acknowledge-provisional-geometry', action='store_true')
    parser.add_argument('--record-root-initialization', action='store_true')
    run(parser.parse_args())
