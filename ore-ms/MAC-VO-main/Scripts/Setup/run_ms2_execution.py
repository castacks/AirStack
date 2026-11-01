"""Bounded MS2 camera-local execution through the unchanged v3 runner."""
import argparse
import json
from pathlib import Path
from Scripts.Setup.ms2_execution_adapter import adapted_runner, execution_contract


def run(args, modality):
    target = Path(args.output) / modality
    if target.exists():
        raise FileExistsError(target)
    provenance = execution_contract(args.dataset, args.manifest)
    try:
        with adapted_runner() as runner:
            return runner.run(args, modality)
    finally:
        if target.is_dir():
            with (target / 'adapter-contract.json').open('x') as stream:
                json.dump(provenance, stream, indent=2); stream.write('\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('dataset', 'manifest', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--modality', choices=('rgb', 'thermal'), required=True)
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--record-uncertainty', action='store_true')
    parser.add_argument('--record-arbiter-events', action='store_true')
    parser.add_argument('--record-root-initialization', action='store_true')
    parser.add_argument('--acknowledge-provisional-geometry', action='store_true')
    args = parser.parse_args()
    if not 2 <= args.frames <= 100 or args.start < 0:
        parser.error('bounded execution requires 2–100 frames and nonnegative start')
    run(args, args.modality)
