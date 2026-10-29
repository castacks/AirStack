"""Delegate unchanged v3 execution with a temporary instance-only solver arm."""
import argparse
import json
from pathlib import Path
import time
from unittest.mock import patch


def run(args):
    import torch
    from Scripts.Setup import run_td3_execution_v3 as base
    from Scripts.Setup.bounded_solver_adapter import install
    original_factory = base.MACVO.from_config; restorers = []; records = []; started = time.monotonic_ns()
    def factory(config):
        system = original_factory(config)
        restorers.append(install(system.Optimizer, system.graph, args.arm, records))
        return system
    error = None
    try:
        with patch.object(base.MACVO, 'from_config', side_effect=factory): base.run(args, args.modality)
        torch.cuda.synchronize()
    except BaseException as caught:
        error = {'type': type(caught).__name__, 'message': str(caught)}; raise
    finally:
        for restore in reversed(restorers): restore()
        output = args.output/args.modality; output.mkdir(parents=True, exist_ok=True)
        report = {'arm': args.arm, 'records': records, 'error': error, 'elapsed_ns': time.monotonic_ns()-started,
                  'instance_method_restored': True, 'factory_restored': True,
                  'cuda_peak_allocated_bytes': int(torch.cuda.max_memory_allocated()) if torch.cuda.is_initialized() else None,
                  'scope': 'bounded sequential execution; solver completion is not commit availability or physical accuracy'}
        (output/'solver-adapter.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('dataset', 'manifest', 'output'): p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--modality', choices=('rgb', 'thermal'), required=True); p.add_argument('--arm', choices=('passthrough', 'repair'), required=True)
    p.add_argument('--frames', type=int, default=20); p.add_argument('--start', type=int, default=0)
    p.add_argument('--record-uncertainty', action='store_true'); p.add_argument('--record-arbiter-events', action='store_true')
    p.add_argument('--acknowledge-provisional-geometry', action='store_true'); a = p.parse_args()
    if not a.acknowledge_provisional_geometry or not 2 <= a.frames <= 20: p.error('acknowledge provisional geometry and bound execution to 2–20 frames')
    run(a)
