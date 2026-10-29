"""Declared 100-frame replication; delegate the frozen EXP084 adapter/run function."""
import argparse
from pathlib import Path
from Scripts.Setup.run_td3_solver_arm import run

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('dataset', 'manifest', 'output'): p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--modality', choices=('rgb', 'thermal'), required=True); p.add_argument('--arm', choices=('passthrough', 'repair'), required=True)
    p.add_argument('--frames', type=int, default=100); p.add_argument('--start', type=int, default=0)
    p.add_argument('--record-uncertainty', action='store_true'); p.add_argument('--record-arbiter-events', action='store_true')
    p.add_argument('--acknowledge-provisional-geometry', action='store_true'); a = p.parse_args()
    if not a.acknowledge_provisional_geometry or a.frames != 100 or a.start != 0: p.error('acknowledge provisional geometry and require declared 100 frames, start0')
    run(a)
