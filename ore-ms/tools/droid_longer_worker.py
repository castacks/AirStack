"""Run the frozen DROID adapter with only a validated buffer allocation change."""
import argparse
from contextlib import contextmanager
from pathlib import Path

import droid_ms2_development as native


def allocation_settings(base,frames):
    if type(frames) is not int or frames<=0:raise ValueError('positive source frame count required')
    result=dict(base);required=frames+16+1
    result['buffer']=max(base['buffer'],1<<(required-1).bit_length())
    return result


@contextmanager
def fixed_allocation(plan):
    expected=allocation_settings(native.SETTINGS,plan['source_frames'])
    if plan['settings']!=expected:raise ValueError('only source-count-derived buffer allocation may change')
    original=native.SETTINGS
    native.SETTINGS=expected
    try:yield
    finally:native.SETTINGS=original


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True);parser.add_argument('--case',required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    with fixed_allocation(native.read(args.protocol)):
        success=native.execute(args.protocol,args.case,args.output)
    raise SystemExit(0 if success else 1)
