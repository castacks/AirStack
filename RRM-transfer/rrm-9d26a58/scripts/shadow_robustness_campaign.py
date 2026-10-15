#!/usr/bin/env python3
"""Run/reconstruct a pinned authored shadow campaign; never starts robot/model code."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rrm.shadow_robustness import run_campaign, verify_campaign


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--verify',action='store_true')
    args=parser.parse_args()
    if args.verify:
        result=verify_campaign(args.output)
    else:
        report=run_campaign(args.output)
        result={name:{key:value for key,value in metrics.items() if key!='rows'}
                for name,metrics in report['arms'].items()}
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
