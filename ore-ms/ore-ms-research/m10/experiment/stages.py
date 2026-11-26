"""Materialize split-local observation inputs before the isolated policy runs."""
import argparse
import csv
from pathlib import Path

import numpy as np

from model import FROZEN, OBS, fit, table

SPLITS = {"development":3456,"calibration":3456,"holdout":6912}


def stage(archive, split, target):
    frames = [r for r in table(archive/"inference/frames.csv",OBS) if r["split"]==split]
    if len(frames)!=SPLITS[split]:
        raise ValueError("frozen opportunity count")
    target.mkdir()
    predictions = [r for r in table(archive/"partitions"/f"{split}_predictions.csv",FROZEN)
                   if r["arm"] in ("B01","B02")]
    source = np.load(archive/"inference/masks.npy",mmap_mode="r",allow_pickle=False)
    np.save(target/"masks.npy",source[[int(r["id"]) for r in frames]],allow_pickle=False)
    for name,fields,rows in (("frames.csv",OBS,frames),("candidates.csv",FROZEN,predictions)):
        with (target/name).open("w",newline="") as file:
            writer = csv.DictWriter(file,fields,lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command",required=True)
    s = commands.add_parser("stage")
    s.add_argument("archive",type=Path)
    s.add_argument("split",choices=SPLITS)
    s.add_argument("target",type=Path)
    t = commands.add_parser("fit")
    for name in ("source","labels","output"):
        t.add_argument(name,type=Path)
    args = parser.parse_args()
    if args.command=="fit":
        fit(args.source,args.labels,args.output)
    else:
        stage(args.archive,args.split,args.target)
