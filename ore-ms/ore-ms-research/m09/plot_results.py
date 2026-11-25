"""Plot all validation curves and the matched holdout CRPS by horizon."""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("curves",type=Path)
    parser.add_argument("scores",type=Path)
    parser.add_argument("png",type=Path)
    args=parser.parse_args()
    with args.curves.open() as file:
        curves=list(csv.DictReader(file))
    with args.scores.open() as file:
        scores=list(csv.DictReader(file))
    colors={"data":"#41628a","ablation":"#b37537","physics":"#007c67"}
    fig,axes=plt.subplots(1,3,figsize=(12,3.8),layout="constrained")
    for axis,size in zip(axes[:2],("128","768"),strict=True):
        for model,color in colors.items():
            for i,seed in enumerate(("901","902","903")):
                arm=[r for r in curves if (r["size"],r["model"],r["seed"])==(size,model,seed)]
                axis.plot([int(r["epoch"]) for r in arm],[float(r["validation_burden_nll"]) for r in arm],
                          color=color,alpha=.75,label=model if i==0 else None)
        axis.set(title=f"{size} training realizations",xlabel="Epoch",ylabel="Validation burden NLL")
        axis.grid(alpha=.2)
    axes[0].legend(frameon=False,fontsize=8)
    h=[0,2,6,18,48]
    for model,color in (list(colors.items())+[("joint","#303030")]):
        size="0" if model=="joint" else "768"
        errors=[np.mean([float(r["crps"]) for r in scores if (
            r["case"],r["model"],r["mode"],r["size"],r["target"],r["horizon_s"])==
            ("matched",model,"joint",size,"burden",str(t))]) for t in h]
        axes[2].plot(h,errors,"o-",color=color,label="M08 joint" if model=="joint" else model)
    axes[2].set(title="Matched holdout",xlabel="Horizon (s)",ylabel="Burden CRPS",xticks=h)
    axes[2].grid(alpha=.2)
    axes[2].legend(frameon=False,fontsize=8)
    fig.savefig(args.png,dpi=160)


if __name__=="__main__":
    main()
