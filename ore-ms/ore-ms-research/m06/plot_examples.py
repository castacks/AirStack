"""Export actual RGB codes and a false-color view of the synthetic LWIR codes."""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pairs", type=Path)
    parser.add_argument("png", type=Path)
    args = parser.parse_args()
    with args.pairs.open(newline="") as source:
        rows = list(csv.DictReader(source))
    samples = [
        ("clear", 0, "Clear"), ("rgb_dominant", 0, "RGB attenuation"),
        ("crossover", 0, "Thermal crossover"), ("moving_combined", 3, "Moving foreground"),
        ("noise_and_clip", 0, "Noise and clipping"),
    ]
    fig, axes = plt.subplots(2, len(samples), figsize=(11.5, 4.3), layout="constrained")
    for col, (case, time, title) in enumerate(samples):
        rgb, thermal = np.zeros((7, 9, 3)), np.full((7, 9), np.nan)
        for row in rows:
            if row["case"] != case or int(row["time_s"]) != time or not row["path_id"]:
                continue
            y, x = int(row["row"]), int(row["column"])
            rgb[y, x] = [int(row[f"{channel}_code"])/4095 for channel in "RGB"]
            thermal[y, x] = int(row["lwir_code"])/4095
        axes[0, col].imshow(rgb, interpolation="nearest")
        axes[0, col].set_title(title)
        overlay = axes[1, col].imshow(thermal, vmin=0, vmax=1, cmap="inferno", interpolation="nearest")
    for pane in axes.flat:
        pane.set_xticks([])
        pane.set_yticks([])
    axes[0, 0].set_ylabel("RGB codes")
    axes[1, 0].set_ylabel("LWIR false color")
    fig.colorbar(overlay, ax=axes[1], fraction=.025, label="LWIR code / 4095")
    fig.savefig(args.png, dpi=170)


if __name__ == "__main__":
    main()
