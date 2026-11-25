"""Export the sampled moving-foreground and moving-target visibility traces."""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("png", type=Path)
    args = parser.parse_args()
    with args.frames.open(newline="") as source:
        frames = list(csv.DictReader(source))
    figure, axes = plt.subplots(2, 1, figsize=(7.2, 5.4), layout="constrained")
    for axis, (name, title) in zip(axes, (
        ("moving_foreground", "Moving foreground, stationary target"),
        ("moving_target", "Moving target, stationary foreground"),
    ), strict=True):
        samples = [row for row in frames if row["case"] == name]
        times = [float(row["time_s"]) for row in samples]
        axis.plot(times, [int(row["projected_target"]) for row in samples],
                  "o--", color="#425678", label="Projected target")
        axis.plot(times, [int(row["visible_target"]) for row in samples],
                  "s-", color="#00866e", label="Visible target")
        axis.set(title=title, xlabel="Time (s)", ylabel="Pixel centers", ylim=(-2, 40), xticks=times)
        axis.grid(alpha=.25)
        axis.legend(loc="lower left", frameon=False)
    figure.savefig(args.png, dpi=170)


if __name__ == "__main__":
    main()
