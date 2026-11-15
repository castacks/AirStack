"""Report the frozen sequence comparison without selecting or refitting arms."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from qualify_native_frontend import read


def main():
    root = Path(__file__).resolve().parents[1]
    folder = root/'results/continuity-sequence/scoring-001'
    records = read(folder/'scores.json')['records']
    names = ['uninterrupted', 'retained_gap', 'hold_pose_splice', 'visual_anchor']
    labels = ['Adjacent uninterrupted', 'Retained keyframe across gap', 'Hold-pose splice diagnostic', 'Old-keyframe visual reattachment']
    with (folder/'metrics.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['scene', 'arm', 'coordinate_frame', 'available', 'planned', 'translation_rmse_mm', 'rotation_rmse_deg'])
        for row in records:
            for name, arm in row['arms'].items():
                writer.writerow([row['scene'], name, arm['coordinate_frame'], arm['available_postgap'], arm['planned_postgap'],
                                 None if arm['translation_rmse_m'] is None else arm['translation_rmse_m']*1000, arm['rotation_rmse_deg']])
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    x = np.arange(6)
    for j, (name, label) in enumerate(zip(names, labels)):
        for ax, metric, factor in zip(axes, ['translation_rmse_m', 'rotation_rmse_deg'], [1000, 1]):
            vals = [np.nan if row['arms'][name][metric] is None else row['arms'][name][metric]*factor for row in records]
            ax.plot(x, vals, marker=['o', 's', 'x', '^'][j], label=label)
    axes[0].set_ylabel('Post-gap translation RMSE (mm)')
    axes[1].set_ylabel('Post-gap rotation RMSE (degrees)')
    for ax in axes:
        ax.set_yscale('log')
        ax.grid(alpha=.25)
    axes[0].legend(ncol=2, fontsize=9)
    axes[0].set_title('Original-frame errors, four fixed post-gap frames per artificial sequence')
    axes[1].set_xticks(x, [r['scene'].replace('-plane4m', '\nplane4m').replace('-two-depths', '\ntwo depths') for r in records])
    fig.savefig(folder/'continuity-comparison.png', dpi=160)
    fig.savefig(folder/'continuity-comparison.pdf')
    print('Wrote CSV and static comparison figures; no fits or arm changes.')


if __name__ == '__main__':
    main()
