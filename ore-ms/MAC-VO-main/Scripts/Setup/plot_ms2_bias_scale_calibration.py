"""Figures for the frozen retrospective comparison; no model or data changes."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(run):
    report = json.loads((run / 'report.json').read_text())['recordings']
    assert all(r['status'] == 'PASS' for r in report.values())
    labels = ['Rain (063)', 'Day (064)', 'Night (064)', 'Rain (064)']
    colors = ['#475569', '#2563eb', '#d97706']
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), layout='constrained')
    x = np.arange(4)
    for ax, camera, title in zip(axes[:2], ['rgb', 'thr'], ['RGB disparity intervals', 'Thermal disparity intervals']):
        for j, (family, label) in enumerate([('original', 'Original'), ('scale', 'Scale only'), ('bias_scale', 'Bias + scale')]):
            values = [100 * r['families'][family]['summary'][camera]['all']['stats']['coverage_1.96']['median'] for r in report.values()]
            ax.plot(x, values, 'o-', color=colors[j], label=label)
        ax.axhline(94.999579, color='black', ls='--', lw=1, label='Nominal 95%')
        ax.set(title=title, ylabel='Median per-frame coverage (%)', ylim=(35, 103), xticks=x, xticklabels=labels)
        ax.tick_params(axis='x', rotation=20)
        ax.grid(axis='y', alpha=.2)
    axes[0].legend(fontsize=8, loc='lower right')
    for j, (baseline, label) in enumerate([('original', '/ Original'), ('scale', '/ Scale only'), ('rgb', '/ Always RGB')]):
        values = [r['selection_summary']['midpoint']['mean_l2']['bias_scale']['comparisons'][baseline]['ratio']['median'] for r in report.values()]
        axes[2].bar(x + (j - 1) * .24, values, .24, color=colors[j], label=label)
    axes[2].axhline(1, color='black', ls='--', lw=1)
    axes[2].set(title='Bias + scale selection', ylabel='Median paired mean-distance risk ratio', yscale='log', ylim=(.1, 3), xticks=x, xticklabels=labels)
    axes[2].tick_params(axis='x', rotation=20)
    axes[2].legend(fontsize=8, loc='lower right')
    axes[2].grid(axis='y', alpha=.2)
    fig.suptitle('Retrospective recording exclusion: supplied references, original 3D predictions\nCoverage changes do not establish camera-selection improvement', fontsize=12)
    for suffix in ('png', 'pdf'):
        fig.savefig(run / f'calibration-comparison.{suffix}', dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    plot(p.parse_args().run)
