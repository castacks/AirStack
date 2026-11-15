"""Present locked timing-comparison scores; no fitting or outcome selection."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from qualify_native_frontend import read, write, now


def main():
    root = Path(__file__).resolve().parents[1]
    folder = root/'results/sensor-estimator-comparison/scoring-001'
    rows = read(folder/'scores.json')['rows']
    with (folder/'metrics.csv').open('x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['case', 'motion', 'condition', 'arm', 'query_mode', 'successful', 'status', 'points', 'translation_error_mm', 'rotation_error_deg', 'median_matching_covariance_trace', 'translation_information_std_proxy_mm', 'rotation_information_std_proxy_deg'])
        for row in rows:
            m = row['metrics']
            writer.writerow([row['case'], row['motion'], row['condition'], row['arm'], row['mode'], row['successful'], row['status'], row['points'],
                             None if m is None else m['translation_m']*1000, None if m is None else m['rotation_deg'], row['median_temporal_covariance_trace'],
                             None if m is None else m['translation_information_std_proxy_m']*1000, None if m is None else m['rotation_information_std_proxy_deg']])
    cases = [c+'-'+m for m in ['twist', 'accelerating'] for c in ['day', 'night', 'rain']]
    figure, axes = plt.subplots(2, 2, figsize=(13, 7), sharex=True, constrained_layout=True)
    handles = None
    for column, mode in enumerate(['regenerated', 'common_ids']):
        axes[0, column].set_title('Reselected queries' if mode == 'regenerated' else 'Same query IDs across arms')
        for arm, marker in zip(['sharp', 'memory', 'rolling', 'combined'], ['o', 's', '^', 'x']):
            selected = [next(r for r in rows if r['case'] == key and r['mode'] == mode and r['arm'] == arm) for key in cases]
            for axis, metric, multiplier in zip(axes[:, column], ['translation_m', 'rotation_deg'], [1000, 1]):
                axis.plot(range(6), [r['metrics'][metric]*multiplier if r['successful'] else np.nan for r in selected], marker=marker, label=arm)
        axes[0, column].set_ylabel('Translation error (mm)')
        axes[1, column].set_ylabel('Rotation error (degrees)')
        axes[1, column].set_xticks(range(6), [key.replace('-twist', '\ntwist').replace('-accelerating', '\naccel.') for key in cases])
        axes[0, column].legend(ncol=2, fontsize=8)
        for axis in axes[:, column]:
            axis.grid(alpha=.25)
    figure.savefig(folder/'pose-errors.png', dpi=160)
    figure.savefig(folder/'pose-errors.pdf')
    comparisons = []
    for mode in ['regenerated', 'common_ids']:
        for arm in ['memory', 'rolling', 'combined']:
            pairs = [(r, next(b for b in rows if b['case'] == r['case'] and b['mode'] == mode and b['arm'] == 'sharp'))
                     for r in rows if r['mode'] == mode and r['arm'] == arm and r['motion'] != 'stationary']
            valid = [(r, b) for r, b in pairs if r['successful'] and b['successful']]
            comparisons.append({'mode': mode, 'arm': arm, 'planned_pairs': 6, 'matched_successful_pairs': len(valid),
                                'translation_worse_than_sharp': sum(r['metrics']['translation_m'] > b['metrics']['translation_m'] for r, b in valid),
                                'rotation_worse_than_sharp': sum(r['metrics']['rotation_deg'] > b['metrics']['rotation_deg'] for r, b in valid),
                                'matching_covariance_lower_than_sharp': sum(r['median_temporal_covariance_trace'] < b['median_temporal_covariance_trace'] for r, b in pairs)})
    write(folder/'summary.json', {'created_utc': now(), 'comparisons': comparisons, 'successful_pose_fits': sum(r['successful'] for r in rows),
                                 'planned_pose_attempts': 72, 'scope': 'Descriptive fixed-profile case comparisons. No significance, physical calibration or generalization claims.'})
    print('Saved complete CSV, four-panel pose figure and descriptive paired counts.')


if __name__ == '__main__':
    main()
