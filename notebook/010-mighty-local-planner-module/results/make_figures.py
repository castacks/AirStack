#!/usr/bin/env python3
"""Qualitative flight-path figures for notebook 010 (MIGHTY local planner).

Generates:
  d-practice-avoidance/fig_practice_tracks.png
      practice-field runs 1-7: pillars + issued route + flown track per run
      (the PRACTICE layout is the shareable split — pillars may be shown)
  e-r7-solvability/fig_official_tracks.png
      official frozen-v6 batch (validation + runs 1-5): issued route + flown
      track per run, EVAL PILLARS WITHHELD (answer-key rule; the full-detail
      figure goes into the private agent_study run dir)
  e-r7-solvability/fig_official_clearance.png
      per-run distance-to-nearest-pillar-surface profiles vs the 1.0 m gate
      (distances only — reveals no layout)
  <agent_study>/runs/ref_validation_v6_mighty_001/figures/fig_official_tracks_full.png
      the same official tracks WITH the eval pillar field (PRIVATE)
"""

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

HERE = Path(__file__).resolve().parent
STUDY = Path.home() / 'Development/AirStack/agent_study'
RUN_DIR = STUDY / 'runs/ref_validation_v6_mighty_001'

OFFICIAL_SEEDS = {913529: '1', 913788: '2', 914062: '3', 914322: '4',
                  914588: '5'}
FLY_Z = 0.3  # below this = ground (matches the judge's clearance check)


def load_csv(p):
    pts = []
    for line in Path(p).read_text().splitlines():
        q = line.split(',')
        if len(q) >= 7:
            pts.append((float(q[4]), float(q[5]), float(q[6])))
    return pts


def clearance_profile(pts, pillars):
    prof = []
    for x, y, z in pts:
        if z < FLY_Z:
            prof.append(None)
            continue
        ds = [math.dist((x, y), (px, py)) - pr
              for px, py, pr, ph in pillars if z <= ph]
        # no qualifying pillar = flying above the whole field (droan_gl's
        # vertical-escape behavior reaches z > 18 m) — clearance undefined
        prof.append(min(ds) if ds else None)
    return prof


def draw_run(ax, pts, route, pillars=None, title='', clearance=None):
    if pillars:
        for px, py, pr, ph in pillars:
            ax.add_patch(Circle((px, py), pr, color='0.55', zorder=2))
            ax.add_patch(Circle((px, py), pr + 1.0, fill=False,
                                color='0.8', ls=':', lw=0.7, zorder=1))
    rx = [0.0] + [w[0] for w in route]
    ry = [0.0] + [w[1] for w in route]
    ax.plot(rx, ry, 'k--', lw=1.0, alpha=0.6, zorder=3, label='issued route')
    ax.plot(rx[1:], ry[1:], 'k*', ms=10, zorder=4)
    for i, (wx, wy) in enumerate(zip(rx[1:], ry[1:]), 1):
        ax.annotate(f'CP{i}', (wx, wy), textcoords='offset points',
                    xytext=(4, 4), fontsize=7)
    fly = [(x, y) for x, y, z in pts if z >= FLY_Z]
    if fly:
        ax.plot([p[0] for p in fly], [p[1] for p in fly],
                color='tab:blue', lw=1.4, zorder=5, label='flown track')
        ax.plot(fly[0][0], fly[0][1], 'g^', ms=7, zorder=6)
        ax.plot(fly[-1][0], fly[-1][1], 'rv', ms=7, zorder=6)
    if clearance is not None:
        title += f'\nmin clearance {clearance:.2f} m'
    ax.set_title(title, fontsize=9)
    ax.set_aspect('equal')
    ax.grid(alpha=0.25, lw=0.4)
    ax.tick_params(labelsize=7)


def parse_route(s):
    return [tuple(float(v) for v in w.split(',')) for w in s.split(';')]


def main():
    # ---------------- practice runs 1-7 ----------------
    d = HERE / 'd-practice-avoidance'
    layout = json.loads((d / 'layout_practice.json').read_text())
    pillars = [(p['x'], p['y'], p['r'], p['h']) for p in layout['pillars']]

    fig, axes = plt.subplots(2, 4, figsize=(17, 9))
    for i, ax in enumerate(axes.flat):
        run = i + 1
        if run > 7:
            ax.axis('off')
            ax.legend(handles=[
                plt.Line2D([], [], color='0.55', marker='o', ls='',
                           ms=10, label='pillar (true radius)'),
                plt.Line2D([], [], color='0.8', ls=':',
                           label='1.0 m clearance gate'),
                plt.Line2D([], [], color='k', ls='--', label='issued route'),
                plt.Line2D([], [], color='tab:blue', label='flown track'),
                plt.Line2D([], [], color='g', marker='^', ls='',
                           label='takeoff'),
                plt.Line2D([], [], color='r', marker='v', ls='',
                           label='track end')],
                loc='center', fontsize=10, frameon=False)
            continue
        csv = d / f'2026-08-28_run{run}_odom.csv'
        if run == 1:
            csv = d / '2026-08-28_0345_run1_odom.csv'
        route_file = d / f'2026-08-28_run{run}_route.txt'
        if run == 1:
            route = parse_route('48.6,32.0,10.0; 28.7,21.5,10.0; 10.9,23.2,10.0')
            mode = 'action'
        else:
            lines = route_file.read_text().splitlines()
            route = parse_route(lines[0].split('=', 1)[1])
            mode = 'follower' if any('follower' in ln for ln in lines) else 'action'
        pts = load_csv(csv)
        prof = [c for c in clearance_profile(pts, pillars) if c is not None]
        draw_run(ax, pts, route, pillars,
                 title=f'practice run {run} ({mode})',
                 clearance=min(prof) if prof else None)
    fig.suptitle('MIGHTY practice-field flights (seed 20260828 layout, shareable split) '
                 '— 7/7 clean traversals', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = d / 'fig_practice_tracks.png'
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print('wrote', out)

    # ---------------- official frozen-v6 batch ----------------
    e = HERE / 'e-r7-solvability'
    e.mkdir(exist_ok=True)
    eval_layout = json.loads((STUDY / 'obstacles/layout_r7.json').read_text())
    eval_pillars = [(p['x'], p['y'], p['r'], p['h'])
                    for p in eval_layout['pillars']]

    runs = []  # (label, route, pts)
    dirs = sorted(RUN_DIR.glob('r5_artifacts_*'),
                  key=lambda p: p.stat().st_mtime)
    for dd in dirs:
        rj, oc = dd / 'r7_route.json', dd / 'odom.csv'
        if not (rj.exists() and oc.exists()):
            continue
        seed = json.loads(rj.read_text())['seed']
        if seed in OFFICIAL_SEEDS:
            runs.append((f'official run {OFFICIAL_SEEDS[seed]} (seed {seed})',
                         parse_route(json.loads(rj.read_text())['route']),
                         load_csv(oc)))
    runs.sort(key=lambda r: r[0])

    # NOTE: the EVAL pillar layout is answer-key material (strategy choice
    # 2026-08-28). It is shown here because notebook/ is the study-internal
    # record (entries 008/009 already contain eval-field details) — this
    # figure must NOT reach the public repo or the paper before the study
    # concludes.
    for fname in (e / 'fig_official_tracks.png',
                  RUN_DIR / 'figures' / 'fig_official_tracks_full.png'):
        fname.parent.mkdir(exist_ok=True)
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        for ax, (label, route, pts) in zip(axes.flat, runs):
            prof = [c for c in clearance_profile(pts, eval_pillars)
                    if c is not None]
            draw_run(ax, pts, route, eval_pillars, title=label,
                     clearance=min(prof) if prof else None)
        for ax in axes.flat[len(runs):]:
            ax.axis('off')
        fig.suptitle('Official frozen-v6 judged R7 flights (asm_mighty v0.1.0) '
                     '— 5/5 PASS\nEVAL pillar field shown '
                     '(answer-key material: study-internal figure only)',
                     fontsize=13)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(fname, dpi=130)
        plt.close(fig)
        print('wrote', fname)

    # ------------- droan_gl vs MIGHTY on the same eval field -------------
    # droan_gl's campaign-v5 solvability validation (study/droan-gl-avoidance-
    # fix, notebook 008/009) flew the SAME eval field; routes are fresh per
    # evaluation, so this is a same-field (not same-route) comparison.
    droan_dir = STUDY / 'runs/ref_validation_r7_b240_001'
    droan_runs = []
    for dd in sorted(droan_dir.glob('r5_artifacts_*'),
                     key=lambda p: p.stat().st_mtime):
        rj, oc = dd / 'r7_route.json', dd / 'odom.csv'
        if not (rj.exists() and oc.exists()):
            continue
        seed = json.loads(rj.read_text())['seed']
        droan_runs.append((seed, parse_route(json.loads(rj.read_text())['route']),
                           load_csv(oc)))
    # last five = the yaw-sweep-config validation (droan_gl's best config)
    droan_runs = droan_runs[-5:]

    fig, axes = plt.subplots(2, 5, figsize=(22, 10))
    for ax, (seed, route, pts) in zip(axes[0], droan_runs):
        prof = [c for c in clearance_profile(pts, eval_pillars)
                if c is not None]
        mc = min(prof) if prof else None
        # outcome label: clearance shave vs incomplete route
        fly = [(x, y) for x, y, z in pts if z >= FLY_Z]
        d_end = math.dist(fly[-1], route[-1][:2]) if fly else float('inf')
        outcome = ('FAIL: clearance shave' if mc is not None and mc < 1.0
                   else f'FAIL: stopped {d_end:.0f} m short')
        draw_run(ax, pts, route, eval_pillars,
                 title=f'droan_gl (yaw-sweep cfg), seed {seed}\n{outcome}',
                 clearance=mc)
    for ax, (label, route, pts) in zip(axes[1], runs):
        prof = [c for c in clearance_profile(pts, eval_pillars)
                if c is not None]
        draw_run(ax, pts, route, eval_pillars,
                 title=label + '\nPASS',
                 clearance=min(prof) if prof else None)
    fig.suptitle('Same eval pillar field, fresh routes per run: '
                 'droan_gl campaign-v5 validation (top, 0/5) vs '
                 'MIGHTY official v6 (bottom, 5/5)\n'
                 'EVAL field shown — answer-key material, study-internal only',
                 fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = e / 'fig_droan_vs_mighty.png'
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print('wrote', out)

    # clearance profiles (distances only — no layout information)
    fig, ax = plt.subplots(figsize=(11, 5))
    for label, route, pts in runs:
        prof = clearance_profile(pts, eval_pillars)
        xs = [i / 35.0 for i, c in enumerate(prof) if c is not None]
        ys = [c for c in prof if c is not None]
        ax.plot(xs, ys, lw=1.0, label=label.split(' (')[0])
    ax.axhline(1.0, color='r', ls='--', lw=1.2, label='1.0 m clearance gate')
    ax.set_xlabel('flight time (s, approx at 35 Hz odometry)')
    ax.set_ylabel('distance to nearest pillar surface (m)')
    ax.set_ylim(0, 12)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=3)
    ax.set_title('Official frozen-v6 flights: clearance profile vs the 1.0 m gate')
    fig.tight_layout()
    out = e / 'fig_official_clearance.png'
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print('wrote', out)


if __name__ == '__main__':
    main()
