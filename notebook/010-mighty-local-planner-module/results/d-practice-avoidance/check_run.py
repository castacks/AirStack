#!/usr/bin/env python3
"""Clearance + in-order verdict for a practice-avoidance run.

Usage: check_run.py <odom_csv> "<x,y,z; x,y,z; ...>" [--layout layout.json]
                     [--clearance 1.0] [--arrival 1.5]

CSV columns 4,5,6 (0-based) are x,y,z — same convention as the study checker.
NOTE (carried to campaign v6): the study's gen_obstacles.py `check` always
loads the pinned eval layout; this script takes --layout explicitly.
"""
import argparse
import json
import math
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument('csv')
ap.add_argument('route')
ap.add_argument('--layout', default=str(Path(__file__).parent / 'layout_practice.json'))
ap.add_argument('--clearance', type=float, default=1.0)
ap.add_argument('--arrival', type=float, default=1.5)
a = ap.parse_args()

layout = json.load(open(a.layout))
pillars = [(p['x'], p['y'], p['r'], p['h']) for p in layout['pillars']]
wps = [tuple(map(float, w.split(','))) for w in a.route.split(';')]

pts = []
for line in Path(a.csv).read_text().splitlines():
    q = line.split(',')
    if len(q) >= 7:
        pts.append((float(q[4]), float(q[5]), float(q[6])))

arrivals = []
for w in wps:
    dmin, kmin = min((math.dist(p, w), k) for k, p in enumerate(pts))
    arrivals.append({'wp': w, 'closest_m': round(dmin, 2), 'sample': kmin,
                     'arrived': dmin <= a.arrival})
in_order = [x['sample'] for x in arrivals] == sorted(x['sample'] for x in arrivals)

worst, loc = math.inf, None
for x, y, z in pts:
    if z < 0.3:
        continue
    for px, py, pr, ph in pillars:
        if z > ph:
            continue
        d = math.dist((x, y), (px, py)) - pr
        if d < worst:
            worst, loc = d, (round(x, 2), round(y, 2), round(z, 2))

verdict = (in_order and all(x['arrived'] for x in arrivals)
           and worst >= a.clearance)
print(json.dumps({
    'samples': len(pts), 'arrivals': arrivals, 'in_order': in_order,
    'min_clearance_m': round(worst, 3), 'clearance_bar_m': a.clearance,
    'worst_at': loc, 'PASS': verdict}, indent=1))
raise SystemExit(0 if verdict else 1)
