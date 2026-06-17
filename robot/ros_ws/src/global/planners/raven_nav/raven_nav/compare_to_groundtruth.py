#!/usr/bin/env python3
"""Score compiled fleet AABBs against ground-truth scene annotations.

Reads ``compiled_results.json`` (from ``compile_results``) and the scene's
ground-truth annotation file (``<scene>.json``: list of
``{class, bbox_world:{center_xyz_m, size_xyz_m}}`` in global ENU — the same
frame the compiled detections live in). Measured boxes never match GT exactly,
so matching is **tolerant**: a detection is a true positive for a GT box when
their 3D IoU ≥ ``--iou`` **or** their centroids are within ``--center-dist``
metres. Greedy one-to-one assignment, best pairs first.

Metrics are reported for two detection sets (see compile_results):
  - **either**  — observing ∪ visited (all detections).
  - **visited** — the subset the fleet actually closed out.
Each set yields TP/FP/FN, precision/recall/F1, mean IoU + mean localization
(centroid) and size error over matched pairs.

Annotations are bundled into the raven_nav package share
(``annotations/<scene>.json``); ``--scene RetroNeighborhood`` is the default.
Override with ``--annotations /path/to/file.json``.

Usage:
    python3 -m raven_nav.compare_to_groundtruth \
        --compiled /root/.cache/raven_results/compiled_results.json \
        --scene RetroNeighborhood --class-filter house
"""
from __future__ import annotations

import argparse
import json
import os
from typing import List, Optional

import numpy as np


def _aabb_iou(c_a, s_a, c_b, s_b) -> float:
    """3D IoU of two axis-aligned boxes given centre + full-extent size."""
    c_a, s_a, c_b, s_b = map(lambda v: np.asarray(v, dtype=float),
                             (c_a, s_a, c_b, s_b))
    lo_a, hi_a = c_a - s_a / 2.0, c_a + s_a / 2.0
    lo_b, hi_b = c_b - s_b / 2.0, c_b + s_b / 2.0
    inter_lo = np.maximum(lo_a, lo_b)
    inter_hi = np.minimum(hi_a, hi_b)
    inter = np.clip(inter_hi - inter_lo, 0.0, None)
    inter_vol = float(np.prod(inter))
    vol_a = float(np.prod(np.clip(hi_a - lo_a, 0.0, None)))
    vol_b = float(np.prod(np.clip(hi_b - lo_b, 0.0, None)))
    union = vol_a + vol_b - inter_vol
    return inter_vol / union if union > 0.0 else 0.0


def _resolve_annotations(scene: Optional[str],
                         annotations: Optional[str]) -> str:
    if annotations:
        return annotations
    scene = scene or 'RetroNeighborhood'
    # Prefer the copy bundled into the installed raven_nav package.
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('raven_nav')
        cand = os.path.join(share, 'annotations', f'{scene}.json')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    # Fallback for uninstalled runs: module dir or package root.
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, 'annotations', f'{scene}.json'),
                 os.path.join(here, os.pardir, 'annotations', f'{scene}.json')):
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError(
        f'No annotation file for scene {scene!r}; pass --annotations explicitly.')


def _parse_terms(class_filter: Optional[str]) -> List[str]:
    """A class filter may be a single substring or a comma-separated query
    ('Forklift, Towercrane, red container') → list of lowercased terms."""
    return [t.strip().lower() for t in (class_filter or '').split(',') if t.strip()]


def _class_compatible(a: str, b: str) -> bool:
    """Bidirectional substring match (case-insensitive). Handles GT classes that
    refine a query term, e.g. query 'towercrane' vs GT 'yellow towercrane'."""
    a, b = (a or '').lower().strip(), (b or '').lower().strip()
    return bool(a) and bool(b) and (a in b or b in a)


def _load_gt(path: str, class_filter: Optional[str]) -> List[dict]:
    with open(path) as f:
        data = json.load(f)
    terms = _parse_terms(class_filter)
    out = []
    for item in data:
        cls = str(item.get('class', ''))
        if terms and not any(_class_compatible(t, cls) for t in terms):
            continue
        bw = item.get('bbox_world', {})
        out.append({
            'class': item.get('class', ''),
            'center': np.asarray(bw.get('center_xyz_m', [0, 0, 0]), dtype=float),
            'size': np.asarray(bw.get('size_xyz_m', [0, 0, 0]), dtype=float),
        })
    return out


def _match(dets: List[dict], gts: List[dict],
           iou_thresh: float, center_dist: float,
           class_aware: bool = False) -> dict:
    """Greedy tolerant one-to-one matching. Returns metrics + pair details.

    With ``class_aware`` a detection may only match a GT box whose class is
    compatible with the detection's own label (so a 'water tower' detection
    cannot be credited against a 'red building' GT just by proximity). This
    matters for multi-class queries; for single-class scenes it is a no-op."""
    # Build all eligible candidate pairs with their IoU + centroid distance.
    cands = []
    for di, d in enumerate(dets):
        for gi, g in enumerate(gts):
            if class_aware and not _class_compatible(d.get('label', ''),
                                                     g['class']):
                continue
            iou = _aabb_iou(d['center'], d['size'], g['center'], g['size'])
            dist = float(np.linalg.norm(
                np.asarray(d['center']) - np.asarray(g['center'])))
            if iou >= iou_thresh or dist <= center_dist:
                # Rank key: prefer high IoU, then small distance.
                cands.append((iou, -dist, di, gi, dist))
    cands.sort(reverse=True)

    used_d, used_g = set(), set()
    pairs = []
    for iou, _negdist, di, gi, dist in cands:
        if di in used_d or gi in used_g:
            continue
        used_d.add(di)
        used_g.add(gi)
        d, g = dets[di], gts[gi]
        size_err = float(np.linalg.norm(
            np.asarray(d['size']) - np.asarray(g['size'])))
        pairs.append({
            'det_center': np.asarray(d['center']).tolist(),
            'gt_class': g['class'],
            'gt_center': np.asarray(g['center']).tolist(),
            'iou': iou,
            'center_error_m': dist,
            'size_error_m': size_err,
        })

    tp = len(pairs)
    fp = len(dets) - tp
    fn = len(gts) - tp
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = (2 * precision * recall / (precision + recall)
          if (precision + recall) else 0.0)
    ious = [p['iou'] for p in pairs]
    cerr = [p['center_error_m'] for p in pairs]
    serr = [p['size_error_m'] for p in pairs]
    return {
        'num_detections': len(dets),
        'num_ground_truth': len(gts),
        'tp': tp, 'fp': fp, 'fn': fn,
        'precision': precision, 'recall': recall, 'f1': f1,
        'mean_iou_matched': float(np.mean(ious)) if ious else 0.0,
        'mean_center_error_m': float(np.mean(cerr)) if cerr else None,
        'mean_size_error_m': float(np.mean(serr)) if serr else None,
        'matches': pairs,
    }


def _dets_from_compiled(compiled: dict, visited_only: bool) -> List[dict]:
    out = []
    for h in compiled.get('houses', []):
        if visited_only and str(h.get('status', '')).lower() != 'visited':
            continue
        out.append({'label': h.get('label', ''),
                    'center': np.asarray(h['center_enu'], dtype=float),
                    'size': np.asarray(h['size'], dtype=float)})
    return out


def compare(compiled_path: str, scene: Optional[str],
            annotations: Optional[str], class_filter: Optional[str],
            iou_thresh: float, center_dist: float,
            class_aware: bool = False) -> dict:
    with open(compiled_path) as f:
        compiled = json.load(f)
    ann_path = _resolve_annotations(scene, annotations)
    gts = _load_gt(ann_path, class_filter)

    report = {
        'compiled_path': compiled_path,
        'annotations_path': ann_path,
        'scene': scene,
        'class_filter': class_filter,
        'class_aware': class_aware,
        'iou_threshold': iou_thresh,
        'center_dist_threshold_m': center_dist,
        'num_ground_truth': len(gts),
        # observing ∪ visited (all detections) vs the visited-only subset.
        'either': _match(_dets_from_compiled(compiled, False), gts,
                         iou_thresh, center_dist, class_aware),
        'visited': _match(_dets_from_compiled(compiled, True), gts,
                          iou_thresh, center_dist, class_aware),
    }
    return report


def _print_set(name: str, m: dict) -> None:
    print(f"  [{name}] det={m['num_detections']} gt={m['num_ground_truth']} "
          f"TP={m['tp']} FP={m['fp']} FN={m['fn']} | "
          f"P={m['precision']:.2f} R={m['recall']:.2f} F1={m['f1']:.2f} | "
          f"meanIoU={m['mean_iou_matched']:.2f} "
          f"locErr={m['mean_center_error_m'] if m['mean_center_error_m'] is not None else float('nan'):.2f}m")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--compiled', required=True,
                    help='Path to compiled_results.json.')
    ap.add_argument('--scene', default='RetroNeighborhood',
                    help='Scene name → bundled annotations/<scene>.json.')
    ap.add_argument('--annotations', default=None,
                    help='Explicit annotation file path (overrides --scene).')
    ap.add_argument('--class-filter', default='house',
                    help="Comma-separated GT class substrings to score (e.g. the "
                         "search query 'red building, water tower'); default "
                         "'house'. Empty string scores all GT classes.")
    ap.add_argument('--class-aware', action='store_true',
                    help='Require a detection label to be class-compatible with '
                         'the GT box before it can match (multi-class scenes).')
    ap.add_argument('--iou', type=float, default=0.1,
                    help='Min 3D IoU for a tolerant match (default 0.1).')
    ap.add_argument('--center-dist', type=float, default=10.0,
                    help='Max centroid distance (m) for a tolerant match.')
    ap.add_argument('--out', default=None,
                    help='Where to write the report JSON (default: next to '
                         'compiled_results.json as groundtruth_comparison.json).')
    args = ap.parse_args(argv)

    report = compare(args.compiled, args.scene, args.annotations,
                     args.class_filter, args.iou, args.center_dist,
                     args.class_aware)

    out = args.out or os.path.join(os.path.dirname(args.compiled),
                                   'groundtruth_comparison.json')
    with open(out, 'w') as f:
        json.dump(report, f, indent=2)

    print(f"[compare] scene={args.scene} class~{args.class_filter!r} "
          f"GT={report['num_ground_truth']} "
          f"(iou>={args.iou}, center<={args.center_dist}m)")
    _print_set('either ', report['either'])
    _print_set('visited', report['visited'])
    print(f"[compare] wrote {out}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
