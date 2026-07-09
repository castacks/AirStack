"""Unified Discovery list — merges RayTargets and ConfirmedTarget AABBs.

A `Discovery` is the per-instance record consumed by the task executor and
visualizer. It's produced each tick from:
  - own voxel_behavior AABBs              (high confidence)
  - peer voxel_behavior AABBs (gossip)    (high confidence, merged)
  - RayTargets from triangulation         (lower confidence, status-flagged)

Two rules govern aggregation:
  1. ConfirmedTargets (AABBs) dedupe by spatial overlap — AABB intersection
     OR surface gap below margin. Confidence/status takes the strongest input.
  2. RayTargets that were attached to a BB during build_targets are absorbed
     into the corresponding ConfirmedTarget. RayTargets without a bb_id stay
     as separate Discoveries with status='confirmed' or 'unconfirmed'.

IDs are stable across ticks via a hash of (label, snapped centroid) so the
visualizer and task executor see the same instance index frame-to-frame.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from typing import List, Optional

import numpy as np

from raven_nav.ray_targets import RayTarget


# Max surface gap (not centroid distance) for two same-label AABBs to count as
# one instance. Detected boxes run ~1-2 m oversized, so anything above ~1 m
# bridges the street gap between adjacent buildings (RetroNeighborhood min
# true gap: 5.5 m) and chains a row into one box.
DEDUP_MARGIN_M = 1.0

# Merged boxes never grow beyond this xy extent (> largest expected target):
# blocks fragment-union chaining across neighbouring targets. A fragment that
# would over-grow the union stays a separate box.
MAX_TARGET_EXTENT_M = 28.0

# Same-instance identity for VISITED transfer is overlap-based, not gap-based:
# fragments of one large building sit farther apart (<= ~12 m for 25 m houses)
# than two adjacent buildings' gap (5.5 m), so no distance radius separates
# them. Small gap allowance for offset zero-overlap fragments only.
SAME_INSTANCE_IOU = 0.15
SAME_INSTANCE_CONTAIN = 0.5
SAME_INSTANCE_GAP_M = 1.0


def same_instance_xy(ca, sa, cb, sb) -> bool:
    """True if two xy AABBs (centre, size) are one physical instance:
    meaningful overlap, one mostly inside the other, or near-touching."""
    ca, sa, cb, sb = (np.asarray(v, dtype=float) for v in (ca, sa, cb, sb))
    lo = np.maximum(ca[:2] - sa[:2] / 2.0, cb[:2] - sb[:2] / 2.0)
    hi = np.minimum(ca[:2] + sa[:2] / 2.0, cb[:2] + sb[:2] / 2.0)
    inter = float(np.prod(np.clip(hi - lo, 0.0, None)))
    if inter > 0.0:
        va = float(np.prod(np.maximum(sa[:2], 1e-6)))
        vb = float(np.prod(np.maximum(sb[:2], 1e-6)))
        iou = inter / (va + vb - inter)
        if iou >= SAME_INSTANCE_IOU or inter / min(va, vb) >= SAME_INSTANCE_CONTAIN:
            return True
    gap = np.maximum(np.abs(ca[:2] - cb[:2]) - (sa[:2] + sb[:2]) / 2.0, 0.0)
    return float(np.linalg.norm(gap)) <= SAME_INSTANCE_GAP_M


@dataclass
class ConfirmedTarget:
    """A 3D AABB hypothesis fed in from voxel_behavior. Frame: local 'map'."""
    label: str
    center: np.ndarray
    size:   np.ndarray
    status: str = 'observing'
    confidence: float = 0.0
    ts: float = 0.0


@dataclass
class Discovery:
    instance_id: str
    label: str
    position: np.ndarray
    size: Optional[np.ndarray] = None
    status: str = 'unconfirmed'
    confidence: float = 0.0
    contributing_robots: List[str] = field(default_factory=list)
    last_update_ts: float = 0.0


def aabb_overlap(a: ConfirmedTarget, b: ConfirmedTarget) -> bool:
    ha = a.size / 2.0
    hb = b.size / 2.0
    return bool(
        abs(a.center[0] - b.center[0]) <= (ha[0] + hb[0]) and
        abs(a.center[1] - b.center[1]) <= (ha[1] + hb[1]) and
        abs(a.center[2] - b.center[2]) <= (ha[2] + hb[2])
    )


def aabb_surface_gap(a: ConfirmedTarget, b: ConfirmedTarget) -> float:
    """Shortest distance between the two AABBs' surfaces. 0 if they overlap or
    one is a subset of the other."""
    ha = a.size / 2.0
    hb = b.size / 2.0
    gap = np.maximum(np.abs(a.center - b.center) - (ha + hb), 0.0)
    return float(np.linalg.norm(gap))


def _xy_inter_area(a: ConfirmedTarget, b: ConfirmedTarget) -> float:
    """Intersection area of the two boxes' XY footprints. XY-projection (not 3D)
    is deliberate: a tall structure that clusters into stacked z-layers (same xy,
    disjoint z) is ONE physical instance — 3D intersection wrongly splits it and
    inflates the detection count."""
    lo = np.maximum(a.center[:2] - a.size[:2] / 2.0, b.center[:2] - b.size[:2] / 2.0)
    hi = np.minimum(a.center[:2] + a.size[:2] / 2.0, b.center[:2] + b.size[:2] / 2.0)
    return float(np.prod(np.clip(hi - lo, 0.0, None)))


def _xy_surface_gap(a: ConfirmedTarget, b: ConfirmedTarget) -> float:
    """Shortest distance between the two boxes' XY footprints (0 if they overlap)."""
    gap = np.maximum(
        np.abs(a.center[:2] - b.center[:2]) - (a.size[:2] + b.size[:2]) / 2.0, 0.0)
    return float(np.linalg.norm(gap))


def _should_merge(a: ConfirmedTarget, b: ConfirmedTarget,
                  margin: float = DEDUP_MARGIN_M) -> bool:
    if a.label != b.label:
        return False
    if _xy_inter_area(a, b) > 0.0:
        return True
    return _xy_surface_gap(a, b) <= margin


def _merge_two(a: ConfirmedTarget, b: ConfirmedTarget) -> ConfirmedTarget:
    """AABB-union and pick the stronger status / confidence / freshest ts."""
    a_lo = a.center - a.size / 2.0
    a_hi = a.center + a.size / 2.0
    b_lo = b.center - b.size / 2.0
    b_hi = b.center + b.size / 2.0
    lo = np.minimum(a_lo, b_lo)
    hi = np.maximum(a_hi, b_hi)
    center = (lo + hi) / 2.0
    size = hi - lo
    # 'visited' is sticky; otherwise prefer the higher-confidence input.
    if a.status == 'visited' or b.status == 'visited':
        status = 'visited'
    else:
        status = a.status if a.confidence >= b.confidence else b.status
    return ConfirmedTarget(
        label=a.label, center=center, size=size, status=status,
        confidence=max(a.confidence, b.confidence),
        ts=max(a.ts, b.ts),
    )


# Cross-robot "same target" test: two boxes coincide enough to be one physical
# target — meaningful overlap (IoU) or one largely inside the other. Unlike the
# gap-based local combine, this never chains across distinct adjacent targets.
SIMILAR_IOU = 0.1
SIMILAR_CONTAIN = 0.5


def _aabb_inter_vol(a: ConfirmedTarget, b: ConfirmedTarget) -> float:
    inter = np.clip(
        np.minimum(a.center + a.size / 2.0, b.center + b.size / 2.0)
        - np.maximum(a.center - a.size / 2.0, b.center - b.size / 2.0), 0.0, None)
    return float(np.prod(inter))


def _same_target(a: ConfirmedTarget, b: ConfirmedTarget) -> bool:
    if a.label != b.label:
        return False
    iv = _xy_inter_area(a, b)          # XY-projected: fuse stacked z-fragments
    if iv <= 0.0:
        return False
    va = float(np.prod(np.maximum(a.size[:2], 1e-6)))
    vb = float(np.prod(np.maximum(b.size[:2], 1e-6)))
    iou = iv / (va + vb - iv) if (va + vb - iv) > 0 else 0.0
    contain = iv / max(min(va, vb), 1e-9)
    return iou >= SIMILAR_IOU or contain >= SIMILAR_CONTAIN


def _union_within_cap(a: ConfirmedTarget, b: ConfirmedTarget) -> bool:
    """Union may not grow past MAX_TARGET_EXTENT_M in xy (absorbing a
    contained fragment into an already-large box is fine — no growth)."""
    lo = np.minimum(a.center - a.size / 2.0, b.center - b.size / 2.0)
    hi = np.maximum(a.center + a.size / 2.0, b.center + b.size / 2.0)
    ext = (hi - lo)[:2]
    cur = np.maximum(a.size[:2], b.size[:2])
    return bool(np.all((ext <= MAX_TARGET_EXTENT_M) | (ext <= cur + 1e-6)))


def _greedy_merge(sources, predicate) -> List[ConfirmedTarget]:
    # Canonical sort so the greedy merge is order-independent (same set per robot).
    sources = sorted(
        sources,
        key=lambda ct: (str(ct.label), float(ct.center[0]),
                        float(ct.center[1]), float(ct.center[2])))
    out: List[ConfirmedTarget] = []
    for ct in sources:
        idx = next((i for i, e in enumerate(out)
                    if predicate(e, ct) and _union_within_cap(e, ct)), None)
        if idx is None:
            out.append(ct)
        else:
            out[idx] = _merge_two(out[idx], ct)
    return out


def _fold_into_rep(rep: ConfirmedTarget, other: ConfirmedTarget) -> ConfirmedTarget:
    """Suppress `other` into representative `rep`: keep rep's GEOMETRY unchanged
    (this is what makes the operator runaway-proof — a box is never grown by
    union), but inherit the stronger evidence: 'visited' is sticky, confidence
    and freshness take the max."""
    status = ('visited'
              if 'visited' in (str(rep.status).lower(), str(other.status).lower())
              else rep.status)
    return ConfirmedTarget(
        label=rep.label, center=rep.center, size=rep.size, status=status,
        confidence=max(rep.confidence, other.confidence),
        ts=max(rep.ts, other.ts))


def nms_dedupe(sources, predicate,
               max_extent_by_label=None) -> List[ConfirmedTarget]:
    """Non-Max-Suppression-style keep-representative dedupe (replaces union-merge).

    Greedy agglomerative *union* chains: a box grows to touch the next fragment,
    the grown box reaches the one after, and a whole row collapses into one
    ballooned mega-box (only `MAX_TARGET_EXTENT_M` capped it). Here the merge
    operator is SUPPRESSION instead: keep the strongest box per instance and
    fold matching fragments/duplicates into it WITHOUT resizing it. The output
    box is therefore always a real detection and can never balloon — distinct
    neighbours (which don't match a *fixed* representative) always stay split.

    Ranking: larger footprint first (most evidence / fullest extent, and lets a
    visited/observing fragment inherit onto the fuller box), with the canonical
    centre as a deterministic tie-break so every robot keeps the same rep.
    `max_extent_by_label`: optional per-class hard cap — a box whose xy extent
    exceeds its class prior is dropped as a mis-detection (a lattice tower can't
    be 40 m wide)."""
    def _rank(ct):
        vol = float(np.prod(np.maximum(np.abs(ct.size), 1e-6)))
        return (-vol, float(ct.center[0]), float(ct.center[1]), float(ct.center[2]))

    kept: List[ConfirmedTarget] = []
    for ct in sorted(sources, key=_rank):
        if max_extent_by_label is not None:
            cap = max_extent_by_label.get(ct.label)
            if cap is not None and float(np.max(np.abs(ct.size[:2]))) > cap:
                continue
        idx = next((i for i, e in enumerate(kept) if predicate(e, ct)), None)
        if idx is None:
            kept.append(ct)
        else:
            kept[idx] = _fold_into_rep(kept[idx], ct)
    return kept


def merge_confirmed_targets(sources: List[ConfirmedTarget]) -> List[ConfirmedTarget]:
    """Local combine: dedupe a robot's own fragments (xy-overlap or gap within
    margin) into one box per target before transmit. Keep-representative (NMS) so
    stacked z-fragments collapse without the union operator ballooning the box."""
    return nms_dedupe(sources, _should_merge)


def merge_similar_targets(sources: List[ConfirmedTarget]) -> List[ConfirmedTarget]:
    """Cross-robot merge: fuse boxes that are the SAME target (xy overlap /
    containment); dissimilar boxes stay split. Keep-representative (NMS): the
    kept box is a real detection, never a chained union across neighbours."""
    return nms_dedupe(sources, _same_target)


def _house_pred(a: ConfirmedTarget, b: ConfirmedTarget) -> bool:
    if a.label != b.label:
        return False
    if _same_target(a, b):
        return True
    if a.status == 'visited' or b.status == 'visited':
        # Overlap-based (xy): a visited fragment flips only the box it is
        # actually part of — a gap radius here cascades visits across
        # adjacent buildings (measured: 27-60% spurious visits).
        return same_instance_xy(a.center, a.size, b.center, b.size)
    return False


def merge_house_boxes(sources: List[ConfirmedTarget]) -> List[ConfirmedTarget]:
    """One box per house for display + visited-flip: same-target boxes coincide,
    and a visited fragment flips the same-instance (overlapping) box. Keep-
    representative (NMS): visited status is inherited onto the fuller box without
    union growth (visited stays sticky via _fold_into_rep)."""
    return nms_dedupe(sources, _house_pred)


def _stable_id(label: str, center: np.ndarray) -> str:
    snap = np.round(center, 0).astype(int)
    h = hashlib.md5(f"{label}|{snap[0]},{snap[1]},{snap[2]}".encode()).hexdigest()
    return h[:10]


def build_discoveries(
    ray_targets: List[RayTarget],
    confirmed_targets: List[ConfirmedTarget],
    contributing_robot: str,
    peer_contributions: Optional[List[str]] = None,
    now_ts: float = 0.0,
) -> List[Discovery]:
    """Fold ray targets + confirmed AABBs into one Discovery list.

    `contributing_robots` per Discovery records who saw it — used by the
    task executor to know whether a discovery is single-source or
    multi-confirmed across the fleet.
    """
    discoveries: List[Discovery] = []

    # 1. ConfirmedTargets first (each gets a Discovery with size).
    ct_id_to_disc_idx = {}
    for ct in confirmed_targets:
        disc = Discovery(
            instance_id=_stable_id(ct.label, ct.center),
            label=ct.label,
            position=np.asarray(ct.center, dtype=float),
            size=np.asarray(ct.size, dtype=float),
            status=ct.status,
            confidence=ct.confidence,
            contributing_robots=[contributing_robot],
            last_update_ts=max(ct.ts, now_ts),
        )
        if peer_contributions:
            disc.contributing_robots.extend(peer_contributions)
        discoveries.append(disc)
        ct_id_to_disc_idx[id(ct)] = len(discoveries) - 1

    # 2. RayTargets — those that bound to a BB during build_targets get folded
    # in by spatial proximity (bb_id is local to ray_targets, but the BB
    # coordinates match). Others become standalone Discoveries.
    for rt in ray_targets:
        absorbed = False
        if rt.status == 'confirmed' and rt.bb_id is not None:
            for i, disc in enumerate(discoveries):
                if disc.label != rt.label:
                    continue
                if np.linalg.norm(disc.position - rt.position) <= 2.0:
                    # Boost confidence: rays + BB beats BB alone.
                    disc.confidence = max(disc.confidence, rt.confidence)
                    discoveries[i] = disc
                    absorbed = True
                    break
        if absorbed:
            continue
        discoveries.append(Discovery(
            instance_id=_stable_id(rt.label, rt.position),
            label=rt.label,
            position=np.asarray(rt.position, dtype=float),
            size=None,
            status=rt.status,
            confidence=rt.confidence,
            contributing_robots=[contributing_robot],
            last_update_ts=rt.last_update_ts or now_ts,
        ))

    return discoveries


def discoveries_to_json(discoveries: List[Discovery]) -> str:
    """Serialize for the /raven_nav/discoveries String topic."""
    out = []
    for d in discoveries:
        item = {
            'instance_id': d.instance_id,
            'label': d.label,
            'cx': float(d.position[0]),
            'cy': float(d.position[1]),
            'cz': float(d.position[2]),
            'status': d.status,
            'confidence': float(d.confidence),
            'contributors': list(d.contributing_robots),
            'ts': float(d.last_update_ts),
        }
        if d.size is not None:
            item['sx'] = float(d.size[0])
            item['sy'] = float(d.size[1])
            item['sz'] = float(d.size[2])
        out.append(item)
    return json.dumps(out)


def confirmed_targets_to_json(targets: List[ConfirmedTarget]) -> str:
    """Serialize ConfirmedTargets for the gossip payload."""
    out = []
    for t in targets:
        out.append({
            'label': t.label,
            'cx': float(t.center[0]),
            'cy': float(t.center[1]),
            'cz': float(t.center[2]),
            'sx': float(t.size[0]),
            'sy': float(t.size[1]),
            'sz': float(t.size[2]),
            'status': t.status,
            'confidence': float(t.confidence),
            'ts': float(t.ts),
        })
    return json.dumps(out)
