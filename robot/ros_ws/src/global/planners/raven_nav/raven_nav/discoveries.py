"""Unified Discovery list — merges RayTargets and ConfirmedTarget AABBs.

A `Discovery` is the per-instance record consumed by the task executor and
visualizer. It's produced each tick from:
  - own voxel_behavior AABBs              (high confidence)
  - peer voxel_behavior AABBs (gossip)    (high confidence, merged)
  - RayTargets from triangulation         (lower confidence, status-flagged)

Two rules govern aggregation:
  1. ConfirmedTargets (AABBs) dedupe by spatial overlap — AABB intersection
     OR centroid-to-centroid distance below half_diag_A + half_diag_B + margin.
     Confidence/status of the merged entry takes the strongest input.
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


DEDUP_MARGIN_M = 1.0


@dataclass
class ConfirmedTarget:
    """A 3D AABB hypothesis fed in from voxel_behavior. Frame: local 'map'."""
    label: str
    center: np.ndarray  # (3,)
    size:   np.ndarray  # (3,) full extents
    status: str = 'observing'   # 'observing' | 'visited'
    confidence: float = 0.0
    ts: float = 0.0

    def half_diag(self) -> float:
        return float(np.linalg.norm(self.size) / 2.0)


@dataclass
class Discovery:
    instance_id: str
    label: str
    position: np.ndarray  # (3,) centroid (BB center or triangulated point)
    size: Optional[np.ndarray] = None  # (3,) when AABB known, else None
    status: str = 'unconfirmed'        # unconfirmed | confirmed | visited
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


def _should_merge(a: ConfirmedTarget, b: ConfirmedTarget,
                  margin: float = DEDUP_MARGIN_M) -> bool:
    if a.label != b.label:
        return False
    if aabb_overlap(a, b):
        return True
    d = float(np.linalg.norm(a.center - b.center))
    return d <= (a.half_diag() + b.half_diag() + margin)


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


def merge_confirmed_targets(
    sources: List[ConfirmedTarget],
) -> List[ConfirmedTarget]:
    """Dedup a flat list of ConfirmedTargets (own + every peer) into a clean set."""
    out: List[ConfirmedTarget] = []
    for ct in sources:
        merged_idx = None
        for i, existing in enumerate(out):
            if _should_merge(existing, ct):
                merged_idx = i
                break
        if merged_idx is None:
            out.append(ct)
        else:
            out[merged_idx] = _merge_two(out[merged_idx], ct)
    return out


def _stable_id(label: str, center: np.ndarray) -> str:
    snap = np.round(center, 0).astype(int)  # 1m grid snap
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
