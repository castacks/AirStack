"""Consensus-based auction for ray-mode target assignment.

Bids are computed from RayGroup descriptors. For now: one bid per label =
-min_dist_to_robot of the best group of that label. Higher bid = closer = wins.
Ties broken by lower robot id. Future heuristic factors (avg_score, num_rays,
angular spread) live on RayGroup so adding them is a one-line change here.
"""
from __future__ import annotations

from typing import Dict, List, Optional

from raven_nav.ray_groups import RayGroup


# Sentinel bid for a target this robot has won and is committed to. Pegged
# high enough that no realistic distance bid from a peer can outbid it.
LOCKED_BID = 1e9


def compute_my_bids(groups: List[RayGroup]) -> Dict[str, float]:
    bids: Dict[str, float] = {}
    for g in groups:
        b = -g.min_dist_to_robot
        if g.label not in bids or b > bids[g.label]:
            bids[g.label] = b
    return bids


def assign(
    my_id: int,
    my_bids: Dict[str, float],
    peer_bids: Dict[str, Dict[str, float]],
    peer_ids: Dict[str, int],
) -> Optional[str]:
    """Return the label this robot won (highest of its winning bids), or None.

    Winner of a label = robot with max bid; ties broken by lower id.
    """
    if not my_bids:
        return None
    won: Dict[str, float] = {}
    for label, my_bid in my_bids.items():
        winner_bid = my_bid
        winner_id = my_id
        for peer_name, bids in peer_bids.items():
            if label not in bids:
                continue
            pid = peer_ids.get(peer_name)
            if pid is None:
                continue
            pb = bids[label]
            if pb > winner_bid or (pb == winner_bid and pid < winner_id):
                winner_bid = pb
                winner_id = pid
        if winner_id == my_id:
            won[label] = my_bid
    if not won:
        return None
    return max(won.items(), key=lambda kv: kv[1])[0]
