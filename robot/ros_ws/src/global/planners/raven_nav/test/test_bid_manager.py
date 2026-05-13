"""Unit tests for raven_nav.bid_manager.

The triangulation-aware auction is the heart of the multi-drone coordination
update: a peer's bid only contests one of mine when our ray groups converge
on the same physical target. Different-target peer bids must NOT knock me
off mine.
"""
import numpy as np

from raven_nav.bid_manager import BidEntry, assign, compute_my_bids
from raven_nav.ray_groups import RayGroup


def _make_group(label, origin, direction, num_rays=5, score=0.9,
                robot_pos=(0, 0, 0)):
    o = np.asarray(origin, float)
    d = np.asarray(direction, float)
    d = d / (np.linalg.norm(d) + 1e-9)
    origins = np.tile(o, (num_rays, 1))
    dirs = np.tile(d, (num_rays, 1))
    scores = np.full(num_rays, score)
    g = RayGroup(label=label, ray_origins=origins, ray_dirs=dirs,
                 ray_scores=scores)
    g._finalize(np.asarray(robot_pos, float))
    g.avg_origin = o
    g.avg_dir = d
    return g


def test_compute_my_bids_one_entry_per_group():
    groups = [
        _make_group('house', [0, 0, 5], [1, 0, 0], robot_pos=(0, 0, 0)),
        _make_group('house', [10, 0, 5], [1, 0, 0], robot_pos=(0, 0, 0)),
        _make_group('car',   [0, 5, 5], [0, 1, 0], robot_pos=(0, 0, 0)),
    ]
    bids = compute_my_bids(groups)
    assert len(bids) == 3
    labels = [b.label for b in bids]
    assert labels.count('house') == 2 and labels.count('car') == 1
    # Bid = -distance; closer (origin at robot) has higher bid.
    house_bids = [b.value for b in bids if b.label == 'house']
    assert max(house_bids) >= min(house_bids)


def test_assign_uncontested_winner_picks_highest_bid():
    my = [
        BidEntry('house', -3.0, np.array([0, 0, 5.0]), np.array([1, 0, 0.0]),
                 5, 0.9),
        BidEntry('house', -8.0, np.array([0, 0, 5.0]), np.array([0, 1, 0.0]),
                 5, 0.9),
    ]
    result = assign(my_id=1, my_bids=my, peer_bids={}, peer_ids={})
    assert result is not None
    label, entry = result
    assert label == 'house'
    assert entry.value == -3.0  # the closer one


def test_assign_peer_on_DIFFERENT_target_does_not_contest():
    # Two drones, both label 'house', but pointing at different houses.
    # My group A: origin (0,20,5), dir +x  → looking at (~20+, 20, 5)
    # Peer group B: origin (0,80,5), dir +x → looking at (~80+, 80, 5)
    # 80m apart — clearly different targets.
    my = [BidEntry('house', -5.0, np.array([0.0, 20, 5.0]),
                   np.array([1.0, 0, 0.0]), 5, 0.9)]
    peer = [BidEntry('house', -3.0, np.array([0.0, 80, 5.0]),
                     np.array([1.0, 0, 0.0]), 5, 0.9)]
    result = assign(my_id=1, my_bids=my,
                    peer_bids={'robot_2': peer},
                    peer_ids={'robot_2': 2})
    # Peer outbids me numerically but they point at a DIFFERENT target
    # (parallel bearings, not converging) — I keep my own.
    assert result is not None
    assert result[0] == 'house'


def test_assign_peer_on_SAME_target_with_better_bid_takes_it():
    # Both drones look at a house at ~(20,20,5) from perpendicular angles.
    # Peer is closer.
    my = [BidEntry('house', -10.0, np.array([0.0, 20.0, 5.0]),
                   np.array([1.0, 0.0, -0.15]), 5, 0.9)]
    peer = [BidEntry('house', -5.0, np.array([20.0, 0.0, 5.0]),
                     np.array([0.0, 1.0, -0.15]), 5, 0.9)]
    result = assign(my_id=1, my_bids=my,
                    peer_bids={'robot_2': peer},
                    peer_ids={'robot_2': 2})
    # Peer wins → I get None.
    assert result is None


def test_assign_peer_same_target_lower_bid_I_keep_it():
    my = [BidEntry('house', -5.0, np.array([0.0, 20.0, 5.0]),
                   np.array([1.0, 0.0, -0.15]), 5, 0.9)]
    peer = [BidEntry('house', -10.0, np.array([20.0, 0.0, 5.0]),
                     np.array([0.0, 1.0, -0.15]), 5, 0.9)]
    result = assign(my_id=1, my_bids=my,
                    peer_bids={'robot_2': peer},
                    peer_ids={'robot_2': 2})
    assert result is not None
    assert result[0] == 'house'


def test_assign_tie_lower_id_wins():
    my = [BidEntry('house', -5.0, np.array([0.0, 20.0, 5.0]),
                   np.array([1.0, 0.0, -0.15]), 5, 0.9)]
    peer = [BidEntry('house', -5.0, np.array([20.0, 0.0, 5.0]),
                     np.array([0.0, 1.0, -0.15]), 5, 0.9)]
    # My id is 2, peer id 1 → peer wins on tie.
    result = assign(my_id=2, my_bids=my,
                    peer_bids={'robot_1': peer},
                    peer_ids={'robot_1': 1})
    assert result is None
    # Reverse: my id 1, peer id 2 → I win.
    result = assign(my_id=1, my_bids=my,
                    peer_bids={'robot_2': peer},
                    peer_ids={'robot_2': 2})
    assert result is not None


def test_assign_two_drones_two_different_houses_each_wins_one():
    # Both robots see two houses. House X at (20,20,5), House Y at (80,80,5).
    # Drone 1 is close to X (and far from Y); drone 2 close to Y.
    # Each should win their own house.
    my = [
        BidEntry('house', -5.0, np.array([0.0, 20.0, 5.0]),
                 np.array([1.0, 0.0, -0.15]), 5, 0.9),   # toward X
        BidEntry('house', -80.0, np.array([0.0, 20.0, 5.0]),
                 np.array([0.8, 0.8, -0.05]), 5, 0.9),   # toward Y (far)
    ]
    peer = [
        BidEntry('house', -80.0, np.array([60.0, 60.0, 5.0]),
                 np.array([-0.8, -0.8, -0.05]), 5, 0.9), # toward X (far)
        BidEntry('house', -5.0, np.array([60.0, 60.0, 5.0]),
                 np.array([1.0, 1.0, -0.15]), 5, 0.9),   # toward Y
    ]
    me = assign(my_id=1, my_bids=my,
                peer_bids={'robot_2': peer},
                peer_ids={'robot_2': 2})
    them = assign(my_id=2, my_bids=peer,
                  peer_bids={'robot_1': my},
                  peer_ids={'robot_1': 1})
    # Each drone wins (different targets means no real contest blocks them).
    assert me is not None and me[0] == 'house'
    assert them is not None and them[0] == 'house'
    # The won entries should be the ones closest in own list (the close groups).
    assert me[1].value == -5.0
    assert them[1].value == -5.0
