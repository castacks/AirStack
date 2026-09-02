"""Behaviour priority — numpy port of the original RAVEN BehaviorManager.

Source: RayFronts_raven/rayfronts/behavior_manager.py

`self.behaviors = [voxel, ray, lvlm, frontier]` (OG:24) — the first whose
`condition_check` returns True wins, and frontier's is unconditionally True so
the chain always terminates. The OG's commented-out ablation lists
(OG:25-28) map onto our params: `frontier_only_baseline=true` is OG:28 and
`lvlm_enabled=false` is OG:25.

Mode switching resets the waypoint lock and both waypoints — OG
mapping_server_rosnode.py:387-388 + :508-511 (`mode_switch_trigger`).
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from raven_nav.behaviors.common import BehaviorOutput, TickContext
from raven_nav.behaviors.frontier_behavior import FrontierBehavior
from raven_nav.behaviors.lvlm_behavior import LvlmBehavior
from raven_nav.behaviors.ray_behavior import RayBehavior
from raven_nav.behaviors.voxel_behavior import VoxelBehavior


class BehaviorManager:
    def __init__(self, score_threshold: float = 0.95,
                 voxel_score_threshold: float = 0.98,
                 voxel_min_cluster_size: int = 30,
                 lvlm_enabled: bool = True,
                 lvlm_request_interval_s: float = 30.0,
                 lvlm_ray_threshold: float = 0.9,
                 frontier_only: bool = False,
                 rng: Optional[np.random.Generator] = None) -> None:
        self.behavior_mode = 'Frontier-based'
        self.frontier_only = bool(frontier_only)
        self.voxel_behavior = VoxelBehavior(
            score_threshold=voxel_score_threshold,
            min_cluster_size=voxel_min_cluster_size)
        self.ray_behavior = RayBehavior(score_threshold=score_threshold)
        self.lvlm_behavior = LvlmBehavior(
            request_interval_s=lvlm_request_interval_s,
            ray_threshold=lvlm_ray_threshold, enabled=lvlm_enabled)
        self.frontier_behavior = FrontierBehavior(rng=rng)
        # OG:24 — priority order voxel > ray > LVLM > frontier.
        self.behaviors = [self.voxel_behavior, self.ray_behavior,
                          self.lvlm_behavior, self.frontier_behavior]

    # ── per-tick perception (runs whatever the mode, so the frontier-only
    #    baseline still reports passive detections) ──────────────────────────
    def perceive(self, ctx: TickContext) -> None:
        self.voxel_behavior.update(ctx)
        self.ray_behavior.update(ctx)

    def mode_select(self, ctx: TickContext) -> str:
        """OG:30-34. Returns the selected mode name."""
        if self.frontier_only:
            # OG:28 `self.behaviors = [self.frontier_behavior]`
            self.behavior_mode = self.frontier_behavior.name
            return self.behavior_mode
        for behavior in self.behaviors:
            if behavior.condition_check(ctx):
                self.behavior_mode = behavior.name
                return self.behavior_mode
        self.behavior_mode = self.frontier_behavior.name
        return self.behavior_mode

    def behavior_execute(self, behavior_mode: str,
                         ctx: TickContext) -> BehaviorOutput:
        """OG:36-48."""
        for behavior in self.behaviors:
            if behavior.name == behavior_mode:
                return behavior.execute(ctx)
        return self.frontier_behavior.execute(ctx)

    @property
    def completed_queries(self) -> List[str]:
        return sorted({lbl for lbl, _c, _s
                       in self.voxel_behavior.visited_labels})
