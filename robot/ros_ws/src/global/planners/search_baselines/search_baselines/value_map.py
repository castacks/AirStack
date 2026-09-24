"""VLFM's value map: a scalar 'how promising' field over the ground plane.

This is the part of VLFM (Yokoyama et al., ICRA 2024) that makes it VLFM rather
than plain frontier exploration. The robot scores its CURRENT RGB view against
the target text, paints that scalar over the ground the camera can see, and
fuses it with what is already there. Frontier selection is then argmax over the
field, not nearest-first and not a VLM picking from a menu of pictures.

Two things are faithful to the paper and worth not "simplifying":

* **Confidence falls off with angle from the optical axis.** A view tells you
  most about what is straight ahead and least about what is at the edge of the
  frame, so a pixel's contribution is weighted by
  `cos^2( (theta / (fov/2)) * pi/2 )`. Without it, a glancing look at a corner
  updates the map as strongly as a square-on look.
* **Fusion is confidence-weighted, not a running average.** A confident new
  observation should overwrite a tentative old one, and two tentative looks
  should not add up to a confident belief:

      value_new = (c_new * v_new + c_old * v_old) / (c_new + c_old)
      conf_new  = (c_new^2   + c_old^2)          / (c_new + c_old)

  The confidence update is superlinear on purpose: repeated agreeing looks
  harden the estimate, which a plain mean never does.

What is NOT faithful: the paper scores with BLIP-2 ITM, which is a ~50 ms
similarity model. This stack has one generative VLM, so the scalar comes from
asking it for a number (see `vlfm_scorer`), at ~2.5 s per call. The map is the
same; the update RATE is 50x slower, which is a real difference and is why
`keyframe_period_s` exists.
"""

import math

import numpy as np


class ValueMap:
    """A (size, size) value field plus its confidence, in the same grid frame as
    the occupancy map — same origin, same resolution — so a frontier cell index
    reads straight across with no resampling."""

    def __init__(self, size, resolution_m, origin_cells):
        self.size = int(size)
        self.res = float(resolution_m)
        self.ox, self.oy = float(origin_cells[0]), float(origin_cells[1])
        self.value = np.zeros((self.size, self.size), dtype=np.float32)
        self.conf = np.zeros((self.size, self.size), dtype=np.float32)
        self._ii, self._jj = np.meshgrid(
            np.arange(self.size), np.arange(self.size), indexing='ij')

    # ── grid <-> map frame ────────────────────────────────────────────────
    def cell_of(self, x, y):
        return (int(np.floor(x / self.res) + self.ox),
                int(np.floor(-y / self.res) + self.oy))

    def value_at(self, i, j):
        if 0 <= i < self.size and 0 <= j < self.size:
            return float(self.value[i, j])
        return 0.0

    def observe(self, cam_xy, heading_rad, hfov_rad, max_range_m, score,
                min_range_m=0.0):
        """Paint one scored view into the field.

        `score` is either a single [0, 1] number for the whole view — canonical
        VLFM, where the scalar says nothing about WHERE in the frame the evidence
        was — or a LIST of per-column scores, left to right. The list form paints
        each column into its own angular slice of the cone, so a target off to
        one side raises the value in that bearing rather than smearing across the
        whole field of view. See `vlfm_scorer.score_view_tiled` for why the
        tiled form exists at all.
        """
        half = 0.5 * float(hfov_rad)
        if half <= 0.0 or max_range_m <= 0.0:
            return 0
        if np.isscalar(score) or (hasattr(score, '__len__') and len(score) == 1):
            cols = [float(np.clip(np.atleast_1d(score)[0], 0.0, 1.0))]
        else:
            cols = [float(np.clip(v, 0.0, 1.0)) for v in score]

        # Cell centres in map-frame metres, relative to the camera.
        cx = (self._ii - self.ox + 0.5) * self.res - cam_xy[0]
        cy = -((self._jj - self.oy + 0.5) * self.res) - cam_xy[1]
        dist = np.hypot(cx, cy)

        # Signed angle off the optical axis, wrapped to [-pi, pi].
        ang = np.arctan2(cy, cx) - heading_rad
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi

        cone = (dist <= max_range_m) & (dist >= min_range_m) & (np.abs(ang) <= half)
        if not cone.any():
            return 0

        # cos^2 falloff to zero at the edge of frame.
        c_new = np.zeros_like(self.conf)
        c_new[cone] = np.cos((np.abs(ang[cone]) / half) * (math.pi / 2.0)) ** 2

        # Per-cell VALUE: which image column does this bearing fall in? Column 0
        # is the LEFT of the image, which is the +angle (port) side of the cone,
        # so the mapping runs from +half down to -half.
        n = len(cols)
        v_new = np.zeros_like(self.value)
        if n == 1:
            v_new[cone] = cols[0]
        else:
            idx = np.clip(((half - ang[cone]) / (2.0 * half) * n).astype(int),
                          0, n - 1)
            v_new[cone] = np.asarray(cols, dtype=np.float32)[idx]

        c_old = self.conf
        v_old = self.value
        denom = c_new + c_old
        upd = cone & (denom > 1e-6)
        if upd.any():
            self.value[upd] = ((c_new[upd] * v_new[upd] + c_old[upd] * v_old[upd])
                               / denom[upd])
            self.conf[upd] = ((c_new[upd] ** 2 + c_old[upd] ** 2) / denom[upd])
        return int(cone.sum())

    def as_occupancy_bytes(self):
        """The field as 0..100 for a nav_msgs/OccupancyGrid, -1 where nothing has
        been scored yet so a viewer can draw it transparent."""
        out = np.full((self.size, self.size), -1, dtype=np.int8)
        seen = self.conf > 1e-6
        if seen.any():
            out[seen] = np.clip(self.value[seen] * 100.0, 0, 100).astype(np.int8)
        return out
