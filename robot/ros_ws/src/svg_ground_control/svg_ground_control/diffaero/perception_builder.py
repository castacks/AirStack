from dataclasses import dataclass
import numpy as np


@dataclass
class Intrinsics:
    """Intrinsices of the sim/real camera (resolution that produces raw depth image)"""
    fx: float
    fy: float
    cx: float
    cy: float
    H: int # native render height
    W: int # native render width

@dataclass
class PerceptionGrid:
    """DiffAero's fixed grid set by the camera (static across different sims)"""
    H: int = 9
    W: int = 16
    max_dist: float = 5.0
    fov_deg: float = 86.0
    
class PerceptionBuilder:
    """
    Converts a native-resolution planar depth image into the 9x16 encoded
    perception vector that DiffAero expects.

    Pipeline per frame:
        planar (H x W)  ->  crop to 86° FOV  ->  planar-to-Euclidean
                        ->  min-pool to 9x16  ->  encode 1 - r/max_dist
    """

    def __init__(
        self,
        intrinsics: Intrinsics,
        grid: PerceptionGrid = PerceptionGrid(),
        flip_lr: bool = False,
        flip_ud: bool = False,
    ):
        self.grid = grid
        self.flip_lr = flip_lr
        self.flip_ud = flip_ud

        self._crop = _compute_crop(intrinsics, self.grid.fov_deg) # Used later to crop to DiffAero's training FOV
        self._euclid_scale = _compute_euclid_scale(intrinsics, self._crop) # Used later to convert planar depth to Euclidean distance
        # Used later to compute the min-pool edges (resizing to 9x16 DiffAero perception grid)
        self._row_edges, self._col_edges = _compute_pool_edges(
            crop=self._crop, out_h=self.grid.H, out_w=self.grid.W
        )

    def __call__(self, planar_native: np.ndarray) -> np.ndarray:
        planar = _clean(planar_native, self.grid.max_dist) # Replace NaN, inf, and zero-or-negative pixels with max_dist (= no obstacle)
        planar = _apply_crop(planar, self._crop) # Slice the native image to the training FOV region
        euclid = _planar_to_euclidean(planar, self._euclid_scale) # Element-wise: true_range = planar_Z * ray_scale
        pooled = _min_pool(euclid, self._row_edges, self._col_edges, self.grid.H, self.grid.W) # Reduce (crop_H, crop_W) -> (out_h, out_w) by taking the minimum range in each angular bin. Min = nearest obstacle surface per cell.
        pooled = _maybe_flip(pooled, self.flip_lr, self.flip_ud) # Flip the pooled image left-right and up-down if desired (for different simulation conventions)
        return _encode(pooled, self.grid.max_dist) # 1 = surface at lens, 0 = nothing within max_dist

def _compute_crop(intr: Intrinsics, fov_deg: float) -> tuple[int, int, int, int]:
    """
    Return (row0, row1, col0, col1) pixel slice that covers fov_deg from the
    optical axis. For a camera already at fov_deg this is the whole image.
    """
    half = np.radians(fov_deg / 2)
    half_w_px = intr.fx * np.tan(half)
    half_h_px = intr.fy * np.tan(half)
    col0 = max(0, int(np.floor(intr.cx - half_w_px)))
    col1 = min(intr.W, int(np.ceil(intr.cx + half_w_px)))
    row0 = max(0, int(np.floor(intr.cy - half_h_px)))
    row1 = min(intr.H, int(np.ceil(intr.cy + half_h_px)))
    return row0, row1, col0, col1


def _compute_euclid_scale(
    intr: Intrinsics, crop: tuple[int, int, int, int]
) -> np.ndarray:
    """
    Converts planar Z-depth to true 3D Euclidean range.
    Pre-sliced to the crop region so it matches the cropped depth shape.
    """
    # d = sqrt(1 + xn^2 + yn^2)
    u = np.arange(intr.W, dtype=np.float32)
    v = np.arange(intr.H, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)                  
    xn = (uu - intr.cx) / intr.fx
    yn = (vv - intr.cy) / intr.fy
    scale = np.sqrt(1.0 + xn * xn + yn * yn).astype(np.float32)  # (H, W)
    row0, row1, col0, col1 = crop
    return scale[row0:row1, col0:col1] # (crop_H, crop_W)


def _compute_pool_edges(
    crop: tuple[int, int, int, int], out_h: int, out_w: int
) -> tuple[np.ndarray, np.ndarray]:
    """
    linspace bin edges over the cropped image for the min-pool step.
    linspace handles non-integer cell sizes (e.g. 84/16 = 5.25).
    """
    row0, row1, col0, col1 = crop
    crop_H = row1 - row0
    crop_W = col1 - col0
    row_edges = np.linspace(0, crop_H, out_h + 1).astype(int)
    col_edges = np.linspace(0, crop_W, out_w + 1).astype(int)
    return row_edges, col_edges


def _clean(planar: np.ndarray, max_dist: float) -> np.ndarray:
    """Replace NaN, inf, and zero-or-negative pixels with max_dist (= no obstacle)."""
    d = np.nan_to_num(planar, nan=max_dist, posinf=max_dist, neginf=max_dist)
    return np.where(d <= 1e-3, max_dist, d).astype(np.float32)


def _apply_crop(
    planar: np.ndarray, crop: tuple[int, int, int, int]
) -> np.ndarray:
    """Slice the native image to the training FOV region."""
    row0, row1, col0, col1 = crop
    return planar[row0:row1, col0:col1]


def _planar_to_euclidean(planar_crop: np.ndarray, euclid_scale: np.ndarray) -> np.ndarray:
    """Element-wise: true_range = planar_Z * ray_scale."""
    return planar_crop * euclid_scale


def _min_pool(
    euclid: np.ndarray,
    row_edges: np.ndarray,
    col_edges: np.ndarray,
    out_h: int,
    out_w: int,
) -> np.ndarray:
    """
    Reduce (crop_H, crop_W) -> (out_h, out_w) by taking the minimum range
    in each angular bin. Min = nearest obstacle surface per cell.
    """
    out = np.empty((out_h, out_w), dtype=np.float32)
    for i in range(out_h):
        for j in range(out_w):
            cell = euclid[row_edges[i]:row_edges[i + 1],
                        col_edges[j]:col_edges[j + 1]]
            out[i, j] = float(cell.min())
    return out


def _maybe_flip(pooled: np.ndarray, flip_lr: bool, flip_ud: bool) -> np.ndarray:
    if flip_lr:
        pooled = pooled[:, ::-1].copy()
    if flip_ud:
        pooled = pooled[::-1, :].copy()
    return pooled


def _encode(euclid_9x16: np.ndarray, max_dist: float) -> np.ndarray:
    """1 = surface at lens, 0 = nothing within max_dist."""
    return 1.0 - np.clip(euclid_9x16, 0.0, max_dist) / max_dist
