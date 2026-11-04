"""Source-aware accumulation of complete query-semantic voxel maps.

Keep the latest observation of each world-grid cell per source. Repeated
snapshots never add votes; overlapping cells average the independent sources'
score vectors. No thresholding, target filtering, compression or BB transport.
"""
import numpy as np


class SemanticMapFusion:
    def __init__(self, resolution=0.5):
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError('semantic voxel resolution must be positive')
        self.resolution = float(resolution)
        self.sources = {}
        self.versions = {}
        self._cached = None

    def update(self, source, xyz, scores, labels, stamp):
        labels = tuple(labels)
        xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        scores = np.asarray(scores, dtype=np.float32)
        if scores.shape != (len(xyz), len(labels)) or not labels or len(set(labels)) != len(labels):
            raise ValueError('semantic map labels and score columns must match')
        if stamp <= self.versions.get(source, -1):
            return False
        if self.sources and labels != next(iter(self.sources.values()))[2]:
            raise ValueError('semantic map vocabulary/order differs from local queries')
        valid = np.isfinite(xyz).all(axis=1) & np.isfinite(scores).all(axis=1)
        xyz, scores = xyz[valid], scores[valid]
        keys = np.floor(xyz.astype(np.float64) / self.resolution).astype(np.int64)
        # Earlier cells persist when Rayfronts prunes its current snapshot.
        old = self.sources.get(source)
        if old is not None:
            keys = np.concatenate([old[0], keys])
            values = np.concatenate([old[1], np.c_[xyz, scores]])
        else:
            values = np.c_[xyz, scores]
        # Last observation wins within one source (including duplicate cells).
        _, reverse_idx = np.unique(keys[::-1], axis=0, return_index=True)
        idx = len(keys) - 1 - reverse_idx
        self.sources[source] = (keys[idx], values[idx], labels)
        self.versions[source] = stamp
        self._cached = None
        return True

    def arrays(self):
        if self._cached is None:
            if not self.sources:
                return None, None
            keys = np.concatenate([s[0] for s in self.sources.values()])
            values = np.concatenate([s[1] for s in self.sources.values()])
            _, inverse, count = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
            total = np.zeros((len(count), values.shape[1]), dtype=np.float64)
            np.add.at(total, inverse, values)
            merged = (total / count[:, None]).astype(np.float32)
            self._cached = merged[:, :3], merged[:, 3:]
        return self._cached
