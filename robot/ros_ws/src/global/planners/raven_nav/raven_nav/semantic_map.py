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
        if not valid.all():
            xyz, scores = xyz[valid], scores[valid]
        keys = np.floor(xyz.astype(np.float64) / self.resolution).astype(np.int64)
        # Earlier cells persist when Rayfronts prunes its current snapshot.
        old = self.sources.get(source)
        old_len = len(old[0]) if old is not None else 0
        if old is not None:
            keys = np.concatenate([old[0], keys])
        # Last observation wins within one source (including duplicate cells).
        _, reverse_idx = np.unique(keys[::-1], axis=0, return_index=True)
        idx = len(keys) - 1 - reverse_idx
        # Select winning rows before copying the wide feature matrix. Full
        # snapshots usually replace almost every old row; concatenating both
        # matrices first copies hundreds of MB that are immediately discarded.
        # The selected keys/order and float32 values are exactly the same.
        values = np.empty((len(idx), 3 + len(labels)), dtype=np.float32)
        old_rows = np.flatnonzero(idx < old_len)
        new_rows = np.flatnonzero(idx >= old_len)
        if len(old_rows):
            values[old_rows] = old[1][idx[old_rows]]
        new_idx = idx[new_rows] - old_len
        values[new_rows, :3] = xyz[new_idx]
        values[new_rows, 3:] = scores[new_idx]
        self.sources[source] = (keys[idx], values, labels)
        self.versions[source] = stamp
        self._cached = None
        return True

    def arrays(self):
        if self._cached is None:
            if not self.sources:
                return None, None
            keys = np.concatenate([s[0] for s in self.sources.values()])
            sources = list(self.sources.values())
            _, inverse, count = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
            columns = sources[0][1].shape[1]
            merged = np.empty((len(count), columns), dtype=np.float32)
            # Each source's unique grid keys (and therefore inverse indices)
            # are sorted. Work on contiguous rows across all feature channels.
            # Keep source order and float64 accumulation/division unchanged.
            indices = []
            offset = 0
            for _, values, _ in sources:
                indices.append(inverse[offset:offset+len(values)])
                offset += len(values)
            for start in range(0, len(count), 1024):
                end = min(start + 1024, len(count))
                total = np.zeros((end-start, columns), dtype=np.float64)
                for (_, values, _), index in zip(sources, indices):
                    lo, hi = np.searchsorted(index, (start, end))
                    total[index[lo:hi]-start] += values[lo:hi]
                total /= count[start:end, None]
                merged[start:end] = total
            self._cached = merged[:, :3], merged[:, 3:]
        return self._cached
