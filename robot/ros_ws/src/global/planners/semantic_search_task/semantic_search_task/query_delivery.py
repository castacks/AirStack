"""Acknowledge reusable-map queries from the mapper's atomic snapshot."""
import os
from pathlib import Path

import numpy as np


class QueryDelivery:
    """Retry one absent label at a time, without changing query order or content.

    The existing initial burst is unchanged. Snapshot labels acknowledge encoding,
    unlike DDS discovery or a successful publish. Upstream deduplicates queries.
    """

    def __init__(self, robot_name, labels, interval_s=5.0):
        self.path = Path(os.getenv('RAVEN_MAP_SNAPSHOT_DIR', '/tmp')) / (
            f'rayfronts_features_{robot_name}.npz')
        self.labels = tuple(dict.fromkeys(labels))
        self.interval_s = interval_s
        self.next_check = 0.0

    def sent(self, now):
        self.next_check = now + self.interval_s

    def missing_query(self, now):
        if now < self.next_check:
            return None
        self.sent(now)
        try:
            with np.load(self.path, allow_pickle=False) as data:
                present = set(data['query_labels'].tolist())
        except (OSError, ValueError, KeyError, EOFError):
            # No acknowledgement yet. Do not create another full burst while
            # the mapper is loading or exporting its first snapshot.
            return None
        return next((label for label in self.labels if label not in present), None)
