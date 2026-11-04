"""Explicit store-owned encoded payload capacity; staging/leases are excluded."""
import hashlib
import time
from Scripts.Setup.retained_inputs import EncodedInputBuffer
from Scripts.Setup.transition_feedback import input_available


class CapacityInputBuffer(EncodedInputBuffer):
    def __init__(self, root, rows, perturb, capacity_bytes=None):
        if capacity_bytes is not None and (type(capacity_bytes) is not int or capacity_bytes < 0):
            raise ValueError('capacity must be None or a nonnegative integer')
        super().__init__(root, rows, perturb)
        self.capacity_bytes = capacity_bytes
        self.evicted_pairs = 0
        self.rejected_pairs = 0

    @staticmethod
    def pair_bytes(entry):
        return sum(len(x['data']) for x in entry['images'].values())

    def resident_bytes(self):
        return sum(self.pair_bytes(e) for entries in self.entries.values() for e in entries.values())

    def advance(self, camera, source):
        if source != self.latest[camera] + 1:
            raise ValueError('source arrivals must advance one group')
        start = time.perf_counter_ns()
        self.latest[camera] = source
        expired = sorted(i for i in self.entries[camera] if i < source - 1)
        for i in expired:
            del self.entries[camera][i]
        metadata = {'camera': camera, 'source': source, 'expired_sources': expired,
                    'stored': False, 'evicted': [], 'rejected_oversize': False}
        if input_available(camera, source, self.perturb):
            # Staged ingress is deliberately outside the retained-store capacity.
            images = {}
            for side in ['left', 'right']:
                record = self.rows[source][camera + '_' + side]
                path = (self.root / record['path']).resolve()
                if not path.is_relative_to(self.root):
                    raise ValueError('retained input leaves dataset')
                data = path.read_bytes()
                images[side] = {'data': data, 'path': record['path'],
                                'sha256': hashlib.sha256(data).hexdigest(),
                                'bytes': len(data), 'time_ns': record['time_ns']}
                self.ingress_bytes += len(data)
            entry = {'source': source, 'camera': camera, 'images': images}
            metadata['images'] = {side: {k: v for k, v in x.items() if k != 'data'}
                                  for side, x in images.items()}
            size = self.pair_bytes(entry)
            if self.capacity_bytes is not None and size > self.capacity_bytes:
                self.rejected_pairs += 1
                metadata['rejected_oversize'] = True
            else:
                while self.capacity_bytes is not None and self.resident_bytes() + size > self.capacity_bytes:
                    oldest_source, oldest_camera = min((i, c) for c, entries in self.entries.items() for i in entries)
                    del self.entries[oldest_camera][oldest_source]
                    metadata['evicted'].append({'camera': oldest_camera, 'source': oldest_source})
                    self.evicted_pairs += 1
                self.entries[camera][source] = entry
                metadata['stored'] = True
        current = self.resident_bytes()
        self.peak_bytes = max(self.peak_bytes, current)
        metadata['retained_sources'] = sorted(self.entries[camera])
        metadata['resident'] = {c: sorted(entries) for c, entries in self.entries.items()}
        metadata['resident_bytes'] = current
        self.audit.append(metadata)
        self.io_ns += time.perf_counter_ns() - start

    def report(self):
        return dict(super().report(), capacity_bytes=self.capacity_bytes,
                    capacity_scope='store-owned encoded payload; staging, leases and total process memory excluded',
                    evicted_pairs=self.evicted_pairs, rejected_pairs=self.rejected_pairs)
