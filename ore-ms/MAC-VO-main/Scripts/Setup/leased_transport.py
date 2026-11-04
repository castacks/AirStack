"""Single-owner encoded handover arena and bounded synchronous control channel."""
from contextlib import contextmanager
import hashlib
from multiprocessing import shared_memory
from multiprocessing.reduction import ForkingPickler


class BoundedConnection:
    def __init__(self, connection):
        self.connection = connection
        self.sent = self.received = self.max_sent_bytes = self.max_received_bytes = 0
        self.max_step_bytes = 0

    @staticmethod
    def encode(value):
        data = ForkingPickler.dumps(value)
        step = isinstance(value, tuple) and value and value[0] == 'step'
        if len(data) > (4096 if step else 65536):
            raise ValueError('control message exceeds serialized byte ceiling')
        if step and value[3] is not None:
            def scalars(x):
                if isinstance(x, dict): return all(isinstance(k, str) and scalars(v) for k, v in x.items())
                return type(x) in [str, int, bool, type(None)]
            if not scalars(value[3]): raise ValueError('step requires scalar descriptor, never encoded bytes')
        return data

    def send(self, value):
        data = self.encode(value)
        self.connection.send_bytes(data)
        self.sent += 1; self.max_sent_bytes = max(self.max_sent_bytes, len(data))
        if isinstance(value, tuple) and value and value[0] == 'step':
            self.max_step_bytes = max(self.max_step_bytes, len(data))

    def recv(self):
        data = self.connection.recv_bytes(65536)
        value = ForkingPickler.loads(data)
        self.encode(value)  # same schema/step ceiling at both endpoints
        self.received += 1; self.max_received_bytes = max(self.max_received_bytes, len(data))
        return value

    def poll(self, timeout): return self.connection.poll(timeout)
    def close(self): self.connection.close()
    def report(self):
        return {k: getattr(self, k) for k in ['sent', 'received', 'max_sent_bytes', 'max_received_bytes', 'max_step_bytes']}


class EncodedLeaseArena:
    def __init__(self, capacity_bytes=2097152):
        if type(capacity_bytes) is not int or capacity_bytes <= 0: raise ValueError('positive arena capacity required')
        self.memory = shared_memory.SharedMemory(create=True, size=capacity_bytes)
        self.capacity_bytes = capacity_bytes; self.active = None; self.token = 0
        self.peak_leased_bytes = 0; self.leases = self.rejections = self.acks = 0

    def publish(self, payload, camera, source, group):
        if self.active is not None: raise RuntimeError('handover slot busy until matching acknowledgment')
        if payload['camera'] != camera or payload['source'] != source or source != group - 1:
            raise ValueError('lease identity mismatch')
        images = payload['images']; total = sum(len(images[s]['data']) for s in ['left', 'right'])
        for image in images.values():
            if len(image['data']) != image['bytes'] or hashlib.sha256(image['data']).hexdigest() != image['sha256']:
                raise ValueError('lease input integrity mismatch')
        if total > self.capacity_bytes:
            self.rejections += 1; return None
        self.token += 1; offset = 0; encoded = {}
        for side in ['left', 'right']:
            image = images[side]; size = len(image['data'])
            self.memory.buf[offset:offset+size] = image['data']
            encoded[side] = dict({k: v for k, v in image.items() if k != 'data'}, offset=offset)
            offset += size
        identity = {'token': self.token, 'camera': camera, 'source': source, 'group': group}
        descriptor = dict(identity, transport='shared-encoded-v1', name=self.memory.name,
                          payload_bytes=total, images=encoded)
        self.active = identity; self.leases += 1; self.peak_leased_bytes = max(self.peak_leased_bytes, total)
        return descriptor

    def acknowledge(self, identity):
        if self.active is None or identity != self.active:
            raise ValueError('wrong, missing or stale lease acknowledgment')
        self.active = None; self.acks += 1

    def report(self):
        return {'allocated_bytes': self.capacity_bytes, 'peak_leased_bytes': self.peak_leased_bytes,
                'leases': self.leases, 'acks': self.acks, 'oversize_rejections': self.rejections,
                'active': self.active, 'slots': 1}

    def close(self):
        self.memory.close(); self.memory.unlink()


@contextmanager
def mapped_payload(descriptor, camera, group):
    if descriptor is None:
        yield None; return
    if descriptor['transport'] != 'shared-encoded-v1' or descriptor['camera'] != camera or descriptor['group'] != group or descriptor['source'] != group - 1:
        raise ValueError('shared descriptor identity mismatch')
    memory = shared_memory.SharedMemory(name=descriptor['name'])
    images = {}; offset = 0
    try:
        for side in ['left', 'right']:
            image = descriptor['images'][side]; size = image['bytes']
            if type(size) is not int or size < 0 or image['offset'] != offset or offset + size > memory.size:
                raise ValueError('shared descriptor bounds mismatch')
            view = memory.buf[offset:offset+size]
            images[side] = dict(image, data=view)
            if hashlib.sha256(view).hexdigest() != image['sha256']: raise ValueError('shared encoded integrity mismatch')
            offset += size
        if offset != descriptor['payload_bytes']: raise ValueError('shared descriptor total mismatch')
        yield {'camera': camera, 'source': descriptor['source'], 'images': images}
    finally:
        for image in images.values(): image.pop('data').release()
        memory.close()


def lease_identity(descriptor):
    return {k: descriptor[k] for k in ['token', 'camera', 'source', 'group']} if descriptor else None
