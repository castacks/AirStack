"""Reliable full-map streams, isolated from the small gossip control plane.

Peers discover addresses through range-filtered gossip. Each peer worker sends
the latest complete snapshot, waits for an application ACK, then advances to
the newest snapshot. No unbounded backlog or partial-map fusion. Binary ROS
CDR is unchanged; only an eight-byte length prefix is added on the stream.
"""
import socket
import struct
import threading
import time
import uuid


class MapStream:
    def __init__(self, receive, measure, allowed, host=None, port=17877, interval=1.0, encode=None):
        self.receive, self.measure, self.allowed = receive, measure, allowed
        self.host = host or socket.gethostbyname(socket.gethostname())
        self.interval = interval
        self.encode = encode or (lambda data, state: (data, None))
        self.session = uuid.uuid4().hex
        self._condition = threading.Condition()
        self._stopped = False
        self._snapshot = None
        self._revision = 0
        self._peers = {}
        self._last_error = {}
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(('0.0.0.0', port))
        self.port = self._server.getsockname()[1]
        self._server.listen(16)
        self._server.settimeout(.5)
        threading.Thread(target=self._listen, daemon=True).start()

    def endpoint(self):
        return {'host': self.host, 'port': self.port, 'protocol': 'raven-map-cdr-v1', 'session': self.session}

    def snapshot(self, data):
        with self._condition:
            self._snapshot = data
            self._revision += 1
            self._condition.notify_all()

    def peer(self, name, endpoint):
        if endpoint.get('protocol') != 'raven-map-cdr-v1':
            return
        address = str(endpoint['host']), int(endpoint['port']), str(endpoint['session'])
        with self._condition:
            new = name not in self._peers
            self._peers[name] = address
            if new:
                threading.Thread(target=self._send_peer, args=(name,), daemon=True).start()
            self._condition.notify_all()

    def _send_peer(self, name):
        delivered, last, base, session = -1, 0., None, None
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopped or (
                    self._snapshot is not None and (self._revision != delivered or self._peers[name][2] != session)
                    and time.monotonic() - last >= self.interval), timeout=self.interval)
                if self._stopped:
                    return
                if (self._snapshot is None or (self._revision == delivered and self._peers[name][2] == session)
                        or time.monotonic() - last < self.interval or not self.allowed(name)):
                    continue
                snapshot, revision, endpoint = self._snapshot, self._revision, self._peers[name]
            if endpoint[2] != session:
                base = None
            address = endpoint[:2]
            written, success = 0, False
            try:
                data, new_base = self.encode(snapshot, base)
                with socket.create_connection(address, timeout=20) as conn:
                    conn.settimeout(30)
                    packet = memoryview(struct.pack('!Q', len(data)) + data)
                    while written < len(packet):
                        n = conn.send(packet[written:written + 1024*1024])
                        if not n:
                            raise ConnectionError('map stream closed')
                        written += n
                    if conn.recv(1) != b'\x01':
                        raise ConnectionError('map receiver rejected snapshot')
                    delivered, success, base, session = revision, True, new_base, endpoint[2]
            except (OSError, ConnectionError, ValueError, RuntimeError) as exc:
                self._last_error[name] = str(exc)
                base = None  # retry a full snapshot after failed ACK/base mismatch
            finally:
                last = time.monotonic()
                if written:
                    self.measure('tx', name, max(0, written-8), written, success)
                if success:
                    self._last_error.pop(name, None)

    def _listen(self):
        while not self._stopped:
            try:
                conn, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._read, args=(conn,), daemon=True).start()

    def _read(self, conn):
        read, source, success = 0, 'unknown', False
        def exact(n):
            nonlocal read
            chunks = bytearray()
            while len(chunks) < n:
                part = conn.recv(min(n-len(chunks), 1024*1024))
                if not part:
                    raise ConnectionError('incomplete map stream')
                read += len(part)
                chunks.extend(part)
            return bytes(chunks)
        try:
            with conn:
                conn.settimeout(30)
                size, = struct.unpack('!Q', exact(8))
                if not 0 < size <= 1024**3:
                    raise ValueError('invalid semantic map size')
                source = self.receive(exact(size))
                if not self.allowed(source):
                    raise ValueError('map source is not a live accepted gossip peer')
                conn.sendall(b'\x01')
                success = True
        except (OSError, ValueError, RuntimeError, ConnectionError):
            pass
        finally:
            if read:
                self.measure('rx', source, max(0, read-8), read, success)

    def close(self):
        with self._condition:
            self._stopped = True
            self._condition.notify_all()
        self._server.close()
