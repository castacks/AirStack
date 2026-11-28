"""Synthetic reference of deadline/latch ordering ONLY, never a runtime guard.

Does not model ROS scheduling, service timeouts, freshness, unsafe-state latches,
Set/Get/readback or command admission. C++ upstream remains authoritative.
"""
class DeadlineReference:
    def __init__(self, started_ns=0, timeout_ns=10_000_000_000):
        self.started_ns=started_ns;self.timeout_ns=timeout_ns
        self.attempted=False;self.initialized=False;self.reason='WAITING'

    def tick(self,now_ns,connected=False,transaction_ok=False):
        if not self.attempted:
            if now_ns-self.started_ns>=self.timeout_ns:
                self.attempted=True;self.reason='STARTUP_DEADLINE'
            elif connected and transaction_ok:
                # Synthetic successful transaction stands for separately evidenced safety/Get/Set.
                self.attempted=True;self.initialized=True;self.reason='INITIALIZED'
        return self.reason

    def recovery_event(self,kind):
        # Recovery documentation cannot mutate the same guard's terminal latch.
        if kind not in {'requested','sealed','isolated','approved'}:raise ValueError(kind)
        return self.reason
