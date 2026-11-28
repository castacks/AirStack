"""Source-derived synthetic guard reference, NOT C++/ROS equivalence proof.

Explicit inputs stand for callbacks/service observations, never commands. Native
header and member origin remain authoritative. No external reset operation.
"""
class GuardReference:
    def __init__(self,origin_ns=0,timeout_ns=10_000_000_000,config_valid=True):
        self.origin=origin_ns;self.timeout=timeout_ns;self.config_valid=config_valid
        self.attempted=not config_valid;self.initialized=False;self.unsafe=False
        self.state_at=None;self.ground_at=None;self.verified_at=None
        self.connected=False;self.armed=False;self.ground=False
        self.reason='WAITING_FOR_SAFE_STARTUP' if config_valid else 'CONFIG_REJECTED'

    def state(self,now,connected,armed=False):
        self.state_at=now;self.connected=connected;self.armed=armed
        if not self.initialized and armed:self.unsafe=True

    def extended_state(self,now,on_ground,in_air=False):
        self.ground_at=now;self.ground=on_ground
        if not self.initialized and in_air:self.unsafe=True

    def safe(self,now):
        return (not self.unsafe and self.state_at is not None and self.ground_at is not None
                and 0<=now-self.state_at<=500_000_000 and 0<=now-self.ground_at<=500_000_000
                and self.connected and not self.armed and self.ground)

    def ready(self,now):
        # Original ready() checks readback freshness, not current safe_to_configure().
        return self.initialized and self.verified_at is not None and 0<=now-self.verified_at<=500_000_000

    def tick(self,now,*,query_available=True,service_available=False,safe_after_wait=True,
             set_ok=False,typed_double_readback=False,safe_after_transaction=True,exception=False):
        if not query_available:return 'QUERY_BUSY'
        if exception:
            self.attempted=True;self.initialized=False;self.reason='EXCEPTION'
        elif self.config_valid and not self.attempted:
            if self.unsafe or now-self.origin>=self.timeout:
                self.attempted=True;self.reason='UNSAFE_BEFORE_INITIALIZATION' if self.unsafe else 'STARTUP_DEADLINE'
            elif self.safe(now):
                if not service_available:return 'WAITING_FOR_PARAMETER_SERVICE'
                if not safe_after_wait:return 'WAITING_FOR_SAFE_STARTUP'
                # The attempt is terminal even if the Set/Get transaction fails.
                self.attempted=True
                self.initialized=set_ok and typed_double_readback and safe_after_transaction
                self.verified_at=now if set_ok and typed_double_readback else None
                self.reason='INITIALIZED' if self.initialized else 'STARTUP_NOT_VERIFIED'
        elif self.initialized:
            self.verified_at=now if typed_double_readback else None
        return ('READY' if self.ready(now) else 'READBACK_UNVERIFIED') if self.initialized else self.reason

    def recovery_observed(self,phase):
        if phase not in {'requested','sealed','isolated','approved'}:raise ValueError(phase)
        return self.reason
