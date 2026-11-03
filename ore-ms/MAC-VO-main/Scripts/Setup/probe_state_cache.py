"""Causal requested-observation cache. No native inference or future truth access."""
CAMERAS=('rgb','thermal')
MODES=('full_read_cache','bounded_probe','bounded_no_probe','unbounded_hold','fixed_rgb','fixed_thermal')
SECOND=1_000_000_000
class ProbeCache:
    def __init__(self,mode):
        if mode not in MODES:raise ValueError('unknown fixed mode')
        self.mode=mode;self.ttl=None if mode=='unbounded_hold' else SECOND
        self.requested={c:set() for c in CAMERAS};self.health={c:{} for c in CAMERAS};self.motion={c:{} for c in CAMERAS};self.resolved={c:{} for c in CAMERAS};self.last_request={c:None for c in CAMERAS};self.burst={c:0 for c in CAMERAS};self.active=None;self.last_clock=-1;self.last_pair=-1
    def _clock(self,now):
        if type(now) is not int or now<self.last_clock:raise ValueError('monotonic exact controller clock required')
        self.last_clock=now
    def view(self,camera,now):
        if type(now) is not int or now<self.last_clock:raise ValueError('view predates cache clock')
        values=self.resolved[camera]
        if not values:return {'status':'UNKNOWN','value':None,'last_value':None,'pair':None,'receipt_ns':None,'age_ns':None}
        pair=max(values);value,receipt=values[pair];age=now-receipt
        if age<0:raise ValueError('future cached receipt')
        stale=self.ttl is not None and age>=self.ttl
        return {'status':'STALE' if stale else 'READY' if value else 'FAILED','value':None if stale else value,'last_value':value,'pair':pair,'receipt_ns':receipt,'age_ns':age}
    def plan(self,pair,now,force_both=False):
        self._clock(now)
        if type(pair) is not int or pair<=self.last_pair or any(pair in self.requested[c] for c in CAMERAS):raise ValueError('unique nonnegative pair decision required')
        self.last_pair=pair
        views={c:self.view(c,now) for c in CAMERAS};good=[c for c in CAMERAS if views[c]['value'] is True]
        if self.mode.startswith('fixed_'):chosen=self.mode.removeprefix('fixed_');reason='fixed_camera'
        else:
            chosen=self.active if self.active in good else good[0] if good else None;reason='keep_fresh_active' if chosen==self.active and chosen is not None else 'switch_fresh_available' if chosen else 'no_fresh_available'
        self.active=chosen
        if force_both:requests=set(CAMERAS);reason='common_bootstrap_or_guard'
        elif self.mode=='full_read_cache':requests=set(CAMERAS);reason='full_read_control'
        elif chosen is None:requests=set(CAMERAS);reason='refresh_unknown_stale_or_failed'
        else:
            requests={chosen}
            if self.mode=='bounded_probe':
                other=next(c for c in CAMERAS if c!=chosen);last=self.last_request[other]
                due=views[other]['status'] in ['UNKNOWN','STALE'] or last is None or now-last>=SECOND
                if due and self.burst[other]==0:self.burst[other]=2
                if self.burst[other]>0:requests.add(other)
        before=dict(self.burst)
        for c in requests:
            self.requested[c].add(pair);self.last_request[c]=now;self.burst[c]=max(0,self.burst[c]-1)
        return {'pair':pair,'decision_ns':now,'action':chosen.upper() if chosen else 'ABSTAIN','requests':[c for c in CAMERAS if c in requests],'reason':reason,'views':views,'burst_before':before,'burst_after':dict(self.burst)}
    def health_event(self,camera,pair,value,now):
        self._clock(now)
        if pair not in self.requested[camera]:return False
        if type(value) is not bool:raise ValueError('explicit native health boolean required')
        if pair in self.health[camera]:raise ValueError('duplicate health interval')
        self.health[camera][pair]=(value,now);self._resolve(camera,pair);return True
    def motion_event(self,camera,pair,valid,now):
        self._clock(now)
        if pair not in self.requested[camera]:return False
        if type(valid) is not bool:raise ValueError('explicit motion validity required')
        if pair in self.motion[camera]:raise ValueError('duplicate first write')
        self.motion[camera][pair]=(valid,now);self._resolve(camera,pair);return True
    def _resolve(self,camera,pair):
        h=self.health[camera].get(pair);m=self.motion[camera].get(pair)
        if h is None:return
        if not h[0]:self.resolved[camera][pair]=(False,h[1])
        elif m is not None:self.resolved[camera][pair]=(m[0],max(h[1],m[1]))
