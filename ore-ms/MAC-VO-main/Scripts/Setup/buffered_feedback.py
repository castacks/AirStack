"""Live readiness or fixed exposed request fixture with late-source-safe ingress."""
from Scripts.Setup.transition_feedback import TransitionFeedback
from Scripts.Setup.readiness_feedback import ReadinessFeedback

class BufferedReadinessFeedback(ReadinessFeedback):
    def ingest_group(self,camera,event,receipt,invocation_source,delivery_group):
        if invocation_source is not None and delivery_group is not None and delivery_group!=invocation_source:
            session=self.live[camera]
            if session is not None and session['status']=='pending':
                raise ValueError('catch-up must not bypass an inactive handshake deadline')
            return TransitionFeedback.ingest(self,camera,event,receipt,invocation_source)
        return self.ingest(camera,event,receipt,invocation_source)


class FrozenRequestFeedback(TransitionFeedback):
    def __init__(self,rows,fixture):
        super().__init__(rows,'bounded_probe');self.fixture=fixture
    def plan(self,pair,now):
        self.cache._clock(now)
        if pair<=self.cache.last_pair:raise ValueError('unique fixture source decisions required')
        self.cache.last_pair=pair;row=self.fixture[pair]
        if row['pair']!=pair:raise ValueError('invalid frozen fixture order')
        for c in row['requests']:
            self.cache.requested[c].add(pair);self.cache.last_request[c]=now
        self.cache.active=None if row['action']=='ABSTAIN' else row['action'].lower()
        return {'pair':pair,'decision_ns':now,'action':row['action'],'requests':list(row['requests']),
                'views':{c:self.cache.view(c,now) for c in ['rgb','thermal']},'frozen_request_fixture':True}
    def ingest_group(self,camera,event,receipt,invocation_source,delivery_group):
        return self.ingest(camera,event,receipt,invocation_source)
