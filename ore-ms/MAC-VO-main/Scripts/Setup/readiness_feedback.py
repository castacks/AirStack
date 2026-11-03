"""Budgeted inactive request continuation; selection and native solver unchanged."""
import copy
from Scripts.Setup.transition_feedback import TransitionFeedback
from Scripts.Setup.probe_state_cache import CAMERAS

class ReadinessFeedback(TransitionFeedback):
    ATTEMPT_LIMIT=4
    EXTRA_LIMIT=20
    def __init__(self,rows,mode):
        if mode!='bounded_probe':raise ValueError('readiness overlay is bounded-probe only')
        super().__init__(rows,mode)
        self.live={c:None for c in CAMERAS};self.session_history=[];self.extra_used=0

    def close(self,session,status,source):
        session.update(status=status,closed_source=source)

    def plan(self,pair,now):
        base=super().plan(pair,now);out=copy.deepcopy(base);source=pair+1
        chosen=base['action'].lower() if base['action']!='ABSTAIN' else None
        common=base['reason'] in ['common_bootstrap_or_guard','refresh_unknown_stale_or_failed']
        for c in CAMERAS:
            s=self.live[c]
            if s is None:continue
            if s['status']=='pending':
                if c==chosen:self.close(s,'became_active',source)
                elif common:self.close(s,'shared_refresh',source)
                elif s['attempts']>=self.ATTEMPT_LIMIT:self.close(s,'attempt_cap',source)
                elif self.extra_used>=self.EXTRA_LIMIT and c not in base['requests']:self.close(s,'extra_budget',source)
            if s['status']!='pending':self.live[c]=None
        for c in CAMERAS:
            if not common and c!=chosen and c in base['requests'] and base['burst_before'][c]==2 and self.live[c] is None:
                s={'id':len(self.session_history),'camera':c,'start_source':source,'start_ns':now,
                   'attempts':0,'requested_sources':[],'extra_sources':[],'status':'pending','evidence':None}
                self.live[c]=s;self.session_history.append(s)
        extras=[]
        for c in CAMERAS:
            s=self.live[c]
            if s is not None and s['status']=='pending' and c not in base['requests'] and s['attempts']<self.ATTEMPT_LIMIT and self.extra_used<self.EXTRA_LIMIT:
                self.cache.requested[c].add(pair);self.cache.last_request[c]=now
                extras.append(c);self.extra_used+=1;s['extra_sources'].append(source)
        requests=[c for c in CAMERAS if c in base['requests'] or c in extras]
        for c in CAMERAS:
            s=self.live[c]
            if s is not None and s['status']=='pending' and c in requests:
                s['attempts']+=1;s['requested_sources'].append(source)
                assert s['attempts']<=self.ATTEMPT_LIMIT
        out['requests']=requests;out['base_decision']=base
        out['readiness_overlay']={'extra_requests':extras,'extra_used':self.extra_used,
           'attempt_limit':self.ATTEMPT_LIMIT,'extra_limit':self.EXTRA_LIMIT,
           'sessions':copy.deepcopy(self.live)}
        return out

    def ingest(self,camera,event,receipt,trigger_source):
        result=super().ingest(camera,event,receipt,trigger_source);s=self.live[camera]
        if s is not None and s['status']=='pending' and result.get('accepted') and result.get('event_kind')=='motion_result_written' and result.get('valid'):
            b=result['binding'];h=self.cache.health[camera].get(b['pair'])
            if h and h[0] and b['source_end']>=s['start_source'] and b['source_start']+1==b['source_end'] and trigger_source==b['source_end']+1:
                s['evidence']={'binding':copy.deepcopy(b),'trigger_source':trigger_source,'receipt_ns':receipt,'event_sequence':event['sequence']}
                self.close(s,'satisfied',trigger_source)
        return result

    def delivery_failure(self,camera,source,receipt):
        result=super().delivery_failure(camera,source,receipt);s=self.live[camera]
        if s is not None and s['status']=='pending':self.close(s,'transport_failed',source)
        return result
