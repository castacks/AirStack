"""Requested native feedback with explicit source-gap bindings; no model/labels."""
from Scripts.Setup.probe_state_cache import ProbeCache,CAMERAS


def execute_request(requested, source, decode, run):
    if type(requested) is not bool:
        raise TypeError('explicit request boolean required')
    if not requested:
        return False
    run(decode(source))
    return True


class SparseFeedback:
    def __init__(self, rows, mode):
        self.rows=rows
        self.cache=ProbeCache(mode)
        self.endmaps={c:{r[c+'_left']['time_ns']:i for i,r in enumerate(rows)} for c in CAMERAS}
        if any(len(m)!=len(rows) for m in self.endmaps.values()):
            raise ValueError('unique source timestamps required')
        self.bindings={c:{} for c in CAMERAS}
        self.last_source={c:0 for c in CAMERAS}

    def plan(self, pair, now):
        return self.cache.plan(pair,now,force_both=pair<2 or pair>=len(self.rows)-3)

    def declare(self,camera,source):
        pair=source-1
        if pair not in self.cache.requested[camera] or source<=self.last_source[camera]:
            raise ValueError('declare only a newly requested source frame')
        start=self.last_source[camera]
        bound={'pair':pair,'source_start':start,'source_end':source,
               'sensor_interval_ns':[self.rows[i][camera+'_left']['time_ns'] for i in (start,source)]}
        self.bindings[camera][pair]=bound
        self.last_source[camera]=source
        return bound

    def ingest(self,camera,event,receipt,trigger_source):
        kind=event['kind']
        result={'accepted':False,'reason':'non_health_motion_event'}
        if kind not in ['pair_end','motion_result_written']:
            return result
        if event['stage']!='pair_processing':
            return dict(result,reason='non_processing_stage')
        if kind=='motion_result_written' and not event['first_write_for_submission']:
            return dict(result,reason='not_first_write')
        end=self.endmaps[camera].get(event['time1_ns'])
        if end is None or end==0:
            return dict(result,reason='unknown_source_end')
        pair=end-1;bound=self.bindings[camera].get(pair)
        if bound is None:
            return dict(result,reason='unrequested_origin')
        if [event['time0_ns'],event['time1_ns']]!=bound['sensor_interval_ns']:
            raise ValueError('native interval differs from declared processed endpoints')
        if trigger_source is None or trigger_source-1 not in self.bindings[camera]:
            return dict(result,reason='unrequested_delivery')
        if kind=='pair_end':
            if trigger_source!=end:
                raise ValueError('health must come from current processing interval')
            valid=event.get('execution_error') is None and event['original_need_interp'] is False
            accepted=self.cache.health_event(camera,pair,valid,receipt)
        else:
            valid=event['original_need_interp'] is False and event.get('derived_camera_delta_xyzw') is not None
            accepted=self.cache.motion_event(camera,pair,valid,receipt)
        return {'accepted':accepted,'reason':'requested_exact_native_interval',
                'valid':valid,'event_kind':kind,'binding':bound,'trigger_source':trigger_source}
