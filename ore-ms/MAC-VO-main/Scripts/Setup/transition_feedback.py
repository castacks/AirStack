"""Synthetic requested input failures; native health and endpoints remain explicit."""
from Scripts.Setup.sparse_feedback import SparseFeedback

OUTAGES=(
    {'name':'rgb_outage','cameras':['rgb'],'start_source':20,'end_source':34},
    {'name':'thermal_outage','cameras':['thermal'],'start_source':50,'end_source':64},
    {'name':'joint_outage','cameras':['rgb','thermal'],'start_source':75,'end_source':79},
)


def input_available(camera,source,perturb):
    if camera not in ['rgb','thermal'] or type(source) is not int or source<0:
        raise ValueError('known camera and nonnegative source required')
    return not perturb or not any(camera in o['cameras'] and o['start_source']<=source<=o['end_source'] for o in OUTAGES)


def requested_delivery(camera,source,requested,perturb,availability=input_available):
    if type(requested) is not bool:
        raise TypeError('explicit request boolean required')
    if not requested:
        return {'delivered':False,'availability_observed':False}
    return {'delivered':availability(camera,source,perturb),'availability_observed':True}


class TransitionFeedback(SparseFeedback):
    def delivery_failure(self,camera,source,receipt):
        pair=source-1
        if source==0 or pair not in self.cache.requested[camera] or pair in self.bindings[camera]:
            raise ValueError('only an undelivered requested frame may report transport failure')
        previous=self.last_source[camera]
        self.cache.health_event(camera,pair,False,receipt)
        assert self.last_source[camera]==previous
        return {'camera':camera,'source':source,'pair':pair,'native_previous_source':previous,
                'synthetic':True,'origin':'input_transport_unavailable','native_health_fabricated':False}
