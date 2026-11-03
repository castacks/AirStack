"""Opt-in earlier copied observations; native count threshold and returns unchanged."""
from .geometry_event_recorder import GeometryEventRecorder

class EarlySignalsMixin:
    def __init__(self,system,*args,**kwargs):
        threshold=getattr(system,'min_num_point',None)
        if type(threshold) is not int or threshold<=0:raise ValueError('actual native minimum point count required')
        super().__init__(system,*args,**kwargs)

    def _select(self,original,*args,**kwargs):
        result=super()._select(original,*args,**kwargs)
        def observe():
            if self.current is None:return
            keys=['time0_ns','time1_ns','selected_count','grid_count','grid_positive_uu_vv_count','grid_trace_q10_q50_q90_px2']
            self._emit('early_covariance_available',**{k:self.current[k] for k in keys},
                scope='existing copied selector grid covariance; no current pose information or accuracy claim')
        self._observe('early_covariance',observe)
        return result

    def _filter(self,original,*args,**kwargs):
        result=super()._filter(original,*args,**kwargs)
        def observe():
            if self.current is None:return
            keys=['time0_ns','time1_ns','selected_count','inbound_count','retained_count','occupied_3x3_cells']
            if type(self.current['retained_count']) is not int:raise ValueError('copied retained count unavailable')
            threshold=self.system.min_num_point
            self._emit('early_tracking_available',**{k:self.current[k] for k in keys},native_min_num_point=threshold,
                tracking_count_threshold_met=self.current['retained_count']>=threshold,
                scope='native count branch only; no optimized motion, adequacy or metric accuracy claim')
        self._observe('early_tracking',observe)
        return result

class EarlyGeometryEventRecorder(EarlySignalsMixin,GeometryEventRecorder):
    pass
