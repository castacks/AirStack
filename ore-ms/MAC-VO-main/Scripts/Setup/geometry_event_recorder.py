"""Explicit opt-in geometry observation; existing recorder is unchanged."""
from types import SimpleNamespace
from .arbiter_event_recorder import ArbiterEventRecorder, scalar
from .native_pose_information import from_graph_data, copied


class GeometryEventRecorder(ArbiterEventRecorder):
    def __init__(self, system, *args, **kwargs):
        config=getattr(system.Optimizer,'config',None)
        if getattr(config,'graph_type',None)!='disp':
            raise ValueError('geometry observation requires verified disp optimizer config')
        self.geometry_inputs={};self.geometry_emitted=set()
        super().__init__(system,*args,**kwargs)

    def _submit(self, original, graph_data):
        def snapshot():
            parent,target=scalar(graph_data.from_idx),scalar(graph_data.frame_idx)
            # Copy only the fixed-factor inputs needed for returned-pose geometry.
            self.geometry_inputs[parent,target]=SimpleNamespace(
                points=SimpleNamespace(data={'pos_Tw':copied(graph_data.points.data['pos_Tw'])}),
                observations=SimpleNamespace(data={k:copied(graph_data.observations.data[k]) for k in ['pixel2_uv_cov','pixel2_disp_cov']}),
                images_intrinsic=copied(graph_data.images_intrinsic),baseline=copied(graph_data.baseline),
                edges_index=copied(graph_data.edges_index),init_motion=copied(graph_data.init_motion))
        self._observe('copy_disparity_inputs',snapshot)
        return super()._submit(original,graph_data)

    def _write(self, original, result, graph):
        if result is not None:
            def observe():
                parent,target=scalar(result.from_idx),scalar(result.frame_idx)
                key=(parent,target);job=self.jobs.get(key)
                if job is None or key in self.geometry_emitted:return
                data=self.geometry_inputs[key];data.init_motion=copied(result.motion)
                self._emit('native_disparity_information',**self._interval(graph,parent,target),
                    submission_id=job['submission_id'],
                    linearization='returned optimizer pose before first write; copied submitted fixed factors',
                    information=from_graph_data(data))
                self.geometry_emitted.add(key);del self.geometry_inputs[key]
            self._observe('native_disparity_information',observe)
        return super()._write(original,result,graph)
