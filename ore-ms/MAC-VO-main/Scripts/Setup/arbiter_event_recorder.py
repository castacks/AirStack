"""Opt-in copied event observations. Never poll/consume an optimizer result."""
import json
import inspect
from functools import wraps
from pathlib import Path
import time

import numpy as np


def array(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    return np.array(value, copy=True)


def scalar(value):
    a = array(value)
    if a.size != 1:
        raise ValueError('single-frame optimizer indices required')
    return int(a.reshape(-1)[0])


def safe_json(value):
    if isinstance(value, np.ndarray):
        return safe_json(value.tolist())
    if isinstance(value, dict):
        return {k: safe_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.generic):
        return value.item()
    return value


def relative_pose(parent, target):
    """Logged copy only: parent^-1 * target, vectors [tx,ty,tz,qx,qy,qz,qw]."""
    p, t = array(parent).astype(float).reshape(-1), array(target).astype(float).reshape(-1)
    if p.shape != (7,) or t.shape != (7,) or not np.isfinite([p, t]).all():
        raise ValueError('invalid pose shape/finite values')
    norms = [np.linalg.norm(a[3:]) for a in [p, t]]
    if any(abs(n-1) > 1e-5 for n in norms):
        raise ValueError('non-unit quaternion')
    # Normalize copies for the derived log value; original vectors remain unchanged.
    p[3:] /= norms[0]
    t[3:] /= norms[1]
    x, y, z, w = p[3:]
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    a, b = -p[3:6], t[3:6]
    q = np.r_[p[6]*b+t[6]*a+np.cross(a, b), p[6]*t[6]-np.dot(a, b)]
    return np.r_[rotation.T @ (t[:3]-p[:3]), q]


class ArbiterEventRecorder:
    """Single-instance observer for MACVO with synchronous or delayed writeback.

    Events are buffered as immutable JSON strings and flushed explicitly. The
    monotonic clock measures observation time, not sensor time or a hard latency
    guarantee. Observation errors are retained without changing estimator control
    flow. Attach before the first frame; supported optimizer schema is one
    from_idx/frame_idx per submission. No body-frame conversion is inferred.
    """
    def __init__(self, system, output, modality, clock=time.monotonic_ns):
        self.system, self.output, self.modality, self.clock = system, Path(output), modality, clock
        self.output.mkdir(parents=True, exist_ok=False)
        self.events, self.errors, self.originals = [], [], []
        self.phase, self.current, self.submission = 'initialization', None, 0
        self.jobs, self.writes = {}, {}
        self.closed = False
        required = [(system, 'run_pair'), (system, 'terminate'), (system.Optimizer, 'start_optimize'),
                    (system.Optimizer, 'write_graph_data'), (system.KeypointSelector, 'select_point'),
                    (system.OutlierFilter, 'filter'), (system.MapRefiner, 'elaborate_map')]
        if not all(callable(getattr(obj, name, None)) for obj, name in required):
            raise TypeError('system lacks required recorder observation points')
        self._wrap(system, 'run_pair', self._pair)
        self._wrap(system, 'terminate', self._terminate)
        self._wrap(system.Optimizer, 'start_optimize', self._submit)
        self._wrap(system.Optimizer, 'write_graph_data', self._write)
        self._wrap(system.KeypointSelector, 'select_point', self._select)
        self._wrap(system.OutlierFilter, 'filter', self._filter)
        self._wrap(system.MapRefiner, 'elaborate_map', self._refine)

    def _wrap(self, obj, name, handler):
        original = getattr(obj, name)
        # Preserve whether a method was originally inherited or instance-owned.
        owned = name in vars(obj)
        self.originals.append((obj, name, original, owned))
        signature = inspect.signature(original)
        @wraps(original)
        def wrapped(*args, **kwargs):
            if kwargs:
                bound = signature.bind(*args, **kwargs)
                bound.apply_defaults()
                args, kwargs = bound.args, bound.kwargs
            return handler(original, *args, **kwargs)
        setattr(obj, name, wrapped)

    def _observe(self, operation, callback):
        try:
            return callback()
        except Exception as error:
            self.errors.append({'operation': operation, 'error_type': type(error).__name__, 'error': str(error)})
            return None

    def _emit(self, kind, **fields):
        event = {'sequence': len(self.events), 'kind': kind, 'modality': self.modality,
                 'observation_monotonic_ns': int(self.clock()), 'stage': self.phase, **fields}
        self.events.append(json.dumps(safe_json(event), allow_nan=False, separators=(',', ':')))

    def _interval(self, graph, parent, target):
        times = graph.frames.data['time_ns']
        return {'time0_ns': scalar(times[parent]), 'time1_ns': scalar(times[target]),
                'from_idx': parent, 'frame_idx': target}

    def _pair(self, original, frame0, frame1):
        previous_phase, previous_current = self.phase, self.current
        self.phase = 'pair_processing'
        self.current = {'time0_ns': None, 'time1_ns': None,
                        'selected_count': None, 'inbound_count': None, 'retained_count': None,
                        'occupied_3x3_cells': None, 'submitted': False}
        self._observe('pair_interval', lambda: self.current.update(time0_ns=int(frame0.stereo.frame_ns), time1_ns=int(frame1.stereo.frame_ns)))
        self._observe('pair_begin', lambda: self._emit('pair_begin', interval={k: self.current[k] for k in ['time0_ns', 'time1_ns']}))
        error = None
        before = self._observe('pair_frame_count', lambda: len(array(self.system.graph.frames.data['need_interp'].tensor)))
        try:
            return original(frame0, frame1)
        except BaseException as caught:
            error = type(caught).__name__
            raise
        finally:
            def end():
                flags = array(self.system.graph.frames.data['need_interp'].tensor)
                new = before is not None and len(flags) == before+1
                self._emit('pair_end', **self.current, execution_error=error,
                           original_need_interp=bool(flags[-1]) if new else None,
                           new_frame_idx=len(flags)-1 if new else None,
                           motion_status='submitted' if self.current['submitted'] else 'not_submitted')
            self._observe('pair_end', end)
            self.phase, self.current = previous_phase, previous_current

    def _select(self, original, frame, count, depth0, depth1, match):
        result = original(frame, count, depth0, depth1, match)
        def observe():
            if self.current is None:
                return
            self.current['selected_count'] = len(result)
            self.current['height_width'] = [int(frame.height), int(frame.width)]
            covariance = array(match.cov)
            if covariance.ndim == 4 and covariance.shape[0] == 1:
                covariance = covariance[0]
            if covariance.ndim != 3 or covariance.shape[0] != 3:
                raise ValueError('expected temporal covariance shape 1x3xHxW or 3xHxW')
            margin = int(self.system.edge_width)
            grid = covariance[:, margin:frame.height-margin:16, margin:frame.width-margin:16].reshape(3, -1).T
            valid = np.isfinite(grid).all(axis=1) & (grid[:, 0] > 0) & (grid[:, 1] > 0)
            trace = grid[valid, :2].sum(axis=1)
            self.current.update(grid_count=len(grid), grid_positive_uu_vv_count=int(valid.sum()),
                grid_trace_q10_q50_q90_px2=np.quantile(trace, [.1, .5, .9]).tolist() if len(trace) else None)
        self._observe('selection', observe)
        return result

    def _filter(self, original, observations, device):
        result = original(observations, device)
        def observe():
            if self.current is None:
                return
            keep = array(result)
            if keep.dtype != np.bool_ or keep.ndim != 1:
                raise ValueError('expected boolean observation filter mask')
            points = array(observations.data['pixel1_uv'])
            if len(points) != len(keep):
                raise ValueError('observation/mask length mismatch')
            retained = points[keep]
            height, width = self.current['height_width']
            if len(retained):
                if not np.isfinite(retained).all() or not np.all((retained >= 0) & (retained < [width, height])):
                    raise ValueError('invalid retained source pixels')
                cells = np.floor(retained/[width, height]*3).astype(int)
                occupied = len(np.unique(cells[:, 1]*3+cells[:, 0]))
            else:
                occupied = 0
            self.current.update(inbound_count=len(keep), retained_count=int(keep.sum()), occupied_3x3_cells=occupied)
        self._observe('outlier_filter', observe)
        return result

    def _submit(self, original, graph_data):
        job = None
        def begin():
            parent, target = scalar(graph_data.from_idx), scalar(graph_data.frame_idx)
            self.submission += 1
            job = {'submission_id': self.submission, **self._interval(self.system.graph, parent, target)}
            self.jobs[(parent, target)] = job
            self._emit('optimization_submitted', **job)
            return job
        job = self._observe('submission', begin)
        try:
            returned = original(graph_data)
        except BaseException as error:
            self._observe('optimization_error', lambda: self._emit('optimization_error', job=job, error_type=type(error).__name__))
            raise
        def submitted():
            if self.current is not None:
                self.current['submitted'] = True
            parallel = getattr(self.system.Optimizer, 'is_parallel_mode', None)
            meaning = 'dispatch_returned' if parallel is True else 'synchronous_solve_returned' if parallel is False else 'unspecified_optimizer_return'
            self._emit('optimization_submit_returned', job=job, call_semantics=meaning)
        self._observe('submission_return', submitted)
        return returned

    def _write(self, original, result, graph):
        snapshot = None
        if result is not None:
            def before():
                parent, target = scalar(result.from_idx), scalar(result.frame_idx)
                return {'parent': parent, 'target': target, 'parent_pose': array(graph.frames.data['pose'][parent]),
                        'native_optimizer_pose': array(result.motion), 'original_need_interp': bool(scalar(graph.frames.data['need_interp'][target]))}
            snapshot = self._observe('result_before_write', before)
        try:
            returned = original(result, graph)  # The only original write call; no extra optimizer poll.
        except BaseException as error:
            self._observe('result_write_error', lambda: self._emit('motion_result_write_error', error_type=type(error).__name__))
            raise
        if snapshot is not None:
            def after():
                parent, target = snapshot['parent'], snapshot['target']
                job = self.jobs.get((parent, target))
                identity = job['submission_id'] if job else None
                self.writes[identity] = self.writes.get(identity, 0)+1
                committed = array(graph.frames.data['pose'][target])
                delta, reason = None, None
                try:
                    if parent == target:
                        raise ValueError('identical parent and target')
                    delta = relative_pose(snapshot['parent_pose'], committed)
                except ValueError as error:
                    reason = str(error)
                self._emit('motion_result_written', **self._interval(graph, parent, target),
                    submission_id=identity, write_ordinal_for_submission=self.writes[identity],
                    first_write_for_submission=self.writes[identity] == 1 and job is not None,
                    original_need_interp=snapshot['original_need_interp'],
                    parent_pose_camera_map_xyzw=snapshot['parent_pose'],
                    optimizer_pose_before_write_native_parameterization=snapshot['native_optimizer_pose'],
                    committed_pose_camera_map_xyzw=committed, derived_camera_delta_xyzw=delta,
                    derived_delta_unavailable_reason=reason, optimizer_convergence=None, pose_covariance=None)
            self._observe('result_after_write', after)
        return returned

    def _terminate(self, original):
        previous = self.phase
        self.phase = 'termination'
        try:
            return original()
        finally:
            self.phase = previous

    def _refine(self, original, frames):
        previous = self.phase
        self.phase = 'refinement'
        self._observe('refinement_begin', lambda: self._emit('refinement_begin'))
        error_type = None
        try:
            return original(frames)
        except BaseException as error:
            error_type = type(error).__name__
            raise
        finally:
            self._observe('refinement_end', lambda: self._emit('refinement_end', execution_error=error_type))
            self.phase = previous

    def finish(self):
        if not self.closed:
            for obj, name, original, owned in reversed(self.originals):
                if owned:
                    setattr(obj, name, original)
                else:
                    delattr(obj, name)
            self.closed = True
        (self.output/'events.jsonl').write_text('\n'.join(self.events)+('\n' if self.events else ''))
        summary = {'schema_version': 1, 'status': 'COMPLETE' if not self.errors else 'OBSERVER_ERRORS',
            'modality': self.modality, 'events': len(self.events), 'optimization_submissions': self.submission,
            'observer_errors': self.errors, 'clock': 'time.monotonic_ns; observer stage time, not sensor time',
            'pose_frame': 'native camera-map pose; no body transform inferred',
            'motion_convention': 'parent^-1 * committed target; normalized quaternion copies for derived delta only',
            'scope': 'before-refinement observation; no gate decisions, accuracy labels or convergence/covariance inference'}
        (self.output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        return summary
