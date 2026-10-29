"""Instance-only synchronous solver adapter for a declared bounded VO experiment."""
import time
import numpy as np
import torch
from Scripts.Setup.weighted_huber_research import solve

FATAL = {'INVALID_INPUT', 'LINEARIZATION_ERROR', 'LINEAR_SOLVER_ERROR', 'REJECTION_LIMIT'}


def copied_pose(result):
    value = result.motion.detach().cpu().numpy().copy()
    if value.shape != (1, 7) or not np.isfinite(value).all() or not np.allclose(np.linalg.norm(value[:, 3:], axis=1), 1, atol=1e-5):
        raise ValueError('finite single-frame unit-quaternion motion required')
    return value[0].tolist()


def install(optimizer, graph, arm, records):
    if arm not in ('passthrough', 'repair'): raise ValueError('explicit bounded solver arm required')
    cfg = vars(optimizer.config)
    expected = {'device': 'cpu', 'vectorize': True, 'parallel': False, 'graph_type': 'disp', 'autodiff': False}
    if optimizer.__class__.__name__ != 'TwoFrame_PGO' or optimizer.is_parallel_mode or cfg != expected:
        raise ValueError('only synchronous original global analytic disp optimizer supported')
    owned = '_optimize' in vars(optimizer); original = optimizer._optimize
    def wrapped(context, data):
        if data.frame_idx.numel() != 1 or data.from_idx.numel() != 1: raise ValueError('one-frame optimization required')
        parent, target = int(data.from_idx.item()), int(data.frame_idx.item())
        times = graph.frames.data['time_ns']
        record = {'ordinal': len(records), 'arm': arm, 'from_idx': parent, 'frame_idx': target,
            'time0_ns': int(times[parent].item()), 'time1_ns': int(times[target].item()),
            'started_monotonic_ns': time.monotonic_ns(), 'stage': 'synchronous_optimize_call',
            'status': 'RUNNING', 'pose_xyzw': None, 'solver': None}
        records.append(record)
        try:
            if arm == 'passthrough':
                returned_context, result = original(context, data)
                record['status'] = 'ORIGINAL_RETURNED_UNQUALIFIED'
            else:
                model = context['pose_graph_class'](data).to(device='cpu', dtype=torch.float64)
                solution = solve(model, model.covariance_array().detach().clone())
                record['solver'] = solution; record['status'] = solution['termination']
                if solution['termination'] in FATAL: raise RuntimeError('research solver fatal status: '+solution['termination'])
                result = model.write_back(); returned_context = context
            try: record['pose_xyzw'] = copied_pose(result)
            except ValueError:
                record['status'] = 'INVALID_RESULT'
                raise
            record['numerically_unresolved'] = record['status'] in ('MAX_ITERATIONS', 'NO_PROGRESS_SMALL_STEP', 'ORIGINAL_RETURNED_UNQUALIFIED')
            return returned_context, result
        except BaseException as error:
            record['error_type'] = type(error).__name__; record['error'] = str(error)
            if record['status'] == 'RUNNING': record['status'] = 'ADAPTER_ERROR'
            raise
        finally:
            record['finished_monotonic_ns'] = time.monotonic_ns()
            record['elapsed_ns'] = record['finished_monotonic_ns']-record['started_monotonic_ns']
    optimizer._optimize = wrapped
    def restore():
        if owned: optimizer._optimize = original
        else: delattr(optimizer, '_optimize')
    return restore
