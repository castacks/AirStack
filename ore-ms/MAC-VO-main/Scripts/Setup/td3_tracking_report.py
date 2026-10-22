"""Read-only summaries of tracking state before and after map refinement."""
import numpy as np


def tracking_report(before, after):
    before, after = np.asarray(before), np.asarray(after)
    if before.ndim != 1 or before.shape != after.shape or len(before) < 2:
        raise ValueError('Aligned one-dimensional flags for at least two frames required')
    if before.dtype != np.bool_ or after.dtype != np.bool_:
        raise ValueError('Flags must be boolean')
    unsupported = np.flatnonzero(before[1:])+1
    transitions = len(before)-1
    state = 'ALL_PAIRS_SUPPORTED' if len(unsupported) == 0 else 'NO_PAIRS_SUPPORTED' if len(unsupported) == transitions else 'SOME_PAIRS_UNSUPPORTED'
    return dict(support_state=state, output_frames=len(before), transition_pairs=transitions,
        unsupported_pairs_before_refinement=len(unsupported), unsupported_frame_indices=unsupported.tolist(),
        stored_flags_after_refinement=int(after.sum()), changed_flag_frame_indices=np.flatnonzero(before != after).tolist(),
        scope='Original need_interp flags may denote lost tracking or skipped keyframes; no physical accuracy guarantee')
