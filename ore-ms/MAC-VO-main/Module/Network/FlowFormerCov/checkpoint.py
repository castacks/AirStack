"""Strict loading with one audited migration for the released RGB checkpoint."""

import torch


# FlowFormer 39f63e6 removed these modules, unused by their forward methods.
LEGACY_UNUSED_KEYS = frozenset({
    "context_encoder.svt.norm.weight", "context_encoder.svt.norm.bias",
    "memory_encoder.feat_encoder.svt.norm.weight",
    "memory_encoder.feat_encoder.svt.norm.bias",
    "memory_decoder.att.pos_emb.rel_ind",
    "memory_decoder.att.pos_emb.rel_height.weight",
    "memory_decoder.att.pos_emb.rel_width.weight",
})
DELTA_KEY = "memory_decoder.delta"


def load_cov_checkpoint(model, weights, *, allow_legacy=True):
    """Load all active state strictly; return a JSON-compatible migration report.

    The legacy signature must match exactly. The only synthesized state is
    the fixed 9x9 sampling grid registered by upstream commit 4bcccbd.
    No missing learned parameter or unknown extra key is tolerated.
    """
    state = {}
    for key, value in weights.items():
        name = key.removeprefix("module.")
        if name in state:
            raise ValueError(f"Duplicate checkpoint key after DDP normalization: {name}")
        state[name] = value

    expected = model.state_dict()
    missing = set(expected) - set(state)
    unexpected = set(state) - set(expected)
    report = {"migration": "none", "checkpoint_entries": len(state),
              "removed_unused_keys": [], "initialized_buffers": []}
    if allow_legacy and missing == {DELTA_KEY} and unexpected == LEGACY_UNUSED_KEYS:
        axis = torch.arange(-4, 5, device=expected[DELTA_KEY].device,
                            dtype=expected[DELTA_KEY].dtype)
        delta = torch.stack(torch.meshgrid(axis, axis, indexing="ij"), dim=-1)
        delta = delta.reshape(1, 9, 9, 2)
        if delta.shape != expected[DELTA_KEY].shape:
            raise ValueError("Unsupported sampling-grid shape for legacy checkpoint")
        for key in LEGACY_UNUSED_KEYS:
            del state[key]
        state[DELTA_KEY] = delta
        report.update(migration="flowformer-pre-cleanup-rgb",
                      removed_unused_keys=sorted(LEGACY_UNUSED_KEYS),
                      initialized_buffers=[DELTA_KEY])
    elif missing or unexpected:
        raise RuntimeError(f"Checkpoint mismatch: missing={sorted(missing)}, "
                           f"unexpected={sorted(unexpected)}")

    model.load_state_dict(state, strict=True)
    return report
