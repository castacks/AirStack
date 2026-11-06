"""Reusable, common-language-space voxel features and arbitrary-query scoring.

Native RayFronts PCA bases are fitted independently. Decompress before sending;
never fuse PCA coefficients from different robots. No new lossy compression is
applied. RGB/count accompany every voxel; scores are computed only after fusion.
"""
import hashlib
import os
from pathlib import Path
import numpy as np


def feature_snapshot_path(robot):
    return Path(os.getenv('RAVEN_MAP_SNAPSHOT_DIR', '/tmp')) / f'rayfronts_features_{robot}.npz'


def query_features(features, queries, probability=True, chunk_size=4096):
    features = np.asarray(features, dtype=np.float32)
    queries = np.asarray(queries, dtype=np.float32)
    if features.ndim != 2 or queries.ndim != 2 or features.shape[1] != queries.shape[1]:
        raise ValueError('incompatible reusable map / query feature dimensions')
    queries = queries / np.maximum(np.linalg.norm(queries, axis=1, keepdims=True), 1e-12)
    out = np.empty((len(features), len(queries)), dtype=np.float32)
    for start in range(0, len(features), chunk_size):
        f = features[start:start+chunk_size]
        f = f / np.maximum(np.linalg.norm(f, axis=1, keepdims=True), 1e-12)
        scores = f @ queries.T
        if probability:
            scores *= 100
            scores -= scores.max(axis=1, keepdims=True)
            np.exp(scores, out=scores)
            scores /= scores.sum(axis=1, keepdims=True)
        out[start:start+len(f)] = scores
    return out


def export_feature_snapshot(server, reference):
    """Runs in RayFronts; exports full native-map information into shared space."""
    import torch
    from omegaconf import OmegaConf
    mapper = server.mapper
    if mapper.global_vox_xyz is None or not reference:
        return
    if server.cfg.querying.compressed:
        raise ValueError('reusable map requires uncompressed query vectors in both experiment modes')
    features = []
    with torch.inference_mode():
        for f in mapper.global_vox_feat.split(4096):
            if mapper.feat_compressor is not None:
                f = mapper.feat_compressor.decompress(f)
            f = mapper.encoder.align_spatial_features_with_language(f.unsqueeze(-1).unsqueeze(-1))
            features.append(f.squeeze(-1).squeeze(-1).float().cpu().numpy())
    features = np.concatenate(features)
    labels = list(server._queries_labels['text'])
    queries = server._queries_feats['text'].float().cpu().numpy()
    # Independent CPU reconstruction must reproduce RayFronts' actual query
    # output; catches dimension, projection, label-order and softmax errors.
    n = min(64, len(features))
    actual = query_features(features[:n], queries, bool(server.cfg.querying.compute_prob))
    expected = reference['scores'][:n]
    error = float(np.max(np.abs(actual-expected)))
    if not np.isfinite(error) or error > .002:
        raise ValueError(f'reusable feature scoring differs from RayFronts: {error}')
    config = OmegaConf.to_yaml(server.cfg.encoder, resolve=True)
    space = hashlib.sha256(config.encode()).hexdigest()[:20]
    robot = os.environ['ROBOT_NAME']
    path = feature_snapshot_path(robot)
    temporary = path.with_suffix(f'.{os.getpid()}.tmp')
    with temporary.open('wb') as stream:
        np.savez(stream, xyz=mapper.global_vox_xyz.float().cpu().numpy(),
                 features=features, rgb=mapper.global_vox_rgb.float().cpu().numpy(),
                 counts=mapper.global_vox_cnt.float().cpu().numpy(),
                 query_labels=np.asarray(labels), query_features=queries,
                 feature_space=np.asarray(space), probability=np.asarray(bool(server.cfg.querying.compute_prob)),
                 reference_error=np.asarray(error))
    temporary.replace(path)
    print(f'[reusable-map] voxels={len(features)} dim={features.shape[1]} '
          f'bytes={path.stat().st_size} score_error={error:.6g}', flush=True)
