"""Reusable, common-language-space voxel features and arbitrary-query scoring.

Native RayFronts PCA bases are fitted independently. Expanded transport decodes
before sending; native transport sends the basis and decodes at the receiver.
Never fuse PCA coefficients from different robots. Wire-only mode instead
introduces explicitly lossy centered PCA without changing the local mapper. RGB/count accompany every voxel; scores are computed only after fusion.
"""
import hashlib
import os
from pathlib import Path
import numpy as np


def feature_snapshot_path(robot):
    return Path(os.getenv('RAVEN_MAP_SNAPSHOT_DIR', '/tmp')) / f'rayfronts_features_{robot}.npz'


def torch_query_features(queries, features, softmax=False, chunk_size=4096):
    """RayFronts-compatible M x N scores without broadcasted M x N x D work.

    Keep normalization and softmax in float32, with softmax across queries for
    each voxel. Chunking only the voxel axis preserves that distribution.
    """
    import torch
    if (queries.ndim != 2 or features.ndim != 2
            or queries.shape[1] != features.shape[1]):
        raise ValueError('incompatible reusable map / query feature dimensions')
    if chunk_size <= 0:
        raise ValueError('chunk_size must be positive')
    q = queries.float()
    q = q / q.norm(dim=-1, keepdim=True)
    out = torch.empty((len(features), len(q)), device=features.device, dtype=torch.float32)
    for start in range(0, len(features), chunk_size):
        f = features[start:start+chunk_size].float()
        f = f / f.norm(dim=-1, keepdim=True)
        scores = f @ q.T
        if softmax:
            scores = torch.softmax(100 * scores, dim=-1)
        out[start:start+len(f)] = scores
    return out


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
    native = {}
    if os.getenv('RAVEN_WIRE_PCA_MAP', 'false').lower() == 'true':
        from raven_nav.wire_pca import WirePCA, project_wire, wire_fingerprint
        from raven_nav.native_map import start_native_ingress
        if mapper.feat_compressor is not None:
            raise ValueError('wire-only experiment requires uncompressed local mapper')
        if not hasattr(server, '_wire_pca'):
            server._wire_pca = WirePCA(100)
        codec = server._wire_pca
        coefficients = codec.encode(features, mapper.device)
        if coefficients is None:
            return  # insufficient initial voxels for a fixed 100-D fit
        restored = project_wire(server, coefficients[:n], codec.basis, codec.mean)
        # Distortion is measured, not rejected: PCA sharing is deliberately lossy.
        distortion = float(np.max(np.abs(query_features(restored, queries,
            bool(server.cfg.querying.compute_prob)) - expected)))
        rmse = float(np.sqrt(np.mean((restored-features[:n])**2)))
        if not np.isfinite(distortion) or not np.isfinite(rmse):
            raise ValueError('nonfinite wire PCA reconstruction')
        native = dict(native_coefficients=coefficients, native_basis=codec.basis,
            native_basis_id=np.asarray(wire_fingerprint(codec.basis,codec.mean)),
            native_reference_error=np.asarray(distortion), wire_pca_mean=codec.mean,
            wire_pca_rmse=np.asarray(rmse), wire_pca_fit_voxels=np.asarray(codec.fit_voxels),
            local_mapper_pca=np.asarray(False))
        start_native_ingress(server, space, features.shape[1])
    elif os.getenv('RAVEN_NATIVE_COMPRESSED_MAP', 'false').lower() == 'true':
        from raven_nav.native_map import basis_fingerprint, project_native, start_native_ingress
        if mapper.feat_compressor is None or getattr(mapper.feat_compressor, 'basis', None) is None:
            raise ValueError('native map mode requires the mapper PCA basis')
        coefficients = mapper.global_vox_feat.float().cpu().numpy()
        basis = mapper.feat_compressor.basis.float().cpu().numpy()
        restored = project_native(server, coefficients[:n], basis)
        native_error = float(np.max(np.abs(query_features(restored, queries,
            bool(server.cfg.querying.compute_prob)) - expected)))
        if not np.isfinite(native_error) or native_error > .002:
            raise ValueError(f'native feature decoding differs from RayFronts: {native_error}')
        native = dict(native_coefficients=coefficients, native_basis=basis,
                      native_basis_id=np.asarray(basis_fingerprint(basis)),
                      native_reference_error=np.asarray(native_error))
        start_native_ingress(server, space, features.shape[1])
    temporary = path.with_suffix(f'.{os.getpid()}.tmp')
    with temporary.open('wb') as stream:
        np.savez(stream, xyz=mapper.global_vox_xyz.float().cpu().numpy(),
                 features=features, rgb=mapper.global_vox_rgb.float().cpu().numpy(),
                 counts=mapper.global_vox_cnt.float().cpu().numpy(),
                 query_labels=np.asarray(labels), query_features=queries,
                 feature_space=np.asarray(space), probability=np.asarray(bool(server.cfg.querying.compute_prob)),
                 reference_error=np.asarray(error), **native)
    temporary.replace(path)
    print(f'[reusable-map] voxels={len(features)} dim={features.shape[1]} '
          f'bytes={path.stat().st_size} score_error={error:.6g}', flush=True)


def install_empty_occupancy_guard(mapper_class):
    """An empty depth update has no active frontier bbox, rather than crashing.

    RayFronts accum_occ_voxels returns None for an empty pending batch, while
    process_posed_rgbd unconditionally reads its .shape. Preserve nonempty
    results exactly and let the existing zero-row check skip frontier updates.
    """
    original = mapper_class.accum_occ_voxels
    def accum_occ_voxels(self):
        result = original(self)
        if result is None:
            import torch
            return torch.empty((0, 3), device=self.device, dtype=torch.float32)
        return result
    mapper_class.accum_occ_voxels = accum_occ_voxels
