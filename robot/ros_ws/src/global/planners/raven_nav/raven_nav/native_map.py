"""Decode each sender's native PCA map before common-space feature fusion.

The basis is source-specific; never apply the receiver's PCA basis. Projection
runs in the existing RayFronts process, sharing its already-loaded encoder/GPU.
Only the smaller native CDR travels across the network. Local handoff/ACK stay
atomic and ACK occurs only after Raven fuses the decoded map.
"""
import hashlib
import threading
import time
import traceback
import numpy as np


def basis_fingerprint(basis):
    basis = np.ascontiguousarray(basis, dtype='<f4')
    return hashlib.sha256(str(basis.shape).encode() + basis.tobytes()).hexdigest()


def canonical_channels(space, dimension):
    return ['rgb_r', 'rgb_g', 'rgb_b', 'observation_count'] + [
        f'feature:{space}:{i}' for i in range(dimension)]


def project_native(server, coefficients, basis):
    import torch
    basis = np.asarray(basis, dtype=np.float32)
    coefficients = np.asarray(coefficients, dtype=np.float32)
    if coefficients.ndim != 2 or basis.ndim != 2 or coefficients.shape[1] != basis.shape[1]:
        raise ValueError('native coefficients / basis dimensions do not match')
    if not len(coefficients):
        return np.empty((0, server._queries_feats['text'].shape[1]), dtype=np.float32)
    device = server.mapper.device
    pieces = []
    with torch.inference_mode():
        b = torch.from_numpy(basis.copy()).to(device)
        for start in range(0, len(coefficients), 4096):
            c = torch.from_numpy(coefficients[start:start+4096].copy()).to(device)
            # RayFronts PcaCompressor.decompress is Y @ basis.T (no mean).
            reconstructed = c @ b.T
            aligned = server.mapper.encoder.align_spatial_features_with_language(
                reconstructed.unsqueeze(-1).unsqueeze(-1))
            pieces.append(aligned.squeeze(-1).squeeze(-1).float().cpu().numpy())
    return np.concatenate(pieces)


class NativeFeatureDecoder:
    def __init__(self, feature_space, dimension, project):
        self.space, self.dimension, self.project = feature_space, dimension, project
        self.bases = {}

    def decode(self, msg):
        from coordination_msgs.msg import SemanticVoxelMap
        from sensor_msgs.msg import PointCloud2, PointField
        from sensor_msgs_py import point_cloud2
        if (msg.representation != 'native_pca_v1' or msg.feature_space != self.space
                or msg.decoded_dimension != self.dimension or msg.basis_cols == 0):
            raise ValueError('incompatible native feature representation')
        basis = None
        if len(msg.pca_basis):
            if len(msg.pca_basis) != msg.basis_rows * msg.basis_cols:
                raise ValueError('incomplete source PCA basis')
            basis = np.asarray(msg.pca_basis, dtype=np.float32).reshape(msg.basis_rows, msg.basis_cols)
            if basis_fingerprint(basis) != msg.basis_id:
                raise ValueError('source PCA basis fingerprint mismatch')
        elif msg.source_robot in self.bases and self.bases[msg.source_robot][0] == msg.basis_id:
            basis = self.bases[msg.source_robot][1]
        if basis is None or (not msg.is_delta and not len(msg.pca_basis)):
            raise ValueError('missing source PCA basis; full resync required')
        expected = ['rgb_r', 'rgb_g', 'rgb_b', 'observation_count'] + [
            f'pca:{self.space}:{i}' for i in range(msg.basis_cols)]
        if list(msg.labels) != expected:
            raise ValueError('native coefficient channel schema mismatch')
        names = ['x','y','z'] + [f'sim_{i}' for i in range(len(msg.labels))]
        values = point_cloud2.read_points_numpy(msg.cloud, field_names=names, skip_nans=True)
        features = self.project(values[:,7:], basis)
        if features.shape != (len(values), self.dimension) or not np.isfinite(features).all():
            raise ValueError('decoded native features have wrong shape or nonfinite values')
        matrix = np.ascontiguousarray(np.c_[values[:,:7], features], dtype='<f4')
        labels = canonical_channels(self.space, self.dimension)
        fields = ['x','y','z'] + [f'sim_{i}' for i in range(len(labels))]
        out = SemanticVoxelMap(source_robot=msg.source_robot, is_delta=msg.is_delta,
            base_revision=msg.base_revision, labels=labels, voxel_size=msg.voxel_size,
            cloud=PointCloud2(header=msg.cloud.header,height=1,width=len(matrix),
                fields=[PointField(name=f,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,f in enumerate(fields)],
                point_step=4*len(fields),row_step=4*len(fields)*len(matrix),
                data=matrix.tobytes(),is_bigendian=False,is_dense=True))
        self.bases[msg.source_robot] = (msg.basis_id, basis)
        return out


def start_native_ingress(server, space, dimension):
    import os
    from rclpy.serialization import deserialize_message, serialize_message
    from coordination_msgs.msg import SemanticVoxelMap
    from coordination_bringup.map_snapshot import native_received_snapshots, write_received_snapshot
    if getattr(server, '_native_ingress_thread', None) is not None:
        return
    robot = os.environ['ROBOT_NAME']
    decoder = NativeFeatureDecoder(space, dimension, lambda c,b: project_native(server,c,b))
    previous = {}
    def run():
        while server.status.name not in ('CLOSING','CLOSED'):
            for source, data, version in native_received_snapshots(robot, previous):
                try:
                    msg = deserialize_message(data, SemanticVoxelMap)
                    decoded = decoder.decode(msg)
                    write_received_snapshot(robot, source, serialize_message(decoded))
                except Exception:
                    # No fusion ACK: sender retries a full map including basis.
                    print('[native-map] decode failed: ' + traceback.format_exc(), flush=True)
                previous[source] = version
            time.sleep(.1)
    server._native_ingress_thread = threading.Thread(target=run, daemon=True)
    server._native_ingress_thread.start()
