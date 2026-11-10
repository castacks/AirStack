"""Lossless voxel updates relative to each recipient's acknowledged map.

No thresholds or target filtering: compare complete float32 vectors exactly.
Missing/pruned cells are retained by the receiver's accumulated map. A new
receiver session gets a full snapshot; failures reset the sender's base.
"""
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from coordination_msgs.msg import SemanticVoxelMap


def encode_delta(data, previous):
    msg = deserialize_message(data, SemanticVoxelMap)
    fields = ('x', 'y', 'z') + tuple(f'sim_{i}' for i in range(len(msg.labels)))
    matrix = point_cloud2.read_points_numpy(msg.cloud, field_names=fields, skip_nans=True)
    keys = np.floor(matrix[:, :3].astype(np.float64) / msg.voxel_size).astype('<i8')
    unique, reverse = np.unique(keys[::-1], axis=0, return_index=True)
    matrix = np.ascontiguousarray(matrix[len(keys)-1-reverse], dtype='<f4')
    key_dtype = np.dtype([('x', '<i8'), ('y', '<i8'), ('z', '<i8')])
    keys = np.ascontiguousarray(unique).view(key_dtype).reshape(-1)
    revision = msg.cloud.header.stamp.sec * 1_000_000_000 + msg.cloud.header.stamp.nanosec
    schema = (msg.representation, msg.feature_space, msg.basis_id,
              msg.basis_rows, msg.basis_cols, msg.decoded_dimension)
    state = (keys, matrix, tuple(msg.labels), float(msg.voxel_size), revision, schema)
    changed = np.ones(len(keys), dtype=bool)
    if previous is not None and previous[2:4] == state[2:4] and previous[5:] == state[5:]:
        old_keys, old_matrix = previous[:2]
        idx = np.searchsorted(old_keys, keys)
        valid = np.flatnonzero(idx < len(old_keys))
        valid = valid[old_keys[idx[valid]] == keys[valid]]
        changed[valid] = np.any(matrix[valid] != old_matrix[idx[valid]], axis=1)
        msg.is_delta = True
        msg.base_revision = previous[4]
        if msg.representation in ('native_pca_v1', 'wire_pca_v1'):
            msg.pca_basis = []  # cached by this recipient after the ACKed full map
            msg.pca_mean = []
    else:
        msg.is_delta = False
        msg.base_revision = 0
    matrix = np.ascontiguousarray(matrix[changed], dtype='<f4')
    msg.cloud = PointCloud2(header=msg.cloud.header, height=1, width=len(matrix),
        fields=msg.cloud.fields, is_bigendian=False, point_step=4*len(fields),
        row_step=4*len(fields)*len(matrix), data=matrix.tobytes(), is_dense=True)
    return serialize_message(msg), state
