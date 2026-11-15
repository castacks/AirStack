"""Small fixed utility adapters, ordered subset policy and analytic scene visibility."""
import numpy as np


def utility_features(packet, temporal_covariance, temporal_flow, K):
    uv, z0, z1 = packet['source_uv'], packet['X0'][:, 2], packet['X1'][:, 2]
    values = np.column_stack([
        (uv[:, 0]-K[0, 2])/K[0, 0], (uv[:, 1]-K[1, 2])/K[1, 1],
        np.log(z0), np.log(z1),
        np.log(np.trace(packet['C0'], axis1=1, axis2=2)),
        np.log(np.trace(packet['C1'], axis1=1, axis2=2)),
        np.log(np.sum(temporal_covariance, axis=1)),
        np.log1p(np.linalg.norm(temporal_flow, axis=1)),
    ])
    if values.shape[1] != 8 or not np.isfinite(values).all():
        raise ValueError('INVALID_UTILITY_FEATURES')
    return values


def standardize(features, mean, scale):
    return (features-mean)/scale


def training_statistics(features):
    mean, scale = np.mean(features, axis=0), np.std(features, axis=0)
    return mean, np.where(scale == 0, 1., scale)


def ordered_log_probability_and_score(theta, features, sequence):
    theta, features = np.asarray(theta, float), np.asarray(features, float)
    sequence = np.asarray(sequence, dtype=int)
    if len(set(sequence.tolist())) != len(sequence) or np.any(sequence < 0) or np.any(sequence >= len(features)):
        raise ValueError('INVALID_SUBSET_SEQUENCE')
    remaining = np.ones(len(features), bool)
    log_probability, score = 0., np.zeros_like(theta)
    for chosen in sequence:
        ids = np.flatnonzero(remaining)
        logits = features[ids]@theta
        normalizer = np.logaddexp.reduce(logits)
        probabilities = np.exp(logits-normalizer)
        log_probability += float(features[chosen]@theta-normalizer)
        score += features[chosen]-probabilities@features[ids]
        remaining[chosen] = False
    return log_probability, score


def sample_subset(theta, features, k, seed):
    remaining = list(range(len(features)))
    rng, chosen = np.random.default_rng(seed), []
    for _ in range(min(k, len(features))):
        logits = features[remaining]@theta
        probabilities = np.exp(logits-np.logaddexp.reduce(logits))
        position = rng.choice(len(remaining), p=probabilities)
        chosen.append(remaining.pop(position))
    return np.asarray(chosen, dtype=int)


def top_subset(theta, features, k):
    # Candidate rows must already be ordered by source (v,u); stable ties retain that order.
    return np.argsort(-(features@theta), kind='stable')[:min(k, len(features))]


def residual_utility_weights(theta, features):
    logits = features@theta
    return np.exp(logits-np.logaddexp.reduce(logits))*len(features)


def bounded_pose_loss(error, successful=True):
    if not successful:
        return 1.
    error = np.asarray(error)
    risk = np.sum((error[:3]/.01)**2)+np.sum((error[3:]/np.deg2rad(.1))**2)
    return float(-np.expm1(-risk))


def ray_hit(pixels, K, rotation, center, geometry):
    pixels = np.asarray(pixels, float)
    rays = np.concatenate([pixels, np.ones(pixels.shape[:-1]+(1,))], axis=-1)@np.linalg.inv(K).T
    directions = rays@np.asarray(rotation).T
    center = np.asarray(center)
    near, far = geometry['depths_m']
    near_scale = (near-center[2])/directions[..., 2]
    candidate = center+near_scale[..., None]*directions
    normal = np.asarray(geometry['near_normal_xy'])
    near_exists = (candidate[..., :2]@normal <= geometry['near_offset_m']) & (near_scale > 0)
    depth = np.where(near_exists, near, far)
    scale = (depth-center[2])/directions[..., 2]
    world = center+scale[..., None]*directions
    valid = (directions[..., 2] > 0)&(scale > 0)&np.isfinite(world).all(axis=-1)
    return world, np.where(near_exists, 0, 1), valid


def visible_projection(world, K, rotation, center, geometry, shape=(256, 640)):
    world, center, rotation = np.asarray(world, float), np.asarray(center), np.asarray(rotation)
    local = (world-center)@rotation
    with np.errstate(divide='ignore', invalid='ignore'):
        homogeneous = local@K.T
        pixels = homogeneous[..., :2]/homogeneous[..., 2:]
        hit, surface, valid_ray = ray_hit(pixels, K, rotation, center, geometry)
    height, width = shape
    in_image = (pixels[..., 0] >= 0)&(pixels[..., 0] <= width-1)&(pixels[..., 1] >= 0)&(pixels[..., 1] <= height-1)
    same_point = np.linalg.norm(hit-world, axis=-1) <= 1e-7
    return pixels, valid_ray&(local[..., 2] > 0)&in_image&same_point
