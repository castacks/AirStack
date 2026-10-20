import numpy as np
import jax
import jax.numpy as jnp
import flax.linen as nn
from flax.linen.initializers import constant, orthogonal


def reflect_policy_inputs(description, joint_state, general_state, reflection):
    description_sign, general_sign, general_permutation = reflection
    general = jnp.take_along_axis(general_state, jnp.broadcast_to(general_permutation, general_state.shape), axis=-1) * general_sign
    return description * description_sign, joint_state, general


def get_policy(config, version):
    if version == "r9":
        required = {"network_width_multiplier": 2.0, "policy_symmetric_actor": True,
                    "policy_joint_history": True, "policy_body_history": True,
                    "history_lags": [1, 2, 4, 8], "store_observations_as_fp16": True}
        if any(config.get(key) != value for key, value in required.items()):
            raise ValueError("Deployment requires Bo's R9 actor with symmetry and joint/body history")
        return Policy(config["std_dev"], config["softmax_temperature"], config["softmax_temperature_min"],
                      config["stability_epsilon"], config["policy_mean_abs_clip"], config["policy_std_min_clip"],
                      config["policy_std_max_clip"], config["network_width_multiplier"], True, True, True, (1, 2, 4, 8))
    if config["policy_privileged_observations"] or not (config["policy_symmetric_actor"] and config["policy_joint_history"] and config["policy_body_history"] and config["store_observations_as_fp16"]):
        raise ValueError("Deployment requires a symmetric actor with joint and body history, FP16 observations and no privileged inputs")
    return Policy(config["std_dev"], config["softmax_temperature"], config["softmax_temperature_min"],
                  config["stability_epsilon"], config["policy_mean_abs_clip"], config["policy_std_min_clip"],
                  config["policy_std_max_clip"], config["network_width_multiplier"], True, True, True, tuple(config["history_lags"]),
                  config["history_routing"], config["encoder_width_multiplier"], config["core_width_multiplier"],
                  config["latent_width_multiplier"], config["history_width_multiplier"])


class Policy(nn.Module):
    std_dev: float
    softmax_temperature: float
    softmax_temperature_min: float
    stability_epsilon: float
    policy_mean_abs_clip: float
    policy_std_min_clip: float
    policy_std_max_clip: float
    network_width_multiplier: float
    symmetric_actor: bool = False
    joint_history: bool = False
    body_history: bool = False
    history_lags: tuple = (1, 2, 4, 8)
    history_routing: bool = False
    encoder_width_multiplier: float = 1.0
    core_width_multiplier: float = 1.0
    latent_width_multiplier: float = 1.0
    history_width_multiplier: float = 1.0


    @nn.compact
    def __call__(self, joint_description, joint_state, general_state, reflection=None, history_features=None, body_features=None, body_reflection=None):
        if self.joint_history:
            if history_features is None:
                if not self.is_initializing():
                    raise ValueError("Joint-history inference requires causal history features")
                history_features = jnp.zeros(joint_state.shape[:-1] + (4 * len(self.history_lags),), dtype=joint_state.dtype)
            joint_history_encoding = nn.gelu(nn.Dense(int(16 * self.history_width_multiplier), name="encoder_history_hidden")(jnp.concatenate([joint_state, history_features.astype(joint_state.dtype)], axis=-1)))
            history_residual = nn.Dense(int(8 * self.latent_width_multiplier), kernel_init=constant(0), bias_init=constant(0), name="encoder_history_output")(joint_history_encoding)
        if self.body_history and body_features is None:
            raise ValueError("Body-history inference requires causal history features")
        if self.symmetric_actor:
            if reflection is None:
                raise ValueError("Symmetric actor requires environment reflection metadata")
            reflected = reflect_policy_inputs(joint_description, joint_state, general_state, reflection)
            joint_description, joint_state, general_state = tuple(jnp.stack([a, b]) for a, b in zip((joint_description, joint_state, general_state), reflected))
            if self.body_history:
                body_features = jnp.stack([body_features, body_features * body_reflection])
        joint_log_softmax_temperature = self.param("joint_log_softmax_temperature", constant(jnp.log(self.softmax_temperature - self.softmax_temperature_min)), (1,))
        joints_present = joint_description[..., [-1]]
        joint_description = joint_description[..., :-1]
        if self.body_history and self.history_routing:
            body_history_encoding = nn.gelu(nn.Dense(int(16 * self.history_width_multiplier), name="encoder_body_history_hidden")(body_features.astype(general_state.dtype)))
        joint_description_latent = nn.Dense(int(256 * self.network_width_multiplier * self.encoder_width_multiplier), name="encoder_dense_0")(jnp.concatenate([joint_description, joint_state], axis=-1))
        joint_description_latent = nn.LayerNorm(name="encoder_layernorm")(joint_description_latent)
        joint_description_latent = nn.gelu(joint_description_latent)
        joint_description_latent = nn.Dense(int(256 * self.network_width_multiplier * self.encoder_width_multiplier), name="encoder_dense_1")(joint_description_latent)
        routing_logits = joint_description_latent
        if self.history_routing:
            if self.joint_history:
                routing_logits += nn.Dense(routing_logits.shape[-1], kernel_init=constant(0), bias_init=constant(0), name="encoder_joint_history_route")(joint_history_encoding)
            if self.body_history:
                routing_logits += nn.Dense(routing_logits.shape[-1], kernel_init=constant(0), bias_init=constant(0), name="encoder_body_history_route")(body_history_encoding)[..., None, :]
        joint_state_mask = jnp.clip(nn.tanh(routing_logits), -1.0 + self.stability_epsilon, 1.0 - self.stability_epsilon)
        latent_joint_state = nn.Dense(int(8 * self.latent_width_multiplier), name="encoder_latent_state")(joint_state)
        latent_joint_state = nn.gelu(latent_joint_state)
        if self.joint_history:
            latent_joint_state += history_residual
        latent_joint_state = latent_joint_state * joints_present
        joint_e_x = jnp.exp(joint_state_mask / (jnp.exp(joint_log_softmax_temperature) + self.softmax_temperature_min))
        joint_state_mask = joint_e_x / (joint_e_x.sum(axis=-1, keepdims=True) + self.stability_epsilon)
        joint_state_mask_expanded = jnp.repeat(jnp.expand_dims(joint_state_mask, axis=-1), latent_joint_state.shape[-1], axis=-1)
        masked_joint_state = joint_state_mask_expanded * jnp.expand_dims(latent_joint_state, axis=-2)
        masked_joint_state = jnp.reshape(masked_joint_state, masked_joint_state.shape[:-2] + (masked_joint_state.shape[-2] * masked_joint_state.shape[-1],))
        joint_latent = jnp.sum(masked_joint_state, axis=-2)

        combined_input = jnp.concatenate([joint_latent, general_state], axis=-1)

        action_latent = nn.WeightNorm(nn.Dense(int(512 * self.network_width_multiplier * self.core_width_multiplier), kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0), name="core_dense_0"), variable_filter={'kernel',}, name="core_weightnorm_0")(combined_input)
        if self.body_history:
            if not self.history_routing:
                body_history_encoding = nn.gelu(nn.Dense(int(16 * self.history_width_multiplier), name="encoder_body_history_hidden")(body_features.astype(general_state.dtype)))
            action_latent += nn.Dense(action_latent.shape[-1], kernel_init=constant(0), bias_init=constant(0), name="encoder_body_history_output")(body_history_encoding)
        action_latent = nn.LayerNorm(name="core_layernorm")(action_latent)
        action_latent = nn.gelu(action_latent)
        action_latent = nn.WeightNorm(nn.Dense(int(256 * self.network_width_multiplier * self.core_width_multiplier), kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0), name="core_dense_1"), variable_filter={'kernel',}, name="core_weightnorm_1")(action_latent)
        action_latent = nn.gelu(action_latent)
        action_latent = nn.WeightNorm(nn.Dense(int(256 * self.network_width_multiplier * self.core_width_multiplier), kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0), name="core_dense_2"), variable_filter={'kernel',}, name="core_weightnorm_2")(action_latent)
        action_latent = nn.gelu(action_latent)
        action_latent = nn.WeightNorm(nn.Dense(int(256 * self.network_width_multiplier * self.core_width_multiplier), kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0), name="core_dense_3"), variable_filter={'kernel',}, name="core_weightnorm_3")(action_latent)
        action_latent = nn.gelu(action_latent)
        action_latent = nn.WeightNorm(nn.Dense(int(256 * self.network_width_multiplier * self.encoder_width_multiplier), kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0), name="core_dense_4"), variable_filter={'kernel',}, name="core_weightnorm_4")(action_latent)

        action_latent = jnp.expand_dims(action_latent, axis=-2)
        policy_mean = action_latent * joint_state_mask
        policy_mean = jnp.sum(policy_mean, axis=-1)
        policy_mean = jnp.clip(policy_mean, -self.policy_mean_abs_clip, self.policy_mean_abs_clip)

        policy_logstd = nn.Dense(1, kernel_init=orthogonal(0.1), bias_init=constant(np.log(self.std_dev)), name="logstd")(joint_description_latent)
        policy_logstd = jnp.clip(policy_logstd, np.log(self.policy_std_min_clip), np.log(self.policy_std_max_clip))

        policy_logstd = policy_logstd.squeeze(-1)
        if self.symmetric_actor:
            policy_mean = jnp.mean(policy_mean, axis=0)
            policy_logstd = jnp.mean(policy_logstd, axis=0)
        return policy_mean, policy_logstd


def history_features(samples, age, lags):
    lags = jnp.array(lags)
    values = samples[..., lags - 1, :].reshape(samples.shape[:-2] + (len(lags) * samples.shape[-1],))
    valid = (age[..., None] >= lags).reshape(age.shape + (1,) * (samples.ndim - age.ndim - 2) + (len(lags),))
    valid = jnp.broadcast_to(valid, samples.shape[:-2] + (len(lags),))
    return jnp.concatenate([values, valid.astype(samples.dtype)], axis=-1)


def history_update(samples, sample, done):
    samples = jnp.concatenate([sample.astype(samples.dtype)[..., None, :], samples[..., :-1, :]], axis=-2)
    done = done.reshape(done.shape + (1,) * (samples.ndim - done.ndim))
    return jnp.where(done, jnp.zeros_like(samples), samples)
