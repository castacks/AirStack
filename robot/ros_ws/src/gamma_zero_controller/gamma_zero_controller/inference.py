import hashlib
import json
from pathlib import Path
import zipfile

import jax
import jax.numpy as jnp
import numpy as np
from flax import serialization

from gamma_zero_controller.architecture import get_policy


class StarlingPolicy:
    def __init__(self, assets, bundle):
        self.assets = json.loads(Path(assets).read_text())
        payload = Path(bundle).read_bytes()
        if self.assets["format"] != "starling_cmu-ctbr-inputs-v1" or hashlib.sha256(payload).hexdigest() != self.assets["bundle_sha256"]:
            raise ValueError("STARLING_CMU assets and bundle do not match")
        with zipfile.ZipFile(bundle) as archive:
            manifest = json.loads(archive.read("manifest.json"))
            self.algorithm = json.loads(archive.read("config_algorithm.json"))
            actor = archive.read("policy.msgpack")
        if manifest["format"] != "gamma-actor-v2" or hashlib.sha256(actor).hexdigest() != manifest["actor_sha256"]:
            raise ValueError("Invalid actor bundle")
        self.policy = get_policy(self.algorithm, "current")
        self.params = jax.tree_util.tree_map(jnp.asarray, serialization.msgpack_restore(actor))
        self.apply = jax.jit(self.policy.apply)
        self.descriptions = np.asarray(self.assets["descriptions"], dtype=np.float16).astype(np.float32)[None]
        self.reflection = tuple(jnp.asarray(x, dtype=jnp.int32 if i == 2 else jnp.float32) for i, x in enumerate(self.assets["reflection"]))
        self.body_indices = np.asarray(self.assets["body_indices"])
        self.body_reflection = jnp.asarray(self.assets["body_reflection"])
        self.lags = np.asarray(self.assets["history_lags"])
        if self.lags.tolist() != self.algorithm["history_lags"]:
            raise ValueError("History lags differ between assets and actor")
        self.reset_history()


    def reset_history(self):
        self.joint_history = np.zeros((4, int(self.lags.max()), 3), dtype=np.float16)
        self.body_history = np.zeros((int(self.lags.max()), len(self.body_indices)), dtype=np.float16)
        self.age = 0


    def inputs(self, state, goal_velocity, previous_action):
        previous_action, goal_velocity = np.asarray(previous_action), np.asarray(goal_velocity)
        if previous_action.shape != (4,) or goal_velocity.shape != (4,) or not np.isfinite(np.r_[previous_action, goal_velocity]).all():
            raise ValueError("Expected finite four-dimensional action and velocity command")
        if np.any(np.abs(goal_velocity) > self.assets["max_command_velocity"]) or np.any(goal_velocity*(1-np.asarray(self.assets["command_axis_mask"]))):
            raise ValueError("Velocity command outside exported training envelope")
        general = np.asarray(self.assets["general_template"]).copy()
        for key, norm in self.assets["dynamic"].items():
            value = goal_velocity if key == "goal_velocity" else np.asarray(state[key])
            if value.shape != (len(norm["indices"]),) or not np.isfinite(value).all():
                raise ValueError("Invalid policy state: "+key)
            general[norm["indices"]] = np.clip(value / norm["divisor"] - norm["offset"], norm["low"], norm["high"])
        joint = np.zeros((4, 4))
        joint[:, 2] = np.clip(previous_action / self.assets["previous_action_divisor"], -10, 10)
        return joint.astype(np.float16).astype(np.float32), general.astype(np.float16).astype(np.float32)


    def act(self, state, goal_velocity, previous_action):
        joint, general = self.inputs(state, goal_velocity, previous_action)
        valid = (self.age >= self.lags).astype(np.float32)
        joint_features = np.concatenate([self.joint_history[:, self.lags-1].reshape(4, -1), np.broadcast_to(valid, (4, len(self.lags)))], axis=-1)
        body = general[self.body_indices]
        body_features = np.r_[body, self.body_history[self.lags-1].reshape(-1), valid]
        mean, _ = self.apply(self.params, self.descriptions, joint[None], general[None], reflection=self.reflection,
                             history_features=joint_features[None], body_features=body_features[None], body_reflection=self.body_reflection)
        action = np.asarray(jax.device_get(mean))[0]
        if action.shape != (4,) or not np.isfinite(action).all():
            raise RuntimeError("Invalid policy action")
        self.joint_history[:, 1:] = self.joint_history[:, :-1].copy()
        self.joint_history[:, 0] = np.column_stack([joint[:, :2], action / self.assets["previous_action_divisor"]])
        self.body_history[1:] = self.body_history[:-1].copy()
        self.body_history[0] = body
        self.age = min(self.age+1, int(self.lags.max()))
        physical = action*np.asarray(self.assets["action_scale"]) + self.assets["action_offset"]
        limits = np.asarray(self.assets["action_limits"])
        return action, np.clip(physical, limits[:, 0], limits[:, 1])
