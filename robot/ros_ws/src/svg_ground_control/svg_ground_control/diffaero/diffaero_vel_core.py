"""DiffAero velocity-command policy wrapper.

Loads a TorchScript actor exported with ``action_is_velocity=true`` (see
``diffaero/utils/exporter.py``). Unlike the attitude+thrust policy in
``diffaero_core.py``, the exported actor returns a *single* world-ENU velocity
setpoint ``[vx, vy, vz]`` — there is no accel→attitude conversion, because the
flight controller owns attitude in velocity mode.

Observation layout (``obs_frame=local``): ``[target_vel_local(3), v_local(3)]``
— a 6-D state (note: no body up-axis ``uz``, unlike the 9-D attitude obs). When
the checkpoint was trained with ``env=oa`` (obstacle avoidance) the actor also
consumes a perception grid; this wrapper feeds the same pre-encoded 9×16 ToF
grid used by the attitude commander (``perception_encoded`` path), falling back
to raw depth or zeros.

Yaw: the velocity policy was trained with ``align_yaw_with_vel_ema=true`` — the
drone noses into its direction of travel. The exported actor does not emit a yaw
setpoint, so this wrapper tracks a velocity EMA and exposes the desired ENU yaw
(heading aligned with the EMA); the commander turns that into a yaw-rate command.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import yaml

from svg_ground_control.diffaero.diffaero_core import DiffAeroObs
from svg_ground_control.diffaero.perception_builder import (
    Intrinsics, PerceptionBuilder, PerceptionGrid)


@dataclass
class DiffAeroVelCmd:
    vel_cmd_enu: np.ndarray         # (3,) world-frame velocity setpoint [m/s]
    vel_norm: float                 # magnitude of vel_cmd_enu
    desired_yaw_enu: float          # ENU yaw (rad, CCW from +x/East) of the EMA heading


class DiffAeroVelPolicy:
    def __init__(
        self,
        intrinsics: Intrinsics,
        checkpoint_path: str,
        grid: PerceptionGrid = PerceptionGrid(),
        vel_ema_factor: float | None = None,
        max_vel_xy: float | None = None,
        max_vel_z: float | None = None,
        max_vel: float | None = None,
        flip_lr: bool = False,
        flip_ud: bool = False,
    ):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        ckpt_dir = self._resolve_checkpoint_dir(checkpoint_path)
        cfg = self._load_hydra_config(ckpt_dir)
        self._validate_config(cfg)

        self.uses_perception = cfg["env"]["name"] == "obstacle_avoidance"
        dyn = cfg["dynamics"]

        # Network architecture (mlp | cnn | rnn | rcnn) selects the exported
        # actor's call signature; planar policies emit a 2-D [vx, vy] action
        # (the exported actor pads vz=0 internally), so the action-limit tensors
        # must be 2-D to match.
        self.network_name = str(cfg["network"]["name"]).lower()
        self.planar = bool(dyn.get("planar", False))
        self.action_dim = 2 if self.planar else 3

        # Defaults come from the training config unless overridden by the caller.
        self.vel_ema_factor = (
            vel_ema_factor if vel_ema_factor is not None
            else float(dyn["vel_ema_factor"]["default"]))
        # Below this horizontal speed the travel direction is noisy/undefined, so
        # the desired yaw is held instead of chasing the EMA — this stops the
        # heading from spinning when hovering at/near the goal (matches training,
        # dynamics/pointmass.py). Only planar configs define it; default to 0.3.
        self.yaw_hold_speed = float(dyn.get("yaw_hold_speed", 0.3))
        self._last_desired_yaw: float | None = None
        max_vel_xy = (max_vel_xy if max_vel_xy is not None
                      else float(dyn["max_vel"]["xy"]["default"]))
        max_vel_z = (max_vel_z if max_vel_z is not None
                     else float(dyn["max_vel"]["z"]["default"]))
        if max_vel is None:
            max_vel = float(cfg["env"].get("max_target_vel", max_vel_xy))

        pt2_path = self._resolve_pt2(checkpoint_path)
        print(f"Loading DiffAero velocity TorchScript actor from {pt2_path} ...")
        self.module = torch.jit.load(str(pt2_path), map_location=self.device)
        self.module.eval()

        # Action limits — these rescale the actor's [-1,1] tanh output to a
        # velocity setpoint (matches min/max_action in exporter.py). Planar
        # policies omit the z-component (no vertical action).
        if self.planar:
            self.min_action = torch.tensor(
                [[-max_vel_xy, -max_vel_xy]],
                dtype=torch.float32, device=self.device)
            self.max_action = torch.tensor(
                [[max_vel_xy, max_vel_xy]],
                dtype=torch.float32, device=self.device)
        else:
            self.min_action = torch.tensor(
                [[-max_vel_xy, -max_vel_xy, -max_vel_z]],
                dtype=torch.float32, device=self.device)
            self.max_action = torch.tensor(
                [[max_vel_xy, max_vel_xy, max_vel_z]],
                dtype=torch.float32, device=self.device)
        self.max_vel_t = torch.tensor(max_vel, dtype=torch.float32, device=self.device)
        self.vel_ema: torch.Tensor | None = None

        # Recurrent nets (rnn/rcnn) thread a GRU hidden state across ticks. Shape
        # matches the exporter: (rnn_n_layers, batch=1, rnn_hidden_dim).
        self.is_recurrent = self.network_name in ("rnn", "rcnn")
        self.hidden_shape: tuple[int, int, int] | None = None
        if self.is_recurrent:
            net = cfg["network"]
            self.hidden_shape = (
                int(net["rnn_n_layers"]), 1, int(net["rnn_hidden_dim"]))
        self.hidden: torch.Tensor | None = None
        self._reset_hidden()

        self.perception_builder = PerceptionBuilder(
            intrinsics, grid=grid, flip_lr=flip_lr, flip_ud=flip_ud)
        self._up = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float32, device=self.device)

        kind = "planar (2-D action, vz=0)" if self.planar else "full 3-D"
        if self.uses_perception:
            print(f"Velocity checkpoint: {self.network_name} net, {kind}, "
                  "obstacle-avoidance perception (state + grid).")
        else:
            print(f"Velocity checkpoint: {self.network_name} net, {kind}, "
                  "state-only observations (no perception).")

    def _reset_hidden(self) -> None:
        """Zero the GRU hidden state (recurrent nets only)."""
        if self.is_recurrent:
            self.hidden = torch.zeros(
                self.hidden_shape, dtype=torch.float32, device=self.device)
        else:
            self.hidden = None

    def reset(self) -> None:
        """Clear the velocity EMA (and GRU hidden state) from the previous episode."""
        self.vel_ema = None
        self._last_desired_yaw = None
        self._reset_hidden()

    @torch.no_grad()
    def compute(self, obs: DiffAeroObs) -> DiffAeroVelCmd:
        R = torch.tensor(obs.R_enu, dtype=torch.float32, device=self.device)   # FLU→ENU
        v_world = torch.tensor(obs.velocity_enu, dtype=torch.float32, device=self.device)

        Rz = self._build_yaw_frame(R)   # (3,3) yaw-only frame, cols = [fwd, left, up]

        target_vel_world = self._compute_target_vel(obs.goal_enu, obs.position_enu)
        # Project into yaw frame (obs_frame=local convention from training).
        target_vel_local = Rz.t() @ target_vel_world
        v_local = Rz.t() @ v_world
        state6 = torch.cat([target_vel_local, v_local]).unsqueeze(0)   # (1,6)

        # Velocity EMA drives the desired yaw (heading aligned with travel, as in
        # training). Initialize from the heading direction rather than raw
        # velocity so transient drift after a reset does not corrupt the yaw.
        if self.vel_ema is None:
            self.vel_ema = Rz[:, 0].clone()
        else:
            self.vel_ema = torch.lerp(self.vel_ema, v_world, self.vel_ema_factor)

        # orientation/Rz are part of the exported signature; orientation is unused
        # by the velocity actor (it only rescales + rotates the action via Rz).
        orientation = self.vel_ema.unsqueeze(0)
        if orientation.norm() < 0.3:
            orientation = Rz[:, 0].unsqueeze(0)

        # Perception tensor — prefer pre-encoded ToF, then raw depth, then zeros.
        if obs.perception_encoded is not None:
            perception_t = torch.tensor(
                obs.perception_encoded, dtype=torch.float32, device=self.device
            ).unsqueeze(0)
        elif obs.depth_planar is not None:
            perception = self.perception_builder(obs.depth_planar)
            perception_t = torch.tensor(
                perception, dtype=torch.float32, device=self.device).unsqueeze(0)
        else:
            perception_t = torch.zeros(
                1, self.perception_builder.grid.H, self.perception_builder.grid.W,
                dtype=torch.float32, device=self.device)

        # Dispatch on network type — each exported actor has a distinct call
        # signature (see diffaero/utils/exporter.py forward_*_vel):
        #   mlp/rnn : perception is packed into the state tuple
        #   cnn/rcnn: perception is a separate positional arg
        #   rnn/rcnn: additionally take/return a GRU hidden state
        Rz_b = Rz.unsqueeze(0)
        if self.network_name == "cnn":
            vel_cmd = self.module(
                state6, perception_t, orientation, Rz_b,
                self.min_action, self.max_action)
        elif self.network_name == "rcnn":
            vel_cmd, self.hidden = self.module(
                state6, perception_t, orientation, Rz_b,
                self.min_action, self.max_action, self.hidden)
        elif self.network_name == "rnn":
            actor_state = (state6, perception_t) if self.uses_perception else state6
            vel_cmd, self.hidden = self.module(
                actor_state, orientation, Rz_b,
                self.min_action, self.max_action, self.hidden)
        else:  # mlp (default / current path)
            actor_state = (state6, perception_t) if self.uses_perception else state6
            vel_cmd = self.module(
                actor_state, orientation, Rz_b,
                self.min_action, self.max_action)
        vel_cmd_enu = vel_cmd.squeeze(0).cpu().numpy()
        vel_norm = float(np.linalg.norm(vel_cmd_enu))
        desired_yaw_enu = self._desired_yaw_enu(Rz)

        return DiffAeroVelCmd(
            vel_cmd_enu=vel_cmd_enu,
            vel_norm=vel_norm,
            desired_yaw_enu=desired_yaw_enu,
        )

    def _desired_yaw_enu(self, Rz: torch.Tensor) -> float:
        """ENU yaw (CCW from +x/East) the drone should nose into.

        While travelling faster than ``yaw_hold_speed`` the heading tracks the
        velocity EMA (direction of travel) and is latched. Below that speed the
        EMA direction is noisy/undefined (e.g. hovering at the goal), so the yaw
        is held at the last travel heading rather than chased — preventing the
        heading vector from spinning near the goal. Non-planar policies keep the
        previous behaviour (fall back to the current forward axis)."""
        if self.vel_ema is not None and self.vel_ema[:2].norm() >= self.yaw_hold_speed:
            yaw = float(np.arctan2(self.vel_ema[1].item(), self.vel_ema[0].item()))
            self._last_desired_yaw = yaw
            return yaw
        if self.planar and self._last_desired_yaw is not None:
            return self._last_desired_yaw
        fwd = Rz[:, 0]
        return float(np.arctan2(fwd[1].item(), fwd[0].item()))

    def _build_yaw_frame(self, R: torch.Tensor) -> torch.Tensor:
        """Yaw-only rotation matrix: strips pitch/roll, columns = [fwd, left, up] in ENU."""
        fwd = R[:, 0].clone()
        fwd[2] = 0.0
        fwd = torch.nn.functional.normalize(fwd, dim=0)
        left = torch.cross(self._up, fwd, dim=0)
        left = torch.nn.functional.normalize(left, dim=0)
        return torch.stack([fwd, left, self._up], dim=1)

    def _compute_target_vel(self, goal_enu: np.ndarray, position: np.ndarray) -> torch.Tensor:
        """target_vel = (goal - pos) / max(dist/max_vel, 1) — saturates to max_vel far out."""
        rel = (
            torch.tensor(goal_enu, dtype=torch.float32, device=self.device)
            - torch.tensor(position, dtype=torch.float32, device=self.device))
        dist = rel.norm()
        denom = torch.maximum(dist / self.max_vel_t, torch.ones((), device=self.device))
        return rel / denom

    # ------------------------------------------------------------------
    # Checkpoint resolution / validation
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_checkpoint_dir(checkpoint_path: str) -> Path:
        p = Path(checkpoint_path)
        if p.is_file():
            p = p.parent
        if (p / ".hydra" / "config.yaml").exists():
            return p
        if (p.parent / ".hydra" / "config.yaml").exists():
            return p.parent
        raise FileNotFoundError(
            f"Could not find .hydra/config.yaml near checkpoint path {checkpoint_path}")

    @staticmethod
    def _resolve_pt2(checkpoint_path: str) -> Path:
        p = Path(checkpoint_path)
        if p.is_file():
            return p
        for c in (p / "checkpoints" / "exported_actor.pt2", p / "exported_actor.pt2"):
            if c.exists():
                return c
        raise FileNotFoundError(
            f"Could not find exported_actor.pt2 under {checkpoint_path}")

    @staticmethod
    def _load_hydra_config(ckpt_dir: Path) -> dict:
        with open(ckpt_dir / ".hydra" / "config.yaml") as f:
            return yaml.safe_load(f)

    @staticmethod
    def _validate_config(cfg: dict) -> None:
        dyn = cfg.get("dynamics", {})
        if dyn.get("name") != "velocity_pointmass":
            raise ValueError(
                f"Expected dynamics.name=velocity_pointmass, got {dyn.get('name')!r}")
        if not dyn.get("action_is_velocity", False):
            raise ValueError("Checkpoint dynamics.action_is_velocity is not true")
