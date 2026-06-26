from dataclasses import dataclass, field
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
import torch

from svg_ground_control.diffaero.perception_builder import PerceptionBuilder, Intrinsics, PerceptionGrid


@dataclass
class DiffAeroObs:
    position_enu:       np.ndarray             # (3,) ENU world
    velocity_enu:       np.ndarray             # (3,) ENU world — MEASURED, never finite-differenced
    R_enu:              np.ndarray             # (3,3) FLU-body → ENU world rotation matrix
    goal_enu:           np.ndarray             # (3,) ENU world
    depth_planar:       np.ndarray | None = None  # (H,W) metric planar depth from sim; fed through PerceptionBuilder
    perception_encoded: np.ndarray | None = None  # (9,16) pre-encoded grid [0=clear, 1=obstacle]; skips PerceptionBuilder


@dataclass
class DiffAeroCmd:
    attitude_ned_frd_wxyz: np.ndarray   # PX4-ready quaternion [w,x,y,z]
    attitude_enu_flu_xyzw: np.ndarray   # ENU/FLU quaternion [x,y,z,w] — publish this to AttitudeThrust
    thrust_norm: float                  # normalized [0,1], anchored to max_accel
    acc_cmd_enu: np.ndarray             # debug: acceleration command in ENU
    acc_norm: float                     # debug: acceleration magnitude


class DiffAeroPolicy:
    # ENU inertial → NED inertial (same convention as Pegasus Simulator).
    _rot_ENU_to_NED = Rotation.from_quat([0.70711, 0.70711, 0.0, 0.0])
    # FLU body → FRD body (+π around X).
    _rot_FLU_to_FRD = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])

    def __init__(
        self,
        intrinsics: Intrinsics,
        checkpoint_path: str,
        grid: PerceptionGrid = PerceptionGrid(),
        vel_ema_factor: float = 0.1,
        max_acc_xy: float = 20.0,
        max_acc_z: float = 40.0,
        max_accel: float = 30.0,
        max_vel: float = 5.0,
        flip_lr: bool = False,
        flip_ud: bool = False,
    ):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        pt2_path = self._resolve_pt2(checkpoint_path)
        print(f"Loading DiffAero TorchScript actor from {pt2_path} ...")
        self.module = torch.jit.load(str(pt2_path), map_location=self.device)
        self.module.eval()

        self.min_action = torch.tensor(
            [[-max_acc_xy, -max_acc_xy, 0.0]], dtype=torch.float32, device=self.device)
        self.max_action = torch.tensor(
            [[max_acc_xy, max_acc_xy, max_acc_z]], dtype=torch.float32, device=self.device)

        self.vel_ema_factor = vel_ema_factor
        self.vel_ema: torch.Tensor | None = None
        self.max_vel_t = torch.tensor(max_vel, dtype=torch.float32, device=self.device)
        self.max_accel = max_accel

        self.perception_builder = PerceptionBuilder(intrinsics, grid=grid, flip_lr=flip_lr, flip_ud=flip_ud)
        self._up = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float32, device=self.device)

    def reset(self) -> None:
        """Clear the velocity EMA accumulated from the previous episode."""
        self.vel_ema = None

    @torch.no_grad()
    def compute(self, obs: DiffAeroObs) -> DiffAeroCmd:
        R = torch.tensor(obs.R_enu, dtype=torch.float32, device=self.device)        # (3,3) FLU→ENU
        v_world = torch.tensor(obs.velocity_enu, dtype=torch.float32, device=self.device)

        Rz = self._build_yaw_frame(R)   # (3,3) yaw-only frame
        uz = R[:, 2]                    # body up-axis in ENU world (3rd column of FLU→ENU)

        target_vel_world = self._compute_target_vel(obs.goal_enu, obs.position_enu)

        # Project into yaw frame (obs_frame=local convention from training).
        target_vel_local = Rz.t() @ target_vel_world   # (3,)
        v_local = Rz.t() @ v_world                     # (3,)
        state9 = torch.cat([target_vel_local, uz, v_local]).unsqueeze(0)  # (1,9)

        # Velocity EMA drives yaw orientation fed to the actor.
        # Initialize from heading direction (not raw velocity) so that transient
        # drift or post-interruption velocity does not corrupt the yaw on the
        # first tick after a reset.
        if self.vel_ema is None:
            self.vel_ema = Rz[:, 0].clone()   # start aligned with heading, blend toward velocity
        else:
            self.vel_ema = torch.lerp(self.vel_ema, v_world, self.vel_ema_factor)

        orientation = self.vel_ema.unsqueeze(0)     # (1,3)
        if orientation.norm() < 0.3:
            orientation = Rz[:, 0].unsqueeze(0)    # fall back to forward axis when near-stationary

        # Build perception tensor — prefer pre-encoded, then raw depth, then zeros.
        if obs.perception_encoded is not None:
            perception_t = torch.tensor(
                obs.perception_encoded, dtype=torch.float32, device=self.device
            ).unsqueeze(0)                          # (1,9,16)
        elif obs.depth_planar is not None:
            perception = self.perception_builder(obs.depth_planar)
            perception_t = torch.tensor(
                perception, dtype=torch.float32, device=self.device
            ).unsqueeze(0)
        else:
            perception_t = torch.zeros(
                1, self.perception_builder.grid.H, self.perception_builder.grid.W,
                dtype=torch.float32, device=self.device,
            )

        acc_cmd, quat_cmd, acc_norm = self.module(
            (state9, perception_t),
            orientation,
            Rz.unsqueeze(0),
            self.min_action,
            self.max_action,
        )

        acc_norm = float(acc_norm.reshape(-1)[0].cpu())
        q_des, thrust_norm = self._to_attitude_setpoint(
            quat_cmd.squeeze(0).cpu().numpy(), acc_norm)

        return DiffAeroCmd(
            attitude_ned_frd_wxyz=q_des,
            attitude_enu_flu_xyzw=quat_cmd.squeeze(0).cpu().numpy(),
            thrust_norm=thrust_norm,
            acc_cmd_enu=acc_cmd.squeeze(0).cpu().numpy(),
            acc_norm=acc_norm,
        )

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
            - torch.tensor(position, dtype=torch.float32, device=self.device)
        )
        dist = rel.norm()
        denom = torch.maximum(dist / self.max_vel_t, torch.ones((), device=self.device))
        return rel / denom

    def _to_attitude_setpoint(
        self, quat_xyzw_enu_flu: np.ndarray, acc_norm: float
    ) -> tuple[np.ndarray, float]:
        """Convert ENU/FLU actor output to NED/FRD quaternion + normalized thrust."""
        R_des_enu = Rotation.from_quat(quat_xyzw_enu_flu).as_matrix()
        q_des = self._quat_ENU_FLU_to_NED_FRD(R_des_enu)
        thrust_norm = float(np.clip(acc_norm / self.max_accel, 0.0, 1.0))
        return q_des, thrust_norm

    @staticmethod
    def _quat_ENU_FLU_to_NED_FRD(R_enu_flu: np.ndarray) -> np.ndarray:
        rot = (
            DiffAeroPolicy._rot_ENU_to_NED
            * Rotation.from_matrix(R_enu_flu)
            * DiffAeroPolicy._rot_FLU_to_FRD
        )
        q = rot.as_quat()   # scipy xyzw
        return np.array([q[3], q[0], q[1], q[2]])  # MAVLink wxyz

    @staticmethod
    def _resolve_pt2(checkpoint_path: str) -> Path:
        p = Path(checkpoint_path)
        if p.is_file():
            return p
        for candidate in [p / "checkpoints" / "exported_actor.pt2", p / "exported_actor.pt2"]:
            if candidate.exists():
                return candidate
        raise FileNotFoundError(
            f"Could not find exported_actor.pt2 under {checkpoint_path}.")
