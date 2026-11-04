"""FK / IK helpers.

`Kinematics.fk` uses the TBC DH table in config (for planning collision checks).
`controller_ik` / `controller_fk` call the CS3AS InverseKin / PositiveKin APIs.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .dashboard import as_vector
from .geometry import Capsule, Sphere
from .transforms import modified_dh, matrix_to_pose, pose_to_matrix


class Kinematics:
  def __init__(self, cfg, robot=None):
    self.cfg = cfg
    self.robot = robot
    dh = cfg.dh if hasattr(cfg, "dh") else cfg.get("dh")
    self.dh = [item.to_dict() if hasattr(item, "to_dict") else dict(item) for item in dh]
    limits = cfg.joint_limits_deg if hasattr(cfg, "joint_limits_deg") else cfg["joint_limits_deg"]
    self.joint_limits = np.asarray(
      [list(item) if not hasattr(item, "to_dict") else list(item) for item in limits],
      dtype=float,
    )
    radii = cfg.link_radii_mm if hasattr(cfg, "link_radii_mm") else cfg.get("link_radii_mm", [40] * 6)
    self.link_radii = [float(r) for r in radii]

  def fk_matrices(self, joints_deg: Sequence[float]) -> List[np.ndarray]:
    joints = np.asarray(joints_deg, dtype=float)
    if joints.size != 6:
      raise ValueError("need 6 joint angles")
    frames = [np.eye(4)]
    t = np.eye(4)
    for i, row in enumerate(self.dh):
      theta = joints[i] + float(row["theta_offset_deg"])
      t = t @ modified_dh(float(row["a_mm"]), float(row["alpha_deg"]), float(row["d_mm"]), theta)
      frames.append(t.copy())
    return frames

  def fk(self, joints_deg: Sequence[float]) -> np.ndarray:
    return matrix_to_pose(self.fk_matrices(joints_deg)[-1])

  def link_capsules(self, joints_deg: Sequence[float]) -> List[Capsule]:
    frames = self.fk_matrices(joints_deg)
    capsules = []
    origins = [f[:3, 3] for f in frames]
    for i in range(len(origins) - 1):
      capsules.append(
        Capsule(origins[i].copy(), origins[i + 1].copy(), self.link_radii[min(i, len(self.link_radii) - 1)])
      )
    return capsules

  def tcp_sphere(self, joints_deg: Sequence[float], radius: float = 30.0) -> Sphere:
    pose = self.fk(joints_deg)
    return Sphere(pose[:3].copy(), radius)

  def jacobian(self, joints_deg: Sequence[float], h: float = 1e-3) -> np.ndarray:
    q = np.asarray(joints_deg, dtype=float)
    j = np.zeros((6, 6))
    p0 = self.fk(q)
    for i in range(6):
      dq = q.copy()
      dq[i] += h
      j[:, i] = (self.fk(dq) - p0) / h
    return j

  def ik(
    self,
    pose: Sequence[float],
    seed: Optional[Sequence[float]] = None,
    iters: int = 80,
    pos_tol: float = 0.5,
    rot_tol: float = 1.0,
    damp: float = 1e-2,
  ) -> Tuple[np.ndarray, bool]:
    """Damped-least-squares IK on the local DH model (planning only)."""
    q = np.zeros(6) if seed is None else np.asarray(seed, dtype=float).copy()
    target = np.asarray(pose, dtype=float)
    for _ in range(iters):
      err = target - self.fk(q)
      err[3:] = np.array([_wrap(v) for v in err[3:]])
      if np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < rot_tol:
        return self._clamp(q), True
      jac = self.jacobian(q)
      jjt = jac @ jac.T + damp * np.eye(6)
      q = self._clamp(q + jac.T @ np.linalg.solve(jjt, err))
    return self._clamp(q), False

  def controller_fk(self, joints_deg: Sequence[float]) -> np.ndarray:
    if self.robot is None:
      raise RuntimeError("controller FK needs a connected robot")
    return np.asarray(as_vector(self.robot.robot_control.PositiveKin(list(joints_deg))), dtype=float)

  def controller_ik(
    self,
    pose: Sequence[float],
    joint_near: Optional[Sequence[float]] = None,
  ) -> np.ndarray:
    if self.robot is None:
      raise RuntimeError("controller IK needs a connected robot")
    if joint_near is None:
      response = self.robot.robot_control.InverseKin(list(pose))
    else:
      response = self.robot.robot_control.InverseKin(
        list(pose), use_joint_near=1, joint_near=list(joint_near)
      )
    return np.asarray(as_vector(response), dtype=float)

  def _clamp(self, joints: np.ndarray) -> np.ndarray:
    lo, hi = self.joint_limits[:, 0], self.joint_limits[:, 1]
    return np.clip(joints, lo, hi)


def _wrap(angle: float) -> float:
  return (angle + 180.0) % 360.0 - 180.0
