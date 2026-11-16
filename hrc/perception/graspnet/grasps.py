"""Grasp pose records: GraspNet camera frame → flange / TCP / base (Dobot mm, deg)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from ...models.pgea import PGEA5026Model
from ...utils.transforms import matrix_to_pose, pose_to_matrix

# GraspNet-1Billion stores width/height/depth/center in metres.
GRASPNET_MAX_WIDTH_M = 0.10


def _mat4(rotation_3x3: np.ndarray, t_m: Sequence[float]) -> np.ndarray:
  T = np.eye(4)
  T[:3, :3] = np.asarray(rotation_3x3, dtype=float).reshape(3, 3)
  T[:3, 3] = np.asarray(t_m, dtype=float).reshape(3) * 1000.0  # m → mm
  return T


@dataclass
class GraspProposal:
  score: float
  width_m: float
  height_m: float
  depth_m: float
  rotation_cam: np.ndarray
  center_cam_m: np.ndarray
  object_id: int = -1
  source: str = "graspnet"
  extra: dict = field(default_factory=dict)

  @property
  def width_mm(self) -> float:
    return float(self.width_m) * 1000.0

  def T_cam(self) -> np.ndarray:
    """Camera ← grasp, millimetres (OpenCV camera frame)."""
    return _mat4(self.rotation_cam, self.center_cam_m)

  def pose_cam_mm(self) -> np.ndarray:
    return matrix_to_pose(self.T_cam())

  def T_in(self, T_parent_cam_mm: np.ndarray) -> np.ndarray:
    return np.asarray(T_parent_cam_mm, dtype=float) @ self.T_cam()

  def pose_in(self, T_parent_cam_mm: np.ndarray) -> np.ndarray:
    return matrix_to_pose(self.T_in(T_parent_cam_mm))

  def gripper_position_0_1000(self, model: Optional[PGEA5026Model] = None) -> float:
    """Map proposed jaw gap onto DH 0 (closed) … 1000 (open)."""
    model = model or PGEA5026Model()
    gap = float(np.clip(self.width_mm, model.MIN_GAP_MM, model.MAX_GAP_MM))
    frac = (gap - model.MIN_GAP_MM) / max(model.STROKE_MM, 1e-6)
    return float(np.clip(frac, 0.0, 1.0) * 1000.0)

  def fits_pgea(self, model: Optional[PGEA5026Model] = None, slack_mm: float = 0.5) -> bool:
    model = model or PGEA5026Model()
    return (model.MIN_GAP_MM - slack_mm) <= self.width_mm <= (model.MAX_GAP_MM + slack_mm)

  def as_dict(self) -> dict:
    pose = self.pose_cam_mm()
    return {
      "score": float(self.score),
      "source": self.source,
      "width_mm": round(self.width_mm, 2),
      "height_mm": round(float(self.height_m) * 1000.0, 2),
      "depth_mm": round(float(self.depth_m) * 1000.0, 2),
      "center_cam_m": np.asarray(self.center_cam_m, dtype=float).reshape(3).tolist(),
      "pose_cam_mm_deg": pose.round(3).tolist(),
      "object_id": int(self.object_id),
    }

  @classmethod
  def from_graspnet_row(cls, row: Sequence[float], source: str = "graspnet") -> "GraspProposal":
    """One GraspGroup row: score, width, height, depth, R(9), t(3), obj_id."""
    a = np.asarray(row, dtype=float).reshape(-1)
    if a.size < 17:
      raise ValueError(f"GraspNet row has {a.size} values, expected 17")
    return cls(
      score=float(a[0]),
      width_m=float(a[1]),
      height_m=float(a[2]),
      depth_m=float(a[3]),
      rotation_cam=a[4:13].reshape(3, 3),
      center_cam_m=a[13:16],
      object_id=int(a[16]),
      source=source,
    )


def filter_for_pgea(
  grasps: Sequence[GraspProposal],
  *,
  min_score: float = 0.0,
  slack_mm: float = 0.5,
  model: Optional[PGEA5026Model] = None,
) -> List[GraspProposal]:
  model = model or PGEA5026Model()
  out = []
  for g in grasps:
    if g.score < min_score:
      continue
    if not g.fits_pgea(model, slack_mm=slack_mm):
      continue
    out.append(g)
  return out


def sort_by_score(grasps: Sequence[GraspProposal]) -> List[GraspProposal]:
  return sorted(grasps, key=lambda g: float(g.score), reverse=True)
