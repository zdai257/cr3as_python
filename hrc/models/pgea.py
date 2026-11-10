"""Digital model of the DH Robotics PGEA-50-26 gripper."""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from ..utils.geometry import Capsule, Sphere
from ..utils.transforms import pose_to_matrix


def repo_root() -> Path:
  return Path(__file__).resolve().parents[2]


def gripper_model_dir() -> Path:
  return repo_root() / "models" / "pgea_50_26"


class PGEA5026Model:
  """Datasheet envelope + CS3AS tool frame (z = 118 mm)."""

  NAME = "DH_PGEA_50_26"
  LENGTH_MM = 118.0
  WIDTH_MM = 55.0
  THICKNESS_MM = 29.0
  STROKE_MM = 26.0
  MIN_GAP_MM = 1.2
  MAX_GAP_MM = 27.2
  MASS_KG = 0.5
  FORCE_N = (15.0, 50.0)
  BODY_RADIUS_MM = 28.0
  FINGER_RADIUS_MM = 8.0

  def __init__(self, cfg=None):
    self.cfg = cfg
    offset = [0.0, 0.0, self.LENGTH_MM]
    rpy = [0.0, 0.0, 0.0]
    if cfg is not None:
      offset = list(cfg.get("tool_offset_mm", offset)) if hasattr(cfg, "get") else list(
        getattr(cfg, "tool_offset_mm", offset)
      )
      rpy = list(cfg.get("tcp_rpy_deg", rpy)) if hasattr(cfg, "get") else list(
        getattr(cfg, "tcp_rpy_deg", rpy)
      )
    self.tcp_offset_mm = np.asarray(offset, dtype=float)
    self.tcp_rpy_deg = np.asarray(rpy, dtype=float)

  @property
  def urdf_path(self) -> Path:
    return gripper_model_dir() / "urdf" / "pgea_50_26.urdf"

  @property
  def mesh_dir(self) -> Path:
    return gripper_model_dir() / "meshes"

  def tool_pose(self) -> List[float]:
    """Dobot SetTool vector [x, y, z, rx, ry, rz] in mm / deg."""
    return [
      float(self.tcp_offset_mm[0]),
      float(self.tcp_offset_mm[1]),
      float(self.tcp_offset_mm[2]),
      float(self.tcp_rpy_deg[0]),
      float(self.tcp_rpy_deg[1]),
      float(self.tcp_rpy_deg[2]),
    ]

  def tool_matrix(self) -> np.ndarray:
    pose = self.tool_pose()
    return pose_to_matrix(pose)

  def opening_mm(self, position_0_1000: Optional[float]) -> float:
    """Map DH position (0=closed, 1000=open) to fingertip gap in mm."""
    if position_0_1000 is None:
      return float(self.MAX_GAP_MM)
    frac = float(np.clip(position_0_1000, 0.0, 1000.0)) / 1000.0
    return float(self.MIN_GAP_MM + frac * self.STROKE_MM)

  def jaw_half_mm(self, position_0_1000: Optional[float]) -> float:
    return 0.5 * self.opening_mm(position_0_1000)

  def capsules(self, flange_T: np.ndarray, position_0_1000: Optional[float] = 1000) -> List[Capsule]:
    """Collision capsules in world mm given the 4x4 flange transform."""
    T = np.asarray(flange_T, dtype=float)
    origin = T[:3, 3]
    z_axis = T[:3, 2]
    y_axis = T[:3, 1]
    tcp = origin + z_axis * float(self.tcp_offset_mm[2])
    body = Capsule(origin.copy(), tcp.copy(), self.BODY_RADIUS_MM)
    half = self.jaw_half_mm(position_0_1000)
    finger_z = origin + z_axis * (float(self.tcp_offset_mm[2]) - 9.0)
    left = finger_z + y_axis * half
    right = finger_z - y_axis * half
    fingers = [
      Capsule(left.copy(), (left + z_axis * 18.0).copy(), self.FINGER_RADIUS_MM),
      Capsule(right.copy(), (right + z_axis * 18.0).copy(), self.FINGER_RADIUS_MM),
    ]
    return [body, *fingers]

  def tcp_sphere(self, flange_T: np.ndarray, radius: float = 20.0) -> Sphere:
    T = np.asarray(flange_T, dtype=float)
    tcp = T[:3, 3] + T[:3, 2] * float(self.tcp_offset_mm[2])
    return Sphere(tcp.copy(), radius)

  def as_dict(self) -> dict:
    return {
      "name": self.NAME,
      "mass_kg": self.MASS_KG,
      "length_mm": self.LENGTH_MM,
      "width_mm": self.WIDTH_MM,
      "thickness_mm": self.THICKNESS_MM,
      "stroke_mm": self.STROKE_MM,
      "tcp_offset_mm": self.tcp_offset_mm.tolist(),
      "urdf": str(self.urdf_path),
    }
