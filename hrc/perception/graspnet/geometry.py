"""RGB-D → camera point cloud, and flange/base frame helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from .mount import T_flange_cam, T_flange_tcp
from ...utils.transforms import pose_to_matrix


@dataclass
class CameraIntrinsics:
  width: int = 1920
  height: int = 1080
  fx: float = 800.0
  fy: float = 800.0
  cx: float = 960.0
  cy: float = 540.0
  model: str = "pinhole"

  @classmethod
  def from_cfg(cls, cfg=None) -> "CameraIntrinsics":
    from ..cameras.base import camera_section

    section = camera_section(cfg)
    def g(name, default):
      if section is None:
        return default
      if hasattr(section, "get"):
        value = section.get(name, default)
        return default if value is None else value
      return getattr(section, name, default)
    w = int(g("width", 1920))
    h = int(g("height", 1080))
    return cls(
      width=w,
      height=h,
      fx=float(g("fx", 800.0)),
      fy=float(g("fy", 800.0)),
      cx=float(g("cx", w / 2.0)),
      cy=float(g("cy", h / 2.0)),
      model=str(g("intrinsics_model", "pinhole")),
    )

  def overlay_frame(self, frame) -> "CameraIntrinsics":
    """Copy live fx/fy/cx/cy (and size) from a CameraFrame when present."""
    if frame is None:
      return self
    if getattr(frame, "fx", None) is not None:
      self.fx = float(frame.fx)
    if getattr(frame, "fy", None) is not None:
      self.fy = float(frame.fy)
    if getattr(frame, "cx", None) is not None:
      self.cx = float(frame.cx)
    if getattr(frame, "cy", None) is not None:
      self.cy = float(frame.cy)
    rgb = getattr(frame, "rgb", None)
    if rgb is not None and hasattr(rgb, "shape") and len(rgb.shape) >= 2:
      self.height = int(rgb.shape[0])
      self.width = int(rgb.shape[1])
    else:
      if getattr(frame, "width", None):
        self.width = int(frame.width)
      if getattr(frame, "height", None):
        self.height = int(frame.height)
    return self


def depth_to_cloud(
  depth: np.ndarray,
  intr: CameraIntrinsics,
  *,
  rgb: Optional[np.ndarray] = None,
  factor: float = 1.0,
  min_m: float = 0.05,
  max_m: float = 1.5,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
  """Unproject an organized depth image to Nx3 camera metres (OpenCV frame).

  `factor` maps stored depth to metres (1000 if the image is millimetres).
  """
  z = np.asarray(depth, dtype=np.float32)
  if z.ndim == 3:
    z = z[..., 0]
  z = z / float(factor)
  h, w = z.shape[:2]
  u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
  valid = np.isfinite(z) & (z > min_m) & (z < max_m)
  z = z[valid]
  x = (u[valid] - float(intr.cx)) * z / max(float(intr.fx), 1e-6)
  y = (v[valid] - float(intr.cy)) * z / max(float(intr.fy), 1e-6)
  cloud = np.stack([x, y, z], axis=1)
  colors = None
  if rgb is not None:
    img = np.asarray(rgb)
    if img.shape[0] == h and img.shape[1] == w:
      colors = img[valid].reshape(-1, img.shape[-1] if img.ndim == 3 else 1)
      if colors.shape[-1] > 3:
        colors = colors[:, :3]
      colors = colors.astype(np.float32)
      if colors.max() > 1.5:
        colors = colors / 255.0
  return cloud, colors


def sample_cloud(cloud: np.ndarray, n: int, colors: Optional[np.ndarray] = None, rng=None):
  rng = np.random.default_rng() if rng is None else rng
  n = int(n)
  if cloud.shape[0] == 0:
    return cloud, colors
  if cloud.shape[0] >= n:
    idx = rng.choice(cloud.shape[0], n, replace=False)
  else:
    extra = rng.choice(cloud.shape[0], n - cloud.shape[0], replace=True)
    idx = np.concatenate([np.arange(cloud.shape[0]), extra])
  sampled = cloud[idx]
  sampled_c = None if colors is None else colors[idx]
  return sampled, sampled_c


def synthetic_table_depth(
  intr: CameraIntrinsics,
  *,
  z_m: float = 0.42,
  box_center_uv: Tuple[int, int] = (960, 620),
  box_size_px: Tuple[int, int] = (280, 200),
  box_height_m: float = 0.06,
) -> np.ndarray:
  """A flat table + a raised rectangle, for offline GraspNet tests (no robot)."""
  h, w = int(intr.height), int(intr.width)
  depth = np.full((h, w), float(z_m), dtype=np.float32)
  cx, cy = box_center_uv
  bw, bh = box_size_px
  y0, y1 = max(0, cy - bh // 2), min(h, cy + bh // 2)
  x0, x1 = max(0, cx - bw // 2), min(w, cx + bw // 2)
  depth[y0:y1, x0:x1] = float(z_m) - float(box_height_m)
  return depth


def T_base_flange_mm(tcp_pose_mm=None, cfg=None, joints=None, kin=None) -> np.ndarray:
  """Base ← flange in millimetres.

  Prefer the controller TCP pose (InverseKin source of truth) minus the tool.
  """
  T_ft = T_flange_tcp(cfg)
  if tcp_pose_mm is not None:
    T_base_tcp = pose_to_matrix(tcp_pose_mm)
    return T_base_tcp @ np.linalg.inv(T_ft)
  if kin is not None and joints is not None:
    return np.asarray(kin.flange_matrix(joints), dtype=float)
  return np.eye(4)


def T_base_cam_mm(tcp_pose_mm=None, cfg=None, joints=None, kin=None) -> np.ndarray:
  return T_base_flange_mm(tcp_pose_mm=tcp_pose_mm, cfg=cfg, joints=joints, kin=kin) @ T_flange_cam(cfg)
