"""Nominal eye-in-hand extrinsics from the 3D-printed CS3AS wrist camera holder.

CAD: models/camera_holder/cr3_camera_half_clear.step (mm).
Numbers are a geometry-derived guess — set calibrated=true after a real calibration.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from ...utils.transforms import pose_to_matrix, rpy_deg_to_matrix
from ...models.pgea import PGEA5026Model, repo_root

# --- Parsed from cr3_camera_half_clear.step ---
CAD_WRIST_CENTER_MM = np.array([-57.88664685705, 0.0, 0.0], dtype=float)
CAD_COLLAR_INNER_RADIUS_MM = 41.1
CAD_COLLAR_OUTER_RADIUS_MM = 45.1
CAD_TILT_DEG = 5.0
# Midpoint of the two M3 camera screws (CAD).
CAD_HOLE_MID_MM = np.array([34.46655819713, 0.0, 49.102743151523], dtype=float)
# Board + M12 lens from the screw plane along the optical axis (TBC).
LENS_ALONG_OPTICAL_MM = 8.0

# ELP 06MT-KL170IR
CAMERA_BOARD_MM = (38.0, 38.0)
CAMERA_FOV_DIAG_DEG = 170.0
CAMERA_SENSOR = "OV2710 1/2.7-inch"


def _as_list(value, default):
  if value is None:
    return list(default)
  if hasattr(value, "to_dict"):
    value = value.to_dict()
  return [float(v) for v in value]


def _cfg_section(cfg, *names):
  cur = cfg
  for name in names:
    if cur is None:
      return None
    if hasattr(cur, "get"):
      cur = cur.get(name)
    else:
      cur = getattr(cur, name, None)
  return cur


def cad_to_flange_mm(point_cad_mm: Sequence[float]) -> np.ndarray:
  """Shift CAD so the wrist axis is the flange origin (CAD +Z = flange +Z)."""
  return np.asarray(point_cad_mm, dtype=float) - CAD_WRIST_CENTER_MM


def nominal_t_flange_cam_mm(lens_offset_mm: float = LENS_ALONG_OPTICAL_MM) -> np.ndarray:
  hole = cad_to_flange_mm(CAD_HOLE_MID_MM)
  n = rpy_deg_to_matrix(0.0, -CAD_TILT_DEG, 0.0) @ np.array([0.0, 0.0, 1.0])
  return hole + float(lens_offset_mm) * n


def T_flange_cam(cfg=None) -> np.ndarray:
  """4x4 transform: camera OpenCV frame (m later) expressed in the flange, mm."""
  section = _cfg_section(cfg, "perception", "camera_mount") if cfg is not None else None
  t = _as_list(
    None if section is None else (section.get("t_flange_cam_mm") if hasattr(section, "get") else None),
    nominal_t_flange_cam_mm(),
  )
  rpy = _as_list(
    None if section is None else (section.get("rpy_flange_cam_deg") if hasattr(section, "get") else None),
    [0.0, -CAD_TILT_DEG, 0.0],
  )
  return pose_to_matrix([t[0], t[1], t[2], rpy[0], rpy[1], rpy[2]])


def T_flange_tcp(cfg=None) -> np.ndarray:
  gripper = _cfg_section(cfg, "gripper") if cfg is not None else None
  model = PGEA5026Model(gripper)
  return model.tool_matrix()


def cad_path() -> Path:
  return repo_root() / "models" / "camera_holder" / "cr3_camera_half_clear.step"


def mount_as_dict(cfg=None) -> dict:
  section = _cfg_section(cfg, "perception", "camera_mount") if cfg is not None else None
  calibrated = False
  if section is not None and hasattr(section, "get"):
    calibrated = bool(section.get("calibrated", False))
  t = T_flange_cam(cfg)[:3, 3]
  return {
    "cad": str(cad_path()),
    "calibrated": calibrated,
    "collar_inner_diameter_mm": 2.0 * CAD_COLLAR_INNER_RADIUS_MM,
    "tilt_deg": CAD_TILT_DEG,
    "t_flange_cam_mm": t.round(3).tolist(),
    "camera": CAMERA_SENSOR,
    "board_mm": list(CAMERA_BOARD_MM),
    "fov_diag_deg": CAMERA_FOV_DIAG_DEG,
    "gripper_tcp_z_mm": float(PGEA5026Model().LENGTH_MM),
  }
