"""Rotation and pose conversions. Dobot Cartesian poses use Rx,Ry,Rz in degrees."""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np

DEG = np.pi / 180.0
RAD = 180.0 / np.pi


def wrap_deg(angle: float) -> float:
  return (angle + 180.0) % 360.0 - 180.0


def rpy_deg_to_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
  """Intrinsic XYZ (Rx, then Ry, then Rz) rotation, matching Dobot TCP RPY."""
  ax, ay, az = np.array([rx, ry, rz], dtype=float) * DEG
  cx, sx = np.cos(ax), np.sin(ax)
  cy, sy = np.cos(ay), np.sin(ay)
  cz, sz = np.cos(az), np.sin(az)
  rx_m = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
  ry_m = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
  rz_m = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
  return rz_m @ ry_m @ rx_m


def matrix_to_rpy_deg(rot: np.ndarray) -> np.ndarray:
  sy = -rot[2, 0]
  cy = np.sqrt(max(0.0, 1.0 - sy * sy))
  if cy > 1e-8:
    rx = np.arctan2(rot[2, 1], rot[2, 2])
    ry = np.arctan2(sy, cy)
    rz = np.arctan2(rot[1, 0], rot[0, 0])
  else:
    rx = np.arctan2(-rot[1, 2], rot[1, 1])
    ry = np.arctan2(sy, cy)
    rz = 0.0
  return np.array([rx, ry, rz]) * RAD


def rpy_deg_to_quat(rx: float, ry: float, rz: float) -> np.ndarray:
  """Return quaternion [w, x, y, z]."""
  rot = rpy_deg_to_matrix(rx, ry, rz)
  return matrix_to_quat(rot)


def matrix_to_quat(rot: np.ndarray) -> np.ndarray:
  m = np.asarray(rot, dtype=float)
  t = np.trace(m)
  if t > 0:
    s = 0.5 / np.sqrt(t + 1.0)
    w = 0.25 / s
    x = (m[2, 1] - m[1, 2]) * s
    y = (m[0, 2] - m[2, 0]) * s
    z = (m[1, 0] - m[0, 1]) * s
  elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
    s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
    w = (m[2, 1] - m[1, 2]) / s
    x = 0.25 * s
    y = (m[0, 1] + m[1, 0]) / s
    z = (m[0, 2] + m[2, 0]) / s
  elif m[1, 1] > m[2, 2]:
    s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
    w = (m[0, 2] - m[2, 0]) / s
    x = (m[0, 1] + m[1, 0]) / s
    y = 0.25 * s
    z = (m[1, 2] + m[2, 1]) / s
  else:
    s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
    w = (m[1, 0] - m[0, 1]) / s
    x = (m[0, 2] + m[2, 0]) / s
    y = (m[1, 2] + m[2, 1]) / s
    z = 0.25 * s
  q = np.array([w, x, y, z], dtype=float)
  return q / np.linalg.norm(q)


def quat_to_matrix(quat: Sequence[float]) -> np.ndarray:
  w, x, y, z = np.asarray(quat, dtype=float)
  n = w * w + x * x + y * y + z * z
  s = 2.0 / n if n > 0 else 0.0
  wx, wy, wz = s * w * x, s * w * y, s * w * z
  xx, xy, xz = s * x * x, s * x * y, s * x * z
  yy, yz, zz = s * y * y, s * y * z, s * z * z
  return np.array(
    [
      [1 - (yy + zz), xy - wz, xz + wy],
      [xy + wz, 1 - (xx + zz), yz - wx],
      [xz - wy, yz + wx, 1 - (xx + yy)],
    ]
  )


def quat_slerp(q0: Sequence[float], q1: Sequence[float], t: float) -> np.ndarray:
  a = np.asarray(q0, dtype=float)
  b = np.asarray(q1, dtype=float)
  a = a / np.linalg.norm(a)
  b = b / np.linalg.norm(b)
  dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
  if dot < 0:
    b = -b
    dot = -dot
  if dot > 0.9995:
    out = a + t * (b - a)
    return out / np.linalg.norm(out)
  theta = np.arccos(dot)
  s0 = np.sin((1 - t) * theta) / np.sin(theta)
  s1 = np.sin(t * theta) / np.sin(theta)
  return s0 * a + s1 * b


def pose_to_matrix(pose: Sequence[float]) -> np.ndarray:
  x, y, z, rx, ry, rz = [float(v) for v in pose]
  t = np.eye(4)
  t[:3, :3] = rpy_deg_to_matrix(rx, ry, rz)
  t[:3, 3] = [x, y, z]
  return t


def matrix_to_pose(mat: np.ndarray) -> np.ndarray:
  rpy = matrix_to_rpy_deg(mat[:3, :3])
  return np.array([mat[0, 3], mat[1, 3], mat[2, 3], rpy[0], rpy[1], rpy[2]])


def invert_pose(pose: Sequence[float]) -> np.ndarray:
  return matrix_to_pose(np.linalg.inv(pose_to_matrix(pose)))


def compose_pose(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
  """Apply pose b in the frame of pose a."""
  return matrix_to_pose(pose_to_matrix(a) @ pose_to_matrix(b))


def interpolate_pose(start: Sequence[float], goal: Sequence[float], t: float) -> np.ndarray:
  s = np.asarray(start, dtype=float)
  g = np.asarray(goal, dtype=float)
  xyz = s[:3] + t * (g[:3] - s[:3])
  q = quat_slerp(rpy_deg_to_quat(*s[3:]), rpy_deg_to_quat(*g[3:]), t)
  rpy = matrix_to_rpy_deg(quat_to_matrix(q))
  return np.concatenate([xyz, rpy])


def modified_dh(a: float, alpha_deg: float, d: float, theta_deg: float) -> np.ndarray:
  """Modified DH (Craig). a, d in mm; alpha, theta in deg."""
  al = alpha_deg * DEG
  th = theta_deg * DEG
  ca, sa = np.cos(al), np.sin(al)
  ct, st = np.cos(th), np.sin(th)
  return np.array(
    [
      [ct, -st, 0, a],
      [st * ca, ct * ca, -sa, -d * sa],
      [st * sa, ct * sa, ca, d * ca],
      [0, 0, 0, 1],
    ]
  )
