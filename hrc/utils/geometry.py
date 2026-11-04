"""Collision primitives used by planners (mm)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence

import numpy as np


def _as3(value: Sequence[float]) -> np.ndarray:
  arr = np.asarray(value, dtype=float).reshape(-1)
  if arr.size != 3:
    raise ValueError(f"expected xyz, got {value}")
  return arr


def point_segment_distance(point: Sequence[float], a: Sequence[float], b: Sequence[float]) -> float:
  p, a3, b3 = _as3(point), _as3(a), _as3(b)
  ab = b3 - a3
  denom = float(np.dot(ab, ab))
  if denom < 1e-12:
    return float(np.linalg.norm(p - a3))
  t = float(np.clip(np.dot(p - a3, ab) / denom, 0.0, 1.0))
  return float(np.linalg.norm(p - (a3 + t * ab)))


def segment_segment_distance(
  a0: Sequence[float], a1: Sequence[float], b0: Sequence[float], b1: Sequence[float]
) -> float:
  p1, q1, p2, q2 = _as3(a0), _as3(a1), _as3(b0), _as3(b1)
  d1, d2, r = q1 - p1, q2 - p2, p1 - p2
  a, e, f = float(np.dot(d1, d1)), float(np.dot(d2, d2)), float(np.dot(d2, r))
  eps = 1e-12
  if a <= eps and e <= eps:
    return float(np.linalg.norm(p1 - p2))
  if a <= eps:
    return point_segment_distance(p1, p2, q2)
  if e <= eps:
    return point_segment_distance(p2, p1, q1)
  b, c = float(np.dot(d1, d2)), float(np.dot(d1, r))
  denom = a * e - b * b
  s = (b * f - c * e) / denom if denom > eps else 0.0
  s = float(np.clip(s, 0.0, 1.0))
  t = (b * s + f) / e
  if t < 0.0:
    t = 0.0
    s = float(np.clip(-c / a, 0.0, 1.0))
  elif t > 1.0:
    t = 1.0
    s = float(np.clip((b - c) / a, 0.0, 1.0))
  return float(np.linalg.norm((p1 + s * d1) - (p2 + t * d2)))


@dataclass
class Sphere:
  center: np.ndarray
  radius: float

  def at(self, t: float, velocity: Sequence[float] = (0, 0, 0)) -> "Sphere":
    return Sphere(self.center + np.asarray(velocity, dtype=float) * t, self.radius)


@dataclass
class Capsule:
  p1: np.ndarray
  p2: np.ndarray
  radius: float

  def at(self, t: float, velocity: Sequence[float] = (0, 0, 0)) -> "Capsule":
    v = np.asarray(velocity, dtype=float)
    return Capsule(self.p1 + v * t, self.p2 + v * t, self.radius)


@dataclass
class Obstacle:
  name: str
  kind: str
  radius_mm: float
  velocity_mms: np.ndarray
  center: Optional[np.ndarray] = None
  p1: Optional[np.ndarray] = None
  p2: Optional[np.ndarray] = None

  @classmethod
  def from_cfg(cls, item) -> "Obstacle":
    data = item.to_dict() if hasattr(item, "to_dict") else dict(item)
    kind = str(data["type"]).lower()
    vel = np.asarray(data.get("velocity_mms", [0, 0, 0]), dtype=float)
    if kind == "sphere":
      return cls(
        name=str(data.get("name", "sphere")),
        kind="sphere",
        radius_mm=float(data["radius_mm"]),
        velocity_mms=vel,
        center=_as3(data["center"]),
      )
    if kind == "capsule":
      return cls(
        name=str(data.get("name", "capsule")),
        kind="capsule",
        radius_mm=float(data["radius_mm"]),
        velocity_mms=vel,
        p1=_as3(data["p1"]),
        p2=_as3(data["p2"]),
      )
    raise ValueError(f"unsupported obstacle type: {kind}")

  def primitive_at(self, t: float):
    if self.kind == "sphere":
      return Sphere(self.center, self.radius_mm).at(t, self.velocity_mms)
    return Capsule(self.p1, self.p2, self.radius_mm).at(t, self.velocity_mms)


def collide_sphere_capsule(sphere: Sphere, capsule: Capsule, margin: float = 0.0) -> bool:
  return point_segment_distance(sphere.center, capsule.p1, capsule.p2) <= (
    sphere.radius + capsule.radius + margin
  )


def collide_capsule_capsule(a: Capsule, b: Capsule, margin: float = 0.0) -> bool:
  return segment_segment_distance(a.p1, a.p2, b.p1, b.p2) <= (a.radius + b.radius + margin)


def collide_sphere_sphere(a: Sphere, b: Sphere, margin: float = 0.0) -> bool:
  return float(np.linalg.norm(a.center - b.center)) <= (a.radius + b.radius + margin)


def collide_any(body, obstacle, margin: float = 0.0) -> bool:
  if isinstance(body, Sphere) and isinstance(obstacle, Sphere):
    return collide_sphere_sphere(body, obstacle, margin)
  if isinstance(body, Sphere) and isinstance(obstacle, Capsule):
    return collide_sphere_capsule(body, obstacle, margin)
  if isinstance(body, Capsule) and isinstance(obstacle, Sphere):
    return collide_sphere_capsule(obstacle, body, margin)
  if isinstance(body, Capsule) and isinstance(obstacle, Capsule):
    return collide_capsule_capsule(body, obstacle, margin)
  raise TypeError(f"cannot collide {type(body)} vs {type(obstacle)}")
