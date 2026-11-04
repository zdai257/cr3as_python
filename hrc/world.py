"""Scene snapshot used by every planner."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence

import numpy as np

from .utils.geometry import Capsule, Obstacle, Sphere, collide_any
from .utils.kinematics import Kinematics


@dataclass
class Observation:
  """Stable observation dict for classical and learned planners / VLA."""

  timestamp: float
  joints: np.ndarray
  pose: np.ndarray
  world: "WorldState"
  rgb: Optional[np.ndarray] = None
  depth: Optional[np.ndarray] = None
  pointcloud: Optional[np.ndarray] = None
  skin_conductance: Optional[np.ndarray] = None
  skin_approach: Optional[np.ndarray] = None

  def as_policy_obs(self) -> dict:
    return {
      "timestamp": self.timestamp,
      "joints": self.joints,
      "pose": self.pose,
      "obstacles": [
        {
          "name": o.name,
          "type": o.kind,
          "radius_mm": o.radius_mm,
          "velocity_mms": o.velocity_mms,
          "center": o.center,
          "p1": o.p1,
          "p2": o.p2,
        }
        for o in self.world.obstacles
      ],
      "rgb": self.rgb,
      "depth": self.depth,
      "pointcloud": self.pointcloud,
      "skin_conductance": self.skin_conductance,
      "skin_approach": self.skin_approach,
    }


@dataclass
class WorldState:
  obstacles: List[Obstacle] = field(default_factory=list)
  timestamp: float = 0.0
  margin_mm: float = 40.0

  def at(self, t: float) -> "WorldState":
    return WorldState(list(self.obstacles), timestamp=t, margin_mm=self.margin_mm)

  def primitives(self, t: float):
    return [obs.primitive_at(t) for obs in self.obstacles]

  def collides_tcp(self, xyz: Sequence[float], t: float, radius: float = 30.0) -> bool:
    body = Sphere(np.asarray(xyz, dtype=float), radius)
    return any(collide_any(body, prim, self.margin_mm) for prim in self.primitives(t))

  def collides_arm(self, kin: Kinematics, joints: Sequence[float], t: float) -> bool:
    for cap in kin.link_capsules(joints):
      for prim in self.primitives(t):
        if collide_any(cap, prim, self.margin_mm):
          return True
    return False
