"""Baseline: straight Cartesian line, no avoidance (control condition)."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .base import PathPlanner, Trajectory, Waypoint, register
from ..utils.transforms import interpolate_pose
from ..world import Observation, WorldState


@register("cartesian_linear")
class CartesianLinearPlanner(PathPlanner):
  def plan(
    self,
    start: Sequence[float],
    goal: Sequence[float],
    world: WorldState,
    start_joints: Optional[Sequence[float]] = None,
    observation: Optional[Observation] = None,
  ) -> Trajectory:
    hrc = getattr(self.cfg, "hrc", self.cfg)
    speed = float(hrc.tcp_speed_mms) if hasattr(hrc, "tcp_speed_mms") else 80.0
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    dist = float(np.linalg.norm(goal[:3] - start[:3]))
    n = max(2, int(np.ceil(dist / 20.0)) + 1)
    wps = []
    t = 0.0
    prev = start[:3]
    for i in range(n):
      alpha = i / (n - 1)
      pose = interpolate_pose(start, goal, alpha)
      t += float(np.linalg.norm(pose[:3] - prev)) / max(speed, 1e-6)
      prev = pose[:3]
      hit = world.collides_tcp(pose[:3], t)
      wps.append(Waypoint(pose, space="cartesian", interp="movl", t=t))
      if hit and i not in (0, n - 1):
        return Trajectory(wps, planner=self.name, success=False, reason="linear path hits obstacle")
    return Trajectory(wps, planner=self.name, success=True, cost=dist)
