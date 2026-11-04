"""Joint-space linear interpolation. Uses local DH IK when no seed joints."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .base import PathPlanner, Trajectory, Waypoint, register
from ..world import Observation, WorldState


@register("joint_linear")
class JointLinearPlanner(PathPlanner):
  def plan(
    self,
    start: Sequence[float],
    goal: Sequence[float],
    world: WorldState,
    start_joints: Optional[Sequence[float]] = None,
    observation: Optional[Observation] = None,
  ) -> Trajectory:
    if self.kinematics is None:
      return Trajectory(planner=self.name, success=False, reason="joint_linear needs kinematics")
    q0 = np.asarray(start_joints if start_joints is not None else np.zeros(6), dtype=float)
    q_start, ok_s = self.kinematics.ik(start, seed=q0)
    q_goal, ok_g = self.kinematics.ik(goal, seed=q_start)
    if not (ok_s and ok_g):
      return Trajectory(planner=self.name, success=False, reason="IK failed")
    n = 12
    wps = []
    for i in range(n):
      a = i / (n - 1)
      q = q_start + a * (q_goal - q_start)
      t = a * 1.0
      if world.collides_arm(self.kinematics, q, t):
        return Trajectory(wps, planner=self.name, success=False, reason="joint path hits obstacle")
      wps.append(Waypoint(q.copy(), space="joint", interp="movj", t=t))
    return Trajectory(wps, planner=self.name, success=True)
