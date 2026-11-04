"""Artificial potential field in Cartesian xyz; orientation slerps to the goal."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .base import PathPlanner, Trajectory, Waypoint, register
from ..utils.transforms import interpolate_pose
from ..world import Observation, WorldState


@register("apf")
class ArtificialPotentialFieldPlanner(PathPlanner):
  def plan(
    self,
    start: Sequence[float],
    goal: Sequence[float],
    world: WorldState,
    start_joints: Optional[Sequence[float]] = None,
    observation: Optional[Observation] = None,
  ) -> Trajectory:
    hrc = getattr(self.cfg, "hrc", self.cfg)
    apf = hrc.apf
    speed = float(hrc.tcp_speed_mms)
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    pos = start[:3].copy()
    wps = [Waypoint(start.copy(), space="cartesian", interp="movl", t=0.0)]
    t = 0.0
    k_att = float(apf.attract_gain)
    k_rep = float(apf.repulse_gain)
    rho0 = float(apf.influence_mm)
    step = float(apf.step_mm)

    for _ in range(int(apf.max_iter)):
      to_goal = goal[:3] - pos
      dist_g = float(np.linalg.norm(to_goal))
      if dist_g < 15.0:
        t += dist_g / max(speed, 1e-6)
        wps.append(Waypoint(goal.copy(), space="cartesian", interp="movl", t=t))
        return Trajectory(wps, planner=self.name, success=True, cost=t * speed)
      f = k_att * to_goal / max(dist_g, 1e-6)
      for obs in world.obstacles:
        prim = obs.primitive_at(t)
        if hasattr(prim, "center"):
          delta = pos - prim.center
          dist = float(np.linalg.norm(delta)) - prim.radius
        else:
          from ..utils.geometry import point_segment_distance

          dist = point_segment_distance(pos, prim.p1, prim.p2) - prim.radius
          mid = 0.5 * (prim.p1 + prim.p2)
          delta = pos - mid
        if 1e-6 < dist < rho0:
          f += k_rep * (1.0 / dist - 1.0 / rho0) / (dist * dist) * (
            delta / (np.linalg.norm(delta) + 1e-9)
          )
      nrm = float(np.linalg.norm(f))
      if nrm < 1e-9:
        return Trajectory(wps, planner=self.name, success=False, reason="APF local minimum")
      pos = pos + step * f / nrm
      t += step / max(speed, 1e-6)
      alpha = min(1.0, t * speed / max(float(np.linalg.norm(goal[:3] - start[:3])), 1.0))
      pose = interpolate_pose(start, goal, alpha)
      pose[:3] = pos
      if world.collides_tcp(pos, t):
        return Trajectory(wps, planner=self.name, success=False, reason="APF collided")
      wps.append(Waypoint(pose, space="cartesian", interp="movl", t=t))
    return Trajectory(wps, planner=self.name, success=False, reason="APF hit iteration limit")
