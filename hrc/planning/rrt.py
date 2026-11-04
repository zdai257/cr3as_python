"""Cartesian RRT around moving capsules/spheres. Orientation is slerped to the goal."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from .base import PathPlanner, Trajectory, Waypoint, register
from ..utils.transforms import interpolate_pose
from ..world import Observation, WorldState


@dataclass
class _Node:
  pose: np.ndarray
  parent: int = -1
  t: float = 0.0


def _workspace(cfg) -> np.ndarray:
  hrc = getattr(cfg, "hrc", cfg)
  ws = hrc.workspace_mm
  return np.array(
    [
      [float(ws.x[0]), float(ws.x[1])],
      [float(ws.y[0]), float(ws.y[1])],
      [float(ws.z[0]), float(ws.z[1])],
    ]
  )


@register("rrt")
class RRTPlanner(PathPlanner):
  def plan(
    self,
    start: Sequence[float],
    goal: Sequence[float],
    world: WorldState,
    start_joints: Optional[Sequence[float]] = None,
    observation: Optional[Observation] = None,
    rng: Optional[np.random.Generator] = None,
  ) -> Trajectory:
    hrc = getattr(self.cfg, "hrc", self.cfg)
    rrt = hrc.rrt
    speed = float(hrc.tcp_speed_mms)
    bounds = _workspace(self.cfg)
    rng = rng or np.random.default_rng(0)
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    nodes: List[_Node] = [_Node(start, -1, 0.0)]

    def sample() -> np.ndarray:
      if rng.random() < float(rrt.goal_bias):
        return goal.copy()
      xyz = rng.uniform(bounds[:, 0], bounds[:, 1])
      pose = start.copy()
      pose[:3] = xyz
      return pose

    def nearest(q: np.ndarray) -> int:
      d = [np.linalg.norm(n.pose[:3] - q[:3]) for n in nodes]
      return int(np.argmin(d))

    def steer(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
      delta = dst[:3] - src[:3]
      dist = float(np.linalg.norm(delta))
      step = float(rrt.step_mm)
      if dist < 1e-9:
        return src.copy()
      alpha = min(1.0, step / dist)
      return interpolate_pose(src, dst, alpha)

    def edge_clear(parent_idx: int, b_pose: np.ndarray) -> Optional[_Node]:
      a = nodes[parent_idx]
      dist = float(np.linalg.norm(b_pose[:3] - a.pose[:3]))
      n_chk = max(2, int(np.ceil(dist / 10.0)))
      t = a.t
      prev = a.pose[:3]
      for i in range(1, n_chk + 1):
        pose = interpolate_pose(a.pose, b_pose, i / n_chk)
        t += float(np.linalg.norm(pose[:3] - prev)) / max(speed, 1e-6)
        prev = pose[:3]
        if world.collides_tcp(pose[:3], t):
          return None
      return _Node(b_pose.copy(), parent_idx, t)

    for _ in range(int(rrt.max_iter)):
      q_rand = sample()
      idx = nearest(q_rand)
      q_new_pose = steer(nodes[idx].pose, q_rand)
      node = edge_clear(idx, q_new_pose)
      if node is None:
        continue
      nodes.append(node)
      new_idx = len(nodes) - 1
      if np.linalg.norm(node.pose[:3] - goal[:3]) <= float(rrt.goal_tolerance_mm):
        goal_node = edge_clear(new_idx, goal)
        if goal_node is not None:
          nodes.append(goal_node)
          return self._reconstruct(nodes, speed)
    return Trajectory(planner=self.name, success=False, reason="RRT failed to reach goal")

  def _reconstruct(self, nodes: List[_Node], speed: float) -> Trajectory:
    idx = len(nodes) - 1
    chain = []
    while idx >= 0:
      chain.append(nodes[idx])
      idx = nodes[idx].parent
    chain.reverse()
    wps = [
      Waypoint(n.pose.copy(), space="cartesian", interp="movl", t=n.t) for n in chain
    ]
    cost = sum(
      float(np.linalg.norm(wps[i].vector[:3] - wps[i - 1].vector[:3]))
      for i in range(1, len(wps))
    )
    return Trajectory(wps, planner=self.name, success=True, cost=cost)
