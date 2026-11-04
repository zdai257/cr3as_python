"""RRT* on top of Cartesian RRT: rewire nearby nodes for shorter paths."""

from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np

from .base import PathPlanner, Trajectory, Waypoint, register
from .rrt import RRTPlanner, _Node, _workspace
from ..utils.transforms import interpolate_pose
from ..world import Observation, WorldState


@register("rrt_star")
class RRTStarPlanner(RRTPlanner):
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
    rng = rng or np.random.default_rng(1)
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    nodes: List[_Node] = [_Node(start, -1, 0.0)]
    costs = [0.0]
    radius = float(rrt.step_mm) * 3.0

    def sample() -> np.ndarray:
      if rng.random() < float(rrt.goal_bias):
        return goal.copy()
      pose = start.copy()
      pose[:3] = rng.uniform(bounds[:, 0], bounds[:, 1])
      return pose

    def steer(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
      delta = dst[:3] - src[:3]
      dist = float(np.linalg.norm(delta))
      step = float(rrt.step_mm)
      alpha = 1.0 if dist < 1e-9 else min(1.0, step / dist)
      return interpolate_pose(src, dst, alpha)

    def edge_ok(a_pose: np.ndarray, a_t: float, b_pose: np.ndarray) -> Optional[float]:
      dist = float(np.linalg.norm(b_pose[:3] - a_pose[:3]))
      n_chk = max(2, int(np.ceil(dist / 10.0)))
      t = a_t
      prev = a_pose[:3]
      for i in range(1, n_chk + 1):
        pose = interpolate_pose(a_pose, b_pose, i / n_chk)
        t += float(np.linalg.norm(pose[:3] - prev)) / max(speed, 1e-6)
        prev = pose[:3]
        if world.collides_tcp(pose[:3], t):
          return None
      return t

    reached = None
    for _ in range(int(rrt.max_iter)):
      q_rand = sample()
      dists = [np.linalg.norm(n.pose[:3] - q_rand[:3]) for n in nodes]
      nearest = int(np.argmin(dists))
      q_new = steer(nodes[nearest].pose, q_rand)
      t_new = edge_ok(nodes[nearest].pose, nodes[nearest].t, q_new)
      if t_new is None:
        continue
      near = [
        i
        for i, n in enumerate(nodes)
        if np.linalg.norm(n.pose[:3] - q_new[:3]) <= radius
      ]
      parent = nearest
      best_cost = costs[nearest] + float(np.linalg.norm(q_new[:3] - nodes[nearest].pose[:3]))
      best_t = t_new
      for i in near:
        dt = edge_ok(nodes[i].pose, nodes[i].t, q_new)
        if dt is None:
          continue
        cand = costs[i] + float(np.linalg.norm(q_new[:3] - nodes[i].pose[:3]))
        if cand < best_cost:
          best_cost, parent, best_t = cand, i, dt
      nodes.append(_Node(q_new.copy(), parent, best_t))
      costs.append(best_cost)
      new_idx = len(nodes) - 1
      for i in near:
        dt = edge_ok(q_new, best_t, nodes[i].pose)
        if dt is None:
          continue
        cand = best_cost + float(np.linalg.norm(nodes[i].pose[:3] - q_new[:3]))
        if cand < costs[i]:
          nodes[i].parent = new_idx
          nodes[i].t = dt
          costs[i] = cand
      if np.linalg.norm(q_new[:3] - goal[:3]) <= float(rrt.goal_tolerance_mm):
        t_goal = edge_ok(q_new, best_t, goal)
        if t_goal is not None:
          nodes.append(_Node(goal.copy(), new_idx, t_goal))
          reached = len(nodes) - 1
          break

    if reached is None:
      return Trajectory(planner=self.name, success=False, reason="RRT* failed to reach goal")
    chain = []
    idx = reached
    while idx >= 0:
      chain.append(nodes[idx])
      idx = nodes[idx].parent
    chain.reverse()
    wps = [Waypoint(n.pose.copy(), space="cartesian", interp="movl", t=n.t) for n in chain]
    cost = sum(
      float(np.linalg.norm(wps[i].vector[:3] - wps[i - 1].vector[:3]))
      for i in range(1, len(wps))
    )
    return Trajectory(wps, planner=self.name, success=True, cost=cost)
