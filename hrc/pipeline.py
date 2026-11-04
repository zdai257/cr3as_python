"""Modular pick-and-place pipeline with a swappable path planner."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from .perception.base import PerceptionFrontend
from .perception.oracle import OraclePerception
from .planning.base import PathPlanner, Trajectory, make_planner
from .robot import CS3ASRobot
from .utils.dashboard import taught
from .world import Observation


STAGES = ("pre_pick", "pick", "post_pick", "pre_place", "place", "post_place")
GRIP_CLOSE = {"pick"}
GRIP_OPEN = {"place"}


@dataclass
class StageResult:
  name: str
  trajectory: Trajectory
  executed: bool


class PickPlacePipeline:
  def __init__(
    self,
    robot: CS3ASRobot,
    planner: Optional[PathPlanner] = None,
    perception: Optional[PerceptionFrontend] = None,
  ):
    self.robot = robot
    self.cfg = robot.cfg
    self.planner = planner or make_planner(
      str(self.cfg.hrc.planner), cfg=self.cfg, kinematics=robot.kin
    )
    self.perception = perception or OraclePerception(self.cfg, skins=robot.skins)

  def waypoints(self) -> Dict[str, Optional[List[float]]]:
    raw = self.cfg.hrc.waypoints.to_dict()
    out = {}
    for name in STAGES:
      pose = raw.get(name)
      out[name] = [float(v) for v in pose] if taught(pose) else None
    return out

  def observe(self) -> Observation:
    try:
      joints = self.robot.current_joints()
      pose = self.robot.current_pose()
    except Exception:
      joints = np.zeros(6)
      pose = np.zeros(6)
    return self.perception.observe(joints=joints, pose=pose, timestamp=time.time())

  def plan_segment(self, start: Sequence[float], goal: Sequence[float]) -> Trajectory:
    obs = self.observe()
    start_joints = None
    try:
      start_joints = self.robot.current_joints()
    except Exception:
      pass
    return self.planner.plan(start, goal, obs.world, start_joints=start_joints, observation=obs)

  def run(self, execute: Optional[bool] = None) -> List[StageResult]:
    execute = bool(self.cfg.hrc.execute if execute is None else execute)
    poses = self.waypoints()
    missing = [name for name, pose in poses.items() if pose is None]
    if missing:
      print(f"[hrc] waypoints still TBC: {missing}")
      print("[hrc] running oracle demo_start -> demo_goal so planners stay testable")
      start = [float(v) for v in self.cfg.hrc.demo_start]
      goal = [float(v) for v in self.cfg.hrc.demo_goal]
      traj = self.plan_segment(start, goal)
      self._log_traj("demo", traj)
      return [StageResult("demo", traj, executed=False)]

    order = ["home"] + list(STAGES) + ["home"]
    named = {"home": [float(v) for v in self.cfg.robot.home_joints]}
    named.update({k: v for k, v in poses.items() if v is not None})

    results: List[StageResult] = []
    current_cart = None
    if execute:
      self.robot.go_home()
      current_cart = self.robot.current_pose()
    else:
      current_cart = np.asarray(self.cfg.hrc.demo_start, dtype=float)

    for name in STAGES:
      goal = poses[name]
      traj = self.plan_segment(current_cart, goal)
      self._log_traj(name, traj)
      did = False
      if execute:
        self.robot.execute_trajectory(traj)
        if name in GRIP_CLOSE:
          self.robot.gripper.close()
        elif name in GRIP_OPEN:
          self.robot.gripper.open()
        current_cart = self.robot.current_pose()
        did = True
      else:
        current_cart = np.asarray(goal, dtype=float)
      results.append(StageResult(name, traj, executed=did))

    if execute:
      self.robot.go_home()
    return results

  def _log_traj(self, name: str, traj: Trajectory) -> None:
    n = len(traj.waypoints)
    flag = "ok" if traj.success else "FAIL"
    print(
      f"[hrc] {name:10s} planner={traj.planner:16s} {flag:4s} "
      f"wps={n:3d} cost={traj.cost:.1f} {traj.reason}"
    )
