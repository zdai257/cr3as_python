"""Stub for learned policies and vision-language-action models.

Wire a checkpoint later:

  class MyVLA(LearningPlanner):
    def load(self, path): ...
    def infer(self, observation) -> Trajectory: ...

The Observation.as_policy_obs() dict is the stable input contract
(joints, pose, gripper_position, obstacles, rgb, depth, pointcloud,
skin_approach, skin_proximity, skin_conductance). Record with
hrc.data.EpisodeRecorder (125 Hz npz) and export via write_lerobot_dataset
(30 FPS H.264 Hub parquet+mp4) / write_ros2_bag.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .base import PathPlanner, Trajectory, Waypoint, register
from ..world import Observation, WorldState


@register("learning")
class LearningPlanner(PathPlanner):
  def __init__(self, cfg=None, kinematics=None):
    super().__init__(cfg=cfg, kinematics=kinematics)
    hrc = getattr(cfg, "hrc", cfg) if cfg is not None else None
    self.checkpoint = None
    self.device = "cpu"
    if hrc is not None and hasattr(hrc, "learning"):
      self.checkpoint = hrc.learning.checkpoint
      self.device = str(hrc.learning.device)
    self.policy = None
    if self.checkpoint:
      self.load(self.checkpoint)

  def load(self, path: str) -> None:
    """Load a torch / ONNX / JAX policy. Override in subclasses."""
    raise NotImplementedError(
      f"LearningPlanner.load({path!r}) is a stub. "
      "Subclass and implement load() + infer() for VLA / RL policies."
    )

  def infer(self, observation: Observation) -> Trajectory:
    raise NotImplementedError("implement infer(observation) -> Trajectory")

  def plan(
    self,
    start: Sequence[float],
    goal: Sequence[float],
    world: WorldState,
    start_joints: Optional[Sequence[float]] = None,
    observation: Optional[Observation] = None,
  ) -> Trajectory:
    if observation is None:
      observation = Observation(
        timestamp=world.timestamp,
        joints=np.asarray(start_joints if start_joints is not None else np.zeros(6)),
        pose=np.asarray(start, dtype=float),
        world=world,
      )
    if self.policy is None and not self.checkpoint:
      return Trajectory(
        waypoints=[
          Waypoint(np.asarray(start, dtype=float), t=0.0),
          Waypoint(np.asarray(goal, dtype=float), t=1.0),
        ],
        planner=self.name,
        success=False,
        reason="no policy loaded; set hrc.learning.checkpoint or subclass LearningPlanner",
      )
    return self.infer(observation)
