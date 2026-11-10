"""Replay a recorded episode through a LearningPlanner (no motion unless asked)."""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from ..planning.base import Trajectory
from ..planning.learning import LearningPlanner
from ..world import Observation, WorldState
from .dataset import Episode
from .frame import EpisodeFrame


def frame_to_observation(frame: EpisodeFrame, world: Optional[WorldState] = None) -> Observation:
  return Observation(
    timestamp=frame.timestamp,
    joints=np.asarray(frame.joints, dtype=float),
    pose=np.asarray(frame.pose, dtype=float),
    world=world or WorldState(),
    skin_conductance=np.asarray(frame.skin_conductance, dtype=float),
    skin_approach=np.asarray(frame.skin_approach, dtype=float),
    skin_proximity=np.asarray(frame.skin_proximity, dtype=float),
    gripper_position=float(frame.gripper_position),
  )


def replay_policy(episode: Episode, planner: LearningPlanner) -> List[Trajectory]:
  """Run infer() on every frame. Useful for offline policy eval."""
  out: List[Trajectory] = []
  world = WorldState()
  for frame in episode.frames:
    obs = frame_to_observation(frame, world)
    try:
      traj = planner.infer(obs)
    except NotImplementedError:
      traj = planner.plan(frame.pose, frame.pose_target, world, observation=obs)
    out.append(traj)
  return out


def iter_policy_batches(episode: Episode, batch_size: int = 32):
  states = episode.states()
  actions = episode.actions()
  for i in range(0, len(episode), batch_size):
    yield states[i : i + batch_size], actions[i : i + batch_size]
