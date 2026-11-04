"""Planner interface. New algorithms only need to implement `plan()`."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from ..world import Observation, WorldState


@dataclass
class Waypoint:
  vector: np.ndarray
  space: str = "cartesian" # cartesian | joint
  interp: str = "movl"   # movl | movj
  t: float = 0.0
  speed: int = -1      # 1-100, -1 = global


@dataclass
class Trajectory:
  waypoints: List[Waypoint] = field(default_factory=list)
  planner: str = ""
  success: bool = False
  reason: str = ""
  cost: float = 0.0

  def xyz(self) -> np.ndarray:
    if not self.waypoints:
      return np.zeros((0, 3))
    return np.vstack([wp.vector[:3] for wp in self.waypoints])


class PathPlanner(ABC):
  name: str = "base"

  def __init__(self, cfg=None, kinematics=None):
    self.cfg = cfg
    self.kinematics = kinematics

  @abstractmethod
  def plan(
    self,
    start: Sequence[float],
    goal: Sequence[float],
    world: WorldState,
    start_joints: Optional[Sequence[float]] = None,
    observation: Optional[Observation] = None,
  ) -> Trajectory:
    """Return a collision-aware trajectory from start to goal.

    `start` / `goal` are Cartesian poses [x,y,z,rx,ry,rz] in mm/deg unless
    a joint-space planner documents otherwise.
    """

  def replan(self, *args, **kwargs) -> Trajectory:
    return self.plan(*args, **kwargs)


PLANNER_REGISTRY: Dict[str, Callable] = {}


def register(name: str):
  def deco(cls):
    cls.name = name
    PLANNER_REGISTRY[name] = cls
    return cls

  return deco


def make_planner(name: str, cfg=None, kinematics=None) -> PathPlanner:
  if name not in PLANNER_REGISTRY:
    known = ", ".join(sorted(PLANNER_REGISTRY))
    raise KeyError(f"unknown planner {name!r}. registered: {known}")
  return PLANNER_REGISTRY[name](cfg=cfg, kinematics=kinematics)
