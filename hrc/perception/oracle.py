"""Ground-truth obstacles from config. Use this until cameras are integrated."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .base import PerceptionFrontend
from ..utils.geometry import Obstacle
from ..world import Observation, WorldState


class OraclePerception(PerceptionFrontend):
  name = "oracle"

  def __init__(self, cfg, skins=None):
    hrc = cfg.hrc if hasattr(cfg, "hrc") else cfg
    self.margin = float(hrc.collision_margin_mm)
    self.obstacles = [Obstacle.from_cfg(item) for item in hrc.oracle_obstacles]
    self.skins = skins

  def observe(
    self,
    joints=None,
    pose=None,
    timestamp: float = 0.0,
    extra: Optional[dict] = None,
  ) -> Observation:
    extra = extra or {}
    conductance = extra.get("skin_conductance")
    approach = extra.get("skin_approach")
    proximity = extra.get("skin_proximity")
    reading = extra.get("skin_reading")
    if self.skins is not None:
      reading = reading or self.skins.read()
      if reading is not None:
        if conductance is None:
          conductance = reading.conductance
        if approach is None:
          approach = reading.approach
        if proximity is None:
          proximity = reading.proximity
    world = WorldState(list(self.obstacles), timestamp=timestamp, margin_mm=self.margin)
    return Observation(
      timestamp=timestamp,
      joints=np.zeros(6) if joints is None else np.asarray(joints, dtype=float),
      pose=np.zeros(6) if pose is None else np.asarray(pose, dtype=float),
      world=world,
      rgb=extra.get("rgb"),
      depth=extra.get("depth"),
      pointcloud=extra.get("pointcloud"),
      skin_conductance=conductance,
      skin_approach=approach,
      skin_proximity=proximity,
      skin_reading=reading,
      gripper_position=extra.get("gripper_position"),
    )
