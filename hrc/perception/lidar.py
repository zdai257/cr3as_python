"""LiDAR stub. Implement `observe()` to publish a point cloud into Observation.pointcloud."""

from __future__ import annotations

from typing import Optional

from .base import PerceptionFrontend
from ..world import Observation


class LidarPerception(PerceptionFrontend):
  name = "lidar"

  def __init__(self, cfg, fallback: Optional[PerceptionFrontend] = None):
    self.cfg = cfg.perception if hasattr(cfg, "perception") else cfg
    self.fallback = fallback
    self.enabled = bool(getattr(self.cfg, "lidar_enabled", False))

  def observe(
    self,
    joints=None,
    pose=None,
    timestamp: float = 0.0,
    extra: Optional[dict] = None,
  ) -> Observation:
    extra = extra or {}
    if not self.enabled:
      if self.fallback is None:
        raise RuntimeError("LiDAR disabled and no fallback perception")
      return self.fallback.observe(joints, pose, timestamp, extra)
    raise NotImplementedError(
      "LidarPerception: stream a point cloud into Observation.pointcloud "
      "and optionally extract dynamic clusters as Obstacle capsules."
    )
