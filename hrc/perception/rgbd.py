"""RGB-D camera stub. Implement `observe()` once a RealSense / Orbbec / ROS topic exists."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .base import PerceptionFrontend
from ..world import Observation, WorldState


class RGBDPerception(PerceptionFrontend):
  name = "rgbd"

  def __init__(self, cfg, fallback: Optional[PerceptionFrontend] = None):
    self.cfg = cfg.perception if hasattr(cfg, "perception") else cfg
    self.fallback = fallback
    self.enabled = bool(getattr(self.cfg, "rgbd_enabled", False))

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
        raise RuntimeError("RGB-D disabled and no fallback perception")
      return self.fallback.observe(joints, pose, timestamp, extra)
    raise NotImplementedError(
      "RGBDPerception: connect a camera and fill rgb/depth (and optionally "
      "replace oracle obstacles with a reconstructed occupancy map)."
    )
