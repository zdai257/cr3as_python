"""RGB-D perception from the configured camera (RealSense / Kinect / UVC)."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .base import PerceptionFrontend
from .cameras import make_camera
from .cameras.base import _cfg_get, perception_section
from ..world import Observation, WorldState


def _latest_or_read(camera):
  if camera is None:
    return None
  frame = camera.latest()
  if frame is None and not getattr(camera, "_running", False):
    frame = camera.read()
  return frame


class RGBDPerception(PerceptionFrontend):
  name = "rgbd"

  def __init__(self, cfg, camera=None, fallback: Optional[PerceptionFrontend] = None, skins=None):
    self.cfg = cfg
    perc = perception_section(cfg)
    self.camera = camera if camera is not None else make_camera(cfg)
    self.fallback = fallback
    self.skins = skins
    self.enabled = bool(_cfg_get(perc, "rgbd_enabled", False)) or bool(
      getattr(self.camera, "has_depth", False)
    )
    hrc = cfg.hrc if cfg is not None and hasattr(cfg, "hrc") else None
    self.margin = float(hrc.collision_margin_mm) if hrc is not None else 40.0

  def observe(
    self,
    joints=None,
    pose=None,
    timestamp: float = 0.0,
    extra: Optional[dict] = None,
  ) -> Observation:
    extra = dict(extra or {})
    frame = _latest_or_read(self.camera)
    if frame is not None:
      extra.setdefault("rgb", frame.rgb)
      if getattr(frame, "depth", None) is not None:
        extra.setdefault("depth", frame.depth)
      if not timestamp:
        timestamp = frame.timestamp
    has_depth = extra.get("depth") is not None or bool(getattr(self.camera, "has_depth", False))
    if not has_depth and self.fallback is not None:
      obs = self.fallback.observe(joints, pose, timestamp, extra)
      if extra.get("rgb") is not None:
        obs.rgb = extra["rgb"]
      return obs
    reading = extra.get("skin_reading")
    if reading is None and self.skins is not None:
      reading = self.skins.read()
    world = extra.get("world")
    if world is None and self.fallback is not None:
      fb = self.fallback.observe(joints, pose, timestamp, extra)
      world = fb.world
      reading = reading or fb.skin_reading
    if world is None:
      world = WorldState(timestamp=timestamp, margin_mm=self.margin)
    return Observation(
      timestamp=timestamp,
      joints=np.zeros(6) if joints is None else np.asarray(joints, dtype=float),
      pose=np.zeros(6) if pose is None else np.asarray(pose, dtype=float),
      world=world,
      rgb=extra.get("rgb"),
      depth=extra.get("depth"),
      pointcloud=extra.get("pointcloud"),
      skin_conductance=None if reading is None else getattr(reading, "conductance", extra.get("skin_conductance")),
      skin_approach=None if reading is None else getattr(reading, "approach", extra.get("skin_approach")),
      skin_proximity=None if reading is None else getattr(reading, "proximity", extra.get("skin_proximity")),
      skin_reading=reading,
      gripper_position=extra.get("gripper_position"),
    )
