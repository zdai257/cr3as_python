"""Camera facade: UVC / RealSense / Kinect plus JPEG/H.264 helpers.

Public imports stay here. Drivers live in `hrc.perception.cameras`.
Linux UVC gadgets often expose a metadata node (e.g. /dev/video1) — do not open it.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .base import PerceptionFrontend
from .cameras import (
  CameraFrame,
  HandEyeCamera,
  KinectCamera,
  RealSenseCamera,
  UvcCamera,
  list_drivers,
  make_camera,
)
from .cameras.base import encode_rgb_jpeg
from ..world import Observation, WorldState

__all__ = [
  "CameraFrame",
  "H264_FOURCCS",
  "HandEyeCamera",
  "HandEyePerception",
  "KinectCamera",
  "RealSenseCamera",
  "UvcCamera",
  "encode_rgb_jpeg",
  "list_drivers",
  "make_camera",
  "open_h264_writer",
]


def _latest_or_read(camera):
  if camera is None:
    return None
  frame = camera.latest()
  if frame is None and not getattr(camera, "_running", False):
    frame = camera.read()
  return frame


H264_FOURCCS = ("avc1", "H264", "X264")


def open_h264_writer(path, fps: float, size):
  """OpenCV H.264 writer for Hub mp4. Never MPEG-4 `mp4v`.

  UVC capture is often MJPG; Hub export wants avc1 / yuv420p.
  Prefer ffmpeg libx264 in `hrc.data.lerobot_io`; this is the OpenCV fallback.
  """
  from .cameras.base import _try_cv2

  cv2 = _try_cv2()
  if cv2 is None:
    return None
  w, h = int(size[0]), int(size[1])
  for fourcc in H264_FOURCCS:
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc), float(fps), (w, h))
    if writer.isOpened():
      return writer
    writer.release()
  return None


class HandEyePerception(PerceptionFrontend):
  """Wrist camera RGB (+ depth when the driver provides it) and SafeSkin."""

  name = "handeye"

  def __init__(self, cfg, camera=None, skins=None, fallback=None):
    self.cfg = cfg
    self.camera = camera
    self.skins = skins
    self.fallback = fallback
    hrc = cfg.hrc if hasattr(cfg, "hrc") else None
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
    if self.fallback is not None:
      obs = self.fallback.observe(joints, pose, timestamp, extra)
    else:
      reading = extra.get("skin_reading")
      if reading is None and self.skins is not None:
        reading = self.skins.read()
      obs = Observation(
        timestamp=timestamp,
        joints=np.zeros(6) if joints is None else np.asarray(joints, dtype=float),
        pose=np.zeros(6) if pose is None else np.asarray(pose, dtype=float),
        world=WorldState(timestamp=timestamp, margin_mm=self.margin),
        skin_conductance=None if reading is None else reading.conductance,
        skin_approach=None if reading is None else reading.approach,
        skin_proximity=None if reading is None else reading.proximity,
        skin_reading=reading,
        gripper_position=extra.get("gripper_position"),
        rgb=extra.get("rgb"),
        depth=extra.get("depth"),
      )
    if extra.get("rgb") is not None:
      obs.rgb = extra["rgb"]
    if extra.get("depth") is not None:
      obs.depth = extra["depth"]
    return obs

  def close(self) -> None:
    if self.camera is not None:
      self.camera.close()
