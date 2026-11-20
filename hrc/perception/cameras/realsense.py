"""Intel RealSense RGB-D backend (optional pyrealsense2)."""

from __future__ import annotations

from typing import Optional

import numpy as np
import cv2
from .base import BaseCamera, CameraFrame, _cfg_get


class RealSenseCamera(BaseCamera):
  """Color + aligned depth in metres. Live intrinsics overwrite YAML placeholders."""

  driver = "realsense"
  has_depth = True

  def __init__(self, cfg=None, name=None):
    super().__init__(cfg, name=name or "realsense")
    section = self.cfg
    self.device = "realsense"
    self.serial = str(_cfg_get(section, "serial", "") or "")
    self.width = int(_cfg_get(section, "width", 640))
    self.height = int(_cfg_get(section, "height", 480))
    self.fps = float(_cfg_get(section, "fps", 30))
    self.depth_width = int(_cfg_get(section, "depth_width", self.width))
    self.depth_height = int(_cfg_get(section, "depth_height", self.height))
    self.align_to_color = bool(_cfg_get(section, "align_to_color", True))
    self.depth_scale = float(_cfg_get(section, "depth_scale", 0.001))
    self.fx = float(_cfg_get(section, "fx", 615.0))
    self.fy = float(_cfg_get(section, "fy", 615.0))
    self.cx = float(_cfg_get(section, "cx", self.width / 2.0))
    self.cy = float(_cfg_get(section, "cy", self.height / 2.0))
    self.actual_size = (self.width, self.height)
    self.actual_fps = self.fps
    self._pipeline = None
    self._align = None
    self._rs = None

  @property
  def is_open(self) -> bool:
    return self._pipeline is not None

  def open(self) -> "RealSenseCamera":
    if not self.enabled:
      raise RuntimeError("RealSense camera disabled in config/robot.yaml")
    if self.is_open:
      return self
    try:
      import pyrealsense2 as rs
    except ImportError as exc:
      raise RuntimeError(
        "Intel RealSense camera requires pyrealsense2; pip install pyrealsense2"
      ) from exc
    pipeline = rs.pipeline()
    config = rs.config()
    if self.serial:
      config.enable_device(self.serial)
    fps = int(self.fps)
    config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, fps)
    config.enable_stream(
      rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, fps
    )
    profile = pipeline.start(config)
    self._pipeline = pipeline
    self._rs = rs
    self._align = rs.align(rs.stream.color) if self.align_to_color else None
    try:
      device = profile.get_device()
      self.serial = str(device.get_info(rs.camera_info.serial_number))
      depth_sensor = device.first_depth_sensor()
      self.depth_scale = float(depth_sensor.get_depth_scale())
    except Exception:
      pass
    try:
      stream = profile.get_stream(rs.stream.color)
      if hasattr(stream, "as_video_stream_profile"):
        stream = stream.as_video_stream_profile()
      intr = stream.get_intrinsics()
      self.fx = float(intr.fx)
      self.fy = float(intr.fy)
      self.cx = float(intr.ppx)
      self.cy = float(intr.ppy)
      self.width = int(intr.width)
      self.height = int(intr.height)
      self.actual_size = (self.width, self.height)
    except Exception:
      pass
    self.last_error = None
    frame = self._grab()
    if frame is not None:
      self._publish(frame)
    return self

  def _grab(self, bgr: bool = False) -> Optional[CameraFrame]:
    if self._pipeline is None:
      return None
    try:
      frames = self._pipeline.wait_for_frames(1000)
    except Exception:
      return None
    if self._align is not None:
      try:
        frames = self._align.process(frames)
      except Exception:
        pass
    color = frames.get_color_frame()
    depth = frames.get_depth_frame()

    if not color or not depth:
      return None

    # RealSense stream is configured as BGR8.
    # Convert to RGB for the unified CameraFrame interface.
    rgb = np.asanyarray(color.get_data())
    rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    depth_m = np.asanyarray(depth.get_data()).astype(np.float32)
    depth_m *= self.depth_scale

    return self._make_frame(rgb, depth=depth_m, device=self.serial or "realsense")

  def _release(self) -> None:
    pipe = self._pipeline
    self._pipeline = None
    self._align = None
    if pipe is not None:
      try:
        pipe.stop()
      except Exception:
        pass

  def _as_dict_extra(self) -> dict:
    return {
      "align_to_color": self.align_to_color,
      "depth_scale": self.depth_scale,
      "depth_size": [self.depth_width, self.depth_height],
    }
