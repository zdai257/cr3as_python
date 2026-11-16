"""Shared camera frame, config helpers, and capture thread."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


CAMERA_PROFILES = {
  "uvc": "uvc",
  "opencv": "uvc",
  "v4l2": "uvc",
  "handeye": "uvc",
  "elp": "uvc",
  "realsense": "realsense",
  "rs": "realsense",
  "d435": "realsense",
  "d435i": "realsense",
  "k4a": "k4a",
  "kinect": "k4a",
  "azure": "k4a",
  "azure_kinect": "k4a",
}


def _cfg_get(cfg, name: str, default):
  if cfg is None:
    return default
  if hasattr(cfg, "get"):
    value = cfg.get(name, default)
    return default if value is None else value
  return getattr(cfg, name, default)


def perception_section(cfg):
  if cfg is None:
    return None
  if hasattr(cfg, "get"):
    nested = cfg.get("perception")
    if nested is not None:
      return nested
  if hasattr(cfg, "perception"):
    try:
      nested = cfg.perception
      if nested is not None:
        return nested
    except AttributeError:
      pass
  return cfg


def _profile_name(name, default: str = "uvc") -> str:
  if name is None or str(name).strip() == "":
    return default
  key = str(name).strip().lower()
  return CAMERA_PROFILES.get(key, key)


def known_camera_names():
  return sorted(CAMERA_PROFILES.keys())


def active_camera_name(cfg) -> str:
  perc = perception_section(cfg)
  return _profile_name(_cfg_get(perc, "camera", "uvc"))


def _looks_like_camera_section(section) -> bool:
  for key in ("device", "driver", "serial", "color_resolution", "depth_mode", "fourcc"):
    if _cfg_get(section, key, None) is not None:
      return True
  return False


def camera_section(cfg, name=None):
  """Return cameras.<profile>, else legacy perception.handeye for UVC."""
  perc = perception_section(cfg)
  profile = _profile_name(name if name is not None else _cfg_get(perc, "camera", "uvc"))
  cameras = _cfg_get(perc, "cameras", None)
  if cameras is not None:
    section = _cfg_get(cameras, profile, None)
    if section is not None:
      return section
    raw = None if name is None else str(name).strip().lower()
    if raw and raw != profile:
      section = _cfg_get(cameras, raw, None)
      if section is not None:
        return section
  if profile == "uvc":
    hand = _cfg_get(perc, "handeye", None)
    if hand is not None:
      return hand
  if perc is not None and _looks_like_camera_section(perc):
    return perc
  return perc


def _try_cv2():
  try:
    import cv2

    return cv2
  except ImportError:
    return None


@dataclass
class CameraFrame:
  timestamp: float
  rgb: np.ndarray
  device: str
  width: int
  height: int
  fps: float = 0.0
  depth: Optional[np.ndarray] = None  # float32 metres, aligned to rgb
  driver: str = "uvc"
  serial: str = ""
  fx: Optional[float] = None
  fy: Optional[float] = None
  cx: Optional[float] = None
  cy: Optional[float] = None
  has_depth: bool = False

  def as_dict(self) -> dict:
    h, w = (int(self.rgb.shape[0]), int(self.rgb.shape[1])) if self.rgb is not None else (0, 0)
    return {
      "timestamp": self.timestamp,
      "device": self.device,
      "driver": self.driver,
      "serial": self.serial,
      "width": w or self.width,
      "height": h or self.height,
      "fps": self.fps,
      "channels": 3 if self.rgb is not None and self.rgb.ndim == 3 else 0,
      "has_depth": bool(self.has_depth or self.depth is not None),
      "fx": self.fx,
      "fy": self.fy,
      "cx": self.cx,
      "cy": self.cy,
    }


def encode_rgb_jpeg(rgb: np.ndarray, quality: int = 85) -> Optional[bytes]:
  cv2 = _try_cv2()
  if cv2 is None or rgb is None:
    return None
  bgr = rgb if rgb.ndim == 2 else cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
  ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
  if not ok:
    return None
  return bytes(buf)


class BaseCamera:
  """Non-blocking capture thread shared by UVC / RealSense / Kinect."""

  driver = "uvc"
  has_depth = False

  def __init__(self, cfg=None, name=None):
    perc = perception_section(cfg)
    section = camera_section(cfg, name)
    self.cfg = section
    self._perc = perc
    self.model = str(_cfg_get(section, "model", self.driver))
    self.width = int(_cfg_get(section, "width", 640))
    self.height = int(_cfg_get(section, "height", 480))
    self.fps = float(_cfg_get(section, "fps", 30))
    self.fx = float(_cfg_get(section, "fx", self.width / 2.0))
    self.fy = float(_cfg_get(section, "fy", self.height / 2.0))
    self.cx = float(_cfg_get(section, "cx", self.width / 2.0))
    self.cy = float(_cfg_get(section, "cy", self.height / 2.0))
    self.serial = str(_cfg_get(section, "serial", "") or "")
    self.enabled = True
    if self.driver == "uvc":
      self.enabled = bool(_cfg_get(perc, "handeye_enabled", True))
    self.enabled = bool(_cfg_get(section, "enabled", self.enabled))
    self._lock = threading.Lock()
    self._latest: Optional[CameraFrame] = None
    self._thread: Optional[threading.Thread] = None
    self._running = False
    self.last_error: Optional[str] = None
    self.actual_size: Tuple[int, int] = (self.width, self.height)
    self.actual_fps = self.fps

  @property
  def is_open(self) -> bool:
    return False

  def open(self) -> "BaseCamera":
    raise NotImplementedError

  def _grab(self, bgr: bool = False) -> Optional[CameraFrame]:
    raise NotImplementedError

  def _release(self) -> None:
    return None

  def start(self) -> None:
    if self._running:
      return
    if not self.is_open:
      self.open()
    self._running = True
    self._thread = threading.Thread(target=self._loop, daemon=True, name=f"{self.driver}-cam")
    self._thread.start()

  def stop(self) -> None:
    self._running = False
    if self._thread is not None:
      self._thread.join(timeout=1.5)
      self._thread = None

  def close(self) -> None:
    self.stop()
    self._release()

  def read(self, bgr: bool = False) -> Optional[CameraFrame]:
    if self._running:
      return self.latest()
    if not self.is_open:
      return None
    frame = self._grab(bgr=bgr)
    if frame is not None:
      with self._lock:
        self._latest = frame
    return self.latest()

  def latest(self) -> Optional[CameraFrame]:
    with self._lock:
      return self._latest

  def jpeg(self, quality: int = 85) -> Optional[bytes]:
    latest = self.latest()
    if latest is None:
      return None
    return encode_rgb_jpeg(latest.rgb, quality=quality)

  def as_dict(self) -> dict:
    latest = self.latest()
    info = {
      "enabled": self.enabled,
      "open": self.is_open,
      "streaming": self._running,
      "driver": self.driver,
      "model": self.model,
      "device": str(getattr(self, "device", self.serial or self.driver)),
      "serial": self.serial,
      "requested": [self.width, self.height, self.fps],
      "actual": [self.actual_size[0], self.actual_size[1], self.actual_fps],
      "has_depth": bool(self.has_depth),
      "fx": self.fx,
      "fy": self.fy,
      "cx": self.cx,
      "cy": self.cy,
      "last_error": self.last_error,
      "frame": None if latest is None else latest.as_dict(),
    }
    info.update(self._as_dict_extra())
    return info

  def _as_dict_extra(self) -> dict:
    return {}

  def _loop(self) -> None:
    while self._running:
      frame = self._grab()
      if frame is None:
        time.sleep(0.01)
        continue
      with self._lock:
        self._latest = frame

  def _publish(self, frame: CameraFrame) -> CameraFrame:
    with self._lock:
      self._latest = frame
    return frame

  def _make_frame(self, rgb, depth=None, device=None) -> CameraFrame:
    rgb_arr = np.asarray(rgb)
    h, w = int(rgb_arr.shape[0]), int(rgb_arr.shape[1])
    depth_arr = None if depth is None else np.asarray(depth, dtype=np.float32)
    return CameraFrame(
      timestamp=time.time(),
      rgb=rgb_arr,
      device=str(device if device is not None else getattr(self, "device", self.driver)),
      width=w,
      height=h,
      fps=self.actual_fps,
      depth=depth_arr,
      driver=self.driver,
      serial=self.serial,
      fx=self.fx,
      fy=self.fy,
      cx=self.cx,
      cy=self.cy,
      has_depth=bool(self.has_depth and depth_arr is not None),
    )
