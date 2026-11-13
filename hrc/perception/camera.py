"""ELP USB UVC hand-eye camera (06MT-KL170IR class) on /dev/video0.

Linux exposes two V4L2 nodes for one UVC gadget: video0 is capture,
video1 is a metadata node and will not open. Query via
`CS3ASRobot.camera.latest()` / `CS3ASRobot.query().rgb`.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .base import PerceptionFrontend
from ..world import Observation, WorldState


def _cfg_get(cfg, name: str, default):
  if cfg is None:
    return default
  if hasattr(cfg, "get"):
    value = cfg.get(name, default)
    return default if value is None else value
  return getattr(cfg, name, default)


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

  def as_dict(self) -> dict:
    h, w = (int(self.rgb.shape[0]), int(self.rgb.shape[1])) if self.rgb is not None else (0, 0)
    return {
      "timestamp": self.timestamp,
      "device": self.device,
      "width": w or self.width,
      "height": h or self.height,
      "fps": self.fps,
      "channels": 3 if self.rgb is not None and self.rgb.ndim == 3 else 0,
    }


def _handeye_section(cfg):
  if cfg is None:
    return None
  perc = cfg.perception if hasattr(cfg, "perception") else cfg
  nested = None
  if perc is not None:
    nested = perc.get("handeye") if hasattr(perc, "get") else getattr(perc, "handeye", None)
  return nested if nested is not None else perc


def encode_rgb_jpeg(rgb: np.ndarray, quality: int = 85) -> Optional[bytes]:
  cv2 = _try_cv2()
  if cv2 is None or rgb is None:
    return None
  bgr = rgb if rgb.ndim == 2 else cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
  ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
  if not ok:
    return None
  return bytes(buf)


class HandEyeCamera:
  """Non-blocking MJPEG capture for the flange / gripper camera."""

  def __init__(self, cfg=None):
    perc = cfg.perception if cfg is not None and hasattr(cfg, "perception") else cfg
    section = _handeye_section(cfg)
    self.cfg = section
    self.device = str(_cfg_get(section, "device", "/dev/video0"))
    self.metadata_device = str(_cfg_get(section, "metadata_device", "/dev/video1"))
    self.width = int(_cfg_get(section, "width", 1920))
    self.height = int(_cfg_get(section, "height", 1080))
    self.fps = float(_cfg_get(section, "fps", 30))
    self.fourcc = str(_cfg_get(section, "fourcc", "MJPG"))
    self.enabled = bool(_cfg_get(perc, "handeye_enabled", True))
    self.enabled = bool(_cfg_get(section, "enabled", self.enabled))
    self._cap = None
    self._lock = threading.Lock()
    self._latest: Optional[CameraFrame] = None
    self._thread: Optional[threading.Thread] = None
    self._running = False
    self.last_error: Optional[str] = None
    self.actual_size: Tuple[int, int] = (self.width, self.height)
    self.actual_fps = self.fps

  @property
  def is_open(self) -> bool:
    return self._cap is not None and bool(getattr(self._cap, "isOpened", lambda: False)())

  def open(self) -> "HandEyeCamera":
    if not self.enabled:
      raise RuntimeError("hand-eye camera disabled in config/robot.yaml")
    cv2 = _try_cv2()
    if cv2 is None:
      raise RuntimeError("opencv-python-headless is required for the hand-eye camera")
    if self.is_open:
      return self
    cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
    if not cap.isOpened():
      cap = cv2.VideoCapture(self.device)
    if not cap.isOpened():
      raise RuntimeError(
        f"cannot open hand-eye camera {self.device} "
        "(use /dev/video0; /dev/video1 is the UVC metadata node)"
      )
    four = cv2.VideoWriter_fourcc(*self.fourcc)
    cap.set(cv2.CAP_PROP_FOURCC, four)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    cap.set(cv2.CAP_PROP_FPS, self.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = cap.read()
    if not ok or frame is None:
      cap.release()
      raise RuntimeError(f"opened {self.device} but the first grab failed")
    self._cap = cap
    self.actual_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    self.actual_fps = float(cap.get(cv2.CAP_PROP_FPS) or self.fps)
    self.last_error = None
    self._store(frame, cv2)
    return self

  def start(self) -> None:
    if self._running:
      return
    if not self.is_open:
      self.open()
    self._running = True
    self._thread = threading.Thread(target=self._loop, daemon=True, name="handeye-cam")
    self._thread.start()

  def stop(self) -> None:
    self._running = False
    if self._thread is not None:
      self._thread.join(timeout=1.5)
      self._thread = None

  def close(self) -> None:
    self.stop()
    cap = self._cap
    self._cap = None
    if cap is not None:
      try:
        cap.release()
      except Exception:
        pass

  def read(self, bgr: bool = False) -> Optional[CameraFrame]:
    """Grab one frame (or the latest if the grab thread is running)."""
    if self._running:
      return self.latest()
    if not self.is_open:
      return None
    cv2 = _try_cv2()
    ok, frame = self._cap.read()
    if not ok or frame is None:
      return None
    self._store(frame, cv2, bgr=bgr)
    return self.latest()

  def latest(self) -> Optional[CameraFrame]:
    with self._lock:
      return self._latest

  def as_dict(self) -> dict:
    latest = self.latest()
    return {
      "enabled": self.enabled,
      "open": self.is_open,
      "streaming": self._running,
      "device": self.device,
      "requested": [self.width, self.height, self.fps],
      "actual": [self.actual_size[0], self.actual_size[1], self.actual_fps],
      "fourcc": self.fourcc,
      "metadata_device": self.metadata_device,
      "last_error": self.last_error,
      "frame": None if latest is None else latest.as_dict(),
    }

  def jpeg(self, quality: int = 85) -> Optional[bytes]:
    latest = self.latest()
    if latest is None:
      return None
    return encode_rgb_jpeg(latest.rgb, quality=quality)

  def _loop(self) -> None:
    cv2 = _try_cv2()
    while self._running and self._cap is not None:
      ok, frame = self._cap.read()
      if not ok or frame is None:
        time.sleep(0.01)
        continue
      self._store(frame, cv2)

  def _store(self, frame, cv2, bgr: bool = False) -> None:
    rgb = frame if bgr or cv2 is None else cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    sample = CameraFrame(
      timestamp=time.time(),
      rgb=np.asarray(rgb),
      device=self.device,
      width=int(rgb.shape[1]),
      height=int(rgb.shape[0]),
      fps=self.actual_fps,
    )
    with self._lock:
      self._latest = sample


class HandEyePerception(PerceptionFrontend):
  """RGB hand-eye + SafeSkin, with optional oracle obstacles."""

  name = "handeye"

  def __init__(self, cfg, camera: Optional[HandEyeCamera] = None, skins=None, fallback=None):
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
    frame = None
    if self.camera is not None:
      frame = self.camera.latest() or (None if self.camera._running else self.camera.read())
    if frame is not None:
      extra.setdefault("rgb", frame.rgb)
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
    return obs

  def close(self) -> None:
    if self.camera is not None:
      self.camera.close()
