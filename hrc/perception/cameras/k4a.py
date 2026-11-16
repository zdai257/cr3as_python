"""Azure Kinect DK RGB-D backend (optional pyk4a)."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .base import BaseCamera, CameraFrame, _cfg_get


_COLOR_RES = {
  "720p": "RES_720P",
  "1080p": "RES_1080P",
  "1440p": "RES_1440P",
  "1536p": "RES_1536P",
  "2160p": "RES_2160P",
  "3072p": "RES_3072P",
  "off": "OFF",
}

_FPS = {5: "FPS_5", 15: "FPS_15", 30: "FPS_30"}


def _enum(owner, name, fallback=None):
  if owner is None:
    return fallback
  return getattr(owner, name, fallback)


def _k4a_color_intrinsics(device):
  cal = getattr(device, "calibration", None)
  if cal is None:
    return None
  mat = None
  if hasattr(cal, "get_camera_matrix"):
    try:
      import pyk4a

      ctype = getattr(pyk4a, "CalibrationType", None)
      color = _enum(ctype, "COLOR", 1)
      mat = cal.get_camera_matrix(color)
    except Exception:
      try:
        mat = cal.get_camera_matrix(1)
      except Exception:
        mat = None
  if mat is None:
    for attr in ("color_camera_matrix", "rgb_camera_matrix"):
      if hasattr(cal, attr):
        mat = getattr(cal, attr)
        break
  if mat is not None:
    arr = np.asarray(mat, dtype=float)
    if arr.shape == (3, 3):
      return float(arr[0, 0]), float(arr[1, 1]), float(arr[0, 2]), float(arr[1, 2])
  params = getattr(cal, "color_params", None)
  if params is not None and getattr(params, "fx", None) is not None:
    return float(params.fx), float(params.fy), float(params.cx), float(params.cy)
  return None


def _to_rgb(img, bgr: bool = False):
  arr = np.asarray(img)
  if arr.ndim < 3:
    return arr
  if bgr:
    return arr[:, :, :3] if arr.shape[2] >= 3 else arr
  if arr.shape[2] == 4:
    return arr[:, :, 2::-1].copy()
  if arr.shape[2] == 3:
    return arr[:, :, ::-1].copy()
  return arr


class KinectCamera(BaseCamera):
  """Color + transformed depth in metres when aligned=true."""

  driver = "k4a"
  has_depth = True

  def __init__(self, cfg=None, name=None):
    super().__init__(cfg, name=name or "k4a")
    section = self.cfg
    self.device = "k4a"
    self.device_index = int(_cfg_get(section, "device_index", 0))
    self.color_resolution = str(_cfg_get(section, "color_resolution", "720p"))
    self.depth_mode = str(_cfg_get(section, "depth_mode", "NFOV_UNBINNED"))
    self.aligned = bool(_cfg_get(section, "aligned", True))
    self.depth_scale = float(_cfg_get(section, "depth_scale", 0.001))
    self.width = int(_cfg_get(section, "width", 1280))
    self.height = int(_cfg_get(section, "height", 720))
    self.fps = float(_cfg_get(section, "fps", 30))
    self.fx = float(_cfg_get(section, "fx", 610.0))
    self.fy = float(_cfg_get(section, "fy", 610.0))
    self.cx = float(_cfg_get(section, "cx", self.width / 2.0))
    self.cy = float(_cfg_get(section, "cy", self.height / 2.0))
    self.actual_size = (self.width, self.height)
    self.actual_fps = self.fps
    self._k4a = None

  @property
  def is_open(self) -> bool:
    return self._k4a is not None

  def open(self) -> "KinectCamera":
    if not self.enabled:
      raise RuntimeError("Azure Kinect camera disabled in config/robot.yaml")
    if self.is_open:
      return self
    try:
      import pyk4a
      from pyk4a import Config, PyK4A
    except ImportError as exc:
      raise RuntimeError(
        "Azure Kinect camera requires pyk4a (pip install pyk4a) and the Azure Kinect SDK"
      ) from exc
    color_cls = getattr(pyk4a, "ColorResolution", None)
    depth_cls = getattr(pyk4a, "DepthMode", None)
    fps_cls = getattr(pyk4a, "FPS", None)
    color_attr = _COLOR_RES.get(self.color_resolution.lower(), "RES_720P")
    fps_attr = _FPS.get(int(self.fps), "FPS_30")
    kwargs = {
      "color_resolution": _enum(color_cls, color_attr, self.color_resolution),
      "depth_mode": _enum(depth_cls, self.depth_mode, self.depth_mode),
      "camera_fps": _enum(fps_cls, fps_attr, int(self.fps)),
      "synchronized_images_only": True,
    }
    conf = Config(**kwargs)
    try:
      cam = PyK4A(conf, device_id=self.device_index)
    except TypeError:
      cam = PyK4A(conf)
    cam.start()
    self._k4a = cam
    self.serial = str(self.device_index)
    live = _k4a_color_intrinsics(cam)
    if live is not None:
      self.fx, self.fy, self.cx, self.cy = live
    self.last_error = None
    frame = self._grab()
    if frame is not None:
      self._publish(frame)
    return self

  def _grab(self, bgr: bool = False) -> Optional[CameraFrame]:
    if self._k4a is None:
      return None
    try:
      capture = self._k4a.get_capture()
    except Exception:
      return None
    if capture is None or getattr(capture, "color", None) is None:
      return None
    rgb = _to_rgb(capture.color, bgr=bgr)
    depth_u16 = None
    if self.aligned:
      depth_u16 = getattr(capture, "transformed_depth", None)
    if depth_u16 is None:
      depth_u16 = getattr(capture, "depth", None)
    depth_m = None
    if depth_u16 is not None:
      depth_m = np.asarray(depth_u16).astype(np.float32) * float(self.depth_scale)
    self.actual_size = (int(rgb.shape[1]), int(rgb.shape[0]))
    return self._make_frame(rgb, depth=depth_m, device=self.serial or "k4a")

  def _release(self) -> None:
    cam = self._k4a
    self._k4a = None
    if cam is not None:
      try:
        cam.stop()
      except Exception:
        pass

  def _as_dict_extra(self) -> dict:
    return {
      "device_index": self.device_index,
      "color_resolution": self.color_resolution,
      "depth_mode": self.depth_mode,
      "aligned": self.aligned,
      "depth_scale": self.depth_scale,
    }
