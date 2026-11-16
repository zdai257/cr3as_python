"""Config-selected camera backends: UVC, RealSense, Azure Kinect."""

from .base import (
  CAMERA_PROFILES,
  CameraFrame,
  BaseCamera,
  active_camera_name,
  camera_section,
  encode_rgb_jpeg,
  known_camera_names,
  perception_section,
)
from .k4a import KinectCamera
from .realsense import RealSenseCamera
from .uvc import HandEyeCamera, UvcCamera

DRIVERS = {
  "uvc": UvcCamera,
  "realsense": RealSenseCamera,
  "k4a": KinectCamera,
}


def list_drivers():
  return sorted(CAMERA_PROFILES.keys())


def make_camera(cfg, name=None):
  """Build the camera named by `name` or perception.camera (default uvc)."""
  from .base import _cfg_get, _profile_name

  perc = perception_section(cfg)
  raw = name if name is not None else _cfg_get(perc, "camera", "uvc")
  key = str(raw or "uvc").strip().lower()
  if key not in CAMERA_PROFILES:
    known = ", ".join(list_drivers())
    raise ValueError(f"unknown camera driver {raw!r}; known drivers: {known}")
  profile = _profile_name(key)
  cls = DRIVERS[profile]
  return cls(cfg, name=profile)


__all__ = [
  "BaseCamera",
  "CAMERA_PROFILES",
  "CameraFrame",
  "DRIVERS",
  "HandEyeCamera",
  "KinectCamera",
  "RealSenseCamera",
  "UvcCamera",
  "active_camera_name",
  "camera_section",
  "encode_rgb_jpeg",
  "known_camera_names",
  "list_drivers",
  "make_camera",
  "perception_section",
]
