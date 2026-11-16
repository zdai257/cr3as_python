"""UVC / OpenCV webcam backend (any RGB device; ELP is one example)."""

from __future__ import annotations

from typing import Optional

from .base import BaseCamera, _cfg_get, _try_cv2, camera_section, perception_section


def _uvc_source(device):
  if device is None:
    return "/dev/video0"
  if isinstance(device, bool):
    return str(device)
  if isinstance(device, int):
    return device
  text = str(device).strip()
  if text.isdigit():
    return int(text)
  return text


class UvcCamera(BaseCamera):
  """MJPEG/YUYV capture via OpenCV. RGB only — no synthetic depth here."""

  driver = "uvc"
  has_depth = False

  def __init__(self, cfg=None, name=None):
    section = camera_section(cfg, name or "uvc")
    perc = perception_section(cfg)
    super().__init__(cfg, name=name or "uvc")
    self.cfg = section
    self.device = _cfg_get(section, "device", "/dev/video0")
    if not isinstance(self.device, int):
      self.device = str(self.device)
    self.metadata_device = str(_cfg_get(section, "metadata_device", "/dev/video1"))
    self.width = int(_cfg_get(section, "width", 1920))
    self.height = int(_cfg_get(section, "height", 1080))
    self.fps = float(_cfg_get(section, "fps", 30))
    self.fourcc = str(_cfg_get(section, "fourcc", "MJPG"))
    self.fx = float(_cfg_get(section, "fx", 800.0))
    self.fy = float(_cfg_get(section, "fy", 800.0))
    self.cx = float(_cfg_get(section, "cx", self.width / 2.0))
    self.cy = float(_cfg_get(section, "cy", self.height / 2.0))
    self.enabled = bool(_cfg_get(perc, "handeye_enabled", True))
    self.enabled = bool(_cfg_get(section, "enabled", self.enabled))
    self.actual_size = (self.width, self.height)
    self.actual_fps = self.fps
    self._cap = None

  @property
  def is_open(self) -> bool:
    return self._cap is not None and bool(getattr(self._cap, "isOpened", lambda: False)())

  def open(self) -> "UvcCamera":
    if not self.enabled:
      raise RuntimeError("UVC camera disabled in config/robot.yaml")
    cv2 = _try_cv2()
    if cv2 is None:
      raise RuntimeError("opencv-python-headless is required for the UVC camera")
    if self.is_open:
      return self
    source = _uvc_source(self.device)
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
      cap.release()
      cap = cv2.VideoCapture(source)
    if not cap.isOpened():
      raise RuntimeError(
        f"cannot open UVC camera {self.device}"
        + (" (that path is often a metadata node, not capture)" if "video1" in str(self.device) else "")
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
    self._publish(self._frame_from_bgr(frame, cv2))
    return self

  def _grab(self, bgr: bool = False) -> Optional[object]:
    if self._cap is None:
      return None
    cv2 = _try_cv2()
    ok, frame = self._cap.read()
    if not ok or frame is None:
      return None
    if bgr or cv2 is None:
      rgb = frame
    else:
      rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return self._make_frame(rgb, device=self.device)

  def _frame_from_bgr(self, frame, cv2):
    rgb = frame if cv2 is None else cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return self._make_frame(rgb, device=self.device)

  def _release(self) -> None:
    cap = self._cap
    self._cap = None
    if cap is not None:
      try:
        cap.release()
      except Exception:
        pass

  def _as_dict_extra(self) -> dict:
    return {
      "fourcc": self.fourcc,
      "metadata_device": self.metadata_device,
    }


HandEyeCamera = UvcCamera
