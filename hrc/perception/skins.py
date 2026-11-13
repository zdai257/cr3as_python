"""CS3AS SafeSkin readings from Feedback port 30004.

Documented 1440-byte fields (TCP/IP V4.6):

  ArmApproachState  offset 1039  I8  forearm proximity-pause
  J4ApproachState   offset 1040  I8  J4 proximity-pause
  J5ApproachState   offset 1041  I8  J5 proximity-pause
  J6ApproachState   offset 1042  I8  J6 proximity-pause

These I8 values are the live skin modality at 8 ms. They are typically 0/1
pause flags; some controllers emit a non-binary intensity, so they are kept
as signed integers.

The following 61 reserved bytes (offset 1043) are decoded as analog
conductance / proximity channels when non-zero. Layout is configurable
because Dobot does not document analog skin ADC in the public packet map.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import numpy as np

from dobot_sdk.models.status import RobotStatus

SKIN_PARTS: Tuple[str, ...] = ("forearm", "j4", "j5", "j6")
SENSE_RANGE_MM = 150.0


def _cfg_get(cfg, name: str, default):
  if cfg is None:
    return default
  if hasattr(cfg, "get"):
    value = cfg.get(name, default)
    return default if value is None else value
  return getattr(cfg, name, default)


def decode_analog_raw(raw: Sequence[int], layout: str, channels: int) -> np.ndarray:
  """Decode reserved Feedback bytes into analog skin channels."""
  buf = np.asarray(raw, dtype=np.uint8).reshape(-1)
  n = max(1, int(channels))
  layout = str(layout or "uint16_le").lower()
  if buf.size == 0:
    return np.zeros(n, dtype=float)
  if layout in ("uint8", "u8"):
    return buf[:n].astype(float)
  if layout in ("int8", "i8"):
    return buf[:n].view(np.int8).astype(float)
  view = buf.tobytes()
  if layout in ("uint16_le", "u16", "uint16"):
    count = min(n, len(view) // 2)
    values = np.frombuffer(view, dtype="<u2", count=count).astype(float)
  elif layout in ("int16_le", "i16", "int16"):
    count = min(n, len(view) // 2)
    values = np.frombuffer(view, dtype="<i2", count=count).astype(float)
  elif layout in ("float32_le", "f32"):
    count = min(n, len(view) // 4)
    values = np.frombuffer(view, dtype="<f4", count=count).astype(float)
  elif layout in ("float64_le", "f64"):
    count = min(n, len(view) // 8)
    values = np.frombuffer(view, dtype="<f8", count=count).astype(float)
  else:
    count = min(n, len(view) // 2)
    values = np.frombuffer(view, dtype="<u2", count=count).astype(float)
  if values.size < n:
    values = np.pad(values, (0, n - values.size))
  return values[:n]


def approach_to_proximity(approach: np.ndarray) -> np.ndarray:
  """Map I8 approach flags / intensity to a 0-1 proximity signal."""
  a = np.asarray(approach, dtype=float).reshape(-1)
  peak = float(np.max(np.abs(a))) if a.size else 0.0
  if peak > 1.0:
    scale = 100.0 if peak > 10.0 else peak
    return np.clip(np.abs(a) / scale, 0.0, 1.0)
  return (np.abs(a) > 0).astype(float)


@dataclass
class SkinReading:
  """One SafeSkin sample from port 30004."""

  timestamp_ms: int = 0
  parts: Tuple[str, ...] = SKIN_PARTS
  approach: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
  analog: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
  analog_raw: np.ndarray = field(default_factory=lambda: np.zeros(61, dtype=np.uint8))
  proximity: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
  conductance: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
  sense_range_mm: float = SENSE_RANGE_MM

  @classmethod
  def from_status(
    cls,
    status: Optional[RobotStatus],
    *,
    analog_layout: str = "uint16_le",
    analog_channels: int = 4,
    analog_scale: float = 1.0,
    analog_offset: float = 0.0,
    sense_range_mm: float = SENSE_RANGE_MM,
  ) -> Optional["SkinReading"]:
    if status is None:
      return None
    approach = np.array(
      [
        int(status.arm_approach_state),
        int(status.j4_approach_state),
        int(status.j5_approach_state),
        int(status.j6_approach_state),
      ],
      dtype=float,
    )
    raw = np.asarray(getattr(status, "skin_analog_raw", np.zeros(61)), dtype=np.uint8)
    analog = decode_analog_raw(raw, analog_layout, analog_channels)
    analog = analog_offset + analog_scale * analog
    analog_prox = np.zeros(len(SKIN_PARTS), dtype=float)
    if analog.size:
      peak = float(np.max(np.abs(analog)))
      if peak > 0:
        analog_prox = np.clip(np.abs(analog[: len(SKIN_PARTS)]) / peak, 0.0, 1.0)
        if analog.size < len(SKIN_PARTS):
          analog_prox = np.pad(analog_prox, (0, len(SKIN_PARTS) - analog.size))
    approach_prox = approach_to_proximity(approach)
    proximity = analog_prox if float(np.max(analog_prox)) > 0 else approach_prox
    conductance = analog[: len(SKIN_PARTS)].copy()
    if conductance.size < len(SKIN_PARTS):
      conductance = np.pad(conductance, (0, len(SKIN_PARTS) - conductance.size))
    if float(np.max(np.abs(conductance))) == 0.0:
      conductance = proximity.copy()
    return cls(
      timestamp_ms=int(getattr(status, "timestamp", 0) or 0),
      approach=approach,
      analog=np.asarray(analog, dtype=float),
      analog_raw=raw.copy(),
      proximity=proximity.astype(float),
      conductance=conductance.astype(float),
      sense_range_mm=float(sense_range_mm),
    )

  def distance_mm(self) -> np.ndarray:
    """Crude range estimate: 0 proximity -> sense_range_mm, 1 -> contact."""
    return (1.0 - np.clip(self.proximity, 0.0, 1.0)) * float(self.sense_range_mm)

  def triggered(self, threshold: float = 0.0) -> bool:
    if np.any(np.abs(self.approach) > 0):
      return True
    if threshold > 0 and np.any(self.proximity >= threshold):
      return True
    return False

  def as_dict(self) -> dict:
    return {
      "timestamp_ms": self.timestamp_ms,
      "parts": list(self.parts),
      "approach": self.approach.tolist(),
      "analog": self.analog.tolist(),
      "proximity": self.proximity.tolist(),
      "conductance": self.conductance.tolist(),
      "distance_mm": self.distance_mm().tolist(),
    }

  def policy_vector(self) -> np.ndarray:
    """Stable float vector for learned policies: approach + proximity."""
    return np.concatenate([self.approach.astype(float), self.proximity.astype(float)])


class SafeSkinFrontend:
  def __init__(self, robot, cfg):
    self.robot = robot
    self.cfg = cfg
    n = int(_cfg_get(cfg, "channels", 4))
    self._conductance = np.zeros(n, dtype=float)
    self._latest: Optional[SkinReading] = None
    self.enabled = bool(_cfg_get(cfg, "enabled", True))
    self.active = False
    self.analog_layout = str(_cfg_get(cfg, "analog_layout", "uint16_le"))
    self.analog_channels = int(_cfg_get(cfg, "channels", 4))
    self.analog_scale = float(_cfg_get(cfg, "analog_scale", 1.0))
    self.analog_offset = float(_cfg_get(cfg, "analog_offset", 0.0))
    self.sense_range_mm = float(_cfg_get(cfg, "sense_range_mm", SENSE_RANGE_MM))
    self.threshold = float(_cfg_get(cfg, "conductance_threshold", 0.0))
    # firmware_safety=False: keep analog streaming but do not latch
    # controller collision-protect (pendant / next TCP run stay usable).
    self.firmware_safety = bool(_cfg_get(cfg, "firmware_safety", False))
    self.auto_clear = bool(_cfg_get(cfg, "auto_clear", True))
    self.part_ids = (
      int(_cfg_get(cfg, "forearm_part", 3)),
      int(_cfg_get(cfg, "j4_part", 4)),
      int(_cfg_get(cfg, "j5_part", 5)),
      int(_cfg_get(cfg, "j6_part", 6)),
    )
    self._last_clear_s = 0.0

  def _collision_level(self) -> Optional[int]:
    raw = _cfg_get(self.cfg, "collision_level", None)
    if self.firmware_safety:
      return None if raw is None else int(raw)
    return 0 if raw is None else int(raw)

  def enable(self, firmware_safety: Optional[bool] = None) -> None:
    if not self.enabled:
      return
    if firmware_safety is not None:
      self.firmware_safety = bool(firmware_safety)
    rc = self.robot.robot_control
    rc.EnableSafeSkin(1)
    sensitivity = int(_cfg_get(self.cfg, "sensitivity", 2))
    for part in self.part_ids:
      rc.SetSafeSkin(int(part), sensitivity)
    level = self._collision_level()
    if level is not None:
      rc.SetCollisionLevel(int(level))
    if not self.firmware_safety:
      # 0 = stop/disable (must never use for sensing). 2 = ignore if firmware
      # allows it, else 1 = pause. HIT stays a sensor flag, not a Disable.
      applied = False
      for post in (2, 1):
        try:
          resp = rc.SetPostCollisionMode(post)
          text = "" if resp is None else str(resp).strip()
          if text.startswith("0"):
            applied = True
            break
        except Exception:
          continue
      if not applied:
        try:
          rc.SetPostCollisionMode(1)
        except Exception:
          pass
    self.active = True

  def disable(self) -> None:
    if not self.enabled:
      return
    try:
      self.robot.robot_control.EnableSafeSkin(0)
    finally:
      self.active = False

  def reset(self) -> None:
    """Drop latched collision-protect and re-arm sensing."""
    reset = getattr(self.robot, "reset_safety", None) or getattr(
      self.robot, "ResetSafety", None
    )
    if callable(reset):
      reset()
    else:
      try:
        self.robot.robot_control.ClearError()
      except Exception:
        pass
    if self.enabled:
      self.enable()

  def _maybe_clear_protect(self, status: Optional[RobotStatus]) -> None:
    if not self.auto_clear or self.firmware_safety or status is None:
      return
    mode = int(getattr(status, "robot_mode", 0) or 0)
    collision = int(getattr(status, "collision_state", 0) or 0)
    if collision == 0 and mode not in (9, 10):
      return
    now = time.time()
    if now - self._last_clear_s < 0.4:
      return
    self._last_clear_s = now
    reset = getattr(self.robot, "reset_safety", None) or getattr(
      self.robot, "ResetSafety", None
    )
    if callable(reset):
      reset()
    else:
      rc = self.robot.robot_control
      try:
        rc.Stop()
      except Exception:
        pass
      try:
        rc.ClearError()
      except Exception:
        pass

  def read(
    self, status: Optional[RobotStatus] = None, *, clear: bool = False
  ) -> Optional[SkinReading]:
    """Pull the latest 30004 sample into a SkinReading."""
    if status is None:
      status = self.robot.GetStatus()
    if clear:
      self._maybe_clear_protect(status)
    reading = SkinReading.from_status(
      status,
      analog_layout=self.analog_layout,
      analog_channels=self.analog_channels,
      analog_scale=self.analog_scale,
      analog_offset=self.analog_offset,
      sense_range_mm=self.sense_range_mm,
    )
    if reading is not None:
      self._latest = reading
      self._conductance = reading.conductance.copy()
    return reading

  def set_conductance(self, values: Sequence[float]) -> None:
    """Override analog channels (tests / offline replay)."""
    self._conductance = np.asarray(values, dtype=float)
    if self._latest is None:
      self._latest = SkinReading()
    self._latest.analog = self._conductance.copy()
    self._latest.conductance = self._conductance.copy()
    peak = float(np.max(np.abs(self._conductance))) if self._conductance.size else 0.0
    self._latest.proximity = (
      np.clip(np.abs(self._conductance) / peak, 0.0, 1.0) if peak > 0 else self._conductance * 0
    )

  def conductance(self) -> np.ndarray:
    reading = self.read()
    if reading is not None:
      return reading.conductance.copy()
    return self._conductance.copy()

  def proximity(self) -> Optional[np.ndarray]:
    reading = self.read()
    return None if reading is None else reading.proximity.copy()

  def approach_flags(self) -> Optional[np.ndarray]:
    reading = self.read()
    return None if reading is None else reading.approach.copy()

  def latest(self) -> Optional[SkinReading]:
    return self.read()

  def triggered(self) -> bool:
    reading = self.read()
    if reading is not None:
      return reading.triggered(self.threshold)
    return bool(self.threshold > 0 and np.any(self._conductance >= self.threshold))

  def as_dict(self, reading: Optional[SkinReading] = None) -> dict:
    reading = reading if reading is not None else self._latest
    status = self.robot.GetStatus() if hasattr(self.robot, "GetStatus") else None
    collision = int(getattr(status, "collision_state", 0) or 0) if status else 0
    payload = {
      "enabled": self.enabled,
      "active": self.active,
      "firmware_safety": self.firmware_safety,
      "auto_clear": self.auto_clear,
      "parts": list(SKIN_PARTS),
      "collision_state": collision,
      "approach": None,
      "proximity": None,
      "conductance": None,
      "triggered": False,
    }
    if reading is not None:
      payload["approach"] = reading.approach.tolist()
      payload["proximity"] = reading.proximity.tolist()
      payload["conductance"] = reading.conductance.tolist()
      payload["triggered"] = bool(reading.triggered(self.threshold))
      payload["timestamp_ms"] = reading.timestamp_ms
    return payload
