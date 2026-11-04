"""CS3AS safe-skin interface.

Approach bits come from Feedback (forearm / J4 / J5 / J6). Analog conductance
channels are a stub until a DAQ is attached — fill `set_conductance()`.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


class SafeSkinFrontend:
  def __init__(self, robot, cfg):
    self.robot = robot
    self.cfg = cfg
    n = int(cfg.channels) if hasattr(cfg, "channels") else 8
    self._conductance = np.zeros(n, dtype=float)
    self.enabled = bool(cfg.enabled)

  def enable(self) -> None:
    if not self.enabled:
      return
    rc = self.robot.robot_control
    rc.EnableSafeSkin(1)
    sensitivity = int(self.cfg.sensitivity)
    for part in (self.cfg.forearm_part, self.cfg.j4_part, self.cfg.j5_part, self.cfg.j6_part):
      rc.SetSafeSkin(int(part), sensitivity)

  def disable(self) -> None:
    if not self.enabled:
      return
    self.robot.robot_control.EnableSafeSkin(0)

  def set_conductance(self, values: Sequence[float]) -> None:
    """Hook for a future analog front-end (skin conductance / proximity)."""
    self._conductance = np.asarray(values, dtype=float)

  def conductance(self) -> np.ndarray:
    return self._conductance.copy()

  def approach_flags(self) -> Optional[np.ndarray]:
    status = self.robot.GetStatus()
    if status is None:
      return None
    return np.array(
      [
        status.arm_approach_state,
        status.j4_approach_state,
        status.j5_approach_state,
        status.j6_approach_state,
      ],
      dtype=float,
    )

  def triggered(self) -> bool:
    flags = self.approach_flags()
    if flags is None:
      return False
    if np.any(flags):
      return True
    thr = float(self.cfg.conductance_threshold)
    return bool(thr > 0 and np.any(self._conductance >= thr))
