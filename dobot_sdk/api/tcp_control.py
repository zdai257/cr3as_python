# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Take exclusive TCP control and recover from collision / pause / pendant lock.

RequestControl() is only legal when the controller is powered-off or disabled
(and not paused / brake-released). SafeSkin approach, collision-protect, and a
teaching pendant left in ENABLE all block the next TCP session unless we walk
the controller back to a legal state first.
"""

from __future__ import annotations

import re
import time
from typing import Callable, Optional

from ..models.status import RobotMode

_NOT_TCP = re.compile(r"not\s+tcp", re.IGNORECASE)

# Modes where RequestControl() is documented as legal.
_REQUEST_OK = {
  int(RobotMode.UNKNOWN),
  int(RobotMode.INIT),
  int(RobotMode.POWEROFF),
  int(RobotMode.DISABLED),
}

_Sleep = Callable[[float], None]


def cmd_ok(response: Optional[str]) -> bool:
  if not response:
    return False
  text = str(response).strip()
  if _NOT_TCP.search(text):
    return False
  return text.startswith("0,") or text.startswith("0 ")


def in_tcp_mode(robot) -> bool:
  """True when dashboard commands are accepted (already in TCP)."""
  try:
    response = robot.robot_control.RobotMode()
  except Exception:
    return False
  if not response or _NOT_TCP.search(str(response)):
    return False
  return cmd_ok(response)


def parse_robot_mode(response: Optional[str]) -> int:
  if not response:
    return int(RobotMode.UNKNOWN)
  text = str(response)
  if _NOT_TCP.search(text):
    return int(RobotMode.UNKNOWN)
  match = re.search(r"\{(-?\d+)\}", text)
  if match:
    return int(match.group(1))
  return int(RobotMode.UNKNOWN)


def robot_mode(robot) -> int:
  """Prefer dashboard RobotMode(); fall back to 30004 if not in TCP."""
  response = None
  try:
    response = robot.robot_control.RobotMode()
    if response and cmd_ok(response) and not _NOT_TCP.search(str(response)):
      return parse_robot_mode(response)
  except Exception:
    pass
  status = getattr(robot, "GetStatus", lambda: None)()
  if status is not None:
    try:
      return int(status.robot_mode)
    except (TypeError, ValueError):
      pass
  if response:
    return parse_robot_mode(response)
  return int(RobotMode.UNKNOWN)


def mode_name(mode: int) -> str:
  try:
    return RobotMode(int(mode)).name
  except ValueError:
    return f"UNKNOWN({mode})"


def _try(fn, *args) -> Optional[str]:
  try:
    return fn(*args)
  except Exception:
    return None


def _payload_kg(robot) -> float:
  cfg = getattr(robot, "cfg", None)
  if cfg is None:
    return 0.0
  try:
    return float(cfg.robot.payload_kg)
  except Exception:
    return 0.0


def reset_safety(robot, sleep: _Sleep = time.sleep) -> int:
  """Clear collision / pause / alarm without resuming motion and without Disable.

  Uses Stop+ClearError (not Continue, not DisableRobot). A SafeSkin HIT must
  not drop the controller into DISABLED or hand it back to Online/pendant.
  """
  rc = robot.robot_control
  mode = robot_mode(robot)
  if mode in (
    int(RobotMode.RUNNING),
    int(RobotMode.SINGLE_MOVE),
    int(RobotMode.JOG),
    int(RobotMode.PAUSE),
  ):
    _try(rc.Stop)
  _try(rc.ClearError)
  sleep(0.15)
  mode = robot_mode(robot)
  if mode == int(RobotMode.BACKDRIVE):
    _try(rc.StopDrag)
    sleep(0.2)
    mode = robot_mode(robot)
  if mode == int(RobotMode.PAUSE):
    _try(rc.Stop)
    _try(rc.ClearError)
    sleep(0.15)
    mode = robot_mode(robot)
  if mode == int(RobotMode.ERROR):
    _try(rc.ClearError)
    sleep(0.2)
    mode = robot_mode(robot)
  status = getattr(robot, "GetStatus", lambda: None)()
  if status is not None and int(getattr(status, "collision_state", 0) or 0):
    _try(rc.ClearError)
    sleep(0.1)
    mode = robot_mode(robot)
  return mode


def ensure_enabled(robot, sleep: _Sleep = time.sleep, load: Optional[float] = None) -> int:
  """Re-enable after a SafeSkin HIT / collision-protect disable. Stays in TCP."""
  rc = robot.robot_control
  mode = robot_mode(robot)
  if mode == int(RobotMode.ERROR):
    _try(rc.ClearError)
    sleep(0.2)
    mode = robot_mode(robot)
  if mode == int(RobotMode.POWEROFF):
    _try(rc.PowerOn)
    sleep(1.0)
    mode = robot_mode(robot)
  if mode in (int(RobotMode.ENABLE), int(RobotMode.RUNNING), int(RobotMode.JOG)):
    return mode
  kg = _payload_kg(robot) if load is None else float(load)
  if kg > 0:
    _try(rc.EnableRobot, kg)
  else:
    _try(rc.EnableRobot)
  sleep(0.35)
  return robot_mode(robot)


def _reengage_brakes(robot, sleep: _Sleep) -> None:
  rc = robot.robot_control
  for axis in range(1, 7):
    _try(rc.BrakeControl, axis, 0)
  sleep(0.25)


def _disable_if_needed(robot, sleep: _Sleep) -> int:
  """Drive toward DISABLED so RequestControl() becomes legal."""
  rc = robot.robot_control
  mode = robot_mode(robot)
  if mode in (
    int(RobotMode.RUNNING),
    int(RobotMode.SINGLE_MOVE),
    int(RobotMode.JOG),
    int(RobotMode.PAUSE),
  ):
    _try(rc.Stop)
    sleep(0.2)
    mode = robot_mode(robot)
  if mode == int(RobotMode.BACKDRIVE):
    _try(rc.StopDrag)
    sleep(0.2)
    mode = robot_mode(robot)
  if mode == int(RobotMode.BRAKE_OPEN):
    _reengage_brakes(robot, sleep)
    mode = robot_mode(robot)
  if mode == int(RobotMode.ERROR):
    _try(rc.ClearError)
    sleep(0.2)
    mode = robot_mode(robot)
  if mode in (
    int(RobotMode.ENABLE),
    int(RobotMode.RUNNING),
    int(RobotMode.SINGLE_MOVE),
    int(RobotMode.PAUSE),
    int(RobotMode.JOG),
    int(RobotMode.ERROR),
  ):
    _try(rc.DisableRobot)
    sleep(0.25)
    mode = robot_mode(robot)
  return mode


def take_tcp_control(
  robot,
  timeout: float = 12.0,
  sleep: _Sleep = time.sleep,
  keep_enabled: bool = True,
) -> int:
  """Recover controller state and switch to TCP mode.

  Returns the RobotMode integer after takeover. Raises RuntimeError if the
  pendant / collision-protect lock cannot be cleared in time.

  keep_enabled: if True, do not leave the arm DISABLED. RequestControl()
  still legally requires a brief disable when the pendant owns ENABLE, but
  we EnableRobot() immediately after TCP is granted so a SafeSkin HIT cannot
  dump the session into Online/Disabled.
  """
  deadline = time.time() + max(0.5, float(timeout))
  last_request = ""
  last_mode = robot_mode(robot)

  while time.time() < deadline:
    tcp = in_tcp_mode(robot)
    last_mode = robot_mode(robot)

    if tcp:
      last_mode = reset_safety(robot, sleep=sleep)
      if last_mode == int(RobotMode.BRAKE_OPEN):
        _reengage_brakes(robot, sleep)
        last_mode = robot_mode(robot)
      if keep_enabled and last_mode in (
        int(RobotMode.DISABLED),
        int(RobotMode.POWEROFF),
        int(RobotMode.ERROR),
        int(RobotMode.PAUSE),
      ):
        last_mode = ensure_enabled(robot, sleep=sleep)
      return last_mode

    # Already in TCP? RequestControl first so we do not Disable an owned session.
    last_request = _try(robot.robot_control.RequestControl) or ""
    if cmd_ok(last_request):
      sleep(0.15)
      last_mode = reset_safety(robot, sleep=sleep)
      if keep_enabled:
        last_mode = ensure_enabled(robot, sleep=sleep)
      return last_mode

    # Pendant / Online owns the controller. RequestControl is only legal when
    # powered-off or disabled — disable only as a last step, then re-enable.
    last_mode = _disable_if_needed(robot, sleep)
    if last_mode not in _REQUEST_OK:
      _try(robot.robot_control.DisableRobot)
      sleep(0.25)
      last_mode = robot_mode(robot)
    last_request = _try(robot.robot_control.RequestControl) or last_request
    if cmd_ok(last_request):
      sleep(0.15)
      last_mode = reset_safety(robot, sleep=sleep)
      if keep_enabled:
        last_mode = ensure_enabled(robot, sleep=sleep)
      return last_mode
    sleep(0.25)

  raise RuntimeError(
    "could not take TCP control "
    f"(mode={mode_name(last_mode)}/{last_mode}, last={last_request!r}). "
    "RequestControl() is only allowed when powered-off or disabled "
    "(not paused, enabled, Online/hand-auto, or brake-released). "
    "On the pendant: turn off hand-auto/Online, clear collision, then retry."
  )
