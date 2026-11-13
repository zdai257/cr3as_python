"""DH Robotics PGEA-50-26 over tool RS485, matching Dobot+ DHGrip_v2.

Two holding-register maps exist in the wild:

  dh_ros  DHGrip_v2-1-6 / dh_gripper_ros PGE  (init 0xA5, pos 0x0103)
  pge     PGE-A datasheet                     (init 1,    pos 0x0102)

The pendant plugin DHGrip_v2 must Init the fingers once. TCP then attaches
the existing Modbus map (no 0xA5, no tool power-cycle — those undo plugin init).
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from .models.pgea import PGEA5026Model
from .utils.dashboard import parse_dashboard


GRIP_STATE_NAMES = {
  0: "moving",
  1: "arrived",
  2: "gripped",
  3: "dropped",
}

REGISTER_MAPS: Dict[str, Dict[str, int]] = {
  "dh_ros": {
    "reg_init": 0x0100,
    "init_value": 0xA5,
    "reg_force": 0x0101,
    "reg_position": 0x0103,
    "reg_speed": 0x0104,
    "reg_status": 0x0200,
    "reg_feedback_pos": 0x0202,
    "reg_grip_state": 0x0201,
  },
  "pge": {
    "reg_init": 0x0100,
    "init_value": 1,
    "reg_force": 0x0101,
    "reg_position": 0x0102,
    "reg_speed": 0x0103,
    "reg_status": 0x0200,
    "reg_feedback_pos": 0x0201,
    "reg_grip_state": 0x0201,
  },
}


def _cfg_get(cfg, name: str, default=None):
  if cfg is None:
    return default
  if hasattr(cfg, "get"):
    value = cfg.get(name, default)
    return default if value is None else value
  return getattr(cfg, name, default)


class PGEAGripper:
  """PGE / PGE-A register map (0-1000 position, 20-100% force)."""

  def __init__(self, robot, cfg):
    self.robot = robot
    self.cfg = cfg
    self.model = PGEA5026Model(cfg)
    self.index: Optional[int] = None
    self.connected = False
    self.last_command: Optional[int] = None
    self.last_error: Optional[str] = None
    self.map_name = str(_cfg_get(cfg, "register_map", "dh_ros") or "dh_ros")
    self._use_map(self.map_name, apply_overrides=True)

  def _apply_cfg_overrides(self) -> None:
    for key in self.regs:
      raw = _cfg_get(self.cfg, key, None)
      if raw is not None:
        self.regs[key] = int(raw)

  def _use_map(self, name: str, apply_overrides: bool = False) -> None:
    if name not in REGISTER_MAPS:
      name = "dh_ros"
    self.map_name = name
    self.regs = dict(REGISTER_MAPS[name])
    if apply_overrides:
      self._apply_cfg_overrides()

  def connect(self, power_cycle: Optional[bool] = None, initialize: Optional[bool] = None) -> None:
    """Attach tool RS485 Modbus. Prefer pendant DHGrip Init; skip 0xA5 if ready."""
    if self.connected:
      return
    ensure = getattr(self.robot, "ensure_enabled", None)
    if callable(ensure):
      ensure()
    io = self.robot.io
    cycle = bool(_cfg_get(self.cfg, "power_cycle", False)) if power_cycle is None else bool(power_cycle)
    try:
      io.SetToolMode(1)
    except Exception:
      pass
    if cycle:
      io.SetToolPower(0)
      time.sleep(0.35)
      io.SetToolPower(1)
      time.sleep(0.45)
    skip_init = bool(_cfg_get(self.cfg, "skip_init_if_ready", True))
    baud = int(_cfg_get(self.cfg, "baud", 115200))
    stop = int(_cfg_get(self.cfg, "stop_bits", 1))
    parities = [str(_cfg_get(self.cfg, "parity", "N") or "N")]
    if "E" not in parities:
      parities.append("E")
    if "N" not in parities:
      parities.append("N")
    maps = [self.map_name]
    if "pge" not in maps:
      maps.append("pge")
    if "dh_ros" not in maps:
      maps.append("dh_ros")
    last = None
    for parity in parities:
      try:
        io.SetTool485(baud, parity, stop)
      except Exception as exc:
        last = exc
        continue
      time.sleep(0.15)
      self._close_stale_masters()
      response = self.robot.communication.ModbusRTUCreate(
        int(_cfg_get(self.cfg, "slave_id", 1)),
        baud,
        parity,
        int(_cfg_get(self.cfg, "data_bits", 8)),
        stop,
      )
      try:
        error_id, values, _ = parse_dashboard(response)
      except Exception as exc:
        last = exc
        continue
      if error_id != 0 or not values:
        last = RuntimeError(f"ModbusRTUCreate failed: {response}")
        continue
      self.index = int(values[0])
      configured = str(_cfg_get(self.cfg, "register_map", "dh_ros") or "dh_ros")
      for name in maps:
        self._use_map(name, apply_overrides=(name == configured))
        try:
          status = None
          try:
            status = self._read(int(self.regs["reg_status"]))
          except Exception:
            status = None
          ready = status in (1, 2, 3)
          if initialize is False or (skip_init and initialize is not True and ready):
            self.connected = True
            self.last_error = None if ready else f"attached, not inited (status={status})"
            return
          if not ready and initialize is not True and skip_init:
            self.connected = True
            self.last_error = (
              f"gripper status={status}; Init once with pendant DHGrip_v2, then Take Cell"
            )
            return
          self._initialize()
          self.connected = True
          self.last_error = None
          return
        except Exception as exc:
          last = exc
      try:
        self.robot.communication.ModbusClose(self.index)
      except Exception:
        pass
      self.index = None
    raise RuntimeError(f"DHGrip / PGEA Modbus connect failed: {last}")

  def _close_stale_masters(self) -> None:
    for idx in range(0, 6):
      try:
        self.robot.communication.ModbusClose(idx)
      except Exception:
        pass

  def _initialize(self) -> None:
    self._write(int(self.regs["reg_init"]), int(self.regs["init_value"]))
    deadline = time.time() + 3.0
    ready = False
    while time.time() < deadline:
      status = self._read(int(self.regs["reg_status"]))
      if status in (1, 2, 3):
        ready = True
        break
      time.sleep(0.15)
    if not ready:
      # Some PGE-A firmware ACKs init without flipping 0x0200 immediately.
      time.sleep(0.4)
    self._write(int(self.regs["reg_force"]), int(_cfg_get(self.cfg, "force_percent", 50)))
    self._write(int(self.regs["reg_speed"]), int(_cfg_get(self.cfg, "speed_percent", 50)))

  def disconnect(self) -> None:
    if self.index is None:
      self.connected = False
      return
    try:
      self.robot.communication.ModbusClose(self.index)
    finally:
      self.index = None
      self.connected = False

  def set_position(self, position_0_1000: int, wait: bool = True) -> None:
    pos = int(max(0, min(1000, position_0_1000)))
    self._write(int(self.regs["reg_position"]), pos)
    self.last_command = pos
    if wait:
      time.sleep(float(_cfg_get(self.cfg, "settle_s", 0.3)))

  def open(self) -> None:
    self.set_position(int(_cfg_get(self.cfg, "open_position", 1000)))

  def close(self) -> None:
    self.set_position(int(_cfg_get(self.cfg, "close_position", 0)))

  def position(self) -> Optional[int]:
    if self.index is None:
      return None
    return self._read(int(self.regs["reg_feedback_pos"]))

  def opening_mm(self) -> Optional[float]:
    pos = self.position()
    if pos is None:
      return None
    return self.model.opening_mm(pos)

  def status_word(self) -> Optional[int]:
    if self.index is None:
      return None
    return self._read(int(self.regs["reg_status"]))

  def grip_state(self) -> Optional[int]:
    """0 moving, 1 arrived, 2 object gripped, 3 object dropped (DH map)."""
    if self.index is None:
      return None
    return self._read(int(self.regs["reg_grip_state"]))

  def grip_state_name(self) -> str:
    if not self.connected:
      return "disconnected"
    state = self.grip_state()
    if state is None:
      return "unknown"
    return GRIP_STATE_NAMES.get(int(state) & 0xFF, f"code_{int(state)}")

  def as_dict(self) -> dict:
    pos = None
    opening = None
    state = None
    if self.connected:
      try:
        pos = self.position()
        opening = self.model.opening_mm(pos)
        state = self.grip_state()
      except Exception as exc:
        self.last_error = str(exc)
    return {
      "connected": self.connected,
      "enabled": bool(_cfg_get(self.cfg, "enabled", True)),
      "map": self.map_name,
      "position": pos,
      "command": self.last_command,
      "opening_mm": opening,
      "grip_state": state,
      "grip_state_name": self.grip_state_name() if self.connected else "disconnected",
      "last_error": self.last_error,
      "model": self.model.as_dict(),
    }

  def _read(self, addr: int) -> Optional[int]:
    if self.index is None:
      raise RuntimeError("gripper Modbus master is not open")
    response = self.robot.communication.GetHoldRegs(self.index, int(addr), 1, "U16")
    _, values, _ = parse_dashboard(response)
    return int(values[0]) if values else None

  def _write(self, addr: int, value: int) -> None:
    if self.index is None:
      raise RuntimeError("gripper Modbus master is not open")
    self.robot.communication.SetHoldRegs(self.index, addr, 1, [int(value)], "U16")
