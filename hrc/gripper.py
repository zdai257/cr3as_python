"""DH Robotics PGEA-50-26 parallel gripper over tool RS485 Modbus RTU."""

from __future__ import annotations

import time
from typing import Optional

from .utils.dashboard import parse_dashboard


class PGEAGripper:
  """PGE / PGE-A register map (0-1000 position, 20-100% force)."""

  def __init__(self, robot, cfg):
    self.robot = robot
    self.cfg = cfg
    self.index: Optional[int] = None
    self.connected = False

  def connect(self) -> None:
    io = self.robot.io
    io.SetToolPower(1)
    time.sleep(0.2)
    io.SetTool485(int(self.cfg.baud), str(self.cfg.parity), int(self.cfg.stop_bits))
    response = self.robot.communication.ModbusRTUCreate(
      int(self.cfg.slave_id),
      int(self.cfg.baud),
      str(self.cfg.parity),
      int(self.cfg.data_bits),
      int(self.cfg.stop_bits),
    )
    error_id, values, _ = parse_dashboard(response)
    if error_id != 0 or not values:
      raise RuntimeError(f"ModbusRTUCreate failed: {response}")
    self.index = int(values[0])
    self._write(int(self.cfg.reg_init), int(self.cfg.init_value))
    time.sleep(0.4)
    self._write(int(self.cfg.reg_force), int(self.cfg.force_percent))
    self._write(int(self.cfg.reg_speed), int(self.cfg.speed_percent))
    self.connected = True

  def disconnect(self) -> None:
    if self.index is None:
      return
    try:
      self.robot.communication.ModbusClose(self.index)
    finally:
      self.index = None
      self.connected = False

  def set_position(self, position_0_1000: int) -> None:
    pos = int(max(0, min(1000, position_0_1000)))
    self._write(int(self.cfg.reg_position), pos)
    time.sleep(float(self.cfg.settle_s))

  def open(self) -> None:
    self.set_position(int(self.cfg.open_position))

  def close(self) -> None:
    self.set_position(int(self.cfg.close_position))

  def position(self) -> Optional[int]:
    if self.index is None:
      return None
    response = self.robot.communication.GetHoldRegs(
      self.index, int(self.cfg.reg_feedback_pos), 1, "U16"
    )
    _, values, _ = parse_dashboard(response)
    return int(values[0]) if values else None

  def _write(self, addr: int, value: int) -> None:
    if self.index is None:
      raise RuntimeError("gripper Modbus master is not open")
    self.robot.communication.SetHoldRegs(self.index, addr, 1, [int(value)], "U16")
