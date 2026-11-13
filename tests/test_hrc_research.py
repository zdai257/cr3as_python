"""Offline tests for gripper model, SafeSkin 30004 parsing, and dataset I/O."""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dobot_sdk.api.tcp_control import (
  cmd_ok,
  in_tcp_mode,
  parse_robot_mode,
  reset_safety,
  take_tcp_control,
)
from dobot_sdk.models.status import RobotMode, RobotStatus
from dobot_sdk.protocol.data_types import TEST_VALUE_MAGIC, MyType
from dobot_sdk.protocol.feedback import FeedbackParser

from hrc.data import Episode, write_lerobot_dataset, write_ros2_bag
from hrc.gripper import PGEAGripper
from hrc.models import PGEA5026Model
from hrc.perception.skins import SafeSkinFrontend, SkinReading
from hrc.utils.kinematics import Kinematics
from dobot_sdk import load_robot_config


def test_pgea_model():
  model = PGEA5026Model()
  assert model.urdf_path.is_file(), model.urdf_path
  assert (model.mesh_dir / "body.stl").is_file()
  assert (model.mesh_dir / "finger.stl").is_file()
  pose = model.tool_pose()
  assert pose[:3] == [0.0, 0.0, 118.0]
  assert abs(model.opening_mm(0) - 1.2) < 1e-6
  assert abs(model.opening_mm(1000) - 27.2) < 1e-6
  T = np.eye(4)
  caps = model.capsules(T, 1000)
  assert len(caps) == 3
  print("PGEA-50-26 model OK", model.urdf_path)


def test_tool_offset_fk():
  cfg = load_robot_config()
  kin = Kinematics(cfg.kinematics, gripper_model=PGEA5026Model(cfg.gripper))
  frames = kin.fk_matrices([0, 0, 0, 0, 0, 0])
  flange = kin.flange_matrix([0, 0, 0, 0, 0, 0])
  tcp = frames[-1][:3, 3]
  delta = tcp - flange[:3, 3]
  assert abs(np.linalg.norm(delta) - 118.0) < 1e-6
  print("tool offset FK OK", delta)


def _packet_with_skins(approach=(1, 0, 2, 0), analog_u16=(100, 200, 300, 400)) -> bytes:
  pkt = np.zeros(1, dtype=MyType)
  pkt["len"][0] = 1440
  pkt["TestValue"][0] = TEST_VALUE_MAGIC
  pkt["ArmApproachState"][0] = approach[0]
  pkt["J4ApproachState"][0] = approach[1]
  pkt["J5ApproachState"][0] = approach[2]
  pkt["J6ApproachState"][0] = approach[3]
  raw = np.zeros(61, dtype=np.uint8)
  for i, value in enumerate(analog_u16):
    raw[2 * i] = value & 0xFF
    raw[2 * i + 1] = (value >> 8) & 0xFF
  pkt["reserve7"][0] = raw
  data = pkt.tobytes()
  assert len(data) == 1440
  return data


def test_skins_from_30004():
  parser = FeedbackParser()
  status = parser.parse(_packet_with_skins())
  assert status is not None
  assert status.arm_approach_state == 1
  assert status.j5_approach_state == 2
  assert status.skin_analog[0] == 100
  assert status.skin_analog[3] == 400
  reading = SkinReading.from_status(status)
  assert reading is not None
  assert reading.triggered()
  assert reading.proximity.shape == (4,)
  assert reading.conductance.shape[0] >= 4
  print("SafeSkin 30004 parse OK", reading.as_dict())


def test_episode_roundtrip():
  ep = Episode.synthetic(n=12, fps=50.0, task="unit")
  with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / "episode_0000.npz"
    ep.save(path)
    loaded = Episode.load(path)
    assert len(loaded) == 12
    assert loaded.states().shape == (12, 17)
    lerobot_dir = Path(tmp) / "lerobot"
    write_lerobot_dataset([loaded], lerobot_dir, fps=50.0)
    assert (lerobot_dir / "meta" / "info.json").is_file()
    bag = Path(tmp) / "bag"
    write_ros2_bag(loaded, bag)
    assert (bag / "metadata.yaml").is_file()
    assert (bag / "episode_0.db3").is_file()
  print("episode / lerobot / ros2 bag OK")


class _FakeRC:
  def __init__(self, mode: int, tcp: bool = True):
    self.mode = int(mode)
    self.tcp = tcp
    self.calls = []

  def _record(self, name, ok=True):
    self.calls.append(name)
    return f"0,{{}},{name}();" if ok else f"-1,{{}},{name}();"

  def RobotMode(self):
    if not self.tcp:
      return "Control Mode Is Not Tcp"
    return f"0,{{{self.mode}}},RobotMode();"

  def RequestControl(self):
    self.calls.append("RequestControl")
    if self.mode in (0, 1, 3, 4):
      self.tcp = True
      return "0,{},RequestControl();"
    return "-1,{},RequestControl();"

  def ClearError(self):
    if self.mode == 9:
      self.mode = 5
    return self._record("ClearError")

  def Stop(self):
    if self.mode in (7, 8, 10, 11):
      self.mode = 5
    return self._record("Stop")

  def DisableRobot(self):
    self.mode = 4
    return self._record("DisableRobot")

  def EnableRobot(self, load=0.0, **kwargs):
    self.mode = 5
    return self._record("EnableRobot")

  def PowerOn(self):
    self.mode = 4
    return self._record("PowerOn")

  def StopDrag(self):
    if self.mode == 6:
      self.mode = 4
    return self._record("StopDrag")

  def EnableSafeSkin(self, status):
    return self._record("EnableSafeSkin")

  def SetSafeSkin(self, part, sensitivity):
    return self._record("SetSafeSkin")

  def SetCollisionLevel(self, level):
    return self._record("SetCollisionLevel")

  def SetPostCollisionMode(self, mode):
    return self._record("SetPostCollisionMode")


class _FakeStatus:
  def __init__(self, mode, collision=0):
    self.robot_mode = mode
    self.collision_state = collision
    self.enable_status = mode in (5, 7, 8, 10, 11)


class _FakeRobot:
  def __init__(self, mode: int, tcp: bool = True, collision: int = 0):
    self.robot_control = _FakeRC(mode, tcp=tcp)
    self._status = _FakeStatus(mode, collision)

  def GetStatus(self):
    self._status.robot_mode = self.robot_control.mode
    return self._status


def test_tcp_control_recovery():
  assert cmd_ok("0,{},RequestControl();")
  assert not cmd_ok("Control Mode Is Not Tcp")
  assert parse_robot_mode("0,{10},RobotMode();") == 10

  already = _FakeRobot(RobotMode.ERROR, tcp=True)
  assert in_tcp_mode(already)
  mode = take_tcp_control(already, timeout=2.0, sleep=lambda _s: None)
  assert already.robot_control.tcp
  assert "ClearError" in already.robot_control.calls
  assert mode in (int(RobotMode.ENABLE), int(RobotMode.ERROR), int(RobotMode.DISABLED))

  disabled = _FakeRobot(RobotMode.DISABLED, tcp=True)
  mode = take_tcp_control(disabled, timeout=2.0, sleep=lambda _s: None)
  assert "EnableRobot" in disabled.robot_control.calls
  assert mode == int(RobotMode.ENABLE)

  pendant = _FakeRobot(RobotMode.ENABLE, tcp=False)
  assert not in_tcp_mode(pendant)
  mode = take_tcp_control(pendant, timeout=2.0, sleep=lambda _s: None)
  assert pendant.robot_control.tcp
  assert "DisableRobot" in pendant.robot_control.calls
  assert "RequestControl" in pendant.robot_control.calls
  assert "EnableRobot" in pendant.robot_control.calls
  assert mode == int(RobotMode.ENABLE)

  stay = _FakeRobot(RobotMode.ENABLE, tcp=False)
  mode = take_tcp_control(stay, timeout=2.0, sleep=lambda _s: None, keep_enabled=False)
  assert "DisableRobot" in stay.robot_control.calls
  assert mode == int(RobotMode.DISABLED)

  paused = _FakeRobot(RobotMode.PAUSE, tcp=True)
  reset_safety(paused, sleep=lambda _s: None)
  assert paused.robot_control.mode == int(RobotMode.ENABLE)
  print("TCP takeover / safety reset OK")


def test_skins_and_gripper_status():
  cfg = load_robot_config()
  robot = _FakeRobot(RobotMode.DISABLED, tcp=True)
  skins = SafeSkinFrontend(robot, cfg.skins)
  assert skins.firmware_safety is False
  skins.enable()
  assert skins.active
  assert "EnableSafeSkin" in robot.robot_control.calls
  assert "SetCollisionLevel" in robot.robot_control.calls
  info = skins.as_dict()
  assert info["active"] is True
  assert info["firmware_safety"] is False
  skins.disable()
  assert skins.active is False

  grip = PGEAGripper(robot, cfg.gripper)
  payload = grip.as_dict()
  assert payload["connected"] is False
  assert payload["grip_state_name"] == "disconnected"
  print("SafeSkin / gripper status dict OK", info["parts"], payload["model"]["name"])


if __name__ == "__main__":
  test_pgea_model()
  test_tool_offset_fk()
  test_skins_from_30004()
  test_episode_roundtrip()
  test_tcp_control_recovery()
  test_skins_and_gripper_status()
  print("All HRC research tests passed")
