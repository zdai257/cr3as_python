"""Offline tests for gripper model, SafeSkin 30004 parsing, and dataset I/O."""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dobot_sdk.models.status import RobotStatus
from dobot_sdk.protocol.data_types import TEST_VALUE_MAGIC, MyType
from dobot_sdk.protocol.feedback import FeedbackParser

from hrc.data import Episode, write_lerobot_dataset, write_ros2_bag
from hrc.models import PGEA5026Model
from hrc.perception.skins import SkinReading
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


if __name__ == "__main__":
  test_pgea_model()
  test_tool_offset_fk()
  test_skins_from_30004()
  test_episode_roundtrip()
  print("All HRC research tests passed")
