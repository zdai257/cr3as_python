"""Offline tests for gripper model, SafeSkin 30004 parsing, and dataset I/O."""

from __future__ import annotations

import json
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

from hrc.data import (
  CameraSample,
  Episode,
  EpisodeRecorder,
  HUB_VIDEO_FPS,
  resample_episode,
  write_lerobot_dataset,
  write_ros2_bag,
)
from hrc.gripper import REGISTER_MAPS, PGEAGripper
from hrc.models import PGEA5026Model
from hrc.perception.camera import HandEyeCamera
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
  reading = SkinReading.from_status(status, analog_max=1023.0)
  assert reading is not None
  assert reading.triggered()
  assert reading.proximity.shape == (4,)
  assert reading.conductance.shape[0] >= 4
  assert abs(float(reading.proximity[0]) - 100.0 / 1023.0) < 1e-6
  assert float(reading.proximity[3]) < 0.5
  assert float(reading.distance_mm()[0]) > 100.0
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
    info = json.loads((lerobot_dir / "meta" / "info.json").read_text(encoding="utf-8"))
    for key in (
      "observation.skins",
      "observation.skin_analog",
      "observation.skin_distance",
      "observation.skin_approach",
      "data_path",
      "video_path",
    ):
      if key.startswith("observation"):
        assert key in info["features"], key
      else:
        assert key in info, key
    assert (lerobot_dir / "README.md").is_file()
    assert (lerobot_dir / "meta" / "episodes_stats.jsonl").is_file()
    assert loaded.skin_distance().shape == (12, 4)
    assert loaded.skin_analog().shape == (12, 4)
    from hrc.data import episode_stats, read_lerobot_episodes, write_replay_html

    roundtrip = read_lerobot_episodes(lerobot_dir)
    assert len(roundtrip) == 1
    assert len(roundtrip[0]) == 12
    html = write_replay_html(roundtrip, Path(tmp) / "replay.html")
    text = html.read_text(encoding="utf-8")
    assert "Play" in text
    assert "CS3AS" in text
    stats = episode_stats(loaded)
    assert stats["n_frames"] == 12
    assert "hit_rate" in stats
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
  assert REGISTER_MAPS["dh_ros"]["reg_position"] == 0x0103
  assert REGISTER_MAPS["dh_ros"]["init_value"] == 0xA5
  assert grip.map_name == "dh_ros"
  assert grip.regs["reg_position"] == 0x0103
  assert cfg.gripper.get("power_cycle") is False
  print("SafeSkin / gripper status dict OK", info["parts"], payload["model"]["name"])


def test_camera_replay_html():
  from hrc.data.lerobot_io import attach_episode_videos, episode_video_paths
  from hrc.data.viewer import discover_episodes, write_replay_html

  root = Path(__file__).resolve().parents[1]
  dest = root / "data" / "lerobot" / "skins"
  mp4 = dest / "videos" / "chunk-000" / "observation.images.handeye" / "episode_000000.mp4"
  if not mp4.is_file():
    print("skip camera replay (no skins mp4)")
    return
  found = episode_video_paths(dest, 0)
  assert "observation.images.handeye" in found
  episodes = discover_episodes(dest)
  assert episodes
  assert episodes[0].videos
  npz = root / "data" / "episodes" / "skins.npz"
  if npz.is_file():
    from hrc.data import Episode

    ep = Episode.load(npz)
    attach_episode_videos(ep, source_path=npz)
    assert ep.videos, "skins.npz should pick up data/lerobot/skins hand-eye mp4"
  with tempfile.TemporaryDirectory() as tmp:
    html = Path(tmp) / "replay.html"
    # Copy-style relative links: write beside a fake dataset layout
    layout = Path(tmp) / "skins"
    (layout / "videos" / "chunk-000" / "observation.images.handeye").mkdir(parents=True)
    linked = layout / "videos" / "chunk-000" / "observation.images.handeye" / "episode_000000.mp4"
    try:
      linked.symlink_to(mp4)
    except OSError:
      linked.write_bytes(mp4.read_bytes()[:64])
    episodes[0].videos = {"observation.images.handeye": str(linked)}
    path = write_replay_html(episodes, layout / "replay.html", camera=True)
    text = path.read_text(encoding="utf-8")
    assert "<video" in text
    assert "episode_000000.mp4" in text
    assert "observation.images.handeye" in text
    hidden = write_replay_html(episodes, Path(tmp) / "nocam.html", camera=False)
    assert "episode_000000.mp4" not in hidden.read_text(encoding="utf-8")
  print("camera replay HTML OK", list(episodes[0].videos))


def test_resample_episode_to_camera_fps():
  ep = Episode.synthetic(n=125, fps=125.0, task="resample")
  for frame in ep.frames[10:13]:
    frame.skin_approach[:] = 1.0
  down = resample_episode(ep, 30.0)
  assert down.fps == 30.0
  assert 25 <= len(down) <= 40
  assert down.states().shape[1] == 17
  assert down.skin_proximity().shape == (len(down), 4)
  assert float(np.max(np.abs(down.skin_approach()))) > 0.0
  print("resample 125→30 Hz OK", len(ep), "->", len(down))


def _tiny_jpeg():
  import shutil
  import subprocess

  ffmpeg = shutil.which("ffmpeg")
  if ffmpeg is None:
    return None
  with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / "f.jpg"
    subprocess.check_call(
      [
        ffmpeg,
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "lavfi",
        "-i",
        "color=c=green:s=64x64:d=1",
        "-frames:v",
        "1",
        str(path),
      ]
    )
    return path.read_bytes()


def test_recorder_hub_export_defaults():
  from hrc.data.lerobot_io import probe_video
  from hrc.perception.camera import H264_FOURCCS, open_h264_writer

  rec = EpisodeRecorder(type("Robot", (), {"cfg": None})())
  assert rec.fps == 125.0
  assert rec.hub_fps == float(HUB_VIDEO_FPS) == 30.0
  cfg = load_robot_config()
  rec_cfg = EpisodeRecorder(type("Robot", (), {"cfg": cfg})(), cfg=cfg)
  assert rec_cfg.fps == 125.0
  assert rec_cfg.hub_fps == 30.0
  assert H264_FOURCCS[0] == "avc1"
  assert open_h264_writer is not None

  ep = Episode.synthetic(n=125, fps=125.0, task="hub_defaults")
  rec.episode = ep
  with tempfile.TemporaryDirectory() as tmp:
    dest = Path(tmp) / "no_cam"
    out = rec.export_lerobot(dest, episode=ep)
    info = json.loads((out / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["fps"] == 30
    assert 25 <= int(info["total_frames"]) <= 40
    assert int(info["total_videos"]) == 0
    assert len(ep) == 125
    assert ep.fps == 125.0

    writer_dir = Path(tmp) / "writer_native"
    write_lerobot_dataset([ep], writer_dir)
    winfo = json.loads((writer_dir / "meta" / "info.json").read_text(encoding="utf-8"))
    assert winfo["fps"] == 125
    assert int(winfo["total_frames"]) == 125

    jpeg = _tiny_jpeg()
    if jpeg:
      t0 = float(ep.timestamps()[0])
      ep.camera_samples = [
        CameraSample(timestamp=t0 + i / 30.0, jpeg=jpeg, width=64, height=64) for i in range(30)
      ]
      cam_dir = Path(tmp) / "cam"
      write_lerobot_dataset([ep], cam_dir)
      cinfo = json.loads((cam_dir / "meta" / "info.json").read_text(encoding="utf-8"))
      assert cinfo["fps"] == 30
      assert int(cinfo["total_videos"]) == 1
      mp4 = cam_dir / "videos" / "chunk-000" / "observation.images.handeye" / "episode_000000.mp4"
      assert mp4.is_file()
      probed = probe_video(mp4)
      codec = str(probed.get("codec") or "").lower()
      if codec:
        assert codec in {"h264", "avc1", "libx264"}
      assert "mp4v" not in codec
      assert "mpeg4" not in codec
  print("recorder / Hub export defaults OK")


def test_handeye_camera_offline():
  cfg = load_robot_config()
  cam = HandEyeCamera(cfg)
  assert cam.device == "/dev/video0"
  assert cam.width == 1920
  assert cam.height == 1080
  payload = cam.as_dict()
  assert payload["open"] is False
  assert payload["streaming"] is False
  none = HandEyeCamera(None)
  assert none.device == "/dev/video0"
  print("hand-eye camera config OK", payload["device"], payload["requested"])


if __name__ == "__main__":
  test_pgea_model()
  test_tool_offset_fk()
  test_skins_from_30004()
  test_episode_roundtrip()
  test_tcp_control_recovery()
  test_skins_and_gripper_status()
  test_handeye_camera_offline()
  test_resample_episode_to_camera_fps()
  test_recorder_hub_export_defaults()
  test_camera_replay_html()
  print("All HRC research tests passed")
