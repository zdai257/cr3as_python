"""Live episode recorder from CS3AS Feedback (port 30004, 8 ms)."""

from __future__ import annotations

import threading
import time
from typing import Optional

from .dataset import Episode, default_dataset_dir, next_episode_path
from .frame import CameraSample, EpisodeFrame
from .lerobot_io import HUB_VIDEO_FPS, write_lerobot_dataset


def _learning_value(learning, name, default):
  if learning is None:
    return default
  if hasattr(learning, "get"):
    value = learning.get(name, default)
  else:
    value = getattr(learning, name, default)
  return default if value is None else value


class EpisodeRecorder:
  def __init__(
    self,
    robot,
    cfg=None,
    fps: Optional[float] = None,
    hub_fps: Optional[float] = None,
    task: Optional[str] = None,
  ):
    self.robot = robot
    self.cfg = cfg or getattr(robot, "cfg", None)
    learning = None
    if self.cfg is not None and hasattr(self.cfg, "hrc"):
      learning = getattr(self.cfg.hrc, "learning", None)
    # Native npz stays on the 30004 bus (~125 Hz). Hub parquet/mp4 uses hub_fps.
    self.fps = float(fps if fps is not None else _learning_value(learning, "fps", 125.0))
    self.hub_fps = float(
      hub_fps if hub_fps is not None else _learning_value(learning, "hub_fps", HUB_VIDEO_FPS)
    )
    self.task = str(task if task is not None else _learning_value(learning, "task", "cs3as_hrc"))
    self.episode = Episode(task=self.task, fps=self.fps)
    self._lock = threading.Lock()
    self._running = False
    self._attached = False
    self._last_action = None
    self._last_grip_cmd = None
    self._grip_cache = None
    self._grip_t = 0.0
    self._cam_t = 0.0

  def set_action(self, action, gripper_command: Optional[float] = None) -> None:
    import numpy as np

    with self._lock:
      self._last_action = np.asarray(action, dtype=float).reshape(-1)
      if gripper_command is not None:
        self._last_grip_cmd = float(gripper_command)

  def _poll_gripper(self):
    now = time.time()
    if now - self._grip_t < 0.2:
      return self._grip_cache
    grip = getattr(self.robot, "gripper", None)
    if grip is None or not getattr(grip, "connected", False):
      self._grip_t = now
      return self._grip_cache
    try:
      self._grip_cache = grip.position()
    except Exception:
      pass
    self._grip_t = now
    return self._grip_cache

  def _sample_camera(self) -> None:
    cam = getattr(self.robot, "camera", None)
    if cam is None or not getattr(cam, "is_open", False):
      return
    now = time.time()
    cam_fps = float(getattr(cam, "actual_fps", 0) or getattr(cam, "fps", 0) or self.hub_fps or 30.0)
    period = 1.0 / max(cam_fps, 1.0)
    if now - self._cam_t < period:
      return
    frame = cam.latest()
    if frame is None:
      return
    try:
      from ..perception.camera import encode_rgb_jpeg

      jpeg = encode_rgb_jpeg(frame.rgb)
    except Exception:
      jpeg = None
    if not jpeg:
      return
    self._cam_t = now
    with self._lock:
      self.episode.camera_samples.append(
        CameraSample(
          timestamp=frame.timestamp,
          jpeg=jpeg,
          width=int(frame.width),
          height=int(frame.height),
        )
      )

  def _frame_from_status(self, status) -> EpisodeFrame:
    grip = self._poll_gripper()
    skin = None
    try:
      skin = self.robot.skins.read(status)
    except Exception:
      skin = None
    with self._lock:
      action = self._last_action
      grip_cmd = self._last_grip_cmd
    self._sample_camera()
    return EpisodeFrame.from_status(
      status,
      gripper_position=grip,
      gripper_command=grip_cmd,
      action=action,
      skin=skin,
      wall_time=time.time(),
    )

  def _on_status(self, status) -> None:
    if not self._running or status is None:
      return
    frame = self._frame_from_status(status)
    with self._lock:
      self.episode.append(frame)

  def start(self, task: Optional[str] = None) -> None:
    if task:
      self.task = task
      self.episode.task = task
    self.episode = Episode(task=self.task, fps=self.fps)
    self._running = True
    if not self._attached:
      existing = getattr(self.robot, "_callback", None)

      def chained(status):
        if existing:
          existing(status)
        self._on_status(status)

      self.robot.StartFeedbackMonitor(callback=chained)
      self._attached = True

  def capture(self) -> Optional[EpisodeFrame]:
    status = self.robot.GetStatus()
    if status is None:
      return None
    frame = self._frame_from_status(status)
    with self._lock:
      if self._running:
        self.episode.append(frame)
    return frame

  def stop(self) -> Episode:
    self._running = False
    with self._lock:
      return self.episode

  def save(self, path=None):
    ep = self.stop()
    if path is None:
      path = next_episode_path(default_dataset_dir(self.cfg))
    ep.save(path)
    return path, ep

  def export_lerobot(self, dest, episode: Optional[Episode] = None):
    """Write a Hub-ready LeRobot folder (30 FPS H.264). Native npz is unchanged.

    Does not pass `fps=episode.fps` (125). With camera samples the writer already
    resamples telemetry onto `video_fps`; without a camera, parquet still uses
    `hub_fps` so Hub rows are not 125 Hz.
    """
    ep = episode if episode is not None else self.episode
    has_video = bool(getattr(ep, "camera_samples", None)) or bool(getattr(ep, "videos", None))
    kwargs = {"video_fps": float(self.hub_fps)}
    if not has_video:
      kwargs["fps"] = float(self.hub_fps)
    return write_lerobot_dataset([ep], dest, **kwargs)
