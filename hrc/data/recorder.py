"""Live episode recorder from CS3AS Feedback (port 30004, 8 ms)."""

from __future__ import annotations

import threading
import time
from typing import Optional

from .dataset import Episode, default_dataset_dir, next_episode_path
from .frame import EpisodeFrame


class EpisodeRecorder:
  def __init__(self, robot, cfg=None, fps: Optional[float] = None, task: Optional[str] = None):
    self.robot = robot
    self.cfg = cfg or getattr(robot, "cfg", None)
    learning = None
    if self.cfg is not None and hasattr(self.cfg, "hrc"):
      learning = getattr(self.cfg.hrc, "learning", None)
    self.fps = float(fps if fps is not None else (learning.fps if learning else 125.0))
    self.task = str(task if task is not None else (learning.task if learning else "cs3as_hrc"))
    self.episode = Episode(task=self.task, fps=self.fps)
    self._lock = threading.Lock()
    self._running = False
    self._attached = False
    self._last_action = None
    self._last_grip_cmd = None

  def set_action(self, action, gripper_command: Optional[float] = None) -> None:
    import numpy as np

    with self._lock:
      self._last_action = np.asarray(action, dtype=float).reshape(-1)
      if gripper_command is not None:
        self._last_grip_cmd = float(gripper_command)

  def _frame_from_status(self, status) -> EpisodeFrame:
    grip = None
    try:
      if getattr(self.robot, "gripper", None) is not None:
        grip = self.robot.gripper.position()
    except Exception:
      grip = None
    skin = None
    try:
      skin = self.robot.skins.read(status)
    except Exception:
      skin = None
    with self._lock:
      action = self._last_action
      grip_cmd = self._last_grip_cmd
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
