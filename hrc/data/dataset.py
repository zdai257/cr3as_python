"""Native episode store (npz + json). Convert to LeRobot / rosbag from here."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Union

import numpy as np

from .frame import ACTION_NAMES, STATE_NAMES, CameraSample, EpisodeFrame

ARRAY_KEYS = (
  "timestamp",
  "timestamp_ms",
  "joints",
  "joints_target",
  "joint_vel",
  "pose",
  "pose_target",
  "tcp_vel",
  "gripper_position",
  "gripper_command",
  "skin_approach",
  "skin_proximity",
  "skin_conductance",
  "skin_analog",
  "skin_distance_mm",
  "action",
  "robot_mode",
  "collision_state",
)


def default_dataset_dir(cfg=None) -> Path:
  root = Path(__file__).resolve().parents[2] / "data" / "episodes"
  if cfg is None:
    return root
  hrc = cfg.hrc if hasattr(cfg, "hrc") else cfg
  learning = hrc.learning if hasattr(hrc, "learning") else None
  if learning is None:
    return root
  path = learning.get("dataset_dir") if hasattr(learning, "get") else getattr(learning, "dataset_dir", None)
  if not path:
    return root
  p = Path(str(path))
  return p if p.is_absolute() else Path(__file__).resolve().parents[2] / p


@dataclass
class Episode:
  frames: List[EpisodeFrame] = field(default_factory=list)
  task: str = "cs3as_hrc"
  robot_type: str = "dobot_cs3as_pgea"
  fps: float = 125.0
  episode_index: int = 0
  extras: Dict[str, Any] = field(default_factory=dict)
  camera_samples: List[CameraSample] = field(default_factory=list)

  def __len__(self) -> int:
    return len(self.frames)

  def append(self, frame: EpisodeFrame) -> None:
    self.frames.append(frame)

  def timestamps(self) -> np.ndarray:
    return np.asarray([f.timestamp for f in self.frames], dtype=np.float64)

  def states(self) -> np.ndarray:
    if not self.frames:
      return np.zeros((0, len(STATE_NAMES)), dtype=np.float32)
    return np.stack([f.state_vector() for f in self.frames]).astype(np.float32)

  def actions(self) -> np.ndarray:
    if not self.frames:
      return np.zeros((0, len(ACTION_NAMES)), dtype=np.float32)
    return np.stack([f.action for f in self.frames]).astype(np.float32)

  def skin_proximity(self) -> np.ndarray:
    if not self.frames:
      return np.zeros((0, 4), dtype=np.float32)
    return np.stack([f.skin_proximity for f in self.frames]).astype(np.float32)

  def skin_analog(self) -> np.ndarray:
    if not self.frames:
      return np.zeros((0, 4), dtype=np.float32)
    return np.stack([f.skin_analog for f in self.frames]).astype(np.float32)

  def skin_distance(self) -> np.ndarray:
    if not self.frames:
      return np.zeros((0, 4), dtype=np.float32)
    return np.stack([f.skin_distance_mm for f in self.frames]).astype(np.float32)

  def skin_approach(self) -> np.ndarray:
    if not self.frames:
      return np.zeros((0, 4), dtype=np.float32)
    return np.stack([f.skin_approach for f in self.frames]).astype(np.float32)

  def as_arrays(self) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    if not self.frames:
      return {k: np.zeros((0,)) for k in ARRAY_KEYS}
    data["timestamp"] = self.timestamps()
    data["timestamp_ms"] = np.asarray([f.timestamp_ms for f in self.frames], dtype=np.int64)
    data["joints"] = np.stack([f.joints for f in self.frames])
    data["joints_target"] = np.stack([f.joints_target for f in self.frames])
    data["joint_vel"] = np.stack([f.joint_vel for f in self.frames])
    data["pose"] = np.stack([f.pose for f in self.frames])
    data["pose_target"] = np.stack([f.pose_target for f in self.frames])
    data["tcp_vel"] = np.stack([f.tcp_vel for f in self.frames])
    data["gripper_position"] = np.asarray([f.gripper_position for f in self.frames], dtype=np.float32)
    data["gripper_command"] = np.asarray([f.gripper_command for f in self.frames], dtype=np.float32)
    data["skin_approach"] = np.stack([f.skin_approach for f in self.frames])
    data["skin_proximity"] = np.stack([f.skin_proximity for f in self.frames])
    data["skin_conductance"] = np.stack([f.skin_conductance for f in self.frames])
    data["skin_analog"] = np.stack([f.skin_analog for f in self.frames])
    data["skin_distance_mm"] = np.stack([f.skin_distance_mm for f in self.frames])
    data["action"] = self.actions()
    data["robot_mode"] = np.asarray([f.robot_mode for f in self.frames], dtype=np.int32)
    data["collision_state"] = np.asarray([f.collision_state for f in self.frames], dtype=np.int32)
    return data

  def meta(self) -> Dict[str, Any]:
    return {
      "task": self.task,
      "robot_type": self.robot_type,
      "fps": self.fps,
      "episode_index": self.episode_index,
      "n_frames": len(self.frames),
      "state_names": list(STATE_NAMES),
      "action_names": list(ACTION_NAMES),
      "extras": self.extras,
    }

  def save(self, path: Union[str, Path]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **self.as_arrays())
    path.with_suffix(".json").write_text(json.dumps(self.meta(), indent=2), encoding="utf-8")
    return path

  @classmethod
  def load(cls, path: Union[str, Path]) -> "Episode":
    path = Path(path)
    packed = np.load(path, allow_pickle=False)
    meta_path = path.with_suffix(".json")
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
    n = int(packed["timestamp"].shape[0])
    frames = []
    for i in range(n):
      frames.append(
        EpisodeFrame(
          timestamp=float(packed["timestamp"][i]),
          timestamp_ms=int(packed["timestamp_ms"][i]),
          joints=np.asarray(packed["joints"][i], dtype=np.float32),
          joints_target=np.asarray(packed["joints_target"][i], dtype=np.float32),
          joint_vel=np.asarray(packed["joint_vel"][i], dtype=np.float32),
          pose=np.asarray(packed["pose"][i], dtype=np.float32),
          pose_target=np.asarray(packed["pose_target"][i], dtype=np.float32),
          tcp_vel=np.asarray(packed["tcp_vel"][i], dtype=np.float32),
          gripper_position=float(packed["gripper_position"][i]),
          gripper_command=float(packed["gripper_command"][i]),
          skin_approach=np.asarray(packed["skin_approach"][i], dtype=np.float32),
          skin_proximity=np.asarray(packed["skin_proximity"][i], dtype=np.float32),
          skin_conductance=np.asarray(packed["skin_conductance"][i], dtype=np.float32),
          skin_analog=np.asarray(packed["skin_analog"][i], dtype=np.float32),
          skin_distance_mm=(
            np.asarray(packed["skin_distance_mm"][i], dtype=np.float32)
            if "skin_distance_mm" in packed.files
            else (1.0 - np.clip(np.asarray(packed["skin_proximity"][i], dtype=np.float32), 0.0, 1.0)) * 150.0
          ),
          action=np.asarray(packed["action"][i], dtype=np.float32),
          robot_mode=int(packed["robot_mode"][i]),
          collision_state=int(packed["collision_state"][i]),
        )
      )
    return cls(
      frames=frames,
      task=str(meta.get("task", "cs3as_hrc")),
      robot_type=str(meta.get("robot_type", "dobot_cs3as_pgea")),
      fps=float(meta.get("fps", 125.0)),
      episode_index=int(meta.get("episode_index", 0)),
      extras=dict(meta.get("extras") or {}),
    )

  @classmethod
  def synthetic(cls, n: int = 20, fps: float = 125.0, task: str = "synthetic") -> "Episode":
    ep = cls(task=task, fps=fps)
    t0 = time.time()
    for i in range(n):
      t = i / fps
      joints = np.array([10.0 * np.sin(t + k * 0.2) for k in range(6)], dtype=np.float32)
      prox = np.array([max(0.0, np.sin(t * 2 + k)) for k in range(4)], dtype=np.float32)
      frame = EpisodeFrame(
        timestamp=t0 + t,
        timestamp_ms=int((t0 + t) * 1000),
        joints=joints,
        joints_target=joints,
        pose=np.array([320.0, 10.0 * i, 280.0, 180.0, 0.0, 180.0], dtype=np.float32),
        gripper_position=float((i % 11) * 100),
        gripper_command=float((i % 11) * 100),
        skin_proximity=prox,
        skin_conductance=prox,
        skin_approach=(prox > 0.8).astype(np.float32),
        skin_analog=(prox * 1023.0).astype(np.float32),
        skin_distance_mm=((1.0 - prox) * 150.0).astype(np.float32),
        action=np.concatenate([joints, np.array([float((i % 11) * 100)], dtype=np.float32)]),
      )
      ep.append(frame)
    return ep


def next_episode_path(directory: Union[str, Path], prefix: str = "episode") -> Path:
  directory = Path(directory)
  directory.mkdir(parents=True, exist_ok=True)
  existing = sorted(directory.glob(f"{prefix}_*.npz"))
  idx = 0
  if existing:
    try:
      idx = int(existing[-1].stem.split("_")[-1]) + 1
    except ValueError:
      idx = len(existing)
  return directory / f"{prefix}_{idx:04d}.npz"
