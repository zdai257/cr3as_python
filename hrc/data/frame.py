"""One synchronized sample of the CS3AS cell for learning-based control."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import numpy as np

from dobot_sdk.models.status import RobotStatus

from ..perception.skins import SkinReading

STATE_NAMES = (
  "j1", "j2", "j3", "j4", "j5", "j6",
  "x", "y", "z", "rx", "ry", "rz",
  "gripper",
  "skin_forearm", "skin_j4", "skin_j5", "skin_j6",
)
ACTION_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6", "gripper")
SKIN_PARTS = ("forearm", "j4", "j5", "j6")


def _f(values, n: int, default: float = 0.0) -> np.ndarray:
  if values is None:
    return np.full(n, default, dtype=np.float32)
  arr = np.asarray(values, dtype=np.float32).reshape(-1)
  if arr.size >= n:
    return arr[:n].copy()
  return np.pad(arr, (0, n - arr.size))


@dataclass
class EpisodeFrame:
  timestamp: float
  timestamp_ms: int = 0
  joints: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float32))
  joints_target: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float32))
  joint_vel: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float32))
  pose: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float32))
  pose_target: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float32))
  tcp_vel: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float32))
  gripper_position: float = 0.0
  gripper_command: float = 0.0
  skin_approach: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))
  skin_proximity: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))
  skin_conductance: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))
  skin_analog: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))
  skin_distance_mm: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))
  action: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float32))
  robot_mode: int = 0
  collision_state: int = 0
  extra: Dict[str, Any] = field(default_factory=dict)

  @classmethod
  def from_status(
    cls,
    status: RobotStatus,
    *,
    gripper_position: Optional[float] = None,
    gripper_command: Optional[float] = None,
    action: Optional[np.ndarray] = None,
    skin: Optional[SkinReading] = None,
    wall_time: Optional[float] = None,
  ) -> "EpisodeFrame":
    js = status.joint_state
    pose = status.tool_vector_actual
    pose_t = status.tool_vector_target
    tcp = status.tcp_speed_actual
    if skin is None:
      skin = SkinReading.from_status(status)
    grip = 0.0 if gripper_position is None else float(gripper_position)
    cmd = grip if gripper_command is None else float(gripper_command)
    joints = _f(js.q_actual, 6)
    act = _f(action, 7) if action is not None else np.concatenate([joints, np.array([cmd], dtype=np.float32)])
    return cls(
      timestamp=float(wall_time if wall_time is not None else (status.timestamp / 1000.0)),
      timestamp_ms=int(status.timestamp),
      joints=joints,
      joints_target=_f(js.q_target, 6),
      joint_vel=_f(js.qd_actual, 6),
      pose=_f(pose.to_list(), 6),
      pose_target=_f(pose_t.to_list(), 6),
      tcp_vel=_f(tcp.to_list(), 6),
      gripper_position=grip,
      gripper_command=cmd,
      skin_approach=_f(skin.approach if skin else None, 4),
      skin_proximity=_f(skin.proximity if skin else None, 4),
      skin_conductance=_f(skin.conductance if skin else None, 4),
      skin_analog=_f(skin.analog if skin else getattr(status, "skin_analog", None), 4),
      skin_distance_mm=_f(skin.distance_mm() if skin is not None else None, 4),
      action=act.astype(np.float32),
      robot_mode=int(status.robot_mode),
      collision_state=int(status.collision_state),
    )

  def state_vector(self) -> np.ndarray:
    return np.concatenate(
      [
        self.joints,
        self.pose,
        np.array([self.gripper_position], dtype=np.float32),
        self.skin_proximity.astype(np.float32),
      ]
    ).astype(np.float32)

  def as_dict(self) -> Dict[str, Any]:
    return {
      "timestamp": self.timestamp,
      "timestamp_ms": self.timestamp_ms,
      "joints": self.joints.tolist(),
      "joints_target": self.joints_target.tolist(),
      "joint_vel": self.joint_vel.tolist(),
      "pose": self.pose.tolist(),
      "pose_target": self.pose_target.tolist(),
      "tcp_vel": self.tcp_vel.tolist(),
      "gripper_position": self.gripper_position,
      "gripper_command": self.gripper_command,
      "skin_approach": self.skin_approach.tolist(),
      "skin_proximity": self.skin_proximity.tolist(),
      "skin_conductance": self.skin_conductance.tolist(),
      "skin_analog": self.skin_analog.tolist(),
      "skin_distance_mm": self.skin_distance_mm.tolist(),
      "action": self.action.tolist(),
      "robot_mode": self.robot_mode,
      "collision_state": self.collision_state,
    }


@dataclass
class CameraSample:
  timestamp: float
  jpeg: bytes
  width: int
  height: int
