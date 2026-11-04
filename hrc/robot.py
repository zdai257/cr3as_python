"""CS3AS robot: DobotRobot plus gripper, skins, kinematics, trajectory execution."""

from __future__ import annotations

import time
from typing import Optional, Sequence

import numpy as np

from dobot_sdk import CoordinateType, DobotRobot, load_robot_config

from .gripper import PGEAGripper
from .perception.skins import SafeSkinFrontend
from .planning.base import Trajectory
from .utils.dashboard import as_vector, parse_dashboard
from .utils.kinematics import Kinematics


class CS3ASRobot(DobotRobot):
  """Interoperable CS3AS cell: arm + DH PGEA-50-26 + safe skins."""

  def __init__(self, config=None, **kwargs):
    self.cfg = config or load_robot_config()
    super().__init__(
      ip=kwargs.get("ip", self.cfg.ip),
      dashboard_port=kwargs.get("dashboard_port", self.cfg.dashboard_port),
      feedback_port=kwargs.get("feedback_port", self.cfg.feedback_port),
      feedback_port_30005=kwargs.get(
        "feedback_port_30005", self.cfg.robot.get("feedback_port_30005")
      ),
      feedback_port_30006=kwargs.get(
        "feedback_port_30006", self.cfg.robot.get("feedback_port_30006")
      ),
      connect_timeout=kwargs.get("connect_timeout", self.cfg.connect_timeout),
      receive_timeout=kwargs.get("receive_timeout", self.cfg.receive_timeout),
    )
    self.gripper = PGEAGripper(self, self.cfg.gripper)
    self.skins = SafeSkinFrontend(self, self.cfg.skins)
    self.kin = Kinematics(self.cfg.kinematics, robot=self)

  def prepare(self, enable: bool = True, connect_gripper: bool = True) -> None:
    self.robot_control.RequestControl()
    self.robot_control.ClearError()
    self.StartFeedbackMonitor()
    if self.cfg.skins.enabled:
      self.skins.enable()
    if enable:
      self.robot_control.EnableRobot(load=float(self.cfg.robot.payload_kg))
      self.robot_control.SpeedFactor(int(self.cfg.robot.speed_factor))
      self.wait_idle()
    if connect_gripper and self.cfg.gripper.enabled:
      self.gripper.connect()

  def shutdown(self) -> None:
    try:
      if self.gripper.connected:
        self.gripper.open()
        self.gripper.disconnect()
    except Exception:
      pass
    try:
      self.robot_control.DisableRobot()
    except Exception:
      pass
    try:
      self.StopFeedbackMonitor()
    except Exception:
      pass

  def go_home(self) -> str:
    joints = [float(v) for v in self.cfg.robot.home_joints]
    response = self.motion.MovJ(joints, CoordinateType.JOINT)
    self.wait_idle()
    return response

  def current_pose(self) -> np.ndarray:
    return np.asarray(as_vector(self.robot_control.GetPose()), dtype=float)

  def current_joints(self) -> np.ndarray:
    return np.asarray(as_vector(self.robot_control.GetAngle()), dtype=float)

  def inverse_kin(self, pose: Sequence[float], joint_near: Optional[Sequence[float]] = None):
    return self.kin.controller_ik(pose, joint_near=joint_near)

  def forward_kin(self, joints: Sequence[float]):
    return self.kin.controller_fk(joints)

  def wait_idle(self, timeout: float = 30.0, poll: float = 0.05) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
      response = self.robot_control.RobotMode()
      try:
        error_id, values, _ = parse_dashboard(response)
        mode = int(values[0]) if values else -1
      except (ValueError, IndexError):
        error_id, mode = -1, -1
      if error_id == 0 and mode in (4, 5):
        # 4 = disabled idle, 5 = enabled idle
        if mode == 5 or not self._enabled():
          return True
      if mode == 9:
        raise RuntimeError(f"robot alarm while waiting: {response}")
      time.sleep(poll)
    return False

  def execute_trajectory(self, traj: Trajectory, wait: bool = True) -> None:
    if not traj.success:
      raise RuntimeError(f"refusing to execute failed plan: {traj.reason}")
    for wp in traj.waypoints:
      vector = [float(v) for v in wp.vector]
      kwargs = {}
      if wp.speed > 0:
        kwargs["v"] = int(wp.speed)
      if wp.space == "joint":
        self.motion.MovJ(vector, CoordinateType.JOINT, **kwargs)
      elif wp.interp == "movj":
        self.motion.MovJ(vector, CoordinateType.CARTESIAN, **kwargs)
      else:
        self.motion.MovL(vector, CoordinateType.CARTESIAN, **kwargs)
      if wait:
        self.wait_idle()
      if self.skins.triggered():
        self.robot_control.Pause()
        raise RuntimeError("safe-skin approach triggered; motion paused")

  def _enabled(self) -> bool:
    status = self.GetStatus()
    return bool(status and status.enable_status)

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.shutdown()
    return super().__exit__(exc_type, exc_val, exc_tb)
