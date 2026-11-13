"""CS3AS robot: DobotRobot plus gripper, skins, kinematics, trajectory execution."""

from __future__ import annotations

import time
from typing import Optional, Sequence

import numpy as np

from dobot_sdk import CoordinateType, DobotRobot, load_robot_config
from dobot_sdk.api.tcp_control import mode_name

from .gripper import PGEAGripper
from .perception.skins import SKIN_PARTS, SafeSkinFrontend
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
    self.kin = Kinematics(
      self.cfg.kinematics, robot=self, gripper_model=self.gripper.model
    )
    self._session_enabled = False

  def prepare(self, enable: bool = True, connect_gripper: bool = True) -> None:
    """Take exclusive TCP control, clear latched safety, then arm the cell.

    SafeSkin Set*/Enable* commands are queued and only flush while the arm
    is enabled, so sensing sessions must EnableRobot (idle, no motion).
    """
    self._session_enabled = bool(enable)
    self.StartFeedbackMonitor()
    time.sleep(0.2)
    self.take_tcp_control(keep_enabled=enable)
    self.reset_safety()
    if enable:
      self.ensure_enabled(load=float(self.cfg.robot.payload_kg))
      self.robot_control.SpeedFactor(int(self.cfg.robot.speed_factor))
      self.wait_idle()
    self.apply_tool_frame()
    if self.cfg.skins.enabled:
      self.skins.enable()
    if connect_gripper and self.cfg.gripper.enabled:
      self.gripper.connect()

  def shutdown(self, disable: bool = False, skins_off: bool = True) -> None:
    """Drop the TCP sockets. Default: do not DisableRobot.

    DisableRobot / Online takeover after a SafeSkin HIT is what kills the
    next stream. We ClearError, optionally turn SafeSkin off *while still
    in TCP* so the pendant does not inherit a latched HIT, then disconnect.
    """
    try:
      if self.gripper.connected:
        self.gripper.open()
        self.gripper.disconnect()
    except Exception:
      pass
    try:
      self.reset_safety()
    except Exception:
      pass
    if skins_off:
      try:
        if self.cfg.skins.enabled:
          self.skins.disable()
      except Exception:
        pass
      try:
        self.robot_control.ClearError()
      except Exception:
        pass
    if disable:
      try:
        self.robot_control.DisableRobot()
      except Exception:
        pass
      try:
        self.robot_control.ClearError()
      except Exception:
        pass
    try:
      self.StopFeedbackMonitor()
    except Exception:
      pass

  def recover_if_protected(self) -> int:
    """If a SafeSkin HIT latched pause/error/disable, stay in TCP and re-enable."""
    status = self.GetStatus()
    if status is not None:
      mode = int(status.robot_mode)
      collision = int(getattr(status, "collision_state", 0) or 0)
    else:
      mode = self.robot_mode_value()
      collision = 0
    hit = False
    try:
      reading = self.skins.read(status, clear=False)
      hit = bool(reading is not None and reading.triggered())
    except Exception:
      hit = False
    latched = collision or mode in (9, 10) or (mode == 4 and hit)
    if not latched:
      return mode
    mode = self.reset_safety()
    if self.skins.firmware_safety:
      return mode
    mode = self.ensure_enabled(load=float(self.cfg.robot.payload_kg))
    if self.cfg.skins.enabled:
      self.skins.enable()
    return mode

  def cell_status(self) -> dict:
    """Arm + SafeSkin + gripper snapshot for UI / examples."""
    status = self.GetStatus()
    mode = self.robot_mode_value()
    reading = self.skins.latest() if self.skins.enabled else None
    gripper = {}
    try:
      gripper = self.gripper.as_dict()
    except Exception as exc:
      gripper = {"connected": self.gripper.connected, "last_error": str(exc)}
    return {
      "ip": self.ip,
      "tcp": self.in_tcp_mode(),
      "mode": mode,
      "mode_name": mode_name(mode),
      "enabled": bool(status.enable_status) if status else False,
      "collision_state": int(getattr(status, "collision_state", 0) or 0) if status else 0,
      "safety": status.get_safety_state_desc() if status else "unknown",
      "skins": self.skins.as_dict(reading),
      "gripper": gripper,
      "skin_parts": list(SKIN_PARTS),
    }

  def apply_tool_frame(self) -> None:
    """Load the PGEA-50-26 TCP (x=0, y=0, z=118 mm) onto the controller."""
    gripper_cfg = self.cfg.gripper
    apply = True
    if hasattr(gripper_cfg, "get"):
      apply = bool(gripper_cfg.get("apply_tool", True))
    if not apply:
      return
    index = int(gripper_cfg.get("tool_index", 1)) if hasattr(gripper_cfg, "get") else 1
    pose = [float(v) for v in self.gripper.model.tool_pose()]
    persist = gripper_cfg.get("tool_persist", 0) if hasattr(gripper_cfg, "get") else 0
    self.robot_control.SetTool(index, pose, type=int(persist) if persist is not None else 0)
    self.robot_control.Tool(index)

  def observation(self, extra: Optional[dict] = None):
    from .world import Observation, WorldState

    extra = extra or {}
    joints = extra.get("joints")
    pose = extra.get("pose")
    try:
      if joints is None:
        joints = self.current_joints()
      if pose is None:
        pose = self.current_pose()
    except Exception:
      joints = np.zeros(6) if joints is None else joints
      pose = np.zeros(6) if pose is None else pose
    reading = self.skins.read()
    grip = extra.get("gripper_position")
    if grip is None and self.gripper.connected:
      try:
        grip = self.gripper.position()
      except Exception:
        grip = None
    return Observation(
      timestamp=time.time(),
      joints=np.asarray(joints, dtype=float),
      pose=np.asarray(pose, dtype=float),
      world=WorldState(margin_mm=float(self.cfg.hrc.collision_margin_mm)),
      skin_conductance=None if reading is None else reading.conductance,
      skin_approach=None if reading is None else reading.approach,
      skin_proximity=None if reading is None else reading.proximity,
      skin_reading=reading,
      gripper_position=grip,
    )

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
    recovers = 0
    while time.time() < deadline:
      response = self.robot_control.RobotMode()
      try:
        error_id, values, _ = parse_dashboard(response)
        mode = int(values[0]) if values else -1
      except (ValueError, IndexError):
        error_id, mode = -1, -1
      if error_id == 0 and mode == 5:
        return True
      if error_id == 0 and mode == 4 and not self._session_enabled:
        return True
      if mode in (9, 10) and not self.skins.firmware_safety and recovers < 3:
        recovers += 1
        self.recover_if_protected()
        time.sleep(poll)
        continue
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
