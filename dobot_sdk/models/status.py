# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Robot status models
"""

from dataclasses import dataclass, field
from typing import List
from enum import IntEnum


class RobotMode(IntEnum):
  """Robot operating mode
  
  See the TCP/IP protocol document
  """
  UNKNOWN = 0     # Unknown / uninitialized (may appear in Feedback)
  INIT = 1      # Initializing
  BRAKE_OPEN = 2   # Any joint brake released
  POWEROFF = 3    # Powered off
  DISABLED = 4    # Disabled (brakes engaged)
  ENABLE = 5     # Enabled and idle
  BACKDRIVE = 6   # Backdrive / drag mode
  RUNNING = 7    # Running (project or TCP queued motion)
  SINGLE_MOVE = 8  # Single move (jog, RunTo, ...)
  ERROR = 9     # Alarm
  PAUSE = 10     # Paused
  JOG = 11      # Jogging


@dataclass
class JointState:
  """Joint state"""
  q_actual: List[float] = field(default_factory=lambda: [0.0] * 6) # Actual angles
  q_target: List[float] = field(default_factory=lambda: [0.0] * 6) # Target angles
  qd_actual: List[float] = field(default_factory=lambda: [0.0] * 6) # Actual velocity
  qd_target: List[float] = field(default_factory=lambda: [0.0] * 6) # Target velocity
  qdd_target: List[float] = field(default_factory=lambda: [0.0] * 6) # Target acceleration
  i_actual: List[float] = field(default_factory=lambda: [0.0] * 6) # Actual current
  i_target: List[float] = field(default_factory=lambda: [0.0] * 6) # Target current
  m_actual: List[float] = field(default_factory=lambda: [0.0] * 6) # Actual torque
  m_target: List[float] = field(default_factory=lambda: [0.0] * 6) # Target torque
  temperatures: List[float] = field(default_factory=lambda: [0.0] * 6) # Motor temperature
  joint_modes: List[float] = field(default_factory=lambda: [0.0] * 6) # Joint control mode
  voltages: List[float] = field(default_factory=lambda: [0.0] * 6) # Joint voltage


@dataclass
class CartesianPose:
  """Cartesian pose"""
  x: float = 0.0
  y: float = 0.0
  z: float = 0.0
  rx: float = 0.0
  ry: float = 0.0
  rz: float = 0.0
  
  def to_list(self) -> List[float]:
    """Convert to list"""
    return [self.x, self.y, self.z, self.rx, self.ry, self.rz]


@dataclass
class Quaternion:
  """Quaternion [qw, qx, qy, qz]"""
  qw: float = 0.0
  qx: float = 0.0
  qy: float = 0.0
  qz: float = 0.0
  
  def to_list(self) -> List[float]:
    """Convert to list"""
    return [self.qw, self.qx, self.qy, self.qz]


@dataclass
class RobotStatus:
  """
  Full robot status
  
  Parsed from a 1440-byte Feedback packet
  """
  # Basic status
  robot_mode: RobotMode = RobotMode.INIT
  speed_scaling: float = 0.0 # Speed scale 0-100
  enable_status: bool = False # Enable status
  brake_status: bool = False # Brake status
  error_status: bool = False # Error status
  running_status: int = 0 # Running status (offset 1028; 0=idle, nonzero=moving)
  drag_status: int = 0 # Drag status (0=none, 1=joint drag, 2=force drag)
  jog_status: int = 0 # Jogging
  robot_type: int = 0 # Robot model id
  
  # IO status
  digital_inputs: int = 0 # Digital inputs (64-bit)
  digital_outputs: int = 0 # Digital outputs (64-bit)
  safety_io_in: int = 0 # Safety IO in
  safety_io_out: int = 0 # Safety IO out
  
  # Timing
  timestamp: int = 0 # Unix timestamp (ms)
  run_time: int = 0 # Uptime (ms)
  
  # Electrical
  voltage: float = 0.0 # Robot voltage
  current: float = 0.0 # Robot current
  
  # Program state
  program_state: float = 0.0 # Script run state
  
  # Current command ID
  current_command_id: int = 0
  
  # Joint state
  joint_state: JointState = field(default_factory=JointState)
  
  # Cartesian pose
  tool_vector_actual: CartesianPose = field(default_factory=CartesianPose)
  tool_vector_target: CartesianPose = field(default_factory=CartesianPose)
  tcp_speed_actual: CartesianPose = field(default_factory=CartesianPose) # Actual TCP velocity
  tcp_speed_target: CartesianPose = field(default_factory=CartesianPose) # Target TCP velocity
  
  # Force
  actual_tcp_force: List[float] = field(default_factory=lambda: [0.0] * 6) # TCP wrench (FT sensor)
  tcp_force: List[float] = field(default_factory=lambda: [0.0] * 6) # TCP force (from joint current)
  
  # Payload
  load: float = 0.0 # Tool payload (kg)
  load_center_x: float = 0.0 # Payload CoM X (mm)
  load_center_y: float = 0.0 # Payload CoM Y (mm)
  load_center_z: float = 0.0 # Payload CoM Z (mm)
  
  # Frames
  user_coordinate: int = 0 # User frame index
  tool_coordinate: int = 0 # Tool frame index
  user_value: List[float] = field(default_factory=lambda: [0.0] * 6) # User-frame values
  tool_value: List[float] = field(default_factory=lambda: [0.0] * 6) # Tool-frame values
  
  # Quaternion
  target_quaternion: Quaternion = field(default_factory=Quaternion) # Target quaternion
  actual_quaternion: Quaternion = field(default_factory=Quaternion) # Actual quaternion
  
  # Velocity / accel scales
  velocity_ratio: int = 0 # Joint velocity scale (0-100)
  acceleration_ratio: int = 0 # Joint accel scale (0-100)
  xyz_velocity_ratio: int = 0 # Cartesian linear-velocity scale (0-100)
  r_velocity_ratio: int = 0 # Cartesian angular-velocity scale (0-100)
  xyz_acceleration_ratio: int = 0 # Cartesian linear-accel scale (0-100)
  r_acceleration_ratio: int = 0 # Cartesian angular-accel scale (0-100)
  
  # Queue status
  run_queued_cmd: int = 0 # Queued-command running flag
  pause_cmd_flag: int = 0 # Queued-command pause flag
  
  # Arm config
  hand_type: List[int] = field(default_factory=lambda: [0] * 4) # Arm config (reserved)
  
  # Tool-button signals
  drag_button_signal: int = 0 # Drag-button signal
  enable_button_signal: int = 0 # Enable-button signal
  record_button_signal: int = 0 # Record-button signal
  reappear_button_signal: int = 0 # Playback-button signal
  jaw_button_signal: int = 0 # Gripper-button signal
  
  # Six-axis FT sensor
  six_force_online: int = 0 # FT online (0=offline, 1=online, 2=fault)
  six_force_value: List[float] = field(default_factory=lambda: [0.0] * 6) # Raw FT values
  
  # Safety status
  collision_state: int = 0 # Collision status
  arm_approach_state: int = 0 # Forearm safe-skin approach pause
  j4_approach_state: int = 0 # J4 safe-skin approach pause
  j5_approach_state: int = 0 # J5 safe-skin approach pause
  j6_approach_state: int = 0 # J6 safe-skin approach pause
  safety_state: int = 0 # Safety bitmask
  
  # Vibration
  vibration_dis_z: float = 0.0 # Z vibration displacement
  
  # Mode
  auto_manual_mode: int = 0 # Manual / auto mode
  export_status: int = 0 # USB export status
  
  def is_ready(self) -> bool:
    """True if the robot is ready"""
    return (
      self.robot_mode == RobotMode.ENABLE and
      not self.error_status and
      self.enable_status
    )
  
  def has_error(self) -> bool:
    """True if there is an error"""
    return self.robot_mode == RobotMode.ERROR or self.error_status
  
  def is_moving(self) -> bool:
    """True if the robot is moving
    
    RunningStatus byte offset 1028
    Values:
    - 0: idle
    - nonzero: moving (see protocol)
    """
    return self.running_status != 0
  
  def get_safety_state_desc(self) -> str:
    """Describe the safety bitmask"""
    states = []
    if self.safety_state & 0x01: states.append("E-stop")
    if self.safety_state & 0x02: states.append("Protective stop")
    if self.safety_state & 0x04: states.append("Reduced mode")
    if self.safety_state & 0x08: states.append("Not stopped")
    if self.safety_state & 0x10: states.append("Moving")
    if self.safety_state & 0x20: states.append("System e-stop")
    if self.safety_state & 0x40: states.append("User e-stop")
    if self.safety_state & 0x80: states.append("Safe origin")
    return ", ".join(states) if states else "Normal"
