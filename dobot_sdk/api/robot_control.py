# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Robot Controlmodule - status, kinematics, reachability, robot control"""

from typing import Tuple, Sequence
from ..core.connection import DobotConnection


class RobotControl:
  """Robot-control module - status, kinematics, reachability, logs, scripts"""

  def __init__(self, connection: DobotConnection):
    self.connection = connection

  def _send_cmd(self, command: str) -> str:
    """Send command and receive response"""
    return self.connection.send_receive_text(command)

  # ==================== Control mode and power====================

  def RequestControl(self) -> str:
    """
    RequestControlRequest TCP control mode（immediate command）
    
    Notes:
    - Other TCP commands require TCP mode
    - Switch to TCP only when powered-off or disabled (not paused / brake-released)
    - Call this before EnableRobot and motion commands
    
    Returns:
      str: ErrorID,{},RequestControl();
    """
    return self._send_cmd("RequestControl()")

  def PowerOn(self) -> str:
    """Power on the robot（immediate command）"""
    return self._send_cmd("PowerOn()")

  def EnableRobot(self, load: float = 0.0, **kwargs) -> str:
    """
    Enable the robot（immediate command）

    prototype：EnableRobot(load,centerX,centerY,centerZ,isCheck)

    Args:
      load: Payload mass (kg)
      centerX: Payload CoM X (mm)
      centerY: Payload CoM Y (mm)
      centerZ: Payload CoM Z (mm)
      isCheck: Check payload (0=no, 1=yes)
    """
    if load == 0.0:
      cmd = "EnableRobot()"
    elif 'centerX' not in kwargs:
      cmd = f"EnableRobot({load:.6f})"
    else:
      cx = kwargs.get('centerX', 0.0)
      cy = kwargs.get('centerY', 0.0)
      cz = kwargs.get('centerZ', 0.0)
      check = kwargs.get('isCheck', 0)
      cmd = f"EnableRobot({load:.6f},{cx:.6f},{cy:.6f},{cz:.6f},{check})"
    return self._send_cmd(cmd)

  def DisableRobot(self) -> str:
    """Disable the robot（immediate command）"""
    return self._send_cmd("DisableRobot()")

  def ClearError(self) -> str:
    """Clear robot alarms（immediate command）"""
    return self._send_cmd("ClearError()")

  # ==================== Motion control ====================

  def RunScript(self, script_name: str) -> str:
    """
    Run a project/script（immediate command）
    
    Args:
      script_name: Script file name    """
    return self._send_cmd(f"RunScript(\"{script_name}\")")

  def Stop(self) -> str:
    """Stop motion (or the running project)（immediate command）"""
    return self._send_cmd("Stop()")

  def Pause(self) -> str:
    """Pause motion (or the running project)（immediate command）"""
    return self._send_cmd("Pause()")

  def Continue(self) -> str:
    """Resume motion (or a paused project)（immediate command）"""
    return self._send_cmd("Continue()")

  def EmergencyStop(self, mode: int) -> str:
    """
    Emergency stop（immediate command）
    
    Args:
      mode: 1 = press e-stop, 0 = release e-stop
    """
    if mode not in [0, 1]:
      raise ValueError("mode must be 0 or 1")
    return self._send_cmd(f"EmergencyStop({mode})")

  # ==================== Brakes and drag====================

  def BrakeControl(self, axis_id: int, value: int) -> str:
    """
    Control the brake of one joint（immediate command）    
    Args:
      axis_id: Joint index (1-6)
      value: 0 = brake on, 1 = release
    """
    if not 1 <= axis_id <= 6:
      raise ValueError("Joint index must be in 1-6")
    if value not in [0, 1]:
      raise ValueError("value must be 0 or 1")
    return self._send_cmd(f"BrakeControl({axis_id},{value})")

  def StartDrag(self) -> str:
    """StartDragrobot enters jointBackdrive / drag mode（immediate command）"""
    return self._send_cmd("StartDrag()")

  def StopDrag(self) -> str:
    """StopDragrobot leavesBackdrive / drag mode（immediate command）"""
    return self._send_cmd("StopDrag()")

  def DragSensitivity(self, index: int, value: int) -> str:
    """
    DragSensitivitysetDragsensitivity（immediate command）
    
    Args:
      index: axis index, valuerange: [0,6]。0same sensitivity on every axis。1~6setsJ1~J6axis sensitivity。
      value: Dragsensitivity, valuesmaller, Draghigher drag resistance。valuerange: [1, 90]。
    """
    if not 0 <= index <= 6:
      raise ValueError("axis index must be in0-6")
    if not 1 <= value <= 90:
      raise ValueError("Dragsensitivity must be in1-90")
    return self._send_cmd(f"DragSensitivity({index},{value})")

  # ==================== speedand accelspeedset ====================

  def SpeedFactor(self, factor: int) -> str:
    """
    SpeedFactorSet global speed scale（immediate command）
    
    Args:
      factor: Speed scale (1-100)
    """
    if not 1 <= factor <= 100:
      raise ValueError("speedscale must be in1-100")
    return self._send_cmd(f"SpeedFactor({factor})")

  def AccJ(self, acc: int) -> str:
    """
    AccJset jointmovesaccel forspeedscale（immediate command）
    
    Args:
      acc: Accel scale (1-100)
    """
    if not 1 <= acc <= 100:
      raise ValueError(" accelspeedscale must be in1-100")
    return self._send_cmd(f"AccJ({acc})")

  def AccL(self, acc: int) -> str:
    """
    AccLset linear/arcmovesaccel forspeedscale（immediate command）
    
    Args:
      acc: Accel scale (1-100)
    """
    if not 1 <= acc <= 100:
      raise ValueError(" accelspeedscale must be in0-100")
    return self._send_cmd(f"AccL({acc})")

  def VelJ(self, vel: int) -> str:
    """
    VelJset jointmovesforspeedscale（immediate command）
    
    Args:
      vel: Speed scale (1-100)
    """
    if not 1 <= vel <= 100:
      raise ValueError("speedscale must be in0-100")
    return self._send_cmd(f"VelJ({vel})")

  def VelL(self, vel: int) -> str:
    """
    VelLset linear/arcmovesforspeedscale（immediate command）
    
    Args:
      vel: Speed scale (1-100)
    """
    if not 1 <= vel <= 100:
      raise ValueError("speedscale must be in0-100")
    return self._send_cmd(f"VelL({vel})")

  def CP(self, value: int) -> str:
    """
    CPset continuous-path scale（immediate command）
    
    Args:
      value: continuous-path scale (0-100)
    """
    if not 0 <= value <= 100:
      raise ValueError("continuous-path scale must be in0-100")
    return self._send_cmd(f"CP({value})")

  # ==================== Framesset====================

  def User(self, index: int) -> str:
    """
    Userset globalUser frame（queued command）    
    Args:
      index: User frame index (0-50)
    """
    if not 0 <= index <= 50:
      raise ValueError("User frame indexmust be in0-50")
    return self._send_cmd(f"User({index})")

  def SetUser(self, index: int, pose: Sequence[float], type: int = None) -> str:
    """
    SetUsermodifyUser frame（immediate command）
    
    Args:
      index: User frame index(1-50)
      pose: 6coordinate values[x,y,z,rx,ry,rz]
      type: whetherFramespersist globally。0: this command changesFramesonly in the current projectRunningtakes effect in。1: this command changesFrameswill be stored on the controller。
    """
    if not 1 <= index <= 50:
      raise ValueError("User frame indexmust be in1-50")
    if len(pose) != 6:
      raise ValueError("poseneeds6 values")
    pose_str = "{" + ",".join([f"{v:.6f}" for v in pose]) + "}"
    if type is not None:
      if type not in [0, 1]:
        raise ValueError("typemust be0 or 1")
      return self._send_cmd(f"SetUser({index},{pose_str},{type})")
    return self._send_cmd(f"SetUser({index},{pose_str})")

  def CalcUser(self, index: int, matrix_direction: int, offset: Sequence[float]) -> str:
    """
    CalcUsercomputeUser frame（immediate command）
    Args:
      index: User frame index (0-50)
      matrix_direction: multiply side (1-left-multiply，Framesabout the baseFramesrotation; 0-right-multiply，Framesrotate about itself)
      offset: offsetvalue [x,y,z,rx,ry,rz]
    """
    if not 0 <= index <= 50:
      raise ValueError("User frame indexmust be in0-50")
    if matrix_direction not in [0, 1]:
      raise ValueError("matrix_direction must be0 or 1")
    if len(offset) != 6:
      raise ValueError("offsetneeds6 values")
    offset_str = "{" + ",".join([f"{v:.6f}" for v in offset]) + "}"
    return self._send_cmd(f"CalcUser({index},{matrix_direction},{offset_str})")

  def Tool(self, index: int) -> str:
    """
    Toolset globalTool frame（queued command）    
    Args:
      index: Tool frame index (0-50)
    """
    if not 0 <= index <= 50:
      raise ValueError("Tool frame indexmust be in0-50")
    return self._send_cmd(f"Tool({index})")

  def SetTool(self, index: int, pose: Sequence[float], type: int = None) -> str:
    """
    Set a tool frame（immediate command）
    
    Args:
      index: Tool frame index(1-50)
      pose: 6coordinate values[x,y,z,rx,ry,rz]
      type: whetherFramespersist globally。0: this command changesFramesonly in the current projectRunningtakes effect in。1: this command changesFrameswill be stored on the controller。
    """
    if not 1 <= index <= 50:
      raise ValueError("Tool frame indexmust be in1-50")
    if len(pose) != 6:
      raise ValueError("poseneeds6 values")
    pose_str = "{" + ",".join([f"{v:.6f}" for v in pose]) + "}"
    if type is not None:
      if type not in [0, 1]:
        raise ValueError("typemust be0 or 1")
      return self._send_cmd(f"SetTool({index},{pose_str},{type})")
    return self._send_cmd(f"SetTool({index},{pose_str})")

  def CalcTool(self, index: int, matrix_direction: int, offset: Sequence[float]) -> str:
    """
    CalcToolcomputeTool frame（immediate command）
    Args:
      index: Tool frame index (0-50)
      matrix_direction: multiply side (1-left-multiply，Framesabout the flangeFramesrotation; 0-right-multiply，Framesrotate about itself)
      offset: offsetvalue [x,y,z,rx,ry,rz]
    """
    if not 0 <= index <= 50:
      raise ValueError("Tool frame indexmust be in0-50")
    if matrix_direction not in [0, 1]:
      raise ValueError("matrix_direction must be0 or 1")
    if len(offset) != 6:
      raise ValueError("offsetneeds6 values")
    offset_str = "{" + ",".join([f"{v:.6f}" for v in offset]) + "}"
    return self._send_cmd(f"CalcTool({index},{matrix_direction},{offset_str})")

  # ==================== Payloadset ====================

  def SetPayload(self, load_or_name, *args, **kwargs) -> str:
    """
    SetPayloadset toolPayload（queued command）

    two call styles（matches the protocol document）：
    form 1：SetPayload(load, x, y, z)
    form 2：SetPayload(name)

    keep backward compatibility：SetPayload(load, center=[x,y,z])

    Args:
      load_or_name:
        - float: Payload mass (kg) → form 1
        - str: presetPayloadparameter group name → form 2
      x (optional, float): toolPayloadXpayload CoM (mm)
      y (optional, float): toolPayloadYpayload CoM (mm)
      z (optional, float): toolPayloadZpayload CoM (mm)
      center (optional, Sequence[float]): backward compatible，PayloadCoM[x,y,z]
      preset_name (optional, str): backward compatible，if set, the previous two are ignored，same as form 2
    """
    center = kwargs.get('center', None)
    preset_name = kwargs.get('preset_name', None)

    if isinstance(load_or_name, str):
      return self._send_cmd(f'SetPayload("{load_or_name}")')

    if preset_name is not None:
      return self._send_cmd(f'SetPayload("{preset_name}")')

    load = float(load_or_name)

    x = y = z = None
    if len(args) == 3:
      x, y, z = args
    elif len(args) == 1 and isinstance(args[0], (list, tuple)):
      if len(args[0]) != 3:
        raise ValueError("centerlist needs3 values[x,y,z]")
      x, y, z = args[0]
    elif center is not None:
      if len(center) != 3:
        raise ValueError("centerneeds3 values[x,y,z]")
      x, y, z = center
    elif len(args) != 0:
      raise ValueError("position arguments only support SetPayload(load, x, y, z) three-argument form or SetPayload(load, [x,y,z]) list form")

    if x is not None and y is not None and z is not None:
      return self._send_cmd(f"SetPayload({load:.6f},{float(x):.6f},{float(y):.6f},{float(z):.6f})")
    return self._send_cmd(f"SetPayload({load:.6f})")

  # ==================== collision settings====================

  def SetCollisionLevel(self, level: int) -> str:
    """
    SetCollisionLevelset collision detection level（queued command）    
    Args:
      level: collision detection level(0-5)，00 = collision detection off，1~5higher value = more sensitive
    """
    if not 0 <= level <= 5:
      raise ValueError("collision level must be in0-5")
    return self._send_cmd(f"SetCollisionLevel({level})")

  def SetBackDistance(self, distance: float) -> str:
    """
    SetBackDistanceset collision retreat distance（queued command）
    
    Args:
      distance: collision retreat distance (mm)，valuerange [0, 50]
    """
    if not 0 <= distance <= 50:
      raise ValueError("distance must be in0 to 50(unitmm)")
    return self._send_cmd(f"SetBackDistance({distance:.6f})")

  def SetPostCollisionMode(self, mode: int) -> str:
    """
    SetPostCollisionModeset post-collision behavior（queued command）
    Args:
      mode: post-collisionMode
         0: Disableand stopmoves （V4.6.6see the official docs ✅）
         1: pausemoves     （V4.6.6see the official docs ✅）
         2: ignore collision and continuemoves （⚠️ extraMode，some firmware only，V4.6.6not defined in the official docs）
    """
    if mode not in [0, 1, 2]:
      raise ValueError("post-collisionModemust be0、1 or 2")
    return self._send_cmd(f"SetPostCollisionMode({mode})")

  # ==================== safe skin and safety zones====================

  def EnableSafeSkin(self, status: int) -> str:
    """
    EnableSafeSkinonor disable safe skin（queued command）
    
    Args:
      status: 0=off, 1=on    """
    if status not in [0, 1]:
      raise ValueError("status must be 0 or 1")
    return self._send_cmd(f"EnableSafeSkin({status})")

  def SetSafeSkin(self, part: int, sensitivity: int) -> str:
    """
    SetSafeSkinset safe-skin sensitivity per region（queued command）
    
    Args:
      part: safe-skin region id。3=forearm，4~6=J4~J6
      sensitivity: sensitivity level [0, 3]。0=close，1= low ，2= ，3= high 
    """
    if not (part == 3 or 4 <= part <= 6):
      raise ValueError("part must be 3(forearm) or 4~6 (J4~J6)")
    if not 0 <= sensitivity <= 3:
      raise ValueError("sensitivity must be in0 to 3")
    return self._send_cmd(f"SetSafeSkin({part},{sensitivity})")

  def SetSafeWallEnable(self, index: int, status: int) -> str:
    """
    SetSafeWallEnableonor disable a safety wall（queued command）
    
    Args:
      index: safety-wall index      status: 0=off, 1=on    """
    if status not in [0, 1]:
      raise ValueError("status must be 0 or 1")
    return self._send_cmd(f"SetSafeWallEnable({index},{status})")

  def SetWorkZoneEnable(self, index: int, status: int) -> str:
    """
    Enable or disable a safety zone（queued command）    
    Args:
      index: Safety-zone index
      status: 0=off, 1=on    """
    if status not in [0, 1]:
      raise ValueError("status must be 0 or 1")
    return self._send_cmd(f"SetWorkZoneEnable({index},{status})")

  # ==================== Status query====================

  def RobotMode(self) -> str:
    """Get current robot mode（immediate command）"""
    return self._send_cmd("RobotMode()")

  def GetPose(self, user: int = None, tool: int = None) -> str:
    """
    Get Cartesian pose in the given frames（immediate command）
    
    Args:
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
    """
    has_coord = user is not None or tool is not None
    if has_coord and (user is None or tool is None):
      raise ValueError("user and tool must both be set or both omitted")

    if has_coord:
      if not (0 <= user <= 50):
        raise ValueError("user must be in 0-50")
      if not (0 <= tool <= 50):
        raise ValueError("tool must be in 0-50")
      return self._send_cmd(f"GetPose(user={user},tool={tool})")
    else:
      return self._send_cmd("GetPose()")

  def GetAngle(self) -> str:
    """Get current joint angles（immediate command）"""
    return self._send_cmd("GetAngle()")

  def GetErrorID(self) -> str:
    """Get current error IDs（immediate command）"""
    return self._send_cmd("GetErrorID()")

  def GetScrName(self) -> str:
    """Get the running script name（immediate command）"""
    return self._send_cmd("GetScrName()")

  # ==================== Kinematics====================

  def PositiveKin(self, joints: Sequence[float], user: int = -1, tool: int = -1) -> str:
    """
    Forward kinematics（immediate command）
    
    Args:
      joints: 6 joint angles [j1..j6] (deg)
      user: User frame; -1 = current
      tool: Tool frame; -1 = current
    """
    if len(joints) != 6:
      raise ValueError("Need 6 joint angles")
    joint_str = ",".join([f"{j:.6f}" for j in joints])
    if user != -1 and tool != -1:
      return self._send_cmd(f"PositiveKin({joint_str},user={user},tool={tool})")
    return self._send_cmd(f"PositiveKin({joint_str})")

  def InverseKin(self, pose: Sequence[float], use_joint_near: int = 0, joint_near: Sequence[float] = None, user: int = -1, tool: int = -1) -> str:
    """
    Inverse kinematics（immediate command）
    
    Args:
      pose: Cartesian pose [x,y,z,rx,ry,rz]
      use_joint_near: Use nearest-joint constraint: 0=no, 1=yes
      joint_near: Reference joints [j1..j6]; used when useJointNear=1
      user: User frame; -1 = current
      tool: Tool frame; -1 = current
    """
    if len(pose) != 6:
      raise ValueError("Need 6 pose values")
    pose_str = ",".join([f"{p:.6f}" for p in pose])
    params = [pose_str]
    if use_joint_near != 0:
      params.append(f"useJointNear={use_joint_near}")
      if joint_near is not None:
        if len(joint_near) != 6:
          raise ValueError("joint_near needs 6 joint angles")
        joint_near_str = "jointNear={" + ",".join([f"{j:.6f}" for j in joint_near]) + "}"
        params.append(joint_near_str)
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    return self._send_cmd(f"InverseKin({','.join(params)})")

  # ==================== Reachability====================

  def CheckOddMovL(self, p1: Sequence[float], p2: Sequence[float],
           point_type: str = "joint",
           user: int = -1, tool: int = -1,
           a: float = -1, v: float = -1,
           cp: float = None, r: float = None) -> str:
    """
    Check reachability of a linear move（immediate command）
    
    Args:
      p1: Start [j1..j6] or [x,y,z,rx,ry,rz]
      p2: Goal [j1..j6] or [x,y,z,rx,ry,rz]
      point_type: Point type "joint" or "pose"
      user: User frame; -1 = unspecified
      tool: Tool frame; -1 = unspecified
      a: Accel; -1 = default
      v: Speed; -1 = default
      cp: Continuous-path scale (mutually exclusive with r)
      r: Blend radius mm (mutually exclusive with cp)
    """
    if len(p1) != 6:
      raise ValueError("p1 needs 6 values")
    if len(p2) != 6:
      raise ValueError("p2 needs 6 values")
    if point_type not in ("joint", "pose"):
      raise ValueError("point_type must be 'joint' or 'pose'")
    p_values1 = ",".join([f"{j:.6f}" for j in p1])
    p_values2 = ",".join([f"{j:.6f}" for j in p2])
    params = [f"{point_type}={{{p_values1}}}", f"{point_type}={{{p_values2}}}"]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a:.6f}")
    if v != -1:
      params.append(f"v={v:.6f}")
    if cp is not None:
      params.append(f"cp={cp:.6f}")
    elif r is not None:
      params.append(f"r={r:.6f}")
    return self._send_cmd(f"CheckOddMovL({','.join(params)})")

  def CheckOddMovJ(self, p1: Sequence[float], p2: Sequence[float],
           point_type: str = "joint",
           a: float = -1, v: float = -1,
           cp: float = None) -> str:
    """
    CheckOddMovJcheck jointsmoves ptreachability（immediate command）
    
    Args:
      p1: Start [j1..j6] or [x,y,z,rx,ry,rz]
      p2: Goal [j1..j6] or [x,y,z,rx,ry,rz]
      point_type: Point type "joint" or "pose"
      a: Accel; -1 = default
      v: Speed; -1 = default
      cp: continuous-path
    """
    if len(p1) != 6:
      raise ValueError("p1 needs 6 values")
    if len(p2) != 6:
      raise ValueError("p2 needs 6 values")
    if point_type not in ("joint", "pose"):
      raise ValueError("point_type must be 'joint' or 'pose'")
    p_values1 = ",".join([f"{j:.6f}" for j in p1])
    p_values2 = ",".join([f"{j:.6f}" for j in p2])
    params = [f"{point_type}={{{p_values1}}}", f"{point_type}={{{p_values2}}}"]
    if a != -1:
      params.append(f"a={a:.6f}")
    if v != -1:
      params.append(f"v={v:.6f}")
    if cp is not None:
      params.append(f"cp={cp:.6f}")
    return self._send_cmd(f"CheckOddMovJ({','.join(params)})")

  def CheckOddMovC(self, p1: Sequence[float], p2: Sequence[float], p3: Sequence[float],
           point_type: str = "joint",
           user: int = -1, tool: int = -1,
           a: float = -1, v: float = -1,
           cp: float = None, r: float = None) -> str:
    """
    CheckOddMovCCheck arc move ptreachability（immediate command）
    
    Args:
      p1: Start [j1..j6] or [x,y,z,rx,ry,rz]
      p2: viapt [j1,j2,j3,j4,j5,j6] or [x,y,z,rx,ry,rz]
      p3: Goal [j1..j6] or [x,y,z,rx,ry,rz]
      point_type: Point type "joint" or "pose"
      user: User frame; -1 = unspecified
      tool: Tool frame; -1 = unspecified
      a: Accel; -1 = default
      v: Speed; -1 = default
      cp: Continuous-path scale (mutually exclusive with r)
      r: Blend radius mm (mutually exclusive with cp)
    """
    if len(p1) != 6:
      raise ValueError("p1 needs 6 values")
    if len(p2) != 6:
      raise ValueError("p2 needs 6 values")
    if len(p3) != 6:
      raise ValueError("p3needs6value")
    if point_type not in ("joint", "pose"):
      raise ValueError("point_type must be 'joint' or 'pose'")
    p_values1 = ",".join([f"{j:.6f}" for j in p1])
    p_values2 = ",".join([f"{j:.6f}" for j in p2])
    p_values3 = ",".join([f"{j:.6f}" for j in p3])
    params = [f"{point_type}={{{p_values1}}}",
         f"{point_type}={{{p_values2}}}",
         f"{point_type}={{{p_values3}}}"]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a:.6f}")
    if v != -1:
      params.append(f"v={v:.6f}")
    if cp is not None:
      params.append(f"cp={cp:.6f}")
    elif r is not None:
      params.append(f"r={r:.6f}")
    return self._send_cmd(f"CheckOddMovC({','.join(params)})")

  # ==================== tray ====================

  def CreateTray(self, name: str, dimensions: Sequence[int],
          points: list) -> str:
    """
    CreateTraycreate tray（immediate command）
    supports 1D、2D、3D tray。

    prototype（matches the protocol document）：
    CreateTray(Trayname, {Count}, {P1},{P2}) -- 1D tray
    CreateTray(Trayname, {row,col}, {P1},{P2},{P3},{P4}) -- 2D tray
    CreateTray(Trayname, {row,col,layer}, {P1},{P2},{P3},{P4},{P5},{P6},{P7},{P8}) -- 3D tray

    Args:
      name: tray name（max32byte string，reject digits-only or whitespace-only names）
      dimensions: DoF
        - 1D: [count] ptcount[2,50]
        - 2D: [row, col] rows and columns
        - 3D: [row, col, layer] rows、columns、layers
      points: ptlist，eachpt [x,y,z,rx,ry,rz]format， eachpointseach pose is a separate argument
        - 1D: 2pt [p1, p2]
        - 2D: 4pt [p1, p2, p3, p4]
        - 3D: 8pt [p1, p2, p3, p4, p5, p6, p7, p8]
    """
    dims = ",".join([str(d) for d in dimensions])
    point_strs = []
    for p in points:
      if len(p) != 6:
        raise ValueError(f" eachpointsneeds6value[x,y,z,rx,ry,rz]，got{len(p)}")
      point_strs.append("{pose = {" + ",".join([f"{v:.6f}" for v in p]) + "}}")
    cmd = f"CreateTray({name},{{{dims}}}," + ",".join(point_strs) + ")"
    return self._send_cmd(cmd)

  def GetTrayPoint(self, trayname: str, index: int) -> str:
    """
    GetTrayPointget traypt（immediate command）
    
    Args:
      trayname: tray name（string， and CreateTray from create name maps to）
      index: trayptindex（start index，1-based）
    """
    if not trayname or not trayname.strip():
      raise ValueError("trayname must not be empty")
    if index < 1:
      raise ValueError("index must be >= 1")
    return self._send_cmd(f"GetTrayPoint({trayname},{index})")

  # ==================== log export ====================

  def LogExportUSB(self, log_range: int = None) -> str:
    """
    LogExportUSBexport robot logs toU（immediate command）

    Args:
      log_range: log export range。
        0: export logs/all and logs/user
        1: export logs entire folder
    """
    if log_range is not None:
      if log_range not in [0, 1]:
        raise ValueError("log_rangemust be0 or 1")
      return self._send_cmd(f"LogExportUSB({log_range})")
    return self._send_cmd("LogExportUSB()")

  def GetExportStatus(self) -> str:
    """GetExportStatusget log exportstatus（immediate command）"""
    return self._send_cmd("GetExportStatus()")