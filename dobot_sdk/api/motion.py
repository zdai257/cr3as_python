# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Motion module: motion commands and trajectory control"""

from typing import Sequence
from enum import IntEnum
from ..core.connection import DobotConnection


class CoordinateType(IntEnum):
  """Coordinate-type enum (matches coordtype in the protocol)

  Value mapping:
  0 = joint coordinates
  1 = user / Cartesian coordinates
  2 = tool / Cartesian coordinates
  """
  JOINT = 0   # joint angles — protocol coordtype=0
  CARTESIAN = 1 # Cartesian pose in user frame — protocol coordtype=1
  USER = 1   # alias of CARTESIAN
  TOOL = 2   # Cartesian pose in tool frame — protocol coordtype=2


class Motion:
  """Motion module: all motion commands"""
  
  def __init__(self, connection: DobotConnection):
    self.connection = connection
  
  def _fmt_pose(self, pose: Sequence[float], coord_type: CoordinateType) -> str:
    """Format pose as joint={...} or pose={...}

    Note: USER(1) and TOOL(2) both use the pose= prefix;
    user/tool frames are selected via user= and tool=.
    """
    if len(pose) != 6:
      raise ValueError(f"Pose needs 6 values, got {len(pose)}")
    
    values = ",".join([f"{v:.6f}" for v in pose])
    if coord_type == CoordinateType.JOINT:
      return f"joint={{{values}}}"
    else:
      return f"pose={{{values}}}"
  
  def _send_cmd(self, command: str) -> str:
    """Send command and receive response"""
    return self.connection.send_receive_text(command)
  
  # ==================== Basic motion ====================
  
  def MovJ(self, pose: Sequence[float], 
       coord_type: CoordinateType,
       user: int = -1, tool: int = -1,
       a: int = -1, v: int = -1, cp: int = -1) -> str:
    """
    MovJ joint-interpolated move (queued)
    
    Args:
      pose: 6 values [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
      coord_type: CoordinateType.CARTESIAN or CoordinateType.JOINT
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global)
      cp: Continuous-path scale (0-100, -1 = unused)

    Returns:
      str: ErrorID,{ResultID},MovJ(...);
    """
    pose_str = self._fmt_pose(pose, coord_type)
    
    params = [pose_str]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if v != -1:
      params.append(f"v={v}")
    if cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"MovJ({','.join(params)})"
    return self._send_cmd(cmd)
  
  def MovL(self, pose: Sequence[float],
       coord_type: CoordinateType,
       user: int = -1, tool: int = -1,
       a: int = -1, v: int = -1, speed: int = -1,
       cp: int = -1, r: int = -1) -> str:
    """
    MovLLinear move（queued command）
    
    Args:
      pose: Cartesian pose [x,y,z,rx,ry,rz]
      coord_type: Coordinate type(required)
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
      a: Accel scale (1-100)
      v: Speed scale (1-100), and speedmutually exclusive
      speed: Target speed (mm/s), mutually exclusive with v
      cp: Continuous-path scale (0-100), mutually exclusive with r
      r: Blend radius (mm), mutually exclusive with cp

    Returns:
      str: ErrorID,{ResultID},MovL(...);
    """
    pose_str = self._fmt_pose(pose, coord_type)
    
    params = [pose_str]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")
    if r != -1:
      params.append(f"r={r}")
    elif cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"MovL({','.join(params)})"
    return self._send_cmd(cmd)
  
  def MovLIO(self, pose: Sequence[float], do_list: list,
        coord_type: CoordinateType,
        user: int = -1, tool: int = -1,
        a: int = -1, v: int = -1, speed: int = -1,
        cp: int = -1, r: int = -1) -> str:
    """
    MovLIOLinear moveand writeDO（queued command）
    
    Args:
      pose: Cartesian pose [x,y,z,rx,ry,rz]
      do_list: DOoutput list，each item is[Mode, Distance, Index, Status]
           Mode=0/1, Distance=mm, Index=DO index, Status=0/1
      coord_type: Coordinate type
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
      a: Accel scale (1-100)
      v: Speed scale (1-100), and speedmutually exclusive
      speed: Target speed (mm/s), mutually exclusive with v
      cp: Continuous-path scale (0-100), mutually exclusive with r
      r: Blend radius (mm), mutually exclusive with cp

    Returns:
      str: ErrorID,{ResultID},MovLIO(...);
    """
    pose_str = self._fmt_pose(pose, coord_type)
    
    params = [pose_str]
    for do_item in do_list:
      if len(do_item) != 4:
        raise ValueError("eachDOneeds4value[Mode, Distance, Index, Status]")
      mode, distance, index, status = do_item
      params.append(f"{{{mode},{distance},{index},{status}}}")
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")
    if r != -1:
      params.append(f"r={r}")
    elif cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"MovLIO({','.join(params)})"
    return self._send_cmd(cmd)
  
  def MovJIO(self, pose: Sequence[float], do_list: list,
        coord_type: CoordinateType,
        user: int = -1, tool: int = -1,
        a: int = -1, v: int = -1, cp: int = -1) -> str:
    """
    MovJIOjointmovesand writeDO（queued command）
    
    Args:
      pose: 6 values [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
      do_list: DOoutput list，each item is[Mode, Distance, Index, Status]
           Mode=0/1, Distance=mm, Index=DO index, Status=0/1
      coord_type: Coordinate type
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
      a: Accel scale (1-100)
      v: Speed scale (1-100)
      cp: continuous-path scale (0-100)

    Returns:
      str: ErrorID,{ResultID},MovJIO(...);
    """
    pose_str = self._fmt_pose(pose, coord_type)
    
    params = [pose_str]
    for do_item in do_list:
      if len(do_item) != 4:
        raise ValueError("eachDOneeds4value[Mode, Distance, Index, Status]")
      mode, distance, index, status = do_item
      params.append(f"{{{mode},{distance},{index},{status}}}")
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if v != -1:
      params.append(f"v={v}")
    if cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"MovJIO({','.join(params)})"
    return self._send_cmd(cmd)
  
  def Arc(self, p1: Sequence[float], p2: Sequence[float],
      coord_type: CoordinateType,
      user: int = -1, tool: int = -1,
      a: int = -1, v: int = -1, speed: int = -1,
      cp: int = -1, r: int = -1, mode: int = 0) -> str:
    """
    Arcarc interpolationmoves（queued command）
    
    Args:
      p1: arc via pointpt
      p2: Targetptpose
      coord_type: Coordinate type
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global)
      speed: Target speed (mm/s), mutually exclusive with v
      cp: Continuous-path scale (0-100), mutually exclusive with r
      r: Blend radius (mm), mutually exclusive with cp
      mode: orientation modeMode(0-linear 1-through viapt, 2-fixed)

    Returns:
      str: ErrorID,{ResultID},Arc(...);
    """
    if len(p1) != 6 or len(p2) != 6:
      raise ValueError(f"pose needs6 values")
    
    if coord_type == CoordinateType.JOINT:
      p1_str = f"joint={{{','.join([f'{v:.6f}' for v in p1])}}}"
      p2_str = f"joint={{{','.join([f'{v:.6f}' for v in p2])}}}"
    else:
      p1_str = f"pose={{{','.join([f'{v:.6f}' for v in p1])}}}"
      p2_str = f"pose={{{','.join([f'{v:.6f}' for v in p2])}}}"
    
    params = [p1_str, p2_str]
    
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")
    if r != -1:
      params.append(f"r={r}")
    elif cp != -1:
      params.append(f"cp={cp}")
    if mode != 0:
      params.append(f"mode={mode}")
    
    cmd = f"Arc({','.join(params)})"
    return self._send_cmd(cmd)
  
  def ArcIO(self, p1: Sequence[float], p2: Sequence[float],
        do_list: list,
        coord_type: CoordinateType,
        user: int = -1, tool: int = -1,
        a: int = -1, v: int = -1, speed: int = -1,
        cp: int = -1, r: int = -1, mode: int = 0) -> str:
    """
    ArcIOarcmovesand writeDO（queued command）
    
    Args:
      p1: arc via pointpt
      p2: Targetptpose
      do_list: DOoutput list，each item is[Mode, Distance, Index, Status]
           Mode=0/1, Distance=mm, Index=DO index, Status=0/1
      coord_type: Coordinate type
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
      a: Accel scale (1-100)
      v: Speed scale (1-100)
      speed: Target speed (mm/s), mutually exclusive with v
      cp: Continuous-path scale (0-100), mutually exclusive with r
      r: Blend radius (mm), mutually exclusive with cp
      mode: orientation modeMode(0-2)

    Returns:
      str: ErrorID,{ResultID},ArcIO(...);
    """
    if len(p1) != 6 or len(p2) != 6:
      raise ValueError(f"pose needs6 values")
    
    if coord_type == CoordinateType.JOINT:
      p1_str = f"joint={{{','.join([f'{v:.6f}' for v in p1])}}}"
      p2_str = f"joint={{{','.join([f'{v:.6f}' for v in p2])}}}"
    else:
      p1_str = f"pose={{{','.join([f'{v:.6f}' for v in p1])}}}"
      p2_str = f"pose={{{','.join([f'{v:.6f}' for v in p2])}}}"
    
    params = [p1_str, p2_str]
    for do_item in do_list:
      if len(do_item) != 4:
        raise ValueError("eachDOneeds4value[Mode, Distance, Index, Status]")
      mode_do, distance, index, status = do_item
      params.append(f"{{{mode_do},{distance},{index},{status}}}")
    
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")
    if r != -1:
      params.append(f"r={r}")
    elif cp != -1:
      params.append(f"cp={cp}")
    if mode != 0:
      params.append(f"mode={mode}")
    
    cmd = f"ArcIO({','.join(params)})"
    return self._send_cmd(cmd)
  
  def Circle(self, p1: Sequence[float], p2: Sequence[float],
        count: int, coord_type: CoordinateType,
        user: int = -1, tool: int = -1,
        a: int = -1, v: int = -1, speed: int = -1,
        cp: int = -1, r: int = -1, mode: int = 0) -> str:
    """
    Circlefull-circle interpolationmoves（queued command）
    
    Args:
      p1: circle via pointptpose
      p2: circle endptpose（should match startptsame）
      count: turns (1-999)
      coord_type: Coordinate type
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
      a: Accel scale (1-100)
      v: Speed scale (1-100)
      speed: Target speed (mm/s), mutually exclusive with v
      cp: Continuous-path scale (0-100), mutually exclusive with r
      r: Blend radius (mm), mutually exclusive with cp
      mode: orientation modeMode(0-2)

    Returns:
      str: ErrorID,{ResultID},Circle(...);
    """
    if not 1 <= count <= 999:
      raise ValueError(f"turns must be in1-999")
    
    if len(p1) != 6 or len(p2) != 6:
      raise ValueError(f"pose needs6 values")
    
    if coord_type == CoordinateType.JOINT:
      p1_str = f"joint={{{','.join([f'{v:.6f}' for v in p1])}}}"
      p2_str = f"joint={{{','.join([f'{v:.6f}' for v in p2])}}}"
    else:
      p1_str = f"pose={{{','.join([f'{v:.6f}' for v in p1])}}}"
      p2_str = f"pose={{{','.join([f'{v:.6f}' for v in p2])}}}"
    
    params = [p1_str, p2_str, str(count)]
    
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")
    if r != -1:
      params.append(f"r={r}")
    elif cp != -1:
      params.append(f"cp={cp}")
    if mode != 0:
      params.append(f"mode={mode}")
    
    cmd = f"Circle({','.join(params)})"
    return self._send_cmd(cmd)
  
  # ==================== servomoves ====================
  
  def ServoJ(self, joints: Sequence[float],
        t: float = 0.1, aheadtime: float = 50.0, gain: float = 500.0) -> str:
    """
    ServoJjoint-space servo command（queued command）    
    Args:
      joints: 6Joint angles [j1,j2,j3,j4,j5,j6]
      t: Uptime (s) 0.004-3600.0
      aheadtime: look-ahead(20.0-100.0), likePID Dargs
      gain: P gain (200.0-1000.0), likePID Pargs
    Returns:
      str: ErrorID,{ResultID},ServoJ(...);
    """
    if len(joints) != 6:
      raise ValueError("jointsNeed 6 joint angles")
    if not 0.004 <= t <= 3600.0:
      raise ValueError(f"tmust be in0.004-3600.0")
    if not 20.0 <= aheadtime <= 100.0:
      raise ValueError(f"aheadtimemust be in20.0-100.0")
    if not 200.0 <= gain <= 1000.0:
      raise ValueError(f"gainmust be in200.0-1000.0")
    
    cmd = f"ServoJ({joints[0]:.6f},{joints[1]:.6f},{joints[2]:.6f},{joints[3]:.6f},{joints[4]:.6f},{joints[5]:.6f},t={t:.3f},aheadtime={aheadtime:.1f},gain={gain:.1f})"
    return self._send_cmd(cmd)
  
  def ServoP(self, pose: Sequence[float],
        t: float = 0.1, aheadtime: float = 50.0, gain: float = 500.0) -> str:
    """
    ServoPCartesian servo command（queued command）    
    Args:
      pose: Cartesian pose [x,y,z,rx,ry,rz]
      t: Uptime (s) 0.004-3600.0
      aheadtime: look-ahead(20.0-100.0)
      gain: P gain (200.0-1000.0)

    Returns:
      str: ErrorID,{ResultID},ServoP(...);
    """
    if len(pose) != 6:
      raise ValueError("poseneeds6Cartesian poseargs")
    if not 0.004 <= t <= 3600.0:
      raise ValueError(f"tmust be in0.004-3600.0")
    if not 20.0 <= aheadtime <= 100.0:
      raise ValueError(f"aheadtimemust be in20.0-100.0")
    if not 200.0 <= gain <= 1000.0:
      raise ValueError(f"gainmust be in200.0-1000.0")
    
    cmd = f"ServoP({pose[0]:.6f},{pose[1]:.6f},{pose[2]:.6f},{pose[3]:.6f},{pose[4]:.6f},{pose[5]:.6f},t={t:.3f},aheadtime={aheadtime:.1f},gain={gain:.1f})"
    return self._send_cmd(cmd)
  
  # ==================== Jog ====================
  
  def MoveJog(self, axis: str = "", coord_type: CoordinateType = None,
         user: int = None, tool: int = None) -> str:
    """
    MoveJogJogarm（immediate command)

    notes（matches the protocol document）：
    - coordtypedefaultvalue " xsettings used on successvalue"，omit the parameter when unset
    - user/toolalso omit when not passed explicitly，controller uses its defaultvalue
    - when axisIDjoint axis（J1~J6） when ，coordtypemust be0（ignore passedvalue）
    - when axisIDCartesian axis（X/Y/Z/Rx/Ry/Rz） when ，coordtypemust be1 or 2，0returnsError ID-6

    Args:
      axis: "X+", "X-", "J1+" etc ，empty string stops
      coord_type: Coordinate type，optional。None=use the controllerxset ok
            JOINT=0（jointJog）, USER/CARTESIAN=1（User frame）, TOOL=2（Tool frame）
      user: User frame index，optional。None=use controller default
      tool: Tool frame index，optional。None=use controller default
    Returns:
      str: ErrorID,{},MoveJog(...);
    """
    if not axis:
      return self._send_cmd("MoveJog()")

    params = [axis]
    if coord_type is not None:
      params.append(f"coordtype={int(coord_type)}")
    if user is not None:
      params.append(f"user={user}")
    if tool is not None:
      params.append(f"tool={tool}")

    cmd = f"MoveJog({','.join(params)})"
    return self._send_cmd(cmd)
  
  # ==================== moves to pt====================
  
  def RunTo(self, point: Sequence[float], move_type: int,
       user: int = -1, tool: int = -1,
       a: int = -1, v: int = -1,
       coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
    """
    RunTomovesto the givenpt（immediate command)
    
    Args:
      point: 6 values [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
      move_type: movestype
            0=jointmoves, 1=Linear move
            2=jointmovesto the offset angles（relative joint）
            3= along Tool framerelativeLinear move
            4= along User framerelativeLinear move
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global)
      coord_type: Coordinate type(defaultCARTESIAN)

    Returns:
      str: ErrorID,{ResultID},RunTo(...);
    """
    if move_type not in [0, 1, 2, 3, 4]:
      raise ValueError("move_type must be in [0, 4] in range: 0=joint, 1=linear, "
               "2=joint offset, 3=relative linear move in tool frame, 4=relative linear move in user frame")
    
    point_str = self._fmt_pose(point, coord_type)
    
    params = [point_str, str(move_type)]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if v != -1:
      params.append(f"v={v}")
    
    cmd = f"RunTo({','.join(params)})"
    return self._send_cmd(cmd)
  
  # ==================== trajectory playback ====================
  
  def GetStartPose(self, trace_name: str, path_type: int = 1) -> str:
    """
    GetStartPosefirst point of the named trajectorypoints（immediate command)    
    Args:
      trace_name: trajectory file name（with suffix.csv）
      path_type: trajectory type (1-playback trajectory 2-fit trajectory，default1)

    Returns:
      str: ErrorID,{X,Y,Z,Rx,Ry,Rz},GetStartPose(traceName,pathType);
    """
    if path_type not in [1, 2]:
      raise ValueError("path_type must be1 or 2")
    
    cmd = f"GetStartPose(\"{trace_name}\",{path_type})"
    return self._send_cmd(cmd)
  
  def MovS(self,
       trace_or_points,
       coord_type: CoordinateType = CoordinateType.CARTESIAN,
       freq: float = -1,
       user: int = -1, tool: int = -1,
       a: int = -1, v: int = -1, speed: int = -1) -> str:
    """
    MovSfitmoves（queued command)
    two call styles：
    1. ptpose-list form：MovS([p1, p2, p3, ...], coord_type, freq, user, tool, a, v|speed, freq)
    2. file form：MovS("xxx.csv", coord_type, freq, user, tool, a, v|speed, freq)

    Args:
      trace_or_points: 
        - str: trajectory file name（with suffix， e.g. "xxx.csv"） -> file form
        - Sequence[Sequence[float]]: ptpose list， eachpointsis [x,y,z,rx,ry,rz] or [j1..j6]
      coord_type: ptCoordinate type（ only ptused with the pose-list form）
      freq: filter coefficient（range 0-1, 10 = filter off，-1-1 = unset）
      user: User frameindex(0-50, -1-1 = current)
      tool: Tool frameindex(0-50, -1-1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global), and speedmutually exclusive
      speed: Target speed (mm/s), mutually exclusive with v（preferredspeed）

    Returns:
      str: ErrorID,{ResultID},MovS(...);
    """
    params = []

    if isinstance(trace_or_points, str):
      params.append(f"\"{trace_or_points}\"")
    else:
      points = list(trace_or_points)
      if len(points) < 4 or len(points) > 50:
        raise ValueError("ptpose-list form requires 4~50 points")
      for p in points:
        if len(p) != 6:
          raise ValueError(" eachpointsneeds6value [x,y,z,rx,ry,rz] or [j1..j6]")
        params.append(self._fmt_pose(p, coord_type))

    if freq != -1:
      params.append(f"freq={freq}")
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")

    cmd = f"MovS({','.join(params)})"
    return self._send_cmd(cmd)
  
  def StartPath(self, trace_name: str, is_const: int = 0, 
          multi: float = 1.0, sample: int = 50,
          freq: float = 0.2, user: int = -1, tool: int = -1) -> str:
    """
    StartPathplay back a recordedmovestrajectory（queued command)    
    Args:
      trace_name: trajectory file name（with suffix）
      is_const: constant-speed playback(0-raw 1-constant speed)
      multi: speedcount ( only is_const=0when it applies range0.25-2)
      sample: sample interval (ms, range8-1000)
      freq: filter coefficient (range0-1, 10 = filter off)
      user: User frameindex(0-50, -1-1 = current)
      tool: Tool frameindex(0-50, -1-1 = current)

    Returns:
      str: ErrorID,{ResultID},StartPath(...);
    """
    params = [f"\"{trace_name}\""]
    params.append(f"isConst={is_const}")
    params.append(f"multi={multi}")
    params.append(f"sample={sample}")
    params.append(f"freq={freq}")
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    
    cmd = f"StartPath({','.join(params)})"
    return self._send_cmd(cmd)
  
  # ==================== Relative move ====================
  
  def RelMovJTool(self, offsetX: float, offsetY: float, offsetZ: float,
          offsetRx: float, offsetRy: float, offsetRz: float,
          user: int = -1, tool: int = -1,
          a: int = -1, v: int = -1, cp: int = -1) -> str:
    """
    RelMovJTool along Tool framerelative joint movemoves（queued command）
    
    Args:
      offsetX: X offset (mm)
      offsetY: Y offset (mm)
      offsetZ: Z offset (mm)
      offsetRx: Rx offset (deg)
      offsetRy: Ry offset (deg)
      offsetRz: Rz offset (deg)
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global)
      cp: Continuous-path scale (0-100, -1 = unused)

    Returns:
      str: ErrorID,{ResultID},RelMovJTool(...);
    """
    params = [
      f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
      f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
    ]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if v != -1:
      params.append(f"v={v}")
    if cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"RelMovJTool({','.join(params)})"
    return self._send_cmd(cmd)
  
  def RelMovLTool(self, offsetX: float, offsetY: float, offsetZ: float,
          offsetRx: float, offsetRy: float, offsetRz: float,
          user: int = -1, tool: int = -1,
          a: int = -1, v: int = -1, speed: int = -1,
          cp: int = -1, r: int = -1) -> str:
    """
    RelMovLTool along Tool framerelativeLinear move（queued command）
    
    Args:
      offsetX: X offset (mm)
      offsetY: Y offset (mm)
      offsetZ: Z offset (mm)
      offsetRx: Rx offset (deg)
      offsetRy: Ry offset (deg)
      offsetRz: Rz offset (deg)
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global), and speedmutually exclusive
      speed: Target speed (mm/s), mutually exclusive with v (speedpreferred)
      cp: Continuous-path scale (0-100, -1 = unused), and rmutually exclusive
      r: Blend radius (mm), mutually exclusive with cp (rpreferred)

    Returns:
      str: ErrorID,{ResultID},RelMovLTool(...);
    """
    params = [
      f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
      f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
    ]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")
    if r != -1:
      params.append(f"r={r}")
    elif cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"RelMovLTool({','.join(params)})"
    return self._send_cmd(cmd)
  
  def RelMovJUser(self, offsetX: float, offsetY: float, offsetZ: float,
          offsetRx: float, offsetRy: float, offsetRz: float,
          user: int = -1, tool: int = -1,
          a: int = -1, v: int = -1, cp: int = -1) -> str:
    """
    RelMovJUser along User framerelative joint movemoves（queued command）
    
    Args:
      offsetX: X offset (mm)
      offsetY: Y offset (mm)
      offsetZ: Z offset (mm)
      offsetRx: Rx offset (deg)
      offsetRy: Ry offset (deg)
      offsetRz: Rz offset (deg)
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global)
      cp: Continuous-path scale (0-100, -1 = unused)

    Returns:
      str: ErrorID,{ResultID},RelMovJUser(...);
    """
    params = [
      f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
      f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
    ]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if v != -1:
      params.append(f"v={v}")
    if cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"RelMovJUser({','.join(params)})"
    return self._send_cmd(cmd)
  
  def RelMovLUser(self, offsetX: float, offsetY: float, offsetZ: float,
          offsetRx: float, offsetRy: float, offsetRz: float,
          user: int = -1, tool: int = -1,
          a: int = -1, v: int = -1, speed: int = -1,
          cp: int = -1, r: int = -1) -> str:
    """
    RelMovLUser along User framerelativeLinear move（queued command）
    
    Args:
      offsetX: X offset (mm)
      offsetY: Y offset (mm)
      offsetZ: Z offset (mm)
      offsetRx: Rx offset (deg)
      offsetRy: Ry offset (deg)
      offsetRz: Rz offset (deg)
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global), and speedmutually exclusive
      speed: Target speed (mm/s), mutually exclusive with v (speedpreferred)
      cp: Continuous-path scale (0-100, -1 = unused), and rmutually exclusive
      r: Blend radius (mm), mutually exclusive with cp (rpreferred)

    Returns:
      str: ErrorID,{ResultID},RelMovLUser(...);
    """
    params = [
      f"{offsetX:.6f}", f"{offsetY:.6f}", f"{offsetZ:.6f}",
      f"{offsetRx:.6f}", f"{offsetRy:.6f}", f"{offsetRz:.6f}"
    ]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if speed != -1:
      params.append(f"speed={speed}")
    elif v != -1:
      params.append(f"v={v}")
    if r != -1:
      params.append(f"r={r}")
    elif cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"RelMovLUser({','.join(params)})"
    return self._send_cmd(cmd)
  
  def RelJointMovJ(self, offset1: float, offset2: float, offset3: float,
           offset4: float, offset5: float, offset6: float,
           user: int = -1, tool: int = -1,
           a: int = -1, v: int = -1, cp: int = -1) -> str:
    """
    RelJointMovJalong jointsFramesrelative joint movemoves（queued command）
    
    Args:
      offset1: J1joint offset ( deg)
      offset2: J2joint offset ( deg)
      offset3: J3joint offset ( deg)
      offset4: J4joint offset ( deg)
      offset5: J5joint offset ( deg)
      offset6: J6joint offset ( deg)
      user: User frame index (0-50, -1 = current)
      tool: Tool frame index (0-50, -1 = current)
      a: Accel scale (1-100, -1 = global)
      v: Speed scale (1-100, -1 = global)
      cp: Continuous-path scale (0-100, -1 = unused)

    Returns:
      str: ErrorID,{ResultID},RelJointMovJ(...);
    """
    params = [
      f"{offset1:.6f}", f"{offset2:.6f}", f"{offset3:.6f}",
      f"{offset4:.6f}", f"{offset5:.6f}", f"{offset6:.6f}"
    ]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if v != -1:
      params.append(f"v={v}")
    if cp != -1:
      params.append(f"cp={cp}")
    
    cmd = f"RelJointMovJ({','.join(params)})"
    return self._send_cmd(cmd)
  
  def RelPointTool(self, p: Sequence[float], offset: Sequence[float],
           coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
    """
    RelPointTool along Tool frameCartesianptoffset（immediate command）
    
    Args:
      p: 6value pt[x,y,z,rx,ry,rz] or [j1..j6]
      offset: 6valueoffset[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]
      coord_type: ptCoordinate type。default Cartesian(pose)，or joint(joint)

    Returns:
      str: ErrorID,{X,Y,Z,Rx,Ry,Rz},RelPointTool(...);
    """
    if len(p) != 6:
      raise ValueError("pneeds6pointspose args")
    if len(offset) != 6:
      raise ValueError("offsetneeds6offset values[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]")
    
    p_str = self._fmt_pose(p, coord_type)
    offset_values = ",".join([f"{v:.6f}" for v in offset])
    cmd = f"RelPointTool({p_str},{{{offset_values}}})"
    return self._send_cmd(cmd)
  
  def RelPointUser(self, p: Sequence[float], offset: Sequence[float],
           coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
    """
    RelPointUser along User frameCartesianptoffset（immediate command）
    
    Args:
      p: 6value pt[x,y,z,rx,ry,rz] or [j1..j6]
      offset: 6valueoffset[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]
      coord_type: ptCoordinate type。default Cartesian(pose)，or joint(joint)

    Returns:
      str: ErrorID,{X,Y,Z,Rx,Ry,Rz},RelPointUser(...);
    """
    if len(p) != 6:
      raise ValueError("pneeds6pointspose args")
    if len(offset) != 6:
      raise ValueError("offsetneeds6offset values[offsetX,offsetY,offsetZ,offsetRx,offsetRy,offsetRz]")
    
    p_str = self._fmt_pose(p, coord_type)
    offset_values = ",".join([f"{v:.6f}" for v in offset])
    cmd = f"RelPointUser({p_str},{{{offset_values}}})"
    return self._send_cmd(cmd)
  
  def RelJoint(self, joints: Sequence[float], offset: Sequence[float]) -> str:
    """
    RelJointjointptoffset（immediate command）
    
    Args:
      joints: 6Joint angles[J1,J2,J3,J4,J5,J6]
      offset: 6 offsets[offset1,offset2,offset3,offset4,offset5,offset6]

    Returns:
      str: ErrorID,{J1,J2,J3,J4,J5,J6},RelJoint(...);
    """
    if len(joints) != 6:
      raise ValueError("jointsNeed 6 joint angles[J1,J2,J3,J4,J5,J6]")
    if len(offset) != 6:
      raise ValueError("offsetneeds6 offsets[offset1,offset2,offset3,offset4,offset5,offset6]")
    
    joints_str = ",".join([f"{v:.6f}" for v in joints])
    offset_values = ",".join([f"{v:.6f}" for v in offset])
    cmd = f"RelJoint({joints_str},{{{offset_values}}})"
    return self._send_cmd(cmd)
  
  # ==================== commandIDquery ====================
  
  def GetCurrentCommandID(self) -> str:
    """
    GetCurrentCommandIDqueued command id of the active instructionID（immediate command）
    
    Returns:
      str: ErrorID,{ResultID},GetCurrentCommandID();
    """
    return self._send_cmd("GetCurrentCommandID()")
  
  # ==================== Framesoffset====================
  
  def StartRTOffset(self) -> str:
    """
    StartRTOffsetstartFramesoffset（queued command）
    
    Returns:
      str: ErrorID,{ResultID},StartRTOffset();
    """
    return self._send_cmd("StartRTOffset()")
  
  def EndRTOffset(self) -> str:
    """
    EndRTOffsetendFramesoffset（queued command)    
    Returns:
      str: ErrorID,{ResultID},EndRTOffset();
    """
    return self._send_cmd("EndRTOffset()")
  
  def OffsetPara(self, x: float, y: float, z: float,
          rx: float, ry: float, rz: float) -> str:
    """
    OffsetParasetFramesoffsetvalue（immediate command)
    
    Args:
      x: X offset (mm)
      y: Y offset (mm)
      z: Z offset (mm)
      rx: Rx offset (deg)
      ry: Ry offset (deg)
      rz: Rz offset (deg)

    Returns:
      str: ErrorID,{},OffsetPara(...);
    """
    cmd = f"OffsetPara({x:.6f},{y:.6f},{z:.6f},{rx:.6f},{ry:.6f},{rz:.6f})"
    return self._send_cmd(cmd)
  
  # ==================== path recovery ====================
  
  def SetResumeOffset(self, distance: float) -> str:
    """
    SetResumeOffsetset path-recovery retreat distance（immediate command）
    
    Args:
      distance: retreat distance (mm)

    Returns:
      str: ErrorID,{},SetResumeOffset(distance);
    """
    return self._send_cmd(f"SetResumeOffset({distance:.6f})")
  
  def PathRecovery(self) -> str:
    """
    PathRecoverystart path recovery（immediate command)    
    Returns:
      str: ErrorID,{},PathRecovery();
    """
    return self._send_cmd("PathRecovery()")
  
  def PathRecoveryStop(self) -> str:
    """
    PathRecoveryStopduring path recoveryDisable robot（immediate command）
    
    Returns:
      str: ErrorID,{},PathRecoveryStop();
    """
    return self._send_cmd("PathRecoveryStop()")
  
  def PathRecoveryStatus(self) -> str:
    """
    PathRecoveryStatusquery path recoverystatus（immediate command)    
    Returns:
      str: ErrorID,{status},PathRecoveryStatus();
         status: 0-back at the pause pose 1-smaller deviation, 2-larger deviation
    """
    return self._send_cmd("PathRecoveryStatus()")
