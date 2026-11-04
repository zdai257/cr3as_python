# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""plugin module - force control、conveyor and other extras"""

from typing import Sequence
from ..core.connection import DobotConnection
from .motion import CoordinateType


class Plugins:
  """Plugin module - force control、conveyor and other extras"""

  def __init__(self, connection: DobotConnection):
    self.connection = connection

  def _send_cmd(self, command: str) -> str:
    """Send command and receive response"""
    return self.connection.send_receive_text(command)

  # ==================== FT sensor basics ====================

  def EnableFTSensor(self, status: int) -> str:
    """
    EnableFTSensorenable/disable FT sensor（immediate command）
    
    Args:
      status: 0=off, 1=on    """
    if status not in [0, 1]:
      raise ValueError("status must be 0 or 1")
    return self._send_cmd(f"EnableFTSensor({status})")

  def SixForceHome(self) -> str:
    """SixForceHomezero the FT sensor（immediate command）"""
    return self._send_cmd("SixForceHome()")

  def GetForce(self, tool: int = -1) -> str:
    """
    GetForceread FT sensorvalue（immediate command）    
    Args:
      tool: Tool frame index，valuerange[0, 50]，-1-1 = currentTool frame
    """
    if tool == -1:
      return self._send_cmd("GetForce()")
    if not (0 <= tool <= 50):
      raise ValueError("toolargument must be in[0, 50]")
    return self._send_cmd(f"GetForce({tool})")

  # ==================== force controlBackdrive / drag mode ====================

  def ForceDriveMode(self, direction: Sequence[int], user: int = -1) -> str:
    """
    ForceDriveModeselect whichDragaxes andEnter force-drag mode（immediate command）
    
    Args:
      direction: 6-axis Dragswitch [x,y,z,rx,ry,rz]，00 = axis lockedDrag，11 = axis freeDrag
      user: User frame index，valuerange[0, 50]，omit to ignore this frameUser frame
    """
    if len(direction) != 6:
      raise ValueError("directionneeds6 values")
    for d in direction:
      if d not in [0, 1]:
        raise ValueError("directionargsvaluemust be0 or 1")
    if user != -1 and not (0 <= user <= 50):
      raise ValueError("userargument must be in[0, 50]")
    direction_str = "{" + ",".join([f"{v}" for v in direction]) + "}"
    if user == -1:
      return self._send_cmd(f"ForceDriveMode({direction_str})")
    return self._send_cmd(f"ForceDriveMode({direction_str},{user})")

  def ForceDriveSpeed(self, speed: int) -> str:
    """
    ForceDriveSpeedset force controlDragspeed（immediate command）
    
    Args:
      speed: force controlDragspeedscale，valuerange[1, 100]
    """
    if not 1 <= speed <= 100:
      raise ValueError("speedscale must be in1-100")
    return self._send_cmd(f"ForceDriveSpeed({speed})")

  # ==================== force controlModeparameter setup ====================

  def FCForceMode(self, direction: Sequence[int], force: Sequence[float],
           reference: int = 0, user: int = -1, tool: int = -1) -> str:
    """
    FCForceModewith user parametersonforce control（queued command)    
    Args:
      direction: 6-axis force-control switch [x,y,z,rx,ry,rz]，1meanson，00 = off
      force: 6-axis Target force [fx,fy,fz,frx,fry,frz]，linear[-200,200]N，angular[-12,12]N/m
      reference: referenceCoordinate type (0-Tool frame 1-User frame)
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
    """
    if len(direction) != 6:
      raise ValueError("directionneeds6 values")
    if len(force) != 6:
      raise ValueError("forceneeds6 values")
    for d in direction:
      if d not in [0, 1]:
        raise ValueError("directionargsvaluemust be0 or 1")
    for i, f in enumerate(force):
      if i < 3:
        if not (-200 <= f <= 200):
          raise ValueError("linearTargetforce must be in[-200, 200]")
      else:
        if not (-12 <= f <= 12):
          raise ValueError("angularTargetforce must be in[-12, 12]")
    if user != -1 and not (0 <= user <= 50):
      raise ValueError("userargument must be in[0, 50]")
    if tool != -1 and not (0 <= tool <= 50):
      raise ValueError("toolargument must be in[0, 50]")
    if reference not in [0, 1]:
      raise ValueError("referencemust be0 or 1")
    direction_str = "{" + ",".join([f"{v}" for v in direction]) + "}"
    force_str = "{" + ",".join([f"{v}" for v in force]) + "}"
    params = [direction_str, force_str]
    if reference != 0:
      params.append(f"reference={reference}")
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    return self._send_cmd(f"FCForceMode({','.join(params)})")

  def FCSetDeviation(self, deviation: Sequence[float], control_type: int = -1) -> str:
    """
    FCSetDeviationset force controlModelinear and angular deviation（immediate command）    
    Args:
      deviation: 6-axis deviation [x,y,z,rx,ry,rz]，mmmm，orientation in deg
            linear deviation invaluerange(0, 1000]，defaultvalue100；angular deviation invaluerange(0, 360]，defaultvalue36
      control_type: control type，0-above thresholdvalue when Alarm，1-above thresholdvaluestop searching and continuemoves，-1default
    """
    if len(deviation) != 6:
      raise ValueError("deviation needs6 values")
    for v in deviation[:3]:
      if not (0 < v <= 1000):
        raise ValueError("linear deviation must be in(0, 1000]")
    for v in deviation[3:]:
      if not (0 < v <= 360):
        raise ValueError("angular deviation must be in(0, 360]")
    if control_type not in [-1, 0, 1]:
      raise ValueError("control_typemust be-1、0 or 1")
    dev_str = "{" + ",".join([f"{v}" for v in deviation]) + "}"
    if control_type == -1:
      return self._send_cmd(f"FCSetDeviation({dev_str})")
    return self._send_cmd(f"FCSetDeviation({dev_str},{control_type})")

  def FCSetForceLimit(self, x: float, y: float, z: float,
              rx: float, ry: float, rz: float) -> str:
    """
    FCSetForceLimitset force limits（immediate command）
    
    Args:
      x,y,z: max linear force (N)，valuerange(0, 500]，defaultvalue500
      rx,ry,rz: max moment (N/m)，valuerange(0, 50]，defaultvalue50
    """
    for v in [x, y, z]:
      if not (0 < v <= 500):
        raise ValueError("linear force limit must be in(0, 500]")
    for v in [rx, ry, rz]:
      if not (0 < v <= 50):
        raise ValueError("moment limit must be in(0, 50]")
    return self._send_cmd(f"FCSetForceLimit({x},{y},{z},{rx},{ry},{rz})")

  def FCSetMass(self, x: float, y: float, z: float,
          rx: float, ry: float, rz: float) -> str:
    """
    FCSetMassset force controlModeinertia per axis（immediate command）    
    Args:
      x,y,z: linear inertia(kg)，valuerange(0, 10000]，defaultvalue20
      rx,ry,rz: angular inertia(kg·m²)，valuerange(0, 10000]，defaultvalue20
    """
    for v in [x, y, z, rx, ry, rz]:
      if not (0 < v <= 10000):
        raise ValueError("inertia must be in(0, 10000]")
    return self._send_cmd(f"FCSetMass({x},{y},{z},{rx},{ry},{rz})")

  def FCSetStiffness(self, x: float, y: float, z: float,
             rx: float, ry: float, rz: float) -> str:
    """
    FCSetStiffnessset force controlModestiffness per axis（immediate command）    
    Args:
      x,y,z: linear stiffness(N/mm)，valuerange[0, 10000]，defaultvalue30
      rx,ry,rz: angular stiffness(N/m·deg)，valuerange[0, 10000]，defaultvalue30
    """
    for v in [x, y, z, rx, ry, rz]:
      if not (0 <= v <= 10000):
        raise ValueError("stiffness must be in[0, 10000]")
    return self._send_cmd(f"FCSetStiffness({x},{y},{z},{rx},{ry},{rz})")

  def FCSetDamping(self, x: float, y: float, z: float,
            rx: float, ry: float, rz: float) -> str:
    """
    FCSetDampingset force controlModedamping per axis（immediate command）    
    Args:
      x,y,z: linear damping (N·s/mm)，valuerange[0, 1000]，defaultvalue50
      rx,ry,rz: angular damping(N·s/m·deg)，valuerange[0, 1000]，defaultvalue50
    """
    for v in [x, y, z, rx, ry, rz]:
      if not (0 <= v <= 1000):
        raise ValueError("damping must be in[0, 1000]")
    return self._send_cmd(f"FCSetDamping({x},{y},{z},{rx},{ry},{rz})")

  def FCOff(self) -> str:
    """FCOffleave force modeMode（queued command）"""
    return self._send_cmd("FCOff()")

  def FCSetForceSpeedLimit(self, x: float, y: float, z: float,
                 rx: float, ry: float, rz: float) -> str:
    """
    FCSetForceSpeedLimitset per-axis force-control ratesspeed（immediate command）
    
    Args:
      x,y,z: linear force-control ratespeed (mm/s)
          CRA/CRAFmodel rangevaluerange(0,safety limitTCPspeedvalue]
          other models usevaluerange(0, 300]
      rx,ry,rz: angular force-control ratespeed (deg/s)
           CRA/CRAFmodel rangevaluerange(0,(4*safety limitTCPspeedvalue*0.001)/(3.14*180)]
           other models usevaluerange(0, 90]
    """
    for v in [x, y, z]:
      if not (0 < v <= 300):
        raise ValueError("linear force-control ratespeedmust be in(0, 300]")
    for v in [rx, ry, rz]:
      if not (0 < v <= 90):
        raise ValueError("angular force-control ratespeedmust be in(0, 90]")
    return self._send_cmd(f"FCSetForceSpeedLimit({x},{y},{z},{rx},{ry},{rz})")

  def FCSetForce(self, x: float, y: float, z: float,
           rx: float, ry: float, rz: float) -> str:
    """
    FCSetForcelive constant-force setpoints（immediate command）
    
    Args:
      x,y,z: linear constant forcevalue(N)，valuerange[-200, 200]
      rx,ry,rz: angular constant forcevalue (N/m)，valuerange[-12, 12]
    """
    for v in [x, y, z]:
      if not (-200 <= v <= 200):
        raise ValueError("linear constant forcevaluemust be in[-200, 200]")
    for v in [rx, ry, rz]:
      if not (-12 <= v <= 12):
        raise ValueError("angular constant forcevaluemust be in[-12, 12]")
    return self._send_cmd(f"FCSetForce({x},{y},{z},{rx},{ry},{rz})")

  # ==================== FT collision detection（only forCRAFmodel）====================

  def SetFCCollision(self, force: float, torque: float) -> str:
    """
    SetFCCollisionset FT collision thresholdvalueargs（only forCRAFmodel，immediate command）
    
    Args:
      force: FT collision force thresholdvalue，unitN
          CR5AFvaluerange：[5, 150]
          CR10AFvaluerange：[5, 300]
          CR20AFvaluerange：[5, 500]
      torque: FT collision torque thresholdvalue，unitN/m
          CR5AFvaluerange：[0.5, 15]
          CR10AFvaluerange：[0.5, 30]
          CR20AFvaluerange：[0.5, 50]
    """
    if not (5 <= force <= 500):
      raise ValueError("force thresholdvaluemust be in[5, 500]，range depends on the model")
    if not (0.5 <= torque <= 50):
      raise ValueError("torque thresholdvaluemust be in[0.5, 50]，range depends on the model")
    return self._send_cmd(f"SetFCCollision({force},{torque})")

  def FCCollisionSwitch(self, status: int) -> str:
    """
    FCCollisionSwitchenable/disable FT sensorcollision detection switch（only forCRAFmodel，immediate command）
    
    Args:
      status: 0-disable collision detection 1-oncollision detection    """
    if status not in [0, 1]:
      raise ValueError("status must be 0 or 1")
    return self._send_cmd(f"FCCollisionSwitch({status})")

  # ==================== conveyor tracking ====================

  def CnvInit(self, index: int) -> str:
    """
    CnvInitEnable conveyor（immediate command）
    
    Args:
      index: conveyor index (1-3)
    """
    if not 1 <= index <= 3:
      raise ValueError("conveyor index must be in1-3")
    return self._send_cmd(f"CnvInit({index})")

  def GetCnvObject(self, obj_id: int) -> str:
    """
    GetCnvObjectWaitpart-enter region on the conveyor（immediate command）    
    Args:
      obj_id: part type，valuerange[0, 15]
          0：do not filter by part type，oldest queued part info
    """
    if not 0 <= obj_id <= 15:
      raise ValueError("part type must be in0-15")
    return self._send_cmd(f"GetCnvObject({obj_id})")

  def StartSyncCnv(self) -> str:
    """StartSyncCnvStart conveyor trackingfeature（immediate command）"""
    return self._send_cmd("StartSyncCnv()")

  def CnvMovL(self, pose: Sequence[float],
         coord_type: CoordinateType = CoordinateType.CARTESIAN,
         user: int = -1, tool: int = -1,
         a: int = -1, v: int = -1, speed: int = -1,
         cp: int = -1, r: int = -1) -> str:
    """
    CnvMovLrun conveyor tracking，use linear interpolation（queued command)    
    Args:
      pose: 6 coordinatesvalue [x,y,z,rx,ry,rz] or [j1,j2,j3,j4,j5,j6]
      coord_type: ptCoordinate type (CoordinateType.JOINT or CoordinateType.CARTESIAN)
      user: User frame index (0-50)
      tool: Tool frame index (0-50)
      a: Accel scale (1-100)
      v: Speed scale (1-100), and speedmutually exclusive
      speed: Target speed (mm/s), mutually exclusive with v
      cp: Continuous-path scale (0-100), mutually exclusive with r
      r: Blend radius (mm), mutually exclusive with cp
    """
    if len(pose) != 6:
      raise ValueError("poseneeds6 values")

    if coord_type == CoordinateType.JOINT:
      prefix = "joint"
    else:
      prefix = "pose"

    pose_str = prefix + "={" + ",".join([f"{v:.6f}" for v in pose]) + "}"
    params = [pose_str]
    if user != -1:
      params.append(f"user={user}")
    if tool != -1:
      params.append(f"tool={tool}")
    if a != -1:
      params.append(f"a={a}")
    if v != -1:
      params.append(f"v={v}")
    if speed != -1:
      params.append(f"speed={speed}")
    if cp != -1:
      params.append(f"cp={cp}")
    if r != -1:
      params.append(f"r={r}")
    return self._send_cmd(f"CnvMovL({','.join(params)})")

  def CnvMovC(self, via_point: Sequence[float], target_point: Sequence[float],
         user: int = -1, tool: int = -1,
         a: int = -1, v: int = -1, speed: int = -1,
         cp: int = -1, r: int = -1, mode: int = 0,
         coord_type: CoordinateType = CoordinateType.CARTESIAN) -> str:
    """
    CnvMovCconveyor-tracked arc interpolation（queued command)

    protocol prototype：CnvMovC(P1,P2,user, tool, a, v, cp|r, mode)
    P1=viapt，P2=Targetpt

    Args:
      via_point: arc via pointpt P1 [x,y,z,rx,ry,rz] or [j1..j6]
      target_point: arc endpt P2 [x,y,z,rx,ry,rz] or [j1..j6]
      user: User frame index (0-50)。-1means current
      tool: Tool frame index (0-50)。-1means current
      a: Accel scale (1-100)。-1-1 = default
      v: Speed scale (1-100), and speedmutually exclusive。-1-1 = default
      speed: Target speed (mm/s), mutually exclusive with v（preferredspeed）
      cp: Continuous-path scale (0-100), mutually exclusive with r。-1-1 = unset
      r: Blend radius (mm), mutually exclusive with cp。-1-1 = unset
      mode: interpolationMode。default0
      coord_type: ptCoordinate type（Cartesian or joint）
    """
    if len(via_point) != 6:
      raise ValueError("via_point(viaptP1)needs6 values")
    if len(target_point) != 6:
      raise ValueError("target_point(TargetptP2)needs6 values")

    if coord_type == CoordinateType.JOINT:
      prefix = "joint"
    else:
      prefix = "pose"

    via_str = prefix + "={" + ",".join([f"{val:.6f}" for val in via_point]) + "}"
    target_str = prefix + "={" + ",".join([f"{val:.6f}" for val in target_point]) + "}"

    params = [via_str, target_str]
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
    if cp != -1:
      params.append(f"cp={cp}")
    if r != -1:
      params.append(f"r={r}")
    if mode != 0:
      params.append(f"mode={mode}")
    return self._send_cmd(f"CnvMovC({','.join(params)})")

  def StopSyncCnv(self) -> str:
    """StopSyncCnvStop conveyor trackingfeature（immediate command）"""
    return self._send_cmd("StopSyncCnv()")

  def SetCnvPointOffset(self, x_offset: float, y_offset: float) -> str:
    """
    SetCnvPointOffsetconfigure conveyorUser frame X、Yaxis offset（immediate command）
    
    Args:
      x_offset: Xaxis offset(mm)
      y_offset: Yaxis offset(mm)
    """
    return self._send_cmd(f"SetCnvPointOffset({x_offset},{y_offset})")

  def SetCnvTimeCompensation(self, compensation: float) -> str:
    """
    SetCnvTimeCompensationset compensationTime（immediate command）
    
    Args:
      compensation: compensationTime (ms)
    """
    return self._send_cmd(f"SetCnvTimeCompensation({compensation})")


