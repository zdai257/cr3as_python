# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""IOmodule - numericIO、analogIO、Tool IO"""

from typing import List, Union
from ..core.connection import DobotConnection


class IO:
  """IO module - handle allIOrelated commands"""

  def __init__(self, connection: DobotConnection):
    self.connection = connection

  def _send_cmd(self, command: str) -> str:
    """Send command and receive response"""
    return self.connection.send_receive_text(command)

  # ==================== DOport ====================

  def DO(self, index: int, status: int, time: float = None) -> str:
    """
    Set digital outputportstatus（queued command）
    Args:
      index: DOport index (1-based)
      status: status (0=Off, 1=On)
      time: hold forTime(s)。 when status=1when valid，untilTime after Autobecomes0。

    Returns:
      str: ErrorID,{ResultID},DO(index,status,time);
    """
    if status not in [0, 1]:
      raise ValueError("DO status must be 0 or 1")
    if time is not None:
      return self._send_cmd(f"DO({index},{status},{time:.3f})")
    return self._send_cmd(f"DO({index},{status})")

  def DOInstant(self, index: int, status: int) -> str:
    """
    Set digital outputportstatus（immediate command）
    Args:
      index: DOport index (1-based)
      status: status (0=Off, 1=On)

    Returns:
      str: ErrorID,{},DOInstant(index,status);
    """
    if status not in [0, 1]:
      raise ValueError("DO status must be 0 or 1")
    return self._send_cmd(f"DOInstant({index},{status})")

  def GetDO(self, index: int) -> str:
    """
    getDOportstatus（immediate command）
    Args:
      index: DOport index (1-based)

    Returns:
      str: ErrorID,{status},GetDO(index);
    """
    return self._send_cmd(f"GetDO({index})")

  def DOGroup(self, *index_value_pairs) -> str:
    """
    set severalDOportstatus（queued command）
    Args:
      *index_value_pairs: port index andstatuspaired arguments， e.g. DOGroup(4,1,6,0,2,1,7,0)

    Returns:
      str: ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...);
    """
    if len(index_value_pairs) % 2 != 0:
      raise ValueError("DOGroupneed an even number of arguments（index and valuepaired）")
    params = []
    for i in range(0, len(index_value_pairs), 2):
      index = index_value_pairs[i]
      value = index_value_pairs[i + 1]
      if not isinstance(index, int):
        raise ValueError("DOGroup indexmust beinttype")
      if value not in [0, 1]:
        raise ValueError("DOGroup value must be 0 or 1")
      params.append(str(index))
      params.append(str(value))
    return self._send_cmd(f"DOGroup({','.join(params)})")

  def DOGroupDEC(self, indices: Union[List[int], str], value: int) -> str:
    """
    passed set valueset several from a decimal bitmaskDOportstatus（queued command）
    Args:
      indices: port index list， e.g. [1,2,3,4,5] or a string "{1,2,3,4,5}"
      value: decimalvalue
    Returns:
      str: ErrorID,{ResultID},DOGroupDEC({index1,index2,...,indexN},value);
    """
    if isinstance(indices, list):
      indices_str = "{" + ",".join(str(i) for i in indices) + "}"
    else:
      indices_str = indices
    return self._send_cmd(f"DOGroupDEC({indices_str},{value})")

  def GetDOGroup(self, *indices) -> str:
    """
    read severalDOportstatus（immediate command）
    Args:
      *indices: to readDOport number， e.g. GetDOGroup(1,2)

    Returns:
      str: ErrorID,{status1,status2,...},GetDOGroup(index1,index2,...);
    """
    params = ",".join(str(i) for i in indices)
    return self._send_cmd(f"GetDOGroup({params})")

  def GetDOGroupDEC(self, indices: Union[List[int], str]) -> str:
    """
    read severalDOcurrent portstatus，Return valueas a decimal integer（immediate command）

    Args:
      indices: port index list， e.g. [1,2,3] or a string "{1,2,3}"

    Returns:
      str: ErrorID,{value},GetDOGroupDEC({index1,...,indexN});
    """
    if isinstance(indices, list):
      indices_str = "{" + ",".join(str(i) for i in indices) + "}"
    else:
      indices_str = indices
    return self._send_cmd(f"GetDOGroupDEC({indices_str})")

  # ==================== DIport ====================

  def DI(self, index: int) -> str:
    """
    getDI portstatus（immediate command）
    Args:
      index: DIport index (1-based)

    Returns:
      str: ErrorID,{status},DI(index);
    """
    return self._send_cmd(f"DI({index})")

  def DIGroup(self, *indices) -> str:
    """
    read severalDI portstatus（immediate command）
    Args:
      *indices: to readDIport number， e.g. DIGroup(4,6,2,7)

    Returns:
      str: ErrorID,{status1,status2,...},DIGroup(index1,index2,...);
    """
    params = ",".join(str(i) for i in indices)
    return self._send_cmd(f"DIGroup({params})")

  def DIGroupDEC(self, indices: Union[List[int], str]) -> str:
    """
    read severalDI portstatus，Return valueas a decimal integer（queued command）

    Args:
      indices: port index list， e.g. [1,2] or a string "{1,2}"

    Returns:
      str: ErrorID,{value},DIGroupDEC({index1,index2,...,indexN});
    """
    if isinstance(indices, list):
      indices_str = "{" + ",".join(str(i) for i in indices) + "}"
    else:
      indices_str = indices
    return self._send_cmd(f"DIGroupDEC({indices_str})")

  # ==================== analog output ====================

  def AO(self, index: int, value: float) -> str:
    """
    set analog outputvalue（queued command）
    Args:
      index: AOport index (1-based)
      value: analogvalue(0.0-10.0V)

    Returns:
      str: ErrorID,{ResultID},AO(index,value);
    """
    return self._send_cmd(f"AO({index},{value:.2f})")

  def AOInstant(self, index: int, value: float) -> str:
    """
    set analog outputvalue（immediate command）
    Args:
      index: AOport index (1-based)
      value: analogvalue(0.0-10.0V)

    Returns:
      str: ErrorID,{},AOInstant(index,value);
    """
    return self._send_cmd(f"AOInstant({index},{value:.2f})")

  def GetAO(self, index: int) -> str:
    """
    read analog outputvalue（immediate command）
    Args:
      index: AOport index (1-based)

    Returns:
      str: ErrorID,{value},GetAO(index);
    """
    return self._send_cmd(f"GetAO({index})")

  # ==================== analog input ====================

  def AI(self, index: int) -> str:
    """
    getAI portvalue（immediate command）
    Args:
      index: AIport index (1-based)

    Returns:
      str: ErrorID,{value},AI(index);
    """
    return self._send_cmd(f"AI({index})")

  # ==================== toolDOport ====================

  def ToolDO(self, index: int, status: int) -> str:
    """
    set toolDOportstatus（queued command）
    Args:
      index: toolDOport index (0-1)
      status: status (0=Off, 1=On)

    Returns:
      str: ErrorID,{ResultID},ToolDO(index,status);
    """
    if status not in [0, 1]:
      raise ValueError("DO status must be 0 or 1")
    return self._send_cmd(f"ToolDO({index},{status})")

  def ToolDOInstant(self, index: int, status: int) -> str:
    """
    set toolDOportstatus（immediate command）
    Args:
      index: toolDOport index (0-1)
      status: status (0=Off, 1=On)

    Returns:
      str: ErrorID,{},ToolDOInstant(index,status);
    """
    if status not in [0, 1]:
      raise ValueError("DO status must be 0 or 1")
    return self._send_cmd(f"ToolDOInstant({index},{status})")

  def GetToolDO(self, index: int) -> str:
    """
    read toolDOportstatus（immediate command）
    Args:
      index: toolDOport index (0-1)

    Returns:
      str: ErrorID,{status},GetToolDO(index);
    """
    return self._send_cmd(f"GetToolDO({index})")

  # ==================== toolDIport ====================

  def ToolDI(self, index: int) -> str:
    """
    read toolDI portstatus（immediate command）
    Args:
      index: toolDIport index
    Returns:
      str: ErrorID,{status},ToolDI(index);
    """
    return self._send_cmd(f"ToolDI({index})")

  # ==================== tool analog input ====================

  def ToolAI(self, index: int) -> str:
    """
    read toolAI portvalue（immediate command）
    Args:
      index: toolAIport index
    Returns:
      str: ErrorID,{value},ToolAI(index);
    """
    return self._send_cmd(f"ToolAI({index})")

  # ==================== tool setup ====================

  def SetTool485(self, baud: int, parity: str = "N", stopbit: int = 1, identify: int = None) -> str:
    """
    Set tool RS485 format（immediate command）

    Args:
      baud: baud
      parity: parity("O"=odd parity, "E"=even parity, "N"=no parity) default"N"
      stopbit: stop bits default1
      identify: identification params（optional）
    Returns:
      str: ErrorID,{},SetTool485(baud,parity,stopbit[,identify]);
    """
    if identify is not None:
      return self._send_cmd(f'SetTool485({baud},"{parity}",{stopbit},{identify})')
    return self._send_cmd(f'SetTool485({baud},"{parity}",{stopbit})')

  def SetToolPower(self, status: int) -> str:
    """
    Set tool powerstatus（immediate command）
    Args:
      status: powerstatus(0=off, 1=on)

    Returns:
      str: ErrorID,{},SetToolPower(status);
    """
    if status not in [0, 1]:
      raise ValueError("status must be 0 or 1")
    return self._send_cmd(f"SetToolPower({status})")

  def SetToolMode(self, mode: int, type: int = None, identify: int = None) -> str:
    """
    set tool multifunctional pinMode（immediate command）
    Args:
      mode: Modevalue
      type: typevalue（optional）
      identify: identification params（optional）
    Returns:
      str: ErrorID,{},SetToolMode(mode[,type[,identify]]);
    """
    params = [str(mode)]
    if type is not None:
      params.append(str(type))
      if identify is not None:
        params.append(str(identify))
    return self._send_cmd(f"SetToolMode({','.join(params)})")