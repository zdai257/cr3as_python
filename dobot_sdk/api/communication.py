# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Communication: Modbus and bus registers"""

from typing import List, Union
from ..core.connection import DobotConnection


class Communication:
  """Communication module: Modbus and bus registers"""

  def __init__(self, connection: DobotConnection):
    self.connection = connection

  def _send_cmd(self, command: str) -> str:
    """Send command and receive response"""
    return self.connection.send_receive_text(command)

  # ==================== Modbus master create ====================

  def ModbusCreate(self, ip: str, port: int, slave_id: int, is_rtu: int = 0) -> str:
    """
    Create a Modbus master

    Args:
      ip: Slave IP
      port: Slave port
      slave_id: Slave ID
      is_rtu: RTU mode (0=TCP, 1=RTU), default 0

    Returns:
      str: ErrorID,{index},ModbusCreate(ip,port,slave_id,isRTU);
         index is the master handle used by later Modbus calls
    """
    cmd = f'ModbusCreate("{ip}",{port},{slave_id},{is_rtu})'
    return self._send_cmd(cmd)

  def ModbusRTUCreate(self, slave_id: int, baud: int, 
             parity: str = "E", data_bit: int = 8, 
             stop_bit: int = 1) -> str:
    """
    Create a Modbus master on RS485

    Args:
      slave_id: Slave ID
      baud: RS485 baud rate
      parity: Parity ("O"=odd, "E"=even, "N"=none, default "E")
      data_bit: Data bits, default 8
      stop_bit: Stop bits, default 1

    Returns:
      str: ErrorID,{index},ModbusRTUCreate(slave_id,baud,parity,data_bit,stop_bit);
    """
    if parity not in ["O", "E", "N"]:
      raise ValueError("parity must be 'O', 'E' or 'N'")
    
    cmd = f'ModbusRTUCreate({slave_id},{baud},"{parity}",{data_bit},{stop_bit})'
    return self._send_cmd(cmd)

  def ModbusClose(self, index: int) -> str:
    """
    Close the Modbus slave connection

    Args:
      index: Master index returned by create
    """
    return self._send_cmd(f"ModbusClose({index})")

  # ==================== Discrete inputs====================

  def GetInBits(self, index: int, addr: int, count: int) -> str:
    """
    Read discrete inputs

    Args:
      index: Master index returned by create
      addr: Discrete-input start address
      count: Count in [1, 16]

    Returns:
      str: ErrorID,{value1,value2,...,valuen},GetInBits(index,addr,count);
    """
    if not 1 <= count <= 16:
      raise ValueError("count must be in [1, 16]")
    
    cmd = f"GetInBits({index},{addr},{count})"
    return self._send_cmd(cmd)

  # ==================== Input registers====================

  def GetInRegs(self, index: int, addr: int, count: int, val_type: str = "U16") -> str:
    """
    Read input registers

    Args:
      index: Master index returned by create
      addr: Input-register start address
      count: Count in [1, 4]
      val_type: Value type
           U16（16-bit unsigned (2 bytes, 1 register)；
           U32（32-bit unsigned (4 bytes, 2 registers)；
           F32（32-bit float (4 bytes, 2 registers)；
           F64（64-bit float (8 bytes, 4 registers)；
           Default U16

    Returns:
      str: ErrorID,{value1,value2,...,valuen},GetInRegs(index,addr,count,valType);
    """
    if not 1 <= count <= 4:
      raise ValueError("count must be in [1, 4]")
    if val_type not in ["U16", "U32", "F32", "F64"]:
      raise ValueError("val_type must be U16/U32/F32/F64")
    
    cmd = f'GetInRegs({index},{addr},{count},"{val_type}")'
    return self._send_cmd(cmd)

  # ==================== Coils====================

  def GetCoils(self, index: int, addr: int, count: int) -> str:
    """
    Read coils

    Args:
      index: Master index returned by create
      addr: Coil start address
      count: Count in [1, 16]

    Returns:
      str: ErrorID,{value1,value2,...,valuen},GetCoils(index,addr,count);
    """
    if not 1 <= count <= 16:
      raise ValueError("count must be in [1, 16]")
    
    cmd = f"GetCoils({index},{addr},{count})"
    return self._send_cmd(cmd)

  def SetCoils(self, index: int, addr: int, count: int, val_tab: Union[List[int], str]) -> str:
    """
    Write coils

    Args:
      index: Master index returned by create
      addr: Coil start address
      count: Count in [1, 16]
      val_tab: Values; length must equal count

    Returns:
      str: ErrorID,{},SetCoils(index,addr,count,valTab);
    """
    if not 1 <= count <= 16:
      raise ValueError("count must be in [1, 16]")
    
    if isinstance(val_tab, list):
      val_str = "{" + ",".join(str(v) for v in val_tab) + "}"
    else:
      val_str = val_tab
    
    cmd = f"SetCoils({index},{addr},{count},{val_str})"
    return self._send_cmd(cmd)

  def SetSingleCoil(self, index: int, addr: int, value: int) -> str:
    """
    Write a single coil (added in V4.6.6)

    Args:
      index: Master index returned by create
      addr: Coil address
      value: Value, 0 or 1

    Returns:
      str: ErrorID,{},SetSingleCoil(index,addr,value);
    """
    if value not in [0, 1]:
      raise ValueError("value must be 0 or 1")
    
    cmd = f"SetSingleCoil({index},{addr},{value})"
    return self._send_cmd(cmd)

  # ==================== Holding registers====================

  def GetHoldRegs(self, index: int, addr: int, count: int, val_type: str = "U16") -> str:
    """
    Read holding registers

    Args:
      index: Master index in [0, 4]
      addr: Holding-register start address
      count: Number of holding registers to read
      val_type: Value type
           U16（16-bit unsigned (2 bytes, 1 register)；
           U32（32-bit unsigned (4 bytes, 2 registers)；
           F32（32-bit float (4 bytes, 2 registers)；
           F64（64-bit float (8 bytes, 4 registers)；
           Default U16

    Returns:
      str: ErrorID,{value1,value2,...,valuen},GetHoldRegs(index,addr,count,valType);
    """
    if not 0 <= index <= 4:
      raise ValueError("index must be in [0, 4]")
    if val_type not in ["U16", "U32", "F32", "F64"]:
      raise ValueError("val_type must be U16/U32/F32/F64")
    
    cmd = f'GetHoldRegs({index},{addr},{count},"{val_type}")'
    return self._send_cmd(cmd)

  def SetHoldRegs(self, index: int, addr: int, count: int, 
           val_tab: Union[List[int], str], val_type: str = "U16") -> str:
    """
    Write holding registers

    Args:
      index: Master index in [0, 4]
      addr: Holding-register start address
      count: Count in [1, 4]
      val_tab: Values; length must equal count
      val_type: Value type
           U16（16-bit unsigned (2 bytes, 1 register)；
           U32（32-bit unsigned (4 bytes, 2 registers)；
           F32（32-bit float (4 bytes, 2 registers)；
           F64（64-bit float (8 bytes, 4 registers)；
           Default U16

    Returns:
      str: ErrorID,{},SetHoldRegs(index,addr,count,valTab,valType);
    """
    if not 0 <= index <= 4:
      raise ValueError("index must be in [0, 4]")
    if not 1 <= count <= 4:
      raise ValueError("count must be in [1, 4]")
    if val_type not in ["U16", "U32", "F32", "F64"]:
      raise ValueError("val_type must be U16/U32/F32/F64")
    
    if isinstance(val_tab, list):
      val_str = "{" + ",".join(str(v) for v in val_tab) + "}"
    else:
      val_str = val_tab
    
    cmd = f'SetHoldRegs({index},{addr},{count},{val_str},"{val_type}")'
    return self._send_cmd(cmd)

  def SetSingleHoldReg(self, index: int, addr: int, value: int) -> str:
    """
    Write a single holding register (V4.6.6)

    Args:
      index: Master index in [0, 4]
      addr: Holding-register address
      value: Value to write

    Returns:
      str: ErrorID,{},SetSingleHoldReg(index,addr,value);
    """
    if not 0 <= index <= 4:
      raise ValueError("index must be in [0, 4]")
    
    cmd = f"SetSingleHoldReg({index},{addr},{value})"
    return self._send_cmd(cmd)

  # ==================== Bus registers - input====================

  def GetInputBool(self, address: int) -> str:
    """
    Read a bool from an input register

    Args:
      address: Register address in [0, 63]

    Returns:
      str: ErrorID,{value},GetInputBool(address);
         value value at registervalue， 0 or 1
    """
    if not 0 <= address <= 63:
      raise ValueError("address valuerange must be [0, 63]")
    
    return self._send_cmd(f"GetInputBool({address})")

  def GetInputInt(self, address: int) -> str:
    """
    getInput registersat addressint typed value

    Args:
      address: Register address in [0, 23]

    Returns:
      str: ErrorID,{value},GetInputInt(address);
         value value at registervalue，as int32（int32）
    """
    if not 0 <= address <= 23:
      raise ValueError("address must be in [0, 23]")
    
    return self._send_cmd(f"GetInputInt({address})")

  def GetInputFloat(self, address: int) -> str:
    """
    getInput registersat addressfloat typed value

    Args:
      address: Register address in [0, 23]

    Returns:
      str: ErrorID,{value},GetInputFloat(address);
         value value at registervalue，as float32pt（float）
    """
    if not 0 <= address <= 23:
      raise ValueError("address must be in [0, 23]")
    
    return self._send_cmd(f"GetInputFloat({address})")

  # ==================== bus registers- output registers====================

  def GetOutputBool(self, address: int) -> str:
    """
    read output register atbool typed value

    Args:
      address: Register address in [0, 63]

    Returns:
      str: ErrorID,{value},GetOutputBool(address);
         value value at registervalue， 0 or 1
    """
    if not 0 <= address <= 63:
      raise ValueError("address valuerange must be [0, 63]")
    
    return self._send_cmd(f"GetOutputBool({address})")

  def GetOutputInt(self, address: int) -> str:
    """
    read output register atint typed value

    Args:
      address: Register address in [0, 23]

    Returns:
      str: ErrorID,{value},GetOutputInt(address);
         value value at registervalue，as int32（int32）
    """
    if not 0 <= address <= 23:
      raise ValueError("address must be in [0, 23]")
    
    return self._send_cmd(f"GetOutputInt({address})")

  def GetOutputFloat(self, address: int) -> str:
    """
    read output register atfloat typed value

    Args:
      address: Register address in [0, 23]

    Returns:
      str: ErrorID,{value},GetOutputFloat(address);
         value value at registervalue，as float32pt（float）
    """
    if not 0 <= address <= 23:
      raise ValueError("address must be in [0, 23]")
    
    return self._send_cmd(f"GetOutputFloat({address})")

  def SetOutputBool(self, address: int, value: int) -> str:
    """
    write output register at bool value

    Args:
      address: Register address in [0, 63]
      value: value to writevalue，0 or 1

    Returns:
      str: ErrorID,{},SetOutputBool(address,value);
    """
    if not 0 <= address <= 63:
      raise ValueError("address valuerange must be [0, 63]")
    if value not in [0, 1]:
      raise ValueError("value must be0 or 1")
    
    return self._send_cmd(f"SetOutputBool({address},{value})")

  def SetOutputInt(self, address: int, value: int) -> str:
    """
    write output register at int value

    Args:
      address: Register address in [0, 23]
      value: value to write int32 value

    Returns:
      str: ErrorID,{},SetOutputInt(address,value);
    """
    if not 0 <= address <= 23:
      raise ValueError("address must be in [0, 23]")
    
    return self._send_cmd(f"SetOutputInt({address},{value})")

  def SetOutputFloat(self, address: int, value: float) -> str:
    """
    write output register at float value

    Args:
      address: Register address in [0, 23]
      value: value to write float value

    Returns:
      str: ErrorID,{},SetOutputFloat(address,value);
    """
    if not 0 <= address <= 23:
      raise ValueError("address must be in [0, 23]")
    
    return self._send_cmd(f"SetOutputFloat({address},{value})")


