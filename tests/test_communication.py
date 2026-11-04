"""
communication tests（Modbus）
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_communication_methods():
  """test communication methods"""
  robot = DobotRobot.from_config()
  
  # testModbusmethod
  assert hasattr(robot.communication, 'ModbusCreate')
  assert hasattr(robot.communication, 'ModbusRTUCreate')
  assert hasattr(robot.communication, 'ModbusClose')
  print("Modbuscreate/close method exists")
  
  # testInput registersmethod
  assert hasattr(robot.communication, 'GetInBits')
  assert hasattr(robot.communication, 'GetInRegs')
  print("Input registersexists")
  
  # test coil methods
  assert hasattr(robot.communication, 'GetCoils')
  assert hasattr(robot.communication, 'SetCoils')
  print("coil methods exist")
  
  # testHolding registersmethod
  assert hasattr(robot.communication, 'GetHoldRegs')
  assert hasattr(robot.communication, 'SetHoldRegs')
  print("Holding registersexists")
  
  del robot
  print("communication method testspassed！")


def test_communication_command_generation():
  """test communication command generation"""
  robot = DobotRobot.from_config()
  
  # testModbuscreate
  result = robot.communication.ModbusCreate("192.168.1.101", 502, 1)
  assert result is not None
  print(f"ModbusCreate('192.168.1.101', 502, 1): {result}")
  
  # test readInput registers
  result = robot.communication.GetInRegs(1, 0, 10)
  assert result is not None
  print(f"GetInRegs(1, 0, 10): {result}")
  
  del robot
  print("communication command-generation testspassed！")


if __name__ == "__main__":
  print("Runningcommunication tests...\n")
  
  test_communication_methods()
  print()
  
  test_communication_command_generation()
  print()
  
  print("=" * 50)
  print("communication moduleAll tests passed")
