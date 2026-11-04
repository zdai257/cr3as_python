"""
IO-module tests
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_io_methods():
  """Test IO methods"""
  robot = DobotRobot.from_config()
  
  # Test digital outputs
  assert hasattr(robot.io, 'DO')
  assert hasattr(robot.io, 'DOInstant')
  print("Digital-output methods exist")
  
  # testDImethod
  assert hasattr(robot.io, 'DI')
  print("DIexists")
  
  # testDOgroup methods
  assert hasattr(robot.io, 'DOGroup')
  assert hasattr(robot.io, 'DOGroupDEC')
  assert hasattr(robot.io, 'GetDOGroup')
  print("DOgroup methods exist")
  
  # testDIgroup methods
  assert hasattr(robot.io, 'DIGroup')
  assert hasattr(robot.io, 'DIGroupDEC')
  print("DIgroup methods exist")
  
  # test analog outputs
  assert hasattr(robot.io, 'AO')
  assert hasattr(robot.io, 'AOInstant')
  assert hasattr(robot.io, 'GetAO')
  assert hasattr(robot.io, 'AI')
  print("analog IO methods exist")
  
  # test toolIOmethod
  assert hasattr(robot.io, 'ToolDO')
  assert hasattr(robot.io, 'ToolDOInstant')
  assert hasattr(robot.io, 'ToolDI')
  assert hasattr(robot.io, 'ToolAI')
  print("toolIOexists")
  
  # test tool setup methods
  assert hasattr(robot.io, 'SetToolPower')
  assert hasattr(robot.io, 'SetToolMode')
  assert hasattr(robot.io, 'SetTool485')
  print("tool setup methods exist")
  
  del robot
  print("IO-module testspassed！")


def test_io_parameter_validation():
  """testIOargument checks"""
  robot = DobotRobot.from_config()
  
  # test DO method
  result = robot.io.DO(1, 1)
  assert result is not None
  print(f"DO(1, 1): {result}")
  
  # test DO method（close）
  result = robot.io.DO(1, 0)
  assert result is not None
  print(f"DO(1, 0): {result}")
  
  # test DI method
  result = robot.io.DI(1)
  assert result is not None
  print(f"DI(1): {result}")
  
  del robot
  print("IOargument-check testspassed！")


if __name__ == "__main__":
  print("RunningIO-module tests...\n")
  
  test_io_methods()
  print()
  
  test_io_parameter_validation()
  print()
  
  print("=" * 50)
  print("IO moduleAll tests passed")
