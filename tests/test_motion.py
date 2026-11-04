"""
Motion-module tests
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType


def test_motion_methods():
  """Test motion methods"""
  robot = DobotRobot.from_config()
  
  # Test MovJ
  assert hasattr(robot.motion, 'MovJ')
  print("MovJ exists")
  
  # Test MovL
  assert hasattr(robot.motion, 'MovL')
  print("MovL exists")
  
  # Test Arc
  assert hasattr(robot.motion, 'Arc')
  print("Arc exists")
  
  # Test Circle
  assert hasattr(robot.motion, 'Circle')
  print("Circleexists")
  
  # testRelative movemethod
  assert hasattr(robot.motion, 'RelMovL')
  assert hasattr(robot.motion, 'RelMovLTool')
  assert hasattr(robot.motion, 'RelMovLUser')
  print("Relative moveexists")
  
  # testmovestype setter（these APIs now live in robot_control provided by）
  assert hasattr(robot.robot_control, 'SpeedFactor')
  assert hasattr(robot.robot_control, 'AccJ')
  assert hasattr(robot.robot_control, 'AccL')
  print("speed/ accelspeedsetter exists (robot_control module)")
  
  # note： origin motion under module ToolVoltage API in the arm SDK docsmdnot in the docs，removed
  print("toolVoltagerelated checks：not in the docs, skipped，skipped")
  
  # test servo methods
  assert hasattr(robot.motion, 'ServoJ')
  assert hasattr(robot.motion, 'ServoP')
  print("servo methods exist")
  
  # testJogcontrol methods
  assert hasattr(robot.motion, 'MoveJog')
  print("Jogcontrol methods exist")
  
  del robot
  print("Motion controlmethod testspassed！")


def test_motion_command_generation():
  """testmovescommand generation"""
  robot = DobotRobot.from_config()
  
  # testMovJcommand generation
  result = robot.motion.MovJ([400, 0, 300, 180, 0, 0], CoordinateType.CARTESIAN)
  assert result is not None
  print(f"MovJcommand: {result}")
  
  # testMovLcommand generation
  result = robot.motion.MovL([400, 100, 300, 180, 0, 0], CoordinateType.CARTESIAN)
  assert result is not None
  print(f"MovLcommand: {result}")
  
  # testRelative move（unpack6 independentoffsetargs，as in the protocol）
  result = robot.motion.RelMovLTool(50, 0, 0, 0, 0, 0)
  assert result is not None
  print(f"Relative movecommand: {result}")
  
  # test jointmoves
  result = robot.motion.MovJ([0, -30, -60, 0, 90, 0], CoordinateType.JOINT)
  assert result is not None
  print(f"jointmovescommand: {result}")
  
  del robot
  print("movescommand-generation testspassed！")


def test_coordinate_type_enum():
  """testCoordinate typeenum"""
  assert CoordinateType.JOINT == 0
  assert CoordinateType.CARTESIAN == 1
  assert CoordinateType.USER == 1
  assert CoordinateType.TOOL == 2
  assert CoordinateType.CARTESIAN.name == 'CARTESIAN'
  assert CoordinateType.JOINT.name == 'JOINT'
  assert CoordinateType.USER.name == 'USER'
  assert CoordinateType.TOOL.name == 'TOOL'
  print("Coordinate typeenum testspassed！")


if __name__ == "__main__":
  print("RunningMotion-module tests...\n")
  
  test_motion_methods()
  print()
  
  test_motion_command_generation()
  print()
  
  test_coordinate_type_enum()
  print()
  
  print("=" * 50)
  print("Motion controlmoduleAll tests passed")
