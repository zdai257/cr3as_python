"""
Basic tests
"""

import sys
import os

# Add SDK path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot, load_robot_config
from dobot_sdk.models import RobotStatus, RobotMode
from dobot_sdk import CoordinateType


def test_import():
  """Test imports"""
  print("Import OK")
  assert DobotRobot is not None
  print("DobotRobot available")
  assert CoordinateType is not None
  print("CoordinateType available")


def test_models():
  """Test models"""
  # Test RobotStatus
  status = RobotStatus()
  assert status.robot_mode == RobotMode.INIT
  assert status.speed_scaling == 0.0
  print("RobotStatus OK")
  
  # Test RobotMode
  assert RobotMode.ENABLE == 5
  assert RobotMode.ERROR == 9
  print("RobotMode OK")


def test_robot_creation():
  """Test robot construction"""
  robot = DobotRobot.from_config()
  assert robot.ip == load_robot_config().ip
  
  # Check modules
  assert robot.motion is not None
  print("motion available")
  
  assert robot.io is not None
  print("io available")
  
  assert robot.communication is not None
  print("communication available")
  
  assert robot.plugins is not None
  print("plugins available")
  
  assert robot.robot_control is not None
  print("robot_control available")
  
  print("Robot object created")
  
  # Cleanup
  del robot


def test_coordinate_type():
  """Test CoordinateType"""
  assert CoordinateType.JOINT == 0
  assert CoordinateType.CARTESIAN == 1
  assert CoordinateType.USER == 1
  assert CoordinateType.TOOL == 2
  print("CoordinateType values OK")


if __name__ == "__main__":
  print("Running basic tests...\n")
  
  test_import()
  print()
  
  test_models()
  print()
  
  test_robot_creation()
  print()
  
  test_coordinate_type()
  print()
  
  print("=" * 50)
  print("All tests passed")
