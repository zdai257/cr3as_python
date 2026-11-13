"""
robot-control module tests
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_robot_control_methods():
  """test robot-control methods"""
  robot = DobotRobot.from_config()
  
  # test RequestControl
  assert hasattr(robot.robot_control, 'RequestControl')
  assert hasattr(robot.robot_control, 'EnableRobot')
  assert hasattr(robot, 'TakeTcpControl')
  assert hasattr(robot, 'ResetSafety')
  print("RequestControl exists")
  
  # test enable methods
  assert hasattr(robot.robot_control, 'EnableRobot')
  assert hasattr(robot.robot_control, 'DisableRobot')
  print("enable/disable methods exist")
  
  # testErrorhandler
  assert hasattr(robot.robot_control, 'ClearError')
  assert hasattr(robot.robot_control, 'GetErrorID')
  print("Errorhandler exists")
  
  # testStatus querymethod
  assert hasattr(robot.robot_control, 'GetPose')
  assert hasattr(robot.robot_control, 'GetAngle')
  assert hasattr(robot.robot_control, 'RobotMode')
  print("Status queryexists")
  
  # testspeedsetter
  assert hasattr(robot.robot_control, 'SpeedFactor')
  print("speedsetter exists")
  
  # testFramescalc methods
  assert hasattr(robot.robot_control, 'CalcUser')
  assert hasattr(robot.robot_control, 'CalcTool')
  print("Framescalc methods exist")
  
  # test collision methods
  assert hasattr(robot.robot_control, 'SetCollisionLevel')
  print("collision methods exist")
  
  del robot
  print("robot-control method testspassed！")


def test_robot_control_parameter_validation():
  """test robot-control argument checks"""
  robot = DobotRobot.from_config()
  
  # testenable_robotmethod
  result = robot.robot_control.EnableRobot(load=1.0)
  assert result is not None
  print(f"enable_robot(load=1.0): {result}")
  
  # testspeed_factormethod
  result = robot.robot_control.SpeedFactor(50)
  assert result is not None
  print(f"SpeedFactor(50): {result}")
  
  # testset_collision_levelmethod（argument range0-5）
  result = robot.robot_control.SetCollisionLevel(3)
  assert result is not None
  print(f"SetCollisionLevel(3): {result}")
  
  # testcalc_usermethod
  result = robot.robot_control.CalcUser(0, 1, [0, 0, 0, 0, 0, 0])
  assert result is not None
  print(f"CalcUser(0, 1, [0,0,0,0,0,0]): {result}")
  
  del robot
  print("robot-control argument checkspassed！")


if __name__ == "__main__":
  print("Runningrobot-control module tests...\n")
  
  test_robot_control_methods()
  print()
  
  test_robot_control_parameter_validation()
  print()
  
  print("=" * 50)
  print("robot-control moduleAll tests passed")
