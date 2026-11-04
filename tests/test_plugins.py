"""
Plugin moduletest（force and conveyor）
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dobot_sdk import DobotRobot


def test_plugins_methods():
  """testPlugin modulemethod"""
  robot = DobotRobot.from_config()
  
  # test force-control methods
  assert hasattr(robot.plugins, 'FCForceMode')
  assert hasattr(robot.plugins, 'FCSetDeviation')
  assert hasattr(robot.plugins, 'FCSetForceLimit')
  assert hasattr(robot.plugins, 'FCSetMass')
  assert hasattr(robot.plugins, 'FCSetStiffness')
  assert hasattr(robot.plugins, 'FCSetDamping')
  assert hasattr(robot.plugins, 'FCOff')
  assert hasattr(robot.plugins, 'FCSetForce')
  assert hasattr(robot.plugins, 'SetFCCollision')
  assert hasattr(robot.plugins, 'FCCollisionSwitch')
  print("force-control methods exist")
  
  # test conveyor methods
  assert hasattr(robot.plugins, 'CnvInit')
  assert hasattr(robot.plugins, 'GetCnvObject')
  assert hasattr(robot.plugins, 'StartSyncCnv')
  assert hasattr(robot.plugins, 'CnvMovL')
  assert hasattr(robot.plugins, 'CnvMovC')
  assert hasattr(robot.plugins, 'StopSyncCnv')
  assert hasattr(robot.plugins, 'SetCnvPointOffset')
  assert hasattr(robot.plugins, 'SetCnvTimeCompensation')
  print("conveyor methods exist")
  
  del robot
  print("Plugin modulemethod testspassed！")


def test_plugins_command_generation():
  """test plugin command generation"""
  robot = DobotRobot.from_config()
  
  # test forceModeset
  result = robot.plugins.FCForceMode([1,1,1,0,0,0], [0,0,0,0,0,0])
  assert result is not None
  print(f"FCForceMode([1,1,1,0,0,0], [0,0,0,0,0,0]): {result}")
  
  # test force off
  result = robot.plugins.FCOff()
  assert result is not None
  print(f"FCOff(): {result}")
  
  # test conveyorInit
  result = robot.plugins.CnvInit(1)
  assert result is not None
  print(f"CnvInit(1): {result}")
  
  del robot
  print("plugin command-generation testspassed！")


if __name__ == "__main__":
  print("RunningPlugin moduletest...\n")
  
  test_plugins_methods()
  print()
  
  test_plugins_command_generation()
  print()
  
  print("=" * 50)
  print("Plugin moduleAll tests passed")
