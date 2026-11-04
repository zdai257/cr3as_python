# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Logging demo

How to use SDK logging
"""

import sys
sys.path.insert(0, '..')

from dobot_sdk import DobotRobot, CoordinateType, get_logger, set_log_level, get_log_directory


def main():
  # Set DEBUG (optional; default INFO)
  # Levels: DEBUG, INFO, WARNING, ERROR
  set_log_level("DEBUG")
  
  # Get the logger (optional custom output)
  logger = get_logger()
  
  # Log directory
  log_dir = get_log_directory()
  print(f"Log files go to: {log_dir}")
  
  # Create robot (IP comes from config)
  robot = DobotRobot.from_config()
  
  try:
    # Connect (logged)
    robot.Connect()
    
    # Request control mode
    robot.robot_control.RequestControl()
    
    # Clear alarms
    robot.robot_control.ClearError()
    
    # Enable (logged)
    robot.robot_control.EnableRobot()
    
    # Set speed (logged)
    robot.robot_control.SpeedFactor(50)
    
    # Motion command (logged)
    start_pose = [400, 0, 300, 180, 0, 0]
    robot.motion.MovL(start_pose, CoordinateType.CARTESIAN)
    
    # Disable robot
    robot.robot_control.DisableRobot()
    
  except Exception as e:
    logger.error(f"Operation failed: {e}", exc_info=True)
  finally:
    # Disconnect (logged)
    robot.Disconnect()


if __name__ == "__main__":
  main()
