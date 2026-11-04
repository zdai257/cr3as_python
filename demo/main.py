"""
Main entry
"""

from dobot_sdk import load_robot_config
from DobotDemo import DobotDemo

if __name__ == '__main__':
  # Robot IP (from config)
  dobot = DobotDemo(load_robot_config().ip)
  dobot.start()
