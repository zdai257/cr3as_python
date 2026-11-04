"""
UI entry
"""

from dobot_sdk import load_robot_config
from ui import RobotUI

# Create and run the robot UI
if __name__ == "__main__":
  robot_ui = RobotUI(robot_ip=load_robot_config().ip)
  robot_ui.pack()
  robot_ui.mainloop()
