"""
Example 1: basic connect and enable

Connect, enable, set parameters, and shut down safely
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dobot_sdk import DobotRobot
import time


def main():
  # Robot IP (override via config)
  # Context manager (recommended)
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("1. Connected")
      print("=" * 50)
      
      # Request TCP control mode
      print("Request TCP control mode...")
      response = robot.robot_control.RequestControl()
      print(f"response: {response}")
      
      # Clear alarms if any
      print("Clear alarms...")
      robot.robot_control.ClearError()
      
      # Enable robot (1 kg payload)
      print("Enable robot...")
      response = robot.robot_control.EnableRobot(load=1.0)
      print(f"response: {response}")
      
      # Set global speed scale
      speed = 50
      print(f"Set global speed to {speed}%...")
      robot.robot_control.SpeedFactor(speed)
      
      # Read robot status
      print("\nRead robot status...")
      mode = robot.robot_control.RobotMode()
      print(f"Robot mode: {mode}")
      
      # Wait 2 s
      print(f"\nWait {2} s...")
      time.sleep(2)
      
      skinonoff = robot.robot_control.EnableSafeSkin(1)
      skinsetting = robot.robot_control.SetSafeSkin(5, 2)
      print(f"\n SafeSkin: {skinsetting}")
      
      # Disable robot
      print("\nDisable robot...")
      response = robot.robot_control.DisableRobot()
      print(f"response: {response}")
      
      print("\n" + "=" * 50)
      print("✅ Basic-connection example done")
      print("=" * 50)
      
  except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
