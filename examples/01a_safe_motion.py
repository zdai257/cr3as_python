"""
Example 1a: safe motion

Demonstrate motion commands
"""

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time

HOME = [0, 0, 0, 0, 0, 0]
SAFE_POSE = [-115.77, -6.38, 76.08, 21.82, -91.69, 100.43]

def main():
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("2. Motion-control example")
      print("=" * 50)
      
      # Init
      robot.robot_control.RequestControl()
      robot.robot_control.ClearError()
      robot.robot_control.EnableRobot(load=1.0)

      robot.robot_control.SpeedFactor(10)
      robot.robot_control.AccJ(10)
      
      # Start status monitor
      robot.StartFeedbackMonitor()
      time.sleep(1)
      
      # ========== Joint move (MovJ) ==========
      print("\n--- Joint move (MovJ) ---")
      print("Move to a HOME pose...")
      home = HOME
      robot.motion.MovJ(home, CoordinateType.JOINT)
      time.sleep(8)

      # ========== Joint move (MovJ) ==========
      print("\n--- Joint move (MovJ) ---")
      print("Move to SAFE_POSE...")
      safe_pose = SAFE_POSE
      robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
      time.sleep(8)
      
      # ========== Read current status ==========
      print("\n--- Read current status ---")
      status = robot.GetStatus()
      if status:
        print(f"Current speed scale: {status.speed_scaling:.1f}%")
        print(f"Current pose:")
        print(f" X: {status.tool_vector_actual.x:.2f}")
        print(f" Y: {status.tool_vector_actual.y:.2f}")
        print(f" Z: {status.tool_vector_actual.z:.2f}")
      
      # Stop monitor
      robot.StopFeedbackMonitor()
      robot.robot_control.DisableRobot()
      
      print("\n" + "=" * 50)
      print("✅ Motion-control example done")
      print("=" * 50)
      
  except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
