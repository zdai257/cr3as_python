"""
Example 1a: safe motion

Demonstrate motion commands
"""

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time

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
      
      # Start status monitor
      robot.StartFeedbackMonitor()
      time.sleep(1)
      
      # ========== Joint move (MovJ) ==========
      print("\n--- Joint move (MovJ) ---")
      print("Move to a safe joint pose...")
      safe_pose = [0, -30, -60, 0, 90, 0]
      robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
      time.sleep(3)

      # ========== Joint move (MovJ) ==========
      print("\n--- Joint move (MovJ) ---")
      print("Move to Home...")
      safe_pose = [0, 0, 0, 0, 0, 0]
      robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
      time.sleep(3)
      
      # ========== Read current status ==========
      print("\n--- Read current status ---")
      status = robot.GetStatus()
      if status:
        print(f"Current speed scale: {status.speed_scaling:.1f}%")
        print(f"Current pose:")
        print(f" X: {status.tool_vector_actual.x:.2f}")
        print(f" Y: {status.tool_vector_actual.y:.2f}")
        print(f" Z: {status.tool_vector_actual.z:.2f}")
      
      # Return to a safe pose
      print("\nReturn to a safe pose...")
      robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
      time.sleep(3)
      
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
