"""
example2: Motion control

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
      robot.robot_control.SpeedFactor(30)
      
      # Start status monitor
      robot.StartFeedbackMonitor()
      time.sleep(1)
      
      # ========== Joint move (MovJ) ==========
      print("\n--- Joint move (MovJ) ---")
      print("Move to a safe joint pose...")
      safe_pose = [0, -30, -60, 0, 90, 0]
      robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
      time.sleep(3)
      
      # ========== Cartesianmoves (MovJ) ==========
      print("\n--- Cartesian MovJ ---")
      pose_a = [400, 0, 300, 180, 0, 0]
      print(f"Move to point A: {pose_a}")
      robot.motion.MovJ(pose_a, CoordinateType.CARTESIAN)
      time.sleep(3)
      
      # ========== Linear move (MovL) ==========
      print("\n--- Linear move (MovL) ---")
      pose_b = [400, 100, 300, 180, 0, 0]
      print(f"Linear move to point B: {pose_b}")
      robot.motion.MovL(pose_b, CoordinateType.CARTESIAN)
      time.sleep(3)
      
      # ========== Arc move ==========
      print("\n--- Arc move ---")
      pose_via = [450, 150, 300, 180, 0, 0]
      pose_c = [500, 100, 300, 180, 0, 0]
      print(f"Arc: B -> via -> C")
      robot.motion.Arc(pose_via, pose_c, CoordinateType.CARTESIAN)
      time.sleep(3)
      
      # ========== Circle move ==========
      print("\n--- Circle move ---")
      print(f"Circle: C -> via -> B")
      robot.motion.Circle(pose_via, pose_b, count=1, coord_type=CoordinateType.CARTESIAN)
      time.sleep(4)
      
      # ========== Relative move ==========
      print("\n--- Relative move (RelMovLTool) ---")
      offset = [0, -50, 0, 0, 0, 0]
      print(f"Offset: {offset}")
      robot.motion.RelMovLTool(offset[0], offset[1], offset[2], offset[3], offset[4], offset[5])
      time.sleep(2)
      
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
