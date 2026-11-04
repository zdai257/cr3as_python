"""
Example 5: coordinate frames

Set and use user/tool frames
"""

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time


def main():
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("5. Coordinate-frame example")
      print("=" * 50)
      
      # Init
      robot.robot_control.RequestControl()
      robot.robot_control.ClearError()
      robot.robot_control.EnableRobot(load=1.0)
      robot.robot_control.SpeedFactor(30)
      
      # ========== Set user frame ==========
      print("\n--- Set user frame ---")
      
      # User frame 1 relative to world
      user_pose = [100, 50, 0, 0, 0, 0] # X+100, Y+50
      print(f"Set user frame 1: {user_pose}")
      robot.robot_control.SetUser(1, user_pose)
      
      # Switch to user frame 1
      print("Switch to user frame 1...")
      robot.robot_control.User(1)
      
      # Move in the user frame
      print("Move in user frame 1 to (100, 0, 300)...")
      pose_in_user = [100, 0, 300, 180, 0, 0]
      robot.motion.MovJ(pose_in_user, CoordinateType.CARTESIAN)
      time.sleep(3)
      
      # Switch back to world frame
      print("Switch back to world frame...")
      robot.robot_control.User(0)
      
      # ========== Set tool frame ==========
      print("\n--- Set tool frame ---")
      
      # Tool frame 1, 100 mm along tool Z
      tool_pose = [0, 0, 100, 0, 0, 0] # Z offset 100 mm
      print(f"Set tool frame 1: {tool_pose}")
      robot.robot_control.SetTool(1, tool_pose)
      
      # Switch to tool frame 1
      print("Switch to tool frame 1...")
      robot.robot_control.Tool(1)
      
      # Move in the tool frame
      print("Move while tool frame 1 is active...")
      pose_in_tool = [400, 0, 300, 180, 0, 0]
      robot.motion.MovJ(pose_in_tool, CoordinateType.CARTESIAN)
      time.sleep(3)
      
      # Switch back to default tool frame
      print("Switch back to default tool frame...")
      robot.robot_control.Tool(0)
      
      # ========== Set payload ==========
      print("\n--- Set payload ---")
      print("Payload 2 kg, CoM (0, 0, 100)...")
      robot.robot_control.SetPayload(2.0, 0, 0, 100)
      
      # ========== Calibrate frames (three-point) ==========
      print("\n--- Frame-calibration demo ---")
      print("CalcUser/CalcTool need taught points")
      print("API usage only; points are not taught here")
      
      # CalcUser example (teach 3 points first)
      # robot.robot_control.CalcUser(1) # needs teaching
      
      # CalcTool example (teach 3 points first)
      # robot.robot_control.CalcTool(1) # needs teaching
      
      # ========== Read current pose / frames ==========
      print("\n--- Frame info ---")
      pose = robot.robot_control.GetPose()
      print(f"Current pose: {pose}")
      
      # Return to a safe pose
      print("\nReturn to a safe pose...")
      safe_pose = [0, -30, -60, 0, 90, 0]
      robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
      time.sleep(3)
      
      # Disable
      robot.robot_control.DisableRobot()
      
      print("\n" + "=" * 50)
      print("✅ Coordinate-frame example done")
      print("=" * 50)
      
  except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
