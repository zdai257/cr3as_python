"""
Example 3: errors and status

Query robot status and error IDs
"""

from dobot_sdk import DobotRobot
import time


def parse_error_response(response):
  """Parse GetErrorID response"""
  # Response format: "ErrorID,{[id,...,id]},GetErrorID();"
  if "ErrorID" in response:
    start = response.find("{") + 1
    end = response.find("}")
    if start > 0 and end > start:
      error_codes_str = response[start:end]
      # Strip inner brackets
      error_codes_str = error_codes_str.strip("[]")
      if error_codes_str.strip():
        return [int(x.strip()) for x in error_codes_str.split(",") if x.strip()]
  return []


def main():
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("3. Error and status query")
      print("=" * 50)
      
      # ========== Get error IDs ==========
      print("\n--- Get error IDs ---")
      response = robot.robot_control.GetErrorID()
      print(f"Raw response: {response}")
      
      error_codes = parse_error_response(response)
      if error_codes:
        print(f"\nFound {len(error_codes)} error IDs:")
        for code in error_codes:
          print(f" - Error ID: {code}")
          print(f"  See the official docs for details")
      else:
        print("✅ No error IDs")
      
      # ========== Get robot mode ==========
      print("\n--- Get robot mode ---")
      mode = robot.robot_control.RobotMode()
      print(f"Robot mode: {mode}")
      
      # ========== getCurrent pose ==========
      print("\n--- Get Cartesian pose ---")
      pose = robot.robot_control.GetPose()
      print(f"Current pose: {pose}")
      
      # ========== Read joints deg ==========
      print("\n--- Get joint angles ---")
      angles = robot.robot_control.GetAngle()
      print(f"Joint angles: {angles}")
      
      # ========== Clear-alarm demo ==========
      print("\n--- Clear-alarm demo ---")
      # Only needed when errors are present
      if error_codes:
        print("Clear alarms...")
        robot.robot_control.ClearError()
        # Re-check
        response = robot.robot_control.GetErrorID()
        error_codes = parse_error_response(response)
        if not error_codes:
          print("✅ Alarms cleared")
      
      # ========== Kinematics demo ==========
      print("\n--- Kinematics ---")
      # FK: joints -> Cartesian
      print("Forward kinematics (joints -> Cartesian)...")
      joints = [0, 0, 0, 0, 90, 0]
      forward_result = robot.robot_control.PositiveKin(joints)
      print(f"Input joints: {joints}")
      print(f"FK result: {forward_result}")
      
      # Reachability check
      print("\n--- Linear reachability (CheckOddMovL) ---")
      # CheckOddMovL uses joint points: start and goal joints
      joints_start = [0, 0, 90, 0, 0, 0]
      joints_end = [90, 30, 0, 0, 0, 0]
      result = robot.robot_control.CheckOddMovL(joints_start, joints_end)
      print(f"Check linear move: {joints_start} -> {joints_end}")
      print(f"Result: {result}")
      
      print("\n--- Arc reachability (CheckOddMovC) ---")
      # CheckOddMovC uses joint points: start, via, goal
      joints_via = [60, 30, 0, 0, 0, 0]
      result = robot.robot_control.CheckOddMovC(joints_start, joints_via, joints_end)
      print(f"Check arc move: {joints_start} -> {joints_via} -> {joints_end}")
      print(f"Result: {result}")
      
      print("\n" + "=" * 50)
      print("✅ Error-monitor example done")
      print("=" * 50)
      
  except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
