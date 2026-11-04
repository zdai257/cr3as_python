"""
Example 6: force and conveyor

FT sensor and conveyor tracking
"""

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time


def main():
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("6. Force and conveyor example")
      print("=" * 50)
      
      # Init
      robot.robot_control.RequestControl()
      robot.robot_control.ClearError()
      robot.robot_control.EnableRobot(load=1.0)
      robot.robot_control.SpeedFactor(30)
      
      # ========== FT sensor ==========
      print("\n--- FT sensor ---")
      
      # Check whether an FT sensor is present
      print("Enable FT sensor...")
      response = robot.plugins.EnableFTSensor(1)
      print(f"response: {response}")
      time.sleep(1)
      
      # Zero the FT sensor
      print("Zero the FT sensor...")
      response = robot.plugins.SixForceHome()
      print(f"response: {response}")
      time.sleep(1)
      
      # Read FT data
      print("Read FT data...")
      for i in range(3):
        force_data = robot.plugins.GetForce()
        print(f"Force {i+1}: {force_data}")
        time.sleep(0.5)
      
      # Enter force-drag mode
      print("\nEnter force-drag mode...")
      response = robot.plugins.ForceDriveMode(1)
      print(f"response: {response}")
      print("5 s window to drag the robot...")
      time.sleep(5)
      
      # Leave force-drag mode
      print("Leave force-drag mode...")
      response = robot.plugins.ForceDriveMode(0)
      print(f"response: {response}")
      
      # Disable FT sensor
      print("Disable FT sensor...")
      response = robot.plugins.EnableFTSensor(0)
      print(f"response: {response}")
      
      # ========== Conveyor (demo) ==========
      print("\n--- Conveyor ---")
      
      # A real conveyor needs encoder setup first
      print("Enable conveyor...")
      response = robot.plugins.CnvInit(1)
      print(f"response: {response}")
      
      # Start conveyor tracking
      print("Start conveyor tracking...")
      response = robot.plugins.StartSyncCnv()
      print(f"response: {response}")
      
      # Simulated conveyor tracking
      print("Wait for a part in the pick zone...")
      # response = robot.plugins.GetCnvObject(0)
      # print(f"Part-detect response: {response}")
      
      print("Stop conveyor tracking...")
      response = robot.plugins.StopSyncCnv()
      print(f"response: {response}")
      
      print("Disable conveyor...")
      response = robot.plugins.CnvInit(0)
      print(f"response: {response}")
      
      # Return to a safe pose
      print("\nReturn to a safe pose...")
      safe_pose = [0, -30, -60, 0, 90, 0]
      robot.motion.MovJ(safe_pose, CoordinateType.JOINT)
      time.sleep(3)
      
      # Disable
      robot.robot_control.DisableRobot()
      
      print("\n" + "=" * 50)
      print("✅ Force/conveyor example done")
      print("=" * 50)
      
  except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
