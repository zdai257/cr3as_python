"""
Example 4: IO

Digital, analog, and tool IO
"""

from dobot_sdk import DobotRobot
import time


def main():
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("4. IO example")
      print("=" * 50)
      
      # Init
      robot.robot_control.RequestControl()
      robot.robot_control.ClearError()
      robot.robot_control.EnableRobot(load=1.0)
      
      # ========== Digital outputs ==========
      print("\n--- Digital outputs (DO) ---")
      
      # Option 1: DO(index, status)
      print("Turn on DO1...")
      robot.io.DO(1, 1)
      time.sleep(1)
      
      print("Turn off DO1...")
      robot.io.DO(1, 0)
      time.sleep(1)
      
      # Option 2: queued DO
      print("Set DO2 ON...")
      robot.io.DO(2, 1)
      time.sleep(1)
      
      print("Set DO2 OFF...")
      robot.io.DO(2, 0)
      time.sleep(1)
      
      # Option 3: DOInstant
      print("Set DO3 with the immediate command...")
      robot.io.DOInstant(3, 1)
      time.sleep(1)
      robot.io.DOInstant(3, 0)
      time.sleep(1)
      
      # ========== Digital inputs ==========
      print("\n--- Digital inputs (DI) ---")
      for i in range(1, 5):
        di_status = robot.io.DI(i)
        print(f"DI{i} status: {di_status}")
      
      # ========== Read digital outputs ==========
      print("\n--- Read digital outputs (DO) ---")
      for i in range(1, 4):
        do_status = robot.io.GetDO(i)
        print(f"DO{i} status: {do_status}")
      
      # ========== Analog outputs ==========
      print("\n--- Analog outputs (AO) ---")
      print("Set AO1 to 25%...")
      robot.io.AO(1, 25)
      time.sleep(2)
      
      print("Set AO1 to 75%...")
      robot.io.AO(1, 75)
      time.sleep(2)
      
      print("Set AO1 to 0%...")
      robot.io.AO(1, 0)
      time.sleep(1)
      
      # ========== Analog inputs ==========
      print("\n--- Analog inputs (AI) ---")
      for i in range(1, 3):
        ai_value = robot.io.AI(i)
        print(f"AI{i} value: {ai_value}")
      
      # ========== Tool IO ==========
      print("\n--- Tool IO (Tool IO) ---")
      
      # Set tool DO
      print("Set tool DO1 ON...")
      robot.io.ToolDO(1, 1)
      time.sleep(1)
      
      print("Set tool DO1 OFF...")
      robot.io.ToolDO(1, 0)
      time.sleep(1)
      
      # Read tool DI
      print("Read tool DI...")
      tool_di_status = robot.io.ToolDI(1)
      print(f"Tool DI status: {tool_di_status}")
      
      # Enable tool power
      print("\nTurn on tool power...")
      robot.io.SetToolPower(1) # 1 = on
      
      # ========== Batch IO ==========
      print("\n--- Batch IO ---")
      print("Set DO1-DO4 ON...")
      robot.io.DOGroup(1, 1, 2, 1, 3, 1, 4, 1)
      time.sleep(2)
      
      print("Set DO1-DO4 OFF...")
      robot.io.DOGroup(1, 0, 2, 0, 3, 0, 4, 0)
      time.sleep(1)
      
      # Disable
      robot.robot_control.DisableRobot()
      
      print("\n" + "=" * 50)
      print("✅ IO example done")
      print("=" * 50)
      
  except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
