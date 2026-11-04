"""
Conveyor test

Conveyor tracking demo
"""

import sys
import os

# Add parent dir so dobot_sdk imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType
import time


def main():
  # Robot IP (from config)
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("Conveyor test")
      print("=" * 50)
      
      # Init
      robot.robot_control.RequestControl()
      robot.robot_control.ClearError()
      robot.robot_control.EnableRobot(load=1.0)
      
      # Query robot mode
      mode_response = robot.robot_control.RobotMode()
      print(f"Robot mode: {mode_response}")
      
      while True:
        # Init conveyor
        print("\nInit conveyor...")
        robot.plugins.CnvInit(1)
        
        # move to the camera pose/Wait
        print("move toWait...")
        robot.motion.MovJ(
          [-29.3427, -386.0646, 248.1024, 180.0000, -0.0000, -154.3856],
          CoordinateType.CARTESIAN,
          user=0, tool=0
        )
        time.sleep(3)
        
        # poll until a part is in the pick zone
        print("\nWaitpart entered the pick zone...")
        while True:
          cnv_status = robot.plugins.GetCnvObject(0)
          print(f"conveyorstatus: {cnv_status}")
          
          # parse responsestatus
          try:
            # Response formatlike: "GetCnvObject,{status},GetCnvObject();"
            start = cnv_status.find("{") + 1
            end = cnv_status.find("}")
            if start > 0 and end > start:
              status_values = cnv_status[start:end].split(",")
              if len(status_values) > 3:
                object_detected = int(status_values[3])
                if object_detected == 1:
                  print("part detected!")
                  break
          except Exception as e:
            print(f"parsestatusfailed: {e}")
          
          time.sleep(0.2)
        
        # start sync tracking
        print("\nstart conveyor sync...")
        robot.plugins.StartSyncCnv()
        
        # run conveyor trackingmoves
        print("run conveyor trackingmoves...")
        robot.plugins.CnvMovL([0, 0, 0, 0, 0, 153])
        time.sleep(2)
        
        # fire gripper / vacuum (DO6)
        print("fire vacuum...")
        robot.io.DO(6, 1)
        
        # descend and grasp
        print("descend and grasp...")
        robot.plugins.CnvMovL([0, 0, -50, 0, 0, 153])
        time.sleep(3)
        
        # stop sync
        print("stop conveyor sync...")
        robot.plugins.StopSyncCnv()
        
        # Query robot mode
        mode_response = robot.robot_control.RobotMode()
        print(f"Robot mode: {mode_response}")
        
        # stopmoves
        robot.robot_control.Stop()
        
        # move to place pose
        print("move to place pose...")
        robot.motion.MovL(
          [212.5693, -395.0977, 209.9998, 179.9999, -0.0001, -154.3857],
          CoordinateType.CARTESIAN,
          user=0, tool=0
        )
        time.sleep(3)
        
        # release vacuum
        robot.io.DO(6, 0)
        
        # return to wait pose
        print("return to wait pose...")
        robot.motion.MovJ(
          [-29.3427, -386.0646, 248.1024, 180.0000, -0.0000, -154.3856],
          CoordinateType.CARTESIAN,
          user=0, tool=0
        )
        time.sleep(3)
      
  except KeyboardInterrupt:
    print("\ninterrupted by user")
  except Exception as e:
    print(f"\nError: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
