from dobot_sdk import DobotRobot,CoordinateType
# from dobot_sdk.api.motion import CoordinateType
import time

with DobotRobot.from_config() as robot:
  robot.robot_control.RequestControl()
  robot.robot_control.EnableRobot()

  # p1=[-200, -70, 400, -180, 0, -180]
  # p2=[-200, -100, 400, -180, 0, -180]
  p1=[-200, -70, 400, -180, 0, -180]
  p2=[-8.7346, -27.2151, 83.8642, 33.3090, -89.7513, -98.6253]  

  # Cartesian move
  robot.motion.MovJ(
    pose=p1,
    coord_type=CoordinateType.CARTESIAN
  )
  robot.motion.MovJ(
    pose=p2,
    coord_type=CoordinateType.JOINT
  )  

  # Linear move
  robot.motion.MovL(
    pose=p1,
    coord_type=CoordinateType.CARTESIAN
  )  
  robot.motion.MovL(
    pose=p2,
    coord_type=CoordinateType.JOINT
  )

    # DO
  robot.io.DO(4, 1)  # Turn on DO4
  time.sleep(10)
  robot.io.DO(4, 0)  # Turn off DO4

  # Read input
  di_status = robot.io.DI(6) # Read DI
  print(di_status)