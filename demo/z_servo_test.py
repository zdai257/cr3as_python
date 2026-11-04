"""
servo test

demoServoPservo tracking，build a circlemovestrajectory
"""

import sys
import os

# Add parent dir so dobot_sdk imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import time
from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType


def parse_dobot_response(response_str: str) -> tuple:
  """parseDobotstring payload in the response，split the fields

  formatExample: 'part1,{1,2,3},part3'

  Args:
    response_str: string to parse

  Returns:
    tuple: (prefix, number list, suffix)，parsefailedreturnNone
  """
  first_comma_idx = response_str.find(',')
  if first_comma_idx == -1:
    return None

  prefix = response_str[:first_comma_idx]
  remaining = response_str[first_comma_idx + 1:]

  if not remaining.startswith('{'):
    return None

  brace_depth = 1
  i = 1
  while i < len(remaining) and brace_depth > 0:
    if remaining[i] == '{':
      brace_depth += 1
    elif remaining[i] == '}':
      brace_depth -= 1
    i += 1

  if brace_depth != 0:
    return None

  middle_with_braces = remaining[:i]
  suffix = remaining[i + 1:] if i < len(remaining) else ""

  numbers_str = middle_with_braces[1:-1]
  number_list = [float(x.strip()) for x in numbers_str.split(',')]

  return prefix, number_list, suffix


def generate_circular_trajectory(radius: float, num_points: int) -> list:
  """build a circlemovestrajectorypt

  Args:
    radius: circle radius(mm)
    num_points: trajectorypt

  Returns:
    list: trajectoryptlist， eachpoints [x, y, z, rx, ry, rz]
  """
  trajectory_points = []
  for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    trajectory_points.append([x, y, 0, 0, 0, 0])
  return trajectory_points

def generate_circle_smooth(radius=50, total_points=300):
  points = []
  for i in range(total_points):
    s = i / (total_points - 1)
    angle = 2 * math.pi * (0.5 - 0.5 * math.cos(math.pi * s))
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    points.append([x, y, 0, 0, 0, 0])
  return points

def execute_trajectory_at_frequency(robot, trajectory_points, frequency_hz: float):
  """run the trajectory at a fixed rateptsequence

  Args:
    robot: DobotRobotinstance
    trajectory_points: trajectoryptlist
    frequency_hz: rate Hz(Hz)
  """
  start_time = time.time()
  interval = 1.0 / frequency_hz

  for i, target_point in enumerate(trajectory_points):
    expected_time = start_time + (i + 1) * interval
    cycle_start = time.time()

    # use the newSDK ServoPAPI
    robot.motion.ServoP(target_point)

    cycle_duration = time.time() - cycle_start
    current_time = time.time()
    delay_needed = expected_time - current_time

    if delay_needed > 0:
      time.sleep(delay_needed)
    else:
      print(f"cycle {i + 1} timeout {-delay_needed:.3f} s")

def wait_for_robot_ready(robot, target_state=5.0, polling_interval=0.1):
  """Waitwait until the robot is readystatus

  Args:
    robot: DobotRobotinstance
    target_state: Targetstatusvalue
    polling_interval: poll interval(s)
  """
  while True:
    status_response = robot.robot_control.RobotMode()
    parsed = parse_dobot_response(status_response)

    if parsed is None:
      print("robotstatusparsefailed")
      time.sleep(polling_interval)
      continue

    _, state_data, _ = parsed

    if state_data and state_data[0] == target_state:
      print(f"robot is ready，status: {state_data}")
      break

    time.sleep(polling_interval)


def main():
  # Robot IP (from config)
  try:
    with DobotRobot.from_config() as robot:
      print("=" * 50)
      print("servo test - circular tracking")
      print("=" * 50)
      
      # Init
      robot.robot_control.RequestControl()
      robot.robot_control.ClearError()
      robot.robot_control.EnableRobot(load=1.0)
      
      # activateUser frame0
      robot.robot_control.User(0)
      
      # get current pose
      pose_response = robot.robot_control.GetPose()
      parsed_pose = parse_dobot_response(pose_response)

      if parsed_pose is None:
        print("Error: parse posefailed")
        return

      _, current_position, _ = parsed_pose
      print(f"current pose: {current_position}")

      # Set user frame 1（use the current pose as originpt）
      set_user_result = robot.robot_control.SetUser(1, current_position)
      print(f"Set user frameResult: {set_user_result}")

      # Switch to user frame 1
      robot.robot_control.User(1)

      # move to trajectory startpt（User framerelative pose in）
      start_point = [50, 0, 0, 0, 0, 0]
      print(f"\nmove to trajectory startpt: {start_point}")
      robot.motion.MovL(start_point, CoordinateType.CARTESIAN)
      time.sleep(3)

      # Waitrobot ready
      wait_for_robot_ready(robot)

      # build a circular path
      # trajectory_points = generate_circular_trajectory(radius=50, num_points=300)
      trajectory_points = generate_circle_smooth(radius=50, total_points=300)
      print(f"\ngenerate {len(trajectory_points)} pathpt")

      # stream the trajectory
      print("start trajectory streaming...")
      execute_trajectory_at_frequency(robot, trajectory_points, frequency_hz=33.0)

      print("\ntrajectory executionDone")

  except KeyboardInterrupt:
    print("\ninterrupted by user")
  except Exception as e:
    print(f"\nError: {e}")
    import traceback
    traceback.print_exc()


if __name__ == "__main__":
  main()
