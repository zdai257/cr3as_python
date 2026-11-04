# -*- coding: utf-8 -*-
"""
Example 7: full status monitor

Print every field parsed from Feedback
"""

import sys
import time
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot, set_log_level, load_robot_config


def print_status(status):
  """
  Print full robot status
  
  Args:
    status: RobotStatus object
  """
  # Watch RunningStatus
  running_status = status.running_status
  
  # RunningStatus (protocol)
  # 0: idle
  # 1028: Moving
  # Other values: see protocol
  
  status_text = "Unknown"
  status_icon = ""
  if running_status == 0:
    status_text = "Idle"
    status_icon = "[OK]"
  elif running_status == 1028:
    status_text = "Moving"
    status_icon = "[RUN]"
  else:
    status_text = f"code: {running_status}"
    status_icon = "[WARN]"
  
  print(f"\n{'='*60}")
  print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
  print(f"{'='*60}")
  
  # Runningstatus
  print(f"\n[Run status]")
  print(f" RunningStatus: {status_icon} {status_text}")
  print(f" Robot mode: {status.robot_mode.name}")
  print(f" Enable status: {'enabled' if status.enable_status else 'disabled'}")
  print(f" Brake status: {'released' if status.brake_status else 'engaged'}")
  print(f" Error status: {'error' if status.error_status else 'Normal'}")
  print(f" speedscale: {status.speed_scaling:.1f}%")
  print(f" Drag: {status.drag_status} (0=none, 1=joint drag, 2=force drag)")
  print(f" Robot model id: {status.robot_type}")
  
  # Cartesian
  print(f"\n[Cartesian]")
  print(f" X: {status.tool_vector_actual.x:.2f} mm")
  print(f" Y: {status.tool_vector_actual.y:.2f} mm")
  print(f" Z: {status.tool_vector_actual.z:.2f} mm")
  print(f" Rx: {status.tool_vector_actual.rx:.2f} °")
  print(f" Ry: {status.tool_vector_actual.ry:.2f} °")
  print(f" Rz: {status.tool_vector_actual.rz:.2f} °")
  
  # TCPspeed
  print(f"\n[TCP velocity]")
  print(f" Vx: {status.tcp_speed_actual.x:.2f} mm/s")
  print(f" Vy: {status.tcp_speed_actual.y:.2f} mm/s")
  print(f" Vz: {status.tcp_speed_actual.z:.2f} mm/s")
  
  # Joint angles
  print(f"\n[Joints]")
  joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
  for i, (name, angle) in enumerate(zip(joint_names, status.joint_state.q_actual)):
    print(f" {name}: {angle:.2f} °")
  
  # Joint vel
  print(f"\n[Joint velocity]")
  for i, (name, speed) in enumerate(zip(joint_names, status.joint_state.qd_actual)):
    print(f" {name}: {speed:.2f} °/s")
  
  # Electrical
  print(f"\n[Electrical]")
  print(f" Voltage: {status.voltage:.2f} V")
  print(f" Current: {status.current:.2f} A")
  print(f" Payload: {status.load:.2f} kg")
  print(f" Payload CoM: ({status.load_center_x:.1f}, {status.load_center_y:.1f}, {status.load_center_z:.1f}) mm")
  
  # Motor temperature
  print(f"\n[Motor temp]")
  for i, (name, temp) in enumerate(zip(joint_names, status.joint_state.temperatures)):
    print(f" {name}: {temp:.1f} °C")
  
  # IO status
  print(f"\n[IO]")
  print(f" DI: {bin(status.digital_inputs)}")
  print(f" DO: {bin(status.digital_outputs)}")
  print(f" Safety IO in: {status.safety_io_in}")
  print(f" Safety IO out: {status.safety_io_out}")
  
  # Velocity / accel scales
  print(f"\n[Vel/acc scales]")
  print(f" Joint vel: {status.velocity_ratio}%")
  print(f" Joint acc: {status.acceleration_ratio}%")
  print(f" Cartesian linear vel: {status.xyz_velocity_ratio}%")
  print(f" Cartesian angular vel: {status.r_velocity_ratio}%")
  print(f" Cartesian linear acc: {status.xyz_acceleration_ratio}%")
  print(f" Cartesian angular acc: {status.r_acceleration_ratio}%")
  
  # Frames
  print(f"\n[Frames]")
  print(f" User frame: {status.user_coordinate}")
  print(f" Tool frame: {status.tool_coordinate}")
  
  # Quaternion
  print(f"\n[Quaternion]")
  print(f" Target: [{status.target_quaternion.qw:.4f}, {status.target_quaternion.qx:.4f}, {status.target_quaternion.qy:.4f}, {status.target_quaternion.qz:.4f}]")
  print(f" Actual: [{status.actual_quaternion.qw:.4f}, {status.actual_quaternion.qx:.4f}, {status.actual_quaternion.qy:.4f}, {status.actual_quaternion.qz:.4f}]")
  
  # Safety status
  print(f"\n[Safety]")
  print(f" Safety status: {status.get_safety_state_desc()}")
  print(f" Collision status: {status.collision_state}")
  print(f" Forearm approach: {status.arm_approach_state}")
  print(f" J4 approach: {status.j4_approach_state}")
  print(f" J5 approach: {status.j5_approach_state}")
  print(f" J6 approach: {status.j6_approach_state}")
  
  # Six-axis FT sensor
  print(f"\n[FT sensor]")
  force_status = {0: 'offline', 1: 'online', 2: 'fault'}.get(status.six_force_online, 'Unknown')
  print(f" Online: {force_status}")
  if status.six_force_online == 1:
    print(f" Raw: {[f'{v:.2f}' for v in status.six_force_value]}")
  
  # Tool-button signals
  print(f"\n[Tool buttons]")
  print(f" Drag btn: {status.drag_button_signal}")
  print(f" Enable btn: {status.enable_button_signal}")
  print(f" Record btn: {status.record_button_signal}")
  print(f" Playback btn: {status.reappear_button_signal}")
  print(f" Gripper btn: {status.jaw_button_signal}")
  
  # command info
  print(f"\n[Command]")
  print(f" Current command ID: {status.current_command_id}")
  print(f" Uptime: {status.run_time // 1000} s")
  print(f" Queue running: {status.run_queued_cmd}")
  print(f" Queue paused: {status.pause_cmd_flag}")
  
  # Mode
  print(f"\n[Mode]")
  print(f" Manual/auto: {status.auto_manual_mode}")
  print(f" USB export: {status.export_status}")
  
  # Vibration
  print(f"\n[Vibration]")
  print(f" Z vibration displacement: {status.vibration_dis_z:.4f} mm")
  
  print(f"\n{'='*60}")
  print("Ctrl+C to stop")


def main():
  # Optional log level
  set_log_level("INFO")
  
  cfg = load_robot_config()
  print(f"Connecting to robot {cfg.ip}...")
  robot = DobotRobot.from_config(connect_timeout=10.0, receive_timeout=15.0)
  
  try:
    # Connect
    robot.Connect()
    print("Connected!")
    
    # Request control
    robot.robot_control.RequestControl()
    
    # Clear errors
    robot.robot_control.ClearError()
    
    # Start feedback monitor
    robot.StartFeedbackMonitor(callback=print_status)
    
    print("\nListening for robot status...")
    print("Ctrl+C to stop\n")
    
    # Listen loop
    while True:
      time.sleep(1)
      
  except KeyboardInterrupt:
    print("\n\nStopped by user")
  except Exception as e:
    print(f"\nError: {e}")
    import traceback
    traceback.print_exc()
  finally:
    # Stop monitor and disconnect
    robot.StopFeedbackMonitor()
    robot.Disconnect()
    print("Disconnected")


if __name__ == "__main__":
  main()