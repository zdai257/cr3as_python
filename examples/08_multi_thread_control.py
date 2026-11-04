# -*- coding: utf-8 -*-
import sys
import time
import threading
import queue
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot, CoordinateType, set_log_level, load_robot_config
set_log_level('ERROR') # ERROR-level logs only


class RobotController:
  def __init__(self, ip):
    self.ip = ip
    self.robot = None
    self.running = False
    
    self.status_queue = queue.Queue(maxsize=10)
    self.monitor_thread = None
    self.latest_status = None
    self.status_lock = threading.Lock()
    self.last_printed_state = None # Last printed state
    
    # Status stats
    self.status_combinations = {}
    self.abnormal_events = []
    self.motion_start_time = None
    self.total_motion_count = 0
  
  def _monitor_loop(self):
    """Monitor thread: consume status feedback"""
    print("[INFO] Monitor thread started...")
    while self.running:
      try:
        if not self.status_queue.empty():
          status = self.status_queue.get(timeout=0.1)
          with self.status_lock:
            self.latest_status = status
          self._process_status(status)
        time.sleep(0.005) # 5 ms sleep to reduce CPU
      except queue.Empty:
        continue
      except Exception as e:
        print(f"\n[ERR] Monitor-thread error: {str(e)}")
        break
    print("\n[INFO] Monitor thread stopped")
  
  def _process_status(self, status):
    """Process status and detect anomalies"""
    rs = status.running_status
    rm = status.robot_mode.value if hasattr(status.robot_mode, 'value') else status.robot_mode
    
    # Count status pairs
    key = (rs, rm)
    self.status_combinations[key] = self.status_combinations.get(key, 0) + 1
    
    # Anomaly: RUNNING mode but idle
    is_abnormal = (rm == 7 and rs == 0)
    if is_abnormal:
      self.abnormal_events.append({'time': time.time(), 'rs': rs, 'rm': rm})
    
    # Print status only when it changes
    self._print_status(status)
  
  def _print_status(self, status):
    """Print live status"""
    rs = status.running_status
    rm = status.robot_mode.value if hasattr(status.robot_mode, 'value') else status.robot_mode
    
    # Mode labels
    mode_desc = {
      1: "Init",
      2: "Manual",
      3: "Auto",
      4: "Remote",
      5: "Idle",
      6: "Drag",
      7: "Running",
      9: "Error",
      10: "pause",
      11: "Jog"
    }
    
    # Print only on change
    current_state = (rs, rm)
    if current_state != self.last_printed_state:
      self.last_printed_state = current_state
      status_text = "Moving" if rs != 0 else "Idle"
      status_color = "[RUN]" if rs != 0 else "[IDLE]"
      mode_text = mode_desc.get(rm, f"Unknown({rm})")
      pos = status.tool_vector_actual
      print(f"{status_color} RS:{status_text} | RM:{mode_text} | speed:{status.speed_scaling:.0f}% | X:{pos.x:.1f} Y:{pos.y:.1f} Z:{pos.z:.1f}")
  
  def _wait_for_motion_complete(self, timeout=30):
    """Wait for motion to finish"""
    print(f"[WAIT] Wait for motion to finish")
    start = time.time()
    
    # Wait for motion to start
    while time.time() - start < 5:
      with self.status_lock:
        if self.latest_status and self.latest_status.running_status != 0:
          break
      time.sleep(0.05)
    
    # Wait for motion to end
    while time.time() - start < timeout:
      with self.status_lock:
        if self.latest_status and self.latest_status.running_status == 0:
          # Read joints
          q = self.latest_status.q_actual if self.latest_status else [0]*6
          elapsed = time.time() - start
          print(f"[OK] Motion done: {elapsed:.2f}s | joints: J1={q[0]:.4f} J2={q[1]:.4f} J3={q[2]:.4f}")
          return True
      time.sleep(0.05)
    
    return False
  
  def execute_point_by_point(self):
    """Point-to-point test: RS vs RM accuracy"""
    points = [
      [200, -200, 300, 180, 0, -180],
      [200, -200, 400, 180, 0, -180],
      [300, -200, 400, 180, 0, -180],
      [300, -100, 400, 180, 0, -180],
      [200, -200, 300, 180, 0, -180],
    ]
    
    print(f"\nPoint-to-point test: {len(points)}points")
    
    # Store joints at RM=7 and RM=5
    point_data = {tuple(p[:3]): {'rm7': [], 'rm5': []} for p in points}
    
    for idx, point in enumerate(points, 1):
      print(f" [{idx}/{len(points)}] -> {point[:3]}")
      try:
        self.robot.motion.MovL(point, CoordinateType.CARTESIAN)
        self.total_motion_count += 1
        result = self._wait_for_motion_complete_with_joints(15)
        if result:
          rm7_joints, rm5_joints = result
          point_data[tuple(point[:3])]['rm7'].extend(rm7_joints)
          if rm5_joints:
            point_data[tuple(point[:3])]['rm5'].append(rm5_joints[-1])
        time.sleep(0.3)
      except Exception as e:
        print(f"  [ERR] {str(e)}")
        break
    
    # Compare accuracy
    print("\n" + "="*60)
    print("RS vs RM accuracy")
    print("="*60)
    print(f"{'pt':<15} {'J1 range at RM=7':<18} {'J1 range at RM=5':<18} {'note'}")
    print("-"*60)
    
    for pos, data in point_data.items():
      rm7 = data['rm7']
      rm5 = data['rm5']
      
      if len(rm7) >= 2:
        rm7_j1 = [j[0] for j in rm7]
        rm7_range = f"{min(rm7_j1):.4f}~{max(rm7_j1):.4f}"
        rm7_diff = max(rm7_j1) - min(rm7_j1)
      else:
        rm7_range = "n/a"
        rm7_diff = 0
      
      if len(rm5) >= 1:
        rm5_j1 = [j[0] for j in rm5]
        rm5_range = f"{min(rm5_j1):.4f}~{max(rm5_j1):.4f}"
        rm5_diff = max(rm5_j1) - min(rm5_j1) if len(rm5) > 1 else 0
      else:
        rm5_range = "n/a"
        rm5_diff = 0
      
      conclusion = "RM=5 more stable" if rm7_diff > rm5_diff else "similar"
      print(f"{str(pos):<15} {rm7_range:<18} {rm5_range:<18} {conclusion}")
    
    print("\nDone: {0}moves".format(self.total_motion_count))
  
  def _wait_for_motion_complete_with_joints(self, timeout=30):
    """Wait for motion; compare RS vs RM joint accuracy"""
    start = time.time()
    
    # Wait for motion to start
    while time.time() - start < 5:
      with self.status_lock:
        if self.latest_status and self.latest_status.running_status != 0:
          break
      time.sleep(0.05)
    
    # Collect joints in each mode
    rs0_rm7_joints = [] # RS=0, RM=7 status
    rm5_joints = []   # RM=5 status
    
    motion_ended = False
    while time.time() - start < timeout:
      with self.status_lock:
        if self.latest_status:
          rs = self.latest_status.running_status
          rm = self.latest_status.robot_mode
          q = self.latest_status.joint_state.q_actual
          
          # Record joints at RS=0, RM=7 (transient)
          if rs == 0 and rm == 7:
            rs0_rm7_joints.append(q.copy())
            motion_ended = True
          
          # Record joints at RM=5 (settled)
          if rm == 5 and rs == 0:
            rm5_joints.append(q.copy())
            elapsed = time.time() - start
            print(f"  [OK] {elapsed:.2f}s | J1={q[0]:.4f} J2={q[1]:.4f} J3={q[2]:.4f}")
            return (rs0_rm7_joints, rm5_joints)
      time.sleep(0.05)
    
    return (rs0_rm7_joints, rm5_joints)
  
  def execute_validation(self):
    """State-validation test"""
    print("\n" + "="*50)
    print("State-validation test")
    print("="*50)
    
    test_point = [200, -200, 300, 180, 0, -180]
    print(f"\nt    RS  RM  status      J1angle  stable")
    print("-"*60)
    
    state_records = []
    start_time = time.time()
    
    try:
      response = self.robot.motion.MovL(test_point, CoordinateType.CARTESIAN)
      self.total_motion_count += 1
      
      stable_count = 0
      while stable_count < 50 and (time.time() - start_time) < 20:
        if not self.status_queue.empty():
          status = self.status_queue.get(block=True, timeout=0.1)
          rs = status.running_status if hasattr(status, 'running_status') else 0
          rm = status.robot_mode if hasattr(status, 'robot_mode') else 0
          j1 = status.q_actual[0] if hasattr(status, 'q_actual') and status.q_actual else 0
          elapsed = time.time() - start_time
          
          state_desc = self._get_state_description(rs, rm)
          is_stable = (rs == 0 and rm == 5)
          
          state_records.append({
            'time': elapsed, 'rs': rs, 'rm': rm, 'j1': j1, 'stable': is_stable
          })
          
          print(f"{elapsed:6.2f}s {rs:<4} {rm:<4} {state_desc:<14} {j1:>8.3f}  {'[OK]' if is_stable else ''}")
          
          stable_count = stable_count + 1 if is_stable else 0
      
      # Analyze
      print("\nState durations:")
      state_durations = {}
      for i in range(len(state_records)-1):
        key = f"RS={state_records[i]['rs']},RM={state_records[i]['rm']}"
        dur = state_records[i+1]['time'] - state_records[i]['time']
        state_durations[key] = state_durations.get(key, 0) + dur
      
      for state, dur in sorted(state_durations.items(), key=lambda x: -x[1]):
        print(f" {state:<15}: {dur*1000:>6.0f} ms")
      
      # note
      print("\nnote:")
      print(" Joints are stable at RM=5")
      print(" RM=7 is transient; joints may still change")
      
    except Exception as e:
      print(f"[ERR] {str(e)}")
  
  def _get_state_description(self, rs, rm):
    """State description"""
    rs_desc = "Moving" if rs != 0 else "Idle"
    rm_desc = {
      1: "Init", 2: "Manual", 3: "Auto", 4: "Remote", 5: "Idle",
      6: "Drag", 7: "Running", 9: "Error", 10: "pause", 11: "Jog"
    }.get(rm, f"Unknown({rm})")
    return f"{rs_desc}/{rm_desc}"
  
  def execute_continuous(self):
    """Continuous-motion test"""
    point_a = [200, -200, 300, 180, 0, -180]
    point_b = [300, -100, 400, 180, 0, -180]
    
    print(f"\n{'='*50}")
    print(f"Continuous A ↔ B test")
    print(f"ptA: {point_a}")
    print(f"ptB: {point_b}")
    print(f"Ctrl+C to stop")
    print(f"{'='*50}")
    
    count = 0
    start_time = time.time()
    
    try:
      while self.running:
        target_point = point_a if count % 2 == 0 else point_b
        try:
          self.robot.motion.MovL(target_point, CoordinateType.CARTESIAN)
          self.total_motion_count += 1
          
          if count > 0 and count % 10 == 0:
            elapsed = time.time() - start_time
            rate = count / elapsed if elapsed > 0 else 0
            print(f"\r[MOVE] sent {count} cmds | rate: {rate:.1f} cmd/s", end="", flush=True)
          
          count += 1
          time.sleep(0.05) # 50msinterval
          
        except Exception as e:
          print(f"\n[ERR] send failed: {str(e)}")
          time.sleep(0.5)
          
    except KeyboardInterrupt:
      print("\n[STOP] Continuous motion stopped by user")
    
    elapsed = time.time() - start_time
    rate = count / elapsed if elapsed > 0 else 0
    
    print(f"\n{'='*50}")
    print(f"Continuous-motion test stopped")
    print(f"Commands sent: {count}")
    print(f"Elapsed: {elapsed:.2f}s")
    print(f"Average rate: {rate:.2f} cmd/s")
    print(f"{'='*50}")
  
  def _status_callback(self, status):
    """Status callback"""
    if self.running and not self.status_queue.full():
      try:
        self.status_queue.put(status, block=False)
      except queue.Full:
        # Drop newest if the queue is full
        pass
  
  def start(self):
    """Start controller"""
    try:
      self.robot = DobotRobot(self.ip, connect_timeout=5.0, receive_timeout=10.0)
      self.robot.Connect()
      self.robot.robot_control.RequestControl()
      self.robot.robot_control.ClearError()
      self.robot.StartFeedbackMonitor(callback=self._status_callback)
      self.running = True
      self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
      self.monitor_thread.start()
      print("[OK] Controller started")
      
    except Exception as e:
      print(f"[ERR] Start failed: {str(e)}")
      self.stop()
      raise
  
  def stop(self):
    """Stop controller"""
    self.running = False
    
    if self.monitor_thread:
      self.monitor_thread.join(timeout=2.0)
    
    if self.robot:
      self.robot.StopFeedbackMonitor()
      try:
        self.robot.robot_control.DisableRobot()
      except:
        pass
      try:
        self.robot.Disconnect()
      except:
        pass
    
    # Print summary
    self._print_summary()
    
    print("\n[OK] Controller stopped")
  
  def _print_summary(self):
    """Print test summary"""
    if self.abnormal_events or self.total_motion_count > 0:
      print(f"\n[Summary] moves: {self.total_motion_count}x | fault: {len(self.abnormal_events)}x")
    
    print("\n" + "="*50)


def main(enable_motion=False, motion_type='point_by_point'):
  """Main"""
  controller = RobotController(load_robot_config().ip)
  
  try:
    controller.start()
    time.sleep(1)
    
    # Main thread: send commands
    print("\n" + "-"*50)
    # Power on, enable, set speed
    controller.robot.robot_control.PowerOn()
    controller.robot.robot_control.EnableRobot(load=1.0)
    controller.robot.robot_control.SpeedFactor(30)
    
    # Motion tests
    if enable_motion:
      print("\nStart motion test")
      if motion_type == 'point_by_point':
        controller.execute_point_by_point()
      elif motion_type == 'continuous':
        controller.execute_continuous()
      elif motion_type == 'validation':
        controller.execute_validation()
    else:
      while True:
        time.sleep(0.1)
    
  except KeyboardInterrupt:
    pass
  except Exception as e:
    print(f"[ERR] {str(e)}")
  finally:
    controller.stop()


if __name__ == "__main__":
  ENABLE_MOTION = True
  MOTION_TYPE = 'point_by_point' # validation / point_by_point / continuous
  main(enable_motion=ENABLE_MOTION, motion_type=MOTION_TYPE)
