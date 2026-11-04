"""
DobotDemo - Robot control helper
"""

import sys
import os

# Add parent dir so dobot_sdk imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_sdk import DobotRobot
import threading
from time import sleep
import re


class DobotDemo:
  def __init__(self, ip):
    self.ip = ip
    self.robot = None
    self.feedData = None
    self.__globalLockValue = threading.Lock()
    
    # Init feedback struct
    class item:
      def __init__(self):
        self.robotMode = -1
        self.robotCurrentCommandID = 0
        self.MessageSize = -1
        self.DigitalInputs = -1
        self.DigitalOutputs = -1
        self.robotCurrentCommandID = -1

    self.feedData = item()

  def start(self):
    """Start and enable the robot"""
    try:
      # Use the new SDK
      self.robot = DobotRobot(self.ip)
      self.robot.Connect()
      
      # Request TCP control mode
      self.robot.robot_control.RequestControl()
      
      # Clear alarms
      self.robot.robot_control.ClearError()
      
      # Enable robot
      response = self.robot.robot_control.EnableRobot()
      if "Failed" in response:
        print("Enable failed: is port 29999 busy?")
        return
      print("Enable OK")

      # Start feedback thread
      self.robot.StartFeedbackMonitor(callback=self._feed_callback)
      sleep(1)

      # Two target points
      point_a = [146.3759, -283.4321, 332.3956, 177.7879, -1.8540, 147.5821]
      point_b = [146.3759, -283.4321, 432.3956, 177.7879, -1.8540, 147.5821]

      # Point loop
      from dobot_sdk import CoordinateType
      while True:
        status = self.robot.GetStatus()
        if status:
          self.feedData.DigitalInputs = status.digital_inputs
          self.feedData.DigitalOutputs = status.digital_outputs
          self.feedData.robotMode = status.robot_mode.value
          self.feedData.robotCurrentCommandID = status.current_command_id
        
        print(f"DI: {self.feedData.DigitalInputs} 2DI: {bin(self.feedData.DigitalInputs)} --16: {hex(self.feedData.DigitalInputs)}")
        print(f"DO: {self.feedData.DigitalOutputs} 2DO: {bin(self.feedData.DigitalOutputs)} --16: {hex(self.feedData.DigitalOutputs)}")
        print(f"robomode {self.feedData.robotMode}")
        sleep(2)

    except Exception as e:
      print(f"Start failed: {e}")
      import traceback
      traceback.print_exc()

  def _feed_callback(self, status):
    """Feedback callback"""
    with self.__globalLockValue:
      self.feedData.robotMode = status.robot_mode.value
      self.feedData.DigitalInputs = status.digital_inputs
      self.feedData.DigitalOutputs = status.digital_outputs
      self.feedData.robotCurrentCommandID = status.current_command_id

  def RunPoint(self, point_list):
    """Move to a point (new SDK)"""
    from dobot_sdk import CoordinateType
    
    # Joint move
    response = self.robot.motion.MovJ(point_list, CoordinateType.CARTESIAN)
    print(f"MovJ: {response}")
    
    # Parse command ID
    currentCommandID = self.parseResultId(response)[1]
    print(f"Command ID: {currentCommandID}")
    
    # Wait for motion to finish
    while True:
      print(f"Current mode: {self.feedData.robotMode}")
      if self.feedData.robotMode == 5 and self.feedData.robotCurrentCommandID == currentCommandID:
        print("Motion finished")
        break
      sleep(0.1)

  def parseResultId(self, valueRecv):
    """Parse response"""
    if "Not Tcp" in valueRecv:
      print("Control Mode Is Not Tcp")
      return [1]
    return [int(num) for num in re.findall(r'-?\d+', valueRecv)] or [2]

  def __del__(self):
    """Destructor"""
    if self.robot:
      try:
        self.robot.StopFeedbackMonitor()
        self.robot.Disconnect()
      except Exception:
        pass
