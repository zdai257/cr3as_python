# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
main robot controller
Combines all modules into one control interface"""

import threading
import time
import logging
from typing import Optional, Callable
from ..core.connection import DobotConnection
from ..protocol.feedback import FeedbackParser
from ..models.status import RobotStatus
from ..config import load_robot_config
from .motion import Motion, CoordinateType
from .io import IO
from .communication import Communication
from .plugins import Plugins
from .robot_control import RobotControl
from .error_controller import ErrorController

logger = logging.getLogger("dobot_sdk")


class DobotRobot:
  """
  DobotMain robot controller
  Example:
    with DobotRobot.from_config() as robot:
      robot.robot_control.EnableRobot()
      pose = [400, 0, 300, 180, 0, 0]
      robot.motion.MovJ(pose, CoordinateType.CARTESIAN)
  """

  def __init__(self, ip: str = None,
         dashboard_port: int = None,
         feedback_port: int = None,
         feedback_port_30005: int = None,
         feedback_port_30006: int = None,
         connect_timeout: float = None,
         receive_timeout: float = None):
    """
    Initialize the robot object.

    Omitted connection settings are read from config/robot.yaml
    (default IP 192.168.5.1).

    Args:
      ip: Robot IP address
      dashboard_port: Dashboard port (default 29999)
      feedback_port: Feedback port (default 30004)
               - 30004: realtime feedback, every 8 ms
               - 30005: config feedback, every 200 ms
               - 30006: config feedback, every 1000 ms
      feedback_port_30005: Optional second feedback port (default None)
      feedback_port_30006: Optional third feedback port (default None)
      connect_timeout: Connect timeout (s), default 5
      receive_timeout: Receive timeout (s), default 10
    """
    cfg = load_robot_config()
    ip = cfg.ip if ip is None else ip
    dashboard_port = cfg.dashboard_port if dashboard_port is None else dashboard_port
    feedback_port = cfg.feedback_port if feedback_port is None else feedback_port
    connect_timeout = cfg.connect_timeout if connect_timeout is None else connect_timeout
    receive_timeout = cfg.receive_timeout if receive_timeout is None else receive_timeout
    extra_30005 = cfg.robot.get("feedback_port_30005")
    extra_30006 = cfg.robot.get("feedback_port_30006")
    if feedback_port_30005 is None:
      feedback_port_30005 = extra_30005
    if feedback_port_30006 is None:
      feedback_port_30006 = extra_30006
    self.ip = ip
    self._connect_timeout = connect_timeout
    self._receive_timeout = receive_timeout

    # Connection - Dashboard
    self._dashboard_conn = DobotConnection(ip, dashboard_port)
    self._dashboard_conn.set_timeout(connect_timeout, receive_timeout)
    
    # Connection - Feedback port
    self._feedback_conn = DobotConnection(ip, feedback_port)
    self._feedback_conn.set_timeout(connect_timeout, receive_timeout)
    
    # Optional extra feedback connections
    self._feedback_conn_30005 = None
    if feedback_port_30005:
      self._feedback_conn_30005 = DobotConnection(ip, feedback_port_30005)
      self._feedback_conn_30005.set_timeout(connect_timeout, receive_timeout)
    
    self._feedback_conn_30006 = None
    if feedback_port_30006:
      self._feedback_conn_30006 = DobotConnection(ip, feedback_port_30006)
      self._feedback_conn_30006.set_timeout(connect_timeout, receive_timeout)

    # Protocol parser (Feedback only)
    self._feedback_parser = FeedbackParser()

    # 5 modules
    self.motion = Motion(self._dashboard_conn)          # Motion module
    self.io = IO(self._dashboard_conn)              # IO module
    self.communication = Communication(self._dashboard_conn)    # Communication module
    self.plugins = Plugins(self._dashboard_conn)         # Plugin module
    self.robot_control = RobotControl(self._dashboard_conn)   # Robot-control module
    self.error = ErrorController(ip)               # Error-control module

    # Status monitor
    self._latest_status: Optional[RobotStatus] = None
    self._feedback_thread: Optional[threading.Thread] = None
    self._feedback_running = False
    self._callback: Optional[Callable] = None
    
    # Connection-status callback
    self._connection_callback: Optional[Callable[[bool], None]] = None

  def SetTimeout(self, connect_timeout: float = None, receive_timeout: float = None):
    """
    Set timeout
    
    Args:
      connect_timeout: Connect timeout (s)
      receive_timeout: Receive timeout (s)
    """
    self._dashboard_conn.set_timeout(connect_timeout, receive_timeout)
    self._feedback_conn.set_timeout(connect_timeout, receive_timeout)
    if self._feedback_conn_30005:
      self._feedback_conn_30005.set_timeout(connect_timeout, receive_timeout)
    if self._feedback_conn_30006:
      self._feedback_conn_30006.set_timeout(connect_timeout, receive_timeout)
    
    if connect_timeout is not None:
      self._connect_timeout = connect_timeout
    if receive_timeout is not None:
      self._receive_timeout = receive_timeout

  def EnableAutoReconnect(self, enable: bool = True, callback: Callable[[bool], None] = None):
    """
    Enable or disable auto-reconnect
    
    Args:
      enable: Whether to enable auto-reconnect
      callback: Connection-status callback; receives True/False
           True: connected, False: disconnected
    """
    self._connection_callback = callback
    
    def connection_status_handler(is_connected: bool):
      logger.info(f"Robot connection: {'connected' if is_connected else 'disconnected'}")
      if self._connection_callback:
        try:
          self._connection_callback(is_connected)
        except Exception as e:
          logger.error(f"Connection-status callback failed: {str(e)}")
    
    self._dashboard_conn.enable_auto_reconnect(enable, connection_status_handler)
    self._feedback_conn.enable_auto_reconnect(enable, connection_status_handler)
    if self._feedback_conn_30005:
      self._feedback_conn_30005.enable_auto_reconnect(enable, connection_status_handler)
    if self._feedback_conn_30006:
      self._feedback_conn_30006.enable_auto_reconnect(enable, connection_status_handler)
    
    logger.info(f"Auto-reconnect {'enabled' if enable else 'disabled'}")

  @classmethod
  def from_config(cls, config=None, **overrides):
    """Build a robot using config/robot.yaml (optional field overrides)."""
    cfg = config or load_robot_config()
    params = {
      "ip": cfg.ip,
      "dashboard_port": cfg.dashboard_port,
      "feedback_port": cfg.feedback_port,
      "feedback_port_30005": cfg.robot.get("feedback_port_30005"),
      "feedback_port_30006": cfg.robot.get("feedback_port_30006"),
      "connect_timeout": cfg.connect_timeout,
      "receive_timeout": cfg.receive_timeout,
    }
    params.update(overrides)
    return cls(**params)

  def Connect(self, timeout: float = None):
    """
    Connect to the robot

    Args:
      timeout: Connect timeout (s); defaults to the value set at init
    """
    actual_timeout = timeout if timeout is not None else self._connect_timeout
    
    logger.info(f"Connecting to robot {self.ip}...")
    self._dashboard_conn.connect(actual_timeout)
    self._feedback_conn.connect(actual_timeout)
    
    # connect optional feedback ports
    if self._feedback_conn_30005:
      self._feedback_conn_30005.connect(actual_timeout)
    if self._feedback_conn_30006:
      self._feedback_conn_30006.connect(actual_timeout)
      
    logger.info(f"Robot connected: {self.ip}")
  
  @property
  def IsConnected(self) -> bool:
    """
    Check robot connection status
    
    Returns:
      bool: True if connected, False otherwise
    """
    return (self._dashboard_conn.is_connected and 
        self._feedback_conn.is_connected and
        (not self._feedback_conn_30005 or self._feedback_conn_30005.is_connected) and
        (not self._feedback_conn_30006 or self._feedback_conn_30006.is_connected))

  def Disconnect(self):
    """Disconnect"""
    self.StopFeedbackMonitor()
    self._dashboard_conn.disconnect()
    self._feedback_conn.disconnect()
    
    # disconnect optional feedback ports
    if self._feedback_conn_30005:
      self._feedback_conn_30005.disconnect()
    if self._feedback_conn_30006:
      self._feedback_conn_30006.disconnect()
      
    logger.info(f"Robot disconnected: {self.ip}")

  def GetStatus(self) -> Optional[RobotStatus]:
    """
    Get the latest robot status
    Returns:
      RobotStatus: RobotStatus, or None if monitoring is not started
    """
    return self._latest_status

  def StartFeedbackMonitor(self, callback: Callable = None):
    """
    Start the status-feedback monitor thread
    Args:
      callback: Optional callback receiving RobotStatus
    """
    if self._feedback_running:
      return

    self._callback = callback
    self._feedback_running = True

    self._feedback_thread = threading.Thread(
      target=self._feedback_loop,
      daemon=True
    )
    self._feedback_thread.start()
    logger.info("Status monitor started")

  def StopFeedbackMonitor(self):
    """Stop the status-feedback monitor"""
    self._feedback_running = False
    if self._feedback_thread:
      self._feedback_thread.join(timeout=2.0)
      self._feedback_thread = None
    logger.info("Status monitor stopped")

  def _feedback_loop(self):
    """Feedback receive loop"""
    while self._feedback_running:
      try:
        # receive raw bytes
        raw = self._feedback_conn.receive_bytes(144000)

        # handle coalesced packets
        if len(raw) > 1440:
          raw = self._feedback_conn.receive_bytes(144000)

        if len(raw) < 1440:
          time.sleep(0.01)
          continue

        # slice1440bytes
        packet = raw[:1440]

        # parse
        status = self._feedback_parser.parse(packet)
        if status:
          self._latest_status = status

          # invoke callback
          if self._callback:
            try:
              self._callback(status)
            except Exception as e:
              logger.error(f"Callback error: {e}", exc_info=True)

      except Exception as e:
        logger.error(f"Feedback error: {e}", exc_info=True)
        time.sleep(0.1)

  def __enter__(self):
    """Context-manager enter"""
    self.Connect()
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    """Context-manager exit"""
    self.Disconnect()
    return False

  def GetError(self, language: str = "zh_cn") -> dict:
    """
    Get robot alarms via HTTP    
    Args:
      language: Language code
           "zh_cn" - Simplified Chinese           "zh_hant" - Traditional Chinese 
           "en" - English
           "ja" - Japanese
           "de" - German
           "vi" - Vietnamese           "es" - Spanish
           "fr" - French
           "ko" - Korean
           "ru" - Russian
    
    Returns:
      dict: Alarm dict, format:
      {
        "errMsg": [
          {
            "id": xxx,
            "level": xxx,
            "description": "xxx",
            "solution": "xxx",
            "mode": "xxx",
            "date": "xxxx",
            "time": "xxxx"
          }
        ]
      }
      If no alarm, returns{"errMsg": []}
    """
    return self.error.GetError(language)

  def GetErrorFormatted(self, language: str = "zh_cn") -> str:
    """
    Get formatted robot alarms via HTTP    
    Args:
      language: Language
    
    Returns:
      str: Formatted alarm string    """
    return self.error.GetErrorFormatted(language)

  def __del__(self):
    """Destructor"""
    self.Disconnect()

  # Backward-compatible aliases (snake_case -> PascalCase
  set_timeout = SetTimeout
  enable_auto_reconnect = EnableAutoReconnect
  connect = Connect
  disconnect = Disconnect
  get_status = GetStatus
  start_feedback_monitor = StartFeedbackMonitor
  stop_feedback_monitor = StopFeedbackMonitor
  get_error = GetError
  get_error_formatted = GetErrorFormatted
  @property
  def is_connected(self):
    return self.IsConnected
