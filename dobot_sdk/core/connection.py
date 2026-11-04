# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
TCP connection manager

TCP connect with auto-reconnect and status callbacks
"""

import socket
import threading
import logging
import time
from typing import Optional, Callable
from .exceptions import ConnectionError

logger = logging.getLogger("dobot_sdk")


class DobotConnection:
  """
  TCP connection manager
  
  Maintains the robot TCP socket:
  - Receive timeout
  - Auto-reconnect
  - Exponential backoff
  - Connection-status callback
  """
  
  def __init__(self, ip: str, port: int, buffer_size: int = 144000):
    """
    Initialize the connection
    
    Args:
      ip: Robot IP address
      port: Port (29999 or 30004)
      buffer_size: Receive buffer size
    """
    self.ip = ip
    self.port = port
    self.buffer_size = buffer_size
    self._socket: Optional[socket.socket] = None
    self._lock = threading.Lock()
    self._connected = False
    
    # Timeouts
    self._connect_timeout = 5.0   # Connect timeout (s)
    self._receive_timeout = 10.0   # Receive timeout (s)
    
    # Auto-reconnect settings
    self._auto_reconnect = False   # Whether to enable auto-reconnect
    self._reconnect_running = False # Reconnect thread running
    self._reconnect_thread = None  # Reconnect thread
    self._reconnect_callback = None # Connection-status callback
    
    # Backoff parameters
    self._min_reconnect_delay = 1  # Min reconnect delay (s)
    self._max_reconnect_delay = 30  # Max reconnect delay (s)
    self._reconnect_attempts = 0   # Reconnect attempt count
    
    # Validate port
    if port not in [29999, 30004, 30005, 30006]:
      raise ValueError(f"Invalid port: {port}. Must be 29999, 30004, 30005, or 30006")
  
  def set_timeout(self, connect_timeout: float = None, receive_timeout: float = None):
    """
    Set timeout
    
    Args:
      connect_timeout: Connect timeout (s), default 5
      receive_timeout: Receive timeout (s), default 10
    """
    if connect_timeout is not None:
      self._connect_timeout = connect_timeout
    if receive_timeout is not None:
      self._receive_timeout = receive_timeout
  
  def enable_auto_reconnect(self, enable: bool = True, callback: Callable[[bool], None] = None):
    """
    enable/disableAutoreconnect
    
    Args:
      enable: Whether to enable auto-reconnect
      callback: Connection-status callback; receives True/False
    """
    self._auto_reconnect = enable
    self._reconnect_callback = callback
    
    if enable and not self._reconnect_running and not self._connected:
      self._start_reconnect_loop()
  
  def _start_reconnect_loop(self):
    """start the reconnect thread"""
    if self._reconnect_running:
      return
    
    self._reconnect_running = True
    self._reconnect_thread = threading.Thread(
      target=self._reconnect_loop,
      daemon=True
    )
    self._reconnect_thread.start()
    logger.info(f"Autoreconnect started: {self.ip}:{self.port}")
  
  def _stop_reconnect_loop(self):
    """stop the reconnect thread"""
    self._reconnect_running = False
    if self._reconnect_thread:
      self._reconnect_thread.join(timeout=2.0)
      self._reconnect_thread = None
    self._reconnect_attempts = 0
    logger.info(f"Autoreconnect stopped: {self.ip}:{self.port}")
  
  def _reconnect_loop(self):
    """reconnect loop，exponential backoff"""
    while self._reconnect_running:
      try:
        # compute backoff delay
        delay = min(
          self._min_reconnect_delay * (2 ** self._reconnect_attempts),
          self._max_reconnect_delay
        )
        
        logger.info(f"Wait {delay:.1f} sthen retry reconnect: {self.ip}:{self.port}")
        time.sleep(delay)
        
        if not self._reconnect_running:
          break
        
        # try to reconnect
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(self._connect_timeout)
        self._socket.connect((self.ip, self.port))
        
        # Set recv buffer and timeout
        self._socket.setsockopt(
          socket.SOL_SOCKET, 
          socket.SO_RCVBUF, 
          self.buffer_size
        )
        self._socket.settimeout(self._receive_timeout)
        
        self._connected = True
        self._reconnect_attempts = 0 # Reset reconnect counter
        
        logger.info(f"TCP reconnect succeeded: {self.ip}:{self.port}")
        
        # Invoke connected callback
        if self._reconnect_callback:
          try:
            self._reconnect_callback(True)
          except Exception as e:
            logger.error(f"Connected callback failed: {str(e)}")
        
        # Reconnect succeeded; stop loop
        self._stop_reconnect_loop()
        
      except socket.timeout:
        self._reconnect_attempts += 1
        logger.warning(f"Reconnect timeout ({self._reconnect_attempts}x): {self.ip}:{self.port}")
        
      except socket.error as e:
        self._reconnect_attempts += 1
        logger.warning(f"Reconnect failed ({self._reconnect_attempts}x): {self.ip}:{self.port} - {str(e)}")
        
      except Exception as e:
        self._reconnect_attempts += 1
        logger.error(f"Reconnect error ({self._reconnect_attempts}x): {self.ip}:{self.port} - {str(e)}")
  
  def connect(self, timeout: float = None):
    """
    Open TCP connection
    
    Args:
      timeout: Connect timeout (s); defaults to the value set at init
      
    Raises:
      ConnectionError: Raised on connect failure
    """
    actual_timeout = timeout if timeout is not None else self._connect_timeout
    
    try:
      self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self._socket.settimeout(actual_timeout)
      self._socket.connect((self.ip, self.port))
      
      # Set recv buffer and timeout
      self._socket.setsockopt(
        socket.SOL_SOCKET, 
        socket.SO_RCVBUF, 
        self.buffer_size
      )
      self._socket.settimeout(self._receive_timeout)
      
      self._connected = True
      self._reconnect_attempts = 0 # Reset reconnect counter
      
      logger.info(f"TCP connected: {self.ip}:{self.port}")
      
      # Invoke connected callback
      if self._reconnect_callback:
        try:
          self._reconnect_callback(True)
        except Exception as e:
          logger.error(f"Connected callback failed: {str(e)}")
      
    except socket.timeout:
      logger.error(f"TCP connect timeout: {self.ip}:{self.port}")
      self._trigger_disconnect_callback()
      raise ConnectionError("Connection timed out", ip=self.ip, port=self.port)
    except socket.error as e:
      logger.error(f"TCP connect failed: {self.ip}:{self.port} - {str(e)}")
      self._trigger_disconnect_callback()
      raise ConnectionError(f"Connection failed: {str(e)}", ip=self.ip, port=self.port)
  
  def _trigger_disconnect_callback(self, force: bool = False):
    """
    Fire disconnect callback
    
    Args:
      force: force the callback（skip the connection checkstatus）
    """
    if self._reconnect_callback and (force or self._connected):
      try:
        self._reconnect_callback(False)
      except Exception as e:
        logger.error(f"disconnect callbackfailed: {str(e)}")
  
  def disconnect(self):
    """close the connection"""
    # stopAutoreconnect
    self._stop_reconnect_loop()
    
    if self._socket:
      try:
        self._socket.shutdown(socket.SHUT_RDWR)
        self._socket.close()
        logger.info(f"TCPconnectdisconnected: {self.ip}:{self.port}")
      except Exception as e:
        logger.warning(f"TCPDisconnectwhenError: {str(e)}")
      finally:
        self._socket = None
        self._connected = False
        # force the disconnect callback（user-initiated disconnect）
        self._trigger_disconnect_callback(force=True)
  
  def send_text(self, text: str):
    """
    send a text command（Dashboard ）
    
    Args:
      text: command string to send
    """
    if not self._connected or not self._socket:
      raise ConnectionError("not connected to the robot")
    
    try:
      # encode and send，append newline
      command = text if text.endswith('\n') else text + '\n'
      self._socket.send(command.encode('utf-8'))
      logger.debug(f"send command: {text.strip()}")
    except Exception as e:
      self._connected = False
      logger.error(f"send commandfailed: {text.strip()} - {str(e)}")
      self._trigger_disconnect_callback()
      
      # if enabledAutoreconnect，start reconnect loop
      if self._auto_reconnect and not self._reconnect_running:
        self._start_reconnect_loop()
      
      raise ConnectionError(f"send failed {str(e)}")
  
  def receive_text(self, buffer_size: int = 1024) -> str:
    """
    receive textresponse（Dashboard ）
    
    Args:
      buffer_size: Receive buffer size
      
    Returns:
      str: received string
    """
    if not self._connected or not self._socket:
      raise ConnectionError("not connected to the robot")
    
    try:
      data = self._socket.recv(buffer_size)
      if not data:
        raise ConnectionError("connection closed")
      
      response = data.decode('utf-8').strip()
      logger.debug(f"receiveresponse: {response}")
      return response
    except socket.timeout:
      self._connected = False
      logger.error(f"receiveresponsetimeout: {self.ip}:{self.port}")
      self._trigger_disconnect_callback()
      
      # if enabledAutoreconnect，start reconnect loop
      if self._auto_reconnect and not self._reconnect_running:
        self._start_reconnect_loop()
      
      raise ConnectionError("receive timeout")
    except Exception as e:
      self._connected = False
      logger.error(f"receiveresponsefailed: {str(e)}")
      self._trigger_disconnect_callback()
      
      # if enabledAutoreconnect，start reconnect loop
      if self._auto_reconnect and not self._reconnect_running:
        self._start_reconnect_loop()
      
      raise ConnectionError(f"receivefailed: {str(e)}")
  
  def receive_bytes(self, buffer_size: int = 144000) -> bytes:
    """
    receive raw bytes（Feedback ）
    
    Args:
      buffer_size: Receive buffer size
      
    Returns:
      bytes: raw bytes received
    """
    if not self._connected or not self._socket:
      raise ConnectionError("not connected to the robot")
    
    try:
      data = self._socket.recv(buffer_size)
      if not data:
        raise ConnectionError("connection closed")
      
      logger.debug(f"receive bytes: {len(data)} bytes")
      return data
    except socket.timeout:
      self._connected = False
      logger.error(f"byte receive timed out: {self.ip}:{self.port}")
      self._trigger_disconnect_callback()
      
      # if enabledAutoreconnect，start reconnect loop
      if self._auto_reconnect and not self._reconnect_running:
        self._start_reconnect_loop()
      
      raise ConnectionError("receive timeout")
    except Exception as e:
      self._connected = False
      logger.error(f"receive bytesfailed: {str(e)}")
      self._trigger_disconnect_callback()
      
      # if enabledAutoreconnect，start reconnect loop
      if self._auto_reconnect and not self._reconnect_running:
        self._start_reconnect_loop()
      
      raise ConnectionError(f"receivefailed: {str(e)}")
  
  def send_receive_text(self, text: str, recv_size: int = 1024) -> str:
    """
    send and receiveresponse（thread-safe）
    
    Args:
      text: sent string
      recv_size: Receive buffer size
      
    Returns:
      str: receivedresponse
    """
    with self._lock:
      self.send_text(text)
      return self.receive_text(recv_size)
  
  @property
  def is_connected(self) -> bool:
    """check connectionstatus"""
    return self._connected
  
  @property
  def reconnect_enabled(self) -> bool:
    """check whether enabledAutoreconnect"""
    return self._auto_reconnect
  
  def __enter__(self):
    """Context-manager enter"""
    self.connect()
    return self
  
  def __exit__(self, exc_type, exc_val, exc_tb):
    """Context-manager exit"""
    self.disconnect()
    return False
  
  def __del__(self):
    """Destructor"""
    self.disconnect()
