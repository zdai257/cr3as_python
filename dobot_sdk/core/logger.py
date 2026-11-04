# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Logging module
Unified SDK logging (Windows/Ubuntu)
"""

import logging
import os
import sys
from logging.handlers import RotatingFileHandler
from typing import Optional
from datetime import datetime


class DobotLogger:
  """
  Dobot SDK logger
  
  Features:
    - Levels: DEBUG, INFO, WARNING, ERROR
    - Logs to console and file
    - Rotating files (5 x 10 MB)
    - Cross-platform (Windows/Ubuntu)
    - Structured log format
  """
  
  _instance = None
  
  def __new__(cls, *args, **kwargs):
    if cls._instance is None:
      cls._instance = super().__new__(cls)
    return cls._instance
  
  def __init__(self, log_level: str = "INFO", log_dir: Optional[str] = None):
    """
    Initialize the logger
    
    Args:
      log_level: Log level: "DEBUG", "INFO", "WARNING", "ERROR"
      log_dir: Log directory; default is SDK/logs
    """
    if hasattr(self, '_initialized') and self._initialized:
      return
    
    self._initialized = True
    
    # Resolve log directory
    if log_dir is None:
      # Default log dir: SDK/logs
      sdk_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
      self.log_dir = os.path.join(sdk_dir, 'logs')
    else:
      self.log_dir = log_dir
    
    # Ensure log directory exists
    os.makedirs(self.log_dir, exist_ok=True)
    
    # Log file name: dobot_sdk_YYYYMMDD.log
    today = datetime.now().strftime("%Y%m%d")
    self.log_file = os.path.join(self.log_dir, f"dobot_sdk_{today}.log")
    
    # Create logger
    self.logger = logging.getLogger("dobot_sdk")
    self.logger.setLevel(getattr(logging, log_level.upper()))
    self.logger.propagate = False
    
    # Drop existing handlers to avoid duplicates
    for handler in self.logger.handlers[:]:
      self.logger.removeHandler(handler)
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    
    # Rotating file handler
    file_handler = RotatingFileHandler(
      self.log_file,
      maxBytes=10 * 1024 * 1024, # 10MB
      backupCount=5,       # Keep 5 backups
      encoding='utf-8'
    )
    file_handler.setLevel(logging.DEBUG)
    
    # Log format
    log_format = logging.Formatter(
      '%(asctime)s - %(levelname)s - %(module)s.%(funcName)s:%(lineno)d - %(message)s'
    )
    console_handler.setFormatter(log_format)
    file_handler.setFormatter(log_format)
    
    # Add handlers
    self.logger.addHandler(console_handler)
    self.logger.addHandler(file_handler)
  
  def get_logger(self) -> logging.Logger:
    """Get the logger instance"""
    return self.logger
  
  @property
  def log_directory(self) -> str:
    """Get the log directory"""
    return self.log_dir


# Global log decorator
def log_api_call(func):
  """
  Decorator: log API calls
  
  Records:
    - Function name
    - Arguments
    - Return value
    - Elapsed time
    - Exception (if any)
  """
  import time
  
  def wrapper(*args, **kwargs):
    logger = logging.getLogger("dobot_sdk")
    start_time = time.time()
    
    # Caller info
    class_name = ""
    if len(args) > 0 and hasattr(args[0], '__class__'):
      class_name = args[0].__class__.__name__ + "."
    
    func_name = class_name + func.__name__
    
    # Format arguments
    args_str = []
    for i, arg in enumerate(args[1:], 1): # skip self
      args_str.append(f"arg{i}={arg!r}")
    for k, v in kwargs.items():
      args_str.append(f"{k}={v!r}")
    
    args_str = ", ".join(args_str)
    
    # Log the call
    logger.info(f"API call: {func_name}({args_str})")
    
    try:
      result = func(*args, **kwargs)
      elapsed = (time.time() - start_time) * 1000
      
      # Log the result (truncated)
      result_str = str(result)
      if len(result_str) > 500:
        result_str = result_str[:500] + "..."
      
      logger.debug(f"API return: {func_name} -> {result_str} (elapsed: {elapsed:.2f}ms)")
      
      return result
    
    except Exception as e:
      elapsed = (time.time() - start_time) * 1000
      logger.error(f"API error: {func_name} -> {type(e).__name__}: {str(e)} (elapsed: {elapsed:.2f}ms)", exc_info=True)
      raise
  
  return wrapper


def get_logger() -> logging.Logger:
  """Get the global logger"""
  return DobotLogger().get_logger()


def set_log_level(log_level: str):
  """
  Set the global log level
  
  Args:
    log_level: "DEBUG", "INFO", "WARNING", "ERROR"
  """
  logger = DobotLogger(log_level=log_level).get_logger()
  logger.setLevel(getattr(logging, log_level.upper()))
  for handler in logger.handlers:
    handler.setLevel(logging.DEBUG)


def get_log_directory() -> str:
  """Get the log directory"""
  return DobotLogger().log_directory
