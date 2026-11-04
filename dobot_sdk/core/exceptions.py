# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Custom exceptions
"""


class DobotError(Exception):
  """Base Dobot SDK exception"""
  pass


class ConnectionError(DobotError):
  """Connection error"""
  def __init__(self, message="Connection failed", ip=None, port=None):
    self.ip = ip
    self.port = port
    super().__init__(f"{message} (IP: {ip}, Port: {port})")


class ProtocolError(DobotError):
  """Protocol error"""
  def __init__(self, message="Protocol error", command=None):
    self.command = command
    super().__init__(f"{message} (Command: {command})")


class RobotError(DobotError):
  """Robot error"""
  def __init__(self, message="Robot error", error_code=None):
    self.error_code = error_code
    super().__init__(f"{message} (Code: {error_code})")


class TimeoutError(DobotError):
  """Timeout error"""
  pass
