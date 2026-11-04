# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Dobot SDK V4 - Dobot robot Python SDK

Supported models: CRA, E6, CRAF, NovaLite and other V4-series robots
"""

from .version import __version__
from .api.robot import DobotRobot
from .api.motion import CoordinateType
from .core.logger import get_logger, set_log_level, get_log_directory
from .config import load_robot_config, RobotConfig, DEFAULT_IP

__all__ = [
  "DobotRobot",
  "CoordinateType",
  "__version__",
  "get_logger",
  "set_log_level",
  "get_log_directory",
  "load_robot_config",
  "RobotConfig",
  "DEFAULT_IP",
]
