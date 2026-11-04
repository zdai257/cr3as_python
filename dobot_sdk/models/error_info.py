# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Alarm data models
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import List


@dataclass
class ErrorInfo:
  """
  One alarm record
  
  Parsed from GetError
  """
  id: int
  level: int
  description: str
  solution: str
  mode: str
  date: str
  time: str
  
  @property
  def timestamp_str(self) -> str:
    """Full timestamp string"""
    return f"{self.date} {self.time}"
  
  def __str__(self) -> str:
    return (
      f"Error ID: {self.id}\n"
      f"Level: {self.level}\n"
      f"Description: {self.description}\n"
      f"Solution: {self.solution}\n"
      f"Time: {self.timestamp_str}"
    )


@dataclass
class ErrorReport:
  """
  Alarm report
  
  All current alarms
  """
  errors: List[ErrorInfo]
  timestamp: datetime = field(default_factory=datetime.now)
  
  @property
  def has_errors(self) -> bool:
    """Whether any alarm is present"""
    return len(self.errors) > 0
  
  @property
  def error_count(self) -> int:
    """Alarm count"""
    return len(self.errors)
  
  def get_critical_errors(self) -> List[ErrorInfo]:
    """Alarms with level >= 5"""
    return [e for e in self.errors if e.level >= 5]
  
  def __str__(self) -> str:
    if not self.has_errors:
      return "✅ Robot OK, no alarms"
    
    result = f"⚠️ Found {self.error_count} alarms\n"
    result += "=" * 50 + "\n"
    
    for i, error in enumerate(self.errors, 1):
      result += f"Alarm {i}:\n"
      result += f" ID: {error.id}\n"
      result += f" Level: {error.level}\n"
      result += f" Description: {error.description}\n"
      result += f" Solution: {error.solution}\n"
      result += f" Time: {error.timestamp_str}\n"
      result += "-" * 30 + "\n"
    
    return result
