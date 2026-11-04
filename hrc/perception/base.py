"""Perception front-end. Swap OraclePerception for RGB-D / LiDAR later."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from ..world import Observation, WorldState


class PerceptionFrontend(ABC):
  name: str = "base"

  @abstractmethod
  def observe(
    self,
    joints=None,
    pose=None,
    timestamp: float = 0.0,
    extra: Optional[dict] = None,
  ) -> Observation:
    """Return a world snapshot plus optional sensor buffers."""

  def close(self) -> None:
    return None
