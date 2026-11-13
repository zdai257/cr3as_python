"""Episode / dataset summaries for the replay viewer and LeRobot cards."""

from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np

from ..perception.skins import SKIN_PARTS
from .dataset import Episode


def _minmax(arr: np.ndarray) -> Dict[str, List[float]]:
  a = np.asarray(arr, dtype=np.float64)
  if a.size == 0:
    return {"mean": [], "std": [], "min": [], "max": []}
  if a.ndim == 1:
    a = a.reshape(-1, 1)
  return {
    "mean": a.mean(axis=0).round(4).tolist(),
    "std": a.std(axis=0).round(4).tolist(),
    "min": a.min(axis=0).round(4).tolist(),
    "max": a.max(axis=0).round(4).tolist(),
  }


def episode_stats(episode: Episode) -> dict:
  n = len(episode)
  ts = episode.timestamps()
  duration = float(ts[-1] - ts[0]) if n > 1 else (n / max(episode.fps, 1.0))
  approach = episode.skin_approach()
  prox = episode.skin_proximity()
  analog = episode.skin_analog()
  dist = episode.skin_distance()
  hit_frames = int(np.any(np.abs(approach) > 0, axis=1).sum()) if n else 0
  collision = np.asarray([f.collision_state for f in episode.frames], dtype=int) if n else np.zeros(0, dtype=int)
  return {
    "name": episode.task,
    "task": episode.task,
    "fps": float(episode.fps),
    "n_frames": n,
    "duration_s": round(duration, 3),
    "hit_frames": hit_frames,
    "hit_rate": round(hit_frames / n, 4) if n else 0.0,
    "collision_frames": int(np.count_nonzero(collision)),
    "parts": list(SKIN_PARTS),
    "skin_proximity": _minmax(prox),
    "skin_analog": _minmax(analog),
    "skin_distance_mm": _minmax(dist),
    "joints": _minmax(np.stack([f.joints for f in episode.frames]) if n else np.zeros((0, 6))),
    "gripper": _minmax(np.asarray([f.gripper_position for f in episode.frames], dtype=float)),
    "camera_frames": len(getattr(episode, "camera_samples", None) or []),
    "cameras": list((getattr(episode, "videos", None) or {}).keys()),
    "has_video": bool(getattr(episode, "videos", None)),
  }


def dataset_stats(episodes: Sequence[Episode]) -> dict:
  rows = [episode_stats(ep) for ep in episodes]
  n = sum(r["n_frames"] for r in rows)
  hits = sum(r["hit_frames"] for r in rows)
  duration = sum(r["duration_s"] for r in rows)
  return {
    "episodes": len(rows),
    "n_frames": n,
    "duration_s": round(duration, 3),
    "hit_frames": hits,
    "hit_rate": round(hits / n, 4) if n else 0.0,
    "tasks": sorted({r["task"] for r in rows}),
    "per_episode": rows,
  }
