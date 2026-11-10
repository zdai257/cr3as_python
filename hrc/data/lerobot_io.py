"""Export / import LeRobot v2.1-style datasets (parquet + meta JSON).

Does not depend on the `lerobot` package. If pyarrow is installed, writes
real parquet; otherwise writes JSONL with the same column names so you can
`lerobot-dataset convert` later.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Union

import numpy as np

from .dataset import Episode
from .frame import ACTION_NAMES, STATE_NAMES


def _stats(array: np.ndarray) -> dict:
  arr = np.asarray(array, dtype=np.float64)
  if arr.size == 0:
    return {"mean": [], "std": [], "min": [], "max": [], "count": [0]}
  if arr.ndim == 1:
    arr = arr.reshape(-1, 1)
  return {
    "mean": arr.mean(axis=0).tolist(),
    "std": arr.std(axis=0).tolist(),
    "min": arr.min(axis=0).tolist(),
    "max": arr.max(axis=0).tolist(),
    "count": [int(arr.shape[0])],
  }


def _try_pyarrow():
  try:
    import pyarrow as pa
    import pyarrow.parquet as pq

    return pa, pq
  except ImportError:
    return None, None


def _episode_table(episode: Episode, episode_index: int, index0: int) -> Dict[str, np.ndarray]:
  n = len(episode)
  states = episode.states()
  actions = episode.actions()
  ts = episode.timestamps()
  if n and ts.size:
    t0 = float(ts[0])
    rel = (ts - t0).astype(np.float32)
  else:
    rel = np.zeros((n,), dtype=np.float32)
  return {
    "observation.state": states.astype(np.float32),
    "observation.skins": episode.skin_proximity().astype(np.float32),
    "action": actions.astype(np.float32),
    "timestamp": rel,
    "frame_index": np.arange(n, dtype=np.int64),
    "episode_index": np.full(n, episode_index, dtype=np.int64),
    "index": np.arange(index0, index0 + n, dtype=np.int64),
    "task_index": np.zeros(n, dtype=np.int64),
  }


def _write_parquet(path: Path, table: Dict[str, np.ndarray]) -> None:
  pa, pq = _try_pyarrow()
  path.parent.mkdir(parents=True, exist_ok=True)
  if pa is None:
    rows = []
    n = next(iter(table.values())).shape[0] if table else 0
    for i in range(n):
      row = {}
      for key, arr in table.items():
        value = arr[i]
        row[key] = value.tolist() if hasattr(value, "tolist") else value
      rows.append(row)
    path.with_suffix(".jsonl").write_text(
      "\n".join(json.dumps(r) for r in rows) + ("\n" if rows else ""),
      encoding="utf-8",
    )
    return
  arrays = {}
  for key, arr in table.items():
    arrays[key] = pa.array(list(arr))
  pq.write_table(pa.table(arrays), path)


def write_lerobot_dataset(
  episodes: List[Episode],
  dest: Union[str, Path],
  *,
  fps: Optional[float] = None,
  robot_type: str = "dobot_cs3as_pgea",
) -> Path:
  dest = Path(dest)
  dest.mkdir(parents=True, exist_ok=True)
  meta = dest / "meta"
  data = dest / "data" / "chunk-000"
  meta.mkdir(parents=True, exist_ok=True)
  data.mkdir(parents=True, exist_ok=True)

  fps = float(fps or (episodes[0].fps if episodes else 125.0))
  tasks = []
  episode_rows = []
  all_states = []
  all_actions = []
  index0 = 0
  for i, ep in enumerate(episodes):
    table = _episode_table(ep, i, index0)
    _write_parquet(data / f"episode_{i:06d}.parquet", table)
    n = len(ep)
    episode_rows.append(
      {
        "episode_index": i,
        "tasks": [ep.task],
        "length": n,
      }
    )
    if ep.task not in tasks:
      tasks.append(ep.task)
    if n:
      all_states.append(ep.states())
      all_actions.append(ep.actions())
    index0 += n

  info = {
    "codebase_version": "v2.1",
    "robot_type": robot_type,
    "fps": fps,
    "features": {
      "observation.state": {
        "dtype": "float32",
        "shape": [len(STATE_NAMES)],
        "names": list(STATE_NAMES),
      },
      "observation.skins": {
        "dtype": "float32",
        "shape": [4],
        "names": ["forearm", "j4", "j5", "j6"],
      },
      "action": {
        "dtype": "float32",
        "shape": [len(ACTION_NAMES)],
        "names": list(ACTION_NAMES),
      },
      "timestamp": {"dtype": "float32", "shape": [1]},
      "frame_index": {"dtype": "int64", "shape": [1]},
      "episode_index": {"dtype": "int64", "shape": [1]},
      "index": {"dtype": "int64", "shape": [1]},
      "task_index": {"dtype": "int64", "shape": [1]},
    },
    "total_episodes": len(episodes),
    "total_frames": int(index0),
    "total_tasks": len(tasks) or 1,
    "splits": {"train": f"0:{len(episodes)}"},
  }
  (meta / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
  (meta / "episodes.jsonl").write_text(
    "\n".join(json.dumps(r) for r in episode_rows) + ("\n" if episode_rows else ""),
    encoding="utf-8",
  )
  (meta / "tasks.jsonl").write_text(
    "\n".join(json.dumps({"task_index": i, "task": t}) for i, t in enumerate(tasks or ["cs3as_hrc"])) + "\n",
    encoding="utf-8",
  )
  states = np.concatenate(all_states, axis=0) if all_states else np.zeros((0, len(STATE_NAMES)))
  actions = np.concatenate(all_actions, axis=0) if all_actions else np.zeros((0, len(ACTION_NAMES)))
  stats = {
    "observation.state": _stats(states),
    "observation.skins": _stats(
      np.concatenate([ep.skin_proximity() for ep in episodes], axis=0)
      if episodes
      else np.zeros((0, 4))
    ),
    "action": _stats(actions),
  }
  (meta / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
  return dest


def read_lerobot_info(dest: Union[str, Path]) -> dict:
  return json.loads((Path(dest) / "meta" / "info.json").read_text(encoding="utf-8"))
