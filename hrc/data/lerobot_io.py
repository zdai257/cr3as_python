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
from .frame import ACTION_NAMES, STATE_NAMES, CameraSample, EpisodeFrame


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
    "observation.skin_analog": episode.skin_analog().astype(np.float32),
    "observation.skin_distance": episode.skin_distance().astype(np.float32),
    "observation.skin_approach": episode.skin_approach().astype(np.float32),
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


def _write_handeye_video(
  dest: Path, samples: List[CameraSample], episode_index: int, fps: float = 30.0
) -> Optional[Path]:
  if not samples:
    return None
  try:
    import cv2
  except ImportError:
    return None
  path = (
    dest
    / "videos"
    / "chunk-000"
    / "observation.images.handeye"
    / f"episode_{episode_index:06d}.mp4"
  )
  path.parent.mkdir(parents=True, exist_ok=True)
  first = cv2.imdecode(np.frombuffer(samples[0].jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
  if first is None:
    return None
  h, w = first.shape[:2]
  writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (w, h))
  writer.write(first)
  for sample in samples[1:]:
    bgr = cv2.imdecode(np.frombuffer(sample.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is not None:
      if bgr.shape[0] != h or bgr.shape[1] != w:
        bgr = cv2.resize(bgr, (w, h))
      writer.write(bgr)
  writer.release()
  return path


def write_lerobot_dataset(
  episodes: List[Episode],
  dest: Union[str, Path],
  *,
  fps: Optional[float] = None,
  robot_type: str = "dobot_cs3as_pgea",
  camera_frames=None,
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
      "observation.skin_analog": {
        "dtype": "float32",
        "shape": [4],
        "names": ["forearm", "j4", "j5", "j6"],
      },
      "observation.skin_distance": {
        "dtype": "float32",
        "shape": [4],
        "names": ["forearm", "j4", "j5", "j6"],
      },
      "observation.skin_approach": {
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
    "total_chunks": 1,
    "chunks_size": 1000,
    "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
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
    "observation.skin_analog": _stats(
      np.concatenate([ep.skin_analog() for ep in episodes], axis=0)
      if episodes
      else np.zeros((0, 4))
    ),
    "observation.skin_distance": _stats(
      np.concatenate([ep.skin_distance() for ep in episodes], axis=0)
      if episodes
      else np.zeros((0, 4))
    ),
    "observation.skin_approach": _stats(
      np.concatenate([ep.skin_approach() for ep in episodes], axis=0)
      if episodes
      else np.zeros((0, 4))
    ),
    "action": _stats(actions),
  }

  has_cam = any(getattr(ep, "camera_samples", None) for ep in episodes)
  if has_cam:
    cam_shape = [1080, 1920, 3]
    cam_fps = 30.0
    for i, ep in enumerate(episodes):
      samples = list(getattr(ep, "camera_samples", None) or [])
      if camera_frames and i < len(camera_frames) and camera_frames[i]:
        samples = camera_frames[i]
      video = _write_handeye_video(dest, samples, i, fps=cam_fps)
      if video is not None and samples:
        cam_shape = [int(samples[0].height), int(samples[0].width), 3]
        info["features"]["observation.images.handeye"] = {
          "dtype": "video",
          "shape": cam_shape,
          "names": ["height", "width", "channel"],
          "info": {"video.fps": cam_fps, "video.codec": "mp4v"},
        }
    (meta / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
  stats_rows = []
  for i, ep in enumerate(episodes):
    stats_rows.append(
      {
        "episode_index": i,
        "stats": {
          "observation.state": _stats(ep.states()),
          "observation.skins": _stats(ep.skin_proximity()),
          "observation.skin_analog": _stats(ep.skin_analog()),
          "observation.skin_distance": _stats(ep.skin_distance()),
          "observation.skin_approach": _stats(ep.skin_approach()),
          "action": _stats(ep.actions()),
        },
      }
    )
  (meta / "episodes_stats.jsonl").write_text(
    "\n".join(json.dumps(r) for r in stats_rows) + ("\n" if stats_rows else ""),
    encoding="utf-8",
  )
  (meta / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
  write_hf_card(dest, info=info, tasks=tasks or ["cs3as_hrc"])
  return dest


def write_hf_card(dest: Union[str, Path], *, info: Optional[dict] = None, tasks=None) -> Path:
  """Hugging Face dataset card so the Hub parquet viewer + LeRobot space work."""
  dest = Path(dest)
  info = info or (read_lerobot_info(dest) if (dest / "meta" / "info.json").is_file() else {})
  tasks = tasks or ["cs3as_hrc"]
  n = int(info.get("total_episodes") or 0)
  frames = int(info.get("total_frames") or 0)
  fps = info.get("fps", 125)
  card = f"""---
license: mit
task_categories:
- robotics
tags:
- LeRobot
- robotics
- dobot
- cs3as
- safeskin
configs:
- config_name: default
  data_files: data/*/*.parquet
---

# CS3AS HRC episodes (LeRobot v2.1)

- episodes: {n}
- frames: {frames}
- fps: {fps}
- tasks: {", ".join(str(t) for t in tasks)}
- robot: `{info.get("robot_type", "dobot_cs3as_pgea")}`

Parquet columns include `observation.state`, `observation.skins`, `observation.skin_analog`, `observation.skin_distance`, `observation.skin_approach`, and `action`. Hand-eye video (if recorded) is `observation.images.handeye`.

## Visualize

Local player (no Hub upload):

```bash
python hrc/examples/view_episodes.py {dest}
```

Hugging Face [LeRobot dataset visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset) after `huggingface-cli upload`:

```bash
huggingface-cli upload-folder --repo-type dataset YOUR_ORG/cs3as-hrc {dest}
```

Then open `https://huggingface.co/spaces/lerobot/visualize_dataset?path=YOUR_ORG/cs3as-hrc/0`.

`meta/info.json` follows LeRobot **v2.1** (`data_path` / `video_path` templates). Convert to v3 with `python -m lerobot.scripts.convert_dataset_v21_to_v30` if a newer toolchain requires it.
"""
  path = dest / "README.md"
  path.write_text(card, encoding="utf-8")
  return path


def read_lerobot_episodes(dest: Union[str, Path]) -> List[Episode]:
  """Load episodes back from a LeRobot v2.1 folder (parquet or jsonl)."""
  dest = Path(dest)
  info = read_lerobot_info(dest)
  fps = float(info.get("fps") or 125.0)
  tasks = []
  tasks_path = dest / "meta" / "tasks.jsonl"
  if tasks_path.is_file():
    for line in tasks_path.read_text(encoding="utf-8").splitlines():
      if line.strip():
        tasks.append(json.loads(line).get("task", "cs3as_hrc"))
  data = dest / "data" / "chunk-000"
  files = sorted(data.glob("episode_*.parquet")) + sorted(data.glob("episode_*.jsonl"))
  episodes = []
  for i, path in enumerate(files):
    table = _read_table(path)
    n = len(next(iter(table.values()))) if table else 0
    ep = Episode(
      task=str(tasks[i] if i < len(tasks) else tasks[0] if tasks else "cs3as_hrc"),
      fps=fps,
      episode_index=i,
    )
    states = np.asarray(table.get("observation.state", np.zeros((n, len(STATE_NAMES)))))
    skins = np.asarray(table.get("observation.skins", np.zeros((n, 4))))
    analog = np.asarray(table.get("observation.skin_analog", np.zeros((n, 4))))
    dist = np.asarray(table.get("observation.skin_distance", np.zeros((n, 4))))
    approach = np.asarray(table.get("observation.skin_approach", np.zeros((n, 4))))
    actions = np.asarray(table.get("action", np.zeros((n, len(ACTION_NAMES)))))
    ts = np.asarray(table.get("timestamp", np.arange(n) / max(fps, 1.0)), dtype=float)
    for k in range(n):
      st = np.asarray(states[k], dtype=np.float32).reshape(-1)
      ep.append(
        EpisodeFrame(
          timestamp=float(ts[k]),
          timestamp_ms=int(float(ts[k]) * 1000),
          joints=st[:6] if st.size >= 6 else np.zeros(6, dtype=np.float32),
          pose=st[6:12] if st.size >= 12 else np.zeros(6, dtype=np.float32),
          gripper_position=float(st[12]) if st.size > 12 else 0.0,
          skin_proximity=np.asarray(skins[k], dtype=np.float32).reshape(-1)[:4],
          skin_analog=np.asarray(analog[k], dtype=np.float32).reshape(-1)[:4],
          skin_distance_mm=np.asarray(dist[k], dtype=np.float32).reshape(-1)[:4],
          skin_approach=np.asarray(approach[k], dtype=np.float32).reshape(-1)[:4],
          action=np.asarray(actions[k], dtype=np.float32).reshape(-1)[:7],
        )
      )
    episodes.append(ep)
  return episodes


def _read_table(path: Path) -> Dict[str, np.ndarray]:
  if path.suffix == ".jsonl":
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
      return {}
    table = {}
    for key in rows[0]:
      table[key] = np.asarray([r[key] for r in rows])
    return table
  pa, pq = _try_pyarrow()
  if pa is None:
    raise RuntimeError("pyarrow is required to read parquet episodes")
  table = pq.read_table(path)
  out = {}
  for name in table.column_names:
    out[name] = np.asarray(table[name].to_pylist())
  return out


def read_lerobot_info(dest: Union[str, Path]) -> dict:
  return json.loads((Path(dest) / "meta" / "info.json").read_text(encoding="utf-8"))
