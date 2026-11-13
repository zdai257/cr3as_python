"""Export / import LeRobot v2.1-style datasets (parquet + meta JSON).

Does not depend on the `lerobot` package. If pyarrow is installed, writes
real parquet; otherwise writes JSONL with the same column names so you can
`lerobot-dataset convert` later.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Union

import numpy as np

from .dataset import Episode, resample_episode
from .frame import ACTION_NAMES, STATE_NAMES, CameraSample, EpisodeFrame
from ..perception.camera import open_h264_writer

HUB_VIDEO_FPS = 30.0
HAND_EYE_KEY = "observation.images.handeye"


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


def _handeye_video_path(dest: Path, episode_index: int) -> Path:
  return dest / "videos" / "chunk-000" / HAND_EYE_KEY / f"episode_{episode_index:06d}.mp4"


def probe_video(path: Union[str, Path]) -> dict:
  path = Path(path)
  if shutil.which("ffprobe"):
    cmd = [
      "ffprobe",
      "-v",
      "error",
      "-select_streams",
      "v:0",
      "-show_entries",
      "stream=codec_name,width,height,nb_frames,avg_frame_rate,r_frame_rate,pix_fmt,duration",
      "-show_entries",
      "format=duration",
      "-of",
      "json",
      str(path),
    ]
    data = json.loads(subprocess.check_output(cmd, text=True))
    stream = (data.get("streams") or [{}])[0]

    def _rate(text) -> float:
      text = str(text or "")
      if not text or text in {"0/0", "N/A"}:
        return 0.0
      if "/" in text:
        num, den = text.split("/", 1)
        den_f = float(den)
        return float(num) / den_f if den_f else 0.0
      return float(text)

    n = int(stream.get("nb_frames") or 0)
    fps = _rate(stream.get("avg_frame_rate")) or _rate(stream.get("r_frame_rate"))
    duration = float(stream.get("duration") or (data.get("format") or {}).get("duration") or 0.0)
    if n <= 0 and fps > 0 and duration > 0:
      n = int(round(duration * fps))
    return {
      "codec": str(stream.get("codec_name") or ""),
      "width": int(stream.get("width") or 0),
      "height": int(stream.get("height") or 0),
      "nb_frames": n,
      "fps": float(fps),
      "pix_fmt": str(stream.get("pix_fmt") or ""),
      "duration": float(duration),
    }
  try:
    import cv2
  except ImportError as exc:
    raise RuntimeError("ffprobe or opencv is required to read mp4 metadata") from exc
  cap = cv2.VideoCapture(str(path))
  fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
  n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
  w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
  h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
  cap.release()
  return {
    "codec": "",
    "width": w,
    "height": h,
    "nb_frames": n,
    "fps": fps,
    "pix_fmt": "",
    "duration": (n / fps) if fps else 0.0,
  }


def _ffmpeg_h264_args(fps: float) -> List[str]:
  return [
    "-an",
    "-c:v",
    "libx264",
    "-preset",
    "fast",
    "-crf",
    "23",
    "-pix_fmt",
    "yuv420p",
    "-profile:v",
    "baseline",
    "-level",
    "4.0",
    "-g",
    "2",
    "-keyint_min",
    "2",
    "-bf",
    "0",
    "-movflags",
    "+faststart",
    "-r",
    str(float(fps)),
  ]


def encode_h264(src: Union[str, Path], dest: Union[str, Path], fps: float = HUB_VIDEO_FPS) -> Path:
  """Re-encode any mp4 to browser-playable H.264 (avc1 / yuv420p)."""
  src = Path(src)
  dest = Path(dest)
  dest.parent.mkdir(parents=True, exist_ok=True)
  tmp = dest.with_name(dest.stem + ".tmp.mp4")
  if shutil.which("ffmpeg"):
    cmd = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", str(src)]
    cmd.extend(_ffmpeg_h264_args(fps))
    cmd.append(str(tmp))
    subprocess.check_call(cmd)
    tmp.replace(dest)
    return dest
  try:
    import cv2
  except ImportError as exc:
    raise RuntimeError("ffmpeg or opencv is required to encode Hub videos as H.264") from exc
  cap = cv2.VideoCapture(str(src))
  frames = []
  ok, frame = cap.read()
  while ok and frame is not None:
    frames.append(frame)
    ok, frame = cap.read()
  cap.release()
  if not frames:
    raise RuntimeError(f"no frames in {src}")
  h, w = frames[0].shape[:2]
  writer = open_h264_writer(tmp, fps, (w, h))
  if writer is None:
    raise RuntimeError("OpenCV could not open an H.264 encoder (install ffmpeg)")
  for frame in frames:
    writer.write(frame)
  writer.release()
  tmp.replace(dest)
  return dest


def encode_h264_from_jpegs(
  jpegs: Sequence[bytes], dest: Union[str, Path], fps: float = HUB_VIDEO_FPS
) -> Optional[Path]:
  if not jpegs:
    return None
  dest = Path(dest)
  dest.parent.mkdir(parents=True, exist_ok=True)
  if not shutil.which("ffmpeg"):
    # Decode JPEGs with OpenCV when ffmpeg is not on PATH.
    try:
      import cv2
    except ImportError as exc:
      raise RuntimeError("ffmpeg is required to encode JPEG camera samples") from exc
    frames = []
    for jpeg in jpegs:
      bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
      if bgr is not None:
        frames.append(bgr)
    if not frames:
      return None
    h, w = frames[0].shape[:2]
    tmp = dest.with_name(dest.stem + ".tmp.mp4")
    writer = open_h264_writer(tmp, fps, (w, h))
    if writer is None:
      raise RuntimeError("OpenCV could not open an H.264 encoder (install ffmpeg)")
    for frame in frames:
      if frame.shape[0] != h or frame.shape[1] != w:
        frame = cv2.resize(frame, (w, h))
      writer.write(frame)
    writer.release()
    tmp.replace(dest)
    return dest
  tmp = dest.with_name(dest.stem + ".tmp.mp4")
  with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    for i, jpeg in enumerate(jpegs):
      (root / f"f{i:06d}.jpg").write_bytes(jpeg)
    cmd = [
      "ffmpeg",
      "-y",
      "-hide_banner",
      "-loglevel",
      "error",
      "-framerate",
      str(float(fps)),
      "-i",
      str(root / "f%06d.jpg"),
    ]
    cmd.extend(_ffmpeg_h264_args(fps))
    cmd.append(str(tmp))
    subprocess.check_call(cmd)
  tmp.replace(dest)
  return dest


def _video_feature(height: int, width: int, fps: float, codec: str = "h264", pix_fmt: str = "yuv420p") -> dict:
  return {
    "dtype": "video",
    "shape": [int(height), int(width), 3],
    "names": ["height", "width", "channels"],
    "info": {
      "video.height": int(height),
      "video.width": int(width),
      "video.codec": codec,
      "video.pix_fmt": pix_fmt,
      "video.is_depth_map": False,
      "video.fps": float(fps),
      "video.channels": 3,
      "has_audio": False,
    },
  }


def _episode_video_src(episode: Episode) -> Optional[Path]:
  for raw in dict(getattr(episode, "videos", None) or {}).values():
    path = Path(raw)
    if path.is_file():
      return path
  return None


def _align_camera_jpegs(samples: Sequence[CameraSample], t_new: np.ndarray, t0: float) -> List[bytes]:
  ts = np.asarray([s.timestamp for s in samples], dtype=np.float64)
  rel = ts - float(t0)
  if not np.isfinite(rel).all() or rel.size == 0:
    rel = np.arange(len(samples), dtype=np.float64) / max(len(t_new) - 1, 1)
  idx = np.abs(rel[:, None] - t_new[None, :]).argmin(axis=0)
  return [samples[int(i)].jpeg for i in idx]


def _write_handeye_video(
  dest: Path,
  samples: List[CameraSample],
  episode_index: int,
  fps: float = HUB_VIDEO_FPS,
  *,
  source_mp4: Optional[Path] = None,
  t_new: Optional[np.ndarray] = None,
  t0: float = 0.0,
) -> Optional[Path]:
  path = _handeye_video_path(dest, episode_index)
  path.parent.mkdir(parents=True, exist_ok=True)
  if samples:
    jpegs = (
      _align_camera_jpegs(samples, t_new, t0)
      if t_new is not None
      else [s.jpeg for s in samples]
    )
    return encode_h264_from_jpegs(jpegs, path, fps=fps)
  if source_mp4 is not None and Path(source_mp4).is_file():
    snapshot = Path(source_mp4)
    if snapshot.resolve() == path.resolve():
      copied = path.with_suffix(".src.mp4")
      shutil.copy2(snapshot, copied)
      snapshot = copied
    try:
      return encode_h264(snapshot, path, fps=fps)
    finally:
      extra = path.with_suffix(".src.mp4")
      if extra.is_file():
        extra.unlink()
  return None


def write_lerobot_dataset(
  episodes: List[Episode],
  dest: Union[str, Path],
  *,
  fps: Optional[float] = None,
  robot_type: str = "dobot_cs3as_pgea",
  camera_frames=None,
  video_fps: float = HUB_VIDEO_FPS,
) -> Path:
  """Write a Hub-ready LeRobot v2.1 folder.

  Native episodes may be 125 Hz. If a camera exists, telemetry is
  undersampled onto `video_fps` (30) so parquet length matches the mp4,
  and the video is encoded as H.264 yuv420p (the Hub visualizer cannot
  play OpenCV `mp4v` / MPEG-4).
  """
  dest = Path(dest)
  dest.mkdir(parents=True, exist_ok=True)
  meta = dest / "meta"
  data = dest / "data" / "chunk-000"
  meta.mkdir(parents=True, exist_ok=True)
  data.mkdir(parents=True, exist_ok=True)

  prepared: List[Episode] = []
  video_meta = None
  n_videos = 0
  for i, ep in enumerate(episodes):
    samples = list(getattr(ep, "camera_samples", None) or [])
    if camera_frames and i < len(camera_frames) and camera_frames[i]:
      samples = list(camera_frames[i])
    src_mp4 = _episode_video_src(ep)
    has_video = bool(samples) or (src_mp4 is not None and src_mp4.is_file())
    target_fps = float(video_fps if has_video else (fps if fps is not None else ep.fps))
    n_out = None
    if has_video and src_mp4 is not None and src_mp4.is_file() and not samples:
      probed = probe_video(src_mp4)
      n_out = int(probed.get("nb_frames") or 0) or None
    if abs(float(ep.fps) - target_fps) > 1e-6 or n_out is not None:
      packed = resample_episode(ep, target_fps, n_frames=n_out)
    else:
      packed = ep
    packed.camera_samples = samples
    if src_mp4 is not None:
      packed.videos = {HAND_EYE_KEY: str(src_mp4)}
    ts = packed.timestamps()
    t_rel = (ts - ts[0]) if ts.size else np.zeros((0,), dtype=np.float64)
    t0 = float(ts[0]) if ts.size else 0.0
    video = _write_handeye_video(
      dest,
      samples,
      i,
      fps=target_fps,
      source_mp4=src_mp4,
      t_new=t_rel if samples else None,
      t0=t0,
    )
    if video is not None:
      packed.videos = {HAND_EYE_KEY: str(video.resolve())}
      probed = probe_video(video)
      video_meta = _video_feature(
        probed["height"] or 1080,
        probed["width"] or 1920,
        float(probed["fps"] or target_fps),
        codec=probed["codec"] or "h264",
        pix_fmt=probed["pix_fmt"] or "yuv420p",
      )
      n_out = int(probed.get("nb_frames") or 0) or n_out
      if n_out and len(packed) != n_out:
        packed = resample_episode(packed, float(probed["fps"] or target_fps), n_frames=n_out)
        packed.videos = {HAND_EYE_KEY: str(video.resolve())}
      n_videos += 1
    prepared.append(packed)

  fps_out = float(
    video_fps
    if n_videos
    else (fps if fps is not None else (prepared[0].fps if prepared else 125.0))
  )
  if fps_out == int(fps_out):
    fps_out = int(fps_out)

  tasks = []
  episode_rows = []
  all_states = []
  all_actions = []
  index0 = 0
  for i, ep in enumerate(prepared):
    table = _episode_table(ep, i, index0)
    _write_parquet(data / f"episode_{i:06d}.parquet", table)
    n = len(ep)
    episode_rows.append({"episode_index": i, "tasks": [ep.task], "length": n})
    if ep.task not in tasks:
      tasks.append(ep.task)
    if n:
      all_states.append(ep.states())
      all_actions.append(ep.actions())
    index0 += n

  def _feat(names):
    return {"dtype": "float32", "shape": [len(names)], "names": list(names)}

  info = {
    "codebase_version": "v2.1",
    "robot_type": robot_type,
    "fps": fps_out,
    "features": {
      "observation.state": _feat(STATE_NAMES),
      "observation.skins": _feat(["forearm", "j4", "j5", "j6"]),
      "observation.skin_analog": _feat(["forearm", "j4", "j5", "j6"]),
      "observation.skin_distance": _feat(["forearm", "j4", "j5", "j6"]),
      "observation.skin_approach": _feat(["forearm", "j4", "j5", "j6"]),
      "action": _feat(ACTION_NAMES),
      "timestamp": {"dtype": "float32", "shape": [1], "names": None},
      "frame_index": {"dtype": "int64", "shape": [1], "names": None},
      "episode_index": {"dtype": "int64", "shape": [1], "names": None},
      "index": {"dtype": "int64", "shape": [1], "names": None},
      "task_index": {"dtype": "int64", "shape": [1], "names": None},
    },
    "total_episodes": len(prepared),
    "total_frames": int(index0),
    "total_tasks": len(tasks) or 1,
    "total_videos": n_videos,
    "total_chunks": 1,
    "chunks_size": 1000,
    "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
    "splits": {"train": f"0:{len(prepared)}"},
  }
  if video_meta is not None:
    info["features"][HAND_EYE_KEY] = video_meta
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
      np.concatenate([ep.skin_proximity() for ep in prepared], axis=0) if prepared else np.zeros((0, 4))
    ),
    "observation.skin_analog": _stats(
      np.concatenate([ep.skin_analog() for ep in prepared], axis=0) if prepared else np.zeros((0, 4))
    ),
    "observation.skin_distance": _stats(
      np.concatenate([ep.skin_distance() for ep in prepared], axis=0) if prepared else np.zeros((0, 4))
    ),
    "observation.skin_approach": _stats(
      np.concatenate([ep.skin_approach() for ep in prepared], axis=0) if prepared else np.zeros((0, 4))
    ),
    "action": _stats(actions),
  }
  stats_rows = []
  for i, ep in enumerate(prepared):
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


def pack_lerobot_for_hub(
  source: Union[str, Path],
  dest: Union[str, Path, None] = None,
  *,
  fps: float = HUB_VIDEO_FPS,
) -> Path:
  """Rebuild a LeRobot folder at camera fps with H.264 videos."""
  source = Path(source)
  dest = Path(dest) if dest is not None else source
  if source.is_file():
    ep = Episode.load(source)
    attach_episode_videos(ep, source_path=source)
    episodes = [ep]
  else:
    episodes = read_lerobot_episodes(source)
  if dest.resolve() == source.resolve():
    with tempfile.TemporaryDirectory() as tmp:
      staged = write_lerobot_dataset(episodes, Path(tmp) / "hub", fps=fps, video_fps=fps)
      for name in ("meta", "data", "videos", "README.md"):
        src_path = staged / name
        dst_path = dest / name
        if dst_path.is_dir():
          shutil.rmtree(dst_path)
        elif dst_path.is_file():
          dst_path.unlink()
        if src_path.is_dir():
          shutil.copytree(src_path, dst_path)
        elif src_path.is_file():
          shutil.copy2(src_path, dst_path)
    write_hf_card(dest)
    return dest
  return write_lerobot_dataset(episodes, dest, fps=fps, video_fps=fps)


def write_hf_card(dest: Union[str, Path], *, info: Optional[dict] = None, tasks=None) -> Path:
  """Hugging Face dataset card so the Hub parquet viewer + LeRobot space work."""
  dest = Path(dest)
  info = info or (read_lerobot_info(dest) if (dest / "meta" / "info.json").is_file() else {})
  if not tasks:
    tasks_path = dest / "meta" / "tasks.jsonl"
    tasks = []
    if tasks_path.is_file():
      for line in tasks_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
          tasks.append(json.loads(line).get("task", "cs3as_hrc"))
    tasks = tasks or ["cs3as_hrc"]
  n = int(info.get("total_episodes") or 0)
  frames = int(info.get("total_frames") or 0)
  fps = info.get("fps", 125)
  repo = Path(__file__).resolve().parents[2]
  try:
    shown = dest.resolve().relative_to(repo)
  except ValueError:
    shown = Path("data/lerobot") / dest.name
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

Parquet columns include `observation.state`, `observation.skins`, `observation.skin_analog`, `observation.skin_distance`, `observation.skin_approach`, and `action`. Hand-eye video (if recorded) is `observation.images.handeye` (H.264 / `avc1` at 30 FPS). Native `data/episodes/*.npz` stays at the 125 Hz robot bus; this Hub folder is undersampled to the camera clock so the LeRobot visualizer can play image + state together.

## Visualize

Local player (cameras play next to joints / SafeSkin; `--serve` so the browser can load mp4s):

```bash
python hrc/examples/view_episodes.py {shown} --camera --serve
python hrc/examples/visualize_episode.py {shown} --gui --camera
```

Hugging Face [LeRobot dataset visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset) after upload.

`YOUR_ORG` is **not** a CS3AS constant. It is your Hugging Face **username** (personal account) or **organization slug**. Print it with:

```bash
huggingface-cli whoami
# or:  hf auth whoami
```

Use the `name` field for a personal dataset (`alice/cs3as-hrc`). Use an org you can write to for a shared dataset (`my-lab/cs3as-hrc`).

```bash
huggingface-cli upload-folder --repo-type dataset YOUR_ORG/cs3as-hrc {shown}
```

Then paste `YOUR_ORG/cs3as-hrc` into the visualizer, or open:

`https://huggingface.co/spaces/lerobot/visualize_dataset?path=/YOUR_ORG/cs3as-hrc/episode_0`

`meta/info.json` follows LeRobot **v2.1** (`data_path` / `video_path` templates). Convert to v3 with `python -m lerobot.scripts.convert_dataset_v21_to_v30` if a newer toolchain requires it.
"""
  path = dest / "README.md"
  path.write_text(card, encoding="utf-8")
  return path


def episode_video_paths(dest: Union[str, Path], episode_index: int = 0) -> Dict[str, Path]:
  """Map LeRobot video feature keys to mp4 files for one episode."""
  dest = Path(dest)
  found: Dict[str, Path] = {}
  template = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
  chunk_size = 1000
  info_path = dest / "meta" / "info.json"
  features = {}
  if info_path.is_file():
    info = json.loads(info_path.read_text(encoding="utf-8"))
    template = str(info.get("video_path") or template)
    chunk_size = int(info.get("chunks_size") or 1000)
    features = dict(info.get("features") or {})
  chunk = int(episode_index) // max(chunk_size, 1)
  for key, feat in features.items():
    if not (isinstance(feat, dict) and feat.get("dtype") == "video"):
      continue
    rel = template.format(
      episode_chunk=chunk, video_key=key, episode_index=int(episode_index)
    )
    path = dest / rel
    if path.is_file():
      found[str(key)] = path.resolve()
  pattern = f"**/episode_{int(episode_index):06d}.mp4"
  for path in dest.glob(pattern):
    found.setdefault(path.parent.name, path.resolve())
  return found


def attach_episode_videos(
  episode: Episode,
  *,
  dataset_dir: Union[str, Path, None] = None,
  source_path: Union[str, Path, None] = None,
) -> Episode:
  """Fill episode.videos from a LeRobot folder and/or a sibling npz/mp4."""
  found: Dict[str, Path] = {}
  for key, raw in dict(getattr(episode, "videos", None) or {}).items():
    path = Path(raw)
    if path.is_file():
      found[str(key)] = path.resolve()
  if dataset_dir is not None:
    found.update(episode_video_paths(dataset_dir, episode.episode_index))
  if source_path is not None:
    src = Path(source_path)
    sibling = src.with_suffix(".mp4")
    if sibling.is_file():
      found.setdefault("camera", sibling.resolve())
    if src.is_file():
      guess = src.parent.parent / "lerobot" / src.stem
      if (guess / "meta" / "info.json").is_file():
        found.update(episode_video_paths(guess, episode.episode_index))
  episode.videos = {key: str(path) for key, path in found.items()}
  return episode


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
    attach_episode_videos(ep, dataset_dir=dest)
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
