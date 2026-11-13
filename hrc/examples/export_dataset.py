#!/usr/bin/env python3
"""Convert native episode npz files to LeRobot v2.1 and/or rosbag2.

Hub exports are 30 FPS H.264 when a camera exists (joints / SafeSkin are
undersampled onto the camera clock). Native npz stays at the 125 Hz bus.

  python hrc/examples/export_dataset.py data/episodes --lerobot data/lerobot/cs3as --ros2 data/ros2/cs3as
  python hrc/examples/export_dataset.py data/lerobot/skins --lerobot data/lerobot/skins --fps 30
  python hrc/examples/export_dataset.py data/lerobot/skins --push zdai257/cs3as-hrc
  python hrc/examples/export_dataset.py --synthetic  # no robot, writes a tiny demo set
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from hrc.data import Episode, default_dataset_dir, write_lerobot_dataset, write_ros2_bag  # noqa: E402
from hrc.data.lerobot_io import attach_episode_videos, pack_lerobot_for_hub  # noqa: E402


def parse_args():
  parser = argparse.ArgumentParser(description="Export CS3AS episodes")
  parser.add_argument("source", nargs="?", default=None, help="npz, episodes dir, or LeRobot folder")
  parser.add_argument("--lerobot", default=None)
  parser.add_argument("--ros2", default=None, help="directory; one bag subfolder per episode")
  parser.add_argument("--synthetic", action="store_true")
  parser.add_argument(
    "--fps",
    type=float,
    default=None,
    help="LeRobot fps (default: 30 FPS H.264 when a camera exists; else episode fps)",
  )
  parser.add_argument("--push", default=None, help="Hugging Face dataset id, e.g. zdai257/cs3as-hrc")
  return parser.parse_args()


def load_episodes(source: Path):
  if source.is_file():
    ep = Episode.load(source)
    attach_episode_videos(ep, source_path=source)
    return [ep]
  if (source / "meta" / "info.json").is_file():
    from hrc.data import read_lerobot_episodes

    return read_lerobot_episodes(source)
  files = sorted(source.glob("episode_*.npz")) + sorted(source.glob("*.npz"))
  episodes = []
  for path in files:
    ep = Episode.load(path)
    attach_episode_videos(ep, source_path=path)
    episodes.append(ep)
  return episodes


def push_hub(folder: Path, repo_id: str) -> str:
  try:
    from huggingface_hub import HfApi
  except ImportError as exc:
    raise SystemExit("pip install huggingface_hub") from exc
  api = HfApi()
  api.create_repo(repo_id, repo_type="dataset", exist_ok=True, private=False)
  api.upload_folder(
    folder_path=str(folder),
    repo_id=repo_id,
    repo_type="dataset",
    allow_patterns=["data/**", "meta/**", "videos/**/*.mp4", "README.md"],
    ignore_patterns=[
      "**/.DS_Store",
      "**/.cache/**",
      "**/replay.html",
      "**/*.lock",
      "**/*.metadata",
    ],
    delete_patterns=["**/.DS_Store", "replay.html"],
  )
  return f"https://huggingface.co/datasets/{repo_id}"


def main() -> int:
  args = parse_args()
  if args.synthetic:
    episodes = [Episode.synthetic(n=40, task="synthetic_hrc")]
    if args.lerobot:
      dest = write_lerobot_dataset(episodes, args.lerobot, fps=args.fps)
      print(f"lerobot -> {dest}")
    return 0
  src = Path(args.source) if args.source else default_dataset_dir()
  dest = Path(args.lerobot) if args.lerobot else None
  if dest is None and args.push:
    dest = src if src.is_dir() else src.parent
  if dest is not None:
    if src.is_dir() and (src / "meta" / "info.json").is_file():
      out = pack_lerobot_for_hub(src, dest, fps=float(args.fps or 30.0))
    else:
      episodes = load_episodes(src)
      if not episodes:
        print("no episodes found")
        return 1
      print(f"{len(episodes)} episodes, {sum(len(e) for e in episodes)} frames")
      kwargs = {}
      if args.fps is not None:
        kwargs["fps"] = args.fps
        kwargs["video_fps"] = args.fps
      out = write_lerobot_dataset(episodes, dest, **kwargs)
    print(f"lerobot -> {out}")
    dest = out
  if args.ros2:
    episodes = load_episodes(src)
    root = Path(args.ros2)
    for i, ep in enumerate(episodes):
      bag = write_ros2_bag(ep, root / f"episode_{i:04d}")
      print(f"ros2 bag -> {bag}")
  if args.push:
    if dest is None:
      print("pass --lerobot DIR (or a LeRobot source folder) with --push")
      return 2
    url = push_hub(Path(dest), args.push)
    print(f"hub -> {url}")
    print(f"visualizer -> https://huggingface.co/spaces/lerobot/visualize_dataset?path=/{args.push}/episode_0")
  if dest is None and not args.ros2 and not args.push:
    print("pass --lerobot DIR and/or --ros2 DIR and/or --push REPO")
    return 2
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
