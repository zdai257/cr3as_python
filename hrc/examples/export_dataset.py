#!/usr/bin/env python3
"""Convert native episode npz files to LeRobot v2.1 and/or rosbag2.

  python hrc/examples/export_dataset.py data/episodes --lerobot data/lerobot/cs3as --ros2 data/ros2/cs3as
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


def parse_args():
  parser = argparse.ArgumentParser(description="Export CS3AS episodes")
  parser.add_argument("source", nargs="?", default=None, help="directory of episode_*.npz or a single npz")
  parser.add_argument("--lerobot", default=None)
  parser.add_argument("--ros2", default=None, help="directory; one bag subfolder per episode")
  parser.add_argument("--synthetic", action="store_true")
  return parser.parse_args()


def load_episodes(source: Path):
  if source.is_file():
    return [Episode.load(source)]
  files = sorted(source.glob("episode_*.npz"))
  return [Episode.load(p) for p in files]


def main() -> int:
  args = parse_args()
  if args.synthetic:
    episodes = [Episode.synthetic(n=40, task="synthetic_hrc")]
  else:
    src = Path(args.source) if args.source else default_dataset_dir()
    episodes = load_episodes(src)
  if not episodes:
    print("no episodes found")
    return 1
  print(f"{len(episodes)} episodes, {sum(len(e) for e in episodes)} frames")
  if args.lerobot:
    dest = write_lerobot_dataset(episodes, args.lerobot)
    print(f"lerobot -> {dest}")
  if args.ros2:
    root = Path(args.ros2)
    for i, ep in enumerate(episodes):
      dest = write_ros2_bag(ep, root / f"episode_{i:04d}")
      print(f"ros2 bag -> {dest}")
  if not args.lerobot and not args.ros2:
    print("pass --lerobot DIR and/or --ros2 DIR")
    return 2
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
