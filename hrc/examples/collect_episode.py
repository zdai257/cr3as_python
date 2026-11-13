#!/usr/bin/env python3
"""Record a CS3AS episode from Feedback port 30004.

Dry-run (no robot motion) still needs a live controller so 30004 streams:

  python hrc/examples/collect_episode.py --seconds 5

Export while recording:

  python hrc/examples/collect_episode.py --seconds 8 --lerobot data/lerobot/demo --ros2 data/ros2/demo
"""

from __future__ import annotations

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from dobot_sdk import load_robot_config  # noqa: E402

from hrc.data import EpisodeRecorder, write_lerobot_dataset, write_ros2_bag  # noqa: E402
from hrc.robot import CS3ASRobot  # noqa: E402


def parse_args():
  cfg = load_robot_config()
  parser = argparse.ArgumentParser(description="Record CS3AS HRC episode")
  parser.add_argument("--seconds", type=float, default=5.0)
  parser.add_argument("--task", default=str(cfg.hrc.learning.task))
  parser.add_argument("--out", default=None, help="npz path; default data/episodes/episode_XXXX.npz")
  parser.add_argument("--lerobot", default=None, help="write a LeRobot v2.1 dataset directory")
  parser.add_argument("--ros2", default=None, help="write a rosbag2 directory")
  parser.add_argument("--enable", action="store_true", help="enable the arm (still no commanded motion)")
  parser.add_argument("--camera", action="store_true", help="record ELP hand-eye JPEG/mp4")
  parser.add_argument("--ip", default=None)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  kwargs = {}
  if args.ip:
    kwargs["ip"] = args.ip
  with CS3ASRobot.from_config(**kwargs) as robot:
    robot.prepare(
      enable=bool(args.enable),
      connect_gripper=True,
      connect_camera=bool(args.camera),
    )
    rec = EpisodeRecorder(robot, cfg=robot.cfg, task=args.task)
    rec.start(args.task)
    print(f"recording {args.seconds:.1f}s of 30004 / gripper / SafeSkin ...")
    t0 = time.time()
    while time.time() - t0 < args.seconds:
      if len(rec.episode) == 0:
        rec.capture()
      time.sleep(0.05)
      n = len(rec.episode)
      print(f"  {n:5d} frames  {time.time() - t0:5.1f}s", end="\r")
    print()
    path, ep = rec.save(args.out)
    print(f"saved {len(ep)} frames -> {path}")
    if args.lerobot:
      dest = write_lerobot_dataset([ep], args.lerobot, fps=ep.fps)
      print(f"lerobot dataset -> {dest}")
    if args.ros2:
      dest = write_ros2_bag(ep, args.ros2)
      print(f"ros2 bag -> {dest}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
