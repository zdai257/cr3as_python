#!/usr/bin/env python3
"""Record a CS3AS episode from Feedback port 30004.

Native npz is 125 Hz (robot bus). `--lerobot` is Hub-ready 30 FPS H.264.

Dry-run (no robot motion) still needs a live controller so 30004 streams:

  python hrc/examples/collect_episode.py --seconds 5

Hub-ready episode (hand-eye JPEG ~30 FPS + parquet aligned to the camera):

  python hrc/examples/collect_episode.py --seconds 8 --enable --camera --lerobot data/lerobot/demo
"""

from __future__ import annotations

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from dobot_sdk import load_robot_config  # noqa: E402

from hrc.data import EpisodeRecorder, write_ros2_bag  # noqa: E402
from hrc.robot import CS3ASRobot  # noqa: E402


def parse_args():
  cfg = load_robot_config()
  parser = argparse.ArgumentParser(description="Record CS3AS HRC episode")
  parser.add_argument("--seconds", type=float, default=5.0)
  parser.add_argument("--task", default=str(cfg.hrc.learning.task))
  parser.add_argument("--out", default=None, help="npz path; default data/episodes/episode_XXXX.npz")
  parser.add_argument(
    "--lerobot",
    default=None,
    help="write a Hub-ready LeRobot v2.1 folder (30 FPS H.264; npz stays 125 Hz)",
  )
  parser.add_argument("--ros2", default=None, help="write a rosbag2 directory")
  parser.add_argument("--enable", action="store_true", help="enable the arm (still no commanded motion)")
  parser.add_argument("--camera", action="store_true", help="record ELP hand-eye JPEG (~30 FPS); Hub mp4 is H.264")
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
    print(f"saved {len(ep)} frames @ {ep.fps:.0f} Hz npz -> {path}")
    if args.lerobot:
      dest = rec.export_lerobot(args.lerobot, episode=ep)
      print(f"lerobot dataset (H.264 / {rec.hub_fps:.0f} FPS) -> {dest}")
    if args.ros2:
      dest = write_ros2_bag(ep, args.ros2)
      print(f"ros2 bag -> {dest}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
