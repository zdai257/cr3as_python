#!/usr/bin/env python3
"""Plot a recorded episode (npz / LeRobot folder) or print a live SafeSkin stream.

Camera mp4s play concurrently in --gui / the HTML player (default on).

  python hrc/examples/visualize_episode.py data/episodes/skins.npz --gui --camera
  python hrc/examples/visualize_episode.py data/lerobot/skins --gui
  python hrc/examples/visualize_episode.py data/episodes/skins.npz --png /tmp/ep.png
  python hrc/examples/visualize_episode.py --live --seconds 10
"""

from __future__ import annotations

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from pathlib import Path  # noqa: E402

from hrc.data import Episode, LivePrinter, plot_episode  # noqa: E402
from hrc.data.lerobot_io import attach_episode_videos  # noqa: E402
from hrc.data.viewer import discover_episodes, replay_gui  # noqa: E402


def parse_args():
  parser = argparse.ArgumentParser(description="Visualise CS3AS episode / skins")
  parser.add_argument("episode", nargs="?", help="path to episode npz or LeRobot folder")
  parser.add_argument("--png", default=None, help="save matplotlib figure")
  parser.add_argument("--show", action="store_true")
  parser.add_argument("--gui", action="store_true", help="play/scrub window with optional camera pane")
  parser.add_argument(
    "--camera",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="show hand-eye video next to telemetry (default: on when an mp4 exists)",
  )
  parser.add_argument("--live", action="store_true")
  parser.add_argument("--seconds", type=float, default=8.0)
  parser.add_argument("--ip", default=None)
  return parser.parse_args()


def live(args) -> int:
  from hrc.robot import CS3ASRobot

  kwargs = {}
  if args.ip:
    kwargs["ip"] = args.ip
  printer = LivePrinter(every=12)
  with CS3ASRobot.from_config(**kwargs) as robot:
    robot.prepare(enable=True, connect_gripper=False)
    for reading in robot.stream_skins(seconds=args.seconds):
      try:
        joints = robot.current_joints()
        pose = robot.current_pose()
      except Exception:
        joints = pose = None
      printer(reading, joints=joints, pose=pose)
  return 0


def main() -> int:
  args = parse_args()
  if args.live:
    return live(args)
  if not args.episode:
    print("pass an episode npz / LeRobot folder or use --live")
    return 2
  src = Path(args.episode)
  if src.is_dir():
    episodes = discover_episodes(src)
    if not episodes:
      print(f"no episodes in {src}")
      return 2
    ep = episodes[0]
  else:
    ep = Episode.load(src)
    attach_episode_videos(ep, source_path=src)
  cams = list((ep.videos or {}).keys())
  print(f"{ep.task}  frames={len(ep)}  fps={ep.fps}" + (f"  cameras={cams}" if cams else "  cameras=none"))
  if args.camera and not cams:
    print("no camera mp4 attached; pass data/lerobot/skins or record with --camera")
  if args.gui:
    replay_gui(ep, camera=bool(args.camera))
    return 0
  if ep.frames:
    from hrc.perception.skins import SkinReading

    f = ep.frames[0]
    print(
      SkinReading(
        timestamp_ms=f.timestamp_ms,
        approach=f.skin_approach,
        proximity=f.skin_proximity,
        conductance=f.skin_conductance,
        analog=f.skin_analog,
      ).as_dict()
    )
  if args.png or args.show:
    plot_episode(ep, out=args.png, show=args.show)
    if args.png:
      print(f"wrote {args.png}")
  else:
    for i, frame in enumerate(ep.frames[:: max(1, len(ep) // 8) ]):
      print(
        f"{i:4d} t={frame.timestamp:.3f} q={frame.joints[:3]} "
        f"skin={frame.skin_proximity} g={frame.gripper_position:.0f}"
      )
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
