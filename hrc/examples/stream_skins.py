#!/usr/bin/env python3
"""Live SafeSkin HIT / analog stream from CS3ASRobot.stream_skins.

Sensing is always the 30004 rate (~125 Hz / 8 ms). Console printing is
downsampled unless --native. Analog channels are the 61 reserved bytes
after the I8 HIT flags; I8 itself is typically a 0/1 pause flag.

  python hrc/examples/stream_skins.py --seconds 20
  python hrc/examples/stream_skins.py --native --record data/episodes/skins.npz
  python hrc/examples/stream_skins.py --lerobot data/lerobot/skins --camera
"""

from __future__ import annotations

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from hrc.data import EpisodeRecorder, write_lerobot_dataset  # noqa: E402
from hrc.data.visualize import format_skin  # noqa: E402
from hrc.robot import CS3ASRobot  # noqa: E402

PENDANT_HINT = """
Pendant / DobotStudio (do this once; otherwise HIT reverts to Online+Disable):
  1. Control source: TCP. Turn OFF hand-auto / Online project.
  2. Collision post-process: Pause (not Stop/Disable). Level 0 for sensing tests.
  3. SafeSkin: keep the sensor; turn OFF obstacle-avoid / stop-on-approach.
TCP session settings revert to those pendant defaults when this script exits.
"""

HOME = [0, 0, 0, 0, 0, 0]
SAFE_POSE = [-115.77, -6.38, 76.08, 21.82, -91.69, 100.43]


def main() -> int:
  parser = argparse.ArgumentParser(description="Stream CS3AS SafeSkin")
  parser.add_argument("--seconds", type=float, default=15.0)
  parser.add_argument(
    "--hz",
    type=float,
    default=12.5,
    help="console print rate; sensing stays at 125 Hz",
  )
  parser.add_argument("--native", action="store_true", help="print every 8 ms sample")
  parser.add_argument("--ip", default=None)
  parser.add_argument("--record", default=None, help="save episode npz")
  parser.add_argument("--lerobot", default=None, help="export LeRobot v2.1 parquet dir")
  parser.add_argument("--camera", action="store_true", help="also open /dev/video0")
  parser.add_argument(
    "--firmware-safety",
    action="store_true",
    help="let the controller pause/protect on approach (default: sense only)",
  )
  args = parser.parse_args()
  kwargs = {}
  if args.ip:
    kwargs["ip"] = args.ip
  interval = 0.0 if args.native else 1.0 / max(args.hz, 0.1)
  with CS3ASRobot.from_config(**kwargs) as robot:
    robot.prepare(enable=True, connect_gripper=False, connect_camera=bool(args.camera))
    if args.firmware_safety:
      robot.skins.enable(firmware_safety=True)
    cell = robot.cell_status()
    print(
      f"CR3AS mode={cell['mode_name']} tcp={cell['tcp']} "
      f"enabled={cell['enabled']} collision={cell['collision_state']} "
      f"safety={cell['safety']}"
    )
    print(
      f"SafeSkin active={cell['skins']['active']} "
      f"firmware_safety={cell['skins']['firmware_safety']} "
      f"auto_clear={cell['skins']['auto_clear']}"
    )
    cam = cell.get("camera") or {}
    print(
      f"camera open={cam.get('open')} {cam.get('device')} "
      f"actual={cam.get('actual')}"
    )
    print("HIT = I8 approach flag; analog = 30004 reserved ADC (fine-grained).")
    print(PENDANT_HINT)
    try:
      print(f"robot current joints: {robot.current_joints()}")
      print(f"robot current pose: {robot.current_pose()}")
    except Exception:
      pass
    rec = None
    if args.record or args.lerobot:
      rec = EpisodeRecorder(robot, cfg=robot.cfg, fps=125.0, task="safeskin")
      rec.start("safeskin")
    last_print = 0.0
    n = 0
    for reading in robot.stream_skins(seconds=args.seconds, recover=True):
      n += 1
      now = time.time()
      if interval == 0.0 or now - last_print >= interval:
        last_print = now
        print(format_skin(reading))
        status = robot.GetStatus()
        collision = int(getattr(status, "collision_state", 0) or 0) if status else 0
        enabled = bool(getattr(status, "enable_status", False)) if status else False
        print(
          f"  n={n}  mode={robot.mode_name()}  enabled={enabled}  "
          f"collision={collision}  tcp={robot.in_tcp_mode()}"
        )
        if args.camera:
          frame = robot.camera.latest()
          if frame is None:
            print("  cam: no frame")
          else:
            print(f"  cam {frame.width}x{frame.height} t={frame.timestamp:.3f}")
        if reading.triggered() and not args.firmware_safety:
          print("  NEAR/HIT -> ClearError + EnableRobot (still TCP)")
        print()
    if rec is not None:
      path, ep = rec.save(args.record)
      print(f"saved {len(ep)} frames @ {ep.fps:.0f} Hz -> {path}")
      if args.lerobot:
        dest = write_lerobot_dataset([ep], args.lerobot, fps=ep.fps)
        print(f"lerobot dataset -> {dest}")
    robot.shutdown(disable=False, skins_off=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
