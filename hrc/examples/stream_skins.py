#!/usr/bin/env python3
"""Print live SafeSkin conductance / proximity from port 30004.

  python hrc/examples/stream_skins.py --seconds 20
"""

from __future__ import annotations

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from hrc.data.visualize import format_skin  # noqa: E402
from hrc.robot import CS3ASRobot  # noqa: E402


def main() -> int:
  parser = argparse.ArgumentParser(description="Stream CS3AS SafeSkin")
  parser.add_argument("--seconds", type=float, default=15.0)
  parser.add_argument("--hz", type=float, default=10.0)
  parser.add_argument("--ip", default=None)
  args = parser.parse_args()
  kwargs = {}
  if args.ip:
    kwargs["ip"] = args.ip
  dt = 1.0 / max(args.hz, 1.0)
  with CS3ASRobot.from_config(**kwargs) as robot:
    robot.prepare(enable=False, connect_gripper=False)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
      reading = robot.skins.read()
      if reading is None:
        print("waiting for 30004 ...")
      else:
        print(format_skin(reading))
        print()
      time.sleep(dt)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
