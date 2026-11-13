#!/usr/bin/env python3
"""Print live SafeSkin conductance / proximity from port 30004.

HIT (I8 approach flag) is treated as a sensor reading. The TCP session
stays enabled, does not DisableRobot, and does not hand control back to
Online/pendant until the process exits.

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
  parser.add_argument("--hz", type=float, default=10.0)
  parser.add_argument("--ip", default=None)
  parser.add_argument(
    "--firmware-safety",
    action="store_true",
    help="let the controller pause/protect on approach (default: sense only)",
  )
  args = parser.parse_args()
  kwargs = {}
  if args.ip:
    kwargs["ip"] = args.ip
  dt = 1.0 / max(args.hz, 1.0)
  with CS3ASRobot.from_config(**kwargs) as robot:
    # Enable idle so queued EnableSafeSkin / SetPostCollisionMode actually run.
    robot.prepare(enable=True, connect_gripper=False)
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
    print("HIT = approach flag only; TCP stays enabled (no Disable / Online).")
    print(PENDANT_HINT)
    joints = robot.current_joints()
    print(f"robot current joints: {joints}")
    pose = robot.current_pose()
    print(f"robot current pose: {pose}")
    print(f"\nWait {2} s...")
    time.sleep(2)

    t0 = time.time()
    while time.time() - t0 < args.seconds:
      reading = robot.skins.read()
      mode = robot.recover_if_protected()
      if reading is None:
        print("waiting for 30004 ...")
      else:
        print(format_skin(reading))
        status = robot.GetStatus()
        collision = int(getattr(status, "collision_state", 0) or 0) if status else 0
        enabled = bool(getattr(status, "enable_status", False)) if status else False
        auto = int(getattr(status, "auto_manual_mode", 0) or 0) if status else 0
        print(
          f"  mode={robot.mode_name(mode)}  enabled={enabled}  "
          f"collision={collision}  tcp={robot.in_tcp_mode()}  auto/manual={auto}"
        )
        if reading.triggered() and not args.firmware_safety:
          print("  NEAR/HIT latched firmware protect -> ClearError + EnableRobot (still TCP)")
        print()
      time.sleep(dt)
    # Disarm SafeSkin while TCP still owns the controller, but do not Disable.
    robot.shutdown(disable=False, skins_off=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
