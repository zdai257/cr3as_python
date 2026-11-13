#!/usr/bin/env python3
"""CS3AS pick-and-place with a swappable moving-obstacle planner.

Default is a dry-run against the oracle world in config/robot.yaml.
Teach waypoints there, then rerun with --execute.

Examples:
  python hrc/examples/pick_and_place.py
  python hrc/examples/pick_and_place.py --planner rrt
  python hrc/examples/pick_and_place.py --planner apf --compare
  python hrc/examples/pick_and_place.py --execute  # motion + gripper; e-stop ready
"""

from __future__ import annotations

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from dobot_sdk import load_robot_config # noqa: E402

from hrc.perception import OraclePerception # noqa: E402
from hrc.pipeline import PickPlacePipeline # noqa: E402
from hrc.planning import PLANNER_REGISTRY, make_planner # noqa: E402
from hrc.robot import CS3ASRobot # noqa: E402


def parse_args():
  cfg = load_robot_config()
  parser = argparse.ArgumentParser(description="CS3AS HRC pick-and-place")
  parser.add_argument(
    "--planner",
    default=str(cfg.hrc.planner),
    choices=sorted(PLANNER_REGISTRY),
    help="path planner (default from config/robot.yaml)",
  )
  parser.add_argument(
    "--execute",
    action="store_true",
    help="enable the arm and run the plan (off unless waypoints are taught)",
  )
  parser.add_argument(
    "--compare",
    action="store_true",
    help="run every classical planner on the oracle demo pair",
  )
  parser.add_argument("--ip", default=None, help="override robot IP")
  return parser.parse_args()


def compare_planners(robot: CS3ASRobot) -> None:
  start = [float(v) for v in robot.cfg.hrc.demo_start]
  goal = [float(v) for v in robot.cfg.hrc.demo_goal]
  perception = OraclePerception(robot.cfg, skins=robot.skins)
  obs = perception.observe(pose=start)
  print("\n=== planner comparison (oracle demo_start -> demo_goal) ===")
  for name in ("cartesian_linear", "joint_linear", "rrt", "rrt_star", "apf"):
    planner = make_planner(name, cfg=robot.cfg, kinematics=robot.kin)
    traj = planner.plan(start, goal, obs.world)
    flag = "ok" if traj.success else "FAIL"
    print(
      f" {name:18s} {flag:4s} wps={len(traj.waypoints):3d} "
      f"cost={traj.cost:7.1f} {traj.reason}"
    )


def main() -> int:
  args = parse_args()
  cfg = load_robot_config()
  execute = bool(args.execute)
  if execute:
    print("EXECUTE mode: clear the workspace and keep the e-stop in hand.")

  kwargs = {}
  if args.ip:
    kwargs["ip"] = args.ip

  with CS3ASRobot(cfg, **kwargs) as robot:
    if execute:
      robot.prepare(enable=True, connect_gripper=True)
    else:
      # TCP connect only; no enable / no gripper init.
      robot.prepare(enable=False, connect_gripper=False)
      print(f"connected {robot.ip} mode={robot.mode_name()} tcp={robot.in_tcp_mode()}")

    planner = make_planner(args.planner, cfg=robot.cfg, kinematics=robot.kin)
    pipeline = PickPlacePipeline(robot, planner=planner)
    print(f"planner={planner.name} execute={execute} ip={robot.ip}")
    pipeline.run(execute=execute)
    if args.compare:
      compare_planners(robot)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
