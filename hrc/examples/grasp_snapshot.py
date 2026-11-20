#!/usr/bin/env python3
"""One-shot GraspNet proposals from the configured camera (or files).

UVC is RGB-only (synthetic depth unless --depth/--cloud). RealSense / Kinect
frames include aligned depth in metres and skip the synthetic table.

  python hrc/examples/grasp_snapshot.py
  python hrc/examples/grasp_snapshot.py --camera
  python hrc/examples/grasp_snapshot.py --camera --driver realsense
  python hrc/examples/grasp_snapshot.py --rgb shot.png --depth depth.png --top 15
  python hrc/examples/grasp_snapshot.py --checkpoint weights/graspnet/checkpoint-rs.tar
  python hrc/examples/grasp_snapshot.py --rgb data/results/color.png --depth data/results/depth.png --workspace-mask data/results/workspace_mask.png --meta data/results/meta.mat --top 50 --checkpoint weights/graspnet/checkpoint-rs.tar --out masked_grasps.json
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from dobot_sdk import load_robot_config  # noqa: E402
from dobot_sdk.config import Cfg  # noqa: E402
from hrc.perception.graspnet import GraspNetEngine  # noqa: E402
from hrc.perception.graspnet.engine import rgbd_from_camera  # noqa: E402


def parse_args():
  parser = argparse.ArgumentParser(description="CS3AS GraspNet snapshot")
  parser.add_argument("--rgb", default=None, help="RGB image (png/jpg). Default: camera or a gray placeholder")
  parser.add_argument("--depth", default=None, help="Depth png/npy (mm or metres)")
  parser.add_argument("--cloud", default=None, help="Nx3 float32 npy, camera frame, metres")
  parser.add_argument("--camera", action="store_true", help="grab from the configured camera")
  parser.add_argument("--driver", default=None, help="override perception.camera (uvc|realsense|k4a)")
  parser.add_argument("--checkpoint", default=None)
  parser.add_argument("--top", type=int, default=10)
  parser.add_argument("--out", default=None, help="write JSON of ranked grasps")
  parser.add_argument("--ip", default=None, help="optional: live TCP pose for base-frame output")
  parser.add_argument(
    "--workspace-mask",
    default=None,
    help="Workspace mask image (non-zero pixels are retained)",
    )
  parser.add_argument(
    "--meta",
    default=None,
    help="GraspNet meta.mat containing intrinsic_matrix and factor_depth",
  )
  return parser.parse_args()


def _with_driver(cfg, driver):
  if not driver:
    return cfg
  data = copy.deepcopy(cfg.to_dict())
  data.setdefault("perception", {})["camera"] = driver
  return Cfg(data)


def _load_image(path: str):
  path = Path(path)
  if path.suffix.lower() == ".npy":
    return np.load(path)
  try:
    import cv2
  except ImportError as exc:
    raise SystemExit("pip install opencv-python-headless to load images") from exc
  bgr = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
  if bgr is None:
    raise SystemExit(f"cannot read {path}")
  if bgr.ndim == 3 and bgr.shape[2] == 3:
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
  return bgr


def main() -> int:
  args = parse_args()
  cfg = _with_driver(load_robot_config(), args.driver)
  engine = GraspNetEngine(cfg, checkpoint=args.checkpoint)
  rgb = None
  live_depth = None
  tcp = None
  joints = None
  kin = None
  if args.rgb:
    rgb = _load_image(args.rgb)
  elif args.camera:
    from hrc.perception.camera import make_camera

    cam = make_camera(cfg)
    try:
      cam.open()
      rgb, live_depth, frame = rgbd_from_camera(cam)
      engine.apply_frame_intrinsics(frame)
    finally:
      cam.close()
    if rgb is None:
      print("camera opened but no frame")
      return 2
  if args.ip is not None or (args.camera and not args.rgb):
    try:
      from hrc.robot import CS3ASRobot

      kwargs = {}
      if args.ip:
        kwargs["ip"] = args.ip
      with CS3ASRobot.from_config(**kwargs) as robot:
        tcp = robot.current_pose()
        joints = robot.current_joints()
        kin = robot.kin
        if rgb is None and args.camera:
          robot.prepare(enable=False, connect_gripper=False, connect_camera=True)
          rgb, more_depth, frame = rgbd_from_camera(robot.camera)
          engine.apply_frame_intrinsics(frame)
          if live_depth is None:
            live_depth = more_depth
    except Exception as exc:
      print(f"robot pose skipped ({exc})")

  depth = None if args.depth is None else _load_image(args.depth)
  if depth is None:
    depth = live_depth

  workspace_mask = None
  depth_factor_override = None

  if args.workspace_mask:
    import cv2

    mask = cv2.imread(args.workspace_mask, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(args.workspace_mask)

    workspace_mask = (mask > 0).astype(np.uint8)

  if args.meta:
    from scipy.io import loadmat

    meta = loadmat(args.meta)

    K = np.asarray(meta["intrinsic_matrix"], dtype=np.float64)
    depth_factor_override = float(
        np.asarray(meta["factor_depth"]).squeeze()
    )

    engine.intr.width = int(rgb.shape[1])
    engine.intr.height = int(rgb.shape[0])
    engine.intr.fx = float(K[0, 0])
    engine.intr.fy = float(K[1, 1])
    engine.intr.cx = float(K[0, 2])
    engine.intr.cy = float(K[1, 2])

    print(
        f"Loaded camera intrinsics: "
        f"fx={engine.intr.fx:.2f}, fy={engine.intr.fy:.2f}, "
        f"cx={engine.intr.cx:.2f}, cy={engine.intr.cy:.2f}"
    )
    print(f"Loaded depth factor: {depth_factor_override}")

  if workspace_mask is not None:
    print(
        f"Workspace mask: {workspace_mask.shape}, "
        f"retained pixels={np.count_nonzero(workspace_mask)}"
    )

  cloud = None if args.cloud is None else np.load(args.cloud)
  result = engine.propose(rgb=rgb, depth=depth, cloud=cloud, tcp_pose_mm=tcp, joints=joints, kin=kin, top_k=args.top, workspace_mask=workspace_mask, depth_factor_override=depth_factor_override)
  depth_src = "file" if args.depth else ("live" if live_depth is not None else ("cloud" if cloud is not None else "synthetic"))
  print(
    f"backend={result.backend}  grasps={len(result.grasps)}  "
    f"cloud={0 if result.cloud_cam_m is None else len(result.cloud_cam_m)}  "
    f"depth={depth_src}  "
    f"pgea_width=[{engine.gripper.MIN_GAP_MM},{engine.gripper.MAX_GAP_MM}] mm"
  )
  if result.note:
    print(result.note)
  rows = result.as_dicts(k=args.top, cfg=cfg)
  for i, row in enumerate(rows):
    print(
      f"  {i:2d}  score={row['score']:.3f}  width={row['width_mm']:.1f} mm  "
      f"g={row.get('gripper_position', 0):.0f}  "
      f"cam={np.array(row['pose_cam_mm_deg'])[:3].round(1).tolist()}"
      + (f"  tcp={np.array(row['pose_tcp_mm_deg'])[:3].round(1).tolist()}" if "pose_tcp_mm_deg" in row else "")
    )
  if args.out:
    Path(args.out).write_text(json.dumps({"backend": result.backend, "note": result.note, "grasps": rows}, indent=2))
    print(f"wrote {args.out}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
