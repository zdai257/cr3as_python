"""GraspNet-1Billion inference wrapper (proposals only; no training).

If graspnet-baseline / a checkpoint is missing, `backend="geometric"` still
returns PGEA-sized pose guesses so the camera→flange→base chain can be tested.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Union

import numpy as np

from .geometry import (
  CameraIntrinsics,
  T_base_cam_mm,
  T_base_flange_mm,
  depth_to_cloud,
  sample_cloud,
  synthetic_table_depth,
)
from .grasps import GraspProposal, filter_for_pgea, sort_by_score
from .mount import T_flange_cam, mount_as_dict, repo_root
from ...models.pgea import PGEA5026Model


def _section(cfg, *names):
  cur = cfg
  for name in names:
    if cur is None:
      return None
    if hasattr(cur, "get"):
      cur = cur.get(name)
    else:
      cur = getattr(cur, name, None)
  return cur


def _g(section, name, default):
  if section is None:
    return default
  if hasattr(section, "get"):
    value = section.get(name, default)
    return default if value is None else value
  return getattr(section, name, default)


def _resolve_checkpoint(cfg, checkpoint: Optional[Union[str, Path]] = None) -> Path:
  default_ckpt = repo_root() / "weights" / "graspnet" / "checkpoint-rs.tar"
  if checkpoint is not None:
    picked = Path(str(checkpoint)).expanduser()
    return picked if picked.is_absolute() else repo_root() / picked
  gn = _section(cfg, "perception", "graspnet")
  driver = "uvc"
  try:
    from ..cameras.base import active_camera_name

    driver = active_camera_name(cfg)
  except Exception:
    pass
  by = _g(gn, "checkpoint_by_driver", None)
  picked = _g(by, driver, None) if by is not None else None
  if picked:
    picked_path = Path(str(picked)).expanduser()
    return picked_path if picked_path.is_absolute() else repo_root() / picked_path
  raw = _g(gn, "checkpoint", str(default_ckpt))
  picked_path = Path(str(raw)).expanduser()
  return picked_path if picked_path.is_absolute() else repo_root() / picked_path


def _unpack_frame(rgb, depth):
  """Allow propose(rgb=CameraFrame) in addition to raw arrays."""
  if rgb is None or isinstance(rgb, np.ndarray):
    return None, rgb, depth
  if hasattr(rgb, "rgb"):
    frame = rgb
    live = getattr(frame, "depth", None) if depth is None else depth
    return frame, getattr(frame, "rgb", None), live
  return None, rgb, depth


@dataclass
class GraspResult:
  grasps: List[GraspProposal]
  cloud_cam_m: Optional[np.ndarray] = None
  backend: str = "geometric"
  rgb: Optional[np.ndarray] = None
  depth: Optional[np.ndarray] = None
  T_flange_cam_mm: Optional[np.ndarray] = None
  T_base_cam_mm: Optional[np.ndarray] = None
  T_base_flange_mm: Optional[np.ndarray] = None
  note: str = ""
  extra: dict = field(default_factory=dict)

  def top(self, k: int = 10) -> List[GraspProposal]:
    return sort_by_score(self.grasps)[: int(k)]

  def as_dicts(self, k: int = 20, cfg=None) -> List[dict]:
    T_fc = self.T_flange_cam_mm if self.T_flange_cam_mm is not None else np.eye(4)
    T_bc = self.T_base_cam_mm
    rows = []
    for g in self.top(k):
      row = g.as_dict()
      row["pose_flange_mm_deg"] = g.pose_in(T_fc).round(3).tolist()
      if T_bc is not None:
        # GraspNet centre is the jaw centre ≈ PGEA TCP. Command this as a Dobot TCP pose.
        row["pose_base_mm_deg"] = g.pose_in(T_bc).round(3).tolist()
        row["pose_tcp_mm_deg"] = row["pose_base_mm_deg"]
      row["gripper_position"] = round(g.gripper_position_0_1000(), 1)
      rows.append(row)
    return rows


class GraspNetEngine:
  """RGB snapshot (+ depth or cloud) → ranked parallel-jaw grasps for the PGEA-50-26."""

  def __init__(self, cfg=None, *, checkpoint: Optional[Union[str, Path]] = None, device: Optional[str] = None):
    self.cfg = cfg
    gn = _section(cfg, "perception", "graspnet")
    self.num_point = int(_g(gn, "num_point", 20000))
    self.num_view = int(_g(gn, "num_view", 300))
    self.collision_thresh = float(_g(gn, "collision_thresh", 0.01))
    self.voxel_size = float(_g(gn, "voxel_size", 0.01))
    self.top_k = int(_g(gn, "top_k", 50))
    self.min_score = float(_g(gn, "min_score", 0.05))
    self.depth_factor = float(_g(gn, "depth_factor", 1.0))
    self.checkpoint = _resolve_checkpoint(cfg, checkpoint)
    self.device_name = device or str(_g(gn, "device", "cpu"))
    self.intr = CameraIntrinsics.from_cfg(cfg)
    self.gripper = PGEA5026Model(_section(cfg, "gripper"))
    self._net = None
    self._net_error: Optional[str] = None

  def available(self) -> bool:
    return self.checkpoint.is_file() and self._try_import() is not None

  def _try_import(self):
    gn = _section(self.cfg, "perception", "graspnet")
    configured_root = _g(gn, "package_root", "third_party/graspnet-baseline")
    package_root = Path(str(configured_root)).expanduser()
    root = package_root if package_root.is_absolute() else repo_root() / package_root

    if root.is_dir():
      extras = [
        root,
        root / "models",
        root / "utils",
        root / "dataset",
        root / "pointnet2",
        root / "knn",
      ]
      for build_dir in (*root.glob("*/build/lib*"), *root.glob("*/build")):
        extras.append(build_dir)
      for extra in extras:
        s = str(extra)
        if extra.is_dir() and s not in sys.path:
          sys.path.insert(0, s)
    else:
      self._net_error = f"graspnet-baseline package_root not found: {root}"
      return None

    try:
      from graspnet import GraspNet, pred_decode  # type: ignore

      return GraspNet, pred_decode
    except Exception as graspnet_error:
      try:
        from models.graspnet import GraspNet, pred_decode  # type: ignore

        return GraspNet, pred_decode
      except Exception as models_error:
        self._net_error = (
          f"cannot import graspnet-baseline from {root}: "
          f"graspnet={graspnet_error}; models.graspnet={models_error}"
        )
        return None

  def load(self) -> bool:
    mods = self._try_import()
    if mods is None or not self.checkpoint.is_file():
      return False
    if self._net is not None:
      return True
    GraspNet, _pred = mods
    try:
      import torch
    except ImportError as exc:
      self._net_error = str(exc)
      return False
    net = GraspNet(
      input_feature_dim=0,
      num_view=self.num_view,
      num_angle=12,
      num_depth=4,
      cylinder_radius=0.05,
      hmin=-0.02,
      hmax_list=[0.01, 0.02, 0.03, 0.04],
      is_training=False,
    )
    device = torch.device(self.device_name if torch.cuda.is_available() or self.device_name == "cpu" else "cpu")
    if self.device_name.startswith("cuda") and not torch.cuda.is_available():
      device = torch.device("cpu")
    net.to(device)
    ckpt = torch.load(str(self.checkpoint), map_location=device)
    state = ckpt["model_state_dict"] if isinstance(ckpt, dict) and "model_state_dict" in ckpt else ckpt
    net.load_state_dict(state)
    net.eval()
    self._net = net
    self._device = device
    self._pred_decode = _pred
    return True

  def apply_frame_intrinsics(self, frame) -> None:
    if frame is None:
      return
    self.intr.overlay_frame(frame)

  def propose_from_camera(self, camera, depth=None, cloud=None, **kwargs) -> GraspResult:
    rgb, live_depth, frame = rgbd_from_camera(camera)
    self.apply_frame_intrinsics(frame)
    if depth is None:
      depth = live_depth
    return self.propose(rgb=rgb, depth=depth, cloud=cloud, **kwargs)

  def propose(
    self,
    rgb: Optional[np.ndarray] = None,
    depth: Optional[np.ndarray] = None,
    cloud: Optional[np.ndarray] = None,
    *,
    tcp_pose_mm: Optional[Sequence[float]] = None,
    joints: Optional[Sequence[float]] = None,
    kin=None,
    top_k: Optional[int] = None,
    allow_geometric: bool = True,
    depth_factor_override: Optional[float] = None,
    workspace_mask: Optional[np.ndarray] = None,
  ) -> GraspResult:
    frame, rgb, depth = _unpack_frame(rgb, depth)
    self.apply_frame_intrinsics(frame)
    rgb_arr = None if rgb is None else np.asarray(rgb)
    depth_arr = depth
    if depth_arr is None and cloud is None:
      depth_arr = synthetic_table_depth(self.intr)
    colors = None
    if cloud is None:
      cloud, colors = depth_to_cloud(
        np.asarray(depth_arr),
        self.intr,
        rgb=rgb_arr,
        factor=(
                float(depth_factor_override)
                if depth_factor_override is not None
                else (
                      self.depth_factor
                      if np.asarray(depth_arr).max() < 20
                      else 1000.0
                      )
                ),
        workspace_mask=workspace_mask,
      )
    cloud = np.asarray(cloud, dtype=np.float32).reshape(-1, 3)
    sampled, _ = sample_cloud(cloud, self.num_point, colors)

    T_fc = T_flange_cam(self.cfg)
    T_bf = T_base_flange_mm(tcp_pose_mm=tcp_pose_mm, cfg=self.cfg, joints=joints, kin=kin)
    T_bc = T_bf @ T_fc

    grasps: List[GraspProposal] = []
    backend = "geometric"
    note = ""
    if self.load():
      try:
        grasps = self._infer_graspnet(sampled)
        backend = "graspnet"
      except Exception as exc:
        note = f"GraspNet forward failed ({exc}); geometric fallback"
        if not allow_geometric:
          raise
        grasps = geometric_grasps(sampled, gripper=self.gripper)
    else:
      note = self._net_error or f"no checkpoint at {self.checkpoint}"
      if not allow_geometric:
        raise RuntimeError(note)
      grasps = geometric_grasps(sampled, gripper=self.gripper)
      note = "geometric fallback (install graspnet-baseline + a GraspNet checkpoint for the real model). " + note

    # grasps = filter_for_pgea(grasps, min_score=self.min_score, model=self.gripper)
    
    grasps = sort_by_score(grasps)[: int(top_k or self.top_k)]
    return GraspResult(
      grasps=grasps,
      cloud_cam_m=cloud,
      backend=backend,
      rgb=rgb_arr,
      depth=None if depth_arr is None else np.asarray(depth_arr),
      T_flange_cam_mm=T_fc,
      T_base_cam_mm=T_bc,
      T_base_flange_mm=T_bf,
      note=note,
      extra={"mount": mount_as_dict(self.cfg), "n_cloud": int(cloud.shape[0])},
    )

  def _infer_graspnet(self, cloud_sampled: np.ndarray) -> List[GraspProposal]:
    import torch

    end_points = {
      "point_clouds": torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(self._device)
    }
    with torch.no_grad():
      end_points = self._net(end_points)
      preds = self._pred_decode(end_points)
    rows = preds[0].detach().cpu().numpy()
    grasps = [GraspProposal.from_graspnet_row(row, source="graspnet") for row in rows]
    if self.collision_thresh > 0 and cloud_sampled.shape[0] > 32:
      grasps = self._collision_filter(grasps, cloud_sampled)
    return grasps

  def _collision_filter(self, grasps: List[GraspProposal], cloud: np.ndarray) -> List[GraspProposal]:
    try:
      from collision_detector import ModelFreeCollisionDetector  # type: ignore
      from graspnetAPI import GraspGroup  # type: ignore
    except Exception:
      return grasps
    arr = np.stack(
      [
        np.concatenate(
          [
            [g.score, g.width_m, g.height_m, g.depth_m],
            np.asarray(g.rotation_cam).reshape(9),
            np.asarray(g.center_cam_m).reshape(3),
            [g.object_id],
          ]
        )
        for g in grasps
      ]
    ) if grasps else np.zeros((0, 17))
    gg = GraspGroup(arr)
    det = ModelFreeCollisionDetector(np.asarray(cloud), voxel_size=self.voxel_size)
    mask = det.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
    return [g for g, hit in zip(grasps, mask) if not hit]


def geometric_grasps(cloud: np.ndarray, *, n: int = 8, gripper: Optional[PGEA5026Model] = None) -> List[GraspProposal]:
  """Top-down parallel-jaw guesses at the cloud centroid (no neural net)."""
  gripper = gripper or PGEA5026Model()
  if cloud.shape[0] < 8:
    c = np.array([0.0, 0.0, 0.35], dtype=float)
  else:
    z = cloud[:, 2]
    band = cloud[(z > np.percentile(z, 15)) & (z < np.percentile(z, 70))]
    c = (band if band.shape[0] else cloud).mean(axis=0)
  width = 0.5 * (gripper.MIN_GAP_MM + gripper.MAX_GAP_MM) / 1000.0
  grasps = []
  for i, yaw in enumerate(np.linspace(0.0, np.pi, n, endpoint=False)):
    cy, sy = np.cos(yaw), np.sin(yaw)
    # Approach along camera +Z; fingers open along a yawed X.
    R = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    grasps.append(
      GraspProposal(
        score=float(0.4 + 0.05 * np.cos(yaw)),
        width_m=float(width),
        height_m=0.02,
        depth_m=0.02,
        rotation_cam=R,
        center_cam_m=c.astype(float),
        source="geometric",
        extra={"yaw_rad": float(yaw), "index": i},
      )
    )
  return grasps


def _take_frame(camera):
  if camera is None:
    return None
  frame = camera.latest()
  if frame is None:
    if not getattr(camera, "is_open", False):
      camera.open()
    frame = camera.read()
  return frame


def snapshot_from_camera(camera, **kwargs) -> Optional[np.ndarray]:
  frame = _take_frame(camera)
  if frame is None:
    return None
  return np.asarray(frame.rgb)


def rgbd_from_camera(camera):
  """Return (rgb, depth_metres_or_None, frame) from the active camera."""
  frame = _take_frame(camera)
  if frame is None:
    return None, None, None
  depth = getattr(frame, "depth", None)
  return np.asarray(frame.rgb), None if depth is None else np.asarray(depth), frame
