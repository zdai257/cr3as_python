"""Live and post-hoc visualisation of joints, gripper, and SafeSkin."""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence, Union

import numpy as np

from ..perception.skins import SKIN_PARTS, SkinReading
from .dataset import Episode


def ascii_bar(value: float, width: int = 16) -> str:
  v = float(np.clip(value, 0.0, 1.0))
  filled = int(round(v * width))
  return "█" * filled + "░" * (width - filled)


def format_skin(reading: SkinReading) -> str:
  lines = [f"SafeSkin t={reading.timestamp_ms} ms"]
  prox = reading.proximity
  dist = reading.distance_mm()
  for i, name in enumerate(SKIN_PARTS):
    p = float(prox[i]) if i < prox.size else 0.0
    a = float(reading.approach[i]) if i < reading.approach.size else 0.0
    d = float(dist[i]) if i < dist.size else 0.0
    flag = "HIT" if abs(a) > 0 else "   "
    lines.append(f"  {name:8s} {ascii_bar(p)}  prox={p:5.2f}  d~{d:5.0f}mm  appr={a:.0f} {flag}")
  return "\n".join(lines)


def format_frame_line(joints, pose, grip, prox) -> str:
  j = " ".join(f"{v:7.1f}" for v in np.asarray(joints, dtype=float)[:6])
  p = " ".join(f"{v:7.1f}" for v in np.asarray(pose, dtype=float)[:3])
  s = " ".join(f"{v:4.2f}" for v in np.asarray(prox, dtype=float)[:4])
  return f"q[{j}]  xyz[{p}]  g={float(grip):6.1f}  skin[{s}]"


def plot_episode(episode: Episode, out: Optional[Union[str, Path]] = None, show: bool = False):
  """3-panel matplotlib figure: joints, skins, gripper. Optional save path."""
  try:
    import matplotlib.pyplot as plt
  except ImportError as exc:
    raise RuntimeError("matplotlib is required for plot_episode (pip install matplotlib)") from exc

  t = episode.timestamps()
  if t.size:
    t = t - t[0]
  joints = np.stack([f.joints for f in episode.frames]) if episode.frames else np.zeros((0, 6))
  prox = episode.skin_proximity()
  grip = np.asarray([f.gripper_position for f in episode.frames], dtype=float)

  fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
  for i in range(joints.shape[1] if joints.ndim == 2 else 0):
    axes[0].plot(t, joints[:, i], label=f"J{i+1}")
  axes[0].set_ylabel("joint deg")
  axes[0].legend(loc="upper right", ncol=6, fontsize=8)
  axes[0].set_title(f"{episode.task}  n={len(episode)}  fps={episode.fps}")

  for i, name in enumerate(SKIN_PARTS):
    if prox.shape[1] > i:
      axes[1].plot(t, prox[:, i], label=name)
  axes[1].set_ylabel("skin proximity")
  axes[1].set_ylim(-0.05, 1.05)
  axes[1].legend(loc="upper right", ncol=4, fontsize=8)

  axes[2].plot(t, grip, label="gripper pos", color="tab:purple")
  axes[2].set_ylabel("gripper 0-1000")
  axes[2].set_xlabel("time s")
  axes[2].legend(loc="upper right", fontsize=8)
  fig.tight_layout()
  if out is not None:
    fig.savefig(out, dpi=120)
  if show:
    plt.show()
  else:
    plt.close(fig)
  return fig


class LivePrinter:
  """Print a compact skin + joint line on each 30004 sample."""

  def __init__(self, every: int = 10):
    self.every = max(1, int(every))
    self.n = 0

  def __call__(self, reading: Optional[SkinReading], joints=None, pose=None, grip=None) -> None:
    self.n += 1
    if self.n % self.every:
      return
    if reading is None:
      print("[skins] no 30004 sample yet")
      return
    print(format_skin(reading))
    if joints is not None:
      print(
        format_frame_line(
          joints,
          pose if pose is not None else np.zeros(6),
          0.0 if grip is None else grip,
          reading.proximity,
        )
      )
