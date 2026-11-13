#!/usr/bin/env python3
"""Replay and analyse recorded CS3AS episodes.

Opens a self-contained HTML player (browser). Optional matplotlib GUI.
Camera mp4s (LeRobot `videos/.../observation.images.*`) play next to the
telemetry by default; `--no-camera` hides them.

  python hrc/examples/view_episodes.py
  python hrc/examples/view_episodes.py data/episodes
  python hrc/examples/view_episodes.py data/lerobot/skins --open
  python hrc/examples/view_episodes.py data/lerobot/skins --camera --serve
  python hrc/examples/view_episodes.py data/episodes/skins.npz --gui
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from hrc.data.analytics import dataset_stats  # noqa: E402
from hrc.data.dataset import default_dataset_dir  # noqa: E402
from hrc.data.viewer import discover_episodes, open_replay, replay_gui, write_replay_html  # noqa: E402


def parse_args():
  parser = argparse.ArgumentParser(description="Replay CS3AS episodes")
  parser.add_argument(
    "path",
    nargs="?",
    default=None,
    help="npz file, episodes dir, or LeRobot v2.1 folder (default: data/episodes)",
  )
  parser.add_argument("--out", default=None, help="HTML path (default: <path>/replay.html)")
  parser.add_argument("--open", action="store_true", help="open the HTML in a browser")
  parser.add_argument("--gui", action="store_true", help="matplotlib play/scrub window")
  parser.add_argument(
    "--camera",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="play hand-eye / other camera mp4s next to joints and skins (default: on)",
  )
  parser.add_argument(
    "--serve",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="HTTP server so the browser can load mp4s (default: on when cameras exist)",
  )
  parser.add_argument("--repo-id", default=None, help="HF dataset id, e.g. your-username/cs3as-hrc")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  src = Path(args.path) if args.path else default_dataset_dir()
  if not src.exists():
    alt = Path(ROOT) / "data" / "lerobot" / "skins"
    if alt.exists():
      src = alt
    else:
      print(f"no episodes at {src}")
      return 2
  episodes = discover_episodes(src)
  if not episodes:
    print(f"no episodes in {src}")
    return 2
  summary = dataset_stats(episodes)
  print(
    f"{src}  episodes={summary['episodes']}  frames={summary['n_frames']}  "
    f"dur={summary['duration_s']}s  HIT={summary['hit_rate']:.1%}"
  )
  for row in summary["per_episode"]:
    cams = row.get("cameras") or []
    print(
      f"  {row['task']:16s}  n={row['n_frames']:5d}  {row['duration_s']:6.2f}s  "
      f"HIT {row['hit_frames']}  analog_max={row['skin_analog']['max']}"
      + (f"  cam={','.join(cams)}" if cams else "")
    )
  has_cam = any(getattr(ep, "videos", None) for ep in episodes)
  if args.camera and not has_cam:
    print("no camera mp4 found; telemetry only. Record with --camera or open data/lerobot/skins")
  if args.gui:
    replay_gui(episodes[0], camera=bool(args.camera))
    return 0
  if args.out:
    out = Path(args.out)
  elif src.is_dir():
    out = src / "replay.html"
  elif has_cam:
    video = Path(next(iter(episodes[0].videos.values())))
    out = src.with_suffix(".html")
    for parent in video.parents:
      if (parent / "meta" / "info.json").is_file():
        out = parent / "replay.html"
        break
  else:
    out = src.with_suffix(".html")
  path = write_replay_html(episodes, out, repo_id=args.repo_id, camera=bool(args.camera))
  print(f"replay HTML -> {path}")
  print("Hugging Face: YOUR_ORG is your HF username or org (`huggingface-cli whoami`)")
  print("  huggingface-cli upload-folder --repo-type dataset YOUR_ORG/cs3as-hrc data/lerobot/skins")
  print("  https://huggingface.co/spaces/lerobot/visualize_dataset?path=/YOUR_ORG/cs3as-hrc/episode_0")
  serve = args.serve if args.serve is not None else bool(args.camera and has_cam)
  if args.open or args.out is None:
    open_replay(path, serve=serve)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
