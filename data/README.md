# Recorded episodes

| Path | Format |
|------|--------|
| `episodes/*.npz` | Native 125 Hz frames (+ sidecar `.json`) |
| `lerobot/<name>/` | LeRobot v2.1 parquet + `meta/` (+ optional hand-eye mp4) |

```bash
python hrc/examples/collect_episode.py --seconds 8 --enable --camera --lerobot data/lerobot/demo
python hrc/examples/view_episodes.py data/episodes --open
python hrc/examples/view_episodes.py data/lerobot/skins --camera --serve
python hrc/examples/visualize_episode.py data/lerobot/skins --gui --camera
```

`--camera` (default on) plays `videos/**/observation.images.handeye/*.mp4` next to joints / SafeSkin. `--no-camera` hides footage. `--serve` starts a local HTTP server so the browser can actually load the mp4.

LeRobot / Hugging Face export is **30 FPS H.264** (`avc1`). Joints and SafeSkin are undersampled onto the camera clock. Native `episodes/*.npz` stays at the 125 Hz bus.

```bash
python hrc/examples/export_dataset.py data/lerobot/skins --lerobot data/lerobot/skins --fps 30 --push zdai257/cs3as-hrc
```

`replay.html` in a folder is a self-contained player (Play / scrub / HIT overlay / camera). The LeRobot folder also has a Hugging Face dataset card (`README.md`) so you can push it and use [lerobot/visualize_dataset](https://huggingface.co/spaces/lerobot/visualize_dataset).

```bash
# YOUR_ORG = Hugging Face username or org slug (`huggingface-cli whoami`)
huggingface-cli upload-folder --repo-type dataset YOUR_ORG/cs3as-hrc data/lerobot/skins
```

Then open `https://huggingface.co/spaces/lerobot/visualize_dataset?path=/YOUR_ORG/cs3as-hrc/episode_0` (or paste `YOUR_ORG/cs3as-hrc` in the space).
