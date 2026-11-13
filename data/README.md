# Recorded episodes

| Path | Format |
|------|--------|
| `episodes/*.npz` | Native 125 Hz frames (+ sidecar `.json`) |
| `lerobot/<name>/` | LeRobot v2.1 parquet + `meta/` (+ optional hand-eye mp4) |

```bash
python hrc/examples/view_episodes.py data/episodes --open
python hrc/examples/view_episodes.py data/lerobot/skins --open
```

`replay.html` in a folder is a self-contained player (Play / scrub / HIT overlay). The LeRobot folder also has a Hugging Face dataset card (`README.md`) so you can push it and use [lerobot/visualize_dataset](https://huggingface.co/spaces/lerobot/visualize_dataset).
