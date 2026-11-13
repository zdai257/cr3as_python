---
license: mit
task_categories:
- robotics
tags:
- LeRobot
- robotics
- dobot
- cs3as
- safeskin
configs:
- config_name: default
  data_files: data/*/*.parquet
---

# CS3AS HRC episodes (LeRobot v2.1)

- episodes: 1
- frames: 345
- fps: 30
- tasks: safeskin
- robot: `dobot_cs3as_pgea`

Parquet columns include `observation.state`, `observation.skins`, `observation.skin_analog`, `observation.skin_distance`, `observation.skin_approach`, and `action`. Hand-eye video (if recorded) is `observation.images.handeye` (H.264 / `avc1` at 30 FPS). Native `data/episodes/*.npz` stays at the 125 Hz robot bus; this Hub folder is undersampled to the camera clock so the LeRobot visualizer can play image + state together.

## Visualize

Local player (cameras play next to joints / SafeSkin; `--serve` so the browser can load mp4s):

```bash
python hrc/examples/view_episodes.py data/lerobot/skins --camera --serve
python hrc/examples/visualize_episode.py data/lerobot/skins --gui --camera
```

Hugging Face [LeRobot dataset visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset) after upload.

`YOUR_ORG` is **not** a CS3AS constant. It is your Hugging Face **username** (personal account) or **organization slug**. Print it with:

```bash
huggingface-cli whoami
# or:  hf auth whoami
```

Use the `name` field for a personal dataset (`alice/cs3as-hrc`). Use an org you can write to for a shared dataset (`my-lab/cs3as-hrc`).

```bash
huggingface-cli upload-folder --repo-type dataset YOUR_ORG/cs3as-hrc data/lerobot/skins
```

Then paste `YOUR_ORG/cs3as-hrc` into the visualizer, or open:

`https://huggingface.co/spaces/lerobot/visualize_dataset?path=/YOUR_ORG/cs3as-hrc/episode_0`

`meta/info.json` follows LeRobot **v2.1** (`data_path` / `video_path` templates). Convert to v3 with `python -m lerobot.scripts.convert_dataset_v21_to_v30` if a newer toolchain requires it.
