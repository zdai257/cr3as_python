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
- frames: 1408
- fps: 125.0
- tasks: safeskin
- robot: `dobot_cs3as_pgea`

Parquet columns include `observation.state`, `observation.skins`, `observation.skin_analog`, `observation.skin_distance`, `observation.skin_approach`, and `action`. Hand-eye video (if recorded) is `observation.images.handeye`.

## Visualize

Local player (no Hub upload):

```bash
python hrc/examples/view_episodes.py data/lerobot/skins
```

Hugging Face [LeRobot dataset visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset) after `huggingface-cli upload`:

```bash
huggingface-cli upload-folder --repo-type dataset YOUR_ORG/cs3as-hrc data/lerobot/skins
```

Then open `https://huggingface.co/spaces/lerobot/visualize_dataset?path=YOUR_ORG/cs3as-hrc/0`.

`meta/info.json` follows LeRobot **v2.1** (`data_path` / `video_path` templates). Convert to v3 with `python -m lerobot.scripts.convert_dataset_v21_to_v30` if a newer toolchain requires it.
