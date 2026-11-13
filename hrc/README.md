# HRC package (`hrc/`)

CS3AS cell: Dobot arm + DH PGEA-50-26 + SafeSkin + ELP hand-eye camera. Numbers live in [`config/robot.yaml`](../config/robot.yaml).

| Module | Role |
|--------|------|
| `robot.py` | `CS3ASRobot` — TCP takeover, Enable, skins, gripper attach, camera, `query()` |
| `gripper.py` | Tool RS485 Modbus (DHGrip map). **Init fingers on the pendant plugin first** |
| `perception/skins.py` | 125 Hz HIT + analog from Feedback 30004 |
| `perception/camera.py` | ELP `/dev/video0` (not `/dev/video1`) |
| `data/` | Record 8 ms frames → npz, LeRobot parquet, HTML replay |
| `planning/` | cartesian / joint / rrt / rrt_star / apf / learning stub |
| `pipeline.py` | Pick-and-place stages |
| `examples/` | Scripts below |

## Cell bring-up

Pendant: control source **TCP**. SafeSkin plugin on. **DHGrip_v2 → Init** once (TCP will not successfully 0xA5-init the gripper). Post-collision **Pause**, not Stop/Disable.

```bash
python hrc/examples/stream_skins.py --seconds 20
python hrc/examples/stream_skins.py --camera --record data/episodes/skins.npz --lerobot data/lerobot/skins
python hrc/examples/collect_episode.py --seconds 8 --enable --camera --lerobot data/lerobot/demo
# npz = 125 Hz bus; --lerobot = 30 FPS H.264 parquet+mp4 (Hub)
python hrc/examples/view_episodes.py data/lerobot/skins --open
python hrc/examples/view_episodes.py data/lerobot/skins --camera --serve
python hrc/examples/visualize_episode.py data/lerobot/skins --gui --camera
```

`CS3ASRobot.stream_skins()` / `record_skins()` / `query()` are the library entry points. `prepare(enable=True)` so queued `EnableSafeSkin` actually flushes.

## Datasets and visualisation

Native: `data/episodes/*.npz` (125 Hz joints, pose, gripper, skin analog / distance / HIT).

LeRobot **v2.1**: `data/lerobot/<name>/` with `meta/info.json`, parquet, optional hand-eye mp4, and a Hugging Face dataset card. Local player:

```bash
python hrc/examples/view_episodes.py data/episodes --open
python hrc/examples/visualize_episode.py data/episodes/skins.npz --png /tmp/ep.png
```

Upload to the Hub, then open [lerobot/visualize_dataset](https://huggingface.co/spaces/lerobot/visualize_dataset). `YOUR_ORG` is your Hugging Face username or org slug (`huggingface-cli whoami`), not a robot-side setting:

```bash
python hrc/examples/export_dataset.py data/lerobot/skins --fps 30 --push YOUR_ORG/cs3as-hrc
# Hub videos must be H.264 (not OpenCV mp4v). Telemetry is undersampled to 30 FPS.
# https://huggingface.co/spaces/lerobot/visualize_dataset?path=/YOUR_ORG/cs3as-hrc/episode_0
```

## Gripper

TCP Open/Close only after pendant **DHGrip_v2-1-6Stable Init**. Do not tool-power-cycle from Python (`power_cycle: false`). If status is already 1/2/3, `gripper.connect()` attaches Modbus and skips `0xA5`.

## Perception

- SafeSkin I8 flags are 0/1 HIT. Fine-grained values are analog ADC in the reserved 30004 bytes (`analog_max` 1023 → proximity / `distance_mm`).
- Hand-eye: `robot.query().rgb` or `robot.camera.latest()`.
- Oracle obstacles in YAML until RGB-D / LiDAR are enabled.
