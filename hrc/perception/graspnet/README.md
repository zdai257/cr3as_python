# GraspNet inference (CS3AS wrist camera)

Inference-only wrapper around [graspnet-baseline](https://github.com/graspnet/graspnet-baseline) (CVPR 2020 GraspNet-1Billion). This package does **not** train or modify the model. It turns one RGB snapshot (+ depth or a point cloud) into parallel-jaw pose proposals, then expresses them in the **flange / TCP / base** frames of the Dobot CS3AS + DH **PGEA-50-26**.

Cameras are selected in `config/robot.yaml` (`perception.camera`: `uvc` | `realsense` | `k4a`). The ELP **06MT-KL170IR** is one **UVC RGB** example. Intel **RealSense D435i** and **Azure Kinect DK** provide live aligned RGB-D, which GraspNet consumes directly (no synthetic table when depth is present).

## Cell geometry (nominal)

| Frame | Notes |
|-------|--------|
| Flange | CS3AS tool flange. PGEA TCP is **118 mm** along +Z. |
| Camera | 3D-printed clamp [`models/camera_holder/cr3_camera_half_clear.step`](../../../models/camera_holder/README.md) is the **ELP** wrist CAD. Optical axis ≈ flange +Z, **5° inward**. `t_flange_cam_mm ≈ [91.66, 0, 57.07]`. **Recalibrate when swapping a D435i or Kinect.** **`calibrated: false` until you run an eye-in-hand calibration.** |
| Gripper | Jaw gap **1.2–27.2 mm**. GraspNet widths outside that range are dropped. |

06MT-KL170IR (UVC example): OV2710, 1920×1080 @ 30 FPS MJPEG, ~170° fisheye. Pinhole `fx/fy` in `config/robot.yaml` is a placeholder — calibrate (or undistort the fisheye) before trusting metric poses. D435i / Kinect overwrite `fx/fy/cx/cy` from the live SDK when the device opens.

## Weights (you download these)

Do **not** commit the tarballs. Put them in `weights/graspnet/`.

`perception.graspnet.checkpoint_by_driver` picks RealSense vs Kinect weights from the active camera driver unless you pass an explicit checkpoint:

| File | Typical camera | Google Drive | Baidu Pan |
|------|----------------|-----------|-----------|
| `checkpoint-rs.tar` (RealSense, also the UVC default) | D435i / UVC | [drive](https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view?usp=sharing) | [pan](https://pan.baidu.com/s/1Eme60l39tTZrilF0I86R5A) |
| `checkpoint-kn.tar` (Kinect) | Azure Kinect DK | [drive](https://drive.google.com/file/d/1vK-d0yxwyJwXHYWOtH1bDMoe--uZ2oLX/view?usp=sharing) | [pan](https://pan.baidu.com/s/1QpYzzyID-aG5CgHjPFNB9g) |

```bash
mkdir -p weights/graspnet
# after download:
ls weights/graspnet/checkpoint-rs.tar
```

Point `perception.graspnet.checkpoint` at that file if you store it somewhere else.

## Install graspnet-baseline (optional, for the real network)

PyTorch + compiled PointNet++ / KNN ops. CPU works; CUDA is much faster.

```bash
git clone https://github.com/graspnet/graspnet-baseline.git third_party/graspnet-baseline
cd third_party/graspnet-baseline
pip install -r requirements.txt
cd pointnet2 && python setup.py install && cd ..
cd knn && python setup.py install && cd ..
pip install git+https://github.com/graspnet/graspnetAPI.git
```

RGB-D camera SDKs are optional extras, not package requirements:

```bash
pip install pyrealsense2   # Intel RealSense D435i
pip install pyk4a          # Azure Kinect DK (+ Azure Kinect SDK)
```

Without graspnet-baseline / without a checkpoint, `GraspNetEngine` still runs a **geometric** top-down fallback so the transform chain and PGEA width filter can be developed offline.

## Run

```bash
# Synthetic depth (no robot, no weights) — UVC / RGB-only
python hrc/examples/grasp_snapshot.py

# Live frame from perception.camera (UVC synthesizes depth; D435i/Kinect use live depth)
python hrc/examples/grasp_snapshot.py --camera
python hrc/examples/grasp_snapshot.py --camera --driver realsense
python hrc/examples/grasp_snapshot.py --camera --driver k4a

# RGB-D files (depth in millimetres or metres)
python hrc/examples/grasp_snapshot.py --rgb shot.png --depth depth.png

# Real GraspNet
python hrc/examples/grasp_snapshot.py --rgb shot.png --depth depth.png \
  --checkpoint weights/graspnet/checkpoint-rs.tar --top 20
```

Library:

```python
from hrc.perception.camera import make_camera
from hrc.perception.graspnet import GraspNetEngine
engine = GraspNetEngine(cfg)          # cfg from load_robot_config()
result = engine.propose_from_camera(make_camera(cfg), tcp_pose_mm=robot.current_pose())
for g in result.top(10):
    print(g.score, g.width_mm, g.pose_in(result.T_base_cam_mm))
```

`result.as_dicts()` includes `pose_flange_mm_deg` and `pose_tcp_mm_deg` (Dobot `[x,y,z,rx,ry,rz]`). GraspNet’s grasp centre is treated as the **jaw centre ≈ PGEA TCP**. Confirm with a calibration board before commanding motion.

## Depth sources

1. Live aligned RGB-D from RealSense (`perception.camera: realsense`) or Azure Kinect (`k4a`). Depth on `CameraFrame` is **metres**.
2. `--depth` PNG / NPY (uint16 mm or float metres).
3. Organized / Nx3 cloud `--cloud cloud.npy` (metres, camera frame).
4. Otherwise a synthetic table at ~0.42 m so the pipeline is runnable on a laptop (RGB-only UVC).

A 170° RGB fisheye cannot invent metric depth. For real grasps from UVC, add a depth camera, a calibrated stereo pair, or an external depth estimator; this wrapper will consume whatever cloud you pass.
