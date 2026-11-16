# CS3AS wrist camera holder (`cr3_camera_half_clear.step`)

3D-printed clamp for the ELP **06MT-KL170IR** (UVC `/dev/video0`) on the CS3AS wrist, next to the DH PGEA-50-26.

Software camera backends (UVC / RealSense D435i / Azure Kinect) are selected in `config/robot.yaml` `perception.camera`. **This CAD is still the ELP wrist clamp** — recalibrate `t_flange_cam_mm` / `rpy_flange_cam_deg` if you mount a D435i or Kinect.

CAD is **nominal**. Treat `config/robot.yaml` `perception.camera_mount` as a starting guess until a proper eye-in-hand calibration replaces it (`calibrated: false`).

## What the STEP contains

FreeCAD / Open CASCADE export. Units **mm**. Parsed collar / holes:

| Feature | CAD number |
|---------|------------|
| Wrist-clamp inner radius | **41.1 mm** (Ø **82.2 mm** J6 housing) |
| Wrist-clamp outer radius | **45.1 mm** |
| Collar axis (CAD) | through `(-57.887, 0, z)`, along CAD **+Z** |
| Collar thickness along CAD Z | ~**27 mm** |
| Camera M3 holes | Ø 3.3 mm (`r=1.65`), **20 mm** apart in CAD Y (`y = ±10`) |
| Hole-plane tilt | **5°** about CAD Y: plane normal `(-sin 5°, 0, cos 5°)` |

The inner cylinder also appears extruded to `z = -164 mm` in the STEP; that is a construction bore, not a 164 mm-long physical part.

## Mapping CAD → flange (nominal)

Assumption used in software (replace after calibration):

- CAD **+Z** = flange **+Z** (gripper approach / PGEA TCP +Z).
- Wrist axis in CAD is shifted to the flange origin by **+57.887 mm** in CAD X.
- CAD `z = 0` is the flange XY datum (collar sits on the wrist, camera boom is at **+Z**).
- Optical axis = hole-plane normal (looks along the gripper, **5° inward** toward the wrist axis — a “look at the fingertips” mount).
- Optical origin = midpoint of the two M3 holes, plus **8 mm** along the optical axis for board + M12 lens (TBC).

That yields the numbers in `config/robot.yaml`:

```
t_flange_cam_mm: [91.66, 0.0, 57.07]
rpy_flange_cam_deg: [0.0, -5.0, 0.0]   # Dobot intrinsic XYZ; Ry = -5°
```

Camera OpenCV frame: X right, Y down, Z forward. With this RPY, camera +Z ≈ flange +Z.

## Camera (06MT-KL170IR)

- OV2710 1/2.7", 1920×1080 @ 30 FPS MJPEG, ~**170°** fisheye, board **32×32 / 38×38 mm**, ~30 g.
- RGB only. GraspNet still needs **depth or a point cloud** (file, RGB-D camera, or a depth network). See [`hrc/perception/graspnet/README.md`](../../hrc/perception/graspnet/README.md).

## PGEA-50-26 (for scale)

Gripper envelope 118 × 55 × 29 mm, TCP **z = 118 mm** from the flange, jaw gap **1.2–27.2 mm**. The camera sits beside that body, ~92 mm off the wrist axis and ~57 mm in front of the flange datum — behind the fingertips.
