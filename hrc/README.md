# HRC research package

Human-robot collaboration experiments on a Dobot **CS3AS** with a DH Robotics **PGEA-50-26** gripper and CS3AS safe skins.

High-level numbers live in [`config/robot.yaml`](../config/robot.yaml) (IP `192.168.5.1`, payload, Home joints, TBC waypoints, oracle obstacles, planner choice).

## Layout

```
hrc/
  robot.py              CS3ASRobot(DobotRobot) — arm + gripper + skins + IK
  gripper.py            PGEA-50-26 via tool RS485 Modbus RTU
  models/               PGEA-50-26 URDF / TCP / collision capsules
  pipeline.py           pick-and-place stages, planner-agnostic
  world.py              Observation / WorldState
  planning/             cartesian_linear, joint_linear, rrt, rrt_star, apf, learning
  perception/           oracle, skins (30004), rgbd / lidar hooks
  data/                 episode recorder, LeRobot export, rosbag2, plots
  utils/                dashboard parse, FK/IK, rotations, capsules
  examples/
```

## Pick-and-place

```bash
# plan-only against the oracle moving forearm (no motion)
python hrc/examples/pick_and_place.py
python hrc/examples/pick_and_place.py --planner rrt --compare

# after teaching waypoints in config/robot.yaml
python hrc/examples/pick_and_place.py --execute
```

Stages: Home `[0,0,0,0,0,0]` → pre_pick → pick (close) → post_pick → pre_place → place (open) → post_place → Home.

Until those Cartesian poses are taught (`null` in the YAML), the script plans `demo_start → demo_goal` around the oracle obstacles so planners can be scored without moving the arm.

## Adding a planner

1. Subclass `PathPlanner` in `hrc/planning/`.
2. Decorate with `@register("my_name")`.
3. Implement `plan(start, goal, world, ...) -> Trajectory`.
4. Select it with `--planner my_name` or `hrc.planner` in YAML.

`learning` is a stub: subclass `LearningPlanner` and implement `load()` / `infer(Observation)`. `Observation.as_policy_obs()` is the intended VLA / policy input (joints, pose, gripper, obstacles, rgb, depth, pointcloud, skin approach / proximity / conductance).

## Gripper model

URDF + meshes: `models/pgea_50_26/` (118 x 55 x 29 mm, TCP z = 118 mm). Driver reference: `third_party/dh_gripper_ros`. `CS3ASRobot.prepare()` calls `SetTool` / `Tool` with that offset.

## SafeSkin (port 30004)

`SafeSkinFrontend.read()` returns a `SkinReading` parsed from the 8 ms Feedback packet: I8 approach flags on forearm / J4 / J5 / J6 plus analog channels decoded from the reserved bytes after offset 1042.

```bash
python hrc/examples/stream_skins.py --seconds 20
```

## Datasets for learned policies

```bash
python hrc/examples/collect_episode.py --seconds 8 --lerobot data/lerobot/demo --ros2 data/ros2/demo
python hrc/examples/export_dataset.py --synthetic --lerobot data/lerobot/synth
python hrc/examples/visualize_episode.py data/episodes/episode_0000.npz --png /tmp/ep.png
```

Native store is `data/episodes/*.npz`. LeRobot v2.1 layout is `meta/info.json` + parquet (JSONL if pyarrow is missing). rosbag2 sqlite3 uses CDR `Float64MultiArray` on `/cs3as/*`.

## Perception hooks

| Source | Status |
|--------|--------|
| Oracle obstacles + constant velocity | used now |
| SafeSkin approach + analog from Feedback 30004 | `SkinReading` |
| RGB-D / LiDAR | disabled stubs; set `perception.rgbd_enabled` when wired |

## Kinematics

`Kinematics.fk` / local DLS IK use the TBC DH table in YAML for collision checks. `CS3ASRobot.inverse_kin` / `forward_kin` call controller `InverseKin` / `PositiveKin`. Replace the DH numbers from a URDF or measurement when you have them.
