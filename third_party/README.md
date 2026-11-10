# Vendored third-party packages

## `dh_gripper_ros`

Imported from https://github.com/DH-Robotics/dh_gripper_ros (master snapshot).

Used here for:

- PGE / PGEA Modbus-RTU register map (`dh_gripper_driver/.../dh_modbus_gripper.cpp`)
- ROS1 driver + `dh_gripper_msgs` if you later run a ROS bridge
- Reference URDFs for other DH grippers (AG95, AG145, DH3, PGC-140)

There is **no PGEA-50-26 URDF** in that repo. The CS3AS cell model is `models/pgea_50_26/`.
