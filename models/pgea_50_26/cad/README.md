# Official PGEA-50-26 CAD

DH-Robotics publishes STEP + 2D drawings as **DH-PGEA_3D Model and 2D Drawing** on the product page (account / form gated):

- https://en.dh-robotics.com/product/pgea
- Catalog: https://en.dh-robotics.com/wp-content/uploads/2025/06/DH_PGEAPGIA-catalog_V255.pdf

Drop the official STEP here as `PGEA-50-26.step` if you obtain it. The URDF in `../urdf/` already matches the published envelope (118 x 55 x 29 mm) and the CS3AS tool offset of **z = 118 mm**.

The ROS driver / messages used to talk to this gripper live in `third_party/dh_gripper_ros` (https://github.com/DH-Robotics/dh_gripper_ros). That repo ships URDFs for AG95 / AG145 / DH3 / PGC-140, not PGEA-50-26 — hence the datasheet model here.
