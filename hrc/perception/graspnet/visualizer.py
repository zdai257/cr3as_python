
"""
GraspNet 3D grasp proposal visualisation.

Projects each GraspProposal's 3D parallel-jaw geometry
from the camera frame onto the original RGB image.

Input:
    RGB image (H,W,3), RGB uint8
    List[GraspProposal]
    Camera intrinsics (fx, fy, cx, cy)

Output:
    RGB overlay image saved as PNG.
"""

from pathlib import Path

import cv2
import numpy as np


def _project(points_cam, intrinsics):
    """
    Project Nx3 camera-frame points into image coordinates.

    Camera convention:
        X: right
        Y: down
        Z: forward
    """
    points_cam = np.asarray(points_cam, dtype=np.float64)

    z = points_cam[:, 2]

    valid = z > 1e-5

    uv = np.full((len(points_cam), 2), np.nan)

    uv[valid, 0] = (
        intrinsics.fx * points_cam[valid, 0] / z[valid]
        + intrinsics.cx
    )

    uv[valid, 1] = (
        intrinsics.fy * points_cam[valid, 1] / z[valid]
        + intrinsics.cy
    )

    return uv, valid


def _draw_segment(image, p1, p2, color, thickness=2):
    """Draw a projected segment, skipping points behind camera."""

    if not (np.isfinite(p1).all() and np.isfinite(p2).all()):
        return

    h, w = image.shape[:2]

    p1 = tuple(np.round(p1).astype(int))
    p2 = tuple(np.round(p2).astype(int))

    # Clip segments to the image boundary.
    ok, q1, q2 = cv2.clipLine(
        (0, 0, w, h), p1, p2
    )

    if ok:
        cv2.line(
            image, q1, q2,
            color, thickness,
            cv2.LINE_AA
        )


def draw_grasp(
    image,
    grasp,
    intrinsics,
    rank=0,
    thickness=2,
):
    """
    Draw one 3D parallel-jaw grasp proposal.

    The grasp geometry is schematic, using the GraspNet
    local grasp frame:

        X: gripper opening direction
        Y: finger thickness/height direction
        Z: finger depth direction

    All geometry is transformed into the camera frame
    before projection.
    """

    R = np.asarray(
        grasp.rotation_cam, dtype=np.float64
    ).reshape(3, 3)

    center = np.asarray(
        grasp.center_cam_m, dtype=np.float64
    ).reshape(3)

    width = float(grasp.width_m)
    height = float(grasp.height_m)
    depth = float(grasp.depth_m)

    # Avoid degenerate geometry.
    height = max(height, 0.005)
    depth = max(depth, 0.01)

    # Local gripper geometry.
    #
    # Two fingers extend along local Z.
    # Their separation is the predicted gripper width.
    #
    # The bridge at the rear indicates the palm.

    x = width / 2.0
    y = height / 2.0
    z0 = -depth / 2.0
    z1 = depth / 2.0

    local = np.array([
        [-x, -y, z0],  # 0
        [-x,  y, z0],  # 1
        [ x, -y, z0],  # 2
        [ x,  y, z0],  # 3

        [-x, -y, z1],  # 4
        [-x,  y, z1],  # 5
        [ x, -y, z1],  # 6
        [ x,  y, z1],  # 7
    ])

    # Transform grasp-local points into camera coordinates.
    points_cam = (R @ local.T).T + center

    uv, valid = _project(points_cam, intrinsics)

    # Higher score -> brighter green.
    score = float(np.clip(grasp.score, 0.0, 1.0))

    color = (
        int(40 + 180 * (1.0 - score)),  # B
        int(100 + 155 * score),         # G
        30,                             # R
    )

    # Finger rails.
    edges = [
        (0, 4), (1, 5),
        (2, 6), (3, 7),

        # Rear palm bridge.
        (0, 2), (1, 3),

        # Finger tips.
        (4, 5), (6, 7),

        # Thickness edges at rear and tips.
        (0, 1), (2, 3),
        (4, 5), (6, 7),
    ]

    for i, j in edges:
        if valid[i] and valid[j]:
            _draw_segment(
                image,
                uv[i],
                uv[j],
                color,
                thickness
            )

    # Project the grasp centre.
    centre_uv, centre_valid = _project(
        center.reshape(1, 3), intrinsics
    )

    if centre_valid[0]:
        p = tuple(
            np.round(centre_uv[0]).astype(int)
        )

        cv2.circle(
            image, p, 4, color, -1,
            cv2.LINE_AA
        )

        label = (
            f"#{rank} "
            f"{grasp.score:.2f} "
            f"{width * 1000:.0f}mm"
        )

        cv2.putText(
            image,
            label,
            (p[0] + 6, p[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA
        )


def draw_grasps(
    rgb,
    grasps,
    intrinsics,
    top_k=10,
):
    """
    Overlay top-K 3D grasp proposals on an RGB image.

    Returns an RGB uint8 image.
    """

    rgb = np.asarray(rgb)

    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(
            f"Expected RGB image (H,W,3), got {rgb.shape}"
        )

    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)

    # OpenCV drawing operates on BGR.
    image = cv2.cvtColor(
        rgb.copy(), cv2.COLOR_RGB2BGR
    )

    for rank, grasp in enumerate(
        sorted(
            grasps,
            key=lambda g: g.score,
            reverse=True
        )[:top_k],
        start=1
    ):
        draw_grasp(
            image,
            grasp,
            intrinsics,
            rank=rank
        )

    # Header.
    cv2.putText(
        image,
        f"GraspNet | {len(grasps)} proposals | "
        f"displaying top {min(len(grasps), top_k)}",
        (12, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA
    )

    return cv2.cvtColor(
        image, cv2.COLOR_BGR2RGB
    )


def save_grasp_overlay(
    rgb,
    grasps,
    intrinsics,
    output_path,
    top_k=10,
):
    """Generate and save the grasp overlay."""

    output_path = Path(output_path)
    output_path.parent.mkdir(
        parents=True, exist_ok=True
    )

    overlay = draw_grasps(
        rgb,
        grasps,
        intrinsics,
        top_k=top_k
    )

    # Convert RGB to BGR for imwrite.
    success = cv2.imwrite(
        str(output_path),
        cv2.cvtColor(
            overlay, cv2.COLOR_RGB2BGR
        )
    )

    if not success:
        raise IOError(
            f"Could not save grasp overlay: {output_path}"
        )

    print(f"[GraspNet] Visualisation saved: {output_path}")

    return output_path
