from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class GroundPlaneCalib:
    """
    Camera calibration + mounting parameters for a simple ground-plane projection model.

    Intrinsics (fx, fy, cx, cy):
      - Standard pinhole camera parameters in pixel units.
      - fx, fy are focal lengths (pixels)
      - cx, cy are principal point (pixels), typically near image center.

    Extrinsics (IMPERIAL UNITS):
      - cam_height_ft: camera optical center height above the ground plane (feet)
      - pitch_rad: camera pitch angle (radians).
        Positive means the camera is tilted downward.
    """
    fx: float
    fy: float
    cx: float
    cy: float
    cam_height_ft: float
    pitch_rad: float


def pixel_to_ground(
    u: float,
    v: float,
    calib: GroundPlaneCalib,
) -> Tuple[Optional[float], Optional[float], bool]:
    """
    Project an image pixel (u, v) to the ground plane and return approximate
    forward and lateral distances in FEET.

    Coordinate conventions:
      - Camera frame: x right, y down, z forward
      - Ground plane is flat
      - Camera roll and yaw are assumed ~0 (pitch only)

    Returns:
      (forward_ft, lateral_ft, valid)

      - forward_ft: forward distance on ground (feet)
      - lateral_ft: rightward distance on ground (feet)
      - valid: False if the ray does not intersect the ground in front of the camera
    """

    # Guard against bad calibration
    if calib.fx <= 0 or calib.fy <= 0 or calib.cam_height_ft <= 0:
        return None, None, False

    # 1) Pixel -> normalized camera ray
    x = (u - calib.cx) / calib.fx
    y = (v - calib.cy) / calib.fy

    # Ray direction in camera coordinates (not normalized)
    dx_cam, dy_cam, dz_cam = x, y, 1.0

    # 2) Rotate ray by camera pitch (about x-axis)
    cp = math.cos(calib.pitch_rad)
    sp = math.sin(calib.pitch_rad)

    dx = dx_cam
    dy = cp * dy_cam - sp * dz_cam   # down component
    dz = sp * dy_cam + cp * dz_cam   # forward component

    # Ray must point downward to hit the ground
    if dy <= 1e-9:
        return None, None, False

    # 3) Solve intersection with ground plane
    # Camera starts cam_height_ft above ground, moving "down" by dy
    t = calib.cam_height_ft / dy

    if t <= 0:
        return None, None, False

    # 4) Ground distances (feet)
    forward_ft = t * dz
    lateral_ft = t * dx

    if forward_ft <= 0:
        return None, None, False

    return forward_ft, lateral_ft, True
