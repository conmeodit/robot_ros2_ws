import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_xy(
    x: float,
    y: float,
    offset_x: float,
    offset_y: float,
    yaw: float,
) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        offset_x + cos_yaw * x - sin_yaw * y,
        offset_y + sin_yaw * x + cos_yaw * y,
    )


def inverse_transform_xy(
    x: float,
    y: float,
    offset_x: float,
    offset_y: float,
    yaw: float,
) -> Tuple[float, float]:
    dx = x - offset_x
    dy = y - offset_y
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
    )


def coerce_homography(values: Sequence[float]) -> Optional[Tuple[float, ...]]:
    if len(values) != 9:
        return None
    homography = tuple(float(value) for value in values)
    if not math.isfinite(sum(homography)):
        return None
    if all(abs(value) < 1e-12 for value in homography):
        return None
    return homography


def project_image_point_to_ground(
    u: float,
    v: float,
    homography: Sequence[float],
) -> Optional[Tuple[float, float]]:
    h = coerce_homography(homography)
    if h is None:
        return None

    denom = h[6] * u + h[7] * v + h[8]
    if abs(denom) < 1e-9:
        return None

    ground_x = (h[0] * u + h[1] * v + h[2]) / denom
    ground_y = (h[3] * u + h[4] * v + h[5]) / denom
    if not math.isfinite(ground_x) or not math.isfinite(ground_y):
        return None
    return ground_x, ground_y


class GroundProjector:
    """Project image pixels from a downward camera onto the floor in base_link."""

    def __init__(
        self,
        *,
        projection_method: str,
        camera_height_m: float,
        camera_offset_x_m: float,
        camera_offset_y_m: float,
        camera_yaw_rad: float,
        homography: Sequence[float],
    ):
        self.projection_method = projection_method.strip().lower()
        self.camera_height_m = max(1e-3, float(camera_height_m))
        self.camera_offset_x_m = float(camera_offset_x_m)
        self.camera_offset_y_m = float(camera_offset_y_m)
        self.camera_yaw_rad = float(camera_yaw_rad)
        self.homography = coerce_homography(homography)
        self.camera_model: Optional[CameraModel] = None

    def set_camera_info(self, k: Sequence[float]):
        if len(k) < 6:
            return
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        if fx <= 0.0 or fy <= 0.0:
            return
        self.camera_model = CameraModel(fx=fx, fy=fy, cx=cx, cy=cy)

    def project_pixel(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        local_xy = None
        if self.projection_method in ('homography', 'auto') and self.homography is not None:
            local_xy = project_image_point_to_ground(u, v, self.homography)
        elif self.projection_method in ('pinhole_downward', 'auto'):
            local_xy = self._project_with_pinhole_downward(u, v)

        if local_xy is None:
            return None

        return transform_xy(
            local_xy[0],
            local_xy[1],
            self.camera_offset_x_m,
            self.camera_offset_y_m,
            self.camera_yaw_rad,
        )

    def _project_with_pinhole_downward(
        self,
        u: float,
        v: float,
    ) -> Optional[Tuple[float, float]]:
        if self.camera_model is None:
            return None

        model = self.camera_model
        ground_x = (v - model.cy) * self.camera_height_m / model.fy
        ground_y = -(u - model.cx) * self.camera_height_m / model.fx
        if not math.isfinite(ground_x) or not math.isfinite(ground_y):
            return None
        return ground_x, ground_y
