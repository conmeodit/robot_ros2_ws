import math

from robot_vision.ground_projection import (
    GroundProjector,
    inverse_transform_xy,
    project_image_point_to_ground,
    transform_xy,
)


def test_project_image_point_to_ground_uses_homography():
    homography = [
        0.01,
        0.0,
        -3.2,
        0.0,
        0.01,
        -2.4,
        0.0,
        0.0,
        1.0,
    ]

    assert project_image_point_to_ground(320.0, 240.0, homography) == (0.0, 0.0)
    assert project_image_point_to_ground(420.0, 240.0, homography) == (1.0, 0.0)


def test_ground_projector_applies_camera_offset_and_yaw():
    projector = GroundProjector(
        projection_method='homography',
        camera_height_m=1.0,
        camera_offset_x_m=0.2,
        camera_offset_y_m=-0.1,
        camera_yaw_rad=math.pi / 2.0,
        homography=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    )

    x, y = projector.project_pixel(1.0, 0.0)

    assert abs(x - 0.2) < 1e-9
    assert abs(y - 0.9) < 1e-9


def test_transform_xy_round_trip():
    world = transform_xy(0.4, -0.2, 1.0, 2.0, 0.7)
    local = inverse_transform_xy(world[0], world[1], 1.0, 2.0, 0.7)

    assert abs(local[0] - 0.4) < 1e-9
    assert abs(local[1] + 0.2) < 1e-9
