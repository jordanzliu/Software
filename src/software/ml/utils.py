import numpy as np


def get_enemy_goal_area(ssl_geometry_data):
    """Compute enemy goal area rectangle from SSL geometry data"""
    if not ssl_geometry_data or not ssl_geometry_data.field:
        return 6.0, 6.18, -0.9, 0.9  # fallback values

    field = ssl_geometry_data.field
    field_length = field.field_length / 1000  # convert mm to m
    goal_width = field.goal_width / 1000
    goal_depth = field.goal_depth / 1000

    # Enemy goal is at positive X end
    x_min = field_length / 2
    x_max = field_length / 2 + goal_depth
    y_min = -goal_width / 2
    y_max = goal_width / 2

    return x_min, x_max, y_min, y_max


def transform_to_robot_frame(x, y, theta, reference_robot):
    """Transform (x, y, theta) into the frame of reference_robot"""
    # Translate to robot origin
    dx = x - reference_robot.p_x
    dy = y - reference_robot.p_y

    # Rotate by negative robot orientation
    cos_theta = np.cos(-reference_robot.r_z)
    sin_theta = np.sin(-reference_robot.r_z)

    rel_x = dx * cos_theta - dy * sin_theta
    rel_y = dx * sin_theta + dy * cos_theta
    rel_theta = theta - reference_robot.r_z

    return rel_x, rel_y, rel_theta
