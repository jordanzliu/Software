import numpy as np


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
