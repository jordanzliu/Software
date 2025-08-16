import numpy as np


def create_observation(sim_state, is_blue=False):
    """Create observation from sim state with global positions, relative positions, and global orientation unit vectors"""
    robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
    enemy_robots = sim_state.yellow_robots if is_blue else sim_state.blue_robots

    # Default values
    friendly_data = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], dtype=np.float32)
    enemy_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32)
    ball_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    if sim_state and robots:
        robot = robots[0]

        # Flip coordinates for blue robot
        robot_x = -robot.p_x if is_blue else robot.p_x
        robot_y = -robot.p_y if is_blue else robot.p_y
        robot_vx = -robot.v_x if is_blue else robot.v_x
        robot_vy = -robot.v_y if is_blue else robot.v_y
        robot_heading = robot.r_z + np.pi if is_blue else robot.r_z

        friendly_data = np.array(
            [
                robot_x,
                robot_y,
                robot_vx,
                robot_vy,
                np.cos(robot_heading),
                np.sin(robot_heading),
                1.0 if robot.can_kick_ball else 0.0,
            ],
            dtype=np.float32,
        )

        # Enemy robot data
        if enemy_robots:
            enemy_robot = enemy_robots[0]
            enemy_x = -enemy_robot.p_x if is_blue else enemy_robot.p_x
            enemy_y = -enemy_robot.p_y if is_blue else enemy_robot.p_y
            enemy_vx = -enemy_robot.v_x if is_blue else enemy_robot.v_x
            enemy_vy = -enemy_robot.v_y if is_blue else enemy_robot.v_y
            enemy_heading = enemy_robot.r_z + np.pi if is_blue else enemy_robot.r_z

            rel_x = enemy_x - robot_x
            rel_y = enemy_y - robot_y

            enemy_data = np.array(
                [
                    rel_x,
                    rel_y,
                    enemy_x,
                    enemy_y,
                    enemy_vx,
                    enemy_vy,
                    np.cos(enemy_heading),
                    np.sin(enemy_heading),
                ],
                dtype=np.float32,
            )

        # Ball data
        if sim_state.ball:
            ball = sim_state.ball
            ball_x = -ball.p_x if is_blue else ball.p_x
            ball_y = -ball.p_y if is_blue else ball.p_y
            ball_vx = -ball.v_x if is_blue else ball.v_x
            ball_vy = -ball.v_y if is_blue else ball.v_y

            rel_x = ball_x - robot_x
            rel_y = ball_y - robot_y

            ball_data = np.array(
                [rel_x, rel_y, ball_x, ball_y, ball_vx, ball_vy], dtype=np.float32
            )

    return np.concatenate([friendly_data, enemy_data, ball_data])


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
