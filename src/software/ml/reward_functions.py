import numpy as np
from software.ml.utils import get_enemy_goal_area


def is_ball_in_enemy_goal(sim_state, ssl_geometry, is_blue=False):
    """Returns true if ball is in enemy goal area"""
    if not sim_state or not sim_state.ball:
        return False

    ball_x = sim_state.ball.p_x
    ball_y = sim_state.ball.p_y

    if is_blue:
        # Blue robot's enemy goal is at negative x
        field_length = ssl_geometry.field.field_length / 1000
        goal_width = ssl_geometry.field.goal_width / 1000
        goal_depth = ssl_geometry.field.goal_depth / 1000
        x_min = -field_length / 2 - goal_depth
        x_max = -field_length / 2
        y_min = -goal_width / 2
        y_max = goal_width / 2
    else:
        x_min, x_max, y_min, y_max = get_enemy_goal_area(ssl_geometry)

    return x_min <= ball_x <= x_max and y_min <= ball_y <= y_max


def position_reward(sim_state, ssl_geometry):
    """Ball position reward (0 to 1) based on field position"""
    if not sim_state or not sim_state.ball or not ssl_geometry:
        return 0.0

    ball_x = sim_state.ball.p_x
    field_length = ssl_geometry.field.field_length / 1000  # convert mm to m
    friendly_goal_x = -field_length / 2
    enemy_goal_x = field_length / 2

    position_reward = (ball_x - friendly_goal_x) / (enemy_goal_x - friendly_goal_x)
    return max(0.0, min(1.0, position_reward))


def goal_reward(sim_state, ssl_geometry, is_blue=False):
    """Goal area reward (10 if ball in enemy goal)"""
    return 1.0 if is_ball_in_enemy_goal(sim_state, ssl_geometry, is_blue) else 0.0


def distance_reward(sim_state, is_blue=False):
    """Distance-based reward (0 to 0.1) for robot proximity to ball"""
    robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
    if not sim_state or not robots or not sim_state.ball:
        return 0.0

    ball_x = sim_state.ball.p_x
    ball_y = sim_state.ball.p_y

    robot = robots[0]
    robot_x = robot.p_x
    robot_y = robot.p_y

    distance = np.sqrt((robot_x - ball_x) ** 2 + (robot_y - ball_y) ** 2)
    return max(0.0, (1.0 - distance / 1.0))


def possession_reward(sim_state, is_blue=False):
    robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
    if not sim_state or not robots or not sim_state.ball:
        return 0.0

    return 1.0 if robots[0].can_kick_ball else 0


def face_ball_orientation_reward(sim_state, is_blue=False):
    robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
    if not sim_state or not robots or not sim_state.ball:
        return 0.0

    robot = robots[0]
    robot_heading_unit_vec = np.array([np.cos(robot.r_z), np.sin(robot.r_z)])
    ball_to_robot_vec = np.array(
        [
            sim_state.ball.p_x - robot.p_x,
            sim_state.ball.p_y - robot.p_y,
        ]
    )
    ball_to_robot_unit_vec = ball_to_robot_vec / np.linalg.norm(ball_to_robot_vec)
    return robot_heading_unit_vec.dot(ball_to_robot_unit_vec)


def dribble_reward(sim_state, action, is_blue=False):
    robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
    if not sim_state or not robots or not sim_state.ball or action is None:
        return 0.0

    return (
        1.0
        if robots[0].can_kick_ball and action[28] > 0.5  # AUTO_DRIBBLE index
        else 0
    )


def kick_reward(sim_state, action, is_blue=False):
    robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
    if not sim_state or not robots or not sim_state.ball or action is None:
        return 0.0

    return (
        1.0
        if robots[0].can_kick_ball and action[27] > 0.5  # AUTO_KICK index
        else 0.0
    )
