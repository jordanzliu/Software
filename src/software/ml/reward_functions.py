import numpy as np
from software.ml.utils import get_enemy_goal_area


def is_ball_in_enemy_goal(sim_state, ssl_geometry):
    """Returns true if ball is in enemy goal area"""
    if not sim_state or not sim_state.ball:
        return False

    ball_x = sim_state.ball.p_x
    ball_y = sim_state.ball.p_y

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


def goal_reward(sim_state, ssl_geometry):
    """Goal area reward (10 if ball in enemy goal)"""
    return 1.0 if is_ball_in_enemy_goal(sim_state, ssl_geometry) else 0.0


def distance_reward(sim_state):
    """Distance-based reward (0 to 0.1) for robot proximity to ball"""
    if not sim_state or not sim_state.yellow_robots or not sim_state.ball:
        return 0.0

    ball_x = sim_state.ball.p_x
    ball_y = sim_state.ball.p_y

    robot = sim_state.yellow_robots[0]
    robot_x = robot.p_x
    robot_y = robot.p_y

    distance = np.sqrt((robot_x - ball_x) ** 2 + (robot_y - ball_y) ** 2)
    return max(0.0, (1.0 - distance / 1.0))


def possession_reward(sim_state):
    if not sim_state or not sim_state.yellow_robots or not sim_state.ball:
        return 0.0

    return 1.0 if sim_state.yellow_robots[0].can_kick_ball else 0


def face_ball_orientation_reward(sim_state):
    if not sim_state or not sim_state.yellow_robots or not sim_state.ball:
        return 0.0

    robot_heading_unit_vec = np.array(
        [np.cos(sim_state.yellow_robots[0].r_z), np.sin(sim_state.yellow_robots[0].r_z)]
    )
    ball_to_robot_vec = np.array(
        [
            sim_state.ball.p_x - sim_state.yellow_robots[0].p_x,
            sim_state.ball.p_y - sim_state.yellow_robots[0].p_y,
        ]
    )
    ball_to_robot_unit_vec = ball_to_robot_vec / np.linalg.norm(ball_to_robot_vec)
    return robot_heading_unit_vec.dot(ball_to_robot_unit_vec)


def dribble_reward(sim_state, action):
    if (
        not sim_state
        or not sim_state.yellow_robots
        or not sim_state.ball
        or action is None
    ):
        return 0.0

    return (
        1.0
        if sim_state.yellow_robots[0].can_kick_ball
        and action[4] > 0.5  # AUTO_DRIBBLE index
        else 0
    )


def kick_reward(sim_state, action):
    if (
        not sim_state
        or not sim_state.yellow_robots
        or not sim_state.ball
        or action is None
    ):
        return 0.0

    return (
        1.0
        if sim_state.yellow_robots[0].can_kick_ball
        and action[3] > 0.5  # AUTO_KICK index
        else 0.0
    )
