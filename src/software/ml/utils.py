from enum import IntEnum

import numpy as np
import torch
from proto.import_all_protos import *

class ActionIndex(IntEnum):
    DIRECTIONAL = 0,
    ROTATION = 1,
    DRIBBLE = 2,
    KICK = 3,
    SIZE = 4

class ObservationIndex(IntEnum):
    FRIENDLY_ROBOT_ABSOLUTE_X = 0
    FRIENDLY_ROBOT_ABSOLUTE_Y = 1
    FRIENDLY_ROBOT_CAN_KICK = 2
    ENEMY_ROBOT_REL_X = 3
    ENEMY_ROBOT_REL_Y = 4
    ENEMY_ROBOT_REL_VX = 5
    ENEMY_ROBOT_REL_VY = 6
    ENEMY_ROBOT_HEADING_COS = 7
    ENEMY_ROBOT_HEADING_SIN = 8
    BALL_REL_X = 9
    BALL_REL_Y = 10
    BALL_REL_VX = 11
    BALL_REL_VY = 12
    GOAL_TOP_REL_X = 13
    GOAL_TOP_REL_Y = 14
    GOAL_BOTTOM_REL_X = 15
    GOAL_BOTTOM_REL_Y = 16
    SIZE = 17


def convert_action_to_primitive_set(sim_state, action, primitive_seq):
    if not sim_state.yellow_robots:
        return PrimitiveSet(), primitive_seq

    # get the unit vector of the commanded velocity
    velocity_command_vec_lookup_table = [
        np.array([3.0, 0.0]), # positive X
        np.array([-3.0, 0.0]),  # negative X
        np.array([0.0, 3.0]),  # positive Y
        np.array([3.0, -3.0]),  # negative Y
    ]
    velocity_command_vec = velocity_command_vec_lookup_table[action[ActionIndex.DIRECTIONAL]]

    # get the desired angular velocity
    rotation_command_lookup_table = [
        5,
        -5
    ]

    # Create DirectVelocityControl
    velocity_control = MotorControl.DirectVelocityControl()
    velocity_control.velocity.x_component_meters = velocity_command_vec[0]
    velocity_control.velocity.y_component_meters = velocity_command_vec[1]
    velocity_control.angular_velocity.radians_per_second = rotation_command_lookup_table[action[ActionIndex.ROTATION]]

    # Create MotorControl with DirectVelocityControl
    motor_control = MotorControl()
    motor_control.direct_velocity_control.CopyFrom(velocity_control)
    motor_control.dribbler_speed_rpm = (
        12000 if action[ActionIndex.DRIBBLE] > 0 else 0
    )

    # Create power control primitive for auto kick
    chicker_control = PowerControl.ChickerControl()
    auto_kick = AutoChipOrKick()
    # TODO: tune kick speed?
    auto_kick.autokick_speed_m_per_s = 10
    chicker_control.auto_chip_or_kick.CopyFrom(auto_kick)
    power_control = PowerControl()
    if action[ActionIndex.KICK] > 0:
        power_control.chicker.CopyFrom(chicker_control)

    # Create DirectControlPrimitive
    control_primitive = DirectControlPrimitive()
    control_primitive.motor_control.CopyFrom(motor_control)
    control_primitive.power_control.CopyFrom(power_control)

    primitive = Primitive()
    primitive.direct_control.CopyFrom(control_primitive)
    primitive.sequence_number = primitive_seq
    primitive_seq += 1
    primitive.time_sent.CopyFrom(Timestamp())

    primitive_set = PrimitiveSet()
    primitive_set.robot_primitives[0].CopyFrom(primitive)
    primitive_set.time_sent.CopyFrom(Timestamp())

    return primitive_set, primitive_seq


def create_observation(sim_state, ssl_geometry, robot_id, is_blue=False):
    """Create observation for a given robot in the coordinate frame of that robot"""
    robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
    enemy_robots = sim_state.yellow_robots if is_blue else sim_state.blue_robots

    if not sim_state or not robots or not enemy_robots:
        return torch.zeros(ObservationIndex.SIZE)

    ego_robot = next(robot for robot in robots if robot.id == robot_id)
    ball = sim_state.ball
    # Transform ball position and velocity to robot frame
    ball_x, ball_y, _ = transform_to_robot_frame(
        ball.p_x, ball.p_y, 0, ego_robot
    )
    ball_vx, ball_vy = rotate_vector_to_robot_frame(ball.v_x, ball.v_y, ego_robot)

    # Transform enemy robots to robot frame
    enemy_robots_obs = []
    for enemy_robot in enemy_robots:
        rel_x, rel_y, rel_theta = transform_to_robot_frame(
            enemy_robot.p_x, enemy_robot.p_y, enemy_robot.r_z, ego_robot
        )
        rel_vx, rel_vy = rotate_vector_to_robot_frame(
            enemy_robot.v_x, enemy_robot.v_y, ego_robot
        )
        enemy_robots_obs.append([rel_x, rel_y, rel_vx, rel_vy, np.cos(rel_theta), np.sin(rel_theta)])

    enemy_goal_top_point = np.array(
        [ssl_geometry.field.field_length / 2, ssl_geometry.field.goal_width / 2]
    )
    enemy_goal_bottom_point = np.array(
        [ssl_geometry.field.field_length / 2, -ssl_geometry.field.goal_width / 2]
    )
    enemy_goal_top_point_robot_frame = transform_to_robot_frame(
        enemy_goal_top_point[0], enemy_goal_top_point[1], 0, ego_robot
    )
    enemy_goal_bottom_point_robot_frame = transform_to_robot_frame(
        enemy_goal_bottom_point[0], enemy_goal_bottom_point[1], 0, ego_robot
    )

    obs = torch.zeros(ObservationIndex.SIZE)
    obs[ObservationIndex.FRIENDLY_ROBOT_ABSOLUTE_X] = ego_robot.p_x
    obs[ObservationIndex.FRIENDLY_ROBOT_ABSOLUTE_Y] = ego_robot.p_y
    obs[ObservationIndex.FRIENDLY_ROBOT_CAN_KICK] = 10 if ego_robot.can_kick_ball else -10
    obs[ObservationIndex.ENEMY_ROBOT_REL_X] = enemy_robots_obs[0][0]
    obs[ObservationIndex.ENEMY_ROBOT_REL_Y] = enemy_robots_obs[0][1]
    obs[ObservationIndex.ENEMY_ROBOT_REL_VX] = enemy_robots_obs[0][2]
    obs[ObservationIndex.ENEMY_ROBOT_REL_VY] = enemy_robots_obs[0][3]
    obs[ObservationIndex.ENEMY_ROBOT_HEADING_COS] = enemy_robots_obs[0][4]
    obs[ObservationIndex.ENEMY_ROBOT_HEADING_SIN] = enemy_robots_obs[0][5]
    obs[ObservationIndex.BALL_REL_X] = ball_x
    obs[ObservationIndex.BALL_REL_Y] = ball_y
    obs[ObservationIndex.BALL_REL_VX] = ball_vx
    obs[ObservationIndex.BALL_REL_VY] = ball_vy
    obs[ObservationIndex.GOAL_TOP_REL_X] = enemy_goal_top_point_robot_frame[0]
    obs[ObservationIndex.GOAL_TOP_REL_Y] = enemy_goal_top_point_robot_frame[1]
    obs[ObservationIndex.GOAL_BOTTOM_REL_X] = enemy_goal_bottom_point_robot_frame[0]
    obs[ObservationIndex.GOAL_BOTTOM_REL_Y] = enemy_goal_bottom_point_robot_frame[1]
    return obs


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

def rotate_vector_to_robot_frame(vx=0, vy=0, reference_robot=None):
    """Rotate a vector (vx, vy) to the frame of reference_robot"""
    cos_theta = np.cos(-reference_robot.r_z)
    sin_theta = np.sin(-reference_robot.r_z)

    rel_vx = vx * cos_theta - vy * sin_theta
    rel_vy = vx * sin_theta + vy * cos_theta

    return rel_vx, rel_vy
