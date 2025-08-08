import gymnasium as gym
import numpy.random
from gymnasium import spaces
import numpy as np
from enum import IntEnum
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from software.thunderscope.binary_context_managers.simulator import Simulator
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from proto.message_translation.tbots_protobuf import create_world_state
import software.python_bindings as tbots_cpp
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.ml.reward_functions import (
    is_ball_in_enemy_goal,
    goal_reward,
    distance_reward,
    possession_reward,
    face_ball_orientation_reward,
    dribble_reward,
    kick_reward,
)


class ActionIndex(IntEnum):
    VELOCITY_X = 0
    VELOCITY_Y = 1
    VELOCITY_ANGULAR = 2
    AUTO_KICK = 3
    AUTO_DRIBBLE = 4


class ObservationIndex(IntEnum):
    FRIENDLY_ROBOT_X = 0
    FRIENDLY_ROBOT_Y = 1
    FRIENDLY_ROBOT_VX = 2
    FRIENDLY_ROBOT_VY = 3
    FRIENDLY_ROBOT_HEADING_COS = 4
    FRIENDLY_ROBOT_HEADING_SIN = 5
    FRIENDLY_ROBOT_CAN_KICK = 6
    ENEMY_ROBOT_REL_X = 7
    ENEMY_ROBOT_REL_Y = 8
    ENEMY_ROBOT_X = 9
    ENEMY_ROBOT_Y = 10
    ENEMY_ROBOT_VX = 11
    ENEMY_ROBOT_VY = 12
    ENEMY_ROBOT_HEADING_COS = 13
    ENEMY_ROBOT_HEADING_SIN = 14
    BALL_REL_X = 15
    BALL_REL_Y = 16
    BALL_X = 17
    BALL_Y = 18
    BALL_VX = 19
    BALL_VY = 20


class SimulatorGymEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}

    def __init__(self, simulator_runtime_dir, enable_realism=False):
        super().__init__()
        self.simulator = None
        self.simulator_runtime_dir = simulator_runtime_dir
        self.simulator_proto_unix_io = None
        self.yellow_io = None
        self.blue_io = None
        self.enable_realism = enable_realism
        self.primitive_seq = 0
        self.ssl_geometry = None

        # Define action space as Box
        self.action_space = spaces.Box(low=-1, high=1, shape=(5,), dtype=np.float32)
        # Define observation space as Box
        self.observation_space = spaces.Box(
            low=-10, high=10, shape=(21,), dtype=np.float32
        )

        self.yellow_io = ProtoUnixIO()
        self.blue_io = ProtoUnixIO()
        self.simulator_io = ProtoUnixIO()
        self.world_state_received_buffer = ThreadSafeBuffer(
            1, WorldStateReceivedTrigger
        )
        self.ssl_wrapper_buffer = ThreadSafeBuffer(10, SSL_WrapperPacket)
        self.simulator_state_buffer = ThreadSafeBuffer(10, SimulatorState)

    def _get_obs(self, sim_state):
        # Default values if no data available
        friendly_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        enemy_data = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32)
        ball_data = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        if sim_state and sim_state.yellow_robots:
            yellow_robot = sim_state.yellow_robots[0]

            # Yellow robot data
            friendly_data = np.array(
                [
                    yellow_robot.p_x,
                    yellow_robot.p_y,
                    yellow_robot.v_x,
                    yellow_robot.v_y,
                    np.cos(yellow_robot.r_z),
                    np.sin(yellow_robot.r_z),
                    1.0 if yellow_robot.can_kick_ball else 0.0,
                ],
                dtype=np.float32,
            )

            # Relative position to blue robot
            if sim_state.blue_robots:
                blue_robot = sim_state.blue_robots[0]
                rel_x = blue_robot.p_x - yellow_robot.p_x
                rel_y = blue_robot.p_y - yellow_robot.p_y

                enemy_data = np.array(
                    [
                        rel_x,
                        rel_y,
                        blue_robot.p_x,
                        blue_robot.p_y,
                        blue_robot.v_x,
                        blue_robot.v_y,
                        np.cos(blue_robot.r_z),
                        np.sin(blue_robot.r_z),
                    ],
                    dtype=np.float32,
                )

            # Relative position to yellow robot
            if sim_state.ball:
                ball = sim_state.ball
                rel_x = ball.p_x - yellow_robot.p_x
                rel_y = ball.p_y - yellow_robot.p_y

                ball_data = np.array(
                    [rel_x, rel_y, ball.p_x, ball.p_y, ball.v_x, ball.v_y],
                    dtype=np.float32,
                )

        return np.concatenate([friendly_data, enemy_data, ball_data])

    def _convert_action_to_primitive_set(self, action):
        # Scale velocities from [-1, 1] to actual robot limits
        local_vx = action[ActionIndex.VELOCITY_X] * 3.0  # 3.0 m/s max speed
        local_vy = action[ActionIndex.VELOCITY_Y] * 3.0  # 3.0 m/s max speed
        angular_velocity = (
            action[ActionIndex.VELOCITY_ANGULAR] * 10.0
        )  # 10.0 rad/s max angular speed

        # Transform velocity from robot frame to world frame
        if self.sim_state and self.sim_state.yellow_robots:
            robot_angle = self.sim_state.yellow_robots[0].r_z
            cos_theta = np.cos(robot_angle)
            sin_theta = np.sin(robot_angle)
            velocity_x = local_vx * cos_theta - local_vy * sin_theta
            velocity_y = local_vx * sin_theta + local_vy * cos_theta
        else:
            velocity_x = local_vx
            velocity_y = local_vy

        # Create DirectVelocityControl
        velocity_control = MotorControl.DirectVelocityControl()
        velocity_control.velocity.x_component_meters = velocity_x
        velocity_control.velocity.y_component_meters = velocity_y
        velocity_control.angular_velocity.radians_per_second = angular_velocity

        # Create MotorControl with DirectVelocityControl
        motor_control = MotorControl()
        motor_control.direct_velocity_control.CopyFrom(velocity_control)
        motor_control.dribbler_speed_rpm = (
            12000 if action[ActionIndex.AUTO_DRIBBLE] > 0.5 else 0
        )

        # Create power control primitive for auto kick
        chicker_control = PowerControl.ChickerControl()
        auto_kick = AutoChipOrKick()
        # TODO: add to action space instead of hardcoding
        auto_kick.autokick_speed_m_per_s = 5
        chicker_control.auto_chip_or_kick.CopyFrom(auto_kick)
        power_control = PowerControl()
        if action[ActionIndex.AUTO_KICK] > 0.5:
            power_control.chicker.CopyFrom(chicker_control)

        # Create DirectControlPrimitive
        control_primitive = DirectControlPrimitive()
        control_primitive.motor_control.CopyFrom(motor_control)
        control_primitive.power_control.CopyFrom(power_control)

        primitive = Primitive()
        primitive.direct_control.CopyFrom(control_primitive)
        primitive.sequence_number = self.primitive_seq
        self.primitive_seq += 1
        primitive.time_sent.CopyFrom(Timestamp())

        primitive_set = PrimitiveSet()
        primitive_set.robot_primitives[0].CopyFrom(primitive)
        primitive_set.time_sent.CopyFrom(Timestamp())

        return primitive_set

    def _compute_reward(self, sim_state, action):
        return (
            # position_reward(sim_state, self.ssl_geometry) * 50
            +goal_reward(sim_state, self.ssl_geometry) * 100
            + distance_reward(sim_state) * 0.1
            + face_ball_orientation_reward(sim_state) * 0.1
            + possession_reward(sim_state)
            + dribble_reward(sim_state, action) * 5
            + kick_reward(sim_state, action) * 10
        )

    def reset(self, seed=None, options=None):
        if self.simulator is None:
            self.simulator = Simulator(
                self.simulator_runtime_dir, enable_realism=self.enable_realism
            )
            self.simulator.__enter__()
            print("simulator started")

            self.yellow_io = ProtoUnixIO()
            self.blue_io = ProtoUnixIO()
            self.simulator_io = ProtoUnixIO()
            self.simulator.setup_proto_unix_io(
                blue_full_system_proto_unix_io=self.blue_io,
                yellow_full_system_proto_unix_io=self.yellow_io,
                simulator_proto_unix_io=self.simulator_io,
            )
            self.simulator_io.register_observer(
                WorldStateReceivedTrigger, self.world_state_received_buffer
            )
            self.yellow_io.register_observer(
                SimulatorState, self.simulator_state_buffer
            )
            self.yellow_io.register_observer(SSL_WrapperPacket, self.ssl_wrapper_buffer)
            print("proto IO set up")

        # Reset the world by sending a WorldState to simulator_io
        blue_bots = [
            tbots_cpp.Point(
                numpy.random.uniform(low=1, high=2),
                numpy.random.uniform(low=-1, high=1),
            )
        ]
        yellow_bots = [
            tbots_cpp.Point(
                numpy.random.uniform(low=-2, high=-1),
                numpy.random.uniform(low=-1, high=1),
            )
        ]
        ball_initial_pos = tbots_cpp.Point(
            numpy.random.uniform(low=-0.5, high=0.5),
            numpy.random.uniform(low=-0.5, high=0.5),
        )
        initial_world_state = create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=tbots_cpp.Vector(0, 0),
        )
        print("sending reset world state")
        world_state_received = None
        while world_state_received is None:
            self.simulator_io.send_proto(WorldState, initial_world_state)
            world_state_received = self.world_state_received_buffer.get(
                block=False, return_cached=False
            )
        print("world state reset acked")

        # tick the simulator until something happens
        world = None
        while world is None:
            tick = SimulatorTick(milliseconds=100)
            self.simulator_io.send_proto(SimulatorTick, tick)
            world = self.simulator_state_buffer.get(block=False, return_cached=False)
        print("simulator is alive")

        # tick the simulator once for everything to show up
        tick = SimulatorTick(milliseconds=100)
        self.simulator_io.send_proto(SimulatorTick, tick)
        self.sim_state = self.simulator_state_buffer.get(
            block=True, return_cached=False
        )
        # update the field geometry from the SSL_WrapperPacket
        self.ssl_geometry = self.ssl_wrapper_buffer.get(
            block=True, return_cached=False
        ).geometry
        return self._get_obs(self.sim_state), {}

    def step(self, action):
        primitive_set = self._convert_action_to_primitive_set(action)
        self.yellow_io.send_proto(PrimitiveSet, primitive_set)
        # tick the simulator at 100hz but only plan at 10hz
        for i in range(9):
            tick = SimulatorTick(milliseconds=10)
            self.simulator_io.send_proto(SimulatorTick, tick)

        # clear the buffer
        simulator_state = self.simulator_state_buffer.get(
            block=False, return_cached=False
        )
        while simulator_state is not None:
            simulator_state = self.simulator_state_buffer.get(
                block=False, return_cached=False
            )

        # send the last tick
        tick = SimulatorTick(milliseconds=10)
        self.simulator_io.send_proto(SimulatorTick, tick)
        self.sim_state = self.simulator_state_buffer.get(
            block=True, return_cached=False
        )

        # update the field geometry from the SSL_WrapperPacket
        self.ssl_geometry = self.ssl_wrapper_buffer.get(
            block=True, return_cached=False
        ).geometry

        obs = self._get_obs(self.sim_state)
        reward = self._compute_reward(self.sim_state, action)
        terminated = is_ball_in_enemy_goal(self.sim_state, self.ssl_geometry)
        truncated = False
        info = {}
        return obs, reward, terminated, truncated, info

    def render(self):
        sim_state = self.simulator_state_buffer.get(block=False)
        if not sim_state or not self.ssl_geometry:
            return np.zeros((800, 1200, 3), dtype=np.uint8)

        fig, ax = plt.subplots(figsize=(6, 4), dpi=200)

        # Draw field lines using SSL geometry data
        field = self.ssl_geometry.field
        field_length = field.field_length / 1000  # convert mm to m
        field_width = field.field_width / 1000
        goal_width = field.goal_width / 1000
        goal_depth = field.goal_depth / 1000

        # Field boundary
        field_rect = patches.Rectangle(
            (-field_length / 2, -field_width / 2),
            field_length,
            field_width,
            linewidth=2,
            edgecolor="white",
            facecolor="green",
            alpha=0.3,
        )
        ax.add_patch(field_rect)

        # Center circle
        center_circle = patches.Circle(
            (0, 0),
            field.center_circle_radius / 1000,  # convert mm to m
            linewidth=2,
            edgecolor="white",
            facecolor="none",
        )
        ax.add_patch(center_circle)

        # Left goal area
        left_goal_area = patches.Rectangle(
            (-field_length / 2, -goal_width / 2),
            goal_depth,
            goal_width,
            linewidth=2,
            edgecolor="white",
            facecolor="none",
        )
        ax.add_patch(left_goal_area)

        # Right goal area
        right_goal_area = patches.Rectangle(
            (field_length / 2 - goal_depth, -goal_width / 2),
            goal_depth,
            goal_width,
            linewidth=2,
            edgecolor="white",
            facecolor="none",
        )
        ax.add_patch(right_goal_area)

        # Goals
        left_goal = patches.Rectangle(
            (-field_length / 2 - goal_depth, -goal_width / 2),
            goal_depth,
            goal_width,
            linewidth=2,
            edgecolor="white",
            facecolor="none",
        )
        ax.add_patch(left_goal)

        right_goal = patches.Rectangle(
            (field_length / 2, -goal_width / 2),
            goal_depth,
            goal_width,
            linewidth=2,
            edgecolor="white",
            facecolor="none",
        )
        ax.add_patch(right_goal)

        # Draw robots and ball
        robot_radius = 0.09  # 180mm diameter = 90mm radius

        # Yellow robots (friendly)
        for robot in sim_state.yellow_robots:
            circle = patches.Circle(
                (robot.p_x, robot.p_y),
                robot_radius,
                facecolor="yellow",
                edgecolor="black",
            )
            ax.add_patch(circle)
            # Orientation line
            end_x = robot.p_x + robot_radius * np.cos(robot.r_z)
            end_y = robot.p_y + robot_radius * np.sin(robot.r_z)
            ax.plot([robot.p_x, end_x], [robot.p_y, end_y], "k-", linewidth=1)

        # Blue robots (enemy)
        for robot in sim_state.blue_robots:
            circle = patches.Circle(
                (robot.p_x, robot.p_y),
                robot_radius,
                facecolor="blue",
                edgecolor="black",
            )
            ax.add_patch(circle)
            # Orientation line
            end_x = robot.p_x + robot_radius * np.cos(robot.r_z)
            end_y = robot.p_y + robot_radius * np.sin(robot.r_z)
            ax.plot([robot.p_x, end_x], [robot.p_y, end_y], "k-", linewidth=1)

        # Ball
        if sim_state.ball:
            circle = patches.Circle(
                (sim_state.ball.p_x, sim_state.ball.p_y),
                0.0215,
                facecolor="orange",
                edgecolor="black",
            )
            ax.add_patch(circle)

        ax.set_xlim(-6, 6)
        ax.set_ylim(-4, 4)
        ax.set_aspect("equal")
        ax.set_facecolor("darkgreen")
        ax.axis("off")

        fig.canvas.draw()
        buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
        buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
        buf = buf[:, :, 1:]
        plt.close(fig)
        return buf

    def close(self):
        self.simulator.__exit__(None, None, None)


# Register the environment
gym.register(
    id="ThunderbotsSimulator-v0",
    entry_point="software.gym_envs.simulator_gym_env:SimulatorGymEnv",
    max_episode_steps=300,
)
