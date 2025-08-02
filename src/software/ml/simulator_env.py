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
    ENEMY_ROBOT_X = 4
    ENEMY_ROBOT_Y = 5
    ENEMY_ROBOT_VX = 6
    ENEMY_ROBOT_VY = 7
    BALL_X = 8
    BALL_Y = 9
    BALL_VX = 10
    BALL_VY = 11


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
            low=-10, high=10, shape=(12,), dtype=np.float32
        )

        self.yellow_io = ProtoUnixIO()
        self.blue_io = ProtoUnixIO()
        self.simulator_io = ProtoUnixIO()
        self.world_state_received_buffer = ThreadSafeBuffer(
            1, WorldStateReceivedTrigger
        )
        self.ssl_wrapper_buffer = ThreadSafeBuffer(1, SSL_WrapperPacket)
        self.simulator_state_buffer = ThreadSafeBuffer(1, SimulatorState)

    def _get_obs(self, sim_state):
        # Default values if no data available
        friendly_data = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        enemy_data = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        ball_data = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        if sim_state:
            # Get first yellow robot position and velocity
            if sim_state.yellow_robots:
                robot = sim_state.yellow_robots[0]
                friendly_data = np.array(
                    [robot.p_x, robot.p_y, robot.v_x, robot.v_y], dtype=np.float32
                )

            # Get first blue robot position and velocity
            if sim_state.blue_robots:
                robot = sim_state.blue_robots[0]
                enemy_data = np.array(
                    [robot.p_x, robot.p_y, robot.v_x, robot.v_y], dtype=np.float32
                )

            # Get ball position and velocity
            if sim_state.ball:
                ball = sim_state.ball
                ball_data = np.array(
                    [ball.p_x, ball.p_y, ball.v_x, ball.v_y], dtype=np.float32
                )

        return np.concatenate([friendly_data, enemy_data, ball_data])

    def _convert_action_to_primitive_set(self, action):
        # Scale velocities from [-1, 1] to actual robot limits
        velocity_x = action[ActionIndex.VELOCITY_X] * 3.0  # 3.0 m/s max speed
        velocity_y = action[ActionIndex.VELOCITY_Y] * 3.0  # 3.0 m/s max speed
        angular_velocity = (
            action[ActionIndex.VELOCITY_ANGULAR] * 10.0
        )  # 10.0 rad/s max angular speed

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

    def _get_enemy_goal_area(self):
        """Helper function to compute enemy goal area rectangle"""
        if not self.ssl_geometry:
            return 6.0, 6.18, -0.9, 0.9  # fallback values

        field = self.ssl_geometry.field
        field_length = field.field_length / 1000  # convert mm to m
        goal_width = field.goal_width / 1000
        goal_depth = field.goal_depth / 1000

        # Enemy goal is at positive X end
        x_min = field_length / 2
        x_max = field_length / 2 + goal_depth
        y_min = -goal_width / 2
        y_max = goal_width / 2

        return x_min, x_max, y_min, y_max

    def _is_ball_in_enemy_goal(self, sim_state):
        """Returns true if ball is in enemy goal area"""
        if not sim_state or not sim_state.ball:
            return False

        ball_x = sim_state.ball.p_x
        ball_y = sim_state.ball.p_y

        x_min, x_max, y_min, y_max = self._get_enemy_goal_area()
        return x_min <= ball_x <= x_max and y_min <= ball_y <= y_max

    def _position_reward(self, sim_state):
        """Ball position reward (0 to 1) based on field position"""
        if not sim_state or not sim_state.ball or not self.ssl_geometry:
            return 0.0

        ball_x = sim_state.ball.p_x
        field_length = self.ssl_geometry.field.field_length / 1000  # convert mm to m
        friendly_goal_x = -field_length / 2
        enemy_goal_x = field_length / 2

        position_reward = (ball_x - friendly_goal_x) / (enemy_goal_x - friendly_goal_x)
        return max(0.0, min(1.0, position_reward))

    def _goal_reward(self, sim_state):
        """Goal area reward (10 if ball in enemy goal)"""
        return 10.0 if self._is_ball_in_enemy_goal(sim_state) else 0.0

    def _distance_reward(self, sim_state):
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

    def _compute_reward(self, sim_state):
        return (
            # self._position_reward(sim_state)
            # + self._goal_reward(sim_state)
            +self._distance_reward(sim_state)
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
        tick = SimulatorTick(milliseconds=100)
        self.simulator_io.send_proto(SimulatorTick, tick)
        self.sim_state = self.simulator_state_buffer.get(
            block=True, return_cached=False
        )

        # update the field geometry from the SSL_WrapperPacket
        self.ssl_geometry = self.ssl_wrapper_buffer.get(
            block=True, return_cached=False
        ).geometry

        obs = self._get_obs(self.sim_state)
        reward = self._compute_reward(self.sim_state)
        terminated = self._is_ball_in_enemy_goal(self.sim_state)
        truncated = False
        info = {}
        return obs, reward, terminated, truncated, info

    def render(self):
        sim_state = self.simulator_state_buffer.get(block=False)
        if not sim_state or not self.ssl_geometry:
            return np.zeros((400, 600, 3), dtype=np.uint8)

        fig, ax = plt.subplots(figsize=(6, 4), dpi=100)

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

        # Blue robots (enemy)
        for robot in sim_state.blue_robots:
            circle = patches.Circle(
                (robot.p_x, robot.p_y),
                robot_radius,
                facecolor="blue",
                edgecolor="black",
            )
            ax.add_patch(circle)

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
