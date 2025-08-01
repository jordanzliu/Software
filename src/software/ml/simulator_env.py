import gymnasium as gym
from gymnasium import spaces
import numpy as np
from enum import IntEnum
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class ActionIndex(IntEnum):
    VELOCITY_X = 0
    VELOCITY_Y = 1
    VELOCITY_ANGULAR = 2
    AUTO_KICK = 3
    AUTO_DRIBBLE = 4


class ObservationIndex(IntEnum):
    FRIENDLY_ROBOT_X = 0
    FRIENDLY_ROBOT_Y = 1
    ENEMY_ROBOT_X = 2
    ENEMY_ROBOT_Y = 3
    BALL_X = 4
    BALL_Y = 5


from software.thunderscope.binary_context_managers.simulator import Simulator
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from proto.message_translation.tbots_protobuf import create_world_state
import software.python_bindings as tbots_cpp


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

        # Define action space as Box
        self.action_space = spaces.Box(low=-1, high=1, shape=(5,), dtype=np.float32)
        # Define observation space as Box
        self.observation_space = spaces.Box(
            low=-10, high=10, shape=(6,), dtype=np.float32
        )

        self.yellow_io = ProtoUnixIO()
        self.blue_io = ProtoUnixIO()
        self.simulator_io = ProtoUnixIO()
        self.ssl_wrapper_buffer = ThreadSafeBuffer(1, SSL_WrapperPacket)

    def _get_obs(self, ssl_wrapper):
        # Default positions if no data available
        friendly_pos = np.array([0.0, 0.0], dtype=np.float32)
        enemy_pos = np.array([0.0, 0.0], dtype=np.float32)
        ball_pos = np.array([0.0, 0.0], dtype=np.float32)

        if ssl_wrapper and ssl_wrapper.detection:
            detection = ssl_wrapper.detection

            # Get first yellow robot position
            if detection.robots_yellow:
                robot = detection.robots_yellow[0]
                friendly_pos = np.array(
                    [robot.x / 1000, robot.y / 1000], dtype=np.float32
                )

            # Get first blue robot position
            if detection.robots_blue:
                robot = detection.robots_blue[0]
                enemy_pos = np.array([robot.x / 1000, robot.y / 1000], dtype=np.float32)

            # Get ball position
            if detection.balls:
                ball = detection.balls[0]
                ball_pos = np.array([ball.x / 1000, ball.y / 1000], dtype=np.float32)

        return np.concatenate([friendly_pos, enemy_pos, ball_pos])

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

    def _get_enemy_goal_area(self, geometry):
        """Helper function to compute enemy goal area rectangle"""
        field = geometry.field
        goal_width = field.goal_width / 1000
        goal_depth = field.goal_depth / 1000
        field_length = field.field_length / 1000

        # Enemy goal is at positive X end
        x_min = field_length / 2
        x_max = field_length / 2 + goal_depth
        y_min = -goal_width / 2
        y_max = goal_width / 2

        return x_min, x_max, y_min, y_max

    def _is_ball_in_enemy_goal(self, ssl_wrapper):
        """Returns true if ball is in enemy goal area"""
        if not ssl_wrapper or not ssl_wrapper.detection or not ssl_wrapper.geometry:
            return False

        if not ssl_wrapper.detection.balls:
            return False

        ball = ssl_wrapper.detection.balls[0]
        ball_x, ball_y = ball.x / 1000, ball.y / 1000

        x_min, x_max, y_min, y_max = self._get_enemy_goal_area(ssl_wrapper.geometry)
        return x_min <= ball_x <= x_max and y_min <= ball_y <= y_max

    def _compute_reward(self, ssl_wrapper):
        if not ssl_wrapper or not ssl_wrapper.detection:
            return 0.0

        # Get ball position
        ball_x, ball_y = 0.0, 0.0
        if ssl_wrapper.detection.balls:
            ball = ssl_wrapper.detection.balls[0]
            ball_x, ball_y = ball.x / 1000, ball.y / 1000

        # Get friendly robot position
        friendly_x, friendly_y = 0.0, 0.0
        if ssl_wrapper.detection.robots_yellow:
            robot = ssl_wrapper.detection.robots_yellow[0]
            friendly_x, friendly_y = robot.x / 1000, robot.y / 1000

        reward = 0.0

        # Position-based reward (0 to 1)
        if ssl_wrapper.geometry:
            field_length = ssl_wrapper.geometry.field.field_length / 1000
            friendly_goal_x = -field_length / 2
            enemy_goal_x = field_length / 2

            # Linear interpolation from friendly goal (0) to enemy goal (1)
            position_reward = (ball_x - friendly_goal_x) / (
                enemy_goal_x - friendly_goal_x
            )
            position_reward = max(0.0, min(1.0, position_reward))
            reward += position_reward

            # Goal area reward (10 if ball in enemy goal)
            if self._is_ball_in_enemy_goal(ssl_wrapper):
                reward += 10.0

        # Distance-based reward (0 to 0.1)
        distance = np.sqrt((friendly_x - ball_x) ** 2 + (friendly_y - ball_y) ** 2)
        distance_reward = max(0.0, 0.1 * (1.0 - distance / 1.0))
        reward += distance_reward
        return reward

    def reset(self, seed=None, options=None):
        if self.simulator is not None:
            self.simulator.__exit__()

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
        # we don't need to receive the wrapper packet twice, it's the same for yellow and blue
        self.yellow_io.register_observer(SSL_WrapperPacket, self.ssl_wrapper_buffer)
        print("proto IO set up")

        # tick the simulator until something happens
        ssl_wrapper = None
        while ssl_wrapper is None:
            tick = SimulatorTick(milliseconds=100)
            self.simulator_io.send_proto(SimulatorTick, tick)
            ssl_wrapper = self.ssl_wrapper_buffer.get(block=False, return_cached=False)
        print("simulator is alive")

        # Reset the world by sending a WorldState to simulator_io
        blue_bots = [tbots_cpp.Point(1, 0)]
        yellow_bots = [tbots_cpp.Point(-1, 0)]
        ball_initial_pos = tbots_cpp.Point(0, 0)

        self.simulator_io.send_proto(
            WorldState,
            create_world_state(
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )
        print("sent reset world state")

        self.ssl_wrapper = self.ssl_wrapper_buffer.get(block=True, return_cached=False)
        return self._get_obs(self.ssl_wrapper), {}

    def step(self, action):
        self.ssl_wrapper = self.ssl_wrapper_buffer.get(block=True, return_cached=False)
        obs = self._get_obs(self.ssl_wrapper)
        primitive_set = self._convert_action_to_primitive_set(action)
        self.yellow_io.send_proto(PrimitiveSet, primitive_set)
        tick = SimulatorTick(milliseconds=100)
        self.simulator_io.send_proto(SimulatorTick, tick)
        reward = self._compute_reward(self.ssl_wrapper)
        terminated = self._is_ball_in_enemy_goal(self.ssl_wrapper)
        truncated = False
        info = {}
        return obs, reward, terminated, truncated, info

    def render(self):
        ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
        if not ssl_wrapper:
            return np.zeros((400, 600, 3), dtype=np.uint8)

        fig, ax = plt.subplots(figsize=(6, 4), dpi=100)

        # Draw field lines from geometry
        if ssl_wrapper.geometry:
            field = ssl_wrapper.geometry.field
            field_length = field.field_length / 1000
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
                field.center_circle_radius / 1000,
                linewidth=2,
                edgecolor="white",
                facecolor="none",
            )
            ax.add_patch(center_circle)

            # Goal areas
            goal_area_depth = field.goal_area_depth / 1000
            goal_area_width = field.goal_area_width / 1000

            # Left goal area
            left_goal_area = patches.Rectangle(
                (-field_length / 2, -goal_area_width / 2),
                goal_area_depth,
                goal_area_width,
                linewidth=2,
                edgecolor="white",
                facecolor="none",
            )
            ax.add_patch(left_goal_area)

            # Right goal area
            right_goal_area = patches.Rectangle(
                (field_length / 2 - goal_area_depth, -goal_area_width / 2),
                goal_area_depth,
                goal_area_width,
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

        if ssl_wrapper.detection:
            # Yellow robots (friendly)
            for robot in ssl_wrapper.detection.robots_yellow:
                circle = patches.Circle(
                    (robot.x / 1000, robot.y / 1000),
                    robot_radius,
                    facecolor="yellow",
                    edgecolor="black",
                )
                ax.add_patch(circle)

            # Blue robots (enemy)
            for robot in ssl_wrapper.detection.robots_blue:
                circle = patches.Circle(
                    (robot.x / 1000, robot.y / 1000),
                    robot_radius,
                    facecolor="blue",
                    edgecolor="black",
                )
                ax.add_patch(circle)

            # Ball
            for ball in ssl_wrapper.detection.balls:
                circle = patches.Circle(
                    (ball.x / 1000, ball.y / 1000),
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
        buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
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
