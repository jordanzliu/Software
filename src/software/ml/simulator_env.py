import gymnasium as gym
import numpy.random
from gymnasium import spaces
import numpy as np
from enum import IntEnum

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
    kick_reward,
    ball_toward_goal_reward,
    position_reward,
)
from software.ml.utils import create_observation
from software.ml.render import render_simulator


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
        self.simulator_state_buffer = ThreadSafeBuffer(1, SimulatorState)
        self.last_action = None

    def _get_obs(self, sim_state):
        return create_observation(sim_state, is_blue=False)

    def _convert_action_to_primitive_set(self, sim_state, action):
        if not sim_state.yellow_robots:
            return PrimitiveSet()

        # get the unit vector of the commanded velocity
        velocity_command_vec = np.array(
            [action[ActionIndex.VELOCITY_X], action[ActionIndex.VELOCITY_Y]]
        )
        velocity_command_vec /= np.linalg.norm(velocity_command_vec)
        # scale by velocity limits
        velocity_command_vec *= 3.0
        angular_velocity = (
            action[ActionIndex.VELOCITY_ANGULAR] * 10.0
        )  # 10.0 rad/s max angular speed

        # transform velocity into robot frame for the direct velocity primitive
        robot = sim_state.yellow_robots[0]
        robot_x_vector = np.array([np.cos(robot.r_z), np.sin(robot.r_z)])
        robot_y_vector = np.array([np.sin(robot.r_z), -np.cos(robot.r_z)])
        vel_x_robot_frame, vel_y_robot_frame = (
            np.vstack((robot_x_vector, robot_y_vector)).T @ velocity_command_vec
        )

        # Create DirectVelocityControl
        velocity_control = MotorControl.DirectVelocityControl()
        velocity_control.velocity.x_component_meters = vel_x_robot_frame
        velocity_control.velocity.y_component_meters = vel_y_robot_frame
        velocity_control.angular_velocity.radians_per_second = angular_velocity

        # Create MotorControl with DirectVelocityControl
        motor_control = MotorControl()
        motor_control.direct_velocity_control.CopyFrom(velocity_control)
        motor_control.dribbler_speed_rpm = (
            12000 if action[ActionIndex.AUTO_DRIBBLE] > 0 else 0
        )

        # Create power control primitive for auto kick
        chicker_control = PowerControl.ChickerControl()
        auto_kick = AutoChipOrKick()
        # TODO: add to action space instead of hardcoding
        auto_kick.autokick_speed_m_per_s = 5
        chicker_control.auto_chip_or_kick.CopyFrom(auto_kick)
        power_control = PowerControl()
        if action[ActionIndex.AUTO_KICK] > 0.0:
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

    def _compute_reward(self, sim_state, action, is_blue=False):
        return (
            position_reward(sim_state, self.ssl_geometry)
            + goal_reward(sim_state, self.ssl_geometry, is_blue) * 10
            + distance_reward(sim_state, max_distance=1.0, is_blue=is_blue) * 0.1
            + distance_reward(sim_state, max_distance=0.1, is_blue=is_blue) * 0.1
            + face_ball_orientation_reward(sim_state, is_blue) * 0.05
            + possession_reward(sim_state, is_blue) * 0.1
            # + dribble_reward(sim_state, action, is_blue)
            + kick_reward(sim_state, action, is_blue)
            + ball_toward_goal_reward(sim_state, self.ssl_geometry, is_blue)
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
        world_state_received = None
        while world_state_received is None:
            self.simulator_io.send_proto(WorldState, initial_world_state)
            world_state_received = self.world_state_received_buffer.get(
                block=False, return_cached=False
            )

        # tick the simulator until something happens
        world = None
        while world is None:
            tick = SimulatorTick(milliseconds=100)
            self.simulator_io.send_proto(SimulatorTick, tick)
            world = self.simulator_state_buffer.get(block=False, return_cached=False)

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
        return create_observation(self.sim_state, is_blue=False), {}

    def step(self, action):
        primitive_set = self._convert_action_to_primitive_set(self.sim_state, action)
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

        obs = create_observation(self.sim_state, is_blue=False)
        reward = self._compute_reward(self.sim_state, action)
        terminated = is_ball_in_enemy_goal(self.sim_state, self.ssl_geometry)
        truncated = False
        self.last_action = action
        info = {}
        return obs, reward, terminated, truncated, info

    def render(self):
        sim_state = self.simulator_state_buffer.get(block=False)
        return render_simulator(
            sim_state, self.ssl_geometry, self.last_action, is_blue=False
        )

    def close(self):
        self.simulator.__exit__(None, None, None)


# Register the environment
gym.register(
    id="ThunderbotsSimulator-v0",
    entry_point="software.gym_envs.simulator_gym_env:SimulatorGymEnv",
    max_episode_steps=300,
)
