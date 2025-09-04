import gymnasium as gym
import numpy.random
from gymnasium import spaces
import numpy as np
from enum import IntEnum

from tensorboard.compat.tensorflow_stub.dtypes import uint8

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
from software.ml.utils import create_observation, ActionIndex, ObservationIndex, convert_action_to_primitive_set
from software.ml.render import render_simulator


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

        # Define action space as MultiDiscrete.
        # 4 directions, positive/negative orientation, auto-dribble, auto-kick
        self.action_space = spaces.MultiDiscrete(nvec=[4, 2, 1, 1])
        # Define observation space as Box
        self.observation_space = spaces.Box(
            low=-10, high=10, shape=(int(ObservationIndex.SIZE),), dtype=np.float32
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


    def _compute_reward(self, sim_state, is_blue=False):
        return (
            goal_reward(sim_state, self.ssl_geometry, is_blue) * 10
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
        return create_observation(self.sim_state, self.ssl_geometry, 0, is_blue=False), {}

    def step(self, action):
        SIM_TICKS_PER_ACTION = 5
        primitive_set, self.primitive_seq = convert_action_to_primitive_set(self.sim_state, action, self.primitive_seq)
        self.yellow_io.send_proto(PrimitiveSet, primitive_set)
        # tick the simulator at 100hz but only plan at 20hz
        for i in range(SIM_TICKS_PER_ACTION):
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

        obs = create_observation(self.sim_state, self.ssl_geometry, 0, is_blue=False)
        reward = self._compute_reward(self.sim_state)
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
