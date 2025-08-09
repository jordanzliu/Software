import numpy.random
from gymnasium import spaces
import numpy as np
from ray.rllib.env.multi_agent_env import MultiAgentEnv

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
from software.ml.utils import render_simulator


class ActionIndex:
    VELOCITY_X = 0
    VELOCITY_Y = 1
    VELOCITY_ANGULAR = 2
    AUTO_KICK = 3
    AUTO_DRIBBLE = 4


class SelfPlaySimulatorEnv(MultiAgentEnv):
    def __init__(self, simulator_runtime_dir, enable_realism=False):
        super().__init__()
        self.simulator = None
        self.simulator_runtime_dir = simulator_runtime_dir
        self.enable_realism = enable_realism
        self.primitive_seq = {"yellow": 0, "blue": 0}
        self.ssl_geometry = None

        # Define action and observation spaces
        self.action_space = spaces.Box(low=-1, high=1, shape=(5,), dtype=np.float32)
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

    def _get_obs(self, sim_state, is_blue=False):
        robots = sim_state.blue_robots if is_blue else sim_state.yellow_robots
        enemy_robots = sim_state.yellow_robots if is_blue else sim_state.blue_robots

        # Default values
        friendly_data = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], dtype=np.float32)
        enemy_data = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32
        )
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

    def _convert_action_to_primitive_set(self, action, is_blue=False):
        # Scale velocities
        local_vx = action[ActionIndex.VELOCITY_X] * 3.0
        local_vy = action[ActionIndex.VELOCITY_Y] * 3.0
        angular_velocity = action[ActionIndex.VELOCITY_ANGULAR] * 10.0

        robots = self.sim_state.blue_robots if is_blue else self.sim_state.yellow_robots
        # the robot movement is supposed to be in global cartesian coordinates
        velocity_x = -local_vx if is_blue else local_vx
        velocity_y = -local_vy if is_blue else local_vy

        # Create primitive
        velocity_control = MotorControl.DirectVelocityControl()
        velocity_control.velocity.x_component_meters = velocity_x
        velocity_control.velocity.y_component_meters = velocity_y
        velocity_control.angular_velocity.radians_per_second = angular_velocity

        motor_control = MotorControl()
        motor_control.direct_velocity_control.CopyFrom(velocity_control)
        motor_control.dribbler_speed_rpm = (
            12000 if action[ActionIndex.AUTO_DRIBBLE] > 0.5 else 0
        )

        chicker_control = PowerControl.ChickerControl()
        auto_kick = AutoChipOrKick()
        auto_kick.autokick_speed_m_per_s = 5
        chicker_control.auto_chip_or_kick.CopyFrom(auto_kick)
        power_control = PowerControl()
        if action[ActionIndex.AUTO_KICK] > 0.5:
            power_control.chicker.CopyFrom(chicker_control)

        control_primitive = DirectControlPrimitive()
        control_primitive.motor_control.CopyFrom(motor_control)
        control_primitive.power_control.CopyFrom(power_control)

        primitive = Primitive()
        primitive.direct_control.CopyFrom(control_primitive)

        agent_key = "blue" if is_blue else "yellow"
        primitive.sequence_number = self.primitive_seq[agent_key]
        self.primitive_seq[agent_key] += 1
        primitive.time_sent.CopyFrom(Timestamp())

        primitive_set = PrimitiveSet()
        primitive_set.robot_primitives[0].CopyFrom(primitive)
        primitive_set.time_sent.CopyFrom(Timestamp())

        return primitive_set

    def _compute_reward(self, sim_state, action, is_blue=False):
        return (
            goal_reward(sim_state, self.ssl_geometry, is_blue) * 100
            + distance_reward(sim_state, is_blue) * 0.1
            + face_ball_orientation_reward(sim_state, is_blue) * 0.1
            + possession_reward(sim_state, is_blue)
            + dribble_reward(sim_state, action, is_blue) * 5
            + kick_reward(sim_state, action, is_blue) * 10
        )

    def reset(self, *, seed=None, options=None):
        if self.simulator is None:
            self.simulator = Simulator(
                self.simulator_runtime_dir, enable_realism=self.enable_realism
            )
            self.simulator.__enter__()

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

        # Reset world
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

        # Tick simulator
        world = None
        while world is None:
            tick = SimulatorTick(milliseconds=100)
            self.simulator_io.send_proto(SimulatorTick, tick)
            world = self.simulator_state_buffer.get(block=False, return_cached=False)

        tick = SimulatorTick(milliseconds=100)
        self.simulator_io.send_proto(SimulatorTick, tick)
        self.sim_state = self.simulator_state_buffer.get(
            block=True, return_cached=False
        )
        self.ssl_geometry = self.ssl_wrapper_buffer.get(
            block=True, return_cached=False
        ).geometry

        return {
            "yellow": self._get_obs(self.sim_state, is_blue=False),
            "blue": self._get_obs(self.sim_state, is_blue=True),
        }, {}

    def step(self, action_dict):
        # Send actions for both agents
        if "yellow" in action_dict:
            primitive_set = self._convert_action_to_primitive_set(
                action_dict["yellow"], is_blue=False
            )
            self.yellow_io.send_proto(PrimitiveSet, primitive_set)

        if "blue" in action_dict:
            primitive_set = self._convert_action_to_primitive_set(
                action_dict["blue"], is_blue=True
            )
            self.blue_io.send_proto(PrimitiveSet, primitive_set)

        # Tick simulator
        for i in range(9):
            tick = SimulatorTick(milliseconds=10)
            self.simulator_io.send_proto(SimulatorTick, tick)

        # Clear buffer
        simulator_state = self.simulator_state_buffer.get(
            block=False, return_cached=False
        )
        while simulator_state is not None:
            simulator_state = self.simulator_state_buffer.get(
                block=False, return_cached=False
            )

        # Final tick
        tick = SimulatorTick(milliseconds=10)
        self.simulator_io.send_proto(SimulatorTick, tick)
        self.sim_state = self.simulator_state_buffer.get(
            block=True, return_cached=False
        )
        self.ssl_geometry = self.ssl_wrapper_buffer.get(
            block=True, return_cached=False
        ).geometry

        # Compute observations and rewards
        obs = {
            "yellow": self._get_obs(self.sim_state, is_blue=False),
            "blue": self._get_obs(self.sim_state, is_blue=True),
        }

        rewards = {}
        if "yellow" in action_dict:
            rewards["yellow"] = self._compute_reward(
                self.sim_state, action_dict["yellow"], is_blue=False
            )
        if "blue" in action_dict:
            rewards["blue"] = self._compute_reward(
                self.sim_state, action_dict["blue"], is_blue=True
            )

        # Check termination
        yellow_goal = is_ball_in_enemy_goal(
            self.sim_state, self.ssl_geometry, is_blue=False
        )
        blue_goal = is_ball_in_enemy_goal(
            self.sim_state, self.ssl_geometry, is_blue=True
        )
        terminated = yellow_goal or blue_goal

        terminateds = {"yellow": terminated, "blue": terminated, "__all__": terminated}
        truncateds = {"yellow": False, "blue": False, "__all__": False}
        infos = {"yellow": {}, "blue": {}}

        return obs, rewards, terminateds, truncateds, infos

    def render(self):
        sim_state = self.simulator_state_buffer.get(block=False)
        return render_simulator(sim_state, self.ssl_geometry)

    def close(self):
        if self.simulator:
            self.simulator.__exit__(None, None, None)
