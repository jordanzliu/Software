import unittest
from software.ml.reward_functions import (
    is_ball_in_enemy_goal,
    position_reward,
    goal_reward,
    distance_reward,
    possession_reward,
    face_ball_orientation_reward,
    dribble_reward,
    kick_reward,
)
from proto.ssl_vision_geometry_pb2 import SSL_GeometryData


class MockBall:
    def __init__(self, p_x, p_y):
        self.p_x = p_x
        self.p_y = p_y


class MockRobot:
    def __init__(self, p_x, p_y, r_z=0.0, can_kick_ball=False):
        self.p_x = p_x
        self.p_y = p_y
        self.r_z = r_z
        self.can_kick_ball = can_kick_ball


class MockSimState:
    def __init__(self, ball=None, yellow_robots=None):
        self.ball = ball
        self.yellow_robots = yellow_robots or []


class TestRewardFunctions(unittest.TestCase):
    def setUp(self):
        self.geometry = SSL_GeometryData()
        self.geometry.field.field_length = 12000
        self.geometry.field.goal_width = 1800
        self.geometry.field.goal_depth = 180

    def test_is_ball_in_enemy_goal_true(self):
        ball = MockBall(6.1, 0.0)
        sim_state = MockSimState(ball=ball)
        self.assertTrue(is_ball_in_enemy_goal(sim_state, self.geometry))

    def test_is_ball_in_enemy_goal_false(self):
        ball = MockBall(0.0, 0.0)
        sim_state = MockSimState(ball=ball)
        self.assertFalse(is_ball_in_enemy_goal(sim_state, self.geometry))

    def test_position_reward_center(self):
        ball = MockBall(0.0, 0.0)
        sim_state = MockSimState(ball=ball)
        reward = position_reward(sim_state, self.geometry)
        self.assertAlmostEqual(reward, 0.5, places=6)

    def test_position_reward_enemy_goal(self):
        ball = MockBall(6.0, 0.0)
        sim_state = MockSimState(ball=ball)
        reward = position_reward(sim_state, self.geometry)
        self.assertAlmostEqual(reward, 1.0, places=6)

    def test_goal_reward_in_goal(self):
        ball = MockBall(6.1, 0.0)
        sim_state = MockSimState(ball=ball)
        reward = goal_reward(sim_state, self.geometry)
        self.assertEqual(reward, 1.0)

    def test_goal_reward_not_in_goal(self):
        ball = MockBall(0.0, 0.0)
        sim_state = MockSimState(ball=ball)
        reward = goal_reward(sim_state, self.geometry)
        self.assertEqual(reward, 0.0)

    def test_distance_reward_close(self):
        ball = MockBall(0.0, 0.0)
        robot = MockRobot(0.5, 0.0)
        sim_state = MockSimState(ball=ball, yellow_robots=[robot])
        reward = distance_reward(sim_state)
        self.assertAlmostEqual(reward, 0.5, places=6)

    def test_distance_reward_far(self):
        ball = MockBall(0.0, 0.0)
        robot = MockRobot(2.0, 0.0)
        sim_state = MockSimState(ball=ball, yellow_robots=[robot])
        reward = distance_reward(sim_state)
        self.assertEqual(reward, 0.0)

    def test_possession_reward_can_kick(self):
        robot = MockRobot(0.0, 0.0, can_kick_ball=True)
        ball = MockBall(0.0, 0.0)
        sim_state = MockSimState(yellow_robots=[robot], ball=ball)
        reward = possession_reward(sim_state)
        self.assertEqual(reward, 1.0)

    def test_possession_reward_cannot_kick(self):
        robot = MockRobot(0.0, 0.0, can_kick_ball=False)
        sim_state = MockSimState(yellow_robots=[robot])
        reward = possession_reward(sim_state)
        self.assertEqual(reward, 0.0)

    def test_face_ball_orientation_reward_facing(self):
        ball = MockBall(1.0, 0.0)
        robot = MockRobot(0.0, 0.0, r_z=0.0)
        sim_state = MockSimState(ball=ball, yellow_robots=[robot])
        reward = face_ball_orientation_reward(sim_state)
        self.assertAlmostEqual(reward, 1.0, places=6)

    def test_face_ball_orientation_reward_opposite(self):
        ball = MockBall(-1.0, 0.0)
        robot = MockRobot(0.0, 0.0, r_z=0.0)
        sim_state = MockSimState(ball=ball, yellow_robots=[robot])
        reward = face_ball_orientation_reward(sim_state)
        self.assertAlmostEqual(reward, -1.0, places=6)

    def test_dribble_reward_active(self):
        robot = MockRobot(0.0, 0.0, can_kick_ball=True)
        sim_state = MockSimState(ball=MockBall(0.0, 0.0), yellow_robots=[robot])
        action = [0, 0, 0, 0, 1.0]
        reward = dribble_reward(sim_state, action)
        self.assertEqual(reward, 1.0)

    def test_dribble_reward_inactive(self):
        robot = MockRobot(0.0, 0.0, can_kick_ball=True)
        sim_state = MockSimState(ball=MockBall(0.0, 0.0), yellow_robots=[robot])
        action = [0, 0, 0, 0, 0.0]
        reward = dribble_reward(sim_state, action)
        self.assertEqual(reward, 0.0)

    def test_kick_reward_active(self):
        robot = MockRobot(0.0, 0.0, can_kick_ball=True)
        sim_state = MockSimState(ball=MockBall(0.0, 0.0), yellow_robots=[robot])
        action = [0, 0, 0, 1.0, 0]
        reward = kick_reward(sim_state, action)
        self.assertEqual(reward, 1.0)

    def test_kick_reward_inactive(self):
        robot = MockRobot(0.0, 0.0, can_kick_ball=True)
        sim_state = MockSimState(ball=MockBall(0.0, 0.0), yellow_robots=[robot])
        action = [0, 0, 0, 0.0, 0]
        reward = kick_reward(sim_state, action)
        self.assertEqual(reward, 0.0)


if __name__ == "__main__":
    unittest.main()
