import unittest
import numpy as np
from software.ml.utils import transform_to_robot_frame, get_enemy_goal_area, create_observation
from proto.ssl_vision_geometry_pb2 import SSL_GeometryData


class MockSimRobot:
    def __init__(self, p_x, p_y, yaw, v_x=0.0, v_y=0.0, can_kick_ball=False):
        self.p_x = p_x
        self.p_y = p_y
        self.r_z = yaw
        self.v_x = v_x
        self.v_y = v_y
        self.can_kick_ball = can_kick_ball


class MockBall:
    def __init__(self, p_x, p_y, v_x=0.0, v_y=0.0):
        self.p_x = p_x
        self.p_y = p_y
        self.v_x = v_x
        self.v_y = v_y


class MockSimState:
    def __init__(self, yellow_robots=None, blue_robots=None, ball=None):
        self.yellow_robots = yellow_robots or []
        self.blue_robots = blue_robots or []
        self.ball = ball


class TestTransformToRobotFrame(unittest.TestCase):
    def test_identity_transform(self):
        """Test transformation when point is at robot's position with same orientation"""
        robot = MockSimRobot(1.0, 2.0, 0.0)
        rel_x, rel_y, rel_theta = transform_to_robot_frame(1.0, 2.0, 0.0, robot)
        self.assertAlmostEqual(rel_x, 0.0, places=6)
        self.assertAlmostEqual(rel_y, 0.0, places=6)
        self.assertAlmostEqual(rel_theta, 0.0, places=6)

    def test_translation_only(self):
        """Test pure translation with no rotation"""
        robot = MockSimRobot(0.0, 0.0, 0.0)
        rel_x, rel_y, rel_theta = transform_to_robot_frame(3.0, 4.0, 0.0, robot)
        self.assertAlmostEqual(rel_x, 3.0, places=6)
        self.assertAlmostEqual(rel_y, 4.0, places=6)
        self.assertAlmostEqual(rel_theta, 0.0, places=6)

    def test_rotation_90_degrees(self):
        """Test rotation by 90 degrees"""
        robot = MockSimRobot(0.0, 0.0, np.pi / 2)
        rel_x, rel_y, rel_theta = transform_to_robot_frame(1.0, 0.0, 0.0, robot)
        self.assertAlmostEqual(rel_x, 0.0, places=6)
        self.assertAlmostEqual(rel_y, -1.0, places=6)
        self.assertAlmostEqual(rel_theta, -np.pi / 2, places=6)

    def test_combined_transform(self):
        """Test combined translation and rotation"""
        robot = MockSimRobot(1.0, 1.0, np.pi / 4)
        rel_x, rel_y, rel_theta = transform_to_robot_frame(2.0, 2.0, np.pi / 2, robot)
        expected_x = np.sqrt(2)
        expected_y = 0.0
        expected_theta = np.pi / 4
        self.assertAlmostEqual(rel_x, expected_x, places=6)
        self.assertAlmostEqual(rel_y, expected_y, places=6)
        self.assertAlmostEqual(rel_theta, expected_theta, places=6)


class TestGetEnemyGoalArea(unittest.TestCase):
    def test_standard_field(self):
        """Test with standard SSL field dimensions"""
        geometry_data = SSL_GeometryData()
        geometry_data.field.field_length = 12000  # 12m in mm
        geometry_data.field.goal_width = 1800  # 1.8m in mm
        geometry_data.field.goal_depth = 180  # 0.18m in mm

        x_min, x_max, y_min, y_max = get_enemy_goal_area(geometry_data)

        self.assertAlmostEqual(x_min, 6.0, places=6)
        self.assertAlmostEqual(x_max, 6.18, places=6)
        self.assertAlmostEqual(y_min, -0.9, places=6)
        self.assertAlmostEqual(y_max, 0.9, places=6)

    def test_fallback_values(self):
        """Test fallback values when geometry data is None"""
        x_min, x_max, y_min, y_max = get_enemy_goal_area(None)

        self.assertEqual(x_min, 6.0)
        self.assertEqual(x_max, 6.18)
        self.assertEqual(y_min, -0.9)
        self.assertEqual(y_max, 0.9)


class TestCreateObservation(unittest.TestCase):
    def test_empty_state(self):
        """Test observation with empty sim state"""
        sim_state = MockSimState()
        obs = create_observation(sim_state, is_blue=False)
        
        self.assertEqual(len(obs), 21)
        # Check default friendly robot data
        np.testing.assert_array_equal(obs[:7], [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        # Check default enemy robot data
        np.testing.assert_array_equal(obs[7:15], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
        # Check default ball data
        np.testing.assert_array_equal(obs[15:21], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_yellow_robot_observation(self):
        """Test observation for yellow robot (is_blue=False)"""
        yellow_robot = MockSimRobot(1.0, 2.0, np.pi/4, 0.5, -0.3, True)
        blue_robot = MockSimRobot(-1.0, 1.0, -np.pi/2, 0.2, 0.1)
        ball = MockBall(0.5, 1.5, 0.1, -0.2)
        sim_state = MockSimState([yellow_robot], [blue_robot], ball)
        
        obs = create_observation(sim_state, is_blue=False)
        
        # Check friendly robot data (yellow)
        expected_friendly = [1.0, 2.0, 0.5, -0.3, np.cos(np.pi/4), np.sin(np.pi/4), 1.0]
        np.testing.assert_array_almost_equal(obs[:7], expected_friendly)
        
        # Check enemy robot data (blue) - relative and global
        expected_enemy = [-2.0, -1.0, -1.0, 1.0, 0.2, 0.1, np.cos(-np.pi/2), np.sin(-np.pi/2)]
        np.testing.assert_array_almost_equal(obs[7:15], expected_enemy)
        
        # Check ball data - relative and global
        expected_ball = [-0.5, -0.5, 0.5, 1.5, 0.1, -0.2]
        np.testing.assert_array_almost_equal(obs[15:21], expected_ball)

    def test_blue_robot_observation(self):
        """Test observation for blue robot (is_blue=True) with coordinate flipping"""
        yellow_robot = MockSimRobot(1.0, 2.0, np.pi/4, 0.5, -0.3)
        blue_robot = MockSimRobot(-1.0, 1.0, -np.pi/2, 0.2, 0.1, True)
        ball = MockBall(0.5, 1.5, 0.1, -0.2)
        sim_state = MockSimState([yellow_robot], [blue_robot], ball)
        
        obs = create_observation(sim_state, is_blue=True)
        
        # Check friendly robot data (blue) - coordinates flipped
        expected_friendly = [1.0, -1.0, -0.2, -0.1, np.cos(-np.pi/2 + np.pi), np.sin(-np.pi/2 + np.pi), 1.0]
        np.testing.assert_array_almost_equal(obs[:7], expected_friendly)
        
        # Check enemy robot data (yellow) - coordinates flipped
        expected_enemy = [-2.0, -1.0, -1.0, -2.0, -0.5, 0.3, np.cos(np.pi/4 + np.pi), np.sin(np.pi/4 + np.pi)]
        np.testing.assert_array_almost_equal(obs[7:15], expected_enemy)
        
        # Check ball data - coordinates flipped
        expected_ball = [-1.5, -0.5, -0.5, -1.5, -0.1, 0.2]
        np.testing.assert_array_almost_equal(obs[15:21], expected_ball)

    def test_missing_robots(self):
        """Test observation with missing enemy robot"""
        yellow_robot = MockSimRobot(1.0, 2.0, 0.0)
        sim_state = MockSimState([yellow_robot], [], None)
        
        obs = create_observation(sim_state, is_blue=False)
        
        # Friendly robot should be populated
        np.testing.assert_array_almost_equal(obs[:7], [1.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        # Enemy robot should be default
        np.testing.assert_array_equal(obs[7:15], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
        # Ball should be default
        np.testing.assert_array_equal(obs[15:21], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
