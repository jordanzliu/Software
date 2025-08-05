import unittest
import numpy as np
from software.ml.utils import transform_to_robot_frame


class MockSimRobot:
    def __init__(self, p_x, p_y, yaw):
        self.p_x = p_x
        self.p_y = p_y
        self.r_z = yaw


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


if __name__ == "__main__":
    unittest.main()
