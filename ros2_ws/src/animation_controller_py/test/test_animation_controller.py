#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd
import rclpy
from animation_controller_py.animation_controller import AnimationControllerPy


class TestAnimationController(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Create a temporary directory for test CSV files
        self.test_dir = tempfile.mkdtemp()

        # Sample joint names (same as in the actual controller)
        self.joint_names = [
            "leg_front_r_1",
            "leg_front_r_2",
            "leg_front_r_3",
            "leg_front_l_1",
            "leg_front_l_2",
            "leg_front_l_3",
            "leg_back_r_1",
            "leg_back_r_2",
            "leg_back_r_3",
            "leg_back_l_1",
            "leg_back_l_2",
            "leg_back_l_3",
        ]

        # Create test CSV files
        self.create_test_csv_files()

    def tearDown(self):
        # Clean up temporary files
        import shutil

        shutil.rmtree(self.test_dir)
    
    def create_test_csv_files(self):
        """Create test CSV files with different scenarios."""

        # Test CSV 1: Simple animation with 3 keyframes
        csv1_data = {
            "timestamp_ns": [0, 1000000000, 2000000000],
            "leg_front_r_1": [0.0, 0.5, 1.0],
            "leg_front_r_2": [0.1, 0.6, 1.1],
            "leg_front_r_3": [0.2, 0.7, 1.2],
            "leg_front_l_1": [0.3, 0.8, 1.3],
            "leg_front_l_2": [0.4, 0.9, 1.4],
            "leg_front_l_3": [0.5, 1.0, 1.5],
            "leg_back_r_1": [0.6, 1.1, 1.6],
            "leg_back_r_2": [0.7, 1.2, 1.7],
            "leg_back_r_3": [0.8, 1.3, 1.8],
            "leg_back_l_1": [0.9, 1.4, 1.9],
            "leg_back_l_2": [1.0, 1.5, 2.0],
            "leg_back_l_3": [1.1, 1.6, 2.1],
        }
        df1 = pd.DataFrame(csv1_data)
        df1.to_csv(Path(self.test_dir) / "test_animation_1.csv", index=False)

        # Test CSV 2: Single keyframe animation
        csv2_data = {
            "leg_front_r_1": [0.5],
            "leg_front_r_2": [0.5],
            "leg_front_r_3": [0.5],
            "leg_front_l_1": [0.5],
            "leg_front_l_2": [0.5],
            "leg_front_l_3": [0.5],
            "leg_back_r_1": [0.5],
            "leg_back_r_2": [0.5],
            "leg_back_r_3": [0.5],
            "leg_back_l_1": [0.5],
            "leg_back_l_2": [0.5],
            "leg_back_l_3": [0.5],
        }
        df2 = pd.DataFrame(csv2_data)
        df2.to_csv(Path(self.test_dir) / "test_animation_2.csv", index=False)

        # Test CSV 3: With comments
        csv3_path = Path(self.test_dir) / "test_animation_3.csv"
        with open(csv3_path, "w") as f:
            f.write("# This is a comment\n")
            f.write("# Another comment\n")
            f.write("leg_front_r_1,leg_front_r_2,leg_front_r_3,leg_front_l_1,leg_front_l_2,leg_front_l_3,")
            f.write("leg_back_r_1,leg_back_r_2,leg_back_r_3,leg_back_l_1,leg_back_l_2,leg_back_l_3\n")
            f.write("0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1\n")
            f.write("1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0,2.1\n")


class TestCSVLoading(TestAnimationController):
    """Test CSV loading functionality."""

    def test_load_simple_csv(self):
        """Test loading a simple CSV file."""
        # Create a mock controller for testing
        node = rclpy.create_node("test_node")
        controller = AnimationControllerPy.__new__(AnimationControllerPy)
        controller.joint_names = self.joint_names
        controller.animations = {}
        controller.get_logger = lambda: node.get_logger()

        # Test loading
        csv_path = Path(self.test_dir) / "test_animation_1.csv"
        success = controller.load_animation_csv("test1", str(csv_path))

        self.assertTrue(success)
        self.assertIn("test1", controller.animations)

        # Check data integrity
        animation_data = controller.animations["test1"]
        self.assertEqual(animation_data.shape, (3, 12))  # 3 keyframes, 12 joints

        # Check first keyframe values
        expected_first_frame = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1])
        np.testing.assert_array_almost_equal(animation_data[0], expected_first_frame)

        # Check last keyframe values
        expected_last_frame = np.array([1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1])
        np.testing.assert_array_almost_equal(animation_data[2], expected_last_frame)

        node.destroy_node()
    
    def test_load_csv_with_comments(self):
        """Test loading CSV with comments."""
        node = rclpy.create_node("test_node")
        controller = AnimationControllerPy.__new__(AnimationControllerPy)
        controller.joint_names = self.joint_names
        controller.animations = {}
        controller.get_logger = lambda: node.get_logger()

        csv_path = Path(self.test_dir) / "test_animation_3.csv"
        success = controller.load_animation_csv("test3", str(csv_path))

        self.assertTrue(success)
        self.assertEqual(controller.animations["test3"].shape, (2, 12))

        node.destroy_node()

    def test_load_single_keyframe_csv(self):
        """Test loading CSV with single keyframe."""
        node = rclpy.create_node("test_node")
        controller = AnimationControllerPy.__new__(AnimationControllerPy)
        controller.joint_names = self.joint_names
        controller.animations = {}
        controller.get_logger = lambda: node.get_logger()

        csv_path = Path(self.test_dir) / "test_animation_2.csv"
        success = controller.load_animation_csv("test2", str(csv_path))

        self.assertTrue(success)
        self.assertEqual(controller.animations["test2"].shape, (1, 12))

        # All joints should be 0.5
        expected_frame = np.array([0.5] * 12)
        np.testing.assert_array_almost_equal(controller.animations["test2"][0], expected_frame)

        node.destroy_node()

    def test_load_csv_missing_joints(self):
        """Test loading CSV with missing joint columns."""
        node = rclpy.create_node("test_node")
        controller = AnimationControllerPy.__new__(AnimationControllerPy)
        controller.joint_names = self.joint_names
        controller.animations = {}
        controller.get_logger = lambda: node.get_logger()

        # Create CSV with missing joints
        incomplete_csv = Path(self.test_dir) / "incomplete.csv"
        with open(incomplete_csv, "w") as f:
            f.write("leg_front_r_1,leg_front_r_2\n")
            f.write("0.0,0.1\n")

        success = controller.load_animation_csv("incomplete", str(incomplete_csv))
        self.assertFalse(success)

        node.destroy_node()


class TestInterpolation(TestAnimationController):
    """Test keyframe interpolation functionality."""

    def setUp(self):
        super().setUp()

        # Create a mock controller with test data
        self.node = rclpy.create_node("test_interpolation_node")
        self.controller = AnimationControllerPy.__new__(AnimationControllerPy)
        self.controller.joint_names = self.joint_names
        self.controller.current_animation_name = "test_animation"
        self.controller.get_logger = lambda: self.node.get_logger()

        # Create test animation data: 3 keyframes
        self.test_keyframes = np.array([
            [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1],  # Frame 0
            [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1],  # Frame 1
            [2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1],  # Frame 2
        ])
        self.controller.animations = {"test_animation": self.test_keyframes}

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()
    
    def test_interpolation_alpha_zero(self):
        """Test interpolation with alpha = 0 (should return first frame)."""
        result = self.controller.interpolate_keyframes(0.0, 0, 1)
        np.testing.assert_array_almost_equal(result, self.test_keyframes[0])

    def test_interpolation_alpha_one(self):
        """Test interpolation with alpha = 1 (should return second frame)."""
        result = self.controller.interpolate_keyframes(1.0, 0, 1)
        np.testing.assert_array_almost_equal(result, self.test_keyframes[1])

    def test_interpolation_alpha_half(self):
        """Test interpolation with alpha = 0.5 (should return midpoint)."""
        result = self.controller.interpolate_keyframes(0.5, 0, 1)
        expected = (self.test_keyframes[0] + self.test_keyframes[1]) / 2.0
        np.testing.assert_array_almost_equal(result, expected)

    def test_interpolation_alpha_quarter(self):
        """Test interpolation with alpha = 0.25."""
        result = self.controller.interpolate_keyframes(0.25, 0, 1)
        expected = self.test_keyframes[0] * 0.75 + self.test_keyframes[1] * 0.25
        np.testing.assert_array_almost_equal(result, expected)

    def test_interpolation_alpha_clamping(self):
        """Test that alpha values are clamped to [0, 1]."""
        # Test alpha > 1
        result_high = self.controller.interpolate_keyframes(1.5, 0, 1)
        np.testing.assert_array_almost_equal(result_high, self.test_keyframes[1])

        # Test alpha < 0
        result_low = self.controller.interpolate_keyframes(-0.5, 0, 1)
        np.testing.assert_array_almost_equal(result_low, self.test_keyframes[0])

    def test_interpolation_frame_clamping(self):
        """Test that frame indices are clamped to valid range."""
        # Test with frame indices out of bounds
        result = self.controller.interpolate_keyframes(0.5, 10, 20)  # Frames don't exist
        expected = (self.test_keyframes[2] + self.test_keyframes[2]) / 2.0  # Should clamp to last frame
        np.testing.assert_array_almost_equal(result, expected)

    def test_interpolation_same_frames(self):
        """Test interpolation between the same frame."""
        result = self.controller.interpolate_keyframes(0.7, 1, 1)
        np.testing.assert_array_almost_equal(result, self.test_keyframes[1])


class TestAnimationLogic(TestAnimationController):
    """Test animation switching and control logic."""

    def setUp(self):
        super().setUp()

        self.node = rclpy.create_node("test_animation_logic_node")
        self.controller = AnimationControllerPy.__new__(AnimationControllerPy)
        self.controller.joint_names = self.joint_names
        self.controller.animations = {}
        self.controller.current_animation_name = None
        self.controller.animation_start_time = None
        self.controller.init_start_time = None
        self.controller.current_joint_positions = None  # Add new attribute for joint positions
        self.controller.last_joint_states_time = None  # Add new attribute for joint states time
        self.controller.init_positions = None  # Add new attribute for init positions
        self.controller.get_logger = lambda: self.node.get_logger()

        # Mock the switch_to_animation_mode method to avoid service calls in tests
        self.controller.switch_to_animation_mode = lambda: None

        # Add test animations
        self.controller.animations["anim1"] = np.random.rand(5, 12)  # 5 frames
        self.controller.animations["anim2"] = np.random.rand(3, 12)  # 3 frames

    def tearDown(self):
        super().tearDown()
        self.node.destroy_node()

    def test_switch_animation_valid(self):
        """Test switching to a valid animation."""
        self.controller.switch_animation("anim1")

        self.assertEqual(self.controller.current_animation_name, "anim1")
        self.assertIsNone(self.controller.animation_start_time)  # Not started until after init
        self.assertIsNotNone(self.controller.init_start_time)  # Init phase started

    def test_switch_animation_invalid(self):
        """Test switching to an invalid animation."""
        original_name = self.controller.current_animation_name
        self.controller.switch_animation("nonexistent")

        # Should not change current animation
        self.assertEqual(self.controller.current_animation_name, original_name)

    def test_switch_animation_restart_same(self):
        """Test restarting the same animation."""
        self.controller.current_animation_name = "anim1"
        self.controller.animation_start_time = 1234.5  # Some arbitrary start time
        self.controller.init_start_time = None  # Not in init phase

        self.controller.switch_animation("anim1")

        # Should reset animation state
        self.assertIsNone(self.controller.animation_start_time)  # Animation not started yet
        self.assertIsNotNone(self.controller.init_start_time)  # Back to init phase
        self.assertEqual(self.controller.current_animation_name, "anim1")  # Still the same animation

    def test_controller_switching_attributes(self):
        """Test that controller switching is called when switching animations."""
        self.assertIsNotNone(self.controller.switch_to_animation_mode)
        
        # Test that switching logic is called when switching animations
        switch_called = False
        def mock_switch():
            nonlocal switch_called
            switch_called = True
        
        self.controller.switch_to_animation_mode = mock_switch
        self.controller.switch_animation("anim1")
        
        # Should have called the switch method
        self.assertTrue(switch_called)


if __name__ == "__main__":
    unittest.main()