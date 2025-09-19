#!/usr/bin/env python3

import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import pandas as pd
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class AnimationControllerPy(Node):
    """Python implementation of animation controller that publishes to forward command controllers."""

    ACTION_SIZE = 12

    def __init__(self):
        super().__init__("animation_controller_py")

        # Parameters
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("init_duration", 2.0)

        self.declare_parameter(
            "joint_names",
            [
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
            ],
        )

        self.declare_parameter("kps", [5.0] * self.ACTION_SIZE)
        self.declare_parameter("kds", [0.25] * self.ACTION_SIZE)
        self.declare_parameter("init_kps", [7.5] * self.ACTION_SIZE)
        self.declare_parameter("init_kds", [0.25] * self.ACTION_SIZE)

        # Get parameters
        self.frame_rate = self.get_parameter("frame_rate").value
        self.init_duration = self.get_parameter("init_duration").value

        self.joint_names = self.get_parameter("joint_names").value
        self.kps = np.array(self.get_parameter("kps").value)
        self.kds = np.array(self.get_parameter("kds").value)
        self.init_kps = np.array(self.get_parameter("init_kps").value)
        self.init_kds = np.array(self.get_parameter("init_kds").value)

        # Validate parameters
        if len(self.joint_names) != self.ACTION_SIZE:
            raise ValueError(f"joint_names must have {self.ACTION_SIZE} elements")
        if len(self.kps) != self.ACTION_SIZE:
            raise ValueError(f"kps must have {self.ACTION_SIZE} elements")
        if len(self.kds) != self.ACTION_SIZE:
            raise ValueError(f"kds must have {self.ACTION_SIZE} elements")
        if self.frame_rate <= 0:
            raise ValueError("frame_rate must be > 0")

        # Animation data
        self.animations: Dict[str, np.ndarray] = {}
        self.current_animation_name: Optional[str] = None

        # Animation state - None means unstarted/done, float means animation start time
        self.animation_start_time: Optional[float] = None
        self.init_start_time: Optional[float] = None

        # Target positions
        self.init_positions: Optional[np.ndarray] = None
        self.current_joint_positions: Optional[np.ndarray] = None
        self.last_joint_states_time: Optional[float] = None

        # Publishers for the three forward command controllers
        self.position_publisher = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)

        self.kp_publisher = self.create_publisher(Float64MultiArray, "/forward_kp_controller/commands", 10)

        self.kd_publisher = self.create_publisher(Float64MultiArray, "/forward_kd_controller/commands", 10)

        # Subscriber for animation selection
        self.animation_select_subscriber = self.create_subscription(
            String, "~/animation_select", self.animation_select_callback, 10
        )

        # Subscriber for throttled (eg 10hz not 250hz) joint states
        # Used in future to detect which animations to perform based on current pose
        self.joint_states_subscriber = self.create_subscription(
            JointState, "/joint_states_throttled", self.joint_states_callback, 10
        )

        # Service client for controller switching
        self.controller_switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

        # Controller names
        self.neural_controllers = ["neural_controller", "neural_controller_three_legged"]
        self.forward_controllers = ["forward_position_controller", "forward_kp_controller", "forward_kd_controller"]

        # Timer for control loop
        self.control_timer = self.create_timer(1.0 / 120.0, self.control_loop)  # 120 Hz

        # Load animations
        if not self.load_all_animations():
            raise RuntimeError("Failed to load animations")

        self.get_logger().info(f"Animation controller initialized with {len(self.animations)} animations")
        self.get_logger().info("Waiting for joint_states messages to initialize current positions...")

    def load_all_animations(self) -> bool:
        """Load all CSV animation files from the animation_controller_py package."""
        try:
            package_path = get_package_share_directory("animation_controller_py")
            launch_dir = Path(package_path) / "launch" / "animations"
        except Exception as e:
            self.get_logger().error(f"Failed to find animation_controller_py package: {e}")
            return False

        if not launch_dir.exists():
            self.get_logger().error(f"Launch directory not found: {launch_dir}")
            return False

        csv_files = list(launch_dir.glob("*.csv"))
        if not csv_files:
            self.get_logger().error(f"No CSV files found in {launch_dir}")
            return False

        for csv_file in csv_files:
            animation_name = csv_file.stem
            if self.load_animation_csv(animation_name, str(csv_file)):
                self.get_logger().info(f"Loaded animation: {animation_name}")
            else:
                self.get_logger().error(f"Failed to load animation: {animation_name}")
                return False

        self.get_logger().info(f"Successfully loaded {len(self.animations)} animations")
        return True

    def load_animation_csv(self, name: str, csv_path: str) -> bool:
        """Load animation data from a CSV file using pandas."""
        try:
            # Read CSV with pandas
            df = pd.read_csv(csv_path, comment="#")

            # Check if all required joints are present
            missing_joints = [joint for joint in self.joint_names if joint not in df.columns]
            if missing_joints:
                self.get_logger().error(f"Missing joints in CSV header: {missing_joints}")
                return False

            if df.empty:
                self.get_logger().error(f"No keyframes loaded from {csv_path}")
                return False

            # Extract only the joint columns in the correct order and convert to numpy array
            joint_data = df[self.joint_names].values

            self.animations[name] = joint_data
            self.get_logger().info(f"Loaded {len(joint_data)} keyframes for animation '{name}'")
            return True

        except Exception as e:
            self.get_logger().error(f"Error loading CSV file {csv_path}: {e}")
            return False

    def joint_states_callback(self, msg: JointState):
        """Handle joint states updates."""
        try:
            # Create mapping from joint name to position
            joint_position_map = {name: pos for name, pos in zip(msg.name, msg.position)}

            # Extract positions for our joints in the correct order
            positions = []
            for joint_name in self.joint_names:
                if joint_name in joint_position_map:
                    positions.append(joint_position_map[joint_name])
                else:
                    self.get_logger().warn(f"Joint '{joint_name}' not found in joint_states")
                    positions.append(0.0)

            self.current_joint_positions = np.array(positions)
            self.last_joint_states_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Error processing joint_states: {e}")

    def animation_select_callback(self, msg: String):
        """Handle animation selection requests."""
        self.switch_animation(msg.data)

    def switch_to_animation_mode(self):
        """Switch from neural controllers to forward command controllers."""

        self.get_logger().info("Switching to animation mode...")

        # Wait for service to be available
        if not self.controller_switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Controller switch service not available")
            return

        # Create switch request
        request = SwitchController.Request()
        request.activate_controllers = self.forward_controllers
        request.deactivate_controllers = self.neural_controllers
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()

        try:
            future = self.controller_switch_client.call_async(request)
            future.add_done_callback(self._switch_to_animation_callback)

        except Exception as e:
            self.get_logger().error(f"Error switching controllers: {e}")

    def _switch_to_animation_callback(self, future):
        """Callback for animation mode switch."""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info("Successfully switched to animation mode")
            else:
                self.get_logger().error("Failed to switch to animation mode")
        except Exception as e:
            self.get_logger().error(f"Error in animation mode switch callback: {e}")

    def switch_animation(self, animation_name: str):
        """Switch to a different animation."""
        if animation_name not in self.animations:
            self.get_logger().warn(f"Animation '{animation_name}' not found")
            return

        # Switch to animation mode since the user may have switched back to neural mode without
        # us knowing
        self.switch_to_animation_mode()

        old_animation = self.current_animation_name if self.current_animation_name else "None"
        self.get_logger().info(f"Starting animation '{animation_name}' (was: {old_animation})")

        # Set new animation
        self.current_animation_name = animation_name

        # Reset state - animation will start after init phase
        self.animation_start_time = None
        self.init_start_time = time.time()

        # Capture current joint positions as initial positions for this animation
        if self.current_joint_positions is not None:
            self.init_positions = self.current_joint_positions.copy()
            self.get_logger().info(
                f"Using current joint positions as initial positions for animation '{animation_name}'"
            )
        else:
            self.init_positions = None
            self.get_logger().warn("No joint_states available, will not play animation")

    def interpolate_keyframes(self, alpha: float, frame_a: int, frame_b: int) -> np.ndarray:
        """Interpolate between two keyframes."""
        alpha = np.clip(alpha, 0.0, 1.0)

        keyframes = self.animations[self.current_animation_name]
        frame_a = min(frame_a, len(keyframes) - 1)
        frame_b = min(frame_b, len(keyframes) - 1)

        return keyframes[frame_a] * (1.0 - alpha) + keyframes[frame_b] * alpha

    def control_loop(self):
        """Main control loop called at 120 Hz."""
        # No animation selected - nothing to do
        if self.current_animation_name is None:
            self.get_logger().debug("No animation selected, skipping control loop")
            return

        current_time = time.time()
        keyframes = self.animations[self.current_animation_name]

        # Phase 1: Initialization - smoothly move to first animation frame
        if self.init_start_time is not None:
            time_since_init = current_time - self.init_start_time
            self.get_logger().debug(f"Initialization phase, time since init: {time_since_init:.2f}s")

            if time_since_init < self.init_duration:
                if self.init_positions is None:
                    self.get_logger().warn("No initial positions set, cannot initialize animation")
                    return

                # Interpolate between init positions and first frame
                alpha = time_since_init / self.init_duration
                interpolated_positions = self.init_positions * (1.0 - alpha) + keyframes[0] * alpha
                self.publish_commands(interpolated_positions, self.init_kps, self.init_kds)
                return
            else:
                # Initialization complete, start animation
                self.animation_start_time = current_time
                self.init_start_time = None  # Clear init state
                self.get_logger().info(f"Starting animation playback: {self.current_animation_name}")

        # Phase 2: Animation playback
        if self.animation_start_time is not None:
            animation_time = current_time - self.animation_start_time
            self.get_logger().debug(f"Animation playback phase, animation time: {animation_time:.2f}s")

            # Calculate current frame position
            frame_time = 1.0 / self.frame_rate
            exact_frame = animation_time / frame_time

            # Check if animation is complete
            if exact_frame >= len(keyframes):
                # Animation done - clear state
                self.get_logger().info(f"Animation '{self.current_animation_name}' completed")
                self.current_animation_name = None
                self.animation_start_time = None
                return

            # Calculate interpolated position
            current_frame_index = int(exact_frame)
            if len(keyframes) > 1 and current_frame_index < len(keyframes) - 1:
                next_frame = current_frame_index + 1
                alpha = exact_frame - current_frame_index
                target_positions = self.interpolate_keyframes(alpha, current_frame_index, next_frame)
            else:
                target_positions = keyframes[min(current_frame_index, len(keyframes) - 1)]

            # Publish commands
            self.publish_commands(target_positions, self.kps, self.kds)

    def publish_commands(self, positions: np.ndarray, kps: np.ndarray, kds: np.ndarray):
        """Publish position, kp, and kd commands to forward controllers."""
        # Position commands
        pos_msg = Float64MultiArray()
        pos_msg.data = positions.tolist()
        self.position_publisher.publish(pos_msg)

        # Kp commands
        kp_msg = Float64MultiArray()
        kp_msg.data = kps.tolist()
        self.kp_publisher.publish(kp_msg)

        # Kd commands
        kd_msg = Float64MultiArray()
        kd_msg.data = kds.tolist()
        self.kd_publisher.publish(kd_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = AnimationControllerPy()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
