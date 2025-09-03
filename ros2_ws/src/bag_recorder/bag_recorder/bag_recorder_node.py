#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess
import threading
import os
import time
from datetime import datetime


class BagRecorderNode(Node):
    def __init__(self):
        super().__init__("bag_recorder_node")

        # Declare parameters for joystick button indices
        self.declare_parameter("record_start_button", 4)  # L1 button
        self.declare_parameter("record_stop_button", 5)  # R1 button
        self.declare_parameter("bag_output_dir", "~/bags")
        self.declare_parameter("start_button_hold_duration", 5.0)  # seconds

        # Get parameters
        self.record_start_button = self.get_parameter("record_start_button").get_parameter_value().integer_value
        self.record_stop_button = self.get_parameter("record_stop_button").get_parameter_value().integer_value
        self.bag_output_dir = self.get_parameter("bag_output_dir").get_parameter_value().string_value
        self.required_press_duration = self.get_parameter("start_button_hold_duration").get_parameter_value().double_value

        # Expand home directory
        self.bag_output_dir = os.path.expanduser(self.bag_output_dir)

        # Create output directory if it doesn't exist
        os.makedirs(self.bag_output_dir, exist_ok=True)

        # State tracking
        self.recording = False
        self.recording_process = None
        self.prev_start_button_state = False
        self.prev_stop_button_state = False
        self.lock = threading.Lock()
        
        # Long press tracking for start button
        self.start_button_press_time = None

        # Subscribe to joy topic
        self.joy_subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.get_logger().info("Bag recorder node initialized")
        self.get_logger().info(f"Start recording button: {self.record_start_button}")
        self.get_logger().info(f"Stop recording button: {self.record_stop_button}")
        self.get_logger().info(f"Start button hold duration: {self.required_press_duration} seconds")
        self.get_logger().info(f"Output directory: {self.bag_output_dir}")

    def get_filtered_topics(self):
        """Get all topics except those containing raw images."""
        try:
            # Get list of all topics
            result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, check=True)
            all_topics = result.stdout.strip().split("\n")

            # Filter out topics that contain raw image data
            # Common raw image topic patterns to exclude
            raw_image_patterns = [
                "/image_raw",
                "/camera/image_raw",
                "image_raw",
                "/rgb/image_raw",
                "/depth/image_raw",
            ]

            filtered_topics = []
            for topic in all_topics:
                topic = topic.strip()
                if topic and not any(pattern in topic.lower() for pattern in raw_image_patterns):
                    filtered_topics.append(topic)

            self.get_logger().info(f"Found {len(all_topics)} total topics, recording {len(filtered_topics)} topics")
            return filtered_topics

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to get topic list: {e}")
            return []

    def start_recording(self):
        """Start MCAP bag recording with filtered topics."""
        with self.lock:
            if self.recording:
                self.get_logger().warn("Already recording!")
                return

            # Get filtered topics
            topics_to_record = self.get_filtered_topics()

            if not topics_to_record:
                self.get_logger().error("No topics to record!")
                return

            # Generate timestamp-based filename
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            bag_filename = f"recording_{timestamp}"
            bag_path = os.path.join(self.bag_output_dir, bag_filename)

            # Build ros2 bag record command
            cmd = ["ros2", "bag", "record", "-o", bag_path, "--storage", "mcap"]
            cmd.extend(topics_to_record)

            try:
                self.recording_process = subprocess.Popen(cmd)
                self.recording = True
                self.get_logger().info(f"Started recording to {bag_path}.mcap")
                self.get_logger().info(f"Recording {len(topics_to_record)} topics")

            except Exception as e:
                self.get_logger().error(f"Failed to start recording: {e}")
                self.recording = False
                self.recording_process = None

    def stop_recording(self):
        """Stop the current bag recording."""
        with self.lock:
            if not self.recording or self.recording_process is None:
                self.get_logger().warn("Not currently recording!")
                return

            try:
                # Send SIGINT to gracefully stop recording
                self.recording_process.terminate()
                self.recording_process.wait(timeout=5)  # Wait up to 5 seconds

                self.get_logger().info("Recording stopped")

            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop gracefully
                self.recording_process.kill()
                self.get_logger().warn("Recording process was force killed")

            except Exception as e:
                self.get_logger().error(f"Error stopping recording: {e}")

            finally:
                self.recording = False
                self.recording_process = None

    def joy_callback(self, msg):
        """Handle joystick input."""
        # Check array bounds
        if len(msg.buttons) <= max(self.record_start_button, self.record_stop_button):
            return

        # Get current button states
        start_button_pressed = msg.buttons[self.record_start_button] == 1
        stop_button_pressed = msg.buttons[self.record_stop_button] == 1
        current_time = time.time()

        # Handle start button with long press requirement
        if start_button_pressed:
            if not self.prev_start_button_state:
                # Button just pressed - start timing
                self.start_button_press_time = current_time
                self.get_logger().info(f"Start recording button {self.record_start_button} pressed - hold for {self.required_press_duration} seconds to start recording")
            elif self.start_button_press_time is not None:
                # Button is being held - check duration
                press_duration = current_time - self.start_button_press_time
                if press_duration >= self.required_press_duration and not self.recording:
                    self.get_logger().info(f"Start recording button held for {self.required_press_duration} seconds - starting recording")
                    self.start_recording()
                    self.start_button_press_time = None  # Reset to prevent multiple starts
        else:
            # Button released - reset timing
            if self.start_button_press_time is not None:
                press_duration = current_time - self.start_button_press_time
                if press_duration < self.required_press_duration:
                    self.get_logger().info(f"Start recording button released after {press_duration:.1f} seconds - recording not started (need {self.required_press_duration} seconds)")
                self.start_button_press_time = None

        # Handle stop button (immediate response)
        if stop_button_pressed and not self.prev_stop_button_state:
            self.get_logger().info(f"Stop recording button {self.record_stop_button} pressed")
            self.stop_recording()

        # Update previous states
        self.prev_start_button_state = start_button_pressed
        self.prev_stop_button_state = stop_button_pressed

    def __del__(self):
        """Cleanup when node is destroyed."""
        if self.recording:
            self.stop_recording()


def main(args=None):
    rclpy.init(args=args)

    bag_recorder = BagRecorderNode()

    try:
        rclpy.spin(bag_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up any ongoing recording
        if bag_recorder.recording:
            bag_recorder.stop_recording()

        bag_recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
