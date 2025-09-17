#!/usr/bin/env python3
"""
Mock camera node that publishes test images to /camera/image_raw and /camera/image_raw/compressed
Cycles through all images in the config folder
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import glob
from ament_index_python.packages import get_package_share_directory


class MockCameraNode(Node):
    def __init__(self):
        super().__init__("mock_camera_node")

        # Declare parameters
        self.declare_parameter("image_folder", "config")  # Folder containing images
        self.declare_parameter("cycle_images", True)  # Whether to cycle through images
        self.declare_parameter("cycle_interval", 1.0)  # Seconds between image changes
        self.declare_parameter("publish_rate", 10.0)  # Hz
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("jpeg_quality", 95)  # JPEG compression quality (0-100)

        # Get parameters
        image_folder = self.get_parameter("image_folder").value
        self.cycle_images = self.get_parameter("cycle_images").value
        self.cycle_interval = self.get_parameter("cycle_interval").value
        publish_rate = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value

        # Get the path to the image folder
        try:
            package_dir = get_package_share_directory("hailo")
            self.image_folder = os.path.join(package_dir, image_folder)
        except Exception:
            # Fallback for development - look in the source directory
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.image_folder = os.path.join(os.path.dirname(current_dir), image_folder)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create publishers
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.compressed_pub = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", 10)

        # Load all images
        self.images = []
        self.image_paths = []
        self.current_image_index = 0
        self.load_images()

        # Create timer to publish at specified rate
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Create timer to cycle images if enabled
        if self.cycle_images and len(self.images) > 1:
            self.cycle_timer = self.create_timer(self.cycle_interval, self.cycle_image_callback)
            self.get_logger().info(f"Cycling through {len(self.images)} images every {self.cycle_interval} seconds")

        self.get_logger().info(f"Mock camera node started, publishing at {publish_rate} Hz")
        self.get_logger().info(
            f"Publishing to /camera/image_raw and /camera/image_raw/compressed (quality={self.jpeg_quality})"
        )

    def load_images(self):
        """Load all images from the configured folder."""
        # Find all image files in the folder
        image_patterns = ["*.png", "*.jpg", "*.jpeg", "*.bmp"]
        image_files = []

        for pattern in image_patterns:
            image_files.extend(glob.glob(os.path.join(self.image_folder, pattern)))

        # Sort for consistent ordering
        image_files.sort()

        if not image_files:
            self.get_logger().warn(f"No image files found in: {self.image_folder}")
            self.get_logger().info("Creating a default test pattern image...")

            # Create a default test pattern if no images found
            height, width = 480, 640
            test_image = np.zeros((height, width, 3), dtype=np.uint8)

            # Add some colored rectangles
            cv2.rectangle(test_image, (0, 0), (width // 2, height // 2), (255, 0, 0), -1)
            cv2.rectangle(test_image, (width // 2, 0), (width, height // 2), (0, 255, 0), -1)
            cv2.rectangle(test_image, (0, height // 2), (width // 2, height), (0, 0, 255), -1)
            cv2.rectangle(test_image, (width // 2, height // 2), (width, height), (255, 255, 0), -1)

            # Add text
            cv2.putText(
                test_image,
                "MOCK CAMERA",
                (width // 2 - 100, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                test_image,
                "No images found",
                (width // 2 - 80, height // 2 + 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            self.images.append(test_image)
            self.image_paths.append("default_test_pattern")
        else:
            # Load all found images
            for image_path in image_files:
                img = cv2.imread(image_path)
                if img is not None:
                    self.images.append(img)
                    self.image_paths.append(image_path)
                    self.get_logger().info(f"Loaded image: {os.path.basename(image_path)} - Shape: {img.shape}")
                else:
                    self.get_logger().warn(f"Failed to load image: {image_path}")

        self.get_logger().info(f"Total images loaded: {len(self.images)} from {self.image_folder}")

    def cycle_image_callback(self):
        """Callback to cycle to the next image."""
        if len(self.images) > 1:
            self.current_image_index = (self.current_image_index + 1) % len(self.images)
            image_name = os.path.basename(self.image_paths[self.current_image_index])
            self.get_logger().info(
                f"Switched to image {self.current_image_index + 1}/{len(self.images)}: {image_name}"
            )

    def timer_callback(self):
        """Publish the current image in both raw and compressed formats."""
        if not self.images:
            return

        current_time = self.get_clock().now().to_msg()
        current_image = self.images[self.current_image_index]

        # Publish raw image
        raw_msg = self.bridge.cv2_to_imgmsg(current_image, encoding="bgr8")
        raw_msg.header.stamp = current_time
        raw_msg.header.frame_id = self.frame_id
        self.image_pub.publish(raw_msg)

        # Create and publish compressed image
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = current_time
        compressed_msg.header.frame_id = self.frame_id
        compressed_msg.format = "jpeg"

        # Encode image as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        _, jpeg_data = cv2.imencode(".jpg", current_image, encode_param)
        compressed_msg.data = jpeg_data.tobytes()

        self.compressed_pub.publish(compressed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
