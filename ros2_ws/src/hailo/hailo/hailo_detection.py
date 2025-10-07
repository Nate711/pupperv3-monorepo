#!/usr/bin/env python3
"""
ROS2 node for Hailo object detection and tracking
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image, CompressedImage
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import supervision as sv
import numpy as np
import cv2
import queue
import sys
import os
from typing import Dict, List
import threading
import zmq
import json
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from hailo import fisheye_utils

# Conditional imports based on sim mode
try:
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    from hailo.utils import HailoAsyncInference

    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False


class HailoDetectionNode(Node):
    def __init__(self):
        super().__init__("hailo_detection_node")

        # Declare and get parameters
        self.declare_parameter("sim", False)
        self.declare_parameter("model_name", "../config/yolov8m.hef")
        self.declare_parameter("yolo_model", "../config/yolov8m.pt")  # For sim mode

        package_dir = Path(get_package_share_directory("hailo"))
        default_labels_path = (package_dir / "config" / "coco.txt").as_posix()
        self.declare_parameter("labels_path", default_labels_path)
        self.declare_parameter("score_threshold", 0.2)

        # For storing model input size
        self.model_h = None
        self.model_w = None

        # Equirectangular projection parameters
        self.h_fov_deg = self.declare_parameter("equirect_h_fov_deg", 180.0).value
        self.v_fov_deg = self.declare_parameter("equirect_v_fov_deg", 180.0).value

        self.sim_mode = self.get_parameter("sim").value

        self.model_path = (package_dir / "config" / self.get_parameter("model_name").value).as_posix()
        self.yolo_model_name = self.get_parameter("yolo_model").value
        self.labels_path = self.get_parameter("labels_path").value
        self.score_threshold = self.get_parameter("score_threshold").value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Set up publishers and subscribers
        self.detection_pub = self.create_publisher(Detection2DArray, "detections", 10)
        self.annotated_pub = self.create_publisher(CompressedImage, "annotated_image", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "detection_markers", 10)
        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10
        )

        # Initialize tracking and annotation
        self.box_annotator = sv.RoundBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.tracker = sv.ByteTrack()

        # Initialize ZMQ publisher for detection messages
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://*:5556")
        self.get_logger().info("ZMQ publisher bound to tcp://*:5556")

        # Initialize fisheye projector
        camera_params_path = os.path.join(os.path.dirname(__file__), "camera_params.yaml")
        fisheye_model = fisheye_utils.create_fisheye_model_from_params(camera_params_path, 1400, 1050)

        # Initialize model based on mode
        if self.sim_mode:
            self.get_logger().info("Running in simulation mode with YOLOv8")
            if not YOLO_AVAILABLE:
                self.get_logger().error("Ultralytics not available! Install with: pip install ultralytics")
                raise RuntimeError("YOLOv8 not available for simulation mode")

            # Initialize YOLOv8
            self.yolo_model = YOLO(self.yolo_model_name)

            # Get COCO class names from YOLOv8
            self.class_names = list(self.yolo_model.names.values())
            self.model_h, self.model_w = 640, 640  # Default YOLOv8 input size

        else:
            self.get_logger().info("Running in real mode with Hailo")
            if not HAILO_AVAILABLE:
                self.get_logger().error("Hailo utils not available!")
                raise RuntimeError("Hailo not available for real mode")

            # Initialize Hailo inference
            self.input_queue = queue.Queue()
            self.output_queue = queue.Queue()
            self.hailo_inference = HailoAsyncInference(
                hef_path=self.model_path,
                input_queue=self.input_queue,
                output_queue=self.output_queue,
            )
            self.model_h, self.model_w, _ = self.hailo_inference.get_input_shape()
            self.get_logger().info(f"Hailo model input shape: {self.model_w}x{self.model_h}")

            # Load class names from file
            with open(self.labels_path, "r", encoding="utf-8") as f:
                self.class_names = f.read().splitlines()

        # Initialize fisheye to equirectangular projector
        self.projector = fisheye_utils.FisheyeToEquirectangular(
            out_width=self.model_w,
            out_height=self.model_h,
            h_fov_deg=self.h_fov_deg,
            v_fov_deg=self.v_fov_deg,
            fisheye_model=fisheye_model,
        )

        if not self.sim_mode:
            # Start inference thread
            self.inference_thread = threading.Thread(target=self.hailo_inference.run)
            self.inference_thread.start()

    def image_callback(self, msg):
        # Convert ROS Image to CV2
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        video_h, video_w = frame.shape[:2]

        self.get_logger().info(f"Received /camera/image_raw/compressed image of size: {video_w}x{video_h}")

        # Rotate 180 degrees
        # ONLY UNCOMMENT IF YOUR CAMERA IS UPSIDE DOWN
        # frame = cv2.rotate(frame, cv2.ROTATE_180)

        start = time.time()
        equirect_frame = self.projector.project(frame)
        equirect_frame_h = equirect_frame.shape[0]
        equirect_frame_w = equirect_frame.shape[1]
        self.get_logger().info(f"Projection to equirect took: {time.time() - start:.3f} seconds")
        self.get_logger().info(f"Equirectangular image size: {equirect_frame_w}x{equirect_frame_h}")

        # If you want to skip fisheye to equirect projection for testing purposes,
        # equirect_frame = frame
        # equirect_frame_h = equirect_frame.shape[0]
        # equirect_frame_w = equirect_frame.shape[1]

        if self.sim_mode:
            # Run YOLOv8 inference
            results = self.yolo_model(equirect_frame, conf=self.score_threshold, verbose=False)

            # Convert YOLOv8 results to our detection format
            detections = self.extract_yolo_detections(results[0])
        else:
            # Preprocess frame
            preprocessed_frame = self.preprocess_frame(
                equirect_frame, self.model_h, self.model_w, equirect_frame_h, equirect_frame_w
            )

            # Run Hailo inference
            self.input_queue.put([preprocessed_frame])
            _, results = self.output_queue.get()

            if len(results) == 1:
                results = results[0]

            # Process Hailo detections
            detections = self.extract_detections(results, equirect_frame_h, equirect_frame_w, self.score_threshold)

        self.get_logger().info(f"Detections: {detections}")

        # Convert detections to ROS messages
        detection_msg, marker_array = self.detections_to_ros_messages(detections, msg.header)

        # Publish detections
        self.detection_pub.publish(detection_msg)
        self.marker_pub.publish(marker_array)

        # Publish ZMQ messages with bounding box positions and IDs
        self.publish_zmq_detections(detections, self.model_w, self.model_h)

        # Create and publish annotated image
        if detections["num_detections"]:
            annotated_frame = self.postprocess_detections(
                equirect_frame, detections, self.class_names, self.tracker, self.box_annotator, self.label_annotator
            )
            _, jpg_buffer = cv2.imencode(".jpg", annotated_frame)
            annotated_msg = CompressedImage()
            annotated_msg.format = "jpeg"
            annotated_msg.data = jpg_buffer.tobytes()
        else:
            _, jpg_buffer = cv2.imencode(".jpg", equirect_frame)
            annotated_msg = CompressedImage()
            annotated_msg.format = "jpeg"
            annotated_msg.data = jpg_buffer.tobytes()
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

    def preprocess_frame(self, frame: np.ndarray, model_h: int, model_w: int, video_h: int, video_w: int) -> np.ndarray:
        if model_h != video_h or model_w != video_w:
            frame = cv2.resize(frame, (model_w, model_h))
        return frame

    def detections_to_ros_messages(
        self, detections: Dict[str, np.ndarray], header
    ) -> tuple[Detection2DArray, MarkerArray]:
        """Convert detections dictionary to ROS Detection2DArray and MarkerArray messages."""
        # Create Detection2DArray message
        detection_msg = Detection2DArray()
        detection_msg.header = header

        # Create MarkerArray message
        marker_array = MarkerArray()

        # Convert detections to ROS messages
        for i in range(detections["num_detections"]):
            if str(detections["class_id"][i]) != "0":
                continue

            # Create Detection2D message
            det = Detection2D()
            det.bbox.center.position.x = float((detections["xyxy"][i][0] + detections["xyxy"][i][2]) / 2)
            det.bbox.center.position.y = float((detections["xyxy"][i][1] + detections["xyxy"][i][3]) / 2)
            det.bbox.size_x = float(detections["xyxy"][i][2] - detections["xyxy"][i][0])
            det.bbox.size_y = float(detections["xyxy"][i][3] - detections["xyxy"][i][1])

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(detections["class_id"][i])
            hyp.hypothesis.score = float(detections["confidence"][i])
            det.results.append(hyp)

            detection_msg.detections.append(det)

            # Create marker for bounding box
            marker = Marker()
            marker.header = header
            marker.ns = "detection_boxes"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01  # Line width
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Add points to form rectangle
            x1, y1 = float(detections["xyxy"][i][0]), float(detections["xyxy"][i][1])
            x2, y2 = float(detections["xyxy"][i][2]), float(detections["xyxy"][i][3])
            points = [(x1, y1, 0.0), (x2, y1, 0.0), (x2, y2, 0.0), (x1, y2, 0.0), (x1, y1, 0.0)]  # Close the rectangle
            for x, y, z in points:
                p = Point()
                p.x = x
                p.y = y
                p.z = z
                marker.points.append(p)

            marker_array.markers.append(marker)

        return detection_msg, marker_array

    def publish_zmq_detections(self, detections: Dict[str, np.ndarray], image_width: int, image_height: int):
        """Publish detection bounding boxes and IDs over ZMQ as JSON."""
        people_locations = []

        for i in range(detections["num_detections"]):
            # Only process person detections (class_id == 0 for COCO person class)
            if str(detections["class_id"][i]) != "0":
                continue

            # Calculate center position and bounding box size
            center_x = float((detections["xyxy"][i][0] + detections["xyxy"][i][2]) / 2)
            center_y = float((detections["xyxy"][i][1] + detections["xyxy"][i][3]) / 2)
            bbox_width = float(detections["xyxy"][i][2] - detections["xyxy"][i][0])
            bbox_height = float(detections["xyxy"][i][3] - detections["xyxy"][i][1])

            # Normalize coordinates and sizes to 0.0-1.0 range
            normalized_x = center_x / image_width
            normalized_y = center_y / image_height
            normalized_width = bbox_width / image_width
            normalized_height = bbox_height / image_height

            # Use tracker ID if available, otherwise use detection index
            tracker_id = i  # Default fallback
            # Note: If we had tracker IDs from the tracker, we'd use those instead

            # in degrees
            elevation, heading = fisheye_utils.equirectangular_pixel_to_elevation_heading(
                center_x, center_y, image_width, image_height, h_fov_deg=self.h_fov_deg, v_fov_deg=self.v_fov_deg
            )

            people_locations.append(
                {
                    "x": normalized_x,
                    "y": normalized_y,
                    "width": normalized_width,
                    "height": normalized_height,
                    "heading": heading,
                    "elevation": elevation,
                    "id": tracker_id,
                }
            )

        # Create the message in the expected format
        zmq_message = {"people": people_locations, "timestamp": time.time()}
        self.get_logger().info(f"Publishing ZMQ message: {zmq_message}")

        # Send JSON message over ZMQ
        json_str = json.dumps(zmq_message)
        self.zmq_socket.send_string(json_str)

    def extract_yolo_detections(self, result) -> Dict[str, np.ndarray]:
        """Extract detections from YOLOv8 results."""
        xyxy: List[np.ndarray] = []
        confidence: List[float] = []
        class_id: List[int] = []
        num_detections: int = 0

        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                cls = int(box.cls[0].cpu().numpy())

                xyxy.append([x1, y1, x2, y2])
                confidence.append(conf)
                class_id.append(cls)
                num_detections += 1

        return {
            "xyxy": np.array(xyxy) if xyxy else np.empty((0, 4)),
            "confidence": np.array(confidence) if confidence else np.empty(0),
            "class_id": np.array(class_id) if class_id else np.empty(0, dtype=int),
            "num_detections": num_detections,
        }

    def extract_detections(
        self, hailo_output: List[np.ndarray], h: int, w: int, threshold: float = 0.5
    ) -> Dict[str, np.ndarray]:
        xyxy: List[np.ndarray] = []
        confidence: List[float] = []
        class_id: List[int] = []
        num_detections: int = 0

        for i, detections in enumerate(hailo_output):
            if len(detections) == 0:
                continue
            for detection in detections:
                bbox, score = detection[:4], detection[4]

                if score < threshold:
                    continue

                bbox[0], bbox[1], bbox[2], bbox[3] = (
                    bbox[1] * w,
                    bbox[0] * h,
                    bbox[3] * w,
                    bbox[2] * h,
                )

                xyxy.append(bbox)
                confidence.append(score)
                class_id.append(i)
                num_detections += 1

        return {
            "xyxy": np.array(xyxy),
            "confidence": np.array(confidence),
            "class_id": np.array(class_id),
            "num_detections": num_detections,
        }

    def postprocess_detections(
        self,
        frame: np.ndarray,
        detections: Dict[str, np.ndarray],
        class_names: List[str],
        tracker: sv.ByteTrack,
        box_annotator: sv.RoundBoxAnnotator,
        label_annotator: sv.LabelAnnotator,
    ) -> np.ndarray:
        sv_detections = sv.Detections(
            xyxy=detections["xyxy"],
            confidence=detections["confidence"],
            class_id=detections["class_id"],
        )

        sv_detections = tracker.update_with_detections(sv_detections)

        labels: List[str] = [
            f"#{tracker_id} {class_names[class_id]}"
            for class_id, tracker_id in zip(sv_detections.class_id, sv_detections.tracker_id)
        ]

        annotated_frame = box_annotator.annotate(scene=frame.copy(), detections=sv_detections)
        annotated_labeled_frame = label_annotator.annotate(
            scene=annotated_frame, detections=sv_detections, labels=labels
        )
        return annotated_labeled_frame


def main(args=None):
    rclpy.init(args=args)
    node = HailoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
