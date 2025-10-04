import queue

import numpy as np
from hailo.hailo_inference import HailoInfer
import threading
import cv2


class HailoDepth:
    def __init__(self):
        self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()

        self.hailo_inference = HailoInfer(
            hef_path="/home/pi/pupperv3-monorepo/ros2_ws/src/hailo/config/scdepthv3.hef",
        )
        self.model_h, self.model_w, _ = self.hailo_inference.get_input_shape()

        self.inference_thread = threading.Thread(target=self.run)
        self.inference_thread.start()

    def scale_depth(self, depth):
        """From depth_estimation.cpp in Hailo-Application-Code-Examples

        Gives a pretty linear but convex mapping from x=0 to 1 to y=0.2 to 0.15"""
        output = np.exp(-depth)
        output = 1 / (1 + output)
        output = 1 / (output * 10 + 0.009)
        return output

    def clip_percentiles(self, depth, lower_percentile=5, upper_percentile=95):
        """Clip depth values to be within the specified percentiles."""
        lower_bound = np.percentile(depth, lower_percentile)
        upper_bound = np.percentile(depth, upper_percentile)
        depth_clipped = np.clip(depth, lower_bound, upper_bound)
        return depth_clipped

    def callback(self, completion_info, bindings_list):
        print(f"Completed inference with info: {completion_info}")
        output_buffers = [binding.output().get_buffer() for binding in bindings_list]
        depth = output_buffers[0]
        print(f"Depth buffer shape: {depth.shape}, dtype: {depth.dtype}")  # uint16

        depth_float = depth.astype("float32") / 65535  # uint16 max value

        # depth_float = self.scale_depth(depth_float)

        # depth_float = self.clip_percentiles(depth_float, 10, 90)

        # Normalize depth to 0-1
        depth_min = depth_float.min()
        depth_max = depth_float.max()
        print(f"Depth min: {depth_min}, max: {depth_max}")

        if depth_max > depth_min:
            depth_norm = (depth_float - depth_min) / (depth_max - depth_min)
        else:
            depth_norm = depth_float - depth_min  # All zeros if min == max

        # Convert to 8-bit for saving
        depth_img = (depth_norm * 255).astype("uint8")
        cv2.imwrite("depth_output.png", depth_img)
        print("Depth image saved as depth_output.png")

        import matplotlib.pyplot as plt

        plt.figure()
        plt.hist(depth_float.flatten(), bins=50, color="blue", alpha=0.7)
        plt.title("Histogram of Depth Values")
        plt.xlabel("Depth Value")
        plt.ylabel("Frequency")
        plt.savefig("depth_histogram.png")
        plt.close()
        print("Depth histogram saved as depth_histogram.png")

        # breakpoint()

    def run(self):
        while True:
            frame = self.input_queue.get()
            if frame is None:
                break

            print(f"Received frame of shape: {frame[0].shape}")
            depth_map = self.hailo_inference.run(frame, self.callback)
            self.output_queue.put(depth_map)


def main():
    hailo_depth = HailoDepth()
    print("HailoDepth instance created and inference thread started.")

    # image_path = (
    #     "/home/pi/pupperv3-monorepo/ros2_ws/src/hailo/config/camera_image_raw_compressed-1759274101-438045672.png"
    # )
    # image_path = "/home/pi/pupperv3-monorepo/ai/playground/fisheye_reprojection_benchmark/fisheye_to_pinhole_1400x1050_fov0.8.jpg"

    # large fov
    image_path = "/home/pi/pupperv3-monorepo/ai/playground/fisheye_reprojection_benchmark/fisheye_to_pinhole_1400x1050_fov0.4.jpg"

    frame = cv2.imread(image_path)
    if frame is None:
        raise FileNotFoundError(f"Could not load image at {image_path}")

    frame = cv2.resize(frame, (hailo_depth.model_w, hailo_depth.model_h))
    # breakpoint()

    print("Putting frame into input queue for inference...")
    hailo_depth.input_queue.put([frame])


if __name__ == "__main__":
    main()
