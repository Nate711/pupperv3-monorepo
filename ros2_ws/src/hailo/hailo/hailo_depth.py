import queue
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

    def callback(self, completion_info, bindings_list):
        print(f"Completed inference with info: {completion_info}")
        breakpoint()
        output_buffers = [binding.output().get_buffer() for binding in bindings_list]

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

    image_path = (
        "/home/pi/pupperv3-monorepo/ros2_ws/src/hailo/config/camera_image_raw_compressed-1759274101-438045672.png"
    )
    frame = cv2.imread(image_path)
    if frame is None:
        raise FileNotFoundError(f"Could not load image at {image_path}")

    frame = cv2.resize(frame, (hailo_depth.model_w, hailo_depth.model_h))
    # breakpoint()
    hailo_depth.input_queue.put([frame])


if __name__ == "__main__":
    main()
