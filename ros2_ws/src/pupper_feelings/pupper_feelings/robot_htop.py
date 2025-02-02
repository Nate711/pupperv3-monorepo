import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
import math


class JointStateEffortVisualizer(Node):
    def __init__(self):
        super().__init__("robot_htop")

        # Declare and get parameters
        # Efforts are displayed in this order on the console
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
        self.declare_parameter("refresh_rate", 20.0)  # Frequency in Hz

        self.joint_names = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        timer_frequency = self.get_parameter("refresh_rate").get_parameter_value().double_value

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # Timer to periodically call display_efforts
        self.timer_period = 1.0 / timer_frequency  # Convert frequency to period in seconds
        self.timer = self.create_timer(
            self.timer_period, self.display_efforts, callback_group=ReentrantCallbackGroup()
        )

        self.joint_efforts = {name: 0.0 for name in self.joint_names}
        self.effort_sumsq = {name: 0.0 for name in self.joint_names}
        self.num_records = {name: 0 for name in self.joint_names}

    def joint_states_callback(self, msg):
        assert len(msg.effort) == 12
        for name, effort in zip(msg.name, msg.effort):
            self.joint_efforts[name] = effort
            self.effort_sumsq[name] += effort**2
            self.num_records[name] += 1

    def display_efforts(self):
        max_effort = 2.0

        # Clear the console
        print("\033c", end="")

        # Write bars representing absolute value of effort
        for joint_name in self.joint_names:
            effort = self.joint_efforts.get(joint_name, 0.0)
            bar_length = int((effort / max_effort) * 50)  # Normalize and scale to 50 characters
            bar = "=" * bar_length if effort >= 0 else "-" * -bar_length
            mean_sumsq = self.effort_sumsq.get(joint_name, 0.0) / self.num_records.get(
                joint_name, 1
            )
            rms = math.sqrt(mean_sumsq)
            print(f"{joint_name:15}: [{bar:<50}] {effort:.2f} \t(RMS: {rms:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateEffortVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
