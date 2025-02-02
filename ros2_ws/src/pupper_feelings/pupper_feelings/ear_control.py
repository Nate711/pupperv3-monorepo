import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from gpiozero import Servo
from time import sleep


def lin_map(val, in_min, in_max, out_min, out_max):
    normalized = (val - in_min) / (in_max - in_min)
    return normalized * (out_max - out_min) + out_min


class DualShockServoController(Node):
    def __init__(self):
        super().__init__("dualshock_servo_controller")
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # GPIO pins for the servos controlling the ears
        self.servo_l_pin = 16
        self.servo_r_pin = 26

        # Initialize Servos with gpiozero
        self.servo_l = Servo(self.servo_l_pin, min_pulse_width=0.000500, max_pulse_width=0.002500)
        self.servo_r = Servo(self.servo_r_pin, min_pulse_width=0.000500, max_pulse_width=0.002500)

        self.get_logger().info(
            "If getting GPIO errors, install requirements with:\n"
            "pip uninstall rpi.gpio\n"
            "sudo apt install python3-rpi-lgpio\n"
            "sudo rm /usr/lib/python3.*/EXTERNALLY-MANAGED\n"
            "pip install gpiozero"
        )

    def joy_callback(self, msg):
        l2_value = msg.axes[2]  # L2 trigger value, from -1 to 1
        right_stick_x = msg.axes[3]  # Right stick X-axis
        right_stick_y = msg.axes[4]  # Right stick Y-axis

        # Check if L2 is pressed beyond 95% (converted to -0.95 since the range is [-1, 1])
        if True:  # l2_value < -0.95:
            # Calculate left and right ear positions based on right stick Y value +- X value
            l_position = lin_map(
                val=(right_stick_y - right_stick_x), in_min=-2, in_max=2, out_min=-1, out_max=1
            )
            r_position = lin_map(
                val=(-right_stick_y - right_stick_x), in_min=-2, in_max=2, out_min=-1, out_max=1
            )

            # Set servo positions using gpiozero
            self.servo_l.value = l_position
            self.servo_r.value = r_position

            self.get_logger().info(
                f"Ears activated. Set servos to L: {l_position}, R: {r_position}"
            )
        else:
            # If L2 is not pressed enough, reset servo positions to neutral (0 position)
            self.servo_l.value = 0
            self.servo_r.value = 0
            self.get_logger().info(f"Ears deactivated. Servos reset to neutral position.")


def main(args=None):
    rclpy.init(args=args)
    dualshock_servo_controller = DualShockServoController()
    rclpy.spin(dualshock_servo_controller)
    dualshock_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
