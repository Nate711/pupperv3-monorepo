import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory
import os

from subprocess import check_output
import time


class Colors:
    """ANSI color codes"""

    BLACK = "\033[0;30m"
    RED = "\033[0;31m"
    GREEN = "\033[0;32m"
    BROWN = "\033[0;33m"
    BLUE = "\033[0;34m"
    PURPLE = "\033[0;35m"
    CYAN = "\033[0;36m"
    LIGHT_GRAY = "\033[0;37m"
    DARK_GRAY = "\033[1;30m"
    LIGHT_RED = "\033[1;31m"
    LIGHT_GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    LIGHT_BLUE = "\033[1;34m"
    LIGHT_PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    LIGHT_WHITE = "\033[1;37m"
    BOLD = "\033[1m"
    FAINT = "\033[2m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"
    BLINK = "\033[5m"
    NEGATIVE = "\033[7m"
    CROSSED = "\033[9m"
    END = "\033[0m"


CLEAR_SCREEN = "\033[H\033[J"
HIDE_CURSOR = "\033[?25l"

TTY = "/dev/tty1"


class JoyListener(Node):
    def __init__(self):
        super().__init__("joy_listener")
        self.subscription = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )
        self.prev_buttons = None  # Store the previous state of all buttons
        self.prev_axes = None
        while not os.path.exists(TTY):
            self.get_logger().info(f"Waiting for {TTY}")
            time.sleep(1)
        self.get_logger().info("Initialized face control")
        self.regular_eyes_and_info()

    def run_on_button(self, new_buttons, button_index, fn):
        if (
            self.prev_buttons[button_index] == 0
            and new_buttons[button_index] == 1
        ):
            fn()

    def run_on_axes_change(self, new_axes, axis_index, axis_value, fn):
        if (
            self.prev_axes[axis_index] != axis_value
            and new_axes[axis_index] == axis_value
        ):
            fn()

    def run_script(self, script_args):
        package_share_directory = get_package_share_directory(
            "pupper_feelings"
        )
        resources_path = os.path.join(package_share_directory, "resources")
        script_path = os.path.join(resources_path, "ascii.sh")
        script_command = f"{script_path} {resources_path}/{script_args}"
        self.get_logger().info(f"Running script with args: {script_args}")
        try:
            subprocess.run(script_command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to execute script: {e}")

    def display_ip_battery_voltage(self, color_code=Colors.BLUE):
        ip_addr = check_output(["hostname", "-I"]).decode("utf-8").strip()
        battery_voltage = (
            check_output(["python3", "/home/pi/utils/check_batt_voltage.py"])
            .decode("utf-8")
            .strip()
        )
        with open(TTY, "w") as tty:
            tty.write(
                color_code
                + "IP address: "
                + str(ip_addr)
                + "\t"
                + str(battery_voltage)
                + "\t press again to refresh"
                + Colors.END
            )

    def display_txt(self, input_file: str, color_code: str):
        # Read all lines from the input file
        package_share_directory = get_package_share_directory(
            "pupper_feelings"
        )
        resources_path = os.path.join(package_share_directory, "resources")
        filepath = os.path.join(resources_path, input_file)

        with open(filepath, "r") as f:
            lines = f.readlines()
        buffer = ""
        for line in lines:
            buffer += line  # Accumulate the lines in the buffer

        with open(TTY, "w") as tty:
            tty.write(CLEAR_SCREEN)
            tty.write(color_code)
            tty.write(buffer)
            tty.write(Colors.END)
            tty.write(HIDE_CURSOR)

    def regular_eyes_and_info(self):
        self.display_txt("regular_eyes.txt", color_code=Colors.BLUE)
        self.display_ip_battery_voltage(color_code=Colors.BLUE)

    def joy_callback(self, msg):
        d_pad_x_axis = 6
        d_pad_y_axis = 7
        tri_button_index = 2
        o_button_index = 1
        x_button_index = 0
        l1_index = 4
        r1_index = 5

        if self.prev_buttons is None:
            self.prev_buttons = msg.buttons
            return

        if self.prev_axes is None:
            self.prev_axes = msg.axes
            return

        self.run_on_button(
            msg.buttons,
            r1_index,
            lambda: self.display_txt(
                "ask.txt", color_code=Colors.LIGHT_PURPLE
            ),
        )

        self.run_on_button(msg.buttons, l1_index, self.regular_eyes_and_info)
        self.run_on_axes_change(
            msg.axes,
            d_pad_x_axis,
            -1.0,
            lambda: self.display_txt(
                "fancy_eyes_right.txt", color_code=Colors.LIGHT_PURPLE
            ),
        )
        self.run_on_axes_change(
            msg.axes,
            d_pad_x_axis,
            1.0,
            lambda: self.display_txt(
                "fancy_eyes_left.txt", color_code=Colors.LIGHT_PURPLE
            ),
        )
        self.run_on_axes_change(
            msg.axes,
            d_pad_y_axis,
            1.0,
            lambda: self.display_txt(
                "fancy_eyes_up.txt", color_code=Colors.LIGHT_PURPLE
            ),
        )
        self.run_on_axes_change(
            msg.axes,
            d_pad_y_axis,
            -1.0,
            lambda: self.display_txt(
                "fancy_eyes_down.txt", color_code=Colors.LIGHT_PURPLE
            ),
        )

        # Update the previous button states
        self.prev_buttons = msg.buttons
        self.prev_axes = msg.axes


def main(args=None):
    rclpy.init(args=args)
    joy_listener = JoyListener()
    rclpy.spin(joy_listener)
    joy_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
