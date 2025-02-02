import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory
import os
import time
from subprocess import check_output


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

# Named pipe path
PIPE_PATH = "/tmp/pupper_feelings_pipe"


def open_desktop_terminal():
    """
    Opens a new xterm on the Pi desktop (DISPLAY=:0) that will display
    whatever is written into PIPE_PATH.
    Returns a file object that you can write to for showing text in that xterm.
    """
    # Set up the environment so xterm runs on the Pi's main display.
    env = os.environ.copy()
    env["DISPLAY"] = ":0"

    # Make sure the pipe exists
    if not os.path.exists(PIPE_PATH):
        os.mkfifo(PIPE_PATH)

    # Spawn xterm which simply cats the named pipe
    subprocess.Popen(
        [
            "xterm",
            "-hold",
            "-title",
            "Pupper Feelings",
            # "-geometry",
            # "116x48",
            # "-fn",
            # "20x40",
            "-bg",
            "black",
            "-fa",
            "Monospace",
            "-fs",
            "9",
            "-maximized",
            "-e",
            "cat {}".format(PIPE_PATH),
        ],
        env=env,
    )

    # Small delay to ensure xterm has time to open and attach to the pipe
    time.sleep(1)

    # Open the pipe for writing
    # buffering=1 ensures line-buffered output so ANSI codes appear immediately
    pipe_fd = open(PIPE_PATH, "w", buffering=1)
    return pipe_fd


class JoyListener(Node):
    def __init__(self):
        super().__init__("joy_listener")

        # Open a new xterm and store a handle to the named pipe
        self.terminal = open_desktop_terminal()

        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )
        self.prev_buttons = None
        self.prev_axes = None

        # Just log that we've started
        self.get_logger().info("Initialized face control")

        # Show the default eyes and info
        self.regular_eyes_and_info()

    def run_on_button(self, new_buttons, button_index, fn):
        if self.prev_buttons and (
            self.prev_buttons[button_index] == 0
            and new_buttons[button_index] == 1
        ):
            fn()

    def run_on_axes_change(self, new_axes, axis_index, axis_value, fn):
        if self.prev_axes and (
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
            check_output(["python3", "/home/pi/pupperv3-monorepo/robot/utils/check_batt_voltage.py", "--percentage_only"])
            .decode("utf-8")
            .strip()
        )
        # Write info to our pipe (the new terminal)
        self.terminal.write(
            "\n"+
            color_code
            + "IP address: "
            + str(ip_addr)
            + "\nBattery percentage: "
            + str(battery_voltage) + "%"
            + Colors.END
            + "\n"
        )

    def display_txt(self, input_file: str, color_code: str):
        package_share_directory = get_package_share_directory(
            "pupper_feelings"
        )
        resources_path = os.path.join(package_share_directory, "resources")
        filepath = os.path.join(resources_path, input_file)

        with open(filepath, "r") as f:
            lines = f.readlines()

        # Clear screen in xterm, then print the ASCII art
        self.terminal.write(CLEAR_SCREEN)
        self.terminal.write(color_code)
        for line in lines:
            self.terminal.write(line)
        self.terminal.write(Colors.END)
        self.terminal.write(HIDE_CURSOR)

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
            self.prev_axes = msg.axes
            return

        # L1 toggles "regular eyes and info"
        self.run_on_button(msg.buttons, l1_index, self.regular_eyes_and_info)

        # R1 toggles "ask" eyes
        self.run_on_button(
            msg.buttons,
            r1_index,
            lambda: self.display_txt(
                "ask.txt", color_code=Colors.LIGHT_PURPLE
            ),
        )

        # D-pad left/right/up/down => fancy eyes
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

        # Update previous states
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
