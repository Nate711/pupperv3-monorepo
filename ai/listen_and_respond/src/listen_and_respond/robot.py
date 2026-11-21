"""Step 3: execution.

Turns Commands from understand.py into ROS 2 calls on the running robot stack
(robot/utils/robot.sh, i.e. `ros2 launch neural_controller launch.py`). This process is a
separate ROS 2 node, so ROS has to be sourced in the shell that starts it.

Implemented so far: start_walking. Other actions are reported as not implemented.

Failure modes (all printed, none retried):
- the robot stack is not running: controller_manager's switch service is missing;
- controller_manager refuses the switch (the launch terminal says why, e.g. hardware not active);
- controller_manager does not answer within SWITCH_TIMEOUT_S.
"""

import os

if "AMENT_PREFIX_PATH" not in os.environ:
    raise RuntimeError("ROS 2 is not sourced. Run `source /opt/ros/jazzy/setup.bash` first, or pass --no-robot.")

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController

from listen_and_respond.commands import Command

SWITCH_SERVICE = "/controller_manager/switch_controller"
SWITCH_TIMEOUT_S = 5

# The 4-legged walking policy. Activating it runs its stand-up sequence (init_duration then
# fade_in_duration in ros2_ws/src/neural_controller/launch/config.yaml) before it walks.
WALKING_CONTROLLER = "neural_controller"
# The other controllers that can own the motors (joy_util_node.controller_names in config.yaml).
# Only one may be active at a time, the same as the joystick's X/O buttons do.
OTHER_CONTROLLERS = [
    "neural_controller_three_legged",
    "forward_position_controller",
    "forward_kp_controller",
    "forward_kd_controller",
]


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("listen_and_respond")
        self.switch_client = self.node.create_client(SwitchController, SWITCH_SERVICE)
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            print(
                f"WARNING: {SWITCH_SERVICE} not found. Start the robot stack (robot/utils/robot.sh); "
                "commands will fail until it is up.",
                flush=True,
            )

    def execute(self, command: Command) -> tuple[bool, str]:
        """Runs one command. Returns (succeeded, what happened)."""
        if command.action == "start_walking":
            return self.switch_to(WALKING_CONTROLLER, OTHER_CONTROLLERS)
        return False, f"{command.action} is not implemented yet"

    def switch_to(self, controller: str, others: list[str]) -> tuple[bool, str]:
        if not self.switch_client.wait_for_service(timeout_sec=1.0):
            return False, f"{SWITCH_SERVICE} not found; is the robot stack running?"
        request = SwitchController.Request()
        request.activate_controllers = [controller]
        request.deactivate_controllers = others
        # BEST_EFFORT, like the joystick: already-inactive controllers in the deactivate list
        # are fine, and asking for an already-active controller is a no-op.
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = Duration(sec=SWITCH_TIMEOUT_S)
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SWITCH_TIMEOUT_S + 1)
        if not future.done():
            return False, f"controller_manager did not answer within {SWITCH_TIMEOUT_S + 1} s"
        if not future.result().ok:
            return False, f"controller_manager refused to activate {controller}; see the robot launch output"
        return True, f"{controller} active"

    def close(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()
