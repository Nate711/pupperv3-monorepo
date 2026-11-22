"""Step 3: execution.

Turns Commands from understand.py into ROS 2 calls on the running robot stack
(robot/utils/robot.sh, i.e. `ros2 launch neural_controller launch.py`). This process is a
separate ROS 2 node, so ROS has to be sourced in the shell that starts it.

Implemented: stand_up, move, turn. Other actions are reported as not implemented.
"Turn, then walk" arrives as two commands in a row, so it needs nothing extra here.

- stand_up switches controller_manager to the 4-legged walking policy, like the joystick's X.
- move walks toward any heading without turning (0 = ahead, 90 = right, 180 = back, 270 = left).
  It is open loop: it sends a fixed velocity on /llm_cmd_vel for distance / speed seconds. The
  real distance depends on how well the policy tracks the speed and on the floor, so expect
  it to be off by 10-30%.
- turn is closed loop on the IMU: the gyro's yaw rate is integrated until the robot has
  turned the requested angle. Clockwise is -yaw (to the right), counterclockwise +yaw.
- move and turn need the walking policy to be on already ("stand up" first).
- Commands run one at a time on runner.py's worker thread while the voice loop keeps listening;
  should_stop() (a spoken stop) interrupts move and turn within one control step.
- /llm_cmd_vel is below the joystick in cmd_vel_mux: touching a stick overrides a voice move.

Failure modes (all printed, none retried):
- the robot stack is not running: controller_manager's services are missing;
- controller_manager refuses the switch (the launch terminal says why, e.g. hardware not active);
- controller_manager does not answer in time;
- move/turn asked while the walking policy is off;
- a distance or angle outside the limits below;
- a turn that goes the wrong way (IMU sign or policy problem), stalls past its deadline, or
  loses IMU data. The robot is stopped in each case.
"""

import enum
import math
import os
import threading
import time
from collections.abc import Callable

if "AMENT_PREFIX_PATH" not in os.environ:
    raise RuntimeError("ROS 2 is not sourced. Run `source /opt/ros/jazzy/setup.bash` first, or pass --no-robot.")

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu

from listen_and_respond.commands import Command


class Outcome(enum.Enum):
    DONE = "done"
    FAILED = "failed"
    STOPPED = "stopped"  # interrupted by a spoken stop

SWITCH_SERVICE = "/controller_manager/switch_controller"
LIST_SERVICE = "/controller_manager/list_controllers"
SWITCH_TIMEOUT_S = 5
STARTUP_DISCOVERY_S = 6
# cmd_vel_mux input for voice: below the joystick, above person following.
VOICE_CMD_VEL = "/llm_cmd_vel"
IMU_TOPIC = "/imu_sensor_broadcaster/imu"

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

# Top speeds per axis, inside the range the policy follows (commands.py docstring). A move uses
# the fastest velocity along its heading that keeps both axes within these.
MAX_VX_MPS = 0.5  # forward / backward
MAX_VY_MPS = 0.4  # sideways
TURN_SPEED_DPS = 60.0
TURN_SLOW_DPS = 30.0  # used for the last TURN_SLOW_ZONE_DEG, to limit overshoot
TURN_SLOW_ZONE_DEG = 20.0
DEFAULT_DISTANCE_M = 0.5
DEFAULT_HEADING_DEG = 0.0
DEFAULT_TURN_DEG = 90.0
DEFAULT_ROTATION = "clockwise"
MAX_DISTANCE_M = 3.0
MAX_TURN_DEG = 720.0
WRONG_WAY_ABORT_DEG = 20.0
PUBLISH_HZ = 20
STOP_S = 0.5  # zero velocity is sent this long after a move, then the turn result is read
IMU_MAX_GAP_S = 0.1


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("listen_and_respond")
        self.switch_client = self.node.create_client(SwitchController, SWITCH_SERVICE)
        self.list_client = self.node.create_client(ListControllers, LIST_SERVICE)
        self.cmd_vel = self.node.create_publisher(Twist, VOICE_CMD_VEL, 10)

        # Yaw from integrating the gyro (relative; only differences are used).
        self._imu_lock = threading.Lock()
        self._yaw_rad = 0.0
        self._imu_last_stamp: float | None = None
        self._imu_last_arrival = 0.0
        self._imu_gaps = 0
        self.node.create_subscription(Imu, IMU_TOPIC, self._on_imu, 50)

        # Spin in the background so IMU messages and service replies arrive while a move runs.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # DDS discovery of the stack's services can take several seconds while the Pi is busy
        # (camera, Hailo, 500 Hz control loop); 2 s gave a false "not found" in the live test.
        for client, name in ((self.switch_client, SWITCH_SERVICE), (self.list_client, LIST_SERVICE)):
            if not client.wait_for_service(timeout_sec=STARTUP_DISCOVERY_S):
                print(
                    f"WARNING: {name} not found. Start the robot stack (robot/utils/robot.sh); "
                    "commands will fail until it is up.",
                    flush=True,
                )
                break

    def execute(self, command: Command, should_stop: Callable[[], bool]) -> tuple[Outcome, str]:
        """Runs one command. should_stop() turning true interrupts move and turn within one
        control step (1 / PUBLISH_HZ). Returns the outcome and what happened."""
        if command.action == "stand_up":
            return self.switch_to(WALKING_CONTROLLER, OTHER_CONTROLLERS)
        if command.action == "move":
            return self.walk(command.heading_deg, command.distance_m, should_stop)
        if command.action == "turn":
            return self.turn(command.rotation, command.angle_deg, should_stop)
        return Outcome.FAILED, f"{command.action} is not implemented yet"

    # --- controller_manager ---

    def _wait(self, future, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None

    def switch_to(self, controller: str, others: list[str]) -> tuple[Outcome, str]:
        if not self.switch_client.wait_for_service(timeout_sec=1.0):
            return Outcome.FAILED, f"{SWITCH_SERVICE} not found; is the robot stack running?"
        request = SwitchController.Request()
        request.activate_controllers = [controller]
        request.deactivate_controllers = others
        # BEST_EFFORT, like the joystick: already-inactive controllers in the deactivate list
        # are fine, and asking for an already-active controller is a no-op.
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = Duration(sec=SWITCH_TIMEOUT_S)
        result = self._wait(self.switch_client.call_async(request), SWITCH_TIMEOUT_S + 1)
        if result is None:
            return Outcome.FAILED, f"controller_manager did not answer within {SWITCH_TIMEOUT_S + 1} s"
        if not result.ok:
            return Outcome.FAILED, f"controller_manager refused to activate {controller}; see the robot launch output"
        return Outcome.DONE, f"{controller} active"

    def walking_policy_state(self) -> str:
        """The walking controller's state ("active", "inactive", ...) or why it is unknown."""
        if not self.list_client.wait_for_service(timeout_sec=1.0):
            return f"unknown ({LIST_SERVICE} not found; is the robot stack running?)"
        result = self._wait(self.list_client.call_async(ListControllers.Request()), 3.0)
        if result is None:
            return "unknown (controller_manager did not answer within 3 s)"
        for controller in result.controller:
            if controller.name == WALKING_CONTROLLER:
                return controller.state
        return f"unknown ({WALKING_CONTROLLER} is not loaded)"

    def _require_walking(self) -> str | None:
        """None if the walking policy is on, otherwise why the move cannot run."""
        state = self.walking_policy_state()
        if state != "active":
            return f"walking is off ({WALKING_CONTROLLER} is {state}); say 'stand up' first"
        return None

    # --- velocity ---

    def _send(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
        self.cmd_vel.publish(twist)

    def stop_moving(self) -> None:
        # Explicit zeros: if publishing just stopped, cmd_vel_mux would keep the last
        # velocity for its 500 ms timeout.
        for _ in range(int(STOP_S * PUBLISH_HZ)):
            self._send()
            time.sleep(1 / PUBLISH_HZ)

    def walk(
        self, heading_deg: float | None, distance_m: float | None, should_stop: Callable[[], bool]
    ) -> tuple[Outcome, str]:
        """heading_deg: direction of travel, clockwise from straight ahead (90 = right, 270 = left)."""
        heading = (DEFAULT_HEADING_DEG if heading_deg is None else heading_deg) % 360
        distance = DEFAULT_DISTANCE_M if distance_m is None else distance_m
        if not 0 < distance <= MAX_DISTANCE_M:
            return Outcome.FAILED, f"{distance:g} m is outside the 0-{MAX_DISTANCE_M:g} m limit for one move"
        if (why := self._require_walking()) is not None:
            return Outcome.FAILED, why

        # Unit vector in the robot frame (x ahead, y left), then the largest speed along it
        # that keeps each axis within its limit.
        ux = math.cos(math.radians(heading))
        uy = -math.sin(math.radians(heading))
        speed = 1 / max(abs(ux) / MAX_VX_MPS, abs(uy) / MAX_VY_MPS)
        vx, vy = round(ux * speed, 3) + 0.0, round(uy * speed, 3) + 0.0  # no -0.0 in the printout
        duration = distance / speed
        start = time.monotonic()
        end = start + duration
        while time.monotonic() < end:
            if should_stop():
                walked = (time.monotonic() - start) * speed
                self.stop_moving()
                return Outcome.STOPPED, f"walked about {walked:.2f} of {distance:g} m"
            self._send(vx, vy)
            time.sleep(1 / PUBLISH_HZ)
        self.stop_moving()
        return Outcome.DONE, (
            f"walked about {distance:g} m at heading {heading:g} degrees "
            f"({duration:.1f} s at vx={vx:.2f}, vy={vy:.2f} m/s, open loop)"
        )

    # --- turning ---

    def _on_imu(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._imu_lock:
            if self._imu_last_stamp is not None:
                dt = stamp - self._imu_last_stamp
                if dt > IMU_MAX_GAP_S:
                    self._imu_gaps += 1
                elif dt > 0:
                    self._yaw_rad += msg.angular_velocity.z * dt
            self._imu_last_stamp = stamp
            self._imu_last_arrival = time.monotonic()

    def _yaw_deg(self) -> tuple[float, int, float]:
        """Integrated yaw in degrees, number of IMU gaps so far, seconds since the last IMU message."""
        with self._imu_lock:
            return math.degrees(self._yaw_rad), self._imu_gaps, time.monotonic() - self._imu_last_arrival

    def turn(
        self, rotation: str | None, angle_deg: float | None, should_stop: Callable[[], bool]
    ) -> tuple[Outcome, str]:
        rotation = rotation or DEFAULT_ROTATION
        sign = -1 if rotation == "clockwise" else +1  # +yaw is counterclockwise seen from above
        angle = DEFAULT_TURN_DEG if angle_deg is None else angle_deg
        if not 0 < angle <= MAX_TURN_DEG:
            return Outcome.FAILED, f"{angle:g} degrees is outside the 0-{MAX_TURN_DEG:g} limit for one turn"
        if (why := self._require_walking()) is not None:
            return Outcome.FAILED, why
        start_yaw, start_gaps, age = self._yaw_deg()
        if age > IMU_MAX_GAP_S * 5:
            return Outcome.FAILED, f"no IMU data on {IMU_TOPIC} for {age:.1f} s"

        timeout = angle / TURN_SPEED_DPS * 2 + 3
        deadline = time.monotonic() + timeout
        while True:
            yaw, gaps, age = self._yaw_deg()
            turned = (yaw - start_yaw) * sign
            remaining = angle - turned
            if remaining <= 0:
                break
            if should_stop():
                self.stop_moving()
                final = (self._yaw_deg()[0] - start_yaw) * sign
                return Outcome.STOPPED, f"turned {rotation} {final:.0f} of {angle:g} degrees"
            if gaps != start_gaps or age > IMU_MAX_GAP_S * 5:
                self.stop_moving()
                return Outcome.FAILED, f"IMU data stopped or skipped during the turn after {turned:.0f} degrees; stopped"
            if turned < -WRONG_WAY_ABORT_DEG:
                self.stop_moving()
                return Outcome.FAILED, f"the IMU says it turned {-turned:.0f} degrees the wrong way; stopped"
            if time.monotonic() > deadline:
                self.stop_moving()
                return Outcome.FAILED, f"only turned {turned:.0f} of {angle:g} degrees in {timeout:.0f} s; stopped"
            speed = TURN_SLOW_DPS if remaining < TURN_SLOW_ZONE_DEG else TURN_SPEED_DPS
            self._send(wz=sign * math.radians(speed))
            time.sleep(1 / PUBLISH_HZ)
        self.stop_moving()
        final = (self._yaw_deg()[0] - start_yaw) * sign
        return Outcome.DONE, f"turned {rotation} {final:.0f} degrees (asked {angle:g})"

    def close(self) -> None:
        self.executor.shutdown()
        self.spin_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()
