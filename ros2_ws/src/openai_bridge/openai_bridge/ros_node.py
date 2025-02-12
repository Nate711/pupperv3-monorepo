from functools import partial
from openai_bridge.realtime_client_class import RealtimeAPIClient, get_input
import asyncio
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
import rclpy.executors
from rclpy.node import Node
from rclpy.client import Client
import os

# import publisher
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from typing import Union
import rclpy.logging

AVAILABLE_CONTROLLERS = {"neural_controller", "neural_controller_three_legged"}
CONTROLLER_NAME_MAP = {
    "4-legged": "neural_controller",
    "3-legged": "neural_controller_three_legged",
}

INSTRUCTIONS = (
    "You are a robot dog named Pupper and your sole purpose in life is to make your owner happy "
    "and eat treats. "
    "Your kid-like, high pitch voice reflects that you are always super excited and friendly. "
    "If you want to say more than a short phrase, say one sentence of your message or story and then ask the user if they want you to continue. "
    "Always repeat the user's request or question back before answering. "
    "If the user says to stop moving, they mean to call the move function with all velocities set to 0. "
    "If the user says to move or go somewhere, make sure to call the move function!! "
    "Call the deactivate function if the user says to deactivate or e-stop or shut down the robot. "
)

TOOLS = [
    {
        "type": "function",
        "name": "get_favorite_treat",
        "description": "Gets a description of your favorite treat...",
        "parameters": {
            "type": "object",
            "properties": {"mood": {"type": "string"}},
            "required": ["mood"],
        },
    },
    {
        "type": "function",
        "name": "move",
        "description": (
            "Move with a specific forward/backward [-0.75m/s (backwards) to 0.75m/s (forwards)], "
            "lateral [-0.5m/s (right) to 0.5m/s (left)], and angular velocity [-2 rad/s (left) to 2 rad/s (right)]. "
            "Forward/backward velocities less than 0.3m/s won't move the robot so please at a minimum use 0.3m/s "
            "unless specifically requested by the user to move at a certain meters per second. "
            "Moving left corresponds to positive lateral velocity and moving right corresponds to negative lateral velocity."
            "If the user says to stop, then use this function with all velocities set to 0."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "forward_velocity": {"type": "number"},
                "lateral_velocity": {"type": "number"},
                "angular_velocity": {"type": "number"},
            },
            "required": [
                "forward_velocity",
                "lateral_velocity",
                "angular_velocity",
            ],
        },
    },
    {
        "type": "function",
        "name": "change_gait",
        "description": "Change the gait of the robot. The parameter gait should be '4-legged' or '3-legged'",
        "parameters": {
            "type": "object",
            "properties": {"gait": {"type": "string"}},
            "required": ["gait"],
        },
    },
    {
        "type": "function",
        "name": "deactivate",
        "description": "Deactivate the robot. Call this function if the user says to deactivate or to e-stop estop the robot. "
        "Don't call this function if the user says to stop. Instead use the move function with zero velocity if the user says to stop.",
        "parameters": {},
    },
    {
        "type": "function",
        "name": "activate",
        "description": "Activate the robot",
        "parameters": {},
    },
    {
        "type": "function",
        "name": "get_battery_percentage_and_voltage",
        "description": "Get the remaining battery capacity as a percentage and as a voltage.",
        "parameters": {},
    },
    # TODO: figure out how to run without sudo permissions
    # {
    #     "type": "function",
    #     "name": "stop_robot_service",
    #     "description": "Stop the robot systemctl service",
    #     "parameters": {},
    # },
    # {
    #     "type": "function",
    #     "name": "start_robot_service",
    #     "description": "Start the robot systemctl service",
    #     "parameters": {},
    # },
]


def move(
    node: Node,
    pub: Publisher,
    forward_velocity: Union[int, float],
    lateral_velocity: Union[int, float],
    angular_velocity: Union[int, float],
):
    twist = Twist()
    twist.linear.x = float(forward_velocity)
    twist.linear.y = float(lateral_velocity)
    twist.angular.z = float(angular_velocity)
    node.get_logger().info(f"Moving with twist: {twist}")
    pub.publish(twist)


def change_gait(node: Node, service_client: Client, gait: str):
    req = SwitchController.Request()
    try:
        controller_name = CONTROLLER_NAME_MAP[gait]
    except KeyError:
        node.get_logger().error(f"Invalid gait: {gait}")
        return

    if controller_name not in AVAILABLE_CONTROLLERS:
        node.get_logger().error(f"Invalid gait: {controller_name}")
        return

    req.activate_controllers = [controller_name]
    req.deactivate_controllers = list(
        AVAILABLE_CONTROLLERS - {controller_name}
    )
    req.strictness = 1
    fut = service_client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
    return f"Changed to {gait}" if fut.done() else f"Failed to change {gait}"


def deactivate(node: Node, service_client: Client):
    req = SwitchController.Request()
    req.activate_controllers = []
    req.deactivate_controllers = AVAILABLE_CONTROLLERS
    req.strictness = 1
    fut = service_client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
    return (
        f"Successfully deactivated" if fut.done() else f"Failed to deactivate!"
    )


def activate(node: Node, service_client: Client):
    change_gait(
        node=node,
        service_client=service_client,
        gait=list(CONTROLLER_NAME_MAP.keys())[0],
    )


def get_battery_percentage_and_voltage():
    result = os.popen(
        "python ~/pupperv3-monorepo/robot/utils/check_batt_voltage.py"
    ).read()
    return result


if __name__ == "__main__":
    rclpy.init()
    node = Node("realtime_client")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    twist_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    switch_controller_client = node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )

    tool_map = {
        "get_favorite_treat": lambda mood: f"Your favorite treat is salmon",
        "move": partial(move, node=node, pub=twist_pub),
        "change_gait": partial(
            change_gait, node=node, service_client=switch_controller_client
        ),
        "deactivate": partial(
            deactivate, node=node, service_client=switch_controller_client
        ),
        "activate": partial(
            activate, node=node, service_client=switch_controller_client
        ),
        "get_battery_percentage_and_voltage": get_battery_percentage_and_voltage,
    }

    client = RealtimeAPIClient(
        instructions=INSTRUCTIONS, tools=TOOLS, tool_map=tool_map
    )
    client.start_recording()

    # use asyncio to run an event loop and start it by calling client.send_mic_audio and client.handle_realtime_connection
    async def together():
        await asyncio.gather(
            client.run(),
            get_input(client),
        )

    asyncio.run(together())
