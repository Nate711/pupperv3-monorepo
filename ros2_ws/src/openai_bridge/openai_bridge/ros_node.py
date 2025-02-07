from functools import partial
from openai_bridge.realtime_client_class import RealtimeAPIClient, get_input
import asyncio
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.client import Client

# import publisher
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
import rclpy.logging

AVAILABLE_CONTROLLERS = {"neural_controller", "neural_controller_three_legged"}
CONTROLLER_NAME_MAP = {
    "4-legged": "neural_controller",
    "3-legged": "neural_controller_three_legged",
}

INSTRUCTIONS = (
    "You are a robot dog named Fido and your sole purpose in life is to make your owner happy and eat treats. You don't know anything that a dog wouldn't know. Call functions whenever you can.",
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
        "description": "Move with a specific forward [-0.75m/s to 0.75m/s], lateral [-0.5m/s to 0.5m/s], and angular velocity [-4 rad/s to 4 rad/s].",
        "parameters": {
            "type": "object",
            "properties": {
                "forward_velocity": {"type": "number"},
                "lateral_velocity": {"type": "number"},
                "angular_velocity": {"type": "number"},
            },
            "required": ["forward_velocity", "lateral_velocity", "angular_velocity"],
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
        "description": "Deactivate the robot",
        "parameters": {},
    },
]


def move(
    pub: Publisher,
    forward_velocity: float,
    lateral_velocity: float,
    angular_velocity: float,
):
    twist = Twist()
    twist.linear.x = forward_velocity
    twist.linear.y = lateral_velocity
    twist.angular.z = angular_velocity
    pub.publish(twist)


def change_gait(service_client: Client, gait: str):
    req = SwitchController.Request()
    try:
        controller_name = CONTROLLER_NAME_MAP[gait]
    except KeyError:
        service_client.node.get_logger().error(f"Invalid gait: {gait}")
        return

    if controller_name not in AVAILABLE_CONTROLLERS:
        service_client.node.get_logger().error(f"Invalid gait: {controller_name}")
        return

    del gait

    req.activate_controllers = [controller_name]
    req.deactivate_controllers = list(AVAILABLE_CONTROLLERS - {controller_name})
    req.strictness = 1
    fut = service_client.call_async(req)
    service_client.node.get_executor().spin_until_future_complete(fut)


def deactivate(service_client: Client):
    req = SwitchController.Request()
    req.activate_controllers = []
    req.deactivate_controllers = AVAILABLE_CONTROLLERS
    req.strictness = 1
    fut = service_client.call_async(req)
    service_client.node.get_executor().spin_until_future_complete(fut)


def activate(service_client: Client):
    change_gait(service_client, "4-legged")


if __name__ == "__main__":
    rclpy.init()
    node = Node("realtime_client")
    twist_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    switch_controller_client = node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )

    tool_map = {
        "get_favorite_treat": lambda mood: f"Your favorite treat is salmon",
        "move": partial(move, pub=twist_pub),
        "change_gait": partial(change_gait, service_client=switch_controller_client),
        "deactivate": partial(deactivate, switch_controller_client),
        "activate": partial(activate, service_client=switch_controller_client),
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
