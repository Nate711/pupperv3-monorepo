import logging
from typing import Any, Tuple
from tool_server_abc import ToolServer

logger = logging.getLogger("NopToolServer")


class NopToolServer(ToolServer):
    """No-op tool server for development without ROS.

    Each method logs the requested action and returns a success tuple.
    """

    def __init__(self):
        logger.info("Initialized NopToolServer (no-op mode)")

    async def get_camera_image(self, context: Any) -> Tuple[bool, str]:
        logger.info("NOOP get_camera_image called")
        return True, "NOOP image captured"

    async def queue_move_for_time(self, vx: float, vy: float, wz: float, duration: float) -> Tuple[bool, str]:
        logger.info(f"NOOP queue_move_for_time called: vx={vx}, vy={vy}, wz={wz}, duration={duration}")
        return True, f"NOOP queued move_for_time vx={vx}, vy={vy}, wz={wz} for {duration}s"

    async def queue_activate(self) -> Tuple[bool, str]:
        logger.info("NOOP queue_activate called")
        return True, "NOOP activate queued"

    async def queue_deactivate(self) -> Tuple[bool, str]:
        logger.info("NOOP queue_deactivate called")
        return True, "NOOP deactivate queued"

    async def queue_stop(self) -> Tuple[bool, str]:
        logger.info("NOOP queue_stop called")
        return True, "NOOP stop queued"

    async def queue_wait(self, duration: float) -> Tuple[bool, str]:
        logger.info(f"NOOP queue_wait called for {duration} seconds")
        return True, f"NOOP wait queued for {duration} seconds"

    async def queue_animation(self, animation_name: str) -> Tuple[bool, str]:
        logger.info(f"NOOP queue_animation called: animation_name={animation_name}")
        return True, f"NOOP animation '{animation_name}' queued"

    async def clear_queue(self) -> Tuple[bool, str]:
        logger.info("NOOP clear_queue called")
        return True, "NOOP cleared command queue"

    async def emergency_stop(self) -> Tuple[bool, str]:
        logger.warning("NOOP emergency_stop called")
        return True, "NOOP emergency stop executed"
