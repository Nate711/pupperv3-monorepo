from abc import ABC, abstractmethod
from typing import Any, Tuple


class ToolServer(ABC):
    """Abstract base class for robot tool servers.

    Implementations should provide queueing behavior for movement and control
    operations. All methods return a (success: bool, message: str) tuple.
    """

    @abstractmethod
    async def get_camera_image(self, context: Any) -> Tuple[bool, str]:
        """Capture an image from the robot's camera and add it to the conversation."""
        raise NotImplementedError

    @abstractmethod
    async def queue_move_for_time(self, vx: float, vy: float, wz: float, duration: float) -> Tuple[bool, str]:
        """Queue a move command that runs for a fixed duration."""
        raise NotImplementedError

    @abstractmethod
    async def queue_activate(self) -> Tuple[bool, str]:
        """Queue activation of motors/controllers."""
        raise NotImplementedError

    @abstractmethod
    async def queue_deactivate(self) -> Tuple[bool, str]:
        """Queue deactivation of motors/controllers."""
        raise NotImplementedError

    @abstractmethod
    async def queue_stop(self) -> Tuple[bool, str]:
        """Queue an immediate stop (zero velocities)."""
        raise NotImplementedError

    @abstractmethod
    async def queue_wait(self, duration: float) -> Tuple[bool, str]:
        """Queue a wait for the specified duration in seconds."""
        raise NotImplementedError

    @abstractmethod
    async def clear_queue(self) -> Tuple[bool, str]:
        """Clear any pending commands from the queue."""
        raise NotImplementedError

    @abstractmethod
    async def emergency_stop(self) -> Tuple[bool, str]:
        """Stop execution, clear the queue, and ensure robot is stopped."""
        raise NotImplementedError
