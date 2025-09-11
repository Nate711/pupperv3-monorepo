import asyncio
import pytest
from unittest.mock import Mock, MagicMock, patch, AsyncMock, call
import sys
import time

# Mock ROS2 modules before importing our module
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["controller_manager_msgs"] = MagicMock()
sys.modules["controller_manager_msgs.srv"] = MagicMock()
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = MagicMock()

# Now we can import our module
from ros_tool_server import (
    RosToolServer,
    MoveCommand,
    StopCommand,
    WaitCommand,
    MoveForTimeCommand,
    ActivateWalkingCommand,
    DeactivateCommand,
    MoveCfg,
    DefaultCfg,
)


@pytest.fixture
def mock_ros_dependencies():
    """Fixture to set up ROS mocks"""
    with patch("ros_tool_server.rclpy") as mock_rclpy, patch("ros_tool_server.Node") as mock_node_class, patch(
        "ros_tool_server.SwitchController"
    ) as mock_switch_controller, patch("ros_tool_server.Twist") as mock_twist_class:

        # Setup mock node
        mock_node = MagicMock()
        mock_node_class.return_value = mock_node

        # Setup mock publisher
        mock_publisher = MagicMock()
        mock_node.create_publisher.return_value = mock_publisher

        # Setup mock service client
        mock_service_client = MagicMock()
        mock_node.create_client.return_value = mock_service_client

        # Setup mock future for service calls
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_result = MagicMock()
        mock_result.ok = True
        mock_future.result.return_value = mock_result
        mock_service_client.call_async.return_value = mock_future

        # Setup Twist mock
        mock_twist_class.return_value = MagicMock()

        yield {
            "rclpy": mock_rclpy,
            "node": mock_node,
            "node_class": mock_node_class,
            "publisher": mock_publisher,
            "service_client": mock_service_client,
            "switch_controller": mock_switch_controller,
            "twist_class": mock_twist_class,
            "future": mock_future,
        }


@pytest.fixture
async def server(mock_ros_dependencies):
    """Fixture to create a RosToolServer instance"""
    server = RosToolServer()
    await server.start_queue_processor()
    yield server
    await server.stop_queue_processor()


@pytest.mark.asyncio
async def test_basic_move_command_execution(server, mock_ros_dependencies):
    """Test basic move command execution"""
    # Queue a move command
    move_cmd = MoveCommand(0.5, 0.0, 0.0, server)
    await server.add_command(move_cmd)

    # Give the queue processor time to execute
    await asyncio.sleep(0.2)

    # Verify twist was published
    mock_ros_dependencies["publisher"].publish.assert_called()
    published_twist = mock_ros_dependencies["publisher"].publish.call_args[0][0]
    assert published_twist.linear.x == 0.5
    assert published_twist.linear.y == 0.0
    assert published_twist.angular.z == 0.0


@pytest.mark.asyncio
async def test_stop_command_execution(server, mock_ros_dependencies):
    """Test stop command execution"""
    stop_cmd = StopCommand()
    await server.add_command(stop_cmd)

    await asyncio.sleep(0.2)

    # Verify stop twist was published
    mock_ros_dependencies["publisher"].publish.assert_called()
    published_twist = mock_ros_dependencies["publisher"].publish.call_args[0][0]
    assert published_twist.linear.x == 0.0
    assert published_twist.linear.y == 0.0
    assert published_twist.angular.z == 0.0


@pytest.mark.asyncio
async def test_wait_command_execution(server):
    """Test wait command execution"""
    wait_duration = 0.1
    wait_cmd = WaitCommand(wait_duration)

    start_time = time.time()
    success, message = await wait_cmd.execute(server)
    elapsed = time.time() - start_time

    assert success
    assert f"Waited for {wait_duration}" in message
    assert elapsed >= wait_duration


@pytest.mark.asyncio
async def test_move_for_time_command(server, mock_ros_dependencies):
    """Test MoveForTimeCommand execution"""
    move_time_cmd = MoveForTimeCommand(0.5, 0.0, 0.0, 0.1, server)

    start_time = time.time()
    success, message = await move_time_cmd.execute(server)
    elapsed = time.time() - start_time

    assert success
    assert "Completed move_for_time" in message
    assert elapsed >= 0.1

    # Should have published twice: once for move, once for stop
    assert mock_ros_dependencies["publisher"].publish.call_count >= 2


@pytest.mark.asyncio
async def test_interrupt_and_stop_no_command_running(server, mock_ros_dependencies):
    """Test interrupt_and_stop when no command is running"""
    success, message = await server._interrupt_and_stop()

    assert success
    assert "stopped" in message.lower()

    # Should publish stop twist
    mock_ros_dependencies["publisher"].publish.assert_called()


@pytest.mark.asyncio
async def test_interrupt_and_stop_during_wait(server, mock_ros_dependencies):
    """Test interrupting a command during a wait"""
    # Queue a long wait command
    wait_cmd = WaitCommand(2.0)  # 2 second wait
    await server.add_command(wait_cmd)

    # Let it start executing
    await asyncio.sleep(0.1)

    # Interrupt it
    start_interrupt = time.time()
    success, message = await server._interrupt_and_stop()
    interrupt_time = time.time() - start_interrupt

    assert success
    assert "interrupted" in message.lower()
    # Should interrupt quickly, not wait the full 2 seconds
    assert interrupt_time < 1.0

    # Should publish stop twist after interruption
    mock_ros_dependencies["publisher"].publish.assert_called()


@pytest.mark.asyncio
async def test_interrupt_move_for_time_during_wait(server, mock_ros_dependencies):
    """Test interrupting MoveForTimeCommand during its wait phase"""
    # Queue a move_for_time with long duration
    move_time_cmd = MoveForTimeCommand(0.5, 0.0, 0.0, 3.0, server)  # 3 second duration
    await server.add_command(move_time_cmd)

    # Let it start and get into the wait phase
    await asyncio.sleep(0.2)

    # Interrupt it
    start_interrupt = time.time()
    success, message = await server._interrupt_and_stop()
    interrupt_time = time.time() - start_interrupt

    assert success
    # Should interrupt quickly, not wait the full 3 seconds
    assert interrupt_time < 1.0

    # Verify stop was called after interruption
    assert mock_ros_dependencies["publisher"].publish.called


@pytest.mark.asyncio
async def test_clear_queue(server):
    """Test clearing the command queue"""
    # Add multiple commands to queue
    await server.add_command(MoveCommand(0.5, 0.0, 0.0, server))
    await server.add_command(WaitCommand(1.0))
    await server.add_command(StopCommand())

    # Clear the queue
    success, message = await server.clear_queue()

    assert success
    assert "3" in message  # Should mention clearing 3 commands
    assert server.command_queue.empty()


@pytest.mark.asyncio
async def test_emergency_stop(server, mock_ros_dependencies):
    """Test emergency stop functionality"""
    # Queue several commands
    await server.add_command(MoveCommand(0.5, 0.0, 0.0, server))
    await server.add_command(WaitCommand(2.0))
    await server.add_command(MoveCommand(0.0, 0.5, 0.0, server))
    await server.add_command(StopCommand())

    # Let first command start
    await asyncio.sleep(0.1)

    # Execute emergency stop
    success, message = await server.emergency_stop()

    assert success
    assert "emergency" in message.lower()
    assert server.command_queue.empty()  # Queue should be cleared

    # Stop should have been published
    mock_ros_dependencies["publisher"].publish.assert_called()


@pytest.mark.asyncio
async def test_activate_walking_command(server, mock_ros_dependencies):
    """Test activate walking command execution"""
    activate_cmd = ActivateWalkingCommand()
    success, message = await activate_cmd.execute(server)

    assert success
    assert "activated successfully" in message.lower()

    # Verify service was called
    mock_ros_dependencies["service_client"].call_async.assert_called()


@pytest.mark.asyncio
async def test_deactivate_command(server, mock_ros_dependencies):
    """Test deactivate command execution"""
    deactivate_cmd = DeactivateCommand()
    success, message = await deactivate_cmd.execute(server)

    assert success
    assert "deactivated successfully" in message.lower()

    # Verify service was called
    mock_ros_dependencies["service_client"].call_async.assert_called()


@pytest.mark.asyncio
async def test_velocity_validation(server, mock_ros_dependencies):
    """Test that velocities are validated in constructor"""
    # Test exceeding max velocities raises error
    with pytest.raises(ValueError, match="vx.*exceeds max limit"):
        MoveCommand(2.0, 0.0, 0.0, server)

    with pytest.raises(ValueError, match="vy.*exceeds max limit"):
        MoveCommand(0.0, 2.0, 0.0, server)

    with pytest.raises(ValueError, match="wz.*exceeds max limit"):
        MoveCommand(0.0, 0.0, 5.0, server)

    # Test all velocities below threshold raises error
    with pytest.raises(ValueError, match="below their movement thresholds"):
        MoveCommand(0.1, 0.1, 0.1, server)

    # Test valid velocities work
    valid_cmd = MoveCommand(0.5, 0.4, 1.0, server)
    assert valid_cmd.vx == 0.5
    assert valid_cmd.vy == 0.4
    assert valid_cmd.wz == 1.0


@pytest.mark.asyncio
async def test_sequential_command_execution(server, mock_ros_dependencies):
    """Test that commands execute sequentially in order"""
    execution_order = []

    class TrackingMoveCommand(MoveCommand):
        def __init__(self, vx, vy, wz, tag, server):
            super().__init__(vx, vy, wz, server)
            self.tag = tag

        async def execute(self, server):
            execution_order.append(self.tag)
            return await super().execute(server)

    # Queue commands
    await server.add_command(TrackingMoveCommand(0.4, 0, 0, "first", server))
    await server.add_command(TrackingMoveCommand(0.5, 0, 0, "second", server))
    await server.add_command(TrackingMoveCommand(0.6, 0, 0, "third", server))

    # Wait for execution
    await asyncio.sleep(0.5)

    assert execution_order == ["first", "second", "third"]


@pytest.mark.asyncio
async def test_queue_methods(server):
    """Test the queue helper methods"""
    # Test queue_move_for_time with valid parameters
    success, message = await server.queue_move_for_time(0.5, 0.0, 0.0, 1.0)
    assert success
    # Immediately after adding, the command should be in the queue
    assert not server.command_queue.empty()

    # Let Move start executing
    await asyncio.sleep(0.1)

    # Assert the MoveForTimeCommand is out of the queue (it should be executing)
    assert server.command_queue.empty()

    # Test queue_move_for_time with invalid parameters (exceeds max)
    success, message = await server.queue_move_for_time(10.0, 0.0, 0.0, 1.0)
    assert not success
    assert "exceeds max limit" in message

    # Test queue_move_for_time with below threshold velocities
    success, message = await server.queue_move_for_time(0.1, 0.1, 0.1, 1.0)
    assert not success
    assert "below their movement thresholds" in message

    # Test queue_activate_walking
    success, message = await server.queue_activate_walking()
    assert success
    # Immediately after adding, the command should be in the queue
    assert not server.command_queue.empty()

    await server.clear_queue()

    # Test queue_deactivate
    success, message = await server.queue_deactivate()
    assert success
    assert not server.command_queue.empty()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
