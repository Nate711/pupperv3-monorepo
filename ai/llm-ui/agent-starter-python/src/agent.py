import logging

logging.getLogger("livekit.agents").setLevel(logging.WARNING)
logging.getLogger("livekit").setLevel(logging.WARNING)

from dotenv import load_dotenv
from livekit.agents import (
    NOT_GIVEN,
    AgentFalseInterruptionEvent,
    JobContext,
    JobProcess,
    MetricsCollectedEvent,
    RoomInputOptions,
    WorkerOptions,
    cli,
    metrics,
)
from livekit.agents import UserInputTranscribedEvent

from pupster import PupsterAgent, get_pupster_session

load_dotenv(".env.local")

logger = logging.getLogger("agent")


# AGENT_DESIGN = "cascade"
AGENT_DESIGN = "openai-cartesia"
# AGENT_DESIGN = "google-cartesia"
# AGENT_DESIGN = "openai-realtime"


def prewarm(proc: JobProcess):
    if AGENT_DESIGN == "cascade":
        from livekit.plugins import silero

        proc.userdata["vad"] = silero.VAD.load()


async def entrypoint(ctx: JobContext):
    # Logging setup
    # Add any other context you want in all log entries here
    ctx.log_context_fields = {
        "room": ctx.room.name,
    }

    session = get_pupster_session(AGENT_DESIGN)

    # @session.on("conversation_item_added")
    # def on_conversation_item_added(event: ConversationItemAddedEvent):
    #     logger.info(
    #         f"Conversation item added from {event.item.role}: {event.item.text_content}. interrupted: {event.item.interrupted}"
    #     )
    #     # to iterate over all types of content:
    #     for content in event.item.content:
    #         if isinstance(content, str):
    #             logger.info(f" - text: {content}")
    #         elif isinstance(content, ImageContent):
    #             # image is either a rtc.VideoFrame or URL to the image
    #             logger.info(f" - image: {content.image}")
    #         elif isinstance(content, AudioContent):
    #             # frame is a list[rtc.AudioFrame]
    #             logger.info(f" - audio: {content.frame}, transcript: {content.transcript}")

    # sometimes background noise could interrupt the agent session, these are considered false positive interruptions
    # when it's detected, you may resume the agent's speech
    @session.on("agent_false_interruption")
    def _on_agent_false_interruption(ev: AgentFalseInterruptionEvent):
        logger.info("false positive interruption, resuming")
        session.generate_reply(instructions=ev.extra_instructions or NOT_GIVEN)

    # Metrics collection, to measure pipeline performance
    # For more information, see https://docs.livekit.io/agents/build/metrics/
    usage_collector = metrics.UsageCollector()

    @session.on("user_input_transcribed")
    def on_user_input_transcribed(event: UserInputTranscribedEvent):
        print(
            f"User input transcribed: {event.transcript}, "
            f"language: {event.language}, "
            f"final: {event.is_final}, "
            f"speaker id: {event.speaker_id}"
        )

    @session.on("metrics_collected")
    def _on_metrics_collected(ev: MetricsCollectedEvent):
        metrics.log_metrics(ev.metrics)
        usage_collector.collect(ev.metrics)

    async def log_usage():
        summary = usage_collector.get_summary()
        logger.info(f"Usage: {summary}")

    ctx.add_shutdown_callback(log_usage)

    # Start the session, which initializes the voice pipeline and warms up the models
    try:
        from ros_tool_server import RosToolServer

        logger.info("Using RosToolServer for robot control")
        tool_impl = RosToolServer()
    except ImportError:
        from nop_tool_server import NopToolServer

        logger.info("ros_tool_server import failed, using NopToolServer")

        tool_impl = NopToolServer()

    await session.start(
        agent=PupsterAgent(tool_impl=tool_impl),
        room=ctx.room,
        room_input_options=RoomInputOptions(),
    )

    # Join the room and connect to the user
    await ctx.connect()


if __name__ == "__main__":
    cli.run_app(WorkerOptions(entrypoint_fnc=entrypoint, prewarm_fnc=prewarm))
