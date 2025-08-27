import logging

from dotenv import load_dotenv
from livekit.agents import (
    NOT_GIVEN,
    AgentFalseInterruptionEvent,
    AgentSession,
    JobContext,
    JobProcess,
    MetricsCollectedEvent,
    RoomInputOptions,
    WorkerOptions,
    cli,
    metrics,
)
from livekit.plugins import silero
from livekit.plugins import cartesia, noise_cancellation, openai, silero
from livekit.plugins import google
from livekit.agents import UserInputTranscribedEvent

from pupster import Assistant

load_dotenv(".env.local")

logger = logging.getLogger("agent")


def prewarm(proc: JobProcess):
    proc.userdata["vad"] = silero.VAD.load()


async def entrypoint(ctx: JobContext):
    # Logging setup
    # Add any other context you want in all log entries here
    ctx.log_context_fields = {
        "room": ctx.room.name,
    }

    # Set up a voice AI pipeline using OpenAI, Cartesia, Deepgram, and the LiveKit turn detector
    # session = AgentSession(
    #     # FASTEST: gemini-2.5-flash and gpt-4.1
    #     # llm=google.LLM(model="gemini-2.5-flash"),
    #     llm=openai.LLM(model="gpt-4.1"),
    #     # llm=openai.LLM(model="gpt-5-mini"),
    #     max_tool_steps=20,
    #     stt=deepgram.STT(model="nova-3", language="multi"),
    #     # only english model supports keyterm boosting. in tests, not necessary for pupster. pupper intepreted as pepper
    #     # stt=deepgram.STT(model="nova-3", language="en", keyterms=["pupster", "pupper"]),
    #     # spanish voice: 79743797-2087-422f-8dc7-86f9efca85f1
    #     # nathan: 70e274d6-3e98-49bf-b482-f7374b045dc8
    #     # tts=cartesia.TTS(voice="70e274d6-3e98-49bf-b482-f7374b045dc8"),
    #     # teresa
    #     # tts=cartesia.TTS(voice="47836b34-00be-4ada-bec2-9b69c73304b5"),
    #     # best dog: e7651bee-f073-4b79-9156-eff1f8ae4fd9
    #     tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9"),
    #     # spanish
    #     # tts=cartesia.TTS(voice="79743797-2087-422f-8dc7-86f9efca85f1"),
    #     turn_detection=MultilingualModel(),
    #     vad=silero.VAD.load(),
    #     # preemptive_generation=True,
    # )

    session = AgentSession(
        llm=openai.realtime.RealtimeModel(modalities=["text"]),
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9", model="sonic-2"),
    )

    # session = AgentSession(
    #     llm=google.beta.realtime.RealtimeModel(
    #         model="gemini-live-2.5-flash-preview",
    #         voice="Puck",
    #         temperature=0.8,
    #         instructions="You are a helpful assistant",
    #         modalities=["text"],
    #     ),
    #     tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9"),
    # )

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
        logger.info(f"Metrics collected: {ev.metrics}")
        usage_collector.collect(ev.metrics)

    async def log_usage():
        summary = usage_collector.get_summary()
        logger.info(f"Usage: {summary}")

    ctx.add_shutdown_callback(log_usage)

    # Start the session, which initializes the voice pipeline and warms up the models
    await session.start(
        agent=Assistant(),
        room=ctx.room,
        room_input_options=RoomInputOptions(),
    )

    # Join the room and connect to the user
    await ctx.connect()


if __name__ == "__main__":
    cli.run_app(WorkerOptions(entrypoint_fnc=entrypoint, prewarm_fnc=prewarm))
