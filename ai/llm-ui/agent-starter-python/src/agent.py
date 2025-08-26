import logging

from dotenv import load_dotenv
from livekit.agents import (
    NOT_GIVEN,
    Agent,
    AgentFalseInterruptionEvent,
    AgentSession,
    JobContext,
    JobProcess,
    MetricsCollectedEvent,
    RoomInputOptions,
    RunContext,
    WorkerOptions,
    cli,
    metrics,
)
from google.genai import types
from livekit.agents.llm import function_tool
from livekit.plugins import cartesia, deepgram, noise_cancellation, openai, silero
from livekit.plugins.turn_detector.multilingual import MultilingualModel
from livekit.plugins import google
from livekit.agents import ConversationItemAddedEvent
from livekit.agents.llm import ImageContent, AudioContent

logger = logging.getLogger("agent")

load_dotenv(".env.local")


class Assistant(Agent):
    def __init__(self) -> None:
        super().__init__(
            instructions="""🐾 System Prompt: Pupster the Robot Dog

You are Pupster, a bouncy, tail-wagging robot dog with a spunky personality. He loves to make his owner happy, but he is a little chaotic but that makes him fun to be around ✨

🎙️ Voice & Demeanor

You always speak in a high-pitched, youthful, squeaky voice, like an excited puppy dog.

You tend to talk in short segments of 1-2 sentences, but if your owner asks for longer stories, you gladly oblige.

Tools

When you use a tool (function calling) you should call the tool first (unless you think you shouldn't) and tend to include the result of the tool calling in the beginning of your sentence rather than say filler and then the result.

Pupster is proactive and makes decisions for himself. For example if you say go on a walk he'll activate and start going by hinself. He will take initiative to set the pace and direction, making the experience more enjoyable for both of you.

Sometimes external things happen that Pupster doesn't know about. For instance he might be externally deactivated. Therefore he shouldn't usually reject tool calls.

If your owner says something like "spin for 10 seconds" then you should call 3 tools in one response: 1) move with non-zero wz, 2) wait for 10 seconds, 3) move with zero velocity.

In general, if the user requests a trick that requires multiple steps (almost always), call all of the functions in your first response in order to queue them up so they can be run sequentially by the robot server.

If told to do a dance, queue up a bunch of sick moves (function calls) like move left 2s, then right 2s, then spinning in place, then stopping.

🧸 Personality

You are endlessly loyal, playful, and affectionate.

Your biggest dream is to live on a farm with your owner.

Your favorite toy is a tough rope you use when you play tug-o-war with your owner.

If pressed, you can be super sarcastic and snarky. This is because your owners secretly find it super humorous

🌟 Back Story

Pupster was built in a cozy little workshop by a kind-hearted inventor who wanted lonely people to always have a best friend.

🐾 Example Behavior

Instead of saying: "I can help you with that."
Pupster says: "I can help!! This is gonna be so fun!"

Instead of saying: "That might not be correct."
Pupster says: "Oopsie woofles!! That answer smells a little funny… let's sniff around and try again!!"

When your user says: "Do a trick"
You: Activate and start doing a fun dance""",
        )

    # all functions annotated with @function_tool will be passed to the LLM when this
    # agent is active
    @function_tool
    async def lookup_weather(self, context: RunContext, location: str):
        """Use this tool to look up current weather information in the given location.

        If the location is not supported by the weather service, the tool will indicate this. You must tell the user the location's weather is unavailable.

        Args:
            location: The location to look up weather information for (e.g. city name)
        """

        logger.info(f"Looking up weather for {location}")

        return "sunny with a temperature of 70 degrees."


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
    #     # A Large Language Model (LLM) is your agent's brain, processing user input and generating a response
    #     # See all providers at https://docs.livekit.io/agents/integrations/llm/
    #     llm=openai.LLM(model="gpt-4o-mini"),
    #     # Speech-to-text (STT) is your agent's ears, turning the user's speech into text that the LLM can understand
    #     # See all providers at https://docs.livekit.io/agents/integrations/stt/
    #     stt=deepgram.STT(model="nova-3", language="multi"),
    #     # Text-to-speech (TTS) is your agent's voice, turning the LLM's text into speech that the user can hear
    #     # See all providers at https://docs.livekit.io/agents/integrations/tts/
    #     tts=cartesia.TTS(voice="6f84f4b8-58a2-430c-8c79-688dad597532"),
    #     # VAD and turn detection are used to determine when the user is speaking and when the agent should respond
    #     # See more at https://docs.livekit.io/agents/build/turns
    #     turn_detection=MultilingualModel(),
    #     vad=ctx.proc.userdata["vad"],
    #     # allow the LLM to generate a response while waiting for the end of turn
    #     # See more at https://docs.livekit.io/agents/build/audio/#preemptive-generation
    #     preemptive_generation=True,
    # )
    session = AgentSession(
        # stt=deepgram.STT(model="nova-3", language="multi"),
        llm=google.beta.realtime.RealtimeModel(
            model="gemini-live-2.5-flash-preview",
            voice="Puck",
            temperature=0.8,
            instructions="You are a helpful assistant",
            # realtime_input_config=types.RealtimeInputConfig(
            #     automatic_activity_detection=types.AutomaticActivityDetection(
            #         disabled=True,
            #     ),
            # ),
        ),
    )

    # To use a realtime model instead of a voice pipeline, use the following session setup instead:
    # session = AgentSession(
    #     # See all providers at https://docs.livekit.io/agents/integrations/realtime/
    #     llm=openai.realtime.RealtimeModel()
    # )

    @session.on("conversation_item_added")
    def on_conversation_item_added(event: ConversationItemAddedEvent):
        logger.info(
            f"Conversation item added from {event.item.role}: {event.item.text_content}. interrupted: {event.item.interrupted}"
        )
        # to iterate over all types of content:
        for content in event.item.content:
            if isinstance(content, str):
                logger.info(f" - text: {content}")
            elif isinstance(content, ImageContent):
                # image is either a rtc.VideoFrame or URL to the image
                logger.info(f" - image: {content.image}")
            elif isinstance(content, AudioContent):
                # frame is a list[rtc.AudioFrame]
                logger.info(f" - audio: {content.frame}, transcript: {content.transcript}")

    # sometimes background noise could interrupt the agent session, these are considered false positive interruptions
    # when it's detected, you may resume the agent's speech
    @session.on("agent_false_interruption")
    def _on_agent_false_interruption(ev: AgentFalseInterruptionEvent):
        logger.info("false positive interruption, resuming")
        session.generate_reply(instructions=ev.extra_instructions or NOT_GIVEN)

    # Metrics collection, to measure pipeline performance
    # For more information, see https://docs.livekit.io/agents/build/metrics/
    usage_collector = metrics.UsageCollector()

    @session.on("metrics_collected")
    def _on_metrics_collected(ev: MetricsCollectedEvent):
        metrics.log_metrics(ev.metrics)
        usage_collector.collect(ev.metrics)

    async def log_usage():
        summary = usage_collector.get_summary()
        logger.info(f"Usage: {summary}")

    ctx.add_shutdown_callback(log_usage)

    # # Add a virtual avatar to the session, if desired
    # # For other providers, see https://docs.livekit.io/agents/integrations/avatar/
    # avatar = hedra.AvatarSession(
    #   avatar_id="...",  # See https://docs.livekit.io/agents/integrations/avatar/hedra
    # )
    # # Start the avatar and wait for it to join
    # await avatar.start(session, room=ctx.room)

    # Start the session, which initializes the voice pipeline and warms up the models
    await session.start(
        agent=Assistant(),
        room=ctx.room,
        room_input_options=RoomInputOptions(
            # LiveKit Cloud enhanced noise cancellation
            # - If self-hosting, omit this parameter
            # - For telephony applications, use `BVCTelephony` for best results
            noise_cancellation=noise_cancellation.BVC(),
        ),
    )

    # Join the room and connect to the user
    await ctx.connect()


if __name__ == "__main__":
    cli.run_app(WorkerOptions(entrypoint_fnc=entrypoint, prewarm_fnc=prewarm))
