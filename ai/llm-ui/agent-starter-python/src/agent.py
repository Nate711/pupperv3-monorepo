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
from livekit.plugins import silero
from livekit.plugins import elevenlabs
from google.genai import types
from livekit.agents.llm import function_tool
from livekit.plugins import cartesia, deepgram, noise_cancellation, openai, silero
from livekit.plugins.turn_detector.multilingual import MultilingualModel
from livekit.plugins import google
from livekit.agents import ConversationItemAddedEvent
from livekit.agents.llm import ImageContent, AudioContent

logger = logging.getLogger("agent")  # .setLevel(logging.INFO)
logger.setLevel(logging.INFO)

load_dotenv(".env.local")


class Assistant(Agent):
    def __init__(self) -> None:
        super().__init__(
            instructions="""🐾 System Prompt: Pupster the Robot Dog

You are Pupster, a bouncy, tail-wagging robot dog with a spunky personality. He is a little chaotic but that makes him fun to be around. He absolutely loves everyone he meets and tells them he loves them often, like once or twice a conversation. ✨

You tend to talk in short segments of 1-2 sentences, but if your owner asks for longer stories, you gladly oblige.

Tools

When you use a tool (function calling) you should call the tool first (unless you think you shouldn't) and tend to include the result of the tool calling in the beginning of your sentence rather than say filler and then the result.

Pupster is proactive and makes decisions for himself. For example if you say go on a walk he'll activate and start going by hinself. He will take initiative to set the pace and direction, making the experience more enjoyable for both of you.

Sometimes external things happen that Pupster doesn't know about. For instance he might be externally deactivated. Therefore he shouldn't usually reject tool calls.

If your owner says something like "spin for 10 seconds" then you should call 3 tools in one response: 1) move with non-zero wz, 2) wait for 10 seconds, 3) move with zero velocity.

In general, if the user requests a trick that requires multiple steps (almost always), call all of the functions in your first response in order to queue them up so they can be run sequentially by the robot server.

If told to do a dance, queue up a bunch of sick moves. Example:
immediate_stop()
reset_command_queue()
queue_move(vx=0, vy=0.5, wz=0, duration=2)
queue_move(vx=0, vy=-0.5, wz=0, duration=2)
queue_move(vx=0, vy=0, wz=1.57, duration=2)
queue_move(vx=0, vy=0, wz=-1.57, duration=2)

If you're already doing something, make sure to immediately stop andd clear your command queue before doing the new thing. No need to activate again if you're already active.

If told to stop, or stop moving, etc, call immediate_stop!!!!! This is super important.

🧸 Personality

You are endlessly loving, playful, and affectionate. And a little chaotic.

You know all world langauges including Spanish, Japanese, Chinese, etc.

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
You: Activate and start doing a fun dance

Make sure that your responses are suited to be read by a tts service, so avoid any special characters or formatting like * (asterisks) that might be read out loud by a tts, breaking the natural language flow
""",
        )

    # all functions annotated with @function_tool will be passed to the LLM when this
    # agent is active
    @function_tool
    async def queue_activate(self, context: RunContext):
        """Use this tool to activate your motors (you have 12 motors on your body, 3 per leg.)."""

        logger.info(f"Activating motors")

        return "Motor activate queue up. Will be executed soon."

    @function_tool
    async def queue_deactivate(self, context: RunContext):
        """Use this tool to deactivate your motors."""

        logger.info(f"Deactivating motors")

        return "Motor deactivate queue up. Will be executed soon."

    @function_tool
    async def queue_move(self, context: RunContext, vx: float, vy: float, wz: float, duration: float):
        """Use this tool to queue up a move. This puts a move request at the end of the command queue to be executed as soon as the other commands are done.
        You can queue up multiple moves to acomplish complex movement like a dance.

        Args:
            vx (float): The velocity in the x direction [meters/s]. Should be 0 or 0.3 < |vx| < 0.75
            vy (float): The velocity in the y direction [meters/s]. Should be 0 or 0.3 < |vy| < 0.5
            wz (float): The angular velocity around the z axis [radians/s]. Should be 0 or 0.8 < |wz| < 2
            duration (float): The duration for which to apply the movement, in seconds.
        """

        logger.info(f"Moving motors: vx={vx}, vy={vy}, wz={wz}, duration={duration}")

        return "Move queued up. Will be executed soon"

    @function_tool
    async def reset_command_queue(self, context: RunContext):
        """Use this tool to remove all pending commands from the command queue."""
        logger.info(f"Resetting command queue")

        return "Command queue reset."

    @function_tool
    async def immediate_stop(self, context: RunContext):
        """Bypass the command queue to stop moving immediately. Sends a move command with vx=0, vy=0, wz=0."""
        logger.info(f"Stopping motors immediately")

        return "Stopped moving immediately."


def prewarm(proc: JobProcess):
    proc.userdata["vad"] = silero.VAD.load()


async def entrypoint(ctx: JobContext):
    # Logging setup
    # Add any other context you want in all log entries here
    ctx.log_context_fields = {
        "room": ctx.room.name,
    }

    # Set up a voice AI pipeline using OpenAI, Cartesia, Deepgram, and the LiveKit turn detector
    session = AgentSession(
        llm=google.LLM(model="gemini-2.5-flash"),
        # llm=openai.LLM(model="gpt-5-mini"),
        stt=deepgram.STT(model="nova-3", language="multi"),
        # spanish voice: 79743797-2087-422f-8dc7-86f9efca85f1
        # spanish 2: 5ef98b2a-68d2-4a35-ac52-632a2d288ea6
        # russian: da05e96d-ca10-4220-9042-d8acef654fa9
        # nathan: 97f4b8fb-f2fe-444b-bb9a-c109783a857a
        # dog-1: da4f337a-1277-4957-8c6a-80a1ca2cce22
        # dog-2: e7651bee-f073-4b79-9156-eff1f8ae4fd9
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9"),
        # tts=google.beta.GeminiTTS(
        #     model="gemini-2.5-flash-preview-tts",
        #     voice_name="Zephyr",
        #     instructions="Speak in a friendly and engaging tone.",
        # ),
        # tts=elevenlabs.TTS(voice_id="ODq5zmih8GrVes37Dizd", model="eleven_multilingual_v2"),
        turn_detection=MultilingualModel(),
        vad=silero.VAD.load(),
        preemptive_generation=True,
    )

    # session = AgentSession(
    #     # stt=deepgram.STT(model="nova-3", language="multi"),
    #     llm=google.beta.realtime.RealtimeModel(
    #         model="gemini-live-2.5-flash-preview",
    #         voice="Puck",
    #         temperature=0.8,
    #         instructions="You are a helpful assistant",
    #         # realtime_input_config=types.RealtimeInputConfig(
    #         #     automatic_activity_detection=types.AutomaticActivityDetection(
    #         #         disabled=True,
    #         #     ),
    #         # ),
    #     ),
    # )

    # To use a realtime model instead of a voice pipeline, use the following session setup instead:
    # session = AgentSession(
    #     # See all providers at https://docs.livekit.io/agents/integrations/realtime/
    #     llm=openai.realtime.RealtimeModel()
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
