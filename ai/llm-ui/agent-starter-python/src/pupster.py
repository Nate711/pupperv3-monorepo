import logging
from pathlib import Path

from livekit.agents import (
    Agent,
    RunContext,
    AgentSession,
)
from livekit.agents.llm import function_tool

import logging

from livekit.plugins import cartesia, openai, google, deepgram
from livekit.agents import UserInputTranscribedEvent
from livekit.plugins.turn_detector.multilingual import MultilingualModel

from ros_tool_server import RosToolServer

logger = logging.getLogger("agent")


def load_system_prompt():
    """Load system prompt from file with robust error handling."""
    path = Path(__file__).parent / "system_prompt.txt"

    try:
        with open(path, "r", encoding="utf-8") as f:
            prompt = f.read().strip()
            logger.info(f"Successfully loaded system prompt from {path}")
            return prompt
    except Exception as e:
        logger.error(f"Failed to load prompt from {path}: {e}")
        raise e


def cascaded_session():
    # Set up a voice AI pipeline using OpenAI, Cartesia, Deepgram, and the LiveKit turn detector
    return AgentSession(
        # FASTEST: gemini-2.5-flash and gpt-4.1
        # llm=google.LLM(model="gemini-2.5-flash"),
        llm=openai.LLM(model="gpt-4.1"),
        # llm=openai.LLM(model="gpt-5-mini"),
        max_tool_steps=20,
        stt=deepgram.STT(model="nova-3", language="multi"),
        # only english model supports keyterm boosting. in tests, not necessary for pupster. pupper intepreted as pepper
        # stt=deepgram.STT(model="nova-3", language="en", keyterms=["pupster", "pupper"]),
        # spanish voice: 79743797-2087-422f-8dc7-86f9efca85f1
        # nathan: 70e274d6-3e98-49bf-b482-f7374b045dc8
        # tts=cartesia.TTS(voice="70e274d6-3e98-49bf-b482-f7374b045dc8"),
        # teresa
        # tts=cartesia.TTS(voice="47836b34-00be-4ada-bec2-9b69c73304b5"),
        # best dog: e7651bee-f073-4b79-9156-eff1f8ae4fd9
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9"),
        # spanish
        # tts=cartesia.TTS(voice="79743797-2087-422f-8dc7-86f9efca85f1"),
        turn_detection=MultilingualModel(),
        vad=silero.VAD.load(),
        # preemptive_generation=True,
    )


def openairealtime_cartesia_session():
    return AgentSession(
        llm=openai.realtime.RealtimeModel(modalities=["text"]),
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9", model="sonic-2"),
    )


def gemini_cartesia_session():
    return AgentSession(
        llm=google.beta.realtime.RealtimeModel(
            model="gemini-live-2.5-flash-preview",
            voice="Puck",
            temperature=0.8,
            instructions="",
            modalities=["text"],
        ),
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9"),
    )


def get_pupster_session(agent_design: str):
    if agent_design == "cascade":
        return cascaded_session()
    elif agent_design == "google-cartesia":
        return gemini_cartesia_session()
    elif agent_design == "openai-cartesia":
        return openairealtime_cartesia_session()
    else:
        logger.error(f"Unknown agent design {agent_design}")
        raise ValueError(f"Unknown agent design {agent_design}")


# TODO: Consider making the ros tool server a subclass of PupsterAgent so that I don't have to re-define functions!
class PupsterAgent(Agent):
    def __init__(self) -> None:
        system_prompt = load_system_prompt()
        super().__init__(instructions=system_prompt)

        self.tool_impl = RosToolServer()

    # all functions annotated with @function_tool will be passed to the LLM when this
    # agent is active
    @function_tool
    async def queue_activate(self, context: RunContext):
        """Use this tool to activate your motors (you have 12 motors on your body, 3 per leg.)."""
        return await self.tool_impl.queue_activate()

    @function_tool
    async def queue_deactivate(self, context: RunContext):
        """Use this tool to deactivate your motors."""
        return await self.tool_impl.queue_deactivate()

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
