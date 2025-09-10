import logging
from pathlib import Path

from livekit.agents import (
    Agent,
    RunContext,
    AgentSession,
)
from livekit.agents.llm import function_tool

import logging

from livekit.plugins import cartesia, openai, google, deepgram, silero
from livekit.agents import UserInputTranscribedEvent
from livekit.plugins.turn_detector.multilingual import MultilingualModel


logger = logging.getLogger("agent")


def load_system_prompt():
    """Load system prompt from file with robust error handling."""
    path = Path(__file__).parent / "system_prompt.md"

    try:
        with open(path, "r", encoding="utf-8") as f:
            prompt = f.read().strip()
            logger.info(f"Successfully loaded system prompt from {path}")
            return prompt
    except Exception as e:
        logger.error(f"Failed to load prompt from {path}: {e}")
        raise e


########### VOICE OPTIONS ###############
# Sacha baren cohen (unintentional)
# tts=cartesia.TTS(voice="66db04d7-bca8-43dc-bf55-d432e4469b07", model="sonic-2")
# Dug
# tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9", model="sonic-2"),
# Nathan
# tts=cartesia.TTS(voice="70e274d6-3e98-49bf-b482-f7374b045dc8", model="sonic-2"),
# Teresa
# tts=cartesia.TTS(voice="47836b34-00be-4ada-bec2-9b69c73304b5", model="sonic-2"),
# Spanish
# tts=cartesia.TTS(voice="79743797-422f-8dc7-86f9efca85f1", model="sonic-2"),
#########################################


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
        # best dog: e7651bee-f073-4b79-9156-eff1f8ae4fd9
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9"),
        # spanish
        # tts=cartesia.TTS(voice="79743797-2087-422f-8dc7-86f9efca85f1"),
        turn_detection=MultilingualModel(),
        vad=silero.VAD.load(),
        # preemptive_generation=True,
    )


def openairealtime_session():
    return AgentSession(
        llm=openai.realtime.RealtimeModel(modalities=["audio"], model="gpt-realtime"),
    )


def openairealtime_cartesia_session():
    return AgentSession(
        llm=openai.realtime.RealtimeModel(modalities=["text"], model="gpt-realtime"),
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
    elif agent_design == "openai-realtime":
        return openairealtime_session()
    else:
        logger.error(f"Unknown agent design {agent_design}")
        raise ValueError(f"Unknown agent design {agent_design}")


# TODO: Consider making the ros tool server a subclass of PupsterAgent so that I don't have to re-define functions!
# TODO: Figure out how to share docstrings across all implementations
class PupsterAgent(Agent):
    def __init__(self, tool_impl) -> None:
        system_prompt = load_system_prompt()
        super().__init__(instructions=system_prompt)

        self.tool_impl = tool_impl

    # Waiting on openai and livekit to support images for realtime models
    # Would work for cascade models
    # @function_tool
    # async def get_camera_image(self, context: RunContext):
    #     """Use this tool to take a picture with Pupster's camera and add it to the conversation."""
    #     return await self.tool_impl.get_camera_image(context)

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
        You can queue up multiple moves to accomplish complex movement like a dance.

        If the user specifies a certain angle (e.g. 180 degrees), you will need 1) come up with a reasonable duration (eg 3 seconds) and
        2) divide the target angle by the duration to get the angular velocity (wz = 60 degrees per second).

        Args:
            vx (float): The velocity in the x direction [meters per second]. Should be 0 or 0.4 < |vx| < 0.75. Positive values move forward, negative backward.
            vy (float): The velocity in the y direction [meters per second]. Should be 0 or 0.4 < |vy| < 0.5. Positive values move to the left, negative to the right.
            wz (float): The angular velocity around the z axis [degrees per second]. Should be 0 or 30 < |wz| < 120. Positive values turn left, negative turn right.
            duration (float): The duration for which to apply the movement, in seconds.

        Example:
            To spin 180 degreees to the right, you could can call: queue_move(vx=0.0, vy=0.0, wz=-90.0, duration=2.0)
            To turn 360 degrees to the left while moving forward, you could call: queue_move(vx=0.5, vy=0.0, wz=90.0, duration=4.0)

        Invalid commands:
            Small movements such as queue_move(vx=0.1, vy=0.0, wz=0.0, duration=2.0) are invalid and will be ignored because the real robot
            is not responsive to small velocities. Use 0 or a velocity above the threshold.
        """

        logger.info(f"Moving motors: vx={vx}, vy={vy}, wz={wz}, duration={duration}")

        return await self.tool_impl.queue_move_for_time(vx, vy, wz, duration)

    @function_tool
    async def queue_stop(self, context: RunContext):
        """Use this tool to queue a Stop command (vx=0, vy=0, wz=0) at the end of the command queue."""
        return await self.tool_impl.queue_stop()

    @function_tool
    async def queue_wait(self, context: RunContext, duration: float):
        """Use this tool to wait for a certain duration before executing the next command in the queue.

        Args:
            duration (float): The duration to wait, in seconds.
        """
        logger.info(f"Waiting for {duration} seconds")

        return await self.tool_impl.queue_wait(duration)

    @function_tool
    async def queue_animation(self, context: RunContext, animation_name: str):
        """Use this tool to queue an animation sequence. This switches to a pre-recorded animation controller.

        Available animations:
        - "twerk": Makes the robot twerk
        - "lie_sit_lie": Given the robot is lying down, makes the robot sit up and then lie back down
        - "stand_sit_shake_sit_stand": Given the robot is standing, makes the robot sit, shake, and then stand back up
        - "stand_sit_stand": Given the robot is standing, makes the robot sit and then stand back up

        Args:
            animation_name (str): The name of the animation to play (e.g., 'twerk', 'sit', 'lie_down', 'shake')

        Example:
            To make the robot twerk: queue_animation(animation_name="twerk")
            To make the robot sit: queue_animation(animation_name="sit")
        """
        logger.info(f"Queueing animation: {animation_name}")

        return await self.tool_impl.queue_animation(animation_name)

    @function_tool
    async def reset_command_queue(self, context: RunContext):
        """Use this tool to remove all pending commands from the command queue."""
        logger.info(f"Resetting command queue")

        return await self.tool_impl.clear_queue()

    @function_tool
    async def immediate_stop(self, context: RunContext):
        """Clears the command queue. Then interrupts and stops the executing command whatever it may be. Finanlly sends a Stop command (vx=0, vy=0, wz=0)."""
        logger.info(f"Immediate stop requested")

        return await self.tool_impl.emergency_stop()
