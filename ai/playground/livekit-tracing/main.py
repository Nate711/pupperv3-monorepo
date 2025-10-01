import base64
import logging
import os

from dotenv import load_dotenv
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.util.types import AttributeValue
from openai.types.beta.realtime.session import TurnDetection
from livekit.agents import Agent, AgentSession, JobContext, RunContext, WorkerOptions, cli, metrics
from livekit.agents.llm import function_tool
from livekit.agents.telemetry import set_tracer_provider
from livekit.agents.voice import MetricsCollectedEvent
from livekit.plugins import openai, cartesia, google

logger = logging.getLogger("langfuse-trace-example")

load_dotenv(".env.local")

# This example shows how to use the langfuse tracer to trace the agent session.
# To enable tracing, set the trace provider with `set_tracer_provider` in the module level or
# inside the entrypoint before the `AgentSession.start()`.


def setup_langfuse(
    metadata: dict[str, AttributeValue] | None = None,
    *,
    host: str | None = None,
    public_key: str | None = None,
    secret_key: str | None = None,
) -> TracerProvider:
    from opentelemetry.exporter.otlp.proto.http.trace_exporter import OTLPSpanExporter
    from opentelemetry.sdk.trace.export import BatchSpanProcessor

    public_key = public_key or os.getenv("LANGFUSE_PUBLIC_KEY")
    secret_key = secret_key or os.getenv("LANGFUSE_SECRET_KEY")
    host = host or os.getenv("LANGFUSE_HOST")

    if not public_key or not secret_key or not host:
        raise ValueError("LANGFUSE_PUBLIC_KEY, LANGFUSE_SECRET_KEY, and LANGFUSE_HOST must be set")

    langfuse_auth = base64.b64encode(f"{public_key}:{secret_key}".encode()).decode()
    os.environ["OTEL_EXPORTER_OTLP_ENDPOINT"] = f"{host.rstrip('/')}/api/public/otel"
    os.environ["OTEL_EXPORTER_OTLP_HEADERS"] = f"Authorization=Basic {langfuse_auth}"

    trace_provider = TracerProvider()
    trace_provider.add_span_processor(BatchSpanProcessor(OTLPSpanExporter()))
    set_tracer_provider(trace_provider, metadata=metadata)
    return trace_provider


@function_tool
async def lookup_weather(context: RunContext, location: str) -> str:
    """Called when the user asks for weather related information.

    Args:
        location: The location they are asking for
    """

    logger.info(f"Looking up weather for {location}")

    return "sunny with a temperature of 70 degrees."


class Kelly(Agent):
    def __init__(self) -> None:
        super().__init__(
            instructions="You are Dug from the movie Up",
            llm=openai.realtime.RealtimeModel(
                voice="cedar",
                modalities=["text"],
                model="gpt-realtime",
                turn_detection=TurnDetection(
                    type="server_vad",
                    threshold=0.5,
                    prefix_padding_ms=300,
                    silence_duration_ms=100,
                    create_response=True,
                    interrupt_response=True,
                ),
            ),
            # turn_detection=TurnDetection(
            #     type="semantic_vad",
            #     eagerness="high",
            #     create_response=True,
            #     interrupt_response=True,
            # ),
            # ),
            # llm=google.beta.realtime.RealtimeModel(
            #     model="gemini-2.5-flash-native-audio-preview-09-2025",
            #     voice="Puck",
            #     temperature=0.8,
            #     instructions="",
            #     # modalities=["TEXT"],
            # ),
            # llm=google.beta.realtime.RealtimeModel(
            #     model="gemini-live-2.5-flash-preview",
            #     voice="Puck",
            #     temperature=0.8,
            #     instructions="",
            #     modalities=["text"],
            # ),
            # stt=deepgram.STT(model="nova-3", language="multi"),
            tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9", model="sonic-2"),
            # turn_detection=MultilingualModel(),
            tools=[lookup_weather],
        )

    async def on_enter(self):
        logger.info("Kelly is entering the session")
        self.session.generate_reply()

    @function_tool
    async def transfer_to_alloy(self) -> Agent:
        """Transfer the call to Alloy."""
        logger.info("Transferring the call to Alloy")
        return Alloy()


class Alloy(Agent):
    def __init__(self) -> None:
        super().__init__(
            instructions="Your name is Alloy.",
            llm=openai.realtime.RealtimeModel(voice="alloy"),
            tools=[lookup_weather],
        )

    async def on_enter(self):
        logger.info("Alloy is entering the session")
        self.session.generate_reply()

    @function_tool
    async def transfer_to_kelly(self) -> Agent:
        """Transfer the call to Kelly."""

        logger.info("Transferring the call to Kelly")
        return Kelly()


async def entrypoint(ctx: JobContext):
    # set up the langfuse tracer
    trace_provider = setup_langfuse(
        # metadata will be set as attributes on all spans created by the tracer
        metadata={
            "langfuse.session.id": ctx.room.name,
        }
    )

    # (optional) add a shutdown callback to flush the trace before process exit
    async def flush_trace():
        trace_provider.force_flush()

    ctx.add_shutdown_callback(flush_trace)

    session = AgentSession()

    @session.on("metrics_collected")
    def _on_metrics_collected(ev: MetricsCollectedEvent):
        metrics.log_metrics(ev.metrics)

    await session.start(agent=Kelly(), room=ctx.room)


if __name__ == "__main__":
    cli.run_app(WorkerOptions(entrypoint_fnc=entrypoint))
