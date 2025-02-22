#!/usr/bin/env uv run
####################################################################
# Sample TUI app with a push to talk interface to the Realtime API #
# If you have `uv` installed and the `OPENAI_API_KEY`              #
# environment variable set, you can run this example with just     #
#                                                                  #
# `./examples/realtime/push_to_talk_app.py`                        #
####################################################################
#
# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "textual",
#     "numpy",
#     "pyaudio",
#     "pydub",
#     "sounddevice",
#     "openai[realtime]",
# ]
#
# [tool.uv.sources]
# openai = { path = "../../", editable = true }
# ///
from __future__ import annotations

import base64
import asyncio
from typing import Any, Callable, Dict, List, cast

from openai_bridge.audio_util import CHANNELS, SAMPLE_RATE, AudioPlayerAsync

from openai import AsyncOpenAI
from openai.types.beta.realtime.session import Session
from openai.types.beta.realtime import (
    ConversationItem,
)
from openai.resources.beta.realtime.realtime import (
    AsyncRealtimeConnection,
    RealtimeClientEvent,
)
import json


class RealtimeAPIClient:
    client: AsyncOpenAI
    should_send_audio: asyncio.Event
    audio_player: AudioPlayerAsync
    last_audio_item_id: str | None
    connection: AsyncRealtimeConnection | None
    session: Session | None
    connected: asyncio.Event
    instructions: str
    tools: List[Dict]
    tool_map: Dict[str, Callable]

    def __init__(
        self,
        instructions: str,
        tools: List[Dict],
        tool_map: Dict[str, Callable],
    ) -> None:
        super().__init__()
        self.connection = None
        self.session = None
        self.client = AsyncOpenAI()
        self.audio_player = AudioPlayerAsync()
        self.last_audio_item_id = None
        self.should_send_audio = asyncio.Event()
        self.connected = asyncio.Event()

        self.instructions = instructions
        self.tools = tools
        self.tool_map = tool_map

    async def on_mount(self) -> None:
        # TODO replace run_worker
        self.run_worker(self.handle_realtime_connection())
        self.run_worker(self.send_mic_audio())

    async def handle_realtime_connection(self) -> None:
        async with self.client.beta.realtime.connect(
            model="gpt-4o-realtime-preview",
        ) as conn:
            self.connection = conn
            self.connected.set()

            # note: this is the default and can be omitted
            # if you want to manually handle VAD yourself, then set `'turn_detection': None`
            print(self.instructions)
            print(self.tools)
            await conn.session.update(
                session={
                    "turn_detection": {"type": "server_vad"},
                    "instructions": self.instructions,
                    "tools": self.tools,
                },
            )

            # await conn.session.update(session={"turn_detection": None})

            acc_items: dict[str, Any] = {}

            async for event in conn:
                if event.type == "session.created":
                    self.session = event.session
                    assert event.session.id is not None
                    print("Session created", event.session.id)
                    continue

                if event.type == "session.updated":
                    self.session = event.session
                    continue

                if event.type == "response.audio.delta":
                    if event.item_id != self.last_audio_item_id:
                        self.audio_player.reset_frame_count()
                        self.last_audio_item_id = event.item_id

                    bytes_data = base64.b64decode(
                        event.delta
                    )  # typically 1200 to 18,000
                    self.audio_player.add_data(
                        {"data": bytes_data, "item_id": event.item_id}
                    )
                    # print("AUDIO DELTA", event.item_id)
                    # Notes: This will be sent multiple times with the same item id!
                    continue

                if event.type == "response.audio_transcript.delta":
                    # print("AUDIO TRANSCRIPT DELTA", event.item_id)
                    try:
                        text = acc_items[event.item_id]
                    except KeyError:
                        acc_items[event.item_id] = event.delta
                    else:
                        acc_items[event.item_id] = text + event.delta
                    # print("Transcript: ", acc_items[event.item_id], end="")
                    print(event.delta, end="", flush=True)
                    continue

                if event.type == "response.done":
                    print("Response done")
                    for output in event.response.output:
                        if output.type == "function_call":
                            # convert output.arguments from json string to dict
                            func_arguments = json.loads(output.arguments)
                            print(
                                f"\n****Calling function: {output.name} *******\narguments: {func_arguments}\n"
                            )
                            tool_output = self.tool_map[output.name](
                                **func_arguments
                            )
                            if tool_output is not None:
                                await self.connection.conversation.item.create(
                                    item=ConversationItem(
                                        type="function_call_output",
                                        call_id=output.call_id,
                                        output=tool_output,
                                    )
                                )
                                await conn.response.create()

                if event.type == "input_audio_buffer.speech_started":
                    print("\n*****Detected user began speaking*****\n")
                    # TODO: fix bug in interruption code that would prevent openai from speaking back
                    # self.audio_player.truncate()
                    # if (
                    #     self.audio_player.latest_played_item_id is not None
                    #     and self.audio_player.is_playing()
                    # ):
                    #     print("*********Truncating*********")
                    #     audio_end_ms = int(
                    #         self.audio_player.get_frame_count() / SAMPLE_RATE * 1000
                    #     )
                    #     print(
                    #         f"{self.audio_player.latest_played_item_id=} {audio_end_ms=}"
                    #     )
                    #     await self.connection.conversation.item.truncate(
                    #         item_id=self.audio_player.latest_played_item_id,
                    #         content_index=0,
                    #         audio_end_ms=audio_end_ms,
                    #     )

                if event.type == "response.function_call_arguments.done":
                    print("Function call arguments done")
                    print(event.name, event.arguments)

                print("Event type: ", event.type)

                if event.type == "error":
                    print("ERROR:")
                    print(event)

    async def _get_connection(self) -> AsyncRealtimeConnection:
        await self.connected.wait()
        assert self.connection is not None
        return self.connection

    async def send_mic_audio(self) -> None:
        import sounddevice as sd  # type: ignore

        # Don't get a response from openai until we have sent audio
        sent_audio = False

        device_info = sd.query_devices()
        print("Device info:")
        print(device_info)

        read_size = int(SAMPLE_RATE * 0.02)

        stream = sd.InputStream(
            channels=CHANNELS,
            samplerate=SAMPLE_RATE,
            dtype="int16",
        )
        stream.start()

        try:
            while True:
                if stream.read_available < read_size:
                    await asyncio.sleep(0)
                    continue

                await self.should_send_audio.wait()

                data, _ = stream.read(read_size)
                # print("READ", len(data))

                connection = await self._get_connection()
                if not sent_audio:
                    asyncio.create_task(
                        connection.send({"type": "response.cancel"})
                    )
                    sent_audio = True

                await connection.input_audio_buffer.append(
                    audio=base64.b64encode(cast(Any, data)).decode("utf-8")
                )

                await asyncio.sleep(0)
        except KeyboardInterrupt:
            pass
        finally:
            stream.stop()
            stream.close()

    async def run(self) -> None:
        await asyncio.gather(
            self.send_mic_audio(),
            self.handle_realtime_connection(),
        )

    def start_recording(self):
        self.should_send_audio.set()

    def stop_recording(self):
        self.should_send_audio.clear()


async def async_input(prompt: str) -> str:
    return await asyncio.to_thread(input, f"{prompt} ")


async def get_input(client) -> None:
    while True:
        result = await async_input("Enter r to start recording. s to stop.")
        if result == "r":
            print("Beginning recording")
            client.start_recording()
        elif result == "s":
            print("Stopping recording")
            client.stop_recording()


if __name__ == "__main__":
    client = RealtimeAPIClient()
    client.start_recording()

    # use asyncio to run an event loop and start it by calling client.send_mic_audio and client.handle_realtime_connection
    async def together():
        await asyncio.gather(
            client.run(),
            get_input(client),
        )

    asyncio.run(together())
