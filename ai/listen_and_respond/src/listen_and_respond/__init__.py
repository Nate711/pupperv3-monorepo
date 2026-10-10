from listen_and_respond.stt import SpeechToText


def main() -> None:
    print("Loading models...", flush=True)
    stt = SpeechToText()
    print("Listening. Speak English or Chinese; Ctrl+C to stop.", flush=True)
    try:
        for u in stt.listen():
            clip = "  CLIPPED" if u.peak >= 0.99 else ""
            print(
                f"[{u.lang or '?'}] {u.text}   "
                f"({u.duration_s:.1f}s speech, {u.decode_s:.2f}s decode, {u.emotion}/{u.event}){clip}",
                flush=True,
            )
    except KeyboardInterrupt:
        print("Stopped.")
