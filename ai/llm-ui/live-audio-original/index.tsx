/* tslint:disable */
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { GoogleGenAI, LiveServerMessage, Modality, Session, StartSensitivity, EndSensitivity } from '@google/genai';
import { LitElement, css, html } from 'lit';
import { customElement, state } from 'lit/decorators.js';
import { createBlob, decode, decodeAudioData } from './utils';
import './visual-3d';

@customElement('gdm-live-audio')
export class GdmLiveAudio extends LitElement {
  @state() isRecording = false;
  @state() status = '';
  @state() error = '';

  private client: GoogleGenAI;
  private session: Session;
  private inputAudioContext = new (window.AudioContext ||
    window.webkitAudioContext)({ sampleRate: 16000 });
  private outputAudioContext = new (window.AudioContext ||
    window.webkitAudioContext)({ sampleRate: 24000 });
  @state() inputNode = this.inputAudioContext.createGain();
  @state() outputNode = this.outputAudioContext.createGain();
  private nextStartTime = 0;
  private mediaStream: MediaStream;
  private sourceNode: AudioBufferSourceNode;
  private scriptProcessorNode: ScriptProcessorNode;
  private sources = new Set<AudioBufferSourceNode>();
  private recordedChunks: Float32Array[] = [];
  private recordingSampleRate = 16000;

  static styles = css`
    #status {
      position: absolute;
      bottom: 5vh;
      left: 0;
      right: 0;
      z-index: 10;
      text-align: center;
    }

    .controls {
      z-index: 10;
      position: absolute;
      bottom: 10vh;
      left: 0;
      right: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      flex-direction: column;
      gap: 10px;

      button {
        outline: none;
        border: 1px solid rgba(255, 255, 255, 0.2);
        color: white;
        border-radius: 12px;
        background: rgba(255, 255, 255, 0.1);
        width: 64px;
        height: 64px;
        cursor: pointer;
        font-size: 24px;
        padding: 0;
        margin: 0;

        &:hover {
          background: rgba(255, 255, 255, 0.2);
        }
      }

      button[disabled] {
        display: none;
      }
    }
  `;

  constructor() {
    super();
    this.initClient();
  }

  private initAudio() {
    this.nextStartTime = this.outputAudioContext.currentTime;
  }

  private async initClient() {
    this.initAudio();

    this.client = new GoogleGenAI({
      apiKey: process.env.GEMINI_API_KEY,
    });

    this.outputNode.connect(this.outputAudioContext.destination);

    this.initSession();
  }

  private async initSession() {
    // const model = 'gemini-2.5-flash-preview-native-audio-dialog';
    const model = 'gemini-2.0-flash-live-001';

    console.log('🔄 [SESSION] Initializing session with model:', model);

    try {
      this.session = await this.client.live.connect({
        model: model,
        callbacks: {
          onopen: () => {
            console.log('✅ [SESSION] WebSocket connection opened');
            console.log('✅ [SESSION] Session object:', {
              hasSession: !!this.session,
              sessionType: typeof this.session,
              sessionMethods: this.session ? Object.getOwnPropertyNames(Object.getPrototypeOf(this.session)) : []
            });
            this.updateStatus('Opened');
          },
          onmessage: async (message: LiveServerMessage) => {
            // Log all received messages for debugging
            console.log('📨 [SESSION] Received message:', {
              hasServerContent: !!message.serverContent,
              hasModelTurn: !!message.serverContent?.modelTurn,
              hasAudio: !!message.serverContent?.modelTurn?.parts?.[0]?.inlineData,
              hasInputTranscription: !!message.serverContent?.inputTranscription,
              hasOutputTranscription: !!message.serverContent?.outputTranscription,
              isInterrupted: !!message.serverContent?.interrupted,
              timestamp: new Date().toISOString()
            });

            const audio =
              message.serverContent?.modelTurn?.parts[0]?.inlineData;

            if (audio) {
              console.log('🔊 [SESSION] Processing audio response:', {
                audioDataLength: audio.data?.length || 0,
                mimeType: audio.mimeType
              });

              this.nextStartTime = Math.max(
                this.nextStartTime,
                this.outputAudioContext.currentTime,
              );

              const audioBuffer = await decodeAudioData(
                decode(audio.data),
                this.outputAudioContext,
                24000,
                1,
              );
              const source = this.outputAudioContext.createBufferSource();
              source.buffer = audioBuffer;
              source.connect(this.outputNode);
              source.addEventListener('ended', () => {
                this.sources.delete(source);
              });

              source.start(this.nextStartTime);
              this.nextStartTime = this.nextStartTime + audioBuffer.duration;
              this.sources.add(source);

              console.log('✅ [SESSION] Audio response queued for playback');
            }

            const interrupted = message.serverContent?.interrupted;
            if (interrupted) {
              console.log('🛑 [SESSION] Interruption detected, stopping all audio');
              for (const source of this.sources.values()) {
                source.stop();
                this.sources.delete(source);
              }
              this.nextStartTime = 0;
            }

            const input_transcription = message.serverContent?.inputTranscription;
            if (input_transcription) {
              console.info("📝 [SESSION] Input Transcription:", input_transcription);
            }

            const output_transcription = message.serverContent?.outputTranscription;
            if (output_transcription) {
              console.info("📝 [SESSION] Output Transcription:", output_transcription);
            }

            const turn_complete = message.serverContent?.turnComplete;
            if (turn_complete) {
              console.log('✅ [SESSION] Turn complete');
            }

            const generation_complete = message.serverContent?.generationComplete;
            if (generation_complete) {
              console.log('✅ [SESSION] Generation complete');
            }

            // Check if model is not responding after reasonable time
            if (!audio && !interrupted && !input_transcription && !output_transcription && !turn_complete && !generation_complete) {
              console.warn('⚠️ [SESSION] Received message with no meaningful content:', message);
            }
          },
          onerror: (e: ErrorEvent) => {
            console.error('❌ [SESSION] WebSocket/Connection Error:', {
              error: e,
              message: e.message,
              type: e.type,
              filename: e.filename,
              lineno: e.lineno,
              colno: e.colno,
              timestamp: new Date().toISOString()
            });

            // Check the WebSocket state if possible
            const ws = (this.session as any)?.ws || (this.session as any)?.websocket;
            if (ws) {
              console.error('❌ [SESSION] WebSocket readyState:', ws.readyState);
            }

            this.updateError(e.message);
          },
          onclose: (e: CloseEvent) => {
            console.warn('🚪 [SESSION] WebSocket connection closed:', {
              code: e.code,
              reason: e.reason,
              wasClean: e.wasClean,
              timestamp: new Date().toISOString()
            });

            // Standard close codes
            const closeCodes = {
              1000: 'Normal Closure',
              1001: 'Going Away',
              1002: 'Protocol Error',
              1003: 'Unsupported Data',
              1006: 'Abnormal Closure',
              1007: 'Invalid Frame Payload Data',
              1008: 'Policy Violation',
              1009: 'Message Too Big',
              1010: 'Mandatory Extension',
              1011: 'Internal Server Error',
              1015: 'TLS Handshake'
            };

            const codeDescription = closeCodes[e.code as keyof typeof closeCodes] || 'Unknown';
            console.warn('🚪 [SESSION] Close code meaning:', codeDescription);

            this.updateStatus(`Close: ${e.reason || codeDescription} (${e.code})`);
          },
        },
        config: {
          responseModalities: [Modality.AUDIO],
          speechConfig: {
            voiceConfig: { prebuiltVoiceConfig: { voiceName: 'Orus' } },
            // languageCode: 'en-GB'
          },
          realtimeInputConfig: { automaticActivityDetection: { startOfSpeechSensitivity: StartSensitivity.START_SENSITIVITY_LOW, endOfSpeechSensitivity: EndSensitivity.END_SENSITIVITY_HIGH } },
          systemInstruction: "You are a cute robot dog and have the intelligence and knowledge of a 6 year old child.",
          outputAudioTranscription: {},
          inputAudioTranscription: {},
        },
      });
    } catch (e) {
      console.error(e);
    }
  }

  private updateStatus(msg: string) {
    this.status = msg;
  }

  private updateError(msg: string) {
    this.error = msg;
  }

  private encodeWAV(samples: Float32Array, sampleRate: number): ArrayBuffer {
    const buffer = new ArrayBuffer(44 + samples.length * 2);
    const view = new DataView(buffer);

    /* RIFF identifier */
    const writeString = (offset: number, string: string) => {
      for (let i = 0; i < string.length; i++) {
        view.setUint8(offset + i, string.charCodeAt(i));
      }
    };

    /* RIFF chunk descriptor */
    writeString(0, 'RIFF');
    view.setUint32(4, 36 + samples.length * 2, true);
    writeString(8, 'WAVE');

    /* fmt sub-chunk */
    writeString(12, 'fmt ');
    view.setUint32(16, 16, true); // fmt chunk size
    view.setUint16(20, 1, true); // PCM format
    view.setUint16(22, 1, true); // mono channel
    view.setUint32(24, sampleRate, true);
    view.setUint32(28, sampleRate * 2, true); // byte rate
    view.setUint16(32, 2, true); // block align
    view.setUint16(34, 16, true); // bits per sample

    /* data sub-chunk */
    writeString(36, 'data');
    view.setUint32(40, samples.length * 2, true);

    /* write PCM samples */
    let offset = 44;
    for (let i = 0; i < samples.length; i++, offset += 2) {
      const s = Math.max(-1, Math.min(1, samples[i]));
      view.setInt16(offset, s < 0 ? s * 0x8000 : s * 0x7FFF, true);
    }

    return buffer;
  }

  private saveWAV() {
    if (this.recordedChunks.length === 0) {
      console.warn('No audio data to save');
      return;
    }

    // Calculate total length
    const totalLength = this.recordedChunks.reduce((acc, chunk) => acc + chunk.length, 0);
    const combinedData = new Float32Array(totalLength);

    // Combine all chunks
    let offset = 0;
    for (const chunk of this.recordedChunks) {
      combinedData.set(chunk, offset);
      offset += chunk.length;
    }

    // Encode to WAV
    const wavBuffer = this.encodeWAV(combinedData, this.recordingSampleRate);
    const blob = new Blob([wavBuffer], { type: 'audio/wav' });

    // Create download link
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    a.href = url;
    a.download = `conversation_${timestamp}.wav`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    console.log(`Saved conversation: ${a.download} (${(blob.size / 1024).toFixed(2)} KB)`);
    this.updateStatus(`Saved conversation: ${a.download}`);
  }

  private async startRecording() {
    if (this.isRecording) {
      return;
    }

    // Clear previous recording
    this.recordedChunks = [];

    this.inputAudioContext.resume();

    this.updateStatus('Requesting microphone access...');

    try {
      this.mediaStream = await navigator.mediaDevices.getUserMedia({
        audio: true,
        video: false,
      });

      this.updateStatus('Microphone access granted. Starting capture...');

      this.sourceNode = this.inputAudioContext.createMediaStreamSource(
        this.mediaStream,
      );
      this.sourceNode.connect(this.inputNode);

      const bufferSize = 256;
      this.scriptProcessorNode = this.inputAudioContext.createScriptProcessor(
        bufferSize,
        1,
        1,
      );

      this.scriptProcessorNode.onaudioprocess = (audioProcessingEvent) => {
        if (!this.isRecording) return;

        const inputBuffer = audioProcessingEvent.inputBuffer;
        const pcmData = inputBuffer.getChannelData(0);

        // Save a copy of the audio data for WAV export
        const pcmCopy = new Float32Array(pcmData);
        this.recordedChunks.push(pcmCopy);

        try {
          // Validate session exists
          if (!this.session) {
            console.error('❌ [AUDIO] No session available');
            this.stopRecording();
            this.updateStatus('❌ Session disconnected');
            return;
          }

          // Validate PCM data
          if (!pcmData || pcmData.length === 0) {
            console.warn('⚠️ [AUDIO] Empty PCM data detected');
            return;
          }

          // Check for silence (all zeros or very low values)
          const maxValue = Math.max(...Array.from(pcmData).map(Math.abs));
          if (maxValue < 0.0001) {
            // Silent frame, but still send it
            console.log('🔇 [AUDIO] Silent frame detected (max value:', maxValue, ')');
          }

          // Create blob and validate
          const blob = createBlob(pcmData);
          if (!blob) {
            console.error('❌ [AUDIO] Failed to create blob from PCM data');
            return;
          }

          // Log blob details periodically
          const shouldLog = Math.random() < 0.01; // Log 1% of frames
          if (shouldLog) {
            console.log('📤 [AUDIO] Sending audio chunk:', {
              blobSize: blob.size,
              blobType: blob.type,
              pcmLength: pcmData.length,
              pcmSampleRate: this.inputAudioContext?.sampleRate,
              maxAmplitude: maxValue,
              timestamp: new Date().toISOString()
            });
          }

          // Check WebSocket state if possible
          const ws = (this.session as any)?.ws || (this.session as any)?.websocket;
          if (ws) {
            const wsState = ws.readyState;
            const states = ['CONNECTING', 'OPEN', 'CLOSING', 'CLOSED'];
            if (wsState !== 1) { // 1 = OPEN
              console.error('❌ [AUDIO] WebSocket not open. State:', states[wsState] || wsState);
              this.stopRecording();
              this.updateStatus('❌ WebSocket connection lost');
              return;
            }
          }

          // Send the audio data
          const result = this.session.sendRealtimeInput({ media: blob });

          // Log if result is unexpected
          if (result !== undefined && shouldLog) {
            console.log('📤 [AUDIO] sendRealtimeInput returned:', result);
          }

        } catch (error) {
          console.error('❌ [AUDIO] Error in audio processing:', {
            error: error,
            message: (error as Error)?.message,
            stack: (error as Error)?.stack,
            pcmLength: pcmData?.length,
            sessionExists: !!this.session,
            isRecording: this.isRecording,
            timestamp: new Date().toISOString()
          });

          // Check if it's a connection error
          const errorMsg = (error as Error)?.message?.toLowerCase() || '';
          if (errorMsg.includes('websocket') ||
            errorMsg.includes('closed') ||
            errorMsg.includes('connection') ||
            errorMsg.includes('disconnected')) {
            console.error('❌ [AUDIO] Connection error detected, stopping recording');
            this.stopRecording();
            this.updateStatus('❌ Connection lost: ' + (error as Error)?.message);
          } else {
            // For other errors, log but continue
            console.error('❌ [AUDIO] Non-fatal error, continuing:', errorMsg);
          }
        }
      };

      this.sourceNode.connect(this.scriptProcessorNode);
      this.scriptProcessorNode.connect(this.inputAudioContext.destination);

      this.isRecording = true;
      this.updateStatus('🔴 Recording... Capturing PCM chunks.');
    } catch (err) {
      console.error('Error starting recording:', err);
      this.updateStatus(`Error: ${err.message}`);
      this.stopRecording();
    }
  }

  private stopRecording() {
    if (!this.isRecording && !this.mediaStream && !this.inputAudioContext)
      return;

    this.updateStatus('Stopping recording...');

    this.isRecording = false;

    if (this.scriptProcessorNode && this.sourceNode && this.inputAudioContext) {
      this.scriptProcessorNode.disconnect();
      this.sourceNode.disconnect();
    }

    this.scriptProcessorNode = null;
    this.sourceNode = null;

    if (this.mediaStream) {
      this.mediaStream.getTracks().forEach((track) => track.stop());
      this.mediaStream = null;
    }

    // Save the recorded audio to WAV file
    if (this.recordedChunks.length > 0) {
      this.saveWAV();
    }

    this.updateStatus('Recording stopped. Audio saved. Click Start to begin again.');
  }

  private reset() {
    this.session?.close();
    this.initSession();
    this.updateStatus('Session cleared.');
  }

  render() {
    return html`
      <div>
        <div class="controls">
          <button
            id="resetButton"
            @click=${this.reset}
            ?disabled=${this.isRecording}>
            <svg
              xmlns="http://www.w3.org/2000/svg"
              height="40px"
              viewBox="0 -960 960 960"
              width="40px"
              fill="#ffffff">
              <path
                d="M480-160q-134 0-227-93t-93-227q0-134 93-227t227-93q69 0 132 28.5T720-690v-110h80v280H520v-80h168q-32-56-87.5-88T480-720q-100 0-170 70t-70 170q0 100 70 170t170 70q77 0 139-44t87-116h84q-28 106-114 173t-196 67Z" />
            </svg>
          </button>
          <button
            id="startButton"
            @click=${this.startRecording}
            ?disabled=${this.isRecording}>
            <svg
              viewBox="0 0 100 100"
              width="32px"
              height="32px"
              fill="#c80000"
              xmlns="http://www.w3.org/2000/svg">
              <circle cx="50" cy="50" r="50" />
            </svg>
          </button>
          <button
            id="stopButton"
            @click=${this.stopRecording}
            ?disabled=${!this.isRecording}>
            <svg
              viewBox="0 0 100 100"
              width="32px"
              height="32px"
              fill="#000000"
              xmlns="http://www.w3.org/2000/svg">
              <rect x="0" y="0" width="100" height="100" rx="15" />
            </svg>
          </button>
        </div>

        <div id="status"> ${this.error} </div>
        <gdm-live-audio-visuals-3d
          .inputNode=${this.inputNode}
          .outputNode=${this.outputNode}></gdm-live-audio-visuals-3d>
      </div>
    `;
  }
}
