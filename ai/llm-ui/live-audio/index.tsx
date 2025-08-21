/* tslint:disable */
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import {GoogleGenAI, LiveServerMessage, Modality, Session, StartSensitivity, EndSensitivity} from '@google/genai';
import {LitElement, css, html} from 'lit';
import {customElement, state} from 'lit/decorators.js';
import {createBlob, decode, decodeAudioData} from './utils';
import './robot-face';

@customElement('gdm-live-audio')
export class GdmLiveAudio extends LitElement {
  @state() isRecording = false;
  @state() status = '';
  @state() error = '';

  private client: GoogleGenAI;
  private session: Session;
  private sessionState: 'disconnected' | 'connecting' | 'connected' | 'closing' | 'closed' = 'disconnected';
  private inputAudioContext = new (window.AudioContext ||
    window.webkitAudioContext)({sampleRate: 16000});
  private outputAudioContext = new (window.AudioContext ||
    window.webkitAudioContext)({sampleRate: 24000});
  @state() inputNode = this.inputAudioContext.createGain();
  @state() outputNode = this.outputAudioContext.createGain();
  private nextStartTime = 0;
  private mediaStream: MediaStream;
  private sourceNode: AudioBufferSourceNode;
  private scriptProcessorNode: ScriptProcessorNode;
  private sources = new Set<AudioBufferSourceNode>();

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
    const model = 'gemini-2.5-flash-preview-native-audio-dialog';

    console.log('🔄 [SESSION] Initializing session...');
    this.sessionState = 'connecting';

    try {
      this.session = await this.client.live.connect({
        model: model,
        callbacks: {
          onopen: () => {
            console.log('✅ [SESSION] WebSocket connection opened');
            this.sessionState = 'connected';
            this.updateStatus('Opened');
          },
          onmessage: async (message: LiveServerMessage) => {
            // console.log(message);
            const transcription = message.serverContent?.outputTranscription;
            if(transcription) {
              console.log(transcription);
            }
            const audio =
              message.serverContent?.modelTurn?.parts[0]?.inlineData;

            if (audio) {
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
              source.addEventListener('ended', () =>{
                this.sources.delete(source);
              });

              source.start(this.nextStartTime);
              this.nextStartTime = this.nextStartTime + audioBuffer.duration;
              this.sources.add(source);
            }

            const interrupted = message.serverContent?.interrupted;
            if(interrupted) {
              for(const source of this.sources.values()) {
                source.stop();
                this.sources.delete(source);
              }
              this.nextStartTime = 0;
            }
          },
          onerror: (e: ErrorEvent) => {
            console.error('❌ [SESSION] WebSocket/Connection Error:', e);
            console.error('❌ [SESSION] Current state:', this.sessionState);
            this.sessionState = 'closed';
            this.handleGoogleAIError(e);
          },
          onclose: (e: CloseEvent) => {
            console.warn('🚪 [SESSION] WebSocket connection closed:', {
              code: e.code,
              reason: e.reason,
              wasClean: e.wasClean,
              previousState: this.sessionState
            });
            this.sessionState = 'closed';
            this.updateStatus('Close:' + e.reason);
          },
        },
        config: {
          responseModalities: [Modality.AUDIO],
          speechConfig: {
            voiceConfig: {prebuiltVoiceConfig: {voiceName: 'Orus'}},
            // languageCode: 'en-GB'
          },
          outputAudioTranscription: {},
          realtimeInputConfig: {automaticActivityDetection: {startOfSpeechSensitivity: StartSensitivity.START_SENSITIVITY_LOW, endOfSpeechSensitivity: EndSensitivity.END_SENSITIVITY_HIGH}},
          systemInstruction: "You are a cute robot dog and have the intelligence and knowledge of a 6 year old child."
        },
      });
      console.log('✅ [SESSION] Session successfully created');
    } catch (e) {
      console.error('❌ [SESSION] Failed to create session:', e);
      this.sessionState = 'closed';
      this.handleGoogleAIError(e);
    }
  }

  private updateStatus(msg: string) {
    this.status = msg;
  }

  private updateError(msg: string) {
    this.error = msg;
  }

  private handleGoogleAIError(error: any) {
    console.error('Google AI API Error:', error);
    
    // Check for HTTP status codes
    if (error.status || error.code) {
      const statusCode = error.status || error.code;
      
      switch (statusCode) {
        case 429:
          console.error('❌ Rate limit exceeded (429): Too many requests');
          this.updateError('Rate limit exceeded. Please wait before trying again.');
          break;
        case 500:
          console.error('❌ Internal server error (500): Google AI service error');
          this.updateError('Google AI service temporarily unavailable.');
          break;
        case 503:
          console.error('❌ Service unavailable (503): Google AI overloaded');
          this.updateError('Google AI service overloaded. Please try again later.');
          break;
        case 401:
          console.error('❌ Unauthorized (401): Invalid API key');
          this.updateError('Invalid API key. Please check your credentials.');
          break;
        case 403:
          console.error('❌ Forbidden (403): Access denied');
          this.updateError('Access denied. Please check your permissions.');
          break;
        case 400:
          console.error('❌ Bad request (400): Invalid request format');
          this.updateError('Invalid request format.');
          break;
        case 502:
          console.error('❌ Bad gateway (502): Upstream server error');
          this.updateError('Google AI gateway error. Please try again.');
          break;
        case 504:
          console.error('❌ Gateway timeout (504): Request timeout');
          this.updateError('Request timeout. Please try again.');
          break;
        default:
          console.error(`❌ HTTP Error (${statusCode}):`, error.message || 'Unknown error');
          this.updateError(`API error (${statusCode}): ${error.message || 'Unknown error'}`);
      }
    } else if (error.message) {
      // Handle generic errors
      console.error('❌ Google AI Error:', error.message);
      this.updateError(`Google AI error: ${error.message}`);
    } else {
      // Handle unknown errors
      console.error('❌ Unknown Google AI Error:', error);
      this.updateError('Unknown Google AI error occurred.');
    }
  }

  private async startRecording() {
    console.log('🎤 [RECORDING] Start recording requested, current session state:', this.sessionState);
    
    if (this.isRecording) {
      console.log('⚠️  [RECORDING] Already recording, ignoring request');
      return;
    }

    if (this.sessionState !== 'connected') {
      console.error('❌ [RECORDING] Cannot start recording - session not connected. State:', this.sessionState);
      this.updateStatus('❌ Cannot start recording - connection not ready');
      return;
    }

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
        if (!this.isRecording) {
          console.log('⏹️  [AUDIO] Skipping audio processing - not recording');
          return;
        }

        if (!this.session) {
          console.error('❌ [AUDIO] No session available for sendRealtimeInput');
          return;
        }

        if (this.sessionState !== 'connected') {
          console.error('❌ [AUDIO] Attempting to send data but session state is:', this.sessionState);
          console.error('❌ [AUDIO] Session object exists:', !!this.session);
          return;
        }

        const inputBuffer = audioProcessingEvent.inputBuffer;
        const pcmData = inputBuffer.getChannelData(0);

        try {
          // Only log every 100th audio frame to avoid spam
          if (Math.random() < 0.01) {
            console.log('🎤 [AUDIO] Sending realtime input, session state:', this.sessionState);
          }
          this.session.sendRealtimeInput({media: createBlob(pcmData)});
        } catch (error) {
          console.error('❌ [AUDIO] Error sending realtime input:', error);
          console.error('❌ [AUDIO] Session state at error:', this.sessionState);
          console.error('❌ [AUDIO] Session exists:', !!this.session);
          
          // Stop recording if we can't send data
          this.stopRecording();
          this.updateStatus('❌ Connection lost during recording');
        }
      };

      this.sourceNode.connect(this.scriptProcessorNode);
      this.scriptProcessorNode.connect(this.inputAudioContext.destination);

      this.isRecording = true;
      console.log('✅ [RECORDING] Recording started successfully');
      this.updateStatus('🔴 Recording... Capturing PCM chunks.');
    } catch (err) {
      console.error('❌ [RECORDING] Error starting recording:', err);
      this.updateStatus(`Error: ${err.message}`);
      this.stopRecording();
    }
  }

  private stopRecording() {
    console.log('🛑 [RECORDING] Stop recording requested, current state:', {
      isRecording: this.isRecording,
      hasMediaStream: !!this.mediaStream,
      hasAudioContext: !!this.inputAudioContext,
      sessionState: this.sessionState
    });
    
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

    this.updateStatus('Recording stopped. Click Start to begin again.');
  }

  private reset() {
    console.log('🔄 [RESET] Resetting session, current state:', this.sessionState);
    
    try {
      if (this.session) {
        console.log('🚪 [RESET] Closing existing session');
        this.sessionState = 'closing';
        this.session.close();
      }
    } catch (e) {
      console.error('❌ [RESET] Error closing session:', e);
      this.handleGoogleAIError(e);
    }
    
    this.sessionState = 'disconnected';
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
        <gdm-robot-face
          .inputNode=${this.inputNode}
          .outputNode=${this.outputNode}></gdm-robot-face>
      </div>
    `;
  }
}