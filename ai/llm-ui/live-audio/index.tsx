/* tslint:disable */
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { GoogleGenAI, LiveServerMessage, Modality, Session, StartSensitivity, EndSensitivity } from '@google/genai';
import { LitElement, css, html } from 'lit';
import { customElement, state } from 'lit/decorators.js';
import { createBlob, decode, decodeAudioData } from './utils';
import './robot-face';

@customElement('gdm-live-audio')
export class GdmLiveAudio extends LitElement {
  @state() isRecording = false;
  @state() status = '';
  @state() error = '';
  @state() robotMode = 'idle';
  @state() selectedModel = 'gemini-live-2.5-flash-preview';
  @state() showConsole = false;
  @state() consoleLogs: Array<{timestamp: string, level: string, message: string}> = [];

  private client: GoogleGenAI;
  private session: Session;
  private sessionState: 'disconnected' | 'connecting' | 'connected' | 'closing' | 'closed' = 'disconnected';
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
  private inputAnalyser: AnalyserNode;
  private visualizerAnimationId: number;

  static styles = css`
    #status {
      position: absolute;
      bottom: 5vh;
      left: 0;
      right: 0;
      z-index: 10;
      text-align: center;
    }

    .mode-indicator {
      position: absolute;
      top: 20px;
      left: 20px;
      z-index: 10;
      background: rgba(0, 0, 0, 0.7);
      color: white;
      padding: 8px 16px;
      border-radius: 20px;
      font-size: 14px;
      font-weight: 500;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      border: 1px solid rgba(255, 255, 255, 0.2);
      transition: all 0.3s ease;
    }

    .mode-indicator.idle {
      background: rgba(100, 100, 100, 0.7);
      border-color: rgba(150, 150, 150, 0.3);
    }

    .mode-indicator.listening {
      background: rgba(25, 162, 230, 0.7);
      border-color: rgba(25, 162, 230, 0.5);
      animation: pulse-listening 0.9s ease-in-out infinite;
    }

    .mode-indicator.thinking {
      background: rgba(128, 90, 213, 0.7);
      border-color: rgba(128, 90, 213, 0.5);
    }

    .mode-indicator.speaking {
      background: rgba(255, 80, 80, 0.7);
      border-color: rgba(255, 80, 80, 0.5);
      animation: pulse-speaking 0.32s ease-in-out infinite;
    }

    .mode-indicator.muted {
      background: rgba(60, 60, 60, 0.7);
      border-color: rgba(80, 80, 80, 0.3);
      filter: grayscale(1);
    }

    @keyframes pulse-listening {
      0%, 100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(25, 162, 230, 0.4); }
      50% { transform: scale(1.05); box-shadow: 0 0 0 10px rgba(25, 162, 230, 0); }
    }

    @keyframes pulse-speaking {
      0%, 100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(255, 80, 80, 0.4); }
      50% { transform: scale(1.03); box-shadow: 0 0 0 8px rgba(255, 80, 80, 0); }
    }

    .model-selector {
      position: absolute;
      top: 20px;
      right: 20px;
      z-index: 10;
    }

    .model-selector select {
      background: rgba(0, 0, 0, 0.7);
      color: white;
      border: 1px solid rgba(255, 255, 255, 0.2);
      border-radius: 8px;
      padding: 8px 12px;
      font-size: 14px;
      font-family: inherit;
      cursor: pointer;
      outline: none;
      transition: all 0.3s ease;
    }

    .model-selector select:hover {
      background: rgba(0, 0, 0, 0.8);
      border-color: rgba(255, 255, 255, 0.3);
    }

    .model-selector select:focus {
      border-color: rgba(25, 162, 230, 0.5);
      box-shadow: 0 0 0 2px rgba(25, 162, 230, 0.2);
    }

    .model-selector option {
      background: #1a1a1a;
      color: white;
      padding: 8px;
    }

    .console-toggle {
      position: absolute;
      bottom: 20px;
      right: 20px;
      z-index: 10;
      background: rgba(0, 0, 0, 0.7);
      color: white;
      border: 1px solid rgba(255, 255, 255, 0.2);
      border-radius: 8px;
      padding: 8px 12px;
      font-size: 12px;
      cursor: pointer;
      transition: all 0.3s ease;
    }

    .console-toggle:hover {
      background: rgba(0, 0, 0, 0.8);
      border-color: rgba(255, 255, 255, 0.3);
    }

    .console-panel {
      position: fixed;
      bottom: 0;
      left: 0;
      right: 0;
      height: 30vh;
      background: rgba(0, 0, 0, 0.9);
      border-top: 1px solid rgba(255, 255, 255, 0.2);
      z-index: 20;
      transform: translateY(100%);
      transition: transform 0.3s ease;
      display: flex;
      flex-direction: column;
    }

    .console-panel.show {
      transform: translateY(0);
    }

    .console-header {
      padding: 8px 16px;
      background: rgba(255, 255, 255, 0.1);
      border-bottom: 1px solid rgba(255, 255, 255, 0.2);
      display: flex;
      justify-content: between;
      align-items: center;
      font-size: 12px;
      color: #ccc;
    }

    .console-close {
      background: none;
      border: none;
      color: white;
      font-size: 16px;
      cursor: pointer;
      padding: 0;
      margin-left: auto;
    }

    .console-content {
      flex: 1;
      overflow-y: auto;
      padding: 8px;
      font-family: 'Monaco', 'Menlo', 'Ubuntu Mono', monospace;
      font-size: 11px;
      line-height: 1.4;
    }

    .console-log {
      padding: 2px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.05);
      display: flex;
      gap: 8px;
    }

    .console-timestamp {
      color: #666;
      flex-shrink: 0;
      width: 60px;
    }

    .console-level {
      flex-shrink: 0;
      width: 50px;
      font-weight: bold;
    }

    .console-level.log {
      color: #fff;
    }

    .console-level.error {
      color: #ff6b6b;
    }

    .console-level.warn {
      color: #ffa726;
    }

    .console-level.info {
      color: #42a5f5;
    }

    .console-message {
      flex: 1;
      color: #ccc;
      word-break: break-word;
    }

    .audio-visualizer {
      position: absolute;
      top: 80px;
      left: 20px;
      z-index: 10;
      width: 200px;
      height: 80px;
      background: rgba(0, 0, 0, 0.7);
      border: 1px solid rgba(255, 255, 255, 0.2);
      border-radius: 8px;
      padding: 8px;
    }

    .visualizer-canvas {
      width: 100%;
      height: 100%;
      background: #000;
      border-radius: 4px;
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
    this.setupConsoleInterception();
    this.initClient();
  }

  private setupConsoleInterception() {
    const originalConsole = {
      log: console.log,
      error: console.error,
      warn: console.warn,
      info: console.info
    };

    const addLog = (level: string, ...args: any[]) => {
      const timestamp = new Date().toLocaleTimeString();
      const message = args.map(arg => 
        typeof arg === 'object' ? JSON.stringify(arg, null, 2) : String(arg)
      ).join(' ');
      
      this.consoleLogs = [...this.consoleLogs.slice(-99), { timestamp, level, message }];
    };

    console.log = (...args: any[]) => {
      originalConsole.log(...args);
      addLog('log', ...args);
    };

    console.error = (...args: any[]) => {
      originalConsole.error(...args);
      addLog('error', ...args);
    };

    console.warn = (...args: any[]) => {
      originalConsole.warn(...args);
      addLog('warn', ...args);
    };

    console.info = (...args: any[]) => {
      originalConsole.info(...args);
      addLog('info', ...args);
    };
  }

  private initAudio() {
    this.nextStartTime = this.outputAudioContext.currentTime;
    
    // Set up input analyzer for visualizer
    this.inputAnalyser = this.inputAudioContext.createAnalyser();
    this.inputAnalyser.fftSize = 256;
    this.inputAnalyser.smoothingTimeConstant = 0.8;
    this.inputNode.connect(this.inputAnalyser);
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
    const model = this.selectedModel;

    console.log(`🔄 [SESSION] Initializing session with model: ${model}...`);
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
            if (transcription) {
              console.log(transcription);
            }

            // Check if this is a setup message (indicates AI is thinking)
            const setupComplete = message.setupComplete;
            if (setupComplete) {
              // AI is now ready and thinking/processing
              this.robotMode = 'thinking';
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
              source.addEventListener('ended', () => {
                this.sources.delete(source);
              });

              source.start(this.nextStartTime);
              this.nextStartTime = this.nextStartTime + audioBuffer.duration;
              this.sources.add(source);
            }

            const interrupted = message.serverContent?.interrupted;
            if (interrupted) {
              for (const source of this.sources.values()) {
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
            voiceConfig: { prebuiltVoiceConfig: { voiceName: 'Orus' } },
            // languageCode: 'en-GB'
          },
          outputAudioTranscription: {},
          realtimeInputConfig: { automaticActivityDetection: { startOfSpeechSensitivity: StartSensitivity.START_SENSITIVITY_LOW, endOfSpeechSensitivity: EndSensitivity.END_SENSITIVITY_HIGH } },
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
          this.session.sendRealtimeInput({ media: createBlob(pcmData) });
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

  private onRobotModeChange = (event: CustomEvent) => {
    this.robotMode = event.detail.mode;
  }

  private onModelChange = (event: Event) => {
    const target = event.target as HTMLSelectElement;
    const newModel = target.value;
    
    if (newModel !== this.selectedModel) {
      console.log(`🔄 [MODEL] Switching from ${this.selectedModel} to ${newModel}`);
      this.selectedModel = newModel;
      
      // Stop recording if active
      if (this.isRecording) {
        this.stopRecording();
      }
      
      // Reset session with new model
      this.reset();
    }
  }

  private toggleConsole = () => {
    this.showConsole = !this.showConsole;
  }

  private clearConsole = () => {
    this.consoleLogs = [];
  }

  protected updated(changedProperties: Map<string, any>) {
    super.updated(changedProperties);
    
    // Auto-scroll console to bottom when new logs are added
    if (changedProperties.has('consoleLogs') && this.showConsole) {
      const consoleContent = this.shadowRoot?.querySelector('.console-content');
      if (consoleContent) {
        consoleContent.scrollTop = consoleContent.scrollHeight;
      }
    }

    // Start visualizer when component is ready
    if (changedProperties.has('isRecording') && this.isRecording) {
      this.startVisualizer();
    } else if (changedProperties.has('isRecording') && !this.isRecording) {
      this.stopVisualizer();
    }
  }

  private startVisualizer() {
    const canvas = this.shadowRoot?.querySelector('.visualizer-canvas') as HTMLCanvasElement;
    if (!canvas || !this.inputAnalyser) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    canvas.width = 184; // 200px - 16px padding
    canvas.height = 64; // 80px - 16px padding

    const bufferLength = this.inputAnalyser.frequencyBinCount;
    const dataArray = new Uint8Array(bufferLength);

    const animate = () => {
      this.inputAnalyser.getByteFrequencyData(dataArray);

      // Clear canvas
      ctx.fillStyle = '#000';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // Draw frequency bars
      const barWidth = canvas.width / bufferLength * 2;
      let x = 0;

      for (let i = 0; i < bufferLength / 2; i++) { // Only show lower frequencies
        const barHeight = (dataArray[i] / 255) * canvas.height;
        
        // Color based on frequency - blue for low, green for mid, red for high
        const hue = (i / (bufferLength / 2)) * 120; // 0-120 (blue to green)
        ctx.fillStyle = `hsl(${120 - hue}, 80%, 60%)`;
        
        ctx.fillRect(x, canvas.height - barHeight, barWidth, barHeight);
        x += barWidth + 1;
      }

      if (this.isRecording) {
        this.visualizerAnimationId = requestAnimationFrame(animate);
      }
    };

    this.visualizerAnimationId = requestAnimationFrame(animate);
  }

  private stopVisualizer() {
    if (this.visualizerAnimationId) {
      cancelAnimationFrame(this.visualizerAnimationId);
    }

    // Clear canvas
    const canvas = this.shadowRoot?.querySelector('.visualizer-canvas') as HTMLCanvasElement;
    if (canvas) {
      const ctx = canvas.getContext('2d');
      if (ctx) {
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
      }
    }
  }

  render() {
    return html`
      <div>
        <div class="mode-indicator ${this.robotMode}">
          ${this.robotMode}
        </div>

        <div class="model-selector">
          <select @change=${this.onModelChange} .value=${this.selectedModel}>
            <option value="gemini-live-2.5-flash-preview">Gemini Live 2.5 Flash</option>
            <option value="gemini-2.5-flash-preview-native-audio-dialog">Gemini 2.5 Flash Native Audio</option>
          </select>
        </div>

        <div class="audio-visualizer">
          <canvas class="visualizer-canvas"></canvas>
        </div>

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
          .outputNode=${this.outputNode}
          @mode-change=${this.onRobotModeChange}></gdm-robot-face>

        <button class="console-toggle" @click=${this.toggleConsole}>
          ${this.showConsole ? 'Hide Console' : 'Show Console'}
        </button>

        <div class="console-panel ${this.showConsole ? 'show' : ''}">
          <div class="console-header">
            <span>Console (${this.consoleLogs.length} logs)</span>
            <button @click=${this.clearConsole} style="margin-left: 10px; background: none; border: 1px solid #666; color: white; padding: 2px 6px; border-radius: 3px; font-size: 10px; cursor: pointer;">Clear</button>
            <button class="console-close" @click=${this.toggleConsole}>×</button>
          </div>
          <div class="console-content">
            ${this.consoleLogs.map(log => html`
              <div class="console-log">
                <span class="console-timestamp">${log.timestamp}</span>
                <span class="console-level ${log.level}">${log.level.toUpperCase()}</span>
                <span class="console-message">${log.message}</span>
              </div>
            `)}
          </div>
        </div>
      </div>
    `;
  }
}