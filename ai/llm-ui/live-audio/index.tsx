/* tslint:disable */
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { LiveServerMessage } from '@google/genai';
import { LitElement } from 'lit';
import { customElement, state } from 'lit/decorators.js';
import { createBlob, decode, decodeAudioData } from './utils';
import { AudioManager } from './audio-manager';
import { SessionManager, SessionState } from './session-manager';
import { VisualizerManager } from './visualizer-manager';
import { ConsoleManager } from './console-manager';
import { ErrorManager } from './error-manager';
import { handleToolCall } from './tools';
import { styles } from './styles';
import { renderTemplate, TemplateProps } from './template';
import './robot-face';

@customElement('gdm-live-audio')
export class GdmLiveAudio extends LitElement {
  @state() isRecording = false;
  @state() status = '';
  @state() error = '';
  @state() robotMode = 'idle';
  @state() selectedModel = 'gemini-live-2.5-flash-preview';
  @state() showConsole = false;
  @state() showInputAnalyzer = true;
  @state() showOutputAnalyzer = true;

  private audioManager: AudioManager;
  private sessionManager: SessionManager;
  private visualizerManager: VisualizerManager;
  private consoleManager: ConsoleManager;
  private errorManager: ErrorManager;
  private sessionState: SessionState = 'disconnected';
  private mediaStream: MediaStream;
  private sourceNode: MediaStreamAudioSourceNode;
  private audioWorkletNode: AudioWorkletNode | null = null;

  static styles = styles;

  constructor() {
    super();
    this.audioManager = new AudioManager();
    this.sessionManager = new SessionManager(process.env.GEMINI_API_KEY!);
    this.visualizerManager = new VisualizerManager();
    this.consoleManager = new ConsoleManager(() => this.requestUpdate());
    this.errorManager = new ErrorManager((error) => { this.error = error; this.requestUpdate(); });
    this.initClient();
  }


  private async initClient() {
    await this.audioManager.initAudio(this.showInputAnalyzer, this.showOutputAnalyzer);
    this.initSession();
  }

  private async initSession() {
    const model = this.selectedModel;

    try {
      await this.sessionManager.initSession(model, {
        onOpen: () => {
          this.sessionState = 'connected';
          this.updateStatus('Opened');
        },
        onMessage: this.handleMessage.bind(this),
        onError: (e: ErrorEvent) => {
          this.sessionState = 'closed';
          this.errorManager.handleGoogleAIError(e);
        },
        onClose: (e: CloseEvent) => {
          this.sessionState = 'closed';
          this.updateStatus('Close:' + e.reason);
        }
      });
    } catch (e) {
      this.sessionState = 'closed';
      this.errorManager.handleGoogleAIError(e);
    }
  }

  private async handleMessage(message: LiveServerMessage) {
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

    const audio = message.serverContent?.modelTurn?.parts[0]?.inlineData;

    if (audio) {
      const nextStartTime = Math.max(
        this.audioManager.getNextStartTime(),
        this.audioManager.getOutputAudioContext().currentTime,
      );

      const audioBuffer = await decodeAudioData(
        decode(audio.data),
        this.audioManager.getOutputAudioContext(),
        24000,
        1,
      );
      const source = this.audioManager.getOutputAudioContext().createBufferSource();
      source.buffer = audioBuffer;
      source.connect(this.audioManager.getOutputNode());
      source.addEventListener('ended', () => {
        this.audioManager.removeSource(source);
      });

      source.start(nextStartTime);
      this.audioManager.setNextStartTime(nextStartTime + audioBuffer.duration);
      this.audioManager.addSource(source);
    }

    const interrupted = message.serverContent?.interrupted;
    if (interrupted) {
      this.audioManager.stopAllSources();
    }

    // Handle tool calls
    const toolCall = (message as any).toolCall;
    if (toolCall) {
      handleToolCall(
        toolCall,
        this.toggleInputAnalyzer.bind(this),
        this.toggleOutputAnalyzer.bind(this),
        this.showInputAnalyzer,
        this.showOutputAnalyzer
      ).then(functionResponses => {
        this.sessionManager.sendToolResponse({ functionResponses });
      }).catch(error => {
        console.error('❌ [TOOLS] Error handling tool call:', error);
      });
    }
  }

  private updateStatus(msg: string) {
    this.status = msg;
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

    this.audioManager.getInputAudioContext().resume();

    this.updateStatus('Requesting microphone access...');

    try {
      this.mediaStream = await navigator.mediaDevices.getUserMedia({
        audio: true,
        video: false,
      });

      this.updateStatus('Microphone access granted. Starting capture...');

      this.sourceNode = this.audioManager.getInputAudioContext().createMediaStreamSource(
        this.mediaStream,
      );
      this.sourceNode.connect(this.audioManager.getInputNode());

      // Create AudioWorklet node
      this.audioWorkletNode = await this.audioManager.createAudioWorkletNode(this.sourceNode);
      
      // Set up message handler for audio data
      this.audioWorkletNode.port.onmessage = (event) => {
        if (event.data.type === 'audioData') {
          if (!this.isRecording) {
            return;
          }

          if (!this.sessionManager.isConnected()) {
            console.error('❌ [AUDIO] Session not connected');
            return;
          }

          const pcmData = event.data.data;

          try {
            // Only log every 100th audio frame to avoid spam
            if (Math.random() < 0.01) {
              console.log('🎤 [AUDIO] Sending realtime input, session state:', this.sessionState);
            }
            this.sessionManager.sendRealtimeInput({ media: createBlob(pcmData) });
          } catch (error) {
            console.error('❌ [AUDIO] Error sending realtime input:', error);

            // Stop recording if we can't send data
            this.stopRecording();
            this.updateStatus('❌ Connection lost during recording');
          }
        }
      };

      // Enable recording in the worklet
      this.audioManager.setWorkletRecording(true);

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
      hasAudioContext: !!this.audioManager.getInputAudioContext(),
      sessionState: this.sessionState
    });

    if (!this.isRecording && !this.mediaStream && !this.audioManager.getInputAudioContext())
      return;

    this.updateStatus('Stopping recording...');

    this.isRecording = false;

    // Disable recording in the worklet
    this.audioManager.setWorkletRecording(false);
    
    if (this.audioWorkletNode && this.sourceNode && this.audioManager.getInputAudioContext()) {
      this.audioManager.disconnectWorklet();
      this.sourceNode.disconnect();
    }

    this.audioWorkletNode = null;
    this.sourceNode = null;

    if (this.mediaStream) {
      this.mediaStream.getTracks().forEach((track) => track.stop());
      this.mediaStream = null;
    }

    this.updateStatus('Recording stopped. Click Start to begin again.');
  }

  private reset() {
    try {
      this.sessionManager.close();
    } catch (e) {
      console.error('❌ [RESET] Error closing session:', e);
      this.errorManager.handleGoogleAIError(e);
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
    this.consoleManager.clearLogs();
  }

  private toggleInputAnalyzer = () => {
    this.showInputAnalyzer = this.audioManager.toggleInputAnalyzer(this.showInputAnalyzer);
    
    if (!this.showInputAnalyzer) {
      this.visualizerManager.stopVisualizer(this.shadowRoot);
    } else if (this.isRecording) {
      setTimeout(() => this.startVisualizer(), 50);
    }
  }

  private toggleOutputAnalyzer = () => {
    this.showOutputAnalyzer = this.audioManager.toggleOutputAnalyzer(this.showOutputAnalyzer);
    
    if (!this.showOutputAnalyzer) {
      this.visualizerManager.stopOutputVisualizer(this.shadowRoot);
    } else {
      setTimeout(() => this.startOutputVisualizer(), 50);
    }
  }

  protected updated(changedProperties: Map<string, any>) {
    super.updated(changedProperties);

    // Auto-scroll console to bottom when new logs are added
    if (this.showConsole) {
      const consoleContent = this.shadowRoot?.querySelector('.console-content');
      if (consoleContent) {
        consoleContent.scrollTop = consoleContent.scrollHeight;
      }
    }

    // Start visualizers when component is ready
    if (changedProperties.has('isRecording') && this.isRecording) {
      if (this.showInputAnalyzer) {
        this.startVisualizer();
      }
    } else if (changedProperties.has('isRecording') && !this.isRecording) {
      if (this.showInputAnalyzer) {
        this.visualizerManager.stopVisualizer(this.shadowRoot);
      }
    }
  }

  connectedCallback() {
    super.connectedCallback();
    // Start output visualizer when component connects (runs only if enabled)
    setTimeout(() => {
      if (this.showOutputAnalyzer) {
        this.startOutputVisualizer();
      }
    }, 100);
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.visualizerManager.cleanup();
  }

  private startVisualizer() {
    this.visualizerManager.startVisualizer(
      this.shadowRoot,
      this.audioManager.getInputAnalyser(),
      this.showInputAnalyzer,
      this.isRecording
    );
  }

  private startOutputVisualizer() {
    this.visualizerManager.startOutputVisualizer(
      this.shadowRoot,
      this.audioManager.getOutputAnalyser(),
      this.showOutputAnalyzer
    );
  }

  render() {
    const props: TemplateProps = {
      robotMode: this.robotMode,
      selectedModel: this.selectedModel,
      onModelChange: this.onModelChange,
      onReset: this.reset.bind(this),
      onStartRecording: this.startRecording.bind(this),
      onStopRecording: this.stopRecording.bind(this),
      onToggleInputAnalyzer: this.toggleInputAnalyzer,
      onToggleOutputAnalyzer: this.toggleOutputAnalyzer,
      onToggleConsole: this.toggleConsole,
      onClearConsole: this.clearConsole,
      onRobotModeChange: this.onRobotModeChange,
      isRecording: this.isRecording,
      showInputAnalyzer: this.showInputAnalyzer,
      showOutputAnalyzer: this.showOutputAnalyzer,
      showConsole: this.showConsole,
      consoleLogs: this.consoleManager.getLogs(),
      error: this.error,
      inputNode: this.audioManager.getInputNode(),
      outputNode: this.audioManager.getOutputNode()
    };
    
    return renderTemplate(props);
  }
}