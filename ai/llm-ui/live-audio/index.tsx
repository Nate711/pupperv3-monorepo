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
import { sendRobotCommand } from './tools';
import { styles } from './styles';
import { renderTemplate, TemplateProps } from './template';
import './robot-face';

@customElement('gdm-live-audio')
export class GdmLiveAudio extends LitElement {
  @state() isRecording = false;
  private autoStartRecording = true;
  @state() status = '';
  @state() error = '';
  @state() selectedModel = 'gemini-live-2.5-flash-preview';
  @state() showConsole = false;
  @state() showInputAnalyzer = true;
  @state() showOutputAnalyzer = true;
  @state() batteryPercentage: string = 'N/A';
  @state() cpuUsage: string = 'N/A';
  @state() inputTranscriptions: Array<{text: string, timestamp: number}> = [];
  @state() outputTranscriptions: Array<{text: string, timestamp: number}> = [];
  private inputTranscriptionBuffer: string[] = [];
  private outputTranscriptionBuffer: string[] = [];
  private lastInputTimestamp: number = 0;
  private lastOutputTimestamp: number = 0;
  private transcriptionDebounceTimer: number | null = null;

  private audioManager: AudioManager;
  private sessionManager: SessionManager;
  private visualizerManager: VisualizerManager;
  private consoleManager: ConsoleManager;
  private errorManager: ErrorManager;
  private sessionState: SessionState = 'disconnected';
  private mediaStream: MediaStream;
  private sourceNode: MediaStreamAudioSourceNode;
  private audioWorkletNode: AudioWorkletNode | null = null;
  private batteryUpdateInterval: number | null = null;

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

          // Auto-start recording if enabled and audio context allows it
          if (this.autoStartRecording && !this.isRecording) {
            setTimeout(() => {
              // Only auto-start if AudioContext is running (user has interacted)
              if (this.audioManager.getInputAudioContext().state === 'running') {
                this.startRecording();
              } else {
                console.log('🎤 [RECORDING] Auto-start skipped - audio context suspended (user interaction required)');
                this.updateStatus('🎤 Click "Start Recording" to begin (browser autoplay policy)');
              }
            }, 500); // Small delay to ensure everything is ready
          }
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
    const outputTranscription = message.serverContent?.outputTranscription;
    if (outputTranscription) {
      console.log('🤖 [OUTPUT]:', outputTranscription.text);
      this.processTranscriptionFragment('output', outputTranscription.text);
    }
    
    const inputTranscription = message.serverContent?.inputTranscription;
    if (inputTranscription) {
      console.log('🎤 [INPUT]:', inputTranscription.text);
      this.processTranscriptionFragment('input', inputTranscription.text);
    }
    
    // Check for turn completion to flush buffers
    const turnComplete = message.serverContent?.turnComplete;
    if (turnComplete) {
      this.flushTranscriptionBuffers();
    }

    // Check if this is a setup message (indicates AI is thinking)
    const setupComplete = message.setupComplete;
    if (setupComplete) {
      // AI is now ready and processing
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

    // Resume audio context (will only work after user interaction)
    await this.audioManager.getInputAudioContext().resume();

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
            // Only log every 1000th audio frame (10s or so) to avoid spam
            if (Math.random() < 0.001) {
              console.log('🎤 [AUDIO] Sending realtime input, session state:', this.sessionState);
            }
            this.sessionManager.sendRealtimeInput({ media: createBlob(pcmData) as any });
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
    
    // Flush any remaining transcription fragments
    this.flushTranscriptionBuffers();

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
    
    // Clear transcription buffers and history
    this.flushTranscriptionBuffers();
    this.transcriptions = [];

    this.sessionState = 'disconnected';
    this.initSession();
    this.updateStatus('Session cleared.');
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

    // Auto-scroll console when logs change (avoid forced reflow)
    if (changedProperties.has('consoleLogs') && this.showConsole) {
      requestAnimationFrame(() => {
        const consoleContent = this.shadowRoot?.querySelector('.console-content');
        if (consoleContent?.lastElementChild) {
          consoleContent.lastElementChild.scrollIntoView({ block: 'end', behavior: 'auto' });
        }
      });
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
    // Start system monitoring
    this.startSystemMonitoring();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.visualizerManager.cleanup();
    this.stopSystemMonitoring();
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

  private async updateBatteryPercentage() {
    try {
      const result = await sendRobotCommand("get_battery");
      if (result.success && result.response && result.response.battery_percentage !== undefined) {
        this.batteryPercentage = `${result.response.battery_percentage}%`;
      } else {
        this.batteryPercentage = 'N/A';
      }
    } catch (error) {
      this.batteryPercentage = 'N/A';
    }
  }

  private async updateCpuUsage() {
    try {
      const result = await sendRobotCommand("get_cpu_usage");
      if (result.success && result.response && result.response.cpu_usage !== undefined) {
        this.cpuUsage = `${Math.round(result.response.cpu_usage)}%`;
      } else {
        this.cpuUsage = 'N/A';
      }
    } catch (error) {
      console.log('💻 [CPU] Error getting CPU usage:', error);
      this.cpuUsage = 'N/A';
    }
  }

  private systemStatsDebounceTimer: number | null = null;
  
  private async updateSystemStats() {
    await Promise.all([
      this.updateBatteryPercentage(),
      this.updateCpuUsage()
    ]);
    
    // Debounce the requestUpdate to avoid excessive re-renders
    if (this.systemStatsDebounceTimer) {
      clearTimeout(this.systemStatsDebounceTimer);
    }
    
    this.systemStatsDebounceTimer = window.setTimeout(() => {
      this.requestUpdate();
      this.systemStatsDebounceTimer = null;
    }, 50); // Small debounce to batch updates
  }

  private startSystemMonitoring() {
    this.updateSystemStats();
    this.batteryUpdateInterval = window.setInterval(() => {
      this.updateSystemStats();
    }, 10000); // Increased to 10 seconds for less frequent updates
  }

  private stopSystemMonitoring() {
    if (this.batteryUpdateInterval) {
      clearInterval(this.batteryUpdateInterval);
      this.batteryUpdateInterval = null;
    }
  }

  private processTranscriptionFragment(type: 'input' | 'output', fragment: string) {
    const now = Date.now();
    const TIMEOUT_MS = 2000; // 2 seconds timeout for incomplete sentences
    
    // Check if we should flush due to timeout
    if (type === 'input' && this.lastInputTimestamp && (now - this.lastInputTimestamp) > TIMEOUT_MS && this.inputTranscriptionBuffer.length > 0) {
      this.flushInputBuffer();
    }
    if (type === 'output' && this.lastOutputTimestamp && (now - this.lastOutputTimestamp) > TIMEOUT_MS && this.outputTranscriptionBuffer.length > 0) {
      this.flushOutputBuffer();
    }
    
    // Add fragment to appropriate buffer (using array for efficiency)
    if (type === 'input') {
      this.inputTranscriptionBuffer.push(fragment);
      this.lastInputTimestamp = now;
      
      // Check for sentence endings
      const bufferText = this.inputTranscriptionBuffer.join('');
      const sentences = this.extractCompleteSentences(bufferText);
      if (sentences.complete.length > 0) {
        sentences.complete.forEach(sentence => {
          this.addTranscription('input', sentence);
        });
        this.inputTranscriptionBuffer = sentences.remaining ? [sentences.remaining] : [];
      }
    } else {
      this.outputTranscriptionBuffer.push(fragment);
      this.lastOutputTimestamp = now;
      
      // Check for sentence endings
      const bufferText = this.outputTranscriptionBuffer.join('');
      const sentences = this.extractCompleteSentences(bufferText);
      if (sentences.complete.length > 0) {
        sentences.complete.forEach(sentence => {
          this.addTranscription('output', sentence);
        });
        this.outputTranscriptionBuffer = sentences.remaining ? [sentences.remaining] : [];
      }
    }
  }
  
  private extractCompleteSentences(text: string): { complete: string[], remaining: string } {
    // Match sentences ending with . ! ? or ... followed by space or end of string
    const sentenceRegex = /[.!?]+(?:\s+|$)/g;
    const matches = Array.from(text.matchAll(sentenceRegex));
    
    if (matches.length === 0) {
      return { complete: [], remaining: text };
    }
    
    const complete: string[] = [];
    let lastIndex = 0;
    
    matches.forEach(match => {
      const endIndex = match.index! + match[0].length;
      const sentence = text.substring(lastIndex, endIndex).trim();
      if (sentence) {
        complete.push(sentence);
      }
      lastIndex = endIndex;
    });
    
    const remaining = text.substring(lastIndex).trim();
    return { complete, remaining };
  }
  
  private flushTranscriptionBuffers() {
    this.flushInputBuffer();
    this.flushOutputBuffer();
  }
  
  private flushInputBuffer() {
    if (this.inputTranscriptionBuffer.length > 0) {
      const text = this.inputTranscriptionBuffer.join('').trim();
      if (text) {
        this.addTranscription('input', text);
      }
      this.inputTranscriptionBuffer = [];
    }
  }
  
  private flushOutputBuffer() {
    if (this.outputTranscriptionBuffer.length > 0) {
      const text = this.outputTranscriptionBuffer.join('').trim();
      if (text) {
        this.addTranscription('output', text);
      }
      this.outputTranscriptionBuffer = [];
    }
  }
  
  private debouncedTranscriptionUpdate() {
    if (this.transcriptionDebounceTimer) {
      clearTimeout(this.transcriptionDebounceTimer);
    }
    this.transcriptionDebounceTimer = window.setTimeout(() => {
      this.requestUpdate();
      this.transcriptionDebounceTimer = null;
    }, 50); // Batch updates every 50ms
  }

  private addTranscription(type: 'input' | 'output', text: string) {
    const MAX_TRANSCRIPTIONS = 20; // Keep only last 20 for display
    const targetArray = type === 'input' ? this.inputTranscriptions : this.outputTranscriptions;
    
    // Use circular buffer pattern - remove oldest if at capacity
    if (targetArray.length >= MAX_TRANSCRIPTIONS) {
      targetArray.shift(); // Remove oldest (O(n) but small array)
    }
    
    targetArray.push({ text, timestamp: Date.now() });
    
    // Debounced update instead of immediate
    this.debouncedTranscriptionUpdate();
    
    // Scroll immediately (simple fix)
    this.scrollToBottom();
  }
  
  private scrollToBottom() {
    // Use requestAnimationFrame to batch DOM operations and avoid forced reflow
    requestAnimationFrame(() => {
      const inputScroll = this.shadowRoot?.querySelector('.input-scroll');
      const outputScroll = this.shadowRoot?.querySelector('.output-scroll');
      
      // Use scrollIntoView on last element instead of scrollHeight (avoids forced reflow)
      if (inputScroll?.lastElementChild) {
        inputScroll.lastElementChild.scrollIntoView({ block: 'end', behavior: 'auto' });
      }
      if (outputScroll?.lastElementChild) {
        outputScroll.lastElementChild.scrollIntoView({ block: 'end', behavior: 'auto' });
      }
    });
  }


  render() {
    const props: TemplateProps = {
      selectedModel: this.selectedModel,
      batteryPercentage: this.batteryPercentage,
      cpuUsage: this.cpuUsage,
      onModelChange: this.onModelChange,
      onReset: this.reset.bind(this),
      onStartRecording: this.startRecording.bind(this),
      onStopRecording: this.stopRecording.bind(this),
      onToggleInputAnalyzer: this.toggleInputAnalyzer,
      onToggleOutputAnalyzer: this.toggleOutputAnalyzer,
      onToggleConsole: this.toggleConsole,
      onClearConsole: this.clearConsole,
      isRecording: this.isRecording,
      showInputAnalyzer: this.showInputAnalyzer,
      showOutputAnalyzer: this.showOutputAnalyzer,
      showConsole: this.showConsole,
      consoleLogs: this.consoleManager.getLogs(),
      error: this.error,
      inputNode: this.audioManager.getInputNode(),
      outputNode: this.audioManager.getOutputNode(),
      inputTranscriptions: this.inputTranscriptions,
      outputTranscriptions: this.outputTranscriptions
    };

    return renderTemplate(props);
  }
}