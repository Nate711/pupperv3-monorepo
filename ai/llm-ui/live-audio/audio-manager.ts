/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export class AudioManager {
  private inputAudioContext: AudioContext;
  private outputAudioContext: AudioContext;
  private inputNode: GainNode;
  private outputNode: GainNode;
  private nextStartTime = 0;
  private inputAnalyser: AnalyserNode | null = null;
  private outputAnalyser: AnalyserNode | null = null;
  private sources = new Set<AudioBufferSourceNode>();

  constructor() {
    this.inputAudioContext = new (window.AudioContext || (window as any).webkitAudioContext)({ sampleRate: 16000 });
    this.outputAudioContext = new (window.AudioContext || (window as any).webkitAudioContext)({ sampleRate: 24000 });
    this.inputNode = this.inputAudioContext.createGain();
    this.outputNode = this.outputAudioContext.createGain();
  }

  initAudio(showInputAnalyzer: boolean, showOutputAnalyzer: boolean) {
    this.nextStartTime = this.outputAudioContext.currentTime;

    // Set up input analyzer for visualizer if enabled
    if (showInputAnalyzer) {
      this.inputAnalyser = this.inputAudioContext.createAnalyser();
      this.inputAnalyser.fftSize = 256;
      this.inputAnalyser.smoothingTimeConstant = 0.8;
      this.inputNode.connect(this.inputAnalyser);
    }

    // Set up output analyzer for visualizer if enabled
    if (showOutputAnalyzer) {
      this.outputAnalyser = this.outputAudioContext.createAnalyser();
      this.outputAnalyser.fftSize = 256;
      this.outputAnalyser.smoothingTimeConstant = 0.8;
      this.outputNode.connect(this.outputAnalyser);
    }

    // Connect output to destination
    this.outputNode.connect(this.outputAudioContext.destination);
  }

  toggleInputAnalyzer(showInputAnalyzer: boolean): boolean {
    const newState = !showInputAnalyzer;
    console.log(`🎛️ [ANALYZER] Input analyzer toggled: ${newState ? 'ON' : 'OFF'}`);

    if (!newState) {
      // Stop and disconnect analyzer
      if (this.inputAnalyser) {
        try {
          this.inputNode.disconnect(this.inputAnalyser);
        } catch (e) {
          // Ignore disconnect errors
        }
        this.inputAnalyser = null;
      }
    } else {
      // Create new analyzer
      this.inputAnalyser = this.inputAudioContext.createAnalyser();
      this.inputAnalyser.fftSize = 256;
      this.inputAnalyser.smoothingTimeConstant = 0.8;
      this.inputNode.connect(this.inputAnalyser);
    }

    return newState;
  }

  toggleOutputAnalyzer(showOutputAnalyzer: boolean): boolean {
    const newState = !showOutputAnalyzer;
    console.log(`🎛️ [ANALYZER] Output analyzer toggled: ${newState ? 'ON' : 'OFF'}`);

    if (!newState) {
      // Stop and disconnect analyzer
      if (this.outputAnalyser) {
        try {
          this.outputNode.disconnect(this.outputAnalyser);
        } catch (e) {
          // Ignore disconnect errors
        }
        this.outputAnalyser = null;
      }
    } else {
      // Create new analyzer
      this.outputAnalyser = this.outputAudioContext.createAnalyser();
      this.outputAnalyser.fftSize = 256;
      this.outputAnalyser.smoothingTimeConstant = 0.8;
      this.outputNode.connect(this.outputAnalyser);
    }

    return newState;
  }

  getInputAnalyser() {
    return this.inputAnalyser;
  }

  getOutputAnalyser() {
    return this.outputAnalyser;
  }

  getInputNode() {
    return this.inputNode;
  }

  getOutputNode() {
    return this.outputNode;
  }

  getInputAudioContext() {
    return this.inputAudioContext;
  }

  getOutputAudioContext() {
    return this.outputAudioContext;
  }

  getNextStartTime() {
    return this.nextStartTime;
  }

  setNextStartTime(time: number) {
    this.nextStartTime = time;
  }

  addSource(source: AudioBufferSourceNode) {
    this.sources.add(source);
  }

  removeSource(source: AudioBufferSourceNode) {
    this.sources.delete(source);
  }

  stopAllSources() {
    for (const source of this.sources.values()) {
      source.stop();
      this.sources.delete(source);
    }
    this.nextStartTime = 0;
  }
}