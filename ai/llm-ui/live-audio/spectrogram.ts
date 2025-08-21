/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import {LitElement, css, html} from 'lit';
import {customElement, property, state} from 'lit/decorators.js';

@customElement('gdm-spectrogram')
export class GdmSpectrogram extends LitElement {
  private canvas!: HTMLCanvasElement;
  private canvasContext!: CanvasRenderingContext2D;
  private analyser!: AnalyserNode;
  private frequencyData!: Uint8Array;
  private spectrogramData: Uint8Array[] = [];
  private animationFrameId!: number;
  
  @property() width = 256;
  @property() height = 128;
  @property() maxHistory = 256; // Number of time slices to keep

  private _audioNode!: AudioNode;

  @property()
  set audioNode(node: AudioNode) {
    this._audioNode = node;
    this.setupAnalyser();
  }

  get audioNode() {
    return this._audioNode;
  }

  static styles = css`
    :host {
      display: block;
      position: absolute;
      top: 20px;
      left: 20px;
      z-index: 100;
      border-radius: 8px;
      overflow: hidden;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.5);
      border: 2px solid rgba(25, 162, 230, 0.3);
      background: rgba(0, 0, 0, 0.7);
    }

    canvas {
      display: block;
      width: 100%;
      height: 100%;
      image-rendering: pixelated;
    }
  `;

  private setupAnalyser() {
    if (!this._audioNode) return;

    this.analyser = this._audioNode.context.createAnalyser();
    this.analyser.fftSize = 512; // Higher resolution for spectrogram
    this.analyser.smoothingTimeConstant = 0.1;
    this.frequencyData = new Uint8Array(this.analyser.frequencyBinCount);
    this._audioNode.connect(this.analyser);
  }

  protected firstUpdated() {
    this.canvas = this.shadowRoot!.querySelector('canvas') as HTMLCanvasElement;
    this.canvasContext = this.canvas.getContext('2d')!;
    
    // Set canvas resolution
    this.canvas.width = this.width;
    this.canvas.height = this.height;
    
    this.startAnimation();
  }

  connectedCallback() {
    super.connectedCallback();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.stopAnimation();
  }

  private startAnimation() {
    const animate = () => {
      this.updateSpectrogram();
      this.drawSpectrogram();
      this.animationFrameId = requestAnimationFrame(animate);
    };
    this.animationFrameId = requestAnimationFrame(animate);
  }

  private stopAnimation() {
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
    }
  }

  private updateSpectrogram() {
    if (!this.analyser || !this.frequencyData) return;

    // Get current frequency data
    this.analyser.getByteFrequencyData(this.frequencyData);
    
    // Add current frame to history
    this.spectrogramData.push(new Uint8Array(this.frequencyData));
    
    // Keep only the most recent frames
    if (this.spectrogramData.length > this.maxHistory) {
      this.spectrogramData.shift();
    }
  }

  private drawSpectrogram() {
    if (!this.canvasContext || !this.spectrogramData.length) return;

    const ctx = this.canvasContext;
    const width = this.canvas.width;
    const height = this.canvas.height;

    // Clear canvas
    ctx.fillStyle = '#000000';
    ctx.fillRect(0, 0, width, height);

    const timeSlices = this.spectrogramData.length;
    const freqBins = this.spectrogramData[0].length;
    const timeStep = width / Math.max(timeSlices, 1);
    const freqStep = height / freqBins;

    // Draw spectrogram
    for (let t = 0; t < timeSlices; t++) {
      const x = (t / Math.max(timeSlices - 1, 1)) * width;
      const frequencySlice = this.spectrogramData[t];

      for (let f = 0; f < freqBins; f++) {
        const y = height - (f / Math.max(freqBins - 1, 1)) * height;
        const intensity = frequencySlice[f] / 255;
        
        // Create color gradient from blue to white based on intensity
        const hue = 200; // Blue hue
        const saturation = 100 - (intensity * 30); // Less saturated for higher intensities
        const lightness = intensity * 70; // Brightness based on intensity
        
        ctx.fillStyle = `hsl(${hue}, ${saturation}%, ${lightness}%)`;
        ctx.fillRect(Math.floor(x), Math.floor(y), Math.max(1, Math.ceil(timeStep)), Math.max(1, Math.ceil(freqStep)));
      }
    }

    // Add frequency scale labels
    ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
    ctx.font = '10px monospace';
    ctx.textAlign = 'left';
    
    // Label some key frequencies
    const sampleRate = this.analyser.context.sampleRate;
    const maxFreq = sampleRate / 2;
    const freqLabels = [
      { freq: 100, label: '100Hz' },
      { freq: 1000, label: '1kHz' },
      { freq: 4000, label: '4kHz' },
    ];

    freqLabels.forEach(({freq, label}) => {
      const binIndex = Math.floor((freq / maxFreq) * freqBins);
      const y = height - (binIndex / freqBins) * height;
      if (y > 12 && y < height - 2) {
        ctx.fillText(label, 2, y - 2);
      }
    });
  }

  protected render() {
    return html`<canvas width="${this.width}" height="${this.height}"></canvas>`;
  }
}

declare global {
  interface HTMLElementTagNameMap {
    'gdm-spectrogram': GdmSpectrogram;
  }
}