/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export class VisualizerManager {
  private visualizerAnimationId?: number;
  private outputVisualizerAnimationId?: number;

  startVisualizer(
    shadowRoot: ShadowRoot | null,
    inputAnalyser: AnalyserNode | null,
    showInputAnalyzer: boolean,
    isRecording: boolean
  ) {
    const canvas = shadowRoot?.querySelector('.visualizer-canvas') as HTMLCanvasElement;
    if (!canvas || !inputAnalyser || !showInputAnalyzer || !isRecording) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    canvas.width = 184; // 200px - 16px padding
    canvas.height = 64; // 80px - 16px padding

    const bufferLength = inputAnalyser.frequencyBinCount;
    const dataArray = new Uint8Array(bufferLength);

    const animate = () => {
      inputAnalyser.getByteFrequencyData(dataArray);

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
    };

    // Use setInterval at 20Hz (50ms) instead of requestAnimationFrame at 60Hz
    // This reduces CPU usage while maintaining smooth visualization
    this.visualizerAnimationId = window.setInterval(() => {
      if (isRecording) {
        animate();
      }
    }, 50);
  }

  stopVisualizer(shadowRoot: ShadowRoot | null) {
    if (this.visualizerAnimationId) {
      clearInterval(this.visualizerAnimationId);
    }

    // Clear canvas
    const canvas = shadowRoot?.querySelector('.visualizer-canvas') as HTMLCanvasElement;
    if (canvas) {
      const ctx = canvas.getContext('2d');
      if (ctx) {
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
      }
    }
  }

  startOutputVisualizer(
    shadowRoot: ShadowRoot | null,
    outputAnalyser: AnalyserNode | null,
    showOutputAnalyzer: boolean
  ) {
    const canvas = shadowRoot?.querySelector('.output-visualizer .visualizer-canvas') as HTMLCanvasElement;
    if (!canvas || !outputAnalyser || !showOutputAnalyzer) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    canvas.width = 184; // 200px - 16px padding
    canvas.height = 64; // 80px - 16px padding

    const bufferLength = outputAnalyser.frequencyBinCount;
    const dataArray = new Uint8Array(bufferLength);

    const animate = () => {
      outputAnalyser.getByteFrequencyData(dataArray);

      // Clear canvas
      ctx.fillStyle = '#000';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // Draw frequency bars
      const barWidth = canvas.width / bufferLength * 2;
      let x = 0;

      for (let i = 0; i < bufferLength / 2; i++) { // Only show lower frequencies
        const barHeight = (dataArray[i] / 255) * canvas.height;

        // Color based on frequency - orange to red for output
        const hue = 30 - (i / (bufferLength / 2)) * 30; // 30-0 (orange to red)
        ctx.fillStyle = `hsl(${hue}, 80%, 60%)`;

        ctx.fillRect(x, canvas.height - barHeight, barWidth, barHeight);
        x += barWidth + 1;
      }
    };

    // Use setInterval at 20Hz (50ms) instead of requestAnimationFrame at 60Hz
    // This reduces CPU usage while maintaining smooth visualization
    this.outputVisualizerAnimationId = window.setInterval(animate, 50);
  }

  stopOutputVisualizer(shadowRoot: ShadowRoot | null) {
    if (this.outputVisualizerAnimationId) {
      clearInterval(this.outputVisualizerAnimationId);
    }

    // Clear canvas
    const canvas = shadowRoot?.querySelector('.output-visualizer .visualizer-canvas') as HTMLCanvasElement;
    if (canvas) {
      const ctx = canvas.getContext('2d');
      if (ctx) {
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
      }
    }
  }

  cleanup() {
    if (this.visualizerAnimationId) {
      clearInterval(this.visualizerAnimationId);
    }
    if (this.outputVisualizerAnimationId) {
      clearInterval(this.outputVisualizerAnimationId);
    }
  }
}