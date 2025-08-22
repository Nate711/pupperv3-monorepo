/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

class AudioProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.isRecording = false;
    
    this.port.onmessage = (event) => {
      if (event.data.type === 'setRecording') {
        this.isRecording = event.data.recording;
      }
    };
  }

  process(inputs, outputs, parameters) {
    const input = inputs[0];
    
    if (input && input.length > 0 && this.isRecording) {
      const inputChannel = input[0];
      
      if (inputChannel.length > 0) {
        // Send the audio data back to the main thread
        this.port.postMessage({
          type: 'audioData',
          data: inputChannel
        });
      }
    }

    return true;
  }
}

registerProcessor('audio-processor', AudioProcessor);