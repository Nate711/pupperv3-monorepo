/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { Type } from '@google/genai';

// Audio visualizer tool definitions
export const turn_on_audio_visualizers = {
  name: "turn_on_audio_visualizers",
  description: "Turn on the audio visualizers for input and output audio streams",
  parameters: {
    type: Type.OBJECT,
    properties: {
      input: {
        type: Type.BOOLEAN,
        description: "Whether to turn on the input audio visualizer"
      },
      output: {
        type: Type.BOOLEAN,
        description: "Whether to turn on the output audio visualizer"
      }
    }
  }
};

export const turn_off_audio_visualizers = {
  name: "turn_off_audio_visualizers",
  description: "Turn off the audio visualizers for input and output audio streams",
  parameters: {
    type: Type.OBJECT,
    properties: {
      input: {
        type: Type.BOOLEAN,
        description: "Whether to turn off the input audio visualizer"
      },
      output: {
        type: Type.BOOLEAN,
        description: "Whether to turn off the output audio visualizer"
      }
    }
  }
};

export const tools = [{
  functionDeclarations: [
    turn_on_audio_visualizers,
    turn_off_audio_visualizers
  ]
}];

// Tool handler function
export function handleToolCall(
  toolCall: any,
  toggleInputAnalyzer: () => void,
  toggleOutputAnalyzer: () => void,
  showInputAnalyzer: boolean,
  showOutputAnalyzer: boolean
) {
  console.log('🔧 [TOOLS] Received tool call:', toolCall);
  const functionResponses = [];

  for (const fc of toolCall.functionCalls) {
    let response = { result: "ok" };

    if (fc.name === "turn_on_audio_visualizers") {
      const input = fc.args?.input ?? true;
      const output = fc.args?.output ?? true;

      console.log(`🎛️ [TOOLS] Turning on visualizers - Input: ${input}, Output: ${output}`);

      // Call existing toggle methods
      if (input && !showInputAnalyzer) {
        toggleInputAnalyzer();
      }
      if (output && !showOutputAnalyzer) {
        toggleOutputAnalyzer();
      }

      response = {
        result: `Audio visualizers turned on - Input: ${input}, Output: ${output}`
      };
    }
    else if (fc.name === "turn_off_audio_visualizers") {
      const input = fc.args?.input ?? true;
      const output = fc.args?.output ?? true;

      console.log(`🎛️ [TOOLS] Turning off visualizers - Input: ${input}, Output: ${output}`);

      // Call existing toggle methods
      if (input && showInputAnalyzer) {
        toggleInputAnalyzer();
      }
      if (output && showOutputAnalyzer) {
        toggleOutputAnalyzer();
      }

      response = {
        result: `Audio visualizers turned off - Input: ${input}, Output: ${output}`
      };
    }

    functionResponses.push({
      id: fc.id,
      name: fc.name,
      response: response
    });
  }

  return functionResponses;
}