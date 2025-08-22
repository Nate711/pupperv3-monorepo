/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { Type } from '@google/genai';

// WebSocket connection for robot control
let robotWebSocket: WebSocket | null = null;

function initRobotWebSocket() {
  // Always attempt a fresh connection if not connected or connection failed
  if (!robotWebSocket || robotWebSocket.readyState === WebSocket.CLOSED || robotWebSocket.readyState === WebSocket.CLOSING) {
    try {
      console.log("🤖 [ROBOT] Attempting to connect to robot server...");
      robotWebSocket = new WebSocket("ws://robot-host:8765");
      
      robotWebSocket.addEventListener("open", () => {
        console.log("🤖 [ROBOT] WebSocket connection established");
      });
      
      robotWebSocket.addEventListener("message", (event) => {
        const response = JSON.parse(event.data);
        console.log("🤖 [ROBOT] Response:", response);
      });
      
      robotWebSocket.addEventListener("close", () => {
        console.log("🤖 [ROBOT] WebSocket connection closed");
        robotWebSocket = null;
      });
      
      robotWebSocket.addEventListener("error", (error) => {
        console.error("🤖 [ROBOT] WebSocket connection error - server may not be running:", error);
        robotWebSocket = null;
      });
    } catch (error) {
      console.error("🤖 [ROBOT] Failed to create WebSocket connection - server may not be running:", error);
      robotWebSocket = null;
    }
  }
}

function sendRobotCommand(name: string, args: any = {}) {
  initRobotWebSocket();
  
  if (robotWebSocket && robotWebSocket.readyState === WebSocket.OPEN) {
    const msg = JSON.stringify({ name, args });
    robotWebSocket.send(msg);
    console.log(`🤖 [ROBOT] Sent command: ${name}`, args);
    return true;
  } else {
    console.error(`🤖 [ROBOT] Cannot send command '${name}' - WebSocket not connected. Server may not be running.`);
    return false;
  }
}

// Robot control tool definitions
export const activate_robot = {
  name: "activate_robot",
  description: "Activate the robot to start operations",
  parameters: {
    type: Type.OBJECT,
    properties: {}
  }
};

export const deactivate_robot = {
  name: "deactivate_robot", 
  description: "Deactivate the robot to stop operations",
  parameters: {
    type: Type.OBJECT,
    properties: {}
  }
};

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
    activate_robot,
    deactivate_robot,
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

    if (fc.name === "activate_robot") {
      console.log('🤖 [TOOLS] Activating robot');
      const success = sendRobotCommand("activate");
      response = {
        result: success ? "Robot activated successfully" : "Failed to activate robot - connection error"
      };
    }
    else if (fc.name === "deactivate_robot") {
      console.log('🤖 [TOOLS] Deactivating robot');
      const success = sendRobotCommand("deactivate");
      response = {
        result: success ? "Robot deactivated successfully" : "Failed to deactivate robot - connection error"
      };
    }
    else if (fc.name === "turn_on_audio_visualizers") {
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